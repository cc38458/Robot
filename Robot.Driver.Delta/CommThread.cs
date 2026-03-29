using Robot.Core.Enums;
using Robot.Core.Logging;
using Robot.Core.Models;
using System.Text.Json;

namespace Robot.Driver.Delta
{
    /// <summary>
    /// 通訊線程 — 獨佔所有 EtherCAT DLL 呼叫
    /// 職責：連線/初始化、狀態輪詢、指令隊列消化、安全優先處理、看門狗
    /// </summary>
    internal class CommThread : IDisposable
    {
        // ── 常數 ──
        private const int AXIS_COUNT = 6;
        private const int POLL_INTERVAL_MS = 10;
        private const int HW_BUFFER_SAFE_LIMIT = 15;  // 硬體 buffer 20 筆，保留 5 筆安全餘量
        private const int CONN_TIMEOUT_MS = 15000;
        private const int CONN_POLL_MS = 100;
        private const double WATCHDOG_TIMEOUT_SEC = 2.0;
        private const double DEFAULT_SDSTOP_TDEC = 0.5;

        // ── 齒輪比 ──
        private static readonly int[] DEFAULT_PULSE2ANG = { 29049, 35959, 29479, 29479, 28889, 18194 };

        // ── 共享狀態（通訊線程寫入，主線程讀取） ──
        private readonly int[] _pos = new int[AXIS_COUNT];
        private readonly int[] _speed = new int[AXIS_COUNT];
        private readonly MotorState[] _state = new MotorState[AXIS_COUNT];
        private readonly int[] _queueLength = new int[AXIS_COUNT];
        private readonly object _stateLock = new();

        // ── 軸卡狀態（主線程持有，但通訊線程可建議變更） ──
        private volatile CardState _cardState = CardState.NULL;
        private volatile bool _cardStateChangeRequest = false;
        private volatile CardState _requestedCardState;

        // ── 安全旗標（主線程設定，通訊線程優先處理） ──
        private volatile bool _estopRequested = false;
        private volatile bool _ralmRequested = false;
        private volatile bool _sdStopAllRequested = false; // 看門狗觸發用

        // ── 立即停止請求（每軸獨立，無視隊列） ──
        private readonly bool[] _immediateStopRequested = new bool[AXIS_COUNT];
        private readonly double[] _immediateStopTDec = new double[AXIS_COUNT];
        private readonly object _immediateStopLock = new();

        // ── 非阻塞式立即停止狀態機 ──
        private enum StopPhase { None, Decelerating }
        private readonly StopPhase[] _stopPhase = new StopPhase[AXIS_COUNT];
        private readonly long[] _stopDeadlineTicks = new long[AXIS_COUNT];

        // ── 輪詢錯誤計數（連續失敗觸發安全停止） ──
        private const int POLL_ERROR_THRESHOLD = 5;
        private readonly int[] _pollErrorCount = new int[AXIS_COUNT];

        // ── 狀態變更鎖（保護 _cardStateChangeRequest/_requestedCardState 的原子性） ──
        private readonly object _stateChangeLock = new();

        // ── 指令隊列（每軸獨立） ──
        private readonly Queue<MotionCommand>[] _queues = new Queue<MotionCommand>[AXIS_COUNT];
        private readonly object _queueLock = new();

        // ── 連線資訊 ──
        private ushort _cardNo;
        private ushort _axisNum;
        private readonly int[] _pulse2Ang = DEFAULT_PULSE2ANG;
        private int[] _zeroPulse = new int[AXIS_COUNT];

        // ── 線程控制 ──
        private Thread? _thread;
        private volatile bool _running;
        private readonly ManualResetEventSlim _startSignal = new(false);
        private readonly ManualResetEventSlim _initSignal = new(false);
        private volatile bool _initRequested = false;
        private volatile bool _endRequested = false;
        private volatile bool _connectResult;
        private volatile bool _initResult;

        // ── 看門狗 ──
        private long _mainThreadHeartbeat;
        private long _commThreadHeartbeat;

        // ── 原點標定 ──
        private volatile bool _calibrateRequested = false;
        private volatile bool _calibrateResult;
        private readonly ManualResetEventSlim _calibrateSignal = new(false);

        // ── AbortAndChangePosition 請求 ──
        private enum AbortChangePosPhase { None, Decelerating, FinalSdStopPending }
        private volatile bool _abortChangePosRequested = false;
        private readonly int[] _abortChangePosTargetMdeg = new int[AXIS_COUNT];
        private double _abortChangePosTDec;
        private readonly object _abortChangePosLock = new();
        private AbortChangePosPhase _abortChangePosPhase = AbortChangePosPhase.None;
        private long _abortChangePosDeadlineTicks;

        // ── 日誌 ──
        private readonly RobotLogger _log;
        private readonly IEtherCatApi _ecat;
        private readonly bool _isMockBackend;

        // ── 零點設定檔路徑 ──
        private readonly string _zeroConfigPath;

        /// <summary>
        /// 建立通訊線程實例。
        /// </summary>
        /// <param name="logger">日誌記錄器。</param>
        /// <param name="zeroConfigPath">零點設定檔路徑。</param>
        /// <param name="useMockBackend">是否使用 Mock 後端。</param>
        public CommThread(RobotLogger logger, string zeroConfigPath = "axis_zero_config.json", bool useMockBackend = false)
        {
            _log = logger;
            _ecat = useMockBackend ? new MockEtherCatApi() : new RealEtherCatApi();
            _isMockBackend = useMockBackend;
            _zeroConfigPath = zeroConfigPath;
            for (int i = 0; i < AXIS_COUNT; i++)
                _queues[i] = new Queue<MotionCommand>();
            _mainThreadHeartbeat = DateTime.UtcNow.Ticks;
            _commThreadHeartbeat = DateTime.UtcNow.Ticks;
            _log.Info($"EtherCAT 後端：{(useMockBackend ? "Mock" : "Real")}");
        }

        // ════════════════════════════════════════
        // 公開屬性（主線程讀取）
        // ════════════════════════════════════════

        /// <summary>
        /// 將通訊線程內部的軸狀態快照複製至呼叫端提供的陣列。
        /// </summary>
        public void GetState(int[] pos, int[] speed, MotorState[] state, int[] queueLen)
        {
            lock (_stateLock)
            {
                Array.Copy(_pos, pos, AXIS_COUNT);
                Array.Copy(_speed, speed, AXIS_COUNT);
                Array.Copy(_state, state, AXIS_COUNT);
                Array.Copy(_queueLength, queueLen, AXIS_COUNT);
            }
        }

        /// <summary>目前軸卡狀態（由通訊線程維護）。</summary>
        public CardState CardState => _cardState;

        /// <summary>主線程呼叫：更新心跳</summary>
        public void MainHeartbeat() => Interlocked.Exchange(ref _mainThreadHeartbeat, DateTime.UtcNow.Ticks);

        /// <summary>通訊線程是否活著</summary>
        public bool IsCommAlive
        {
            get
            {
                var elapsed = DateTime.UtcNow.Ticks - Interlocked.Read(ref _commThreadHeartbeat);
                return elapsed < TimeSpan.FromSeconds(WATCHDOG_TIMEOUT_SEC).Ticks;
            }
        }

        // ════════════════════════════════════════
        // 連線/初始化/結束（由主線程呼叫，通訊線程執行）
        // ════════════════════════════════════════

        /// <summary>
        /// 啟動通訊線程並開始 EtherCAT 連線流程，不等待實際連線完成。
        /// </summary>
        /// <returns>通訊線程成功啟動並已送出連線流程回傳 true，否則 false。</returns>
        public bool StartConnection()
        {
            if (_running) return false;
            _running = true;
            _cardState = CardState.CONNING;
            _startSignal.Reset();

            _thread = new Thread(CommLoop)
            {
                Name = "EtherCAT-CommThread",
                IsBackground = false, // 非背景，避免主線程結束時被強殺
                Priority = ThreadPriority.AboveNormal,
            };
            try
            {
                _thread.Start();
                return true;
            }
            catch (Exception ex)
            {
                _running = false;
                _cardState = CardState.NULL;
                _log.Fatal("通訊線程啟動失敗", ex);
                return false;
            }
        }

        /// <summary>
        /// 請求初始化所有軸（零點、Servo ON），阻塞至完成。
        /// </summary>
        /// <returns>初始化成功回傳 true，否則 false。</returns>
        public bool RequestInitial()
        {
            if (_cardState != CardState.CONNCET) return false;
            _initRequested = true;
            _initSignal.Reset();
            _initSignal.Wait(TimeSpan.FromSeconds(10));
            return _initResult;
        }

        /// <summary>請求結束通訊線程（Servo OFF 並關閉主站）。</summary>
        public void RequestEnd() => _endRequested = true;

        /// <summary>
        /// 主線程呼叫：執行原點標定（僅 Real 模式）。
        /// 讀取目前各軸實際編碼器位置並儲存為新的 ZeroPulse。
        /// Mock 模式不允許，回傳 false。
        /// </summary>
        public bool RequestCalibrateZero()
        {
            if (_isMockBackend)
            {
                _log.Info("Mock 模式：略過原點標定");
                return true;
            }
            if (_cardState < CardState.CONNCET)
            {
                _log.Warn($"原點標定拒絕：目前狀態 {_cardState}，需至少 CONNCET");
                return false;
            }
            _calibrateSignal.Reset();
            _calibrateRequested = true;
            _calibrateSignal.Wait(TimeSpan.FromSeconds(5));
            return _calibrateResult;
        }

        // ════════════════════════════════════════
        // 安全指令（主線程呼叫，立即設旗標）
        // ════════════════════════════════════════

        /// <summary>請求全軸緊急停止並清除所有隊列。</summary>
        public void RequestEstop()
        {
            _estopRequested = true;
            ClearAllQueues();
            _log.Warn("收到緊急停止請求");
        }

        /// <summary>請求全軸警報復歸（Ralm + 重新 Servo ON）。</summary>
        public void RequestRalm()
        {
            _ralmRequested = true;
            _log.Info("收到警報復歸請求");
        }

        /// <summary>
        /// 主線程呼叫：立即停止指定軸（無視該軸隊列）
        /// 流程：清除該軸隊列 → VelocityChange(0, tDec) → 等待 tDec → Sd_Stop
        /// </summary>
        public void RequestImmediateStop(ushort axis, double tDec)
        {
            if (axis >= AXIS_COUNT) return;

            // 先清除該軸隊列
            lock (_queueLock)
            {
                _queues[axis].Clear();
                UpdateQueueLengths();
            }

            lock (_immediateStopLock)
            {
                _immediateStopRequested[axis] = true;
                _immediateStopTDec[axis] = tDec;
            }
            _log.Info($"收到軸 {axis} 立即停止請求 (tDec={tDec}s)");
        }

        /// <summary>
        /// 主線程呼叫：清除所有隊列並發出多軸同步終止+切換至指定位置的請求。
        /// </summary>
        public void RequestAbortAndChangePosition(int[] targetMdeg, double tDec)
        {
            if (targetMdeg == null || targetMdeg.Length < AXIS_COUNT) return;

            lock (_queueLock)
            {
                for (int i = 0; i < AXIS_COUNT; i++)
                    _queues[i].Clear();
                UpdateQueueLengths();
            }

            lock (_abortChangePosLock)
            {
                Array.Copy(targetMdeg, _abortChangePosTargetMdeg, AXIS_COUNT);
                _abortChangePosTDec = tDec;
                _abortChangePosRequested = true;
                _abortChangePosPhase = AbortChangePosPhase.None;
            }
            _log.Info($"收到 AbortAndChangePosition 請求 (tDec={tDec}s)");
        }

        // ════════════════════════════════════════
        // 指令入隊（主線程呼叫）
        // ════════════════════════════════════════

        /// <summary>
        /// 將運動指令加入對應軸的隊列，由通訊線程依序消化。
        /// </summary>
        /// <param name="cmd">運動指令。</param>
        /// <returns>入隊成功回傳 true，軸號無效回傳 false。</returns>
        public bool EnqueueCommand(MotionCommand cmd)
        {
            if (cmd.Axis >= AXIS_COUNT && cmd.Type != CommandType.MultiAxisPVT)
                return false;

            lock (_queueLock)
            {
                if (cmd.Type == CommandType.MultiAxisPVT)
                {
                    // 多軸指令：放入軸 0 的隊列（特殊標記）
                    _queues[0].Enqueue(cmd);
                    _log.Info($"指令入隊：{cmd}");
                }
                else
                {
                    _queues[cmd.Axis].Enqueue(cmd);
                    _log.Info($"指令入隊：{cmd}");
                }
                UpdateQueueLengths();
            }
            return true;
        }

        /// <summary>清除所有軸的指令隊列。</summary>
        public void ClearAllQueues()
        {
            lock (_queueLock)
            {
                for (int i = 0; i < AXIS_COUNT; i++)
                    _queues[i].Clear();
                UpdateQueueLengths();
            }
            _log.Info("所有指令隊列已清除");
        }

        /// <summary>
        /// 更新各軸隊列長度快照（需在 _queueLock 內呼叫）。
        /// </summary>
        private void UpdateQueueLengths()
        {
            // 在 _stateLock 下更新，確保 GetState() 讀取時一致
            // 呼叫端已持有 _queueLock，鎖順序固定為 _queueLock → _stateLock
            lock (_stateLock)
            {
                for (int i = 0; i < AXIS_COUNT; i++)
                    _queueLength[i] = _queues[i].Count;
            }
        }

        // ════════════════════════════════════════
        // 通訊線程主迴圈
        // ════════════════════════════════════════

        /// <summary>
        /// 通訊線程主迴圈：連線 → 輪詢/安全處理/隊列消化，直到收到結束請求。
        /// </summary>
        private void CommLoop()
        {
            _log.Info("通訊線程啟動");

            try
            {
                // Phase 1: 連線
                _connectResult = DoConnect();
                _cardState = _connectResult ? CardState.CONNCET : CardState.CONNERR;
                _startSignal.Set();

                if (!_connectResult)
                {
                    _log.Error("EtherCAT 連線失敗");
                    _running = false;
                    return;
                }
                _log.Info("EtherCAT 連線成功");

                // Phase 2: 主迴圈
                while (_running)
                {
                    // 更新通訊線程心跳
                    Interlocked.Exchange(ref _commThreadHeartbeat, DateTime.UtcNow.Ticks);

                    // ── 最高優先：安全指令 ──
                    if (_estopRequested)
                    {
                        DoEstop();
                        _estopRequested = false;
                    }

                    if (_ralmRequested)
                    {
                        DoRalm();
                        _ralmRequested = false;
                    }

                    if (_sdStopAllRequested)
                    {
                        DoSdStopAll();
                        _sdStopAllRequested = false;
                    }

                    // ── 立即停止請求（每軸獨立，非阻塞狀態機） ──
                    ProcessImmediateStops();

                    // ── AbortAndChangePosition 請求 ──
                    ProcessAbortAndChangePosition();

                    // ── 初始化請求 ──
                    if (_initRequested)
                    {
                        _initResult = DoInitialize();
                        _cardState = _initResult ? CardState.READY : CardState.CONNERR;
                        _initRequested = false;
                        _initSignal.Set();
                    }

                    // ── 原點標定請求 ──
                    if (_calibrateRequested)
                    {
                        _calibrateResult = DoCalibrate();
                        _calibrateRequested = false;
                        _calibrateSignal.Set();
                    }

                    // ── 結束請求 ──
                    if (_endRequested)
                    {
                        DoEnd();
                        _endRequested = false;
                        _running = false;
                        break;
                    }

                    // ── 狀態輪詢（CONNCET 以上才輪詢） ──
                    if (_cardState >= CardState.CONNCET)
                    {
                        DoPollStatus();
                    }

                    // ── 看門狗檢查 ──
                    if (_cardState == CardState.READY)
                    {
                        CheckMainThreadWatchdog();
                    }

                    // ── 指令隊列消化（僅 READY 狀態） ──
                    if (_cardState == CardState.READY)
                    {
                        ProcessQueues();
                    }

                    Thread.Sleep(POLL_INTERVAL_MS);
                }
            }
            catch (Exception ex)
            {
                _log.Fatal("通訊線程異常", ex);
                // 確保 _startSignal 被釋放（避免主線程卡住）
                _connectResult = false;
                _cardState = CardState.CONNERR;
                try { _startSignal.Set(); } catch { }
                // 嘗試急停
                try { DoEstop(); } catch { }
            }
            finally
            {
                // 確保所有等待信號都被釋放
                try { _startSignal.Set(); } catch { }
                try { _initSignal.Set(); } catch { }
                _log.Info("通訊線程結束");
            }
        }

        // ════════════════════════════════════════
        // 連線流程
        // ════════════════════════════════════════

        /// <summary>
        /// 執行 EtherCAT 主站連線流程：Open → GetCardSeq → Initial → 等待初始化完成。
        /// </summary>
        private bool DoConnect()
        {
            ushort cardsNum = 0;
            var ret = _ecat.CS_ECAT_Master_Open(ref cardsNum);
            _log.DllReturn("Master_Open", ret, $"軸卡數量={cardsNum}");
            if (ret != 0 || cardsNum != 1) return false;

            ret = _ecat.CS_ECAT_Master_Get_CardSeq(0, ref _cardNo);
            _log.DllReturn("Master_Get_CardSeq", ret, $"CardNo={_cardNo}");
            if (ret != 0) return false;

            ret = _ecat.CS_ECAT_Master_Initial(_cardNo);
            _log.DllReturn("Master_Initial", ret);
            if (ret != 0) return false;

            // 輪詢初始化完成
            ushort initDone = 1;
            int timer = 0;
            while (initDone == 1 && timer < CONN_TIMEOUT_MS / CONN_POLL_MS)
            {
                Thread.Sleep(CONN_POLL_MS);
                ret = _ecat.CS_ECAT_Master_Check_Initial_Done(_cardNo, ref initDone);
                if (initDone == 99)
                {
                    _log.Error("軸卡初始化失敗 (InitDone=99)");
                    return false;
                }
                timer++;
            }

            if (initDone != 0)
            {
                _log.Error($"軸卡初始化逾時 ({timer * CONN_POLL_MS}ms)");
                return false;
            }

            return true;
        }

        // ════════════════════════════════════════
        // 初始化流程
        // ════════════════════════════════════════

        /// <summary>
        /// 初始化各軸：載入零點設定、Servo ON、CSP 模式。
        /// </summary>
        private bool DoInitialize()
        {
            _log.Info("開始初始化軸...");

            // 1. 確認軸數
            ushort slaveNum = 0;
            var ret = _ecat.CS_ECAT_Master_Get_SlaveNum(_cardNo, ref slaveNum);
            _log.DllReturn("Get_SlaveNum", ret, $"軸數={slaveNum}");
            if (slaveNum < AXIS_COUNT)
            {
                _log.Error($"軸數不足：期望 {AXIS_COUNT}，實際 {slaveNum}");
                return false;
            }
            _axisNum = slaveNum;

            // 2. 讀取零點設定（Mock 模式不讀寫設定檔）
            if (!_isMockBackend)
                LoadZeroConfig();

            // 3. 對每軸設定
            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                // Servo ON
                ret = _ecat.CS_ECAT_Slave_Motion_Set_Svon(_cardNo, i, 0, 1);
                _log.DllReturn("Set_Svon", ret, $"軸{i} ON");
                if (ret != 0) return false;

                // 設定 CSP 模式
                ret = _ecat.CS_ECAT_Slave_Motion_Set_MoveMode(_cardNo, i, 0, 8);
                _log.DllReturn("Set_MoveMode", ret, $"軸{i} → CSP");
                if (ret != 0) return false;

                int posPulse = 0;
                ret = _ecat.CS_ECAT_Slave_Motion_Get_Actual_Position(_cardNo, i, 0, ref posPulse);
                _log.DllReturn("Get_Actual_Position", ret, $"軸{i}: 真實位置={posPulse} pulse");
                if (ret != 0) return false;

                lock (_stateLock)
                {
                    _pos[i] = PulseToMdeg(i, posPulse);
                    _speed[i] = 0;
                    _state[i] = MotorState.STOP;
                }
            }

            _log.Info("所有軸初始化完成，Servo ON");
            return true;
        }

        /// <summary>
        /// 從設定檔載入零點脈波數，檔案不存在或失敗時使用預設值。
        /// </summary>
        private void LoadZeroConfig()
        {
            try
            {
                if (File.Exists(_zeroConfigPath))
                {
                    var json = File.ReadAllText(_zeroConfigPath);
                    var config = JsonSerializer.Deserialize<AxisZeroConfig>(json);
                    if (config != null)
                    {
                        _zeroPulse = config.ZeroPulse;
                        _log.Info($"已載入零點設定檔：{_zeroConfigPath}");
                        return;
                    }
                }
            }
            catch (Exception ex)
            {
                _log.Warn($"讀取零點設定檔失敗：{ex.Message}，使用預設值");
            }

            // 建立預設設定檔
            _zeroPulse = new int[AXIS_COUNT];
            SaveZeroConfig([0,0,0,0,0,0]);
        }

        /// <summary>
        /// 將零點脈波數序列化並寫入設定檔。
        /// </summary>
        private void SaveZeroConfig(int[] newPos)
        {
            try
            {
                var config = new AxisZeroConfig
                {
                    ZeroPulse = newPos,
                    LastModified = DateTime.Now,
                    Note = "各軸零點絕對脈波數設定",
                };
                var json = JsonSerializer.Serialize(config, new JsonSerializerOptions { WriteIndented = true });
                File.WriteAllText(_zeroConfigPath, json);
                _log.Info($"零點設定檔已儲存：{_zeroConfigPath}");
            }
            catch (Exception ex)
            {
                _log.Error($"儲存零點設定檔失敗", ex);
            }
        }

        /// <summary>
        /// 在通訊線程中執行原點標定。
        /// 直接從硬體讀取各軸當前實際位置，存為新的 ZeroPulse。
        /// </summary>
        private bool DoCalibrate()
        {
            if (_isMockBackend)
            {
                _log.Info("Mock 模式：略過原點標定");
                return true;
            }

            _log.Info("開始原點標定，讀取目前編碼器位置...");
            try
            {
                var newZeroPulse = new int[AXIS_COUNT];
                for (ushort i = 0; i < AXIS_COUNT; i++)
                {
                    int pos = 0;
                    var ret = _ecat.CS_ECAT_Slave_Motion_Get_Actual_Position(_cardNo, i, 0, ref pos);
                    _log.DllReturn("Get_Actual_Position", ret, $"軸{i}: 校正位置={pos} pulse");
                    if (ret != 0) return false;
                    newZeroPulse[i] = pos;
                }
                _zeroPulse = newZeroPulse;
                SaveZeroConfig(newZeroPulse);
                _log.Info("原點標定完成，已儲存零點設定檔");
                lock (_stateLock)
                {
                    Array.Clear(_pos, 0, AXIS_COUNT);
                    Array.Clear(_speed, 0, AXIS_COUNT);
                }
                return true;
            }
            catch (Exception ex)
            {
                _log.Error("原點標定失敗", ex);
                return false;
            }
        }

        // ════════════════════════════════════════
        // 狀態輪詢
        // ════════════════════════════════════════

        /// <summary>
        /// 輪詢各軸狀態（位置、速度、Mdone、StatusWord），逐項更新成功讀取的資料。
        /// 連續失敗達閾值時觸發全軸安全停止。
        /// </summary>
        private void DoPollStatus()
        {
            bool anyAlarm = false;
            bool[] needAutoSdStop = new bool[AXIS_COUNT];

            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                int pos = 0, speed = 0;
                ushort mdone = 0, statusWord = 0;

                var r1 = _ecat.CS_ECAT_Slave_Motion_Get_Actual_Position(_cardNo, i, 0, ref pos);
                var r2 = _ecat.CS_ECAT_Slave_Motion_Get_Current_Speed(_cardNo, i, 0, ref speed);
                var r3 = _ecat.CS_ECAT_Slave_Motion_Get_Mdone(_cardNo, i, 0, ref mdone);
                var r4 = _ecat.CS_ECAT_Slave_Motion_Get_StatusWord(_cardNo, i, 0, ref statusWord);

                // 逐項檢查：只有成功讀取的項目才更新，失敗項保留上次值
                int failCount = (r1 != 0 ? 1 : 0) + (r2 != 0 ? 1 : 0)
                              + (r3 != 0 ? 1 : 0) + (r4 != 0 ? 1 : 0);

                if (failCount > 0)
                {
                    _pollErrorCount[i]++;
                    if (_pollErrorCount[i] == 1)
                        _log.Warn($"軸 {i} 輪詢部分失敗 (ret={r1},{r2},{r3},{r4})");
                    if (_pollErrorCount[i] >= POLL_ERROR_THRESHOLD)
                    {
                        _log.Error($"軸 {i} 連續 {_pollErrorCount[i]} 次輪詢含失敗，觸發全軸安全停止");
                        _sdStopAllRequested = true;
                        _pollErrorCount[i] = 0;
                    }
                }
                else
                {
                    _pollErrorCount[i] = 0;
                }

                lock (_stateLock)
                {
                    // 位置：僅在讀取成功時更新
                    if (r1 == 0)
                        _pos[i] = PulseToMdeg(i, pos);

                    // 速度：由原始 pulse/s 轉為 mdeg/s
                    if (r2 == 0)
                        _speed[i] = PulseSpeedToMdegPerSec(i, speed);

                    // 狀態字：僅在讀取成功時檢查警報
                    if (r4 == 0 && (statusWord & 0x0008) != 0)
                    {
                        if (_state[i] != MotorState.ALARM)
                            _log.Warn($"軸 {i} 觸發警報 (StatusWord=0x{statusWord:X4})");
                        _state[i] = MotorState.ALARM;
                        anyAlarm = true;
                    }
                    else if (r4 == 0 && _state[i] == MotorState.ALARM)
                    {
                        _state[i] = MotorState.STOP;
                    }
                    // Mdone：僅在讀取成功時判斷運動完成/開始
                    else if (r3 == 0 && _state[i] == MotorState.MOVING && mdone == 0)
                    {
                        _state[i] = MotorState.STOP;
                        _log.Debug($"軸 {i} 運動完成,當前位置: {_pos[i]}");
                        needAutoSdStop[i] = true; // 延遲到 _stateLock 外處理
                    }
                    else if (r3 == 0 && _state[i] == MotorState.STOP && mdone != 0)
                    {
                        _state[i] = MotorState.MOVING;
                    }
                }
            }

            // CheckAutoSdStop 在 _stateLock 外呼叫，避免 _stateLock → _queueLock 的鎖順序問題
            for (int i = 0; i < AXIS_COUNT; i++)
            {
                if (needAutoSdStop[i])
                    CheckAutoSdStop(i);
            }

            if (anyAlarm && _cardState == CardState.READY)
            {
                _log.Error("偵測到軸警報，執行全軸緊急停止");
                DoEstop();
                lock (_stateChangeLock)
                {
                    _requestedCardState = CardState.ALARM;
                    _cardStateChangeRequest = true;
                }
            }
        }

        /// <summary>
        /// 檢查軸是否為最後執行的指令且需要自動減速停止
        /// </summary>
        private void CheckAutoSdStop(int axis)
        {
            lock (_queueLock)
            {
                // 如果隊列為空，且硬體 buffer 也為空 → 檢查是否需要補 Sd_Stop
                if (_queues[axis].Count == 0)
                {
                    ushort bufLen = 0;
                    _ecat.CS_ECAT_Slave_Motion_Get_Buffer_Length(_cardNo, (ushort)axis, 0, ref bufLen);
                    if (bufLen == 0)
                    {
                        // 軸已完成所有指令，如果速度不為零，補一個減速停止
                        int spd = 0;
                        _ecat.CS_ECAT_Slave_Motion_Get_Current_Speed(_cardNo, (ushort)axis, 0, ref spd);
                        if (Math.Abs(spd) > 0)
                        {
                            _log.Warn($"軸 {axis} 最後指令 endVel≠0 且無後續指令，自動 Sd_Stop");
                            _ecat.CS_ECAT_Slave_Motion_Sd_Stop(_cardNo, (ushort)axis, 0, DEFAULT_SDSTOP_TDEC);
                        }
                    }
                }
            }
        }

        // ════════════════════════════════════════
        // 指令隊列處理
        // ════════════════════════════════════════

        /// <summary>
        /// 消化各軸指令隊列：檢查硬體 buffer 餘量後依序執行隊首指令。
        /// </summary>
        private void ProcessQueues()
        {
            lock (_queueLock)
            {
                for (int i = 0; i < AXIS_COUNT; i++)
                {
                    if (_queues[i].Count == 0) continue;

                    // 檢查軸狀態
                    MotorState axisState;
                    lock (_stateLock) { axisState = _state[i]; }

                    // 檢查硬體 buffer 餘量
                    ushort bufLen = 0;
                    _ecat.CS_ECAT_Slave_Motion_Get_Buffer_Length(_cardNo, (ushort)i, 0, ref bufLen);
                    if (bufLen >= HW_BUFFER_SAFE_LIMIT) continue; // buffer 快滿，等下一輪

                    var cmd = _queues[i].Peek();

                    // 多軸 PVT 特殊處理
                    if (cmd.Type == CommandType.MultiAxisPVT)
                    {
                        ExecuteMultiAxisPVT(cmd);
                        _queues[i].Dequeue();
                        UpdateQueueLengths();
                        continue;
                    }

                    // Stop / VelocityChange / TargetPositionChange 指令可在 MOVING 時執行
                    if (cmd.Type == CommandType.Stop ||
                        cmd.Type == CommandType.VelocityChange ||
                        cmd.Type == CommandType.TargetPositionChange)
                    {
                        ExecuteCommand(cmd);
                        _queues[i].Dequeue();
                        UpdateQueueLengths();
                        continue;
                    }

                    // 其他運動指令需要軸在 STOP 狀態（或 buffer 有空間直接追加）
                    if (axisState == MotorState.STOP || bufLen > 0)
                    {
                        ExecuteCommand(cmd);
                        _queues[i].Dequeue();
                        UpdateQueueLengths();
                    }
                }
            }
        }

        /// <summary>
        /// 透過 EtherCAT DLL 執行單一運動指令（絕對/相對/PV/PVT/停止/變速）。
        /// </summary>
        private void ExecuteCommand(MotionCommand cmd)
        {
            ushort ret;
            _log.Info($"執行指令：{cmd}");

            switch (cmd.Type)
            {
                case CommandType.MoveAbsolute:
                    {
                        int distPulse = MdegToPulse(cmd.Axis, cmd.Dist);
                        ret = _ecat.CS_ECAT_Slave_CSP_Start_Move(
                            _cardNo, cmd.Axis, 0,
                            distPulse,
                            MdegPerSecToPulsePerSec(cmd.Axis, cmd.StrVel),
                            MdegPerSecToPulsePerSec(cmd.Axis, cmd.ConstVel),
                            MdegPerSecToPulsePerSec(cmd.Axis, cmd.EndVel),
                            cmd.TAcc, cmd.TDec, 0, 1); // SCurve=0, IsAbs=1
                        _log.DllReturn("CSP_Start_Move(Abs)", ret, $"軸{cmd.Axis}: {cmd.Dist} mdeg -> {distPulse} pulse");
                        if (ret == 0) lock (_stateLock) { _state[cmd.Axis] = MotorState.MOVING; }
                        break;
                    }

                case CommandType.MoveRelative:
                    {
                        int distPulse = MdegDeltaToPulse(cmd.Axis, cmd.Dist);
                        ret = _ecat.CS_ECAT_Slave_CSP_Start_Move(
                            _cardNo, cmd.Axis, 0,
                            distPulse,
                            MdegPerSecToPulsePerSec(cmd.Axis, cmd.StrVel),
                            MdegPerSecToPulsePerSec(cmd.Axis, cmd.ConstVel),
                            MdegPerSecToPulsePerSec(cmd.Axis, cmd.EndVel),
                            cmd.TAcc, cmd.TDec, 0, 0); // IsAbs=0
                        _log.DllReturn("CSP_Start_Move(Rel)", ret, $"軸{cmd.Axis}: Δ{cmd.Dist} mdeg -> Δ{distPulse} pulse");
                        if (ret == 0) lock (_stateLock) { _state[cmd.Axis] = MotorState.MOVING; }
                        break;
                    }

                case CommandType.MovePV:
                    {
                        ushort dir = (ushort)(cmd.ConstVel >= 0 ? 0 : 1);
                        int absVel = Math.Abs(MdegPerSecToPulsePerSec(cmd.Axis, cmd.ConstVel));
                        int absStr = Math.Abs(MdegPerSecToPulsePerSec(cmd.Axis, cmd.StrVel));
                        ret = _ecat.CS_ECAT_Slave_CSP_Start_V_Move(
                            _cardNo, cmd.Axis, 0,
                            dir, absStr, absVel, cmd.TAcc, 0);
                        _log.DllReturn("CSP_Start_V_Move", ret);
                        if (ret == 0) lock (_stateLock) { _state[cmd.Axis] = MotorState.MOVING; }
                    }
                    break;

                case CommandType.MovePT:
                    if (cmd.TargetPos != null && cmd.TargetTime != null)
                    {
                        int[] targetPulse = ConvertMdegArrayToPulse(cmd.Axis, cmd.TargetPos, isAbsolute: true);
                        ret = _ecat.CS_ECAT_Slave_CSP_Start_PVTComplete_Move(
                            _cardNo, cmd.Axis, 0,
                            cmd.DataCount, ref targetPulse[0], ref cmd.TargetTime[0],
                            MdegPerSecToPulsePerSec(cmd.Axis, cmd.StrVel),
                            MdegPerSecToPulsePerSec(cmd.Axis, cmd.EndVel), 1); // Abs=1
                        _log.DllReturn("CSP_PVTComplete_Move", ret);
                        if (ret == 0) lock (_stateLock) { _state[cmd.Axis] = MotorState.MOVING; }
                    }
                    break;

                case CommandType.Stop:
                    ret = _ecat.CS_ECAT_Slave_Motion_Sd_Stop(_cardNo, cmd.Axis, 0, cmd.TDec);
                    _log.DllReturn("Sd_Stop", ret, $"軸{cmd.Axis}");
                    break;

                case CommandType.VelocityChange:
                    ret = _ecat.CS_ECAT_Slave_CSP_Velocity_Change(
                        _cardNo, cmd.Axis, 0,
                        MdegPerSecToPulsePerSec(cmd.Axis, cmd.NewTargetSpd), cmd.TSec);
                    _log.DllReturn("CSP_Velocity_Change", ret, $"軸{cmd.Axis} → {cmd.NewTargetSpd} mdeg/s, {cmd.TSec}s");
                    break;

                case CommandType.TargetPositionChange:
                    {
                        int targetPulse = MdegToPulse(cmd.Axis, cmd.Dist);
                        ret = _ecat.CS_ECAT_Slave_CSP_TargetPos_Change(_cardNo, cmd.Axis, 0, targetPulse);
                        _log.DllReturn("CSP_TargetPos_Change", ret, $"軸{cmd.Axis} → {cmd.Dist} mdeg -> {targetPulse} pulse");
                        break;
                    }
            }
        }

        /// <summary>
        /// 執行多軸同步 PVT 指令：逐軸設定 Config 後呼叫 Sync_Move 同步啟動。
        /// </summary>
        private void ExecuteMultiAxisPVT(MotionCommand cmd)
        {
            if (cmd.MultiTargetPos == null || cmd.MultiTargetTime == null ||
                cmd.MultiStrVel == null || cmd.MultiEndVel == null ||
                cmd.MultiDataCount == null)
                return;

            _log.Info("執行多軸同步 PVT...");

            // 1. 對每軸下 PVTComplete_Config
            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                if (cmd.MultiDataCount[i] <= 0) continue;
                int[] targetPulse = ConvertMdegArrayToPulse(i, cmd.MultiTargetPos[i], isAbsolute: true);
                var ret = _ecat.CS_ECAT_Slave_CSP_Start_PVTComplete_Config(
                    _cardNo, i, 0,
                    cmd.MultiDataCount[i],
                    ref targetPulse[0],
                    ref cmd.MultiTargetTime[i][0],
                    MdegPerSecToPulsePerSec(i, cmd.MultiStrVel[i]),
                    MdegPerSecToPulsePerSec(i, cmd.MultiEndVel[i]), 1);
                _log.DllReturn("PVTComplete_Config", ret, $"軸{i}");
            }

            // 2. 同步啟動
            ushort[] axisArray = { 0, 1, 2, 3, 4, 5 };
            ushort[] slotArray = { 0, 0, 0, 0, 0, 0 };
            var syncRet = _ecat.CS_ECAT_Slave_CSP_Start_PVT_Sync_Move(
                _cardNo, AXIS_COUNT, ref axisArray[0], ref slotArray[0]);
            _log.DllReturn("PVT_Sync_Move", syncRet);

            if (syncRet == 0)
            {
                lock (_stateLock)
                {
                    for (int i = 0; i < AXIS_COUNT; i++)
                        if (cmd.MultiDataCount[i] > 0)
                            _state[i] = MotorState.MOVING;
                }
            }
        }

        // ════════════════════════════════════════
        // 安全操作
        // ════════════════════════════════════════

        /// <summary>清除隊列並對所有軸發送緊急停止命令。</summary>
        private void DoEstop()
        {
            _log.Warn("執行全軸緊急停止");
            ClearAllQueues();
            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                var ret = _ecat.CS_ECAT_Slave_Motion_Emg_Stop(_cardNo, i, 0);
                _log.DllReturn("Emg_Stop", ret, $"軸{i}");
            }
            lock (_stateLock)
            {
                for (int i = 0; i < AXIS_COUNT; i++)
                    _state[i] = MotorState.STOP;
            }
        }

        /// <summary>清除隊列並對所有軸發送減速停止命令（看門狗觸發時使用）。</summary>
        private void DoSdStopAll()
        {
            _log.Warn("執行全軸減速停止（看門狗觸發）");
            ClearAllQueues();
            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                var ret = _ecat.CS_ECAT_Slave_Motion_Sd_Stop(_cardNo, i, 0, DEFAULT_SDSTOP_TDEC);
                _log.DllReturn("Sd_Stop", ret, $"軸{i}");
            }
        }

        /// <summary>
        /// 非阻塞式處理各軸立即停止請求。
        /// 每次主迴圈迭代呼叫一次，以狀態機方式推進：
        ///   新請求 → VelocityChange(0, tDec) → 等待 deadline → Sd_Stop
        /// 不使用 Thread.Sleep，確保通訊線程不被阻塞。
        /// </summary>
        private void ProcessImmediateStops()
        {
            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                // 1. 檢查新的停止請求
                bool newRequest = false;
                double tDec = 0;

                lock (_immediateStopLock)
                {
                    if (_immediateStopRequested[i])
                    {
                        newRequest = true;
                        tDec = _immediateStopTDec[i];
                        _immediateStopRequested[i] = false;
                    }
                }

                if (newRequest)
                {
                    _log.Info($"軸 {i} 立即停止：發送 VelocityChange(0, {tDec}s)");
                    var ret = _ecat.CS_ECAT_Slave_CSP_Velocity_Change(_cardNo, i, 0, 0, tDec);
                    _log.DllReturn("CSP_Velocity_Change", ret, $"軸{i} → 0 mdeg/s, {tDec}s");
                    _stopPhase[i] = StopPhase.Decelerating;
                    _stopDeadlineTicks[i] = DateTime.UtcNow.Ticks
                        + (long)((tDec + 0.05) * TimeSpan.TicksPerSecond);
                    continue;
                }

                // 2. 推進既有的減速狀態
                if (_stopPhase[i] == StopPhase.Decelerating
                    && DateTime.UtcNow.Ticks >= _stopDeadlineTicks[i])
                {
                    var ret = _ecat.CS_ECAT_Slave_Motion_Sd_Stop(_cardNo, i, 0, 0.05);
                    _log.DllReturn("Sd_Stop", ret, $"軸{i} (立即停止最終確認)");
                    _stopPhase[i] = StopPhase.None;
                }
            }
        }

        /// <summary>
        /// 處理 AbortAndChangePosition 請求。
        /// 流程：
        /// 1. 全軸先做 CSP_Velocity_Change(0, tDec)，避免 CSP 速度環殘留。
        /// 2. 立即呼叫 CSP_Abort_and_Change_Position 切換到指定目標。
        /// 3. 等待 tDec 後，再補一輪 Sd_Stop 收尾，強制脫離殘留追逐。
        /// </summary>
        private void ProcessAbortAndChangePosition()
        {
            int[] targetMdeg;
            double tDec;
            lock (_abortChangePosLock)
            {
                if (!_abortChangePosRequested && _abortChangePosPhase == AbortChangePosPhase.None) return;
                targetMdeg = (int[])_abortChangePosTargetMdeg.Clone();
                tDec = _abortChangePosTDec;
            }

            if (_abortChangePosPhase == AbortChangePosPhase.None)
            {
                for (ushort i = 0; i < AXIS_COUNT; i++)
                {
                    var velRet = _ecat.CS_ECAT_Slave_CSP_Velocity_Change(_cardNo, i, 0, 0, tDec);
                    _log.DllReturn("CSP_Velocity_Change", velRet, $"AbortPhase 軸{i} → 0 mdeg/s, {tDec}s");
                }

                var nodeIds = new ushort[] { 0, 1, 2, 3, 4, 5 };
                var slotIds = new ushort[] { 0, 0, 0, 0, 0, 0 };
                var targetPulse = new int[AXIS_COUNT];
                for (int i = 0; i < AXIS_COUNT; i++)
                    targetPulse[i] = MdegToPulse(i, targetMdeg[i]);

                int maxTargetDeltaMdeg = 1;
                lock (_stateLock)
                {
                    for (int i = 0; i < AXIS_COUNT; i++)
                        maxTargetDeltaMdeg = Math.Max(maxTargetDeltaMdeg, Math.Abs(targetMdeg[i] - _pos[i]));
                }

                int maxVelMdegPerSec = Math.Clamp(maxTargetDeltaMdeg * 4, 5_000, 80_000);
                int maxVelPulse = MdegPerSecToPulsePerSec(0, maxVelMdegPerSec);

                var ret = _ecat.CS_ECAT_Slave_CSP_Abort_and_Change_Position(
                    _cardNo, AXIS_COUNT,
                    ref nodeIds[0], ref slotIds[0], ref targetPulse[0],
                    maxVelPulse, 0, 0.05, tDec, 0);
                _log.DllReturn("CSP_Abort_and_Change_Position", ret, $"tDec={tDec}s, maxVel={maxVelMdegPerSec} mdeg/s");

                lock (_abortChangePosLock)
                {
                    _abortChangePosRequested = false;
                    _abortChangePosPhase = AbortChangePosPhase.FinalSdStopPending;
                    _abortChangePosDeadlineTicks = DateTime.UtcNow.Ticks
                        + (long)((tDec + 0.1) * TimeSpan.TicksPerSecond);
                }
                return;
            }

            if (_abortChangePosPhase == AbortChangePosPhase.FinalSdStopPending
                && DateTime.UtcNow.Ticks >= _abortChangePosDeadlineTicks)
            {
                for (ushort i = 0; i < AXIS_COUNT; i++)
                {
                    var ret = _ecat.CS_ECAT_Slave_Motion_Sd_Stop(_cardNo, i, 0, 0.05);
                    _log.DllReturn("Sd_Stop", ret, $"AbortPhase 軸{i} 最終收尾");
                }

                lock (_abortChangePosLock)
                {
                    _abortChangePosPhase = AbortChangePosPhase.None;
                }
            }
        }

        /// <summary>對所有軸執行警報復歸（Ralm）並重新 Servo ON。</summary>
        private void DoRalm()
        {
            _log.Info("執行全軸警報復歸");
            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                var ret = _ecat.CS_ECAT_Slave_Motion_Ralm(_cardNo, i, 0);
                _log.DllReturn("Ralm", ret, $"軸{i}");

                // 重新 Servo ON
                ret = _ecat.CS_ECAT_Slave_Motion_Set_Svon(_cardNo, i, 0, 1);
                _log.DllReturn("Set_Svon", ret, $"軸{i} ON");
            }
            lock (_stateChangeLock)
            {
                _requestedCardState = CardState.READY;
                _cardStateChangeRequest = true;
            }
        }

        /// <summary>Servo OFF 所有軸並關閉 EtherCAT 主站。</summary>
        private void DoEnd()
        {
            _log.Info("開始關閉程序...");
            // Servo OFF
            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                _ecat.CS_ECAT_Slave_Motion_Set_Svon(_cardNo, i, 0, 0);
                lock (_stateLock) { _state[i] = MotorState.NULL; }
            }
            // 關閉主站
            _ecat.CS_ECAT_Master_Close();
            _cardState = CardState.NULL;
            _log.Info("EtherCAT 主站已關閉");
        }

        // ════════════════════════════════════════
        // 看門狗
        // ════════════════════════════════════════

        /// <summary>檢查主線程心跳是否逾時，逾時則觸發全軸減速停止。</summary>
        private void CheckMainThreadWatchdog()
        {
            var elapsed = DateTime.UtcNow.Ticks - Interlocked.Read(ref _mainThreadHeartbeat);
            if (elapsed > TimeSpan.FromSeconds(WATCHDOG_TIMEOUT_SEC).Ticks)
            {
                _log.Fatal($"主線程心跳逾時！已超過 {WATCHDOG_TIMEOUT_SEC} 秒未更新");
                _sdStopAllRequested = true;
                lock (_stateChangeLock)
                {
                    _requestedCardState = CardState.ALARM;
                    _cardStateChangeRequest = true;
                }
            }
        }

        /// <summary>由主線程定期呼叫，處理通訊線程建議的狀態變更</summary>
        public CardState ProcessCardStateChange()
        {
            lock (_stateChangeLock)
            {
                if (_cardStateChangeRequest)
                {
                    _cardState = _requestedCardState;
                    _cardStateChangeRequest = false;
                }
            }
            return _cardState;
        }

        // ════════════════════════════════════════

        /// <summary>釋放資源並等待通訊線程結束。</summary>
        public void Dispose()
        {
            if (_thread != null && _thread.IsAlive)
            {
                if (_cardState >= CardState.CONNCET)
                {
                    _estopRequested = true;
                    _endRequested = true;
                }
                else
                {
                    _running = false;
                }
            }
            else
            {
                _running = false;
            }

            _startSignal.Set();
            _initSignal.Set();
            _calibrateSignal.Set();

            if (_thread != null && !_thread.Join(TimeSpan.FromSeconds(5)))
            {
                _running = false;
                _thread.Join(TimeSpan.FromSeconds(1));
            }

            _startSignal.Dispose();
            _initSignal.Dispose();
            _calibrateSignal.Dispose();
        }

        private int PulseToMdeg(int axis, int pulse)
            => (int)(1000L * (pulse - _zeroPulse[axis]) / _pulse2Ang[axis]);

        private int PulseSpeedToMdegPerSec(int axis, int pulsePerSec)
            => (int)(1000L * pulsePerSec / _pulse2Ang[axis]);

        private int MdegToPulse(int axis, int mdeg)
            => (int)((long)mdeg * _pulse2Ang[axis] / 1000) + _zeroPulse[axis];

        private int MdegDeltaToPulse(int axis, int mdeg)
            => (int)((long)mdeg * _pulse2Ang[axis] / 1000);

        private int MdegPerSecToPulsePerSec(int axis, int mdegPerSec)
            => (int)((long)mdegPerSec * _pulse2Ang[axis] / 1000);

        private int[] ConvertMdegArrayToPulse(ushort axis, int[] source, bool isAbsolute)
        {
            var pulse = new int[source.Length];
            for (int i = 0; i < source.Length; i++)
                pulse[i] = isAbsolute ? MdegToPulse(axis, source[i]) : MdegDeltaToPulse(axis, source[i]);
            return pulse;
        }
    }
}
