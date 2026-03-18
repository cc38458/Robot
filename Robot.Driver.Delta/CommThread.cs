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

        // ── 指令隊列（每軸獨立） ──
        private readonly Queue<MotionCommand>[] _queues = new Queue<MotionCommand>[AXIS_COUNT];
        private readonly object _queueLock = new();

        // ── 連線資訊 ──
        private ushort _cardNo;
        private ushort _axisNum;
        private int[] _pulse2Ang = DEFAULT_PULSE2ANG;
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

        // ── 日誌 ──
        private readonly RobotLogger _log;
        private readonly IEtherCatApi _ecat;
        private readonly bool _isMockBackend;

        // ── 零點設定檔路徑 ──
        private readonly string _zeroConfigPath;

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
            _thread.Start();

            // 等待連線結果
            _startSignal.Wait(TimeSpan.FromSeconds(20));
            return _connectResult;
        }

        public bool RequestInitial()
        {
            if (_cardState != CardState.CONNCET) return false;
            _initRequested = true;
            _initSignal.Reset();
            _initSignal.Wait(TimeSpan.FromSeconds(10));
            return _initResult;
        }

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

        public void RequestEstop()
        {
            _estopRequested = true;
            ClearAllQueues();
            _log.Warn("收到緊急停止請求");
        }

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

        // ════════════════════════════════════════
        // 指令入隊（主線程呼叫）
        // ════════════════════════════════════════

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

        private void UpdateQueueLengths()
        {
            for (int i = 0; i < AXIS_COUNT; i++)
                _queueLength[i] = _queues[i].Count;
        }

        // ════════════════════════════════════════
        // 通訊線程主迴圈
        // ════════════════════════════════════════

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

                    // ── 立即停止請求（每軸獨立） ──
                    DoImmediateStopIfRequested();

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

                // 設定齒輪比
                ret = _ecat.CS_ECAT_Slave_CSP_Set_Gear(_cardNo, i, 0, _pulse2Ang[i], 1000, 1);
                _log.DllReturn("Set_Gear", ret, $"軸{i}: {_pulse2Ang[i]}/1000");
                if (ret != 0) return false;

                // 啟用虛擬位置
                ret = _ecat.CS_ECAT_Slave_CSP_Virtual_Set_Enable(_cardNo, i, 0, 1);
                _log.DllReturn("Virtual_Set_Enable", ret, $"軸{i}");
                if (ret != 0) return false;

                // 讀取當前實際位置並設定虛擬座標
                int pos = 0;
                _ecat.CS_ECAT_Slave_Motion_Get_Actual_Position(_cardNo, i, 0, ref pos);
                _log.DllReturn("Get_Actual_Position", 0, $"軸{i}:真實位置={pos}");

                // Mock 模式：pos 已是 mdeg，直接使用
                // Real 模式：pos 是 encoder pulse，需轉換
                int initPos = _isMockBackend ? pos : (int)(1000L * (pos - _zeroPulse[i]) / _pulse2Ang[i]);
                ret = _ecat.CS_ECAT_Slave_CSP_Virtual_Set_Command(_cardNo, i, 0, initPos);
                _log.DllReturn("Virtual_Set_Command", ret, $"軸{i}: 當前位置={initPos}");
                

                lock (_stateLock) { _state[i] = MotorState.STOP; }
            }

            _log.Info("所有軸初始化完成，Servo ON");
            return true;
        }

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
                        _pulse2Ang = config.Pulse2Ang;
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
            _pulse2Ang = DEFAULT_PULSE2ANG;
            SaveZeroConfig([0,0,0,0,0,0]);
        }

        private void SaveZeroConfig(int[] newPos)
        {
            try
            {
                var config = new AxisZeroConfig
                {
                    ZeroPulse = newPos,
                    Pulse2Ang = _pulse2Ang,
                    LastModified = DateTime.Now,
                    Note = "各軸零點絕對脈波數與齒輪比設定",
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
        /// 直接從硬體讀取各軸當前位置，換算為絕對脈波數後存入設定檔。
        /// 邏輯：Get_Position 在 Virtual 模式回傳虛擬座標（= 物理 − oldZeroPulse），
        ///       因此絕對物理脈波 = oldZeroPulse + virtualPos。
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
                    _ecat.CS_ECAT_Slave_Motion_Get_Actual_Position(_cardNo, i, 0, ref pos);
                    newZeroPulse[i] =  pos;
                }
                SaveZeroConfig(newZeroPulse);
                _log.Info("原點標定完成，已儲存零點設定檔");

                for (ushort i = 0; i < AXIS_COUNT; i++)
                {
                    // 設定當前位置
                    _ecat.CS_ECAT_Slave_CSP_Virtual_Set_Command(_cardNo, i, 0, 0);
                    _log.DllReturn("Virtual_Set_Command", 0, $"軸{i}: 當前位置={0}");
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

        private void DoPollStatus()
        {
            bool anyAlarm = false;
            ushort ret;

            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                int pos = 0, speed = 0;
                ushort mdone = 0, statusWord = 0;

                ret = _ecat.CS_ECAT_Slave_Motion_Get_Actual_Position(_cardNo, i, 0, ref pos);
                ret = _ecat.CS_ECAT_Slave_Motion_Get_Current_Speed(_cardNo, i, 0, ref speed);
                ret = _ecat.CS_ECAT_Slave_Motion_Get_Mdone(_cardNo, i, 0, ref mdone);
                ret = _ecat.CS_ECAT_Slave_Motion_Get_StatusWord(_cardNo, i, 0, ref statusWord);

                lock (_stateLock)
                {
                    // Mock 模式：pos 已是 mdeg，不需轉換
                    // Real 模式：pos 是 encoder pulse，需 (pulse - zeroPulse) * 1000 / pulse2Ang
                    _pos[i] = _isMockBackend ? pos : (int)(1000L * (pos - _zeroPulse[i]) / _pulse2Ang[i]);
                    _speed[i] = speed;

                    // 檢查警報（StatusWord Bit3）
                    if ((statusWord & 0x0008) != 0)
                    {
                        if (_state[i] != MotorState.ALARM)
                            _log.Warn($"軸 {i} 觸發警報 (StatusWord=0x{statusWord:X4})");
                        _state[i] = MotorState.ALARM;
                        anyAlarm = true;
                    }
                    else if (_state[i] == MotorState.ALARM)
                    {
                        // 警報已清除
                        _state[i] = MotorState.STOP;
                    }
                    else if (_state[i] == MotorState.MOVING && mdone == 0)
                    {
                        // CSP 模式：Mdone==0 表示靜止
                        _state[i] = MotorState.STOP;
                        _log.Debug($"軸 {i} 運動完成,當前位置: {_pos[i]}");

                        // 檢查是否為最後一個指令且 endVel != 0 → 自動 Sd_Stop
                        CheckAutoSdStop(i);
                    }
                    else if (_state[i] == MotorState.STOP && mdone != 0)
                    {
                        _state[i] = MotorState.MOVING;
                    }
                }
            }

            // 任意軸警報 → 全軸急停
            if (anyAlarm && _cardState == CardState.READY)
            {
                _log.Error("偵測到軸警報，執行全軸緊急停止");
                DoEstop();
                _requestedCardState = CardState.ALARM;
                _cardStateChangeRequest = true;
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

                    // Stop / VelocityChange 指令可在 MOVING 時執行
                    if (cmd.Type == CommandType.Stop || cmd.Type == CommandType.VelocityChange)
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

        private void ExecuteCommand(MotionCommand cmd)
        {
            ushort ret;
            _log.Info($"執行指令：{cmd}");

            switch (cmd.Type)
            {
                case CommandType.MoveAbsolute:
                    ret = _ecat.CS_ECAT_Slave_CSP_Start_Move(
                        _cardNo, cmd.Axis, 0,
                        cmd.Dist, cmd.StrVel, cmd.ConstVel, cmd.EndVel,
                        cmd.TAcc, cmd.TDec, 0, 1); // SCurve=0, IsAbs=1
                    _log.DllReturn("CSP_Start_Move(Abs)", ret);
                    if (ret == 0) lock (_stateLock) { _state[cmd.Axis] = MotorState.MOVING; }
                    break;

                case CommandType.MoveRelative:
                    ret = _ecat.CS_ECAT_Slave_CSP_Start_Move(
                        _cardNo, cmd.Axis, 0,
                        cmd.Dist, cmd.StrVel, cmd.ConstVel, cmd.EndVel,
                        cmd.TAcc, cmd.TDec, 0, 0); // IsAbs=0
                    _log.DllReturn("CSP_Start_Move(Rel)", ret);
                    if (ret == 0) lock (_stateLock) { _state[cmd.Axis] = MotorState.MOVING; }
                    break;

                case CommandType.MovePV:
                    {
                        ushort dir = (ushort)(cmd.ConstVel >= 0 ? 0 : 1);
                        int absVel = Math.Abs(cmd.ConstVel);
                        int absStr = Math.Abs(cmd.StrVel);
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
                        ret = _ecat.CS_ECAT_Slave_CSP_Start_PVTComplete_Move(
                            _cardNo, cmd.Axis, 0,
                            cmd.DataCount, ref cmd.TargetPos[0], ref cmd.TargetTime[0],
                            cmd.StrVel, cmd.EndVel, 1); // Abs=1
                        _log.DllReturn("CSP_PVTComplete_Move", ret);
                        if (ret == 0) lock (_stateLock) { _state[cmd.Axis] = MotorState.MOVING; }
                    }
                    break;

                case CommandType.Stop:
                    ret = _ecat.CS_ECAT_Slave_Motion_Sd_Stop(_cardNo, cmd.Axis, 0, cmd.TDec);
                    _log.DllReturn("Sd_Stop", ret, $"軸{cmd.Axis}");
                    break;

                case CommandType.VelocityChange:
                    ret = _ecat.CS_ECAT_Slave_CSP_Velocity_Change(_cardNo, cmd.Axis, 0, cmd.NewTargetSpd, cmd.TSec);
                    _log.DllReturn("CSP_Velocity_Change", ret, $"軸{cmd.Axis} → {cmd.NewTargetSpd} mdeg/s, {cmd.TSec}s");
                    break;
            }
        }

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
                var ret = _ecat.CS_ECAT_Slave_CSP_Start_PVTComplete_Config(
                    _cardNo, i, 0,
                    cmd.MultiDataCount[i],
                    ref cmd.MultiTargetPos[i][0],
                    ref cmd.MultiTargetTime[i][0],
                    cmd.MultiStrVel[i], cmd.MultiEndVel[i], 1);
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
        /// 處理各軸立即停止請求：VelocityChange(0, tDec) → 等待 tDec → Sd_Stop
        /// 此操作在通訊線程中執行，會阻塞 tDec 秒（等待減速完成）
        /// </summary>
        private void DoImmediateStopIfRequested()
        {
            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                bool requested = false;
                double tDec = 0;

                lock (_immediateStopLock)
                {
                    if (_immediateStopRequested[i])
                    {
                        requested = true;
                        tDec = _immediateStopTDec[i];
                        _immediateStopRequested[i] = false;
                    }
                }

                if (!requested) continue;

                _log.Info($"軸 {i} 立即停止：VelocityChange(0, {tDec}s) → 等待 → Sd_Stop");

                // Step 1: 變速至 0
                var ret = _ecat.CS_ECAT_Slave_CSP_Velocity_Change(_cardNo, i, 0, 0, tDec);
                _log.DllReturn("CSP_Velocity_Change", ret, $"軸{i} → 0 mdeg/s, {tDec}s");

                // Step 2: 等待減速完成
                int waitMs = (int)(tDec * 1000) + 50; // 多等 50ms 確保完成
                Thread.Sleep(waitMs);

                // Step 3: Sd_Stop 確保完全停止
                ret = _ecat.CS_ECAT_Slave_Motion_Sd_Stop(_cardNo, i, 0, 0.05);
                _log.DllReturn("Sd_Stop", ret, $"軸{i} (立即停止最終確認)");
            }
        }

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
            _requestedCardState = CardState.READY;
            _cardStateChangeRequest = true;
        }

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

        private void CheckMainThreadWatchdog()
        {
            var elapsed = DateTime.UtcNow.Ticks - Interlocked.Read(ref _mainThreadHeartbeat);
            if (elapsed > TimeSpan.FromSeconds(WATCHDOG_TIMEOUT_SEC).Ticks)
            {
                _log.Fatal($"主線程心跳逾時！已超過 {WATCHDOG_TIMEOUT_SEC} 秒未更新");
                _sdStopAllRequested = true;
                _requestedCardState = CardState.ALARM;
                _cardStateChangeRequest = true;
            }
        }

        /// <summary>由主線程定期呼叫，處理通訊線程建議的狀態變更</summary>
        public CardState ProcessCardStateChange()
        {
            if (_cardStateChangeRequest)
            {
                _cardState = _requestedCardState;
                _cardStateChangeRequest = false;
            }
            return _cardState;
        }

        // ════════════════════════════════════════

        public void Dispose()
        {
            _running = false;
            _startSignal.Set();
            _initSignal.Set();
            _calibrateSignal.Set();
            _thread?.Join(TimeSpan.FromSeconds(5));
            _startSignal.Dispose();
            _initSignal.Dispose();
            _calibrateSignal.Dispose();
        }
    }
}
