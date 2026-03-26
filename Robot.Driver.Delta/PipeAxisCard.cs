using System.Diagnostics;
using System.IO.Pipes;
using Robot.Core.Enums;
using Robot.Core.Interfaces;
using Robot.Core.IPC;
using Robot.Core.Logging;

namespace Robot.Driver.Delta
{
    /// <summary>
    /// IAxisCard 遠端代理：透過命名管道傳送指令、透過共享記憶體讀取狀態。
    /// 當 CommService 崩潰時，自動將 AxisCardState 切換為 ALARM，
    /// 由上層（RA605RobotApp）決定是否進行 in-process 降級接管。
    /// </summary>
    public class PipeAxisCard : IAxisCard
    {
        private const int AXIS_COUNT = 6;
        private const int CONNECT_TIMEOUT_MS = 10000;
        private const int HEARTBEAT_WRITE_INTERVAL_MS = 500;
        private const double COMM_HEARTBEAT_TIMEOUT_SEC = 1.0;

        private readonly RobotLogger _log;
        private readonly string _commServicePath;
        private readonly string _shmPath;
        private readonly string[] _commServiceArgs;

        private Process? _commProcess;
        private NamedPipeClientStream? _pipe;
        private StreamReader? _reader;
        private StreamWriter? _writer;
        private SharedMemoryState? _shm;
        private Timer? _heartbeatTimer;
        private bool _disposed;
        private volatile bool _commServiceDead;

        // 狀態快取（從共享記憶體讀取）
        private readonly int[] _pos = new int[AXIS_COUNT];
        private readonly int[] _speed = new int[AXIS_COUNT];
        private readonly MotorState[] _state = new MotorState[AXIS_COUNT];
        private readonly int[] _queueLength = new int[AXIS_COUNT];
        private CardState _cardState = CardState.NULL;
        private readonly object _pipeLock = new();

        /// <summary>
        /// 建構遠端軸卡代理。
        /// </summary>
        /// <param name="logger">日誌記錄器</param>
        /// <param name="commServicePath">CommService 執行檔路徑</param>
        /// <param name="zeroConfigPath">零點設定檔路徑</param>
        /// <param name="useMock">是否使用 Mock 後端</param>
        public PipeAxisCard(RobotLogger logger, string commServicePath,
                            string zeroConfigPath = "axis_zero_config.json",
                            bool useMock = false)
        {
            _log = logger;
            _commServicePath = commServicePath;
            _shmPath = Path.Combine(Path.GetTempPath(), SharedMemoryState.FILE_NAME);

            var argList = new List<string>();
            if (useMock) argList.Add("--mock");
            argList.Add($"--zero-config={zeroConfigPath}");
            argList.Add($"--shm-path={_shmPath}");
            _commServiceArgs = argList.ToArray();
        }

        /// <summary>CommService 是否已崩潰（供上層判斷是否需要 in-process 降級）。</summary>
        public bool IsCommServiceDead => _commServiceDead;

        // ════════════════════════════════════════
        // IAxisCard 屬性（從共享記憶體讀取）
        // ════════════════════════════════════════

        public int[] Pos { get { RefreshFromShm(); return (int[])_pos.Clone(); } }
        public int[] Speed { get { RefreshFromShm(); return (int[])_speed.Clone(); } }
        public MotorState[] State { get { RefreshFromShm(); return (MotorState[])_state.Clone(); } }
        public CardState AxisCardState { get { RefreshFromShm(); return _cardState; } }
        public int[] QueueLength { get { RefreshFromShm(); return (int[])_queueLength.Clone(); } }

        /// <summary>從共享記憶體更新狀態快取。若 CommService 崩潰則回報 ALARM。</summary>
        private void RefreshFromShm()
        {
            if (_commServiceDead)
            {
                _cardState = CardState.ALARM;
                return;
            }

            if (_shm == null) return;

            // 檢查 CommService 心跳
            if (_shm.IsCommHeartbeatTimeout(COMM_HEARTBEAT_TIMEOUT_SEC))
            {
                _log.Fatal("CommService 心跳逾時！切換為 ALARM 狀態");
                _commServiceDead = true;
                _cardState = CardState.ALARM;
                return;
            }

            if (!_shm.TryReadState(out var cs, _pos, _speed, _state, _queueLength))
            {
                _log.Warn("共享記憶體讀取失敗（SeqLock 衝突）");
                return;
            }

            _cardState = cs;
        }

        // ════════════════════════════════════════
        // IAxisCard 連線管理
        // ════════════════════════════════════════

        public bool Start()
        {
            if (_commProcess != null && !_commProcess.HasExited)
            {
                _log.Warn("CommService 已在執行中");
                return SendCommand(new PipeRequest { Cmd = "Start" });
            }

            try
            {
                // 啟動 CommService 行程
                _log.Info($"啟動 CommService：{_commServicePath}");
                bool launchWithDotnet = string.Equals(
                    Path.GetExtension(_commServicePath), ".dll", StringComparison.OrdinalIgnoreCase);
                var psi = new ProcessStartInfo
                {
                    FileName = launchWithDotnet ? "dotnet" : _commServicePath,
                    Arguments = launchWithDotnet
                        ? $"\"{_commServicePath}\" {string.Join(" ", _commServiceArgs)}"
                        : string.Join(" ", _commServiceArgs),
                    UseShellExecute = false,
                    CreateNoWindow = true,
                };
                _commProcess = Process.Start(psi);

                if (_commProcess == null)
                {
                    _log.Error("無法啟動 CommService 行程");
                    return false;
                }

                _commProcess.EnableRaisingEvents = true;
                _commProcess.Exited += (_, _) =>
                {
                    _log.Fatal("CommService 行程已終止！");
                    _commServiceDead = true;
                };

                // 等待一小段時間讓 CommService 啟動管道伺服器
                Thread.Sleep(1000);

                // 連接命名管道
                _pipe = new NamedPipeClientStream(".", PipeProtocol.PIPE_NAME,
                    PipeDirection.InOut, PipeOptions.Asynchronous);
                _pipe.Connect(CONNECT_TIMEOUT_MS);

                _reader = new StreamReader(_pipe, leaveOpen: true);
                _writer = new StreamWriter(_pipe, leaveOpen: true) { AutoFlush = true };

                // 開啟共享記憶體
                _shm = new SharedMemoryState(_shmPath);

                // 啟動心跳寫入計時器
                _heartbeatTimer = new Timer(HeartbeatCallback, null,
                    HEARTBEAT_WRITE_INTERVAL_MS, HEARTBEAT_WRITE_INTERVAL_MS);

                _commServiceDead = false;
                _log.Info("已連接 CommService");

                // 傳送 Start 指令
                return SendCommand(new PipeRequest { Cmd = "Start" });
            }
            catch (Exception ex)
            {
                _log.Error("啟動 CommService 失敗", ex);
                _commServiceDead = true;
                return false;
            }
        }

        public bool End() => SendCommand(new PipeRequest { Cmd = "End" });
        public bool Initial() => SendCommand(new PipeRequest { Cmd = "Initial" });
        public bool Estop() => SendCommand(new PipeRequest { Cmd = "Estop" });
        public bool Ralm() => SendCommand(new PipeRequest { Cmd = "Ralm" });
        public bool CalibrateZero() => SendCommand(new PipeRequest { Cmd = "CalibrateZero" });

        // ════════════════════════════════════════
        // IAxisCard 運動控制
        // ════════════════════════════════════════

        public bool Stop(ushort axis, double tDec)
            => SendCommand(new PipeRequest { Cmd = "Stop", Axis = axis, TDec = tDec });

        public bool ChangeVelocity(ushort axis, int newSpeed, double tSec)
            => SendCommand(new PipeRequest { Cmd = "ChangeVelocity", Axis = axis, NewSpeed = newSpeed, TSec = tSec });

        public bool MoveAbsolute(ushort axis, int dist, int strVel, int constVel,
                                  int endVel, double tAcc, double tDec)
            => SendCommand(new PipeRequest
            {
                Cmd = "MoveAbsolute", Axis = axis, Dist = dist,
                StrVel = strVel, ConstVel = constVel, EndVel = endVel,
                TAcc = tAcc, TDec = tDec,
            });

        public bool MoveRelative(ushort axis, int dist, int strVel, int constVel,
                                  int endVel, double tAcc, double tDec)
            => SendCommand(new PipeRequest
            {
                Cmd = "MoveRelative", Axis = axis, Dist = dist,
                StrVel = strVel, ConstVel = constVel, EndVel = endVel,
                TAcc = tAcc, TDec = tDec,
            });

        public bool MovePV(ushort axis, int strVel, int constVel, double tAcc)
            => SendCommand(new PipeRequest
            {
                Cmd = "MovePV", Axis = axis,
                StrVel = strVel, ConstVel = constVel, TAcc = tAcc,
            });

        public bool MovePT(ushort axis, int dataCnt, int[] targetPos,
                           int[] targetTime, int strVel, int endVel)
            => SendCommand(new PipeRequest
            {
                Cmd = "MovePT", Axis = axis, DataCnt = dataCnt,
                TargetPos = targetPos, TargetTime = targetTime,
                StrVel = strVel, EndVel = endVel,
            });

        public bool MoveMultiAxisPVT(int[] dataCount, int[][] targetPos,
                                      int[][] targetTime, int[] strVel, int[] endVel)
            => SendCommand(new PipeRequest
            {
                Cmd = "MoveMultiAxisPVT",
                MultiDataCount = dataCount, MultiTargetPos = targetPos,
                MultiTargetTime = targetTime, MultiStrVel = strVel, MultiEndVel = endVel,
            });

        public bool MoveHome(int constVel, double tAcc, double tDec)
            => SendCommand(new PipeRequest
            {
                Cmd = "MoveHome", ConstVel = constVel, TAcc = tAcc, TDec = tDec,
            });

        // ════════════════════════════════════════
        // 內部
        // ════════════════════════════════════════

        /// <summary>透過管道發送指令並等待回應。</summary>
        private bool SendCommand(PipeRequest request)
        {
            if (_commServiceDead)
            {
                _log.Warn($"CommService 已崩潰，拒絕指令 {request.Cmd}");
                return false;
            }

            lock (_pipeLock)
            {
                try
                {
                    if (_writer == null || _reader == null)
                    {
                        _log.Warn("管道尚未建立");
                        return false;
                    }

                    _writer.WriteLine(PipeProtocol.Serialize(request));

                    var responseLine = _reader.ReadLine();
                    if (responseLine == null)
                    {
                        _log.Error("管道讀取失敗（CommService 可能已終止）");
                        _commServiceDead = true;
                        return false;
                    }

                    var response = PipeProtocol.Deserialize<PipeResponse>(responseLine);
                    if (response == null)
                    {
                        _log.Error("無法解析管道回應");
                        return false;
                    }

                    if (!response.Ok)
                        _log.Warn($"指令 {request.Cmd} 失敗：{response.Error}");

                    return response.Ok;
                }
                catch (IOException ex)
                {
                    _log.Error($"管道通訊錯誤：{ex.Message}");
                    _commServiceDead = true;
                    return false;
                }
                catch (Exception ex)
                {
                    _log.Error($"SendCommand 異常", ex);
                    return false;
                }
            }
        }

        /// <summary>心跳計時器回呼：寫入主程式心跳 + 檢查 CommService 存活。</summary>
        private void HeartbeatCallback(object? state)
        {
            if (_disposed) return;

            try
            {
                _shm?.WriteMainHeartbeat();

                // 檢查 CommService 行程是否還在
                if (_commProcess != null && _commProcess.HasExited && !_commServiceDead)
                {
                    _log.Fatal("CommService 行程已意外終止！");
                    _commServiceDead = true;
                }

                // 檢查共享記憶體心跳
                if (_shm != null && _shm.IsCommHeartbeatTimeout(COMM_HEARTBEAT_TIMEOUT_SEC))
                {
                    if (!_commServiceDead)
                    {
                        _log.Fatal("CommService 心跳逾時！");
                        _commServiceDead = true;
                    }
                }
            }
            catch (Exception ex)
            {
                _log.Error($"心跳檢查錯誤：{ex.Message}");
            }
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            _heartbeatTimer?.Change(Timeout.Infinite, Timeout.Infinite);

            if (!_commServiceDead)
            {
                try { SendCommand(new PipeRequest { Cmd = "Estop" }); } catch { }
                try
                {
                    Thread.Sleep(300);
                    SendCommand(new PipeRequest { Cmd = "End" });
                }
                catch { }
            }

            _heartbeatTimer?.Dispose();

            _reader?.Dispose();
            _writer?.Dispose();
            _pipe?.Dispose();
            _shm?.Dispose();

            // 不主動終止 CommService 行程 — 它會偵測到管道斷線後自行安全關機
            GC.SuppressFinalize(this);
        }
    }
}
