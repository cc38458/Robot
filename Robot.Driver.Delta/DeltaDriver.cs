using Robot.Core.Enums;
using Robot.Core.Interfaces;
using Robot.Core.Logging;
using Robot.Core.Models;

namespace Robot.Driver.Delta
{
    /// <summary>
    /// Delta EtherCAT 軸卡驅動 — IAxisCard 實作
    /// 本類別運行於主線程，透過 CommThread 委派所有 DLL 呼叫。
    /// 主線程職責：狀態快取、指令驗證、入隊、心跳維護。
    /// </summary>
    public class DeltaDriver : IAxisCard
    {
        private const int AXIS_COUNT = 6;
        private const int HEARTBEAT_INTERVAL_MS = 200;
        private const int CONNECT_WAIT_TIMEOUT_MS = 20000;
        private const int CONNECT_WAIT_POLL_MS = 50;

        private readonly RobotLogger _log;
        private readonly CommThread _comm;
        private readonly Timer _heartbeatTimer;

        // ── 快取（由 RefreshState 更新） ──
        private readonly int[] _pos = new int[AXIS_COUNT];
        private readonly int[] _speed = new int[AXIS_COUNT];
        private readonly MotorState[] _state = new MotorState[AXIS_COUNT];
        private readonly int[] _queueLength = new int[AXIS_COUNT];
        private CardState _cardState = CardState.NULL;
        private bool _disposed;

        /// <summary>
        /// 建構 Delta 驅動
        /// </summary>
        /// <param name="logger">日誌系統</param>
        /// <param name="zeroConfigPath">零點設定 JSON 檔路徑</param>
        /// <param name="useMockBackend">true: 使用虛擬手臂後端；false: 使用實體 EtherCAT 後端</param>
        public DeltaDriver(RobotLogger logger, string zeroConfigPath = "axis_zero_config.json", bool useMockBackend = false)
        {
            _log = logger;
            _comm = new CommThread(logger, zeroConfigPath, useMockBackend);
            _heartbeatTimer = new Timer(HeartbeatCallback, null,
                Timeout.Infinite, Timeout.Infinite);
        }

        // ════════════════════════════════════════
        // IAxisCard 屬性
        // ════════════════════════════════════════

        public int[] Pos { get { RefreshState(); return (int[])_pos.Clone(); } }
        public int[] Speed { get { RefreshState(); return (int[])_speed.Clone(); } }
        public MotorState[] State { get { RefreshState(); return (MotorState[])_state.Clone(); } }
        public CardState AxisCardState { get { RefreshState(); return _cardState; } }
        public int[] QueueLength { get { RefreshState(); return (int[])_queueLength.Clone(); } }

        /// <summary>從 CommThread 取得最新的軸狀態快照並更新本地快取。</summary>
        private void RefreshState()
        {
            _comm.GetState(_pos, _speed, _state, _queueLength);
            _cardState = _comm.ProcessCardStateChange();
        }

        // ════════════════════════════════════════
        // 連線管理
        // ════════════════════════════════════════

        public bool Start()
        {
            if (_cardState != CardState.NULL)
            {
                _log.Warn($"Start() 拒絕：目前狀態為 {_cardState}，需為 NULL");
                return false;
            }

            _log.Info("開始建立 EtherCAT 連線...");
            var result = _comm.StartConnection();

            if (result)
            {
                _heartbeatTimer.Change(HEARTBEAT_INTERVAL_MS, HEARTBEAT_INTERVAL_MS);
                _log.Info("連線指令已送出，狀態切換為 CONNING，心跳計時器已啟動");
            }
            else
            {
                _log.Error("連線指令送出失敗");
            }

            RefreshState();
            return result;
        }

        public bool Initial()
        {
            RefreshState();
            if (_cardState == CardState.CONNING)
            {
                int waitedMs = 0;
                while (_cardState == CardState.CONNING && waitedMs < CONNECT_WAIT_TIMEOUT_MS)
                {
                    Thread.Sleep(CONNECT_WAIT_POLL_MS);
                    waitedMs += CONNECT_WAIT_POLL_MS;
                    RefreshState();
                }
            }

            if (_cardState != CardState.CONNCET)
            {
                _log.Warn($"Initial() 拒絕：目前狀態為 {_cardState}，需為 CONNCET");
                return false;
            }

            _log.Info("開始初始化軸...");
            var result = _comm.RequestInitial();
            RefreshState();

            if (result)
                _log.Info("初始化成功，AxisCardState → READY");
            else
                _log.Error("初始化失敗");

            return result;
        }

        public bool End()
        {
            RefreshState();
            for (int i = 0; i < AXIS_COUNT; i++)
            {
                if (_state[i] == MotorState.MOVING)
                {
                    _log.Warn($"End() 拒絕：軸 {i} 仍在運動中");
                    return false;
                }
            }

            _heartbeatTimer.Change(Timeout.Infinite, Timeout.Infinite);
            _comm.RequestEnd();
            Thread.Sleep(500);
            RefreshState();
            _log.Info("驅動已關閉");
            return true;
        }

        // ════════════════════════════════════════
        // 安全控制
        // ════════════════════════════════════════

        public bool Estop()
        {
            _log.Warn("主線程請求緊急停止");
            _comm.RequestEstop();

            if (!_comm.IsCommAlive)
                _log.Fatal("通訊線程已斷連，等待硬體看門狗介入");

            RefreshState();
            return true;
        }

        public bool Ralm()
        {
            RefreshState();
            if (_cardState != CardState.ALARM)
            {
                bool hasAlarm = false;
                for (int i = 0; i < AXIS_COUNT; i++)
                    if (_state[i] == MotorState.ALARM) hasAlarm = true;

                if (!hasAlarm)
                {
                    _log.Warn("Ralm() 拒絕：無警報需要清除");
                    return false;
                }
            }

            _comm.RequestRalm();
            Thread.Sleep(200);
            RefreshState();
            _log.Info($"警報復歸完成，AxisCardState = {_cardState}");
            return true;
        }

        public bool CalibrateZero() => _comm.RequestCalibrateZero();

        // ════════════════════════════════════════
        // 運動控制
        // ════════════════════════════════════════

        public bool Stop(ushort axis, double tDec)
        {
            if (!ValidateMotionPrecondition(axis)) return false;
            if (tDec <= 0) { _log.Warn("Stop() 拒絕：tDec 必須 > 0"); return false; }

            // 無視該軸隊列，立即執行：變速至0 → 等待 → sd_stop
            _comm.RequestImmediateStop(axis, tDec);
            return true;
        }

        public bool ChangeVelocity(ushort axis, int newSpeed, double tSec)
        {
            if (!ValidateMotionPrecondition(axis)) return false;
            if (tSec <= 0) { _log.Warn("ChangeVelocity() 拒絕：tSec 必須 > 0"); return false; }

            return _comm.EnqueueCommand(new MotionCommand
            {
                Type = CommandType.VelocityChange,
                Axis = axis,
                NewTargetSpd = newSpeed,
                TSec = tSec,
            });
        }

        public bool MoveAbsolute(ushort axis, int dist, int strVel, int constVel,
                                  int endVel, double tAcc, double tDec)
        {
            if (!ValidateMotionPrecondition(axis)) return false;

            return _comm.EnqueueCommand(new MotionCommand
            {
                Type = CommandType.MoveAbsolute,
                Axis = axis,
                Dist = dist, StrVel = strVel, ConstVel = constVel,
                EndVel = endVel, TAcc = tAcc, TDec = tDec,
            });
        }

        public bool MoveRelative(ushort axis, int dist, int strVel, int constVel,
                                  int endVel, double tAcc, double tDec)
        {
            if (!ValidateMotionPrecondition(axis)) return false;

            return _comm.EnqueueCommand(new MotionCommand
            {
                Type = CommandType.MoveRelative,
                Axis = axis,
                Dist = dist, StrVel = strVel, ConstVel = constVel,
                EndVel = endVel, TAcc = tAcc, TDec = tDec,
            });
        }

        public bool MovePV(ushort axis, int strVel, int constVel, double tAcc)
        {
            if (!ValidateMotionPrecondition(axis)) return false;

            return _comm.EnqueueCommand(new MotionCommand
            {
                Type = CommandType.MovePV,
                Axis = axis,
                StrVel = strVel, ConstVel = constVel, TAcc = tAcc,
            });
        }

        public bool MovePT(ushort axis, int dataCnt, int[] targetPos,
                           int[] targetTime, int strVel, int endVel)
        {
            if (!ValidateMotionPrecondition(axis)) return false;
            if (targetPos == null || targetTime == null ||
                targetPos.Length < dataCnt || targetTime.Length < dataCnt)
            {
                _log.Warn("MovePT() 拒絕：資料陣列長度不足");
                return false;
            }

            return _comm.EnqueueCommand(new MotionCommand
            {
                Type = CommandType.MovePT,
                Axis = axis,
                DataCount = dataCnt,
                TargetPos = (int[])targetPos.Clone(),
                TargetTime = (int[])targetTime.Clone(),
                StrVel = strVel, EndVel = endVel,
            });
        }

        public bool MoveMultiAxisPVT(int[] dataCount, int[][] targetPos,
                                      int[][] targetTime, int[] strVel, int[] endVel)
        {
            RefreshState();
            if (_cardState != CardState.READY)
            {
                _log.Warn($"MoveMultiAxisPVT() 拒絕：狀態 {_cardState} ≠ READY");
                return false;
            }
            if (dataCount?.Length < AXIS_COUNT || targetPos?.Length < AXIS_COUNT ||
                targetTime?.Length < AXIS_COUNT || strVel?.Length < AXIS_COUNT ||
                endVel?.Length < AXIS_COUNT)
            {
                _log.Warn("MoveMultiAxisPVT() 拒絕：參數不完整");
                return false;
            }

            var cmd = new MotionCommand
            {
                Type = CommandType.MultiAxisPVT,
                MultiDataCount = (int[])dataCount!.Clone(),
                MultiTargetPos = new int[AXIS_COUNT][],
                MultiTargetTime = new int[AXIS_COUNT][],
                MultiStrVel = (int[])strVel!.Clone(),
                MultiEndVel = (int[])endVel!.Clone(),
            };
            for (int i = 0; i < AXIS_COUNT; i++)
            {
                cmd.MultiTargetPos[i] = (int[])targetPos![i].Clone();
                cmd.MultiTargetTime[i] = (int[])targetTime![i].Clone();
            }

            return _comm.EnqueueCommand(cmd);
        }

        public bool AbortAndChangePosition(int[] targetMdeg, double tDec)
        {
            RefreshState();
            if (_cardState != CardState.READY)
            {
                _log.Warn($"AbortAndChangePosition() 拒絕：狀態 {_cardState} ≠ READY");
                return false;
            }
            if (targetMdeg == null || targetMdeg.Length < AXIS_COUNT)
            {
                _log.Warn("AbortAndChangePosition() 拒絕：targetMdeg 長度不足");
                return false;
            }
            if (tDec <= 0)
            {
                _log.Warn("AbortAndChangePosition() 拒絕：tDec 必須 > 0");
                return false;
            }

            _comm.RequestAbortAndChangePosition(targetMdeg, tDec);
            return true;
        }

        public bool MoveHome(int constVel, double tAcc, double tDec)
        {
            RefreshState();
            if (_cardState != CardState.READY)
            {
                _log.Warn($"MoveHome() 拒絕：狀態 {_cardState} ≠ READY");
                return false;
            }
            for (int i = 0; i < AXIS_COUNT; i++)
            {
                if (_state[i] != MotorState.STOP)
                {
                    _log.Warn($"MoveHome() 拒絕：軸 {i} 狀態為 {_state[i]}，需全部 STOP");
                    return false;
                }
            }

            _log.Info($"回原點：V={constVel}, Tacc={tAcc}, Tdec={tDec}");
            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                _comm.EnqueueCommand(new MotionCommand
                {
                    Type = CommandType.MoveAbsolute,
                    Axis = i,
                    Dist = 0, StrVel = 0, ConstVel = constVel,
                    EndVel = 0, TAcc = tAcc, TDec = tDec,
                });
            }
            return true;
        }

        // ════════════════════════════════════════
        // 內部
        // ════════════════════════════════════════

        /// <summary>驗證運動前置條件：軸卡需在 READY 狀態且軸號合法。</summary>
        private bool ValidateMotionPrecondition(ushort axis)
        {
            RefreshState();
            if (_cardState != CardState.READY)
            {
                _log.Warn($"運動指令拒絕：狀態 {_cardState} ≠ READY");
                return false;
            }
            if (axis >= AXIS_COUNT)
            {
                _log.Warn($"運動指令拒絕：軸號 {axis} 超出範圍");
                return false;
            }
            return true;
        }

        /// <summary>心跳計時器回呼：更新主線程心跳並檢查通訊線程存活狀態。</summary>
        private void HeartbeatCallback(object? state)
        {
            if (_disposed) return;
            _comm.MainHeartbeat();

            if (!_comm.IsCommAlive && _cardState >= CardState.CONNCET)
                _log.Fatal("偵測到通訊線程斷連！等待硬體看門狗介入");

            RefreshState();
        }

        /// <summary>釋放心跳計時器與通訊線程資源。</summary>
        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            _heartbeatTimer.Change(Timeout.Infinite, Timeout.Infinite);
            _heartbeatTimer.Dispose();
            _comm.Dispose();
            GC.SuppressFinalize(this);
        }
    }
}
