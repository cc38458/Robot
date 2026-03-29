using System.Globalization;
using System.Numerics;
using Robot.Core.Enums;
using Robot.Core.Interfaces;
using Robot.Core.Logging;

namespace Robot.Motion.RA605
{
    /// <summary>
    /// RA605 高階運動控制器 — IMotionController 實作
    /// 負責正逆運動學轉換、末端運動規劃、持續相對移動控制迴圈
    /// </summary>
    public class MotionController : IMotionController
    {
        private const int AXIS_COUNT = 6;
        private const int CSP_LOOP_INTERVAL_MS = 100;           // 持續移動控制迴圈週期
        private const float CSP_LOOP_PERIOD_SEC = 0.1f;         // 與上方保持一致（秒）
        private const int CSP_MAX_VEL_STEP_MDEG = 2000;         // 每 100ms 允許的最大速度變化量（mdeg/s）
        private const float CSP_JOINT_SERVO_KP = 3.0f;          // 外層 joint servo 比例增益（1/s）
        private const int CSP_JOINT_DEADBAND_MDEG = 80;         // 小於此誤差視為已到位，避免抖動
        private const int CSP_MAX_CMD_VEL_MDEG = 18_000;        // 外層允許的最大關節命令速度（mdeg/s）
        private const int CSP_REVERSE_DECEL_STEP_MDEG = 800;    // 未真正反向時，每拍最多減速量（mdeg/s）
        private const int CSP_TRACKING_SLOWDOWN_START_MDEG = 2_000;
        private const int CSP_TRACKING_SLOWDOWN_FULL_MDEG = 8_000;
        private const int IK_UNSOLVED_LOG_INTERVAL = 10;        // IK 無解時每 N 拍記錄一次，避免洗版
        private const int CSP_DEBUG_SNAPSHOT_INTERVAL = 5;      // 每 N 拍輸出一次 CSP 診斷快照
        private const int CSP_FINALIZE_STOP_TDEC_MS = 200;
        private const int CSP_FINALIZE_MOVE_TACC_MS = 100;
        private const int CSP_FINALIZE_MOVE_TDEC_MS = 200;
        private const int CSP_FINALIZE_WAIT_TIMEOUT_MS = 3000;
        private const int CSP_UNEXPECTED_STOP_RESEND_INTERVAL_LOOPS = 3;
        private const int CSP_UNEXPECTED_STOP_RESEND_MIN_ERR_MDEG = 300;
        private const int CSP_UNEXPECTED_STOP_RESEND_MIN_CMD_VEL_MDEG = 300;
        private const int CSP_STALL_DETECT_MIN_EXPECTED_MOVE_MDEG = 120;
        private const int CSP_STALL_DETECT_MIN_ACTUAL_MOVE_MDEG = 40;
        private const float CSP_STALL_DETECT_MOVE_RATIO = 0.2f;
        private const int CSP_STALL_DETECT_CONSECUTIVE_LOOPS = 3;
        private const int CSP_J2_EXPERIMENTAL_MIN_CMD_VEL_MDEG = 1_000;

        // RA605 各軸軟體限位（mdeg）— 進入 CSP 持續模式時，依目標關節位置方向選擇目標端
        // 對應 RA605Kinematics.cs 中 JOINT_MIN/JOINT_MAX（單位：度 × 1000）
        private static readonly int[] CSP_JOINT_LIMIT_POS = {  165_000,  85_000, 185_000,  190_000,  115_000,  360_000 };
        private static readonly int[] CSP_JOINT_LIMIT_NEG = { -165_000, -125_000, -55_000, -190_000, -115_000, -360_000 };

        private readonly RA605Kinematics _kin;
        private readonly RobotLogger _log;

        // ── 持續移動控制 ──
        private Thread? _continuousThread;
        private volatile bool _continuousRunning;
        private readonly object _continuousLock = new();
        private float _cdx, _cdy, _cdz, _cdYaw, _cdPitch, _cdRoll; // 目標速度向量（由外部設定）
        private bool _continuousStopRequested;
        private int[]? _continuousStopTargetMdeg;
        private Matrix4x4 _continuousVirtualPosture;
        private bool _continuousVirtualPostureValid;
        private int[]? _currentTargetJointAngles;
        private int[] _currentTargetJointSpeedMdegPerSec = new int[AXIS_COUNT];
        private int[] _currentCommandedJointSpeedMdegPerSec = new int[AXIS_COUNT];
        private int[] _currentExpectedLimitTargets = new int[AXIS_COUNT];
        private int[] _currentActiveLimitTargets = new int[AXIS_COUNT];
        private int[]? _continuousIkReferenceMdeg;
        private float _lastContinuousTrackingScale = 1f;
        private float _lastContinuousSingularScale = 1f;
        private float _lastContinuousCartesianSlowdownScale = 1f;
        private readonly float[] _lastContinuousAppliedLinearVelocity = new float[3];

        // ── 命令位置追蹤 ──
        // 記錄最後一次運動指令的目標角度（mdeg），用於末端相對移動的基準計算
        // null 表示尚未初始化，需從編碼器讀取
        private int[]? _lastCommandedAngles;
        // 命令位置是否可靠：PV 持續運動中或急停後為 false（目標位置未知）
        private volatile bool _positionReliable = true;
        private readonly object _cmdPosLock = new();

        // ── PV 軸追蹤 ──
        // 記錄哪些軸正處於 PV（等速持續）模式
        private readonly bool[] _pvActive = new bool[AXIS_COUNT];

        /// <summary>底層軸卡驅動介面。</summary>
        public IAxisCard AxisCard { get; }

        /// <summary>
        /// 目前末端位置 [X,Y,Z]（mm），由正運動學即時計算
        /// </summary>
        public float[] EndEffectorPosition
        {
            get
            {
                var mat = EndEffectorPosture;
                return RA605Kinematics.ExtractPosition(mat);
            }
        }

        /// <summary>
        /// 目前末端姿態齊次矩陣
        /// </summary>
        public Matrix4x4 EndEffectorPosture
        {
            get
            {
                var pos = AxisCard.Pos; // mdeg
                return _kin.ForwardMdeg(pos);
            }
        }

        /// <summary>
        /// 目前目標姿態對應的六軸角度（mdeg）。
        /// 持續移動時回傳虛擬末端姿態的 IK 結果；否則退回目前編碼器角度。
        /// </summary>
        public int[] TargetJointAngles
        {
            get
            {
                lock (_continuousLock)
                {
                    return _currentTargetJointAngles != null
                        ? (int[])_currentTargetJointAngles.Clone()
                        : (int[])AxisCard.Pos.Clone();
                }
            }
        }

        /// <summary>
        /// 最近一拍 continuous loop 的目標關節速度（mdeg/s）。
        /// 若目前未處於持續移動，回傳零向量。
        /// </summary>
        public int[] TargetJointSpeedMdegPerSec
        {
            get
            {
                lock (_continuousLock)
                {
                    return (int[])_currentTargetJointSpeedMdegPerSec.Clone();
                }
            }
        }

        /// <summary>
        /// 最近一拍 continuous loop 真正送出的關節命令速度（mdeg/s）。
        /// 若目前未處於持續移動，回傳零向量。
        /// </summary>
        public int[] CommandedJointSpeedMdegPerSec
        {
            get
            {
                lock (_continuousLock)
                {
                    return (int[])_currentCommandedJointSpeedMdegPerSec.Clone();
                }
            }
        }

        public int[] ExpectedLimitTargetsMdeg
        {
            get
            {
                lock (_continuousLock)
                {
                    return (int[])_currentExpectedLimitTargets.Clone();
                }
            }
        }

        public int[] ActiveLimitTargetsMdeg
        {
            get
            {
                lock (_continuousLock)
                {
                    return (int[])_currentActiveLimitTargets.Clone();
                }
            }
        }

        /// <summary>
        /// 持續移動控制中的虛擬末端位置 [X, Y, Z]（mm）。
        /// 若目前沒有有效虛擬姿態，退回目前末端位置。
        /// </summary>
        public float[] VirtualEndEffectorPosition
        {
            get
            {
                lock (_continuousLock)
                {
                    return _continuousVirtualPostureValid
                        ? RA605Kinematics.ExtractPosition(_continuousVirtualPosture)
                        : EndEffectorPosition;
                }
            }
        }

        /// <summary>最近一拍 continuous loop 使用的 tracking slowdown scale。</summary>
        public float ContinuousTrackingScale
        {
            get
            {
                lock (_continuousLock)
                {
                    return _lastContinuousTrackingScale;
                }
            }
        }

        /// <summary>最近一拍 continuous loop 使用的 singular slowdown scale。</summary>
        public float ContinuousSingularScale
        {
            get
            {
                lock (_continuousLock)
                {
                    return _lastContinuousSingularScale;
                }
            }
        }

        /// <summary>最近一拍 continuous loop 用於累積虛擬末端姿態的 cartesian slowdown scale。</summary>
        public float ContinuousCartesianSlowdownScale
        {
            get
            {
                lock (_continuousLock)
                {
                    return _lastContinuousCartesianSlowdownScale;
                }
            }
        }

        /// <summary>最近一拍 continuous loop 實際套用到虛擬末端設定點的平移速度 [X, Y, Z]（mm/s）。</summary>
        public float[] ContinuousAppliedLinearVelocity
        {
            get
            {
                lock (_continuousLock)
                {
                    return (float[])_lastContinuousAppliedLinearVelocity.Clone();
                }
            }
        }

        /// <summary>
        /// 建立高階運動控制器。
        /// </summary>
        /// <param name="axisCard">底層軸卡驅動介面。</param>
        /// <param name="logger">日誌記錄器。</param>
        /// <param name="toolLength">工具頭長度（mm）。</param>
        public MotionController(IAxisCard axisCard, RobotLogger logger, float toolLength = 0f)
        {
            AxisCard = axisCard;
            _log = logger;
            _kin = new RA605Kinematics(toolLength);
        }

        // ════════════════════════════════════════
        // 命令位置管理
        // ════════════════════════════════════════

        /// <summary>
        /// 將命令位置同步為編碼器實際位置。
        /// 適用時機：急停後、PV 停止後、手臂靜止時校正漂移。
        /// </summary>
        private void SyncCommandedPosition()
        {
            var actual = AxisCard.Pos;
            lock (_cmdPosLock)
            {
                _lastCommandedAngles = (int[])actual.Clone();
                _positionReliable = true;
            }
            _log.Info($"命令位置已同步至編碼器：[{string.Join(", ", actual.Select(a => $"{a / 1000f:F2}°"))}]");
        }

        /// <summary>
        /// 標記命令位置不可靠（PV 啟動、急停等場景）。
        /// </summary>
        private void MarkPositionUnreliable()
        {
            _positionReliable = false;
            _log.Warn("命令位置標記為不可靠（需同步後才能安全執行末端相對移動）");
        }

        /// <summary>更新命令位置為指定目標角度（運動指令成功送出後呼叫）。</summary>
        private void UpdateCommandedAngles(int[] targetMdeg)
        {
            lock (_cmdPosLock)
            {
                _lastCommandedAngles = (int[])targetMdeg.Clone();
                _positionReliable = true;
            }
        }

        /// <summary>更新命令位置的單一軸（單軸運動指令成功送出後呼叫）。</summary>
        private void UpdateCommandedAxis(ushort axis, int targetMdeg)
        {
            lock (_cmdPosLock)
            {
                // 若尚未初始化，先從編碼器讀取所有軸
                _lastCommandedAngles ??= (int[])AxisCard.Pos.Clone();
                _lastCommandedAngles[axis] = targetMdeg;
                // 僅更新單軸時，若其他軸正在 PV 仍不可靠
            }
        }

        /// <summary>取得命令位置作為相對移動基準。若不可靠則自動同步並發出警告。</summary>
        private int[] GetCommandedBasePosition()
        {
            lock (_cmdPosLock)
            {
                if (_lastCommandedAngles == null)
                {
                    // 首次使用：從編碼器初始化
                    _lastCommandedAngles = (int[])AxisCard.Pos.Clone();
                    _positionReliable = true;
                    _log.Info("命令位置首次初始化（從編碼器讀取）");
                    return (int[])_lastCommandedAngles.Clone();
                }

                if (!_positionReliable)
                {
                    // 位置不可靠（PV中、急停後等），強制同步
                    _lastCommandedAngles = (int[])AxisCard.Pos.Clone();
                    _positionReliable = true;
                    _log.Warn("命令位置不可靠，已自動同步至編碼器（末端相對移動精度可能受影響）");
                    return (int[])_lastCommandedAngles.Clone();
                }

                return (int[])_lastCommandedAngles.Clone();
            }
        }

        // ════════════════════════════════════════
        // 末端絕對姿態移動
        // ════════════════════════════════════════

        /// <inheritdoc />
        public bool MoveToPosture(Matrix4x4 targetPosture, int moveTimeMs)
        {
            if (AxisCard.AxisCardState != CardState.READY)
            {
                _log.Warn("MoveToPosture 拒絕：軸卡狀態非 READY");
                return false;
            }

            // 1. 取目前角度（作為 IK 參考，選最小轉動解）
            var currentMdeg = AxisCard.Pos;

            // 2. IK 計算目標角度
            var targetMdeg = _kin.InverseMdeg(targetPosture, currentMdeg, out var ikDiagnostic);
            if (targetMdeg == null)
            {
                _log.Error($"MoveToPosture 拒絕：IK 無解（目標超出工作空間） | {ikDiagnostic ?? "無額外診斷"}");
                return false;
            }
            if (!string.IsNullOrEmpty(ikDiagnostic))
                _log.Warn($"MoveToPosture：{ikDiagnostic}");

            // 3. 建構 PVT 資料（簡單兩點：起點→終點）
            int[] dataCount = new int[AXIS_COUNT];
            int[][] targetPos = new int[AXIS_COUNT][];
            int[][] targetTime = new int[AXIS_COUNT][];
            int[] strVel = new int[AXIS_COUNT];
            int[] endVel = new int[AXIS_COUNT];

            for (int i = 0; i < AXIS_COUNT; i++)
            {
                dataCount[i] = 1;
                targetPos[i] = new[] { targetMdeg[i] };
                targetTime[i] = new[] { moveTimeMs };
                strVel[i] = 0;
                endVel[i] = 0;
            }

            _log.Info($"MoveToPosture：目標角度 [{string.Join(", ", targetMdeg.Select(a => $"{a / 1000f:F2}°"))}]，時間 {moveTimeMs}ms");

            var result = AxisCard.MoveMultiAxisPVT(dataCount, targetPos, targetTime, strVel, endVel);
            if (result) UpdateCommandedAngles(targetMdeg);
            return result;
        }

        // ════════════════════════════════════════
        // 末端持續相對移動
        // ════════════════════════════════════════

        /// <inheritdoc />
        public bool StartContinuousMove(float deltaX, float deltaY, float deltaZ,
                                         float deltaYaw, float deltaPitch, float deltaRoll)
        {
            if (AxisCard.AxisCardState != CardState.READY)
            {
                _log.Warn("StartContinuousMove 拒絕：軸卡狀態非 READY");
                return false;
            }

            lock (_continuousLock)
            {
                _cdx = deltaX; _cdy = deltaY; _cdz = deltaZ;
                _cdYaw = deltaYaw; _cdPitch = deltaPitch; _cdRoll = deltaRoll;
            }

            if (!_continuousRunning)
            {
                _continuousRunning = true;
                _continuousThread = new Thread(ContinuousMoveLoop)
                {
                    Name = "ContinuousMove",
                    IsBackground = true,
                };
                _continuousThread.Start();
                _log.Info($"持續相對移動啟動：V=[{deltaX},{deltaY},{deltaZ}] mm/s, ω=[{deltaYaw},{deltaPitch},{deltaRoll}] mdeg/s");
            }
            else
            {
                _log.Info($"持續相對移動速度更新：V=[{deltaX},{deltaY},{deltaZ}], ω=[{deltaYaw},{deltaPitch},{deltaRoll}]");
            }

            return true;
        }

        /// <inheritdoc />
        public bool UpdateContinuousMove(float deltaX, float deltaY, float deltaZ,
                                          float deltaYaw, float deltaPitch, float deltaRoll)
        {
            if (!_continuousRunning)
            {
                // 如果全為零，當作不需要啟動
                if (deltaX == 0 && deltaY == 0 && deltaZ == 0 &&
                    deltaYaw == 0 && deltaPitch == 0 && deltaRoll == 0)
                    return true;

                return StartContinuousMove(deltaX, deltaY, deltaZ, deltaYaw, deltaPitch, deltaRoll);
            }

            // 全零向量等同停止
            if (deltaX == 0 && deltaY == 0 && deltaZ == 0 &&
                deltaYaw == 0 && deltaPitch == 0 && deltaRoll == 0)
                return StopContinuousMove();

            lock (_continuousLock)
            {
                _cdx = deltaX; _cdy = deltaY; _cdz = deltaZ;
                _cdYaw = deltaYaw; _cdPitch = deltaPitch; _cdRoll = deltaRoll;
            }
            return true;
        }

        /// <inheritdoc />
        public bool StopContinuousMove()
        {
            if (!_continuousRunning) return true;

            int[] stopTargetMdeg;
            string stopTargetSource;
            lock (_continuousLock)
            {
                _cdx = 0; _cdy = 0; _cdz = 0;
                _cdYaw = 0; _cdPitch = 0; _cdRoll = 0;

                var currentMdeg = AxisCard.Pos;
                if (_continuousVirtualPostureValid)
                {
                    var ikRef = _continuousIkReferenceMdeg != null
                        ? (int[])_continuousIkReferenceMdeg.Clone()
                        : currentMdeg;
                    stopTargetMdeg = _kin.InverseMdeg(_continuousVirtualPosture, ikRef, out var stopIkDiag)
                        ?? currentMdeg;
                    stopTargetSource = stopTargetMdeg == currentMdeg ? "encoder_fallback" : "virtual_posture";
                    if (stopTargetMdeg == currentMdeg && !string.IsNullOrEmpty(stopIkDiag))
                        _log.Warn($"StopContinuousMove：虛擬末端姿態轉停止角度失敗，退回當前編碼器位置 | {stopIkDiag}");
                }
                else
                {
                    stopTargetMdeg = currentMdeg;
                    stopTargetSource = "encoder";
                }

                _continuousStopRequested = true;
                _continuousStopTargetMdeg = (int[])stopTargetMdeg.Clone();
                _currentTargetJointAngles = (int[])stopTargetMdeg.Clone();
                _continuousIkReferenceMdeg = (int[])stopTargetMdeg.Clone();
            }

            _log.Info($"StopContinuousMove：source={stopTargetSource}, stopTarget=[{FormatMdegArray(stopTargetMdeg)}]");

            _continuousThread?.Join(TimeSpan.FromSeconds(2));
            _continuousThread = null;
            _continuousRunning = false;

            // 持續移動停止後同步命令位置
            UpdateCommandedAngles(stopTargetMdeg);
            _log.Info("CSP 持續相對移動已停止");
            return true;
        }

        /// <summary>
        /// CSP 持續移動控制迴圈
        /// 啟動時對六軸送出指向極限位置的絕對移動指令，進入 CSP 持續模式。
        /// 每 100ms：讀取速度向量 → 累積虛擬末端姿態 → IK → 讀取編碼器 →
        ///           計算目標速度 [(目標mdeg - 當前mdeg) / 0.1s] → 加速度限幅 → 發送 ChangeVelocity。
        /// 增量為零或停止時：呼叫 AbortAndChangePosition 以當前編碼器位置為目標，減速時間 0.3s。
        /// </summary>
        private void ContinuousMoveLoop()
        {
            _log.Info("CSP 持續移動控制迴圈啟動");

            // 初始化虛擬末端姿態（從當前編碼器位置計算 FK）
            var initPos = AxisCard.Pos;
            var virtualPosture = _kin.ForwardMdeg(initPos);
            lock (_continuousLock)
            {
                _continuousVirtualPosture = virtualPosture;
                _continuousVirtualPostureValid = true;
                _continuousStopRequested = false;
                _continuousStopTargetMdeg = null;
                _currentTargetJointAngles = (int[])initPos.Clone();
                _continuousIkReferenceMdeg = (int[])initPos.Clone();
                _currentTargetJointSpeedMdegPerSec = new int[AXIS_COUNT];
                _currentCommandedJointSpeedMdegPerSec = new int[AXIS_COUNT];
                _currentExpectedLimitTargets = new int[AXIS_COUNT];
                _currentActiveLimitTargets = new int[AXIS_COUNT];
                _lastContinuousTrackingScale = 1f;
                _lastContinuousSingularScale = 1f;
                _lastContinuousCartesianSlowdownScale = 1f;
                _lastContinuousAppliedLinearVelocity[0] = 0f;
                _lastContinuousAppliedLinearVelocity[1] = 0f;
                _lastContinuousAppliedLinearVelocity[2] = 0f;
            }

            // 每軸前一拍已命令速度（mdeg/s），用於加速度限幅
            var prevVelMdegPerSec = new int[AXIS_COUNT];
            var prevTargetMdeg = new int[AXIS_COUNT];
            var prevActualMdeg = (int[])initPos.Clone();
            var hasPrevTargetMdeg = new bool[AXIS_COUNT];
            var activeLimitTargets = new int[AXIS_COUNT];
            var desiredVelMdegPerSec = new int[AXIS_COUNT];
            var lastUnexpectedStopResendLoop = new int[AXIS_COUNT];
            var stalledLoopCounts = new int[AXIS_COUNT];
            int ikUnsolvedCount = 0;
            int loopCount = 0;

            // 進入 CSP 持續模式：先預看 0.1s 後的末端目標，依各軸目標方向選擇正/負限位。
            // （strVel=0, constVel=1, endVel=1 → 軸以 1 mdeg/s 緩速向指定端點移動，進入 CSP 等待狀態）
            float startDx, startDy, startDz, startYaw, startPitch, startRoll;
            lock (_continuousLock)
            {
                startDx = _cdx; startDy = _cdy; startDz = _cdz;
                startYaw = _cdYaw; startPitch = _cdPitch; startRoll = _cdRoll;
            }
            var initialPreviewPosture = AdvanceVirtualPosture(virtualPosture, startDx, startDy, startDz, startYaw, startPitch, startRoll);
            var initialTargetMdeg = _kin.InverseMdeg(initialPreviewPosture, initPos, out var initialIkDiag);

            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                int limitTarget = initialTargetMdeg != null
                    ? SelectCspLimitTarget(i, initPos[i], initialTargetMdeg[i])
                    : SelectCspLimitTarget(i, initPos[i], initPos[i]);

                activeLimitTargets[i] = limitTarget;
                AxisCard.MoveAbsolute(i, limitTarget, 0, 1, 1, 0.1, 0.1);
            }
            lock (_continuousLock)
            {
                _currentExpectedLimitTargets = (int[])activeLimitTargets.Clone();
                _currentActiveLimitTargets = (int[])activeLimitTargets.Clone();
            }
            if (initialTargetMdeg == null && !string.IsNullOrEmpty(initialIkDiag))
                _log.Warn($"CSP 持續模式進入時初始預看 IK 無解，先以當前方向預設限位啟動 | {initialIkDiag}");
            _log.Info("CSP 持續模式已進入（各軸正朝目標方向對應的限位緩速移動）");

            var sw = System.Diagnostics.Stopwatch.StartNew();

            try
            {
                int[]? finalizeTargetMdeg = null;
                string? finalizeReason = null;
                while (_continuousRunning && AxisCard.AxisCardState == CardState.READY)
                {
                    loopCount++;

                    // ── 1. 讀取目標速度向量 ──────────────────────────────────────
                    float dx, dy, dz, dYaw, dPitch, dRoll;
                    lock (_continuousLock)
                    {
                        dx = _cdx; dy = _cdy; dz = _cdz;
                        dYaw = _cdYaw; dPitch = _cdPitch; dRoll = _cdRoll;
                    }

                    int[]? stopTarget = null;
                    lock (_continuousLock)
                    {
                        if (_continuousStopRequested)
                        {
                            stopTarget = _continuousStopTargetMdeg != null
                                ? (int[])_continuousStopTargetMdeg.Clone()
                                : (int[])AxisCard.Pos.Clone();
                        }
                    }
                    if (stopTarget != null)
                    {
                        _log.Info($"CSP stop request 已進入控制迴圈：target=[{FormatMdegArray(stopTarget)}]");
                        finalizeTargetMdeg = stopTarget;
                        finalizeReason = "external_stop";
                        break;
                    }

                    // ── 2. 增量全零：穩定停機 ────────────────────────────────────
                    if (dx == 0 && dy == 0 && dz == 0 &&
                        dYaw == 0 && dPitch == 0 && dRoll == 0)
                    {
                        var zeroStopCurrentMdeg = AxisCard.Pos;
                        var zeroStopIkRef = _continuousIkReferenceMdeg != null
                            ? (int[])_continuousIkReferenceMdeg.Clone()
                            : zeroStopCurrentMdeg;
                        var zeroStopTarget = _kin.InverseMdeg(virtualPosture, zeroStopIkRef, out var zeroStopIkDiag)
                            ?? zeroStopCurrentMdeg;
                        if (zeroStopTarget == zeroStopCurrentMdeg && !string.IsNullOrEmpty(zeroStopIkDiag))
                            _log.Warn($"CSP 持續移動：零向量停機退回當前編碼器位置 | {zeroStopIkDiag}");

                        _log.Info($"CSP 零向量停機：target=[{FormatMdegArray(zeroStopTarget)}]");
                        lock (_continuousLock)
                        {
                            _currentTargetJointAngles = (int[])zeroStopTarget.Clone();
                            _continuousIkReferenceMdeg = (int[])zeroStopTarget.Clone();
                            _currentTargetJointSpeedMdegPerSec = new int[AXIS_COUNT];
                            _currentCommandedJointSpeedMdegPerSec = new int[AXIS_COUNT];
                        }
                        finalizeTargetMdeg = zeroStopTarget;
                        finalizeReason = "zero_vector";
                        break;
                    }

                    var currentMdeg = AxisCard.Pos;
                    int trackingErrorMaxMdeg = 0;
                    lock (_continuousLock)
                    {
                        if (_currentTargetJointAngles != null)
                        {
                            for (int i = 0; i < AXIS_COUNT; i++)
                            {
                                trackingErrorMaxMdeg = Math.Max(
                                    trackingErrorMaxMdeg,
                                    Math.Abs(_currentTargetJointAngles[i] - currentMdeg[i]));
                            }
                        }
                    }
                    float trackingSlowdown = GetTrackingSlowdownScale(trackingErrorMaxMdeg);
                    bool pureTranslation = dYaw == 0 && dPitch == 0 && dRoll == 0;
                    float rawSingularSlowdown;
                    lock (_continuousLock)
                    {
                        rawSingularSlowdown = _lastContinuousSingularScale;
                    }

                    // ── 3. 累積虛擬末端姿態 ──────────────────────────────────────
                    // 設定點應只由輸入決定，不應被奇異點/追點保護改寫。
                    // tracking / singular slowdown 僅保留在 joint servo 速度保護層。
                    const float cartesianSlowdown = 1f;
                    float appliedDx = dx;
                    float appliedDy = dy;
                    float appliedDz = dz;
                    lock (_continuousLock)
                    {
                        _lastContinuousCartesianSlowdownScale = cartesianSlowdown;
                        _lastContinuousAppliedLinearVelocity[0] = appliedDx;
                        _lastContinuousAppliedLinearVelocity[1] = appliedDy;
                        _lastContinuousAppliedLinearVelocity[2] = appliedDz;
                    }
                    float newX = virtualPosture.M41 + appliedDx * CSP_LOOP_PERIOD_SEC;
                    float newY = virtualPosture.M42 + appliedDy * CSP_LOOP_PERIOD_SEC;
                    float newZ = virtualPosture.M43 + appliedDz * CSP_LOOP_PERIOD_SEC;

                    float yawDeg   = dYaw   * CSP_LOOP_PERIOD_SEC / 1000f;
                    float pitchDeg = dPitch * CSP_LOOP_PERIOD_SEC / 1000f;
                    float rollDeg  = dRoll  * CSP_LOOP_PERIOD_SEC / 1000f;

                    var rotInc = Matrix4x4.CreateRotationZ(yawDeg   * MathF.PI / 180f)
                               * Matrix4x4.CreateRotationY(pitchDeg * MathF.PI / 180f)
                               * Matrix4x4.CreateRotationX(rollDeg  * MathF.PI / 180f);

                    var curRot = new Matrix4x4(
                        virtualPosture.M11, virtualPosture.M12, virtualPosture.M13, 0,
                        virtualPosture.M21, virtualPosture.M22, virtualPosture.M23, 0,
                        virtualPosture.M31, virtualPosture.M32, virtualPosture.M33, 0,
                        0, 0, 0, 1);
                    var newRot = rotInc * curRot;

                    virtualPosture.M11 = newRot.M11; virtualPosture.M12 = newRot.M12; virtualPosture.M13 = newRot.M13;
                    virtualPosture.M21 = newRot.M21; virtualPosture.M22 = newRot.M22; virtualPosture.M23 = newRot.M23;
                    virtualPosture.M31 = newRot.M31; virtualPosture.M32 = newRot.M32; virtualPosture.M33 = newRot.M33;
                    virtualPosture.M41 = newX; virtualPosture.M42 = newY; virtualPosture.M43 = newZ;
                    lock (_continuousLock)
                    {
                        _continuousVirtualPosture = virtualPosture;
                        _continuousVirtualPostureValid = true;
                    }

                    // ── 4. IK 計算目標關節角 ─────────────────────────────────────
                    int[] ikRefMdeg;
                    lock (_continuousLock)
                    {
                        ikRefMdeg = _continuousIkReferenceMdeg != null
                            ? (int[])_continuousIkReferenceMdeg.Clone()
                            : (int[])currentMdeg.Clone();
                    }
                    var targetMdeg = _kin.InverseMdeg(virtualPosture, ikRefMdeg, out var ikDiag);
                    if (targetMdeg == null)
                    {
                        ikUnsolvedCount++;
                        if (ikUnsolvedCount == 1 || ikUnsolvedCount % IK_UNSOLVED_LOG_INTERVAL == 0)
                            _log.Warn($"CSP 持續移動：IK 無解，保留虛擬目標並等待重新進入可解區 | {ikDiag ?? "無額外診斷"}");

                        // IK 暫時無解時不退出持續移動；將關節速度平順收斂到 0，
                        // 同時保留累積中的虛擬末端姿態，待重新回到可解區後再追上。
                        for (ushort i = 0; i < AXIS_COUNT; i++)
                        {
                            int velChange = -prevVelMdegPerSec[i];
                            int commandedVel = Math.Abs(velChange) > CSP_MAX_VEL_STEP_MDEG
                                ? prevVelMdegPerSec[i] + Math.Sign(velChange) * CSP_MAX_VEL_STEP_MDEG
                                : 0;

                            desiredVelMdegPerSec[i] = 0;
                            AxisCard.ChangeVelocity(i, Math.Abs(commandedVel), CSP_LOOP_PERIOD_SEC);
                            prevVelMdegPerSec[i] = commandedVel;
                        }

                        if (loopCount % CSP_DEBUG_SNAPSHOT_INTERVAL == 0)
                        {
                            _log.Debug($"CSP snapshot[unsolved] loop={loopCount} virtualXYZ={FormatCartesian(virtualPosture)} inputV=[{FormatVelocityVector(dx, dy, dz, dYaw, dPitch, dRoll)}] appliedXYZ=[{appliedDx:F3},{appliedDy:F3},{appliedDz:F3} mm/s] trackingScale={trackingSlowdown:F2} prevSingularScale={rawSingularSlowdown:F2} actual=[{FormatMdegArray(currentMdeg)}] cmdVel=[{FormatMdegArray(prevVelMdegPerSec)}]");
                        }

                        long unsolvedElapsed = sw.ElapsedMilliseconds % CSP_LOOP_INTERVAL_MS;
                        int unsolvedSleepMs = (int)(CSP_LOOP_INTERVAL_MS - unsolvedElapsed);
                        if (unsolvedSleepMs > 0) Thread.Sleep(unsolvedSleepMs);
                        sw.Restart();
                        continue;
                    }

                    if (ikUnsolvedCount > 0)
                    {
                        _log.Info($"CSP 持續移動：IK 已恢復可解，累積無解拍數={ikUnsolvedCount}");
                        ikUnsolvedCount = 0;
                    }
                    if (!string.IsNullOrEmpty(ikDiag))
                        _log.Warn($"CSP 持續移動：{ikDiag}");

                    lock (_continuousLock)
                    {
                        _currentTargetJointAngles = (int[])targetMdeg.Clone();
                        _continuousIkReferenceMdeg = (int[])targetMdeg.Clone();
                    }

                    // ── 5. 計算目標速度並套用加速度限幅，發送 ChangeVelocity ─────
                    float singularSlowdown = GetSingularitySlowdownScale(ikDiag);
                    lock (_continuousLock)
                    {
                        _lastContinuousTrackingScale = trackingSlowdown;
                        _lastContinuousSingularScale = singularSlowdown;
                    }
                    for (ushort i = 0; i < AXIS_COUNT; i++)
                    {
                        float axisServoScale = GetAxisServoScale(i, trackingSlowdown, singularSlowdown, ikDiag, pureTranslation);
                        int servoMaxVelMdeg = Math.Max(3_000, (int)MathF.Round(CSP_MAX_CMD_VEL_MDEG * axisServoScale));
                        int servoVelStepMdeg = Math.Max(400, (int)MathF.Round(CSP_MAX_VEL_STEP_MDEG * axisServoScale));
                        int jointError = targetMdeg[i] - currentMdeg[i];
                        int targetTravel = hasPrevTargetMdeg[i] ? targetMdeg[i] - prevTargetMdeg[i] : 0;
                        int desiredVel = ComputeServoVelocity(jointError, axisServoScale, servoMaxVelMdeg);
                        bool targetReversed = hasPrevTargetMdeg[i]
                            && Math.Sign(targetTravel) != 0
                            && Math.Sign(targetTravel) != Math.Sign(prevVelMdegPerSec[i]);
                        bool wouldReverseNow = prevVelMdegPerSec[i] != 0 &&
                            desiredVel != 0 &&
                            Math.Sign(desiredVel) != Math.Sign(prevVelMdegPerSec[i]);

                        int expectedLimitTarget = SelectCspLimitTarget(i, currentMdeg[i], targetMdeg[i]);
                        _currentExpectedLimitTargets[i] = expectedLimitTarget;
                        if (activeLimitTargets[i] != expectedLimitTarget && (!wouldReverseNow || targetReversed))
                        {
                            AxisCard.ChangeTargetPosition(i, expectedLimitTarget);
                            activeLimitTargets[i] = expectedLimitTarget;
                        }

                        if (wouldReverseNow && !targetReversed)
                        {
                            // 目標點尚未反向，只是目前位置/速度造成誤差跨過零；先平滑減速，不立即反打，也不切換限位端。
                            desiredVel = DecelerateWithoutReversing(prevVelMdegPerSec[i], axisServoScale);
                            _log.Debug($"CSP 軸{i + 1} 抑制立即反向：actual={currentMdeg[i]}, target={targetMdeg[i]}, prevVel={prevVelMdegPerSec[i]}, targetTravel={targetTravel}");
                        }

                        int velChange = desiredVel - prevVelMdegPerSec[i];
                        int commandedVel = Math.Abs(velChange) > servoVelStepMdeg
                            ? prevVelMdegPerSec[i] + Math.Sign(velChange) * servoVelStepMdeg
                            : desiredVel;
                        commandedVel = Math.Clamp(commandedVel, -servoMaxVelMdeg, servoMaxVelMdeg);

                        if (i == 1 &&
                            commandedVel != 0 &&
                            Math.Abs(jointError) >= CSP_UNEXPECTED_STOP_RESEND_MIN_ERR_MDEG &&
                            Math.Abs(commandedVel) < CSP_J2_EXPERIMENTAL_MIN_CMD_VEL_MDEG)
                        {
                            commandedVel = Math.Sign(commandedVel) * CSP_J2_EXPERIMENTAL_MIN_CMD_VEL_MDEG;
                        }

                        int expectedMoveMdeg = (int)Math.Round(Math.Abs(commandedVel) * CSP_LOOP_PERIOD_SEC);
                        int actualMoveMdeg = Math.Abs(currentMdeg[i] - prevActualMdeg[i]);
                        int actualMoveThresholdMdeg = Math.Max(
                            CSP_STALL_DETECT_MIN_ACTUAL_MOVE_MDEG,
                            (int)Math.Round(expectedMoveMdeg * CSP_STALL_DETECT_MOVE_RATIO));
                        bool stalledByPosition =
                            Math.Abs(jointError) >= CSP_UNEXPECTED_STOP_RESEND_MIN_ERR_MDEG &&
                            Math.Abs(commandedVel) >= CSP_UNEXPECTED_STOP_RESEND_MIN_CMD_VEL_MDEG &&
                            expectedMoveMdeg >= CSP_STALL_DETECT_MIN_EXPECTED_MOVE_MDEG &&
                            actualMoveMdeg <= actualMoveThresholdMdeg;
                        stalledLoopCounts[i] = stalledByPosition ? stalledLoopCounts[i] + 1 : 0;

                        bool unexpectedStop =
                            AxisCard.State[i] == MotorState.STOP &&
                            Math.Abs(jointError) >= CSP_UNEXPECTED_STOP_RESEND_MIN_ERR_MDEG &&
                            Math.Abs(commandedVel) >= CSP_UNEXPECTED_STOP_RESEND_MIN_CMD_VEL_MDEG;

                        bool shouldResendAbsMove =
                            (unexpectedStop ||
                             stalledLoopCounts[i] >= CSP_STALL_DETECT_CONSECUTIVE_LOOPS) &&
                            loopCount - lastUnexpectedStopResendLoop[i] >= CSP_UNEXPECTED_STOP_RESEND_INTERVAL_LOOPS;

                        if (shouldResendAbsMove)
                        {
                            bool resent = AxisCard.MoveAbsolute(
                                i,
                                expectedLimitTarget,
                                0,
                                1,
                                1,
                                0.1,
                                0.1);
                            if (resent)
                            {
                                lastUnexpectedStopResendLoop[i] = loopCount;
                                stalledLoopCounts[i] = 0;
                                activeLimitTargets[i] = expectedLimitTarget;
                                _log.Warn(
                                    $"CSP 軸{i + 1} 偵測到 AbsMove 意外結束或位置停滯，依目標側重送 MoveAbsolute：limit={expectedLimitTarget / 1000f:F3}°, " +
                                    $"actual={currentMdeg[i] / 1000f:F3}°, target={targetMdeg[i] / 1000f:F3}°, cmdVel={commandedVel / 1000f:F3}°/s, expectedMove={expectedMoveMdeg / 1000f:F3}°, actualMove={actualMoveMdeg / 1000f:F3}°, stalledLoops={stalledLoopCounts[i]}");
                            }
                            else
                            {
                                _log.Warn($"CSP 軸{i + 1} 意外 STOP 後重送 MoveAbsolute 失敗");
                            }
                        }

                        desiredVelMdegPerSec[i] = desiredVel;
                        // 方向由當前追逐的限位端決定，VelocityChange 只送速度大小。
                        AxisCard.ChangeVelocity(i, Math.Abs(commandedVel), CSP_LOOP_PERIOD_SEC);
                        prevVelMdegPerSec[i] = commandedVel;
                        prevTargetMdeg[i] = targetMdeg[i];
                        hasPrevTargetMdeg[i] = true;
                        prevActualMdeg[i] = currentMdeg[i];
                    }
                    lock (_continuousLock)
                    {
                        _currentTargetJointSpeedMdegPerSec = (int[])desiredVelMdegPerSec.Clone();
                        _currentCommandedJointSpeedMdegPerSec = (int[])prevVelMdegPerSec.Clone();
                        _currentActiveLimitTargets = (int[])activeLimitTargets.Clone();
                    }

                    if (loopCount % CSP_DEBUG_SNAPSHOT_INTERVAL == 0)
                    {
                        _log.Debug(
                            $"CSP snapshot loop={loopCount} virtualXYZ={FormatCartesian(virtualPosture)} inputV=[{FormatVelocityVector(dx, dy, dz, dYaw, dPitch, dRoll)}] appliedXYZ=[{appliedDx:F3},{appliedDy:F3},{appliedDz:F3} mm/s] trackingScale={trackingSlowdown:F2} cartesianScale={cartesianSlowdown:F2} prevSingularScale={rawSingularSlowdown:F2} singularScale={singularSlowdown:F2} pureTranslation={pureTranslation} " +
                            $"actual=[{FormatMdegArray(currentMdeg)}] target=[{FormatMdegArray(targetMdeg)}] desiredVel=[{FormatMdegArray(desiredVelMdegPerSec)}] " +
                            $"cmdVel=[{FormatMdegArray(prevVelMdegPerSec)}] limit=[{FormatMdegArray(activeLimitTargets)}]");
                    }

                    // ── 6. 補償式等待至下一個 100ms 週期 ────────────────────────
                    long elapsed = sw.ElapsedMilliseconds % CSP_LOOP_INTERVAL_MS;
                    int sleepMs = (int)(CSP_LOOP_INTERVAL_MS - elapsed);
                    if (sleepMs > 0) Thread.Sleep(sleepMs);
                    sw.Restart();
                }

                if (finalizeTargetMdeg != null && AxisCard.AxisCardState == CardState.READY)
                {
                    FinalizeContinuousMoveToTarget(finalizeTargetMdeg, finalizeReason ?? "stop");
                }
            }
            catch (Exception ex)
            {
                _log.Error("CSP 持續移動控制迴圈異常", ex);
                try { AxisCard.AbortAndChangePosition(AxisCard.Pos, 0.3); } catch { }
            }
            finally
            {
                lock (_continuousLock)
                {
                    _continuousStopRequested = false;
                    _continuousStopTargetMdeg = null;
                    _continuousVirtualPostureValid = false;
                    _currentTargetJointAngles = null;
                    _currentTargetJointSpeedMdegPerSec = new int[AXIS_COUNT];
                    _currentCommandedJointSpeedMdegPerSec = new int[AXIS_COUNT];
                    _currentExpectedLimitTargets = new int[AXIS_COUNT];
                    _currentActiveLimitTargets = new int[AXIS_COUNT];
                    _continuousIkReferenceMdeg = null;
                    _lastContinuousTrackingScale = 1f;
                    _lastContinuousSingularScale = 1f;
                    _lastContinuousCartesianSlowdownScale = 1f;
                    _lastContinuousAppliedLinearVelocity[0] = 0f;
                    _lastContinuousAppliedLinearVelocity[1] = 0f;
                    _lastContinuousAppliedLinearVelocity[2] = 0f;
                }
            }

            _log.Info("CSP 持續移動控制迴圈結束");
        }

        private void FinalizeContinuousMoveToTarget(int[] targetMdeg, string reason)
        {
            _log.Info($"CSP 結束收尾：reason={reason}, target=[{FormatMdegArray(targetMdeg)}]");

            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                AxisCard.Stop(i, CSP_FINALIZE_STOP_TDEC_MS / 1000.0);
            }

            Thread.Sleep(CSP_FINALIZE_STOP_TDEC_MS + 80);

            var currentMdeg = AxisCard.Pos;
            bool anyMoveIssued = false;
            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                int deltaMdeg = targetMdeg[i] - currentMdeg[i];
                if (Math.Abs(deltaMdeg) <= CSP_JOINT_DEADBAND_MDEG)
                    continue;

                int constVel = Math.Clamp(Math.Abs(deltaMdeg) * 4, 3_000, CSP_MAX_CMD_VEL_MDEG);
                bool ok = AxisCard.MoveAbsolute(
                    i,
                    targetMdeg[i],
                    0,
                    constVel,
                    0,
                    CSP_FINALIZE_MOVE_TACC_MS / 1000.0,
                    CSP_FINALIZE_MOVE_TDEC_MS / 1000.0);

                if (!ok)
                {
                    _log.Warn($"CSP 結束收尾：軸{i + 1} MoveAbsolute 失敗，退回 AbortAndChangePosition");
                    AxisCard.AbortAndChangePosition(targetMdeg, CSP_FINALIZE_MOVE_TDEC_MS / 1000.0);
                    Thread.Sleep(CSP_FINALIZE_MOVE_TDEC_MS + 100);
                    return;
                }

                anyMoveIssued = true;
            }

            lock (_continuousLock)
            {
                _currentTargetJointAngles = (int[])targetMdeg.Clone();
                _continuousIkReferenceMdeg = (int[])targetMdeg.Clone();
                _currentTargetJointSpeedMdegPerSec = new int[AXIS_COUNT];
            }

            if (!anyMoveIssued)
                return;

            var waitSw = System.Diagnostics.Stopwatch.StartNew();
            while (waitSw.ElapsedMilliseconds < CSP_FINALIZE_WAIT_TIMEOUT_MS)
            {
                if (AxisCard.State.All(state => state == MotorState.STOP))
                    return;

                Thread.Sleep(50);
            }

            _log.Warn("CSP 結束收尾：等待 MoveAbsolute 完成逾時");
        }

        private static int SelectCspLimitTarget(int axis, int currentMdeg, int targetMdeg)
            => targetMdeg >= currentMdeg ? CSP_JOINT_LIMIT_POS[axis] : CSP_JOINT_LIMIT_NEG[axis];

        private static Matrix4x4 AdvanceVirtualPosture(Matrix4x4 posture,
            float dx, float dy, float dz, float dYaw, float dPitch, float dRoll)
        {
            var nextPosture = posture;

            float newX = posture.M41 + dx * CSP_LOOP_PERIOD_SEC;
            float newY = posture.M42 + dy * CSP_LOOP_PERIOD_SEC;
            float newZ = posture.M43 + dz * CSP_LOOP_PERIOD_SEC;

            float yawDeg = dYaw * CSP_LOOP_PERIOD_SEC / 1000f;
            float pitchDeg = dPitch * CSP_LOOP_PERIOD_SEC / 1000f;
            float rollDeg = dRoll * CSP_LOOP_PERIOD_SEC / 1000f;

            var rotInc = Matrix4x4.CreateRotationZ(yawDeg * MathF.PI / 180f)
                       * Matrix4x4.CreateRotationY(pitchDeg * MathF.PI / 180f)
                       * Matrix4x4.CreateRotationX(rollDeg * MathF.PI / 180f);

            var curRot = new Matrix4x4(
                posture.M11, posture.M12, posture.M13, 0,
                posture.M21, posture.M22, posture.M23, 0,
                posture.M31, posture.M32, posture.M33, 0,
                0, 0, 0, 1);
            var newRot = rotInc * curRot;

            nextPosture.M11 = newRot.M11; nextPosture.M12 = newRot.M12; nextPosture.M13 = newRot.M13;
            nextPosture.M21 = newRot.M21; nextPosture.M22 = newRot.M22; nextPosture.M23 = newRot.M23;
            nextPosture.M31 = newRot.M31; nextPosture.M32 = newRot.M32; nextPosture.M33 = newRot.M33;
            nextPosture.M41 = newX; nextPosture.M42 = newY; nextPosture.M43 = newZ;

            return nextPosture;
        }

        private static int ComputeServoVelocity(int jointErrorMdeg, float servoScale, int servoMaxVelMdeg)
        {
            if (Math.Abs(jointErrorMdeg) <= CSP_JOINT_DEADBAND_MDEG)
                return 0;

            int desiredVel = (int)MathF.Round(jointErrorMdeg * CSP_JOINT_SERVO_KP * servoScale);
            return Math.Clamp(desiredVel, -servoMaxVelMdeg, servoMaxVelMdeg);
        }

        private static int DecelerateWithoutReversing(int prevVelMdegPerSec, float servoScale)
        {
            if (prevVelMdegPerSec == 0)
                return 0;

            int decelStep = Math.Max(200, (int)MathF.Round(CSP_REVERSE_DECEL_STEP_MDEG * servoScale));
            return Math.Abs(prevVelMdegPerSec) <= decelStep
                ? 0
                : prevVelMdegPerSec - Math.Sign(prevVelMdegPerSec) * decelStep;
        }

        private static float GetTrackingSlowdownScale(int trackingErrorMaxMdeg)
        {
            if (trackingErrorMaxMdeg <= CSP_TRACKING_SLOWDOWN_START_MDEG)
                return 1f;
            if (trackingErrorMaxMdeg >= CSP_TRACKING_SLOWDOWN_FULL_MDEG)
                return 0.25f;

            float ratio = (trackingErrorMaxMdeg - CSP_TRACKING_SLOWDOWN_START_MDEG)
                / (float)(CSP_TRACKING_SLOWDOWN_FULL_MDEG - CSP_TRACKING_SLOWDOWN_START_MDEG);
            return 1f - ratio * 0.75f;
        }

        private static float GetSingularitySlowdownScale(string? ikDiag)
        {
            if (string.IsNullOrEmpty(ikDiag))
                return 1f;
            bool wristSingular = IsWristSingularityDiagnostic(ikDiag);
            bool armSingular = IsArmSingularityDiagnostic(ikDiag);

            if (wristSingular && TryExtractWristBetaAbsDeg(ikDiag, out float wristBetaAbsDeg))
            {
                if (wristBetaAbsDeg <= 0.5f) return 0.35f;
                if (wristBetaAbsDeg <= 1.0f) return 0.45f;
                if (wristBetaAbsDeg <= 2.0f) return 0.60f;
                if (wristBetaAbsDeg <= 3.0f) return 0.75f;
                return 0.90f;
            }
            if (wristSingular)
                return 0.60f;
            if (armSingular)
                return 0.70f;
            return 1f;
        }

        private static float GetAxisServoScale(int axis, float trackingSlowdown, float singularSlowdown, string? ikDiag, bool pureTranslation)
        {
            float scale = MathF.Min(trackingSlowdown, singularSlowdown);
            bool wristSingular = IsWristSingularityDiagnostic(ikDiag);
            bool armSingular = IsArmSingularityDiagnostic(ikDiag);

            if (!pureTranslation)
                return scale;

            if (axis <= 2)
            {
                if (wristSingular && !armSingular)
                    return MathF.Min(trackingSlowdown, MathF.Max(singularSlowdown, 0.75f));
                return scale;
            }

            if (wristSingular)
                return scale;

            return MathF.Min(trackingSlowdown, MathF.Max(singularSlowdown, 0.80f));
        }

        private static bool IsWristSingularityDiagnostic(string? ikDiag)
            => !string.IsNullOrEmpty(ikDiag)
                && (ikDiag.EndsWith(" wrist", StringComparison.Ordinal)
                    || ikDiag.EndsWith(" arm wrist", StringComparison.Ordinal));

        private static bool IsArmSingularityDiagnostic(string? ikDiag)
            => !string.IsNullOrEmpty(ikDiag)
                && (ikDiag.EndsWith(" arm", StringComparison.Ordinal)
                    || ikDiag.EndsWith(" arm wrist", StringComparison.Ordinal));

        private static bool TryExtractWristBetaAbsDeg(string ikDiag, out float wristBetaAbsDeg)
        {
            wristBetaAbsDeg = 0f;

            const string marker = "wrist=[";
            int start = ikDiag.IndexOf(marker, StringComparison.Ordinal);
            if (start < 0)
                return false;

            start += marker.Length;
            int end = ikDiag.IndexOf(']', start);
            if (end < 0)
                return false;

            string[] parts = ikDiag[start..end].Split(',', StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length < 2)
                return false;

            if (!float.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float betaDeg))
                return false;

            wristBetaAbsDeg = MathF.Abs(betaDeg);
            return true;
        }

        private static string FormatMdegArray(int[] values)
            => string.Join(", ", values.Select(v => (v / 1000f).ToString("F3")));

        private static string FormatVelocityVector(float dx, float dy, float dz, float dYaw, float dPitch, float dRoll)
            => $"{dx:F3},{dy:F3},{dz:F3} mm/s | {dYaw:F3},{dPitch:F3},{dRoll:F3} mdeg/s";

        private static string FormatCartesian(Matrix4x4 posture)
            => $"{posture.M41:F3},{posture.M42:F3},{posture.M43:F3}";

        // ════════════════════════════════════════
        // 末端一次性相對移動
        // ════════════════════════════════════════

        /// <inheritdoc />
        public bool MoveRelativeEndEffector(float dx, float dy, float dz,
                                             float dYaw, float dPitch, float dRoll,
                                             int maxSpeed)
        {
            if (AxisCard.AxisCardState != CardState.READY)
            {
                _log.Warn("MoveRelativeEndEffector 拒絕：軸卡狀態非 READY");
                return false;
            }

            // 檢查是否有軸處於 PV 模式，拒絕執行
            for (int i = 0; i < AXIS_COUNT; i++)
            {
                if (_pvActive[i])
                {
                    _log.Warn($"MoveRelativeEndEffector 拒絕：軸{i} 正處於 PV 模式，請先停止 PV 再執行末端相對移動");
                    return false;
                }
            }

            // 1. 取得命令位置作為基準（避免運動中讀取編碼器導致位置飄移）
            var currentPos = GetCommandedBasePosition();
            var currentPosture = _kin.ForwardMdeg(currentPos);
            var curXYZ = RA605Kinematics.ExtractPosition(currentPosture);

            // 2. 目標姿態
            float newX = curXYZ[0] + dx;
            float newY = curXYZ[1] + dy;
            float newZ = curXYZ[2] + dz;

            // 疊加旋轉增量
            float yawDeg = dYaw / 1000f;   // mdeg → deg
            float pitchDeg = dPitch / 1000f;
            float rollDeg = dRoll / 1000f;

            var rotInc = Matrix4x4.CreateRotationZ(yawDeg * MathF.PI / 180f)
                       * Matrix4x4.CreateRotationY(pitchDeg * MathF.PI / 180f)
                       * Matrix4x4.CreateRotationX(rollDeg * MathF.PI / 180f);

            var curRot = new Matrix4x4(
                currentPosture.M11, currentPosture.M12, currentPosture.M13, 0,
                currentPosture.M21, currentPosture.M22, currentPosture.M23, 0,
                currentPosture.M31, currentPosture.M32, currentPosture.M33, 0,
                0, 0, 0, 1);
            var newRot = rotInc * curRot;

            var targetPosture = currentPosture;
            targetPosture.M11 = newRot.M11; targetPosture.M12 = newRot.M12; targetPosture.M13 = newRot.M13;
            targetPosture.M21 = newRot.M21; targetPosture.M22 = newRot.M22; targetPosture.M23 = newRot.M23;
            targetPosture.M31 = newRot.M31; targetPosture.M32 = newRot.M32; targetPosture.M33 = newRot.M33;
            targetPosture.M41 = newX; targetPosture.M42 = newY; targetPosture.M43 = newZ;

            // 3. IK（以 currentPos 為參考，選最小轉動解）
            var targetMdeg = _kin.InverseMdeg(targetPosture, currentPos, out var ikDiagnostic);
            if (targetMdeg == null)
            {
                _log.Error($"MoveRelativeEndEffector 拒絕：IK 無解 | {ikDiagnostic ?? "無額外診斷"}");
                return false;
            }
            if (!string.IsNullOrEmpty(ikDiagnostic))
                _log.Warn($"MoveRelativeEndEffector：{ikDiagnostic}");

            // 4. 計算各軸位移與速度分配
            int[] deltas = new int[AXIS_COUNT];
            int maxDelta = 0;
            for (int i = 0; i < AXIS_COUNT; i++)
            {
                deltas[i] = Math.Abs(targetMdeg[i] - currentPos[i]);
                if (deltas[i] > maxDelta) maxDelta = deltas[i];
            }

            if (maxDelta == 0)
            {
                _log.Info("MoveRelativeEndEffector：位移為零，無需移動");
                return true;
            }

            // maxSpeed 代表最大位移軸的 constVel，其餘軸按位移比例縮放
            // 梯形曲線同步：各軸 tAcc/tDec 相同，constVel 按比例 → 同時到達
            const double tAcc = 0.3;
            const double tDec = 0.3;

            _log.Info($"MoveRelativeEndEffector：Δ=[{dx},{dy},{dz}]mm, Δω=[{dYaw},{dPitch},{dRoll}]mdeg, maxSpeed={maxSpeed}");

            // 5. 逐軸發送 MoveAbsolute
            bool allOk = true;
            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                if (deltas[i] == 0) continue; // 無位移的軸不送指令

                int constVel = (int)((long)deltas[i] * maxSpeed / maxDelta);
                if (constVel < 1) constVel = 1; // 最小速度保護

                _log.Info($"  軸{i}：目標={targetMdeg[i] / 1000f:F2}°, Δ={deltas[i] / 1000f:F2}°, V={constVel} mdeg/s");

                if (!AxisCard.MoveAbsolute(i, targetMdeg[i], 0, constVel, 0, tAcc, tDec))
                {
                    _log.Warn($"  軸{i} MoveAbsolute 失敗");
                    allOk = false;
                }
            }

            // 更新命令位置（即使部分軸失敗，仍以目標角度為命令位置）
            if (allOk) UpdateCommandedAngles(targetMdeg);

            return allOk;
        }

        // ════════════════════════════════════════
        // 軸空間運動
        // ════════════════════════════════════════

        /// <inheritdoc />
        public bool MoveAxisAbsolute(ushort axis, int angleMdeg, int constVel,
                                      double tAcc, double tDec)
        {
            _log.Info($"MoveAxisAbsolute：軸{axis} → {angleMdeg / 1000f:F2}°, V={constVel}");
            var result = AxisCard.MoveAbsolute(axis, angleMdeg, 0, constVel, 0, tAcc, tDec);
            if (result) UpdateCommandedAxis(axis, angleMdeg);
            return result;
        }

        /// <inheritdoc />
        public bool MoveAxisRelative(ushort axis, int deltaAngleMdeg, int constVel,
                                      double tAcc, double tDec)
        {
            _log.Info($"MoveAxisRelative：軸{axis} Δ{deltaAngleMdeg / 1000f:F2}°, V={constVel}");
            var result = AxisCard.MoveRelative(axis, deltaAngleMdeg, 0, constVel, 0, tAcc, tDec);
            if (result)
            {
                // 相對移動：命令位置 = 當前命令位置 + 相對量
                lock (_cmdPosLock)
                {
                    _lastCommandedAngles ??= (int[])AxisCard.Pos.Clone();
                    _lastCommandedAngles[axis] += deltaAngleMdeg;
                }
            }
            return result;
        }

        /// <inheritdoc />
        public bool MoveHome(int constVel, double tAcc, double tDec)
        {
            _log.Info("MoveHome：所有軸回原點");
            var result = AxisCard.MoveHome(constVel, tAcc, tDec);
            if (result) UpdateCommandedAngles(new int[AXIS_COUNT]); // 原點 = 全零
            return result;
        }

        // ════════════════════════════════════════
        // PV 軸管理（內部）
        // ════════════════════════════════════════

        /// <summary>查詢指定軸是否正處於 PV 模式。</summary>
        internal bool IsPvActive(ushort axis) => axis < AXIS_COUNT && _pvActive[axis];

        /// <summary>
        /// 啟動指定軸的 PV 模式。若該軸已在 PV 則改用 ChangeVelocity 變速。
        /// </summary>
        internal bool StartOrUpdatePV(ushort axis, int strVel, int constVel, double tAcc)
        {
            if (_pvActive[axis])
            {
                // 已在 PV 模式：使用變速指令
                _log.Info($"PV 軸{axis} 已啟動，改用 ChangeVelocity：{constVel} mdeg/s, tAcc={tAcc}s");
                return AxisCard.ChangeVelocity(axis, constVel, tAcc);
            }

            // 首次啟動 PV
            MarkPositionUnreliable();
            _pvActive[axis] = true;
            _log.Info($"PV 啟動：軸{axis}, strVel={strVel}, constVel={constVel}, tAcc={tAcc}");
            return AxisCard.MovePV(axis, strVel, constVel, tAcc);
        }

        /// <summary>
        /// 停止指定軸並清除 PV 狀態，同步命令位置。
        /// </summary>
        internal bool StopAxis(ushort axis, double tDec)
        {
            var result = AxisCard.Stop(axis, tDec);
            if (result)
            {
                _pvActive[axis] = false;
                // 停止後同步命令位置（停止位置由減速決定）
                SyncCommandedPosition();
            }
            return result;
        }

        /// <summary>急停後的內部處理：標記位置不可靠，清除所有 PV 狀態。</summary>
        internal void OnEstop()
        {
            Array.Clear(_pvActive, 0, AXIS_COUNT);
            MarkPositionUnreliable();
        }

        /// <summary>警報解除後的內部處理：同步命令位置。</summary>
        internal void OnAlarmCleared()
        {
            Array.Clear(_pvActive, 0, AXIS_COUNT);
            SyncCommandedPosition();
        }

        // ════════════════════════════════════════

        /// <summary>停止持續移動並釋放資源。</summary>
        public void Dispose()
        {
            StopContinuousMove();
            GC.SuppressFinalize(this);
        }
    }
}
