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

        // RA605 各軸軟體限位（mdeg）— 進入 CSP 持續模式時，每軸選擇距離當前位置較遠的一端作為目標
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

            _continuousRunning = false;
            _continuousThread?.Join(TimeSpan.FromSeconds(2));
            _continuousThread = null;

            // CSP 模式停止：以當前編碼器位置為目標，減速時間 0.3s
            AxisCard.AbortAndChangePosition(AxisCard.Pos, 0.3);

            // 持續移動停止後同步命令位置
            SyncCommandedPosition();
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

            // 每軸前一拍已命令速度（mdeg/s），用於加速度限幅
            var prevVelMdegPerSec = new int[AXIS_COUNT];

            // 進入 CSP 持續模式：每軸選擇距當前位置較遠的軟體限位作為目標
            // （strVel=0, constVel=1, endVel=1 → 軸以 1 mdeg/s 緩速向限位移動，進入 CSP 等待狀態）
            for (ushort i = 0; i < AXIS_COUNT; i++)
            {
                int distToPos = Math.Abs(CSP_JOINT_LIMIT_POS[i] - initPos[i]);
                int distToNeg = Math.Abs(initPos[i] - CSP_JOINT_LIMIT_NEG[i]);
                int limitTarget = distToPos >= distToNeg ? CSP_JOINT_LIMIT_POS[i] : CSP_JOINT_LIMIT_NEG[i];
                AxisCard.MoveAbsolute(i, limitTarget, 0, 1, 1, 0.1, 0.1);
            }
            _log.Info("CSP 持續模式已進入（各軸正在向較遠軟體限位緩速移動）");

            var sw = System.Diagnostics.Stopwatch.StartNew();

            try
            {
                while (_continuousRunning && AxisCard.AxisCardState == CardState.READY)
                {
                    // ── 1. 讀取目標速度向量 ──────────────────────────────────────
                    float dx, dy, dz, dYaw, dPitch, dRoll;
                    lock (_continuousLock)
                    {
                        dx = _cdx; dy = _cdy; dz = _cdz;
                        dYaw = _cdYaw; dPitch = _cdPitch; dRoll = _cdRoll;
                    }

                    // ── 2. 增量全零：穩定停機 ────────────────────────────────────
                    if (dx == 0 && dy == 0 && dz == 0 &&
                        dYaw == 0 && dPitch == 0 && dRoll == 0)
                    {
                        AxisCard.AbortAndChangePosition(AxisCard.Pos, 0.3);
                        Thread.Sleep(400); // 等待減速完成
                        break;
                    }

                    // ── 3. 累積虛擬末端姿態 ──────────────────────────────────────
                    float newX = virtualPosture.M41 + dx * CSP_LOOP_PERIOD_SEC;
                    float newY = virtualPosture.M42 + dy * CSP_LOOP_PERIOD_SEC;
                    float newZ = virtualPosture.M43 + dz * CSP_LOOP_PERIOD_SEC;

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

                    // ── 4. IK 計算目標關節角 ─────────────────────────────────────
                    var currentMdeg = AxisCard.Pos;
                    var targetMdeg = _kin.InverseMdeg(virtualPosture, currentMdeg, out var ikDiag);
                    if (targetMdeg == null)
                    {
                        _log.Warn($"CSP 持續移動：IK 無解，穩定停機 | {ikDiag ?? "無額外診斷"}");
                        AxisCard.AbortAndChangePosition(AxisCard.Pos, 0.3);
                        Thread.Sleep(400);
                        break;
                    }
                    if (!string.IsNullOrEmpty(ikDiag))
                        _log.Warn($"CSP 持續移動：{ikDiag}");

                    // ── 5. 計算目標速度並套用加速度限幅，發送 ChangeVelocity ─────
                    for (ushort i = 0; i < AXIS_COUNT; i++)
                    {
                        int desiredVel = (int)((targetMdeg[i] - currentMdeg[i]) / CSP_LOOP_PERIOD_SEC);
                        int velChange = desiredVel - prevVelMdegPerSec[i];
                        int commandedVel = Math.Abs(velChange) > CSP_MAX_VEL_STEP_MDEG
                            ? prevVelMdegPerSec[i] + Math.Sign(velChange) * CSP_MAX_VEL_STEP_MDEG
                            : desiredVel;

                        AxisCard.ChangeVelocity(i, commandedVel, CSP_LOOP_PERIOD_SEC);
                        prevVelMdegPerSec[i] = commandedVel;
                    }

                    // ── 6. 補償式等待至下一個 100ms 週期 ────────────────────────
                    long elapsed = sw.ElapsedMilliseconds % CSP_LOOP_INTERVAL_MS;
                    int sleepMs = (int)(CSP_LOOP_INTERVAL_MS - elapsed);
                    if (sleepMs > 0) Thread.Sleep(sleepMs);
                    sw.Restart();
                }
            }
            catch (Exception ex)
            {
                _log.Error("CSP 持續移動控制迴圈異常", ex);
                try { AxisCard.AbortAndChangePosition(AxisCard.Pos, 0.3); } catch { }
            }

            _log.Info("CSP 持續移動控制迴圈結束");
        }

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
