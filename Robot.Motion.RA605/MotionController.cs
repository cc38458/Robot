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
        private const int CONTINUOUS_CYCLE_MS = 50;  // 持續移動控制迴圈週期
        private const int PVT_POINTS_PER_BATCH = 10; // 每批 PVT 點數
        private const int MAX_JOINT_JUMP_MDEG = 30_000; // 單步最大關節跳動（30°）

        // 持續移動加速度上限
        private const float MAX_LINEAR_ACCEL  = 150f;     // mm/s²（線速度加速度）
        private const float MAX_ANGULAR_ACCEL = 90_000f;  // mdeg/s²（= 90 deg/s²，角速度加速度）
        // 每批次發送週期（睡眠時間），用於計算每批速度允許變化量
        private const float BATCH_PERIOD_SEC  = CONTINUOUS_CYCLE_MS * PVT_POINTS_PER_BATCH / 2 / 1000f;

        private readonly RA605Kinematics _kin;
        private readonly RobotLogger _log;

        // ── 持續移動控制 ──
        private Thread? _continuousThread;
        private volatile bool _continuousRunning;
        private readonly object _continuousLock = new();
        private float _cdx, _cdy, _cdz, _cdYaw, _cdPitch, _cdRoll; // 目標速度向量（由外部設定）

        // 追蹤當前已命令的 EE 速度（已套用加速度限制），僅由 ContinuousMoveLoop 存取
        private float _curVelX, _curVelY, _curVelZ;
        private float _curVelYaw, _curVelPitch, _curVelRoll;

        // 前一批次 PVT 的末速度（mdeg/s），作為下批次的 strVel，確保批次銜接速度連續
        private readonly int[] _prevBatchEndVel = new int[AXIS_COUNT];

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

        public MotionController(IAxisCard axisCard, RobotLogger logger, float toolLength = 0f)
        {
            AxisCard = axisCard;
            _log = logger;
            _kin = new RA605Kinematics(toolLength);
        }

        // ════════════════════════════════════════
        // 末端絕對姿態移動
        // ════════════════════════════════════════

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

            return AxisCard.MoveMultiAxisPVT(dataCount, targetPos, targetTime, strVel, endVel);
        }

        // ════════════════════════════════════════
        // 末端持續相對移動
        // ════════════════════════════════════════

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

        public bool StopContinuousMove()
        {
            if (!_continuousRunning) return true;

            _continuousRunning = false;
            _continuousThread?.Join(TimeSpan.FromSeconds(2));
            _continuousThread = null;

            // 所有軸減速停止
            for (ushort i = 0; i < AXIS_COUNT; i++)
                AxisCard.Stop(i, 0.3);

            _log.Info("持續相對移動已停止");
            return true;
        }

        /// <summary>
        /// 持續移動控制迴圈
        /// 每個週期：讀取目前位姿 → 用已加速度限制的 EE 速度產生 PVT 點 → IK → 送 PVT
        ///
        /// 速度連續性：以 _prevBatchEndVel 作為本批 strVel，確保批次間關節速度連續。
        /// 加速度限制：_curVelX/Y/Z/Yaw/Pitch/Roll 以 MAX_LINEAR/ANGULAR_ACCEL 逐批限速，
        ///             批次內軌跡點依 prevVel→curVel 線性速度斜坡積分產生，
        ///             使第一點位移極小（與 strVel≈0 一致），自然平滑加速。
        /// </summary>
        private void ContinuousMoveLoop()
        {
            _log.Info("持續移動控制迴圈啟動");

            // 每次啟動都從靜止狀態起算
            _curVelX = _curVelY = _curVelZ = 0f;
            _curVelYaw = _curVelPitch = _curVelRoll = 0f;
            Array.Clear(_prevBatchEndVel, 0, AXIS_COUNT);

            const float dtSec = CONTINUOUS_CYCLE_MS / 1000f;
            const int   pointCount = PVT_POINTS_PER_BATCH;
            const float T = pointCount * dtSec; // 整批軌跡總時長 (s)

            try
            {
                while (_continuousRunning && AxisCard.AxisCardState == CardState.READY)
                {
                    // ── 1. 讀取目標速度向量 ─────────────────────────────────────
                    float dx, dy, dz, dYaw, dPitch, dRoll;
                    lock (_continuousLock)
                    {
                        dx = _cdx; dy = _cdy; dz = _cdz;
                        dYaw = _cdYaw; dPitch = _cdPitch; dRoll = _cdRoll;
                    }

                    // ── 2. 加速度限制：逐步將 _curVel* 推向目標速度 ─────────────
                    float linStep = MAX_LINEAR_ACCEL  * BATCH_PERIOD_SEC;
                    float angStep = MAX_ANGULAR_ACCEL * BATCH_PERIOD_SEC;

                    // 記錄本批次起始速度（斜坡積分用）
                    float prevVelX     = _curVelX;     float prevVelY     = _curVelY;
                    float prevVelZ     = _curVelZ;     float prevVelYaw   = _curVelYaw;
                    float prevVelPitch = _curVelPitch; float prevVelRoll  = _curVelRoll;

                    _curVelX     = Ramp(_curVelX,     dx,     linStep);
                    _curVelY     = Ramp(_curVelY,     dy,     linStep);
                    _curVelZ     = Ramp(_curVelZ,     dz,     linStep);
                    _curVelYaw   = Ramp(_curVelYaw,   dYaw,   angStep);
                    _curVelPitch = Ramp(_curVelPitch, dPitch, angStep);
                    _curVelRoll  = Ramp(_curVelRoll,  dRoll,  angStep);

                    // 速度完全歸零則等待（不送空指令）
                    if (_curVelX == 0 && _curVelY == 0 && _curVelZ == 0 &&
                        _curVelYaw == 0 && _curVelPitch == 0 && _curVelRoll == 0)
                    {
                        // 確保下次重新啟動時 strVel = 0（機器人已靜止）
                        Array.Clear(_prevBatchEndVel, 0, AXIS_COUNT);
                        Thread.Sleep(CONTINUOUS_CYCLE_MS);
                        continue;
                    }

                    // ── 3. 讀取目前末端姿態 ──────────────────────────────────────
                    var currentPos = AxisCard.Pos; // mdeg
                    var currentPosture = _kin.ForwardMdeg(currentPos);
                    var curXYZ = RA605Kinematics.ExtractPosition(currentPosture);

                    // ── 4. 產生 PVT 軌跡點 ───────────────────────────────────────
                    // 位置依速度斜坡積分：pos(t) = prevVel*t + (curVel-prevVel)*t²/(2T)
                    // 斜坡積分使批次內速度從 prevVel 線性增加至 curVel，
                    // 確保 strVel（來自上批末速）與第一點之位移一致，避免超加速。
                    int[] dataCount = new int[AXIS_COUNT];
                    int[][] targetPos = new int[AXIS_COUNT][];
                    int[][] targetTime = new int[AXIS_COUNT][];
                    int[] strVel = (int[])_prevBatchEndVel.Clone(); // 前批末速 → 本批起始速
                    int[] endVel = new int[AXIS_COUNT];

                    for (int a = 0; a < AXIS_COUNT; a++)
                    {
                        targetPos[a]  = new int[pointCount];
                        targetTime[a] = new int[pointCount];
                        dataCount[a]  = pointCount;
                    }

                    bool truncated = false;
                    int[] prevMdeg = (int[])currentPos.Clone();

                    for (int p = 0; p < pointCount; p++)
                    {
                        float t = (p + 1) * dtSec;
                        float tRamp = t * t / (2f * T); // = t²/(2T)，斜坡積分係數

                        // 末端空間位移（速度斜坡積分）
                        float newX = curXYZ[0] + prevVelX * t + (_curVelX - prevVelX) * tRamp;
                        float newY = curXYZ[1] + prevVelY * t + (_curVelY - prevVelY) * tRamp;
                        float newZ = curXYZ[2] + prevVelZ * t + (_curVelZ - prevVelZ) * tRamp;

                        // 姿態角位移（mdeg→deg，速度斜坡積分）
                        float yawDeg   = (prevVelYaw   * t + (_curVelYaw   - prevVelYaw)   * tRamp) / 1000f;
                        float pitchDeg = (prevVelPitch * t + (_curVelPitch - prevVelPitch) * tRamp) / 1000f;
                        float rollDeg  = (prevVelRoll  * t + (_curVelRoll  - prevVelRoll)  * tRamp) / 1000f;

                        var rotInc = Matrix4x4.CreateRotationZ(yawDeg   * MathF.PI / 180f)
                                   * Matrix4x4.CreateRotationY(pitchDeg * MathF.PI / 180f)
                                   * Matrix4x4.CreateRotationX(rollDeg  * MathF.PI / 180f);

                        var curRot = new Matrix4x4(
                            currentPosture.M11, currentPosture.M12, currentPosture.M13, 0,
                            currentPosture.M21, currentPosture.M22, currentPosture.M23, 0,
                            currentPosture.M31, currentPosture.M32, currentPosture.M33, 0,
                            0, 0, 0, 1);
                        var newRot = rotInc * curRot;

                        var newPosture = currentPosture;
                        newPosture.M11 = newRot.M11; newPosture.M12 = newRot.M12; newPosture.M13 = newRot.M13;
                        newPosture.M21 = newRot.M21; newPosture.M22 = newRot.M22; newPosture.M23 = newRot.M23;
                        newPosture.M31 = newRot.M31; newPosture.M32 = newRot.M32; newPosture.M33 = newRot.M33;
                        newPosture.M41 = newX; newPosture.M42 = newY; newPosture.M43 = newZ;

                        var ptMdeg = _kin.InverseMdeg(newPosture, prevMdeg, out var ikDiagnostic);
                        if (ptMdeg == null)
                        {
                            _log.Warn($"持續移動：第 {p} 點 IK 無解，截斷 | {ikDiagnostic ?? "無額外診斷"}");
                            truncated = true;
                            for (int a = 0; a < AXIS_COUNT; a++) dataCount[a] = p;
                            break;
                        }
                        if (!string.IsNullOrEmpty(ikDiagnostic))
                            _log.Warn($"持續移動：第 {p} 點 {ikDiagnostic}");

                        bool jumpExceeded = false;
                        for (int a = 0; a < AXIS_COUNT; a++)
                        {
                            if (Math.Abs(ptMdeg[a] - prevMdeg[a]) > MAX_JOINT_JUMP_MDEG)
                            {
                                _log.Warn($"持續移動：第 {p} 點軸{a} 跳動過大（{Math.Abs(ptMdeg[a] - prevMdeg[a]) / 1000f:F1}°），截斷");
                                jumpExceeded = true;
                                break;
                            }
                        }
                        if (jumpExceeded)
                        {
                            truncated = true;
                            for (int a = 0; a < AXIS_COUNT; a++) dataCount[a] = p;
                            break;
                        }

                        for (int a = 0; a < AXIS_COUNT; a++)
                        {
                            targetPos[a][p]  = ptMdeg[a];
                            targetTime[a][p] = (p + 1) * CONTINUOUS_CYCLE_MS;
                        }
                        prevMdeg = ptMdeg;
                    }

                    // ── 5. 計算末速度並送 PVT ────────────────────────────────────
                    if (dataCount[0] > 0)
                    {
                        for (int a = 0; a < AXIS_COUNT; a++)
                        {
                            int cnt = dataCount[a];
                            // 末速度由最後兩點差分估算（mdeg/s）
                            endVel[a] = cnt >= 2
                                ? (int)((targetPos[a][cnt - 1] - targetPos[a][cnt - 2]) / dtSec)
                                : 0;
                        }

                        // 截斷情況：強制末速歸零（確保機器人在截止點停下），並重置速度狀態
                        if (truncated)
                        {
                            Array.Clear(endVel, 0, AXIS_COUNT);
                            _curVelX = _curVelY = _curVelZ = 0f;
                            _curVelYaw = _curVelPitch = _curVelRoll = 0f;
                        }

                        AxisCard.MoveMultiAxisPVT(dataCount, targetPos, targetTime, strVel, endVel);

                        // 保存末速度供下批次作為 strVel
                        endVel.CopyTo(_prevBatchEndVel, 0);
                    }
                    else
                    {
                        // 無有效點（IK 立即失敗）：重置速度狀態
                        Array.Clear(_prevBatchEndVel, 0, AXIS_COUNT);
                        _curVelX = _curVelY = _curVelZ = 0f;
                        _curVelYaw = _curVelPitch = _curVelRoll = 0f;
                    }

                    // 等待接近本批執行完畢再送下一批
                    Thread.Sleep(CONTINUOUS_CYCLE_MS * PVT_POINTS_PER_BATCH / 2);
                }
            }
            catch (Exception ex)
            {
                _log.Error("持續移動控制迴圈異常", ex);
            }

            // 清理速度狀態
            Array.Clear(_prevBatchEndVel, 0, AXIS_COUNT);
            _curVelX = _curVelY = _curVelZ = 0f;
            _curVelYaw = _curVelPitch = _curVelRoll = 0f;

            _log.Info("持續移動控制迴圈結束");
        }

        /// <summary>以最大步長 maxStep 將 current 逼近 target（不超越）</summary>
        private static float Ramp(float current, float target, float maxStep)
            => current < target
                ? MathF.Min(target, current + maxStep)
                : MathF.Max(target, current - maxStep);

        // ════════════════════════════════════════
        // 末端一次性相對移動
        // ════════════════════════════════════════

        public bool MoveRelativeEndEffector(float dx, float dy, float dz,
                                             float dYaw, float dPitch, float dRoll,
                                             int maxSpeed)
        {
            if (AxisCard.AxisCardState != CardState.READY)
            {
                _log.Warn("MoveRelativeEndEffector 拒絕：軸卡狀態非 READY");
                return false;
            }

            // 1. 目前姿態
            var currentPos = AxisCard.Pos;
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

            // 4. 計算移動時間（由 maxSpeed 和最大軸位移決定）
            int maxDelta = 0;
            for (int i = 0; i < AXIS_COUNT; i++)
            {
                int delta = Math.Abs(targetMdeg[i] - currentPos[i]);
                if (delta > maxDelta) maxDelta = delta;
            }

            // 三角形速度曲線（strVel=0, endVel=0）峰值 = 2 * avgVel，
            // 故需 moveTime = 2 * maxDelta / maxSpeed 才能確保峰值 ≤ maxSpeed
            int moveTimeMs = maxSpeed > 0 ? Math.Max(300, (int)(maxDelta * 2000.0 / maxSpeed)) : 1000;

            _log.Info($"MoveRelativeEndEffector：Δ=[{dx},{dy},{dz}]mm, Δω=[{dYaw},{dPitch},{dRoll}]mdeg, 最大速度={maxSpeed}, 時間={moveTimeMs}ms");

            // 5. 送多軸 PVT
            int[] dataCount = new int[AXIS_COUNT];
            int[][] targetPosArr = new int[AXIS_COUNT][];
            int[][] targetTimeArr = new int[AXIS_COUNT][];
            int[] strVel = new int[AXIS_COUNT];
            int[] endVel = new int[AXIS_COUNT];

            for (int i = 0; i < AXIS_COUNT; i++)
            {
                dataCount[i] = 1;
                targetPosArr[i] = new[] { targetMdeg[i] };
                targetTimeArr[i] = new[] { moveTimeMs };
                strVel[i] = 0;
                endVel[i] = 0;
            }

            return AxisCard.MoveMultiAxisPVT(dataCount, targetPosArr, targetTimeArr, strVel, endVel);
        }

        // ════════════════════════════════════════
        // 軸空間運動
        // ════════════════════════════════════════

        public bool MoveAxisAbsolute(ushort axis, int angleMdeg, int constVel,
                                      double tAcc, double tDec)
        {
            _log.Info($"MoveAxisAbsolute：軸{axis} → {angleMdeg / 1000f:F2}°, V={constVel}");
            return AxisCard.MoveAbsolute(axis, angleMdeg, 0, constVel, 0, tAcc, tDec);
        }

        public bool MoveAxisRelative(ushort axis, int deltaAngleMdeg, int constVel,
                                      double tAcc, double tDec)
        {
            _log.Info($"MoveAxisRelative：軸{axis} Δ{deltaAngleMdeg / 1000f:F2}°, V={constVel}");
            return AxisCard.MoveRelative(axis, deltaAngleMdeg, 0, constVel, 0, tAcc, tDec);
        }

        public bool MoveHome(int constVel, double tAcc, double tDec)
        {
            _log.Info("MoveHome：所有軸回原點");
            return AxisCard.MoveHome(constVel, tAcc, tDec);
        }

        // ════════════════════════════════════════

        public void Dispose()
        {
            StopContinuousMove();
            GC.SuppressFinalize(this);
        }
    }
}
