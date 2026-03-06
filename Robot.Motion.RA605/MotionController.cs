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

        private readonly RA605Kinematics _kin;
        private readonly RobotLogger _log;

        // ── 持續移動控制 ──
        private Thread? _continuousThread;
        private volatile bool _continuousRunning;
        private readonly object _continuousLock = new();
        private float _cdx, _cdy, _cdz, _cdYaw, _cdPitch, _cdRoll; // 目前速度向量

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

        public MotionController(IAxisCard axisCard, RobotLogger logger, float toolLength = 60f)
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
            var targetMdeg = _kin.InverseMdeg(targetPosture, currentMdeg);
            if (targetMdeg == null)
            {
                _log.Error("MoveToPosture 拒絕：IK 無解（目標超出工作空間）");
                return false;
            }

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
        /// 每個週期：讀取目前位姿 → 加上速度*Δt → IK → 送 PVT
        /// </summary>
        private void ContinuousMoveLoop()
        {
            _log.Info("持續移動控制迴圈啟動");

            try
            {
                while (_continuousRunning && AxisCard.AxisCardState == CardState.READY)
                {
                    float dx, dy, dz, dYaw, dPitch, dRoll;
                    lock (_continuousLock)
                    {
                        dx = _cdx; dy = _cdy; dz = _cdz;
                        dYaw = _cdYaw; dPitch = _cdPitch; dRoll = _cdRoll;
                    }

                    // 速度為零則跳過
                    if (dx == 0 && dy == 0 && dz == 0 && dYaw == 0 && dPitch == 0 && dRoll == 0)
                    {
                        Thread.Sleep(CONTINUOUS_CYCLE_MS);
                        continue;
                    }

                    // 目前末端姿態
                    var currentPos = AxisCard.Pos; // mdeg
                    var currentPosture = _kin.ForwardMdeg(currentPos);
                    var curXYZ = RA605Kinematics.ExtractPosition(currentPosture);

                    // 計算未來位置（多點：每點間隔 CONTINUOUS_CYCLE_MS）
                    float dtSec = CONTINUOUS_CYCLE_MS / 1000f;

                    int[] dataCount = new int[AXIS_COUNT];
                    int[][] targetPos = new int[AXIS_COUNT][];
                    int[][] targetTime = new int[AXIS_COUNT][];
                    int[] strVel = new int[AXIS_COUNT];
                    int[] endVel = new int[AXIS_COUNT];

                    // 產生多個中間點
                    int pointCount = PVT_POINTS_PER_BATCH;
                    for (int a = 0; a < AXIS_COUNT; a++)
                    {
                        targetPos[a] = new int[pointCount];
                        targetTime[a] = new int[pointCount];
                        dataCount[a] = pointCount;
                    }

                    bool allValid = true;
                    int[] prevMdeg = (int[])currentPos.Clone();

                    for (int p = 0; p < pointCount; p++)
                    {
                        float t = (p + 1) * dtSec;

                        // 新的末端位置（簡化：只做位移增量，姿態暫不疊加 YPR 速度到矩陣）
                        // 注意：這裡用簡化方法，真正的持續移動應該在矩陣空間做增量
                        float newX = curXYZ[0] + dx * t;
                        float newY = curXYZ[1] + dy * t;
                        float newZ = curXYZ[2] + dz * t;

                        // 用目前姿態的旋轉部分加上微量 YPR 旋轉
                        // 簡化處理：直接在目前矩陣上疊加旋轉增量
                        float yawInc = dYaw * t / 1000f;   // mdeg/s → deg (已 *t)
                        float pitchInc = dPitch * t / 1000f;
                        float rollInc = dRoll * t / 1000f;

                        var rotInc = Matrix4x4.CreateRotationZ(yawInc * MathF.PI / 180f)
                                   * Matrix4x4.CreateRotationY(pitchInc * MathF.PI / 180f)
                                   * Matrix4x4.CreateRotationX(rollInc * MathF.PI / 180f);

                        // 新姿態 = 旋轉增量 × 目前旋轉，加上新平移
                        var newPosture = currentPosture;
                        // 提取旋轉部分
                        var curRot = new Matrix4x4(
                            currentPosture.M11, currentPosture.M12, currentPosture.M13, 0,
                            currentPosture.M21, currentPosture.M22, currentPosture.M23, 0,
                            currentPosture.M31, currentPosture.M32, currentPosture.M33, 0,
                            0, 0, 0, 1);
                        var newRot = rotInc * curRot;
                        newPosture.M11 = newRot.M11; newPosture.M12 = newRot.M12; newPosture.M13 = newRot.M13;
                        newPosture.M21 = newRot.M21; newPosture.M22 = newRot.M22; newPosture.M23 = newRot.M23;
                        newPosture.M31 = newRot.M31; newPosture.M32 = newRot.M32; newPosture.M33 = newRot.M33;
                        newPosture.M41 = newX; newPosture.M42 = newY; newPosture.M43 = newZ;

                        var ptMdeg = _kin.InverseMdeg(newPosture, prevMdeg);
                        if (ptMdeg == null)
                        {
                            _log.Warn($"持續移動：第 {p} 點 IK 無解（超出工作空間或關節限位），截斷");
                            allValid = false;
                            for (int a = 0; a < AXIS_COUNT; a++)
                                dataCount[a] = p; // p=0 時不送任何指令
                            break;
                        }

                        // 關節單步跳動保護
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
                            allValid = false;
                            for (int a = 0; a < AXIS_COUNT; a++)
                                dataCount[a] = p;
                            break;
                        }

                        for (int a = 0; a < AXIS_COUNT; a++)
                        {
                            targetPos[a][p] = ptMdeg[a];
                            targetTime[a][p] = (p + 1) * CONTINUOUS_CYCLE_MS;
                        }
                        prevMdeg = ptMdeg;
                    }

                    if (dataCount[0] > 0)
                    {
                        // endVel 不為零（因為要持續銜接）
                        for (int a = 0; a < AXIS_COUNT; a++)
                        {
                            strVel[a] = 0; // DLL 自動計算
                            // 計算末速度估計值（最後兩點的差/時間）
                            if (dataCount[a] >= 2)
                            {
                                int dp = targetPos[a][dataCount[a] - 1] - targetPos[a][dataCount[a] - 2];
                                endVel[a] = (int)(dp / dtSec);
                            }
                            else
                            {
                                endVel[a] = 0;
                            }
                        }

                        AxisCard.MoveMultiAxisPVT(dataCount, targetPos, targetTime, strVel, endVel);
                    }

                    // 等待接近本批執行完畢再送下一批
                    Thread.Sleep(CONTINUOUS_CYCLE_MS * PVT_POINTS_PER_BATCH / 2);
                }
            }
            catch (Exception ex)
            {
                _log.Error("持續移動控制迴圈異常", ex);
            }

            _log.Info("持續移動控制迴圈結束");
        }

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
            var targetMdeg = _kin.InverseMdeg(targetPosture, currentPos);
            if (targetMdeg == null)
            {
                _log.Error("MoveRelativeEndEffector 拒絕：IK 無解");
                return false;
            }

            // 4. 計算移動時間（由 maxSpeed 和最大軸位移決定）
            int maxDelta = 0;
            for (int i = 0; i < AXIS_COUNT; i++)
            {
                int delta = Math.Abs(targetMdeg[i] - currentPos[i]);
                if (delta > maxDelta) maxDelta = delta;
            }

            int moveTimeMs = maxSpeed > 0 ? Math.Max(100, (int)(maxDelta * 1000.0 / maxSpeed)) : 1000;

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
