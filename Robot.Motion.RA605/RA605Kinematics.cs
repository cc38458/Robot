using System.Numerics;

namespace Robot.Motion.RA605
{
    /// <summary>
    /// RA605 六軸機械臂正逆運動學
    /// 正運動學：6 軸角度 → 4×4 齊次矩陣
    /// 逆運動學：4×4 齊次矩陣 → 6 軸角度
    /// 角度單位：度（degree），長度單位：毫米（mm）
    /// </summary>
    public class RA605Kinematics
    {
        private const float ARM_SINGULAR_COS_THRESHOLD = 0.995f;
        private const float WRIST_SINGULAR_DEG_THRESHOLD = 5f;

        // ── 關節角度物理限位（度）── RA605 規格
        private static readonly float[] JOINT_MIN = { -165f, -125f,  -55f, -190f, -115f, -360f };
        private static readonly float[] JOINT_MAX = {  165f,   85f,  185f,  190f,  115f,  360f };

        // ── DH 參數（RA605 機械臂） ──
        private const float D1 = 375f;
        private const float A1 = 30f;
        private const float A2 = 340f;
        private const float A3_X = 40f;
        private const float A3_Z = 338f;
        private const float D6 = 86.5f;

        private static readonly float RAD90 = MathF.PI / 2f;
        private static readonly float DEG2RAD = MathF.PI / 180f;
        private static readonly float RAD2DEG = 180f / MathF.PI;

        /// <summary>工具頭長度（mm）</summary>
        public float ToolLength { get; set; }

        public RA605Kinematics(float toolLength = 0f)
        {
            ToolLength = toolLength;
        }

        // ════════════════════════════════════════
        // 正向運動學
        // ════════════════════════════════════════

        /// <summary>
        /// 正向運動學：6 軸角度（度）→ 4×4 齊次矩陣
        /// </summary>
        public Matrix4x4 Forward(float[] angles)
        {
            if (angles.Length < 6)
                throw new ArgumentException("需要 6 個軸角度");

            float[] a = new float[6];
            for (int i = 0; i < 6; i++)
                a[i] = angles[i] * DEG2RAD;

            var m1 = Matrix4x4.CreateRotationX(RAD90)
                   * Matrix4x4.CreateTranslation(A1, 0, D1)
                   * Matrix4x4.CreateRotationZ(-a[0]);

            var m2 = Matrix4x4.CreateTranslation(A2, 0, 0)
                   * Matrix4x4.CreateRotationZ(-(a[1] - RAD90));

            var m3 = Matrix4x4.CreateTranslation(A3_X, 0, A3_Z)
                   * Matrix4x4.CreateRotationX(RAD90)
                   * Matrix4x4.CreateRotationZ(-a[2]);

            var m4 = Matrix4x4.CreateRotationX(-RAD90)
                   * Matrix4x4.CreateRotationZ(-a[3]);

            var m5 = Matrix4x4.CreateRotationX(RAD90)
                   * Matrix4x4.CreateRotationZ(-a[4]);

            var m6 = Matrix4x4.CreateTranslation(0, 0, D6 + ToolLength)
                   * Matrix4x4.CreateRotationZ(-a[5]);

            return m6 * m5 * m4 * m3 * m2 * m1;
        }

        /// <summary>正向運動學（mdeg 版本）</summary>
        public Matrix4x4 ForwardMdeg(int[] anglesMdeg)
        {
            float[] deg = new float[6];
            for (int i = 0; i < 6; i++)
                deg[i] = anglesMdeg[i] / 1000f;
            return Forward(deg);
        }

        // ════════════════════════════════════════
        // 逆向運動學
        // ════════════════════════════════════════

        /// <summary>
        /// 逆向運動學：4×4 齊次矩陣 → 6 軸角度（度）
        /// refAngles：當前關節角（度），用於在兩組 ZYZ 等價解中選擇轉動量最小者。
        /// 無解回傳 null
        /// </summary>
        public float[]? Inverse(Matrix4x4 eePosture, float[]? refAngles = null)
            => Inverse(eePosture, refAngles, out _);

        public float[]? Inverse(Matrix4x4 eePosture, float[]? refAngles, out string? diagnostic)
        {
            diagnostic = null;

            // 1. 減去工具長度，求手腕中心 OC
            var ocPosture = Matrix4x4.CreateTranslation(0, 0, -(D6 + ToolLength)) * eePosture;
            float ocX = ocPosture.M41;
            float ocY = ocPosture.M42;
            float ocZ = ocPosture.M43;

            // 2. 第一軸
            float axis1 = -MathF.Atan2(ocPosture.M42, ocPosture.M41);

            // 3. 幾何解算
            float R = MathF.Sqrt(ocPosture.M41 * ocPosture.M41 + ocPosture.M42 * ocPosture.M42);
            float Rp = R - A1;
            float Zp = ocPosture.M43 - D1;

            float L3 = MathF.Sqrt(A3_Z * A3_Z + A3_X * A3_X);
            float distSq = Zp * Zp + Rp * Rp;

            float cosD = (A2 * A2 + L3 * L3 - distSq) / (2 * A2 * L3);
            if (cosD < -1f || cosD > 1f)
            {
                diagnostic = $"IK失敗[arm]: OC=({ocX:F1},{ocY:F1},{ocZ:F1}) R={R:F1} Rp={Rp:F1} Zp={Zp:F1} cosD={cosD:F4}";
                return null;
            }

            float d = MathF.Acos(cosD);
            float alpha = MathF.Atan2(A3_X, A3_Z);

            float cosA = (A2 * A2 + distSq - L3 * L3) / (2 * A2 * MathF.Sqrt(distSq));
            if (cosA < -1f || cosA > 1f)
            {
                diagnostic = $"IK失敗[arm]: OC=({ocX:F1},{ocY:F1},{ocZ:F1}) R={R:F1} Rp={Rp:F1} Zp={Zp:F1} cosA={cosA:F4}";
                return null;
            }

            float a = MathF.Acos(cosA);
            float beta = MathF.Atan2(Zp, Rp);
            bool nearArmSingularity = MathF.Abs(cosA) >= ARM_SINGULAR_COS_THRESHOLD || MathF.Abs(cosD) >= ARM_SINGULAR_COS_THRESHOLD;

            float axis3 = RAD90 - d + alpha;
            float axis2 = RAD90 - a - beta;

            // 4. 旋轉矩陣分離
            var ocPosition = Matrix4x4.CreateTranslation(A3_X, 0, A3_Z)
                           * Matrix4x4.CreateRotationX(RAD90)
                           * Matrix4x4.CreateRotationZ(-axis3)
                           * Matrix4x4.CreateTranslation(A2, 0, 0)
                           * Matrix4x4.CreateRotationZ(-(axis2 - RAD90))
                           * Matrix4x4.CreateRotationX(RAD90)
                           * Matrix4x4.CreateTranslation(A1, 0, D1)
                           * Matrix4x4.CreateRotationZ(-axis1);

            if (!Matrix4x4.Invert(ocPosition, out var ocPositionInv))
            {
                diagnostic = $"IK失敗[matrix]: OC=({ocX:F1},{ocY:F1},{ocZ:F1}) arm=[{axis1 * RAD2DEG:F1},{axis2 * RAD2DEG:F1},{axis3 * RAD2DEG:F1}]";
                return null;
            }

            var rMatrix = ocPosture * ocPositionInv;
            CleanMatrix(ref rMatrix);

            float ref1 = refAngles != null && refAngles.Length >= 6 ? refAngles[0] : 0f;
            float ref2 = refAngles != null && refAngles.Length >= 6 ? refAngles[1] : 0f;
            float ref3 = refAngles != null && refAngles.Length >= 6 ? refAngles[2] : 0f;
            float ref4 = refAngles != null && refAngles.Length >= 6 ? refAngles[3] : 0f;
            float ref5 = refAngles != null && refAngles.Length >= 6 ? refAngles[4] : 0f;
            float ref6 = refAngles != null && refAngles.Length >= 6 ? refAngles[5] : 0f;

            // 5. Z-Y-Z 歐拉角 → Axis4, Axis5, Axis6
            // 手腕旋轉 rMatrix_col = Rz(-a[3]) * Ry(-a[4]) * Rz(-a[5])
            // EulerZYZ(Rz(α)*Ry(β)*Rz(γ)) 回傳 (α, β, γ) = (-a[3], -a[4], -a[5])
            // 注意 β 可能被 acos 取正值，此時 α,γ 偏移 π，但 a[3]=-α, a[4]=-β, a[5]=-γ 仍正確
            // 在 J5≈0 或 180° 的奇異點，使用目前 J4/J6 參考角分配 α/γ，避免離開奇異點時 J4 或 J6 跳 180°。
            var euler = EulerZYZ(Matrix4x4.Transpose(rMatrix), -ref4 * DEG2RAD, -ref6 * DEG2RAD);
            float wristBetaDeg = euler.Y * RAD2DEG;
            bool nearWristSingularity = MathF.Abs(wristBetaDeg) <= WRIST_SINGULAR_DEG_THRESHOLD
                || MathF.Abs(wristBetaDeg - 180f) <= WRIST_SINGULAR_DEG_THRESHOLD;

            // 6. 弧度轉角度
            float a1 = axis1 * RAD2DEG;
            float a2 = axis2 * RAD2DEG;
            float a3 = axis3 * RAD2DEG;

            // ZYZ 兩組等價解：(α,β,γ) 與 (α+π,−β,γ+π)
            float a4s1 = -euler.X * RAD2DEG;
            float a5s1 = -euler.Y * RAD2DEG;
            float a6s1 = -euler.Z * RAD2DEG;
            float a4s2 = -(euler.X + MathF.PI) * RAD2DEG;
            float a5s2 =  euler.Y * RAD2DEG;   // −(−β)
            float a6s2 = -(euler.Z + MathF.PI) * RAD2DEG;

            // 以參考角為基準，將每個角度正規化到「距參考最近的等價角」
            a1   = NearestAngle(a1,   ref1);
            a2   = NearestAngle(a2,   ref2);
            a3   = NearestAngle(a3,   ref3);
            a4s1 = NearestAngle(a4s1, ref4);
            a5s1 = NearestAngle(a5s1, ref5);
            a6s1 = NearestAngle(a6s1, ref6);
            a4s2 = NearestAngle(a4s2, ref4);
            a5s2 = NearestAngle(a5s2, ref5);
            a6s2 = NearestAngle(a6s2, ref6);

            // 選解策略：以 J4 轉動幅度最小為首要，嚴格避免 J4 翻轉 180°。
            // 兩組等價解的 J4 恆差 ~180°，所以 J4 偏差幾乎一定有明顯差距；
            // J5 可自由轉動，不參與選解判準。
            float j4dev1 = MathF.Abs(a4s1 - ref4);
            float j4dev2 = MathF.Abs(a4s2 - ref4);

            bool preferSol1;
            if (MathF.Abs(j4dev1 - j4dev2) > 1f) // > 1° → J4 偏差有明顯差距
            {
                preferSol1 = j4dev1 <= j4dev2;
            }
            else
            {
                // 極罕見 J4 偏差相近（ref4 恰在兩解正中間）：以總偏差為次要判準
                float dev1 = j4dev1 + MathF.Abs(a5s1 - ref5) + MathF.Abs(a6s1 - ref6);
                float dev2 = j4dev2 + MathF.Abs(a5s2 - ref5) + MathF.Abs(a6s2 - ref6);
                preferSol1 = dev1 <= dev2;
            }

            float[][] candidates = preferSol1
                ? new[]
                {
                    new[] { a1, a2, a3, a4s1, a5s1, a6s1 },
                    new[] { a1, a2, a3, a4s2, a5s2, a6s2 },
                }
                : new[]
                {
                    new[] { a1, a2, a3, a4s2, a5s2, a6s2 },
                    new[] { a1, a2, a3, a4s1, a5s1, a6s1 },
                };

            // 7. 關節限位檢查
            foreach (var candidate in candidates)
            {
                if (IsWithinJointLimits(candidate))
                {
                    if (nearArmSingularity || nearWristSingularity)
                    {
                        diagnostic = $"IK近奇異: OC=({ocX:F1},{ocY:F1},{ocZ:F1}) arm=[{candidate[0]:F1},{candidate[1]:F1},{candidate[2]:F1}] wrist=[{candidate[3]:F1},{candidate[4]:F1},{candidate[5]:F1}]"
                            + $"{(nearArmSingularity ? " arm" : "")}{(nearWristSingularity ? " wrist" : "")}";
                    }
                    return candidate;
                }
            }

            string fail1 = GetLimitFailure(candidates[0]);
            string fail2 = GetLimitFailure(candidates[1]);
            diagnostic = $"IK失敗[limit]: OC=({ocX:F1},{ocY:F1},{ocZ:F1}) arm=[{a1:F1},{a2:F1},{a3:F1}] s1=[{a4s1:F1},{a5s1:F1},{a6s1:F1}] {fail1}; s2=[{a4s2:F1},{a5s2:F1},{a6s2:F1}] {fail2}"
                + $"{(nearArmSingularity || nearWristSingularity ? " 近奇異" : "")}";
            return null;
        }

        /// <summary>逆向運動學（mdeg 版本）。refMdeg 為當前各軸角度（mdeg），用於選最小轉動解。</summary>
        public int[]? InverseMdeg(Matrix4x4 eePosture, int[]? refMdeg = null)
            => InverseMdeg(eePosture, refMdeg, out _);

        public int[]? InverseMdeg(Matrix4x4 eePosture, int[]? refMdeg, out string? diagnostic)
        {
            float[]? refAngles = refMdeg?.Select(m => m / 1000f).ToArray();
            var deg = Inverse(eePosture, refAngles, out diagnostic);
            if (deg == null) return null;
            int[] mdeg = new int[6];
            for (int i = 0; i < 6; i++)
                mdeg[i] = (int)MathF.Round(deg[i] * 1000f);
            return mdeg;
        }

        // ════════════════════════════════════════
        // 輔助方法
        // ════════════════════════════════════════

        /// <summary>
        /// 從 [X,Y,Z] + Yaw/Pitch/Roll 建構齊次矩陣
        /// 旋轉順序：Z(Yaw) × Y(Pitch) × X(Roll)
        /// </summary>
        public static Matrix4x4 PostureFromXYZYPR(float x, float y, float z,
                                                    float yawDeg, float pitchDeg, float rollDeg)
        {
            var mat = Matrix4x4.CreateRotationZ(yawDeg * DEG2RAD)
                    * Matrix4x4.CreateRotationY(pitchDeg * DEG2RAD)
                    * Matrix4x4.CreateRotationX(rollDeg * DEG2RAD)
                    * Matrix4x4.CreateTranslation(x, y, z);
            CleanMatrix(ref mat);
            return mat;
        }

        /// <summary>從齊次矩陣提取位移 [X,Y,Z]</summary>
        public static float[] ExtractPosition(Matrix4x4 mat)
            => new[] { mat.M41, mat.M42, mat.M43 };

        /// <summary>
        /// Z-Y-Z 歐拉角分解：M = Rz(α) * Ry(β) * Rz(γ)
        /// 回傳 Vector3(α, β, γ)
        /// </summary>
        private static Vector3 EulerZYZ(Matrix4x4 m, float? alphaRef = null, float? gammaRef = null)
        {
            const float eps = 1e-6f;
            float beta = MathF.Acos(Math.Clamp(m.M33, -1f, 1f));
            float alpha, gamma;

            if (MathF.Abs(beta) < eps)
            {
                float theta = MathF.Atan2(m.M21, m.M11); // alpha + gamma
                if (alphaRef.HasValue && gammaRef.HasValue)
                {
                    alpha = alphaRef.Value;
                    gamma = gammaRef.Value;
                    float delta = WrapToPi(theta - (alpha + gamma)) / 2f;
                    alpha += delta;
                    gamma += delta;
                }
                else
                {
                    gamma = 0;
                    alpha = theta;
                }
            }
            else if (MathF.Abs(beta - MathF.PI) < eps)
            {
                float theta = MathF.Atan2(-m.M21, -m.M11); // alpha - gamma
                if (alphaRef.HasValue && gammaRef.HasValue)
                {
                    alpha = alphaRef.Value;
                    gamma = gammaRef.Value;
                    float delta = WrapToPi(theta - (alpha - gamma)) / 2f;
                    alpha += delta;
                    gamma -= delta;
                }
                else
                {
                    gamma = 0;
                    alpha = theta;
                }
            }
            else
            {
                alpha = MathF.Atan2(m.M23, m.M13);
                gamma = MathF.Atan2(m.M32, -m.M31);
            }
            return new Vector3(alpha, beta, gamma);
        }

        /// <summary>
        /// 將 angle 正規化到距 reference 最近的等價角（差值限制在 ±180°）
        /// </summary>
        private static float NearestAngle(float angle, float reference)
        {
            float diff = angle - reference;
            diff -= MathF.Round(diff / 360f) * 360f;
            return reference + diff;
        }

        private static bool IsWithinJointLimits(float[] angles)
        {
            for (int i = 0; i < 6; i++)
            {
                if (angles[i] < JOINT_MIN[i] || angles[i] > JOINT_MAX[i])
                    return false;
            }
            return true;
        }

        private static string GetLimitFailure(float[] angles)
        {
            for (int i = 0; i < 6; i++)
            {
                if (angles[i] < JOINT_MIN[i] || angles[i] > JOINT_MAX[i])
                    return $"超限(J{i + 1}={angles[i]:F1}, 範圍 {JOINT_MIN[i]:F1}~{JOINT_MAX[i]:F1})";
            }
            return "OK";
        }

        private static float WrapToPi(float angle)
        {
            angle -= MathF.Round(angle / (2f * MathF.PI)) * (2f * MathF.PI);
            return angle;
        }

        /// <summary>清理矩陣浮點誤差</summary>
        private static void CleanMatrix(ref Matrix4x4 m)
        {
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                {
                    float v = m[i, j];
                    if (MathF.Abs(v) < 1e-6f) m[i, j] = 0;
                    else if (MathF.Abs(v - 1f) < 1e-6f) m[i, j] = 1f;
                    else if (MathF.Abs(v + 1f) < 1e-6f) m[i, j] = -1f;
                }
        }
    }
}
