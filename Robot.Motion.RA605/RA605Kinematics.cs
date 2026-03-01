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

        public RA605Kinematics(float toolLength = 60f)
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
        /// 無解回傳 null
        /// </summary>
        public float[]? Inverse(Matrix4x4 eePosture)
        {
            // 1. 減去工具長度，求手腕中心 OC
            var ocPosture = Matrix4x4.CreateTranslation(0, 0, -(D6 + ToolLength)) * eePosture;

            // 2. 第一軸
            float axis1 = -MathF.Atan2(ocPosture.M42, ocPosture.M41);

            // 3. 幾何解算
            float R = MathF.Sqrt(ocPosture.M41 * ocPosture.M41 + ocPosture.M42 * ocPosture.M42);
            float Rp = R - A1;
            float Zp = ocPosture.M43 - D1;

            float L3 = MathF.Sqrt(A3_Z * A3_Z + A3_X * A3_X);
            float distSq = Zp * Zp + Rp * Rp;

            float cosD = (A2 * A2 + L3 * L3 - distSq) / (2 * A2 * L3);
            if (cosD < -1f || cosD > 1f) return null;

            float d = MathF.Acos(cosD);
            float alpha = MathF.Atan2(A3_X, A3_Z);

            float cosA = (A2 * A2 + distSq - L3 * L3) / (2 * A2 * MathF.Sqrt(distSq));
            if (cosA < -1f || cosA > 1f) return null;

            float a = MathF.Acos(cosA);
            float beta = MathF.Atan2(Zp, Rp);

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
                return null;

            var rMatrix = ocPosture * ocPositionInv;
            CleanMatrix(ref rMatrix);

            // 5. Z-Y-Z 歐拉角 → Axis4, Axis5, Axis6
            var euler = EulerZYZ(rMatrix);

            // 6. 弧度轉角度
            return new float[]
            {
                axis1 * RAD2DEG, axis2 * RAD2DEG, axis3 * RAD2DEG,
                euler.X * RAD2DEG, euler.Y * RAD2DEG, euler.Z * RAD2DEG,
            };
        }

        /// <summary>逆向運動學（mdeg 版本）</summary>
        public int[]? InverseMdeg(Matrix4x4 eePosture)
        {
            var deg = Inverse(eePosture);
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

        /// <summary>Z-Y-Z 歐拉角分解</summary>
        private static Vector3 EulerZYZ(Matrix4x4 m)
        {
            const float eps = 1e-6f;
            float beta = MathF.Acos(Math.Clamp(m.M33, -1f, 1f));
            float alpha, gamma;

            if (MathF.Abs(beta) < eps || MathF.Abs(beta - MathF.PI) < eps)
            {
                gamma = 0;
                alpha = MathF.Abs(beta) < eps
                    ? MathF.Atan2(m.M21, m.M11)
                    : MathF.Atan2(-m.M21, -m.M11);
            }
            else
            {
                alpha = MathF.Atan2(m.M32, -m.M31);
                gamma = MathF.Atan2(m.M23, m.M13);
            }
            return new Vector3(alpha, beta, gamma);
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
