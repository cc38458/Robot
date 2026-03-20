using System.Numerics;

namespace Robot.Core.Interfaces
{
    /// <summary>
    /// 高階運動控制介面（末端操作層）
    /// 負責正逆運動學轉換，將末端姿態指令轉為軸角度指令
    /// </summary>
    public interface IMotionController : IDisposable
    {
        /// <summary>底層軸卡驅動</summary>
        IAxisCard AxisCard { get; }

        /// <summary>
        /// 目前末端位置姿態 [X, Y, Z]（mm）
        /// 由正向運動學根據各軸角度即時計算
        /// </summary>
        float[] EndEffectorPosition { get; }

        /// <summary>
        /// 目前末端姿態旋轉矩陣（齊次矩陣）
        /// </summary>
        Matrix4x4 EndEffectorPosture { get; }

        // ════════════════════════════════════════
        // 末端運動模式
        // ════════════════════════════════════════

        /// <summary>
        /// 末端絕對姿態移動
        /// 輸入目標齊次矩陣與移動時間，在指定時間內移到指定位置
        /// 內部：IK 計算 → 6 軸 PVT 同步
        /// </summary>
        /// <param name="targetPosture">目標齊次矩陣（4x4，世界座標）</param>
        /// <param name="moveTimeMs">移動時間（毫秒）</param>
        /// <returns>true = 指令已成功送入隊列</returns>
        bool MoveToPosture(Matrix4x4 targetPosture, int moveTimeMs);

        /// <summary>
        /// 末端持續相對移動
        /// 以指定速度向量持續移動，直到呼叫 StopContinuousMove() 或送入零向量
        /// 位置速度單位：mm/s，姿態速度單位：mdeg/s
        /// 內部：控制迴圈週期性計算 PVT 點位送出
        /// ⚠ 隊列中此指令後面不可接其他指令
        /// </summary>
        /// <param name="deltaX">X 方向速度（mm/s）</param>
        /// <param name="deltaY">Y 方向速度（mm/s）</param>
        /// <param name="deltaZ">Z 方向速度（mm/s）</param>
        /// <param name="deltaYaw">偏航角速度（mdeg/s）</param>
        /// <param name="deltaPitch">俯仰角速度（mdeg/s）</param>
        /// <param name="deltaRoll">滾動角速度（mdeg/s）</param>
        bool StartContinuousMove(float deltaX, float deltaY, float deltaZ,
                                 float deltaYaw, float deltaPitch, float deltaRoll);

        /// <summary>
        /// 更新持續相對移動的速度向量（不停止，直接切換方向/速度）
        /// 送入全零向量等同於 StopContinuousMove()
        /// </summary>
        bool UpdateContinuousMove(float deltaX, float deltaY, float deltaZ,
                                  float deltaYaw, float deltaPitch, float deltaRoll);

        /// <summary>
        /// 停止持續相對移動（各軸減速停止）
        /// </summary>
        bool StopContinuousMove();

        /// <summary>
        /// 末端一次性相對移動
        /// 輸入位移量，以指定最大速度移動到目標
        /// 位置單位：mm，姿態單位：mdeg
        /// </summary>
        /// <param name="dx">X 位移（mm）</param>
        /// <param name="dy">Y 位移（mm）</param>
        /// <param name="dz">Z 位移（mm）</param>
        /// <param name="dYaw">偏航角位移（mdeg）</param>
        /// <param name="dPitch">俯仰角位移（mdeg）</param>
        /// <param name="dRoll">滾動角位移（mdeg）</param>
        /// <param name="maxSpeed">最大移動速度（取移動量最大的軸之速度，mdeg/s）</param>
        bool MoveRelativeEndEffector(float dx, float dy, float dz,
                                     float dYaw, float dPitch, float dRoll,
                                     int maxSpeed);

        // ════════════════════════════════════════
        // 軸空間運動模式
        // ════════════════════════════════════════

        /// <summary>
        /// 單軸絕對角度移動
        /// </summary>
        /// <param name="axis">軸號（0–5）</param>
        /// <param name="angleMdeg">目標角度（mdeg）</param>
        /// <param name="constVel">等速速度（mdeg/s）</param>
        /// <param name="tAcc">加速時間（秒）</param>
        /// <param name="tDec">減速時間（秒）</param>
        bool MoveAxisAbsolute(ushort axis, int angleMdeg, int constVel,
                              double tAcc, double tDec);

        /// <summary>
        /// 單軸相對角度移動
        /// </summary>
        bool MoveAxisRelative(ushort axis, int deltaAngleMdeg, int constVel,
                              double tAcc, double tDec);

        /// <summary>
        /// 所有軸回軟體原點
        /// </summary>
        /// <param name="constVel">移動速度（mdeg/s）</param>
        /// <param name="tAcc">加速時間（秒）</param>
        /// <param name="tDec">減速時間（秒）</param>
        bool MoveHome(int constVel, double tAcc, double tDec);
    }
}
