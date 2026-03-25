using Robot.Core.Enums;

namespace Robot.Core.Interfaces
{
    /// <summary>
    /// 軸卡驅動介面
    /// 上層應用只對此介面寫程式，不直接碰 DLL。
    /// 實作類別內部運行獨立通訊線程，所有 EtherCAT DLL 呼叫皆在通訊線程中完成。
    /// </summary>
    public interface IAxisCard : IDisposable
    {
        // ════════════════════════════════════════
        // 屬性（唯讀，由通訊線程持續更新）
        // ════════════════════════════════════════

        /// <summary>各軸目前角度（單位：mdeg，千分之一度），共 6 元素</summary>
        int[] Pos { get; }

        /// <summary>各軸目前速度（單位：mdeg/s），共 6 元素</summary>
        int[] Speed { get; }

        /// <summary>各軸目前狀態，共 6 元素</summary>
        MotorState[] State { get; }

        /// <summary>軸卡整體狀態（由主線程持有）</summary>
        CardState AxisCardState { get; }

        /// <summary>各軸指令隊列中尚未執行的指令數量</summary>
        int[] QueueLength { get; }

        // ════════════════════════════════════════
        // 連線管理
        // ════════════════════════════════════════

        /// <summary>
        /// 開始非同步建立 EtherCAT 連線
        /// 前提：AxisCardState == NULL
        /// 動作：啟動通訊線程，開始連線流程
        ///       立即將 AxisCardState 設為 CONNING
        ///       連線成功 → CONNCET；失敗 → CONNERR
        /// </summary>
        /// <returns>true = 成功接受，false = 拒絕（狀態不符）</returns>
        bool Start();

        /// <summary>
        /// Servo OFF 所有軸，關閉 EtherCAT 主站連線，停止通訊線程
        /// 前提：所有軸 State == STOP（有任何軸 MOVING 時拒絕）
        /// 動作：AxisCardState → NULL
        /// </summary>
        bool End();

        /// <summary>
        /// 初始化所有軸並 Servo ON
        /// 前提：AxisCardState == CONNCET
        /// 動作：設定齒輪比、讀取零點設定、Virtual_Set、Servo ON
        ///       成功後 AxisCardState → READY，各軸 State → STOP
        /// </summary>
        bool Initial();

        // ════════════════════════════════════════
        // 安全控制（不經過隊列，最高優先）
        // ════════════════════════════════════════

        /// <summary>
        /// 緊急停止所有軸（無減速，立即停止）
        /// 無前提限制，任何狀態皆可呼叫
        /// 同時清除所有隊列中的待執行指令
        /// </summary>
        bool Estop();

        /// <summary>
        /// 清除所有軸的警報
        /// 前提：AxisCardState == ALARM 或任意軸 State == ALARM
        /// 動作：警報解除後 AxisCardState → READY
        /// </summary>
        bool Ralm();

        // ════════════════════════════════════════
        // 運動控制（透過通訊線程的指令隊列執行）
        // ════════════════════════════════════════

        /// <summary>
        /// 指定軸軟減速停止（無視該軸隊列，立即執行）
        /// 流程：先透過 VelocityChange 將速度減至 0 → 等待減速完成 → 執行 Sd_Stop
        /// 前提：AxisCardState == READY，axis ∈ [0,5]，tDec > 0
        /// </summary>
        /// <param name="axis">軸號（0–5）</param>
        /// <param name="tDec">減速時間（秒）</param>
        bool Stop(ushort axis, double tDec);

        /// <summary>
        /// 變更指定軸的移動速度（透過指令隊列執行）
        /// 前提：AxisCardState == READY，axis ∈ [0,5]
        /// </summary>
        /// <param name="axis">軸號（0–5）</param>
        /// <param name="newSpeed">新目標速度（mdeg/s）</param>
        /// <param name="tSec">速度變化時間（秒）</param>
        bool ChangeVelocity(ushort axis, int newSpeed, double tSec);

        /// <summary>
        /// 指定軸以梯形速度曲線移動至絕對位置
        /// 前提：AxisCardState == READY
        /// ⚠ endVel 不為零時，若後續無連續動作命令，驅動會自動補 Sd_Stop
        /// </summary>
        bool MoveAbsolute(ushort axis, int dist, int strVel, int constVel,
                          int endVel, double tAcc, double tDec);

        /// <summary>
        /// 指定軸以相對距離移動
        /// 前提：同 MoveAbsolute
        /// ⚠ 同上，endVel 不為零需接續動作
        /// </summary>
        bool MoveRelative(ushort axis, int dist, int strVel, int constVel,
                          int endVel, double tAcc, double tDec);

        /// <summary>
        /// 軸以等速持續運動，無目標位置，不會自動停止
        /// 前提：AxisCardState == READY
        /// ⚠ 必須由外部呼叫 Stop() 才會停下，否則撞機
        /// </summary>
        bool MovePV(ushort axis, int strVel, int constVel, double tAcc);

        /// <summary>
        /// 依傳入的位置表與時間表，驅動自動計算速度曲線（PVTComplete）
        /// 前提：AxisCardState == READY
        /// ⚠ strVel 必須與上一段 endVel 連續
        /// </summary>
        bool MovePT(ushort axis, int dataCnt, int[] targetPos,
                    int[] targetTime, int strVel, int endVel);

        /// <summary>
        /// 多軸同步 PVT 移動（6 軸同時發送，用於末端運動）
        /// 內部使用 PVTComplete_Config + PVT_Sync_Move 實現同步
        /// </summary>
        bool MoveMultiAxisPVT(int[] dataCount, int[][] targetPos,
                              int[][] targetTime, int[] strVel, int[] endVel);

        /// <summary>
        /// 回軟體原點（所有軸移動至 0 度位置）
        /// 前提：AxisCardState == READY，所有軸 State == STOP
        /// </summary>
        bool MoveHome(int constVel, double tAcc, double tDec);
    }
}
