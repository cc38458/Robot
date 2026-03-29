using Robot.Core.Enums;

namespace Robot.Core.Models
{
    /// <summary>
    /// 運動指令資料結構，用於指令隊列
    /// 每個指令包含完整的執行資訊，通訊線程可獨立執行
    /// </summary>
    public class MotionCommand
    {
        /// <summary>指令類型</summary>
        public CommandType Type { get; set; }

        /// <summary>目標軸號（0–5），多軸指令時為主軸或忽略</summary>
        public ushort Axis { get; set; }

        /// <summary>目標位置或距離（單位：mdeg，千分之一度）</summary>
        public int Dist { get; set; }

        /// <summary>初始速度（單位：mdeg/s）</summary>
        public int StrVel { get; set; }

        /// <summary>等速速度（單位：mdeg/s）</summary>
        public int ConstVel { get; set; }

        /// <summary>結束速度（單位：mdeg/s）</summary>
        public int EndVel { get; set; }

        /// <summary>加速時間（秒）</summary>
        public double TAcc { get; set; }

        /// <summary>減速時間（秒）</summary>
        public double TDec { get; set; }

        // ── VelocityChange 專用欄位 ──

        /// <summary>新目標速度（單位：mdeg/s）</summary>
        public int NewTargetSpd { get; set; }

        /// <summary>速度變化時間（秒）</summary>
        public double TSec { get; set; }

        // ── PVT 專用欄位 ──

        /// <summary>PVT 資料筆數</summary>
        public int DataCount { get; set; }

        /// <summary>PVT 目標位置陣列（mdeg）</summary>
        public int[]? TargetPos { get; set; }

        /// <summary>PVT 目標時間陣列（毫秒）</summary>
        public int[]? TargetTime { get; set; }

        // ── 多軸同步 PVT 專用欄位 ──

        /// <summary>多軸 PVT：各軸資料筆數（長度 6）</summary>
        public int[]? MultiDataCount { get; set; }

        /// <summary>多軸 PVT：各軸目標位置陣列（6 組）</summary>
        public int[][]? MultiTargetPos { get; set; }

        /// <summary>多軸 PVT：各軸目標時間陣列（6 組，應一致）</summary>
        public int[][]? MultiTargetTime { get; set; }

        /// <summary>多軸 PVT：各軸初始速度</summary>
        public int[]? MultiStrVel { get; set; }

        /// <summary>多軸 PVT：各軸結束速度</summary>
        public int[]? MultiEndVel { get; set; }

        // ── Barrier 同步欄位 ──

        /// <summary>
        /// Barrier 群組 ID。相同 ID 的指令會等到同群組全部入隊後一起發送。
        /// null 表示不需要同步，立即可發送。
        /// </summary>
        public int? BarrierGroupId { get; set; }

        /// <summary>指令建立時間戳</summary>
        public DateTime CreatedAt { get; set; } = DateTime.Now;

        /// <summary>指令唯一識別碼</summary>
        public Guid Id { get; set; } = Guid.NewGuid();

        /// <summary>
        /// 回傳可讀的指令摘要字串，用於日誌與除錯。
        /// </summary>
        public override string ToString()
        {
            return Type switch
            {
                CommandType.MoveAbsolute => $"[MoveAbs] 軸{Axis} → {Dist} mdeg, V={ConstVel}",
                CommandType.MoveRelative => $"[MoveRel] 軸{Axis} Δ{Dist} mdeg, V={ConstVel}",
                CommandType.MovePV => $"[MovePV] 軸{Axis} V={ConstVel} mdeg/s",
                CommandType.MovePT => $"[MovePT] 軸{Axis} {DataCount}筆",
                CommandType.Stop => $"[Stop] 軸{Axis} 減速{TDec}s",
                CommandType.MultiAxisPVT => $"[MultiPVT] 6軸同步 {MultiDataCount?[0]}筆",
                CommandType.VelocityChange => $"[VelChange] 軸{Axis} → {NewTargetSpd} mdeg/s, {TSec}s",
                CommandType.TargetPositionChange => $"[TargetPosChange] 軸{Axis} → {Dist} mdeg",
                _ => $"[{Type}] 軸{Axis}",
            };
        }
    }
}
