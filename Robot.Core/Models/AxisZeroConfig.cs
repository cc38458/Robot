namespace Robot.Core.Models
{
    /// <summary>
    /// 軸零點設定（從 JSON 檔讀取）
    /// 記錄各軸在絕對編碼器下的零點脈波位置
    /// </summary>
    public class AxisZeroConfig
    {
        /// <summary>
        /// 各軸零點的絕對脈波數（長度 6）
        /// 開機時透過 Virtual_Set_Enable + Virtual_Set_Command 平移座標
        /// </summary>
        public int[] ZeroPulse { get; set; } = new int[6];

        /// <summary>
        /// 各軸脈波與角度的比例（pulse per degree * 1000）
        /// 預設 RA605 的值：{ 29049, 35959, 29479, 29479, 28889, 18194 }
        /// 用於 CSP_Set_Gear(CardNo, NodeID, 0, pulse2ang[i], 1000, 1)
        /// </summary>
        public int[] Pulse2Ang { get; set; } = { 29049, 35959, 29479, 29479, 28889, 18194 };

        /// <summary>設定檔案最後更新時間</summary>
        public DateTime LastModified { get; set; } = DateTime.Now;

        /// <summary>備註</summary>
        public string Note { get; set; } = "";
    }
}
