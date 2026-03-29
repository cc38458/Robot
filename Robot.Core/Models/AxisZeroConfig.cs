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
        /// CommThread 會以此值自行將實際脈波換算為 mdeg
        /// </summary>
        public int[] ZeroPulse { get; set; } = new int[6];

        /// <summary>設定檔案最後更新時間</summary>
        public DateTime LastModified { get; set; } = DateTime.Now;

        /// <summary>備註</summary>
        public string Note { get; set; } = "";
    }
}
