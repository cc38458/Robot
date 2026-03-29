using System.Text.Json;
using System.Text.Json.Serialization;

namespace Robot.Core.IPC
{
    /// <summary>
    /// 命名管道 JSON Line 協定：請求與回應物件定義。
    /// 每筆訊息為一行 JSON + '\n'，透過 StreamReader/StreamWriter 讀寫。
    /// </summary>
    public static class PipeProtocol
    {
        public const string PIPE_NAME = "RobotCommService";

        private static readonly JsonSerializerOptions _jsonOpts = new()
        {
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
        };

        public static string Serialize<T>(T obj) => JsonSerializer.Serialize(obj, _jsonOpts);
        public static T? Deserialize<T>(string json) => JsonSerializer.Deserialize<T>(json, _jsonOpts);
    }

    /// <summary>
    /// 管道請求：主程式 → CommService
    /// </summary>
    public class PipeRequest
    {
        /// <summary>請求唯一 ID（用於配對回應）</summary>
        public string Id { get; set; } = Guid.NewGuid().ToString("N");

        /// <summary>
        /// 指令名稱，對應 IAxisCard 方法名：
        /// Start, End, Initial, Estop, Ralm, Stop, ChangeVelocity,
        /// MoveAbsolute, MoveRelative, MovePV, MovePT, MoveMultiAxisPVT, MoveHome,
        /// AbortAndChangePosition, ChangeTargetPosition
        /// </summary>
        public string Cmd { get; set; } = "";

        // ── 單軸參數 ──
        public ushort? Axis { get; set; }
        public int? Dist { get; set; }
        public int? StrVel { get; set; }
        public int? ConstVel { get; set; }
        public int? EndVel { get; set; }
        public double? TAcc { get; set; }
        public double? TDec { get; set; }
        public int? NewSpeed { get; set; }
        public double? TSec { get; set; }

        // ── PVT 專用 ──
        public int? DataCnt { get; set; }
        public int[]? TargetPos { get; set; }
        public int[]? TargetTime { get; set; }

        // ── 多軸 PVT 專用 ──
        public int[]? MultiDataCount { get; set; }
        public int[][]? MultiTargetPos { get; set; }
        public int[][]? MultiTargetTime { get; set; }
        public int[]? MultiStrVel { get; set; }
        public int[]? MultiEndVel { get; set; }

        // ── AbortAndChangePosition 專用 ──
        /// <summary>各軸目標角度（mdeg），共 6 元素</summary>
        public int[]? MultiDist { get; set; }
    }

    /// <summary>
    /// 管道回應：CommService → 主程式
    /// </summary>
    public class PipeResponse
    {
        /// <summary>對應請求的 ID</summary>
        public string Id { get; set; } = "";

        /// <summary>操作是否成功</summary>
        public bool Ok { get; set; }

        /// <summary>失敗時的錯誤訊息</summary>
        public string? Error { get; set; }
    }
}
