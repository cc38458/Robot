using System.Numerics;
using Robot.Core.Enums;
using Robot.Core.Logging;
using Robot.Driver.Delta;

namespace Robot.Motion.RA605
{
    /// <summary>
    /// RA605 對外高階入口（建議上位程式直接使用本類別）
    /// 功能：
    /// 1) Real/Mock 後端切換
    /// 2) 高階運動控制（透過 MotionController）
    /// 3) 唯讀 Web 監控啟動
    /// </summary>
    public sealed class RA605RobotApp : IDisposable
    {
        private readonly RobotLogger _log;
        private readonly DeltaDriver _driver;
        private readonly MotionController _motion;

        private MonitorServer? _monitor;
        private bool _disposed;

        public RobotBackendMode BackendMode { get; }

        public CardState AxisCardState => _driver.AxisCardState;
        public int[] Pos => _driver.Pos;
        public int[] Speed => _driver.Speed;
        public MotorState[] MotorState => _driver.State;
        public int[] QueueLength => _driver.QueueLength;
        public float[] EndEffectorPosition => _motion.EndEffectorPosition;
        public Matrix4x4 EndEffectorPosture => _motion.EndEffectorPosture;

        /// <summary>
        /// 建立 RA605 高階控制入口。
        /// </summary>
        /// <param name="backendMode">後端模式（Real/Mock）。</param>
        /// <param name="zeroConfigPath">零點設定檔路徑。</param>
        /// <param name="toolLength">工具長度（mm）。</param>
        /// <param name="logDirectory">日誌目錄。</param>
        /// <param name="logPrefix">日誌檔名前綴。</param>
        /// <param name="logLevel">最低記錄日誌等級。</param>
        public RA605RobotApp(
            RobotBackendMode backendMode,
            string zeroConfigPath = "axis_zero_config.json",
            float toolLength = 0f,
            string logDirectory = "logs",
            string logPrefix = "RA605App",
            LogLevel logLevel = LogLevel.INFO)
        {
            BackendMode = backendMode;
            _log = new RobotLogger(logDirectory, logPrefix, logLevel);

            bool useMock = backendMode == RobotBackendMode.Mock;
            _driver = new DeltaDriver(_log, zeroConfigPath, useMockBackend: useMock);
            _motion = new MotionController(_driver, _log, toolLength);

            _log.Info($"RA605RobotApp 建立完成，後端模式：{backendMode}");
        }

        // ===== 生命週期 =====

        /// <summary>
        /// 建立與後端（Real/Mock）的連線。
        /// </summary>
        /// <returns>成功回傳 true，失敗回傳 false。</returns>
        public bool Connect() => _driver.Start();

        /// <summary>
        /// 初始化軸卡與各軸（齒輪比、零點、Servo ON）。
        /// </summary>
        /// <returns>成功回傳 true，失敗回傳 false。</returns>
        public bool Initialize() => _driver.Initial();

        /// <summary>
        /// 關閉軸卡連線並結束驅動。
        /// </summary>
        /// <returns>成功回傳 true，失敗回傳 false。</returns>
        public bool End() => _driver.End();

        // ===== 安全控制 =====

        /// <summary>
        /// 對所有軸執行緊急停止。
        /// </summary>
        /// <returns>成功回傳 true，失敗回傳 false。</returns>
        public bool Estop() => _driver.Estop();

        /// <summary>
        /// 清除警報並嘗試回到可運轉狀態。
        /// </summary>
        /// <returns>成功回傳 true，失敗回傳 false。</returns>
        public bool Ralm() => _driver.Ralm();

        // ===== 高階運動 =====

        /// <summary>
        /// 末端執行器移動到指定絕對姿態。
        /// </summary>
        /// <param name="targetPosture">目標齊次矩陣姿態。</param>
        /// <param name="moveTimeMs">移動時間（毫秒）。</param>
        /// <returns>命令送出成功回傳 true，否則 false。</returns>
        public bool MoveToPosture(Matrix4x4 targetPosture, int moveTimeMs)
            => _motion.MoveToPosture(targetPosture, moveTimeMs);

        /// <summary>
        /// 末端執行器做一次性相對位移/姿態變化。
        /// </summary>
        /// <param name="dx">X 位移（mm）。</param>
        /// <param name="dy">Y 位移（mm）。</param>
        /// <param name="dz">Z 位移（mm）。</param>
        /// <param name="dYaw">Yaw 位移（mdeg）。</param>
        /// <param name="dPitch">Pitch 位移（mdeg）。</param>
        /// <param name="dRoll">Roll 位移（mdeg）。</param>
        /// <param name="maxSpeed">最大速度（mdeg/s）。</param>
        /// <returns>命令送出成功回傳 true，否則 false。</returns>
        public bool MoveRelativeEndEffector(float dx, float dy, float dz,
                                            float dYaw, float dPitch, float dRoll,
                                            int maxSpeed)
            => _motion.MoveRelativeEndEffector(dx, dy, dz, dYaw, dPitch, dRoll, maxSpeed);

        /// <summary>
        /// 開始末端持續相對移動。
        /// </summary>
        /// <param name="deltaX">X 方向速度（mm/s）。</param>
        /// <param name="deltaY">Y 方向速度（mm/s）。</param>
        /// <param name="deltaZ">Z 方向速度（mm/s）。</param>
        /// <param name="deltaYaw">Yaw 角速度（mdeg/s）。</param>
        /// <param name="deltaPitch">Pitch 角速度（mdeg/s）。</param>
        /// <param name="deltaRoll">Roll 角速度（mdeg/s）。</param>
        /// <returns>命令送出成功回傳 true，否則 false。</returns>
        public bool StartContinuousMove(float deltaX, float deltaY, float deltaZ,
                                        float deltaYaw, float deltaPitch, float deltaRoll)
            => _motion.StartContinuousMove(deltaX, deltaY, deltaZ, deltaYaw, deltaPitch, deltaRoll);

        /// <summary>
        /// 更新末端持續相對移動的速度向量。
        /// </summary>
        /// <param name="deltaX">X 方向速度（mm/s）。</param>
        /// <param name="deltaY">Y 方向速度（mm/s）。</param>
        /// <param name="deltaZ">Z 方向速度（mm/s）。</param>
        /// <param name="deltaYaw">Yaw 角速度（mdeg/s）。</param>
        /// <param name="deltaPitch">Pitch 角速度（mdeg/s）。</param>
        /// <param name="deltaRoll">Roll 角速度（mdeg/s）。</param>
        /// <returns>命令送出成功回傳 true，否則 false。</returns>
        public bool UpdateContinuousMove(float deltaX, float deltaY, float deltaZ,
                                         float deltaYaw, float deltaPitch, float deltaRoll)
            => _motion.UpdateContinuousMove(deltaX, deltaY, deltaZ, deltaYaw, deltaPitch, deltaRoll);

        /// <summary>
        /// 停止末端持續相對移動。
        /// </summary>
        /// <returns>停止成功回傳 true，否則 false。</returns>
        public bool StopContinuousMove() => _motion.StopContinuousMove();

        /// <summary>
        /// 單軸絕對角度移動。
        /// </summary>
        /// <param name="axis">軸號（0-5）。</param>
        /// <param name="angleMdeg">目標角度（mdeg）。</param>
        /// <param name="constVel">等速速度（mdeg/s）。</param>
        /// <param name="tAcc">加速時間（秒）。</param>
        /// <param name="tDec">減速時間（秒）。</param>
        /// <returns>命令送出成功回傳 true，否則 false。</returns>
        public bool MoveAxisAbsolute(ushort axis, int angleMdeg, int constVel,
                                     double tAcc = 0.3, double tDec = 0.3)
            => _motion.MoveAxisAbsolute(axis, angleMdeg, constVel, tAcc, tDec);

        /// <summary>
        /// 單軸相對角度移動。
        /// </summary>
        /// <param name="axis">軸號（0-5）。</param>
        /// <param name="deltaAngleMdeg">相對角度（mdeg）。</param>
        /// <param name="constVel">等速速度（mdeg/s）。</param>
        /// <param name="tAcc">加速時間（秒）。</param>
        /// <param name="tDec">減速時間（秒）。</param>
        /// <returns>命令送出成功回傳 true，否則 false。</returns>
        public bool MoveAxisRelative(ushort axis, int deltaAngleMdeg, int constVel,
                                     double tAcc = 0.3, double tDec = 0.3)
            => _motion.MoveAxisRelative(axis, deltaAngleMdeg, constVel, tAcc, tDec);

        /// <summary>
        /// 所有軸回軟體原點。
        /// </summary>
        /// <param name="constVel">等速速度（mdeg/s）。</param>
        /// <param name="tAcc">加速時間（秒）。</param>
        /// <param name="tDec">減速時間（秒）。</param>
        /// <returns>命令送出成功回傳 true，否則 false。</returns>
        public bool MoveHome(int constVel = 20000, double tAcc = 0.5, double tDec = 0.5)
            => _motion.MoveHome(constVel, tAcc, tDec);

        // ===== 原點標定 =====

        /// <summary>
        /// 原點標定：將目前各軸真實編碼器位置存為新的零點設定檔（axis_zero_config.json）。
        /// 僅 Real 後端有效；Mock 後端回傳 false 並記錄警告。
        /// 前提：已完成 Connect + Initialize（AxisCardState == READY）。
        /// </summary>
        public bool CalibrateZero()
        {
            if (BackendMode == RobotBackendMode.Mock)
            {
                _log.Info("Mock 模式：略過原點標定");
                return true;
            }
            return _driver.CalibrateZero();
        }

        // ===== Web 監控（唯讀） =====

        /// <summary>
        /// 啟動唯讀 Web 監控伺服器（不接受控制命令）
        /// </summary>
        /// <param name="port">監控端口，預設 5850</param>
        /// <param name="htmlPath">monitor.html 路徑；未提供時會嘗試常見位置</param>
        /// <returns>可開啟的 URL</returns>
        public string StartWebMonitor(int port = 5850, string? htmlPath = null)
        {
            if (_monitor != null)
                return $"http://localhost:{port}";

            string? resolved = htmlPath ?? FindDefaultMonitorHtmlPath();
            _monitor = new MonitorServer(_driver, _log, port, resolved);
            _monitor.Start();

            return $"http://localhost:{port}";
        }

        public void StopWebMonitor()
        {
            _monitor?.Dispose();
            _monitor = null;
        }

        /// <summary>
        /// 依常見路徑尋找 monitor.html。
        /// </summary>
        /// <returns>找到則回傳完整路徑，否則回傳 null。</returns>
        private static string? FindDefaultMonitorHtmlPath()
        {
            string baseDir = AppDomain.CurrentDomain.BaseDirectory;
            string[] candidates =
            {
                Path.Combine(baseDir, "monitor.html"),
                Path.Combine(baseDir, "..", "..", "..", "..", "Robot.MockConsole", "monitor.html"),
                Path.Combine(baseDir, "..", "..", "..", "monitor.html"),
                Path.Combine("Robot.MockConsole", "monitor.html"),
            };

            foreach (var p in candidates)
            {
                try
                {
                    var full = Path.GetFullPath(p);
                    if (File.Exists(full)) return full;
                }
                catch { }
            }
            return null;
        }

        /// <summary>
        /// 釋放所有資源，並嘗試安全關閉監控與驅動。
        /// </summary>
        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            try { StopWebMonitor(); } catch { }
            try { _motion.Dispose(); } catch { }
            try { _driver.End(); } catch { }
            try { _driver.Dispose(); } catch { }
            _log.Dispose();

            GC.SuppressFinalize(this);
        }
    }
}
