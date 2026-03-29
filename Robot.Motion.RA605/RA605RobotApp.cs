using System.Numerics;
using Robot.Core.Enums;
using Robot.Core.Interfaces;
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
        private readonly IAxisCard _driver;
        private readonly MotionController _motion;

        private MonitorServer? _monitor;
        private bool _disposed;

        /// <summary>目前後端模式（Real 或 Mock）。</summary>
        public RobotBackendMode BackendMode { get; }

        /// <summary>軸卡狀態。</summary>
        public CardState AxisCardState => _driver.AxisCardState;
        /// <summary>各軸目前位置（mdeg）。</summary>
        public int[] Pos => _driver.Pos;
        /// <summary>各軸目前速度（mdeg/s）。</summary>
        public int[] Speed => _driver.Speed;
        /// <summary>各軸馬達狀態。</summary>
        public MotorState[] MotorState => _driver.State;
        /// <summary>各軸指令隊列長度。</summary>
        public int[] QueueLength => _driver.QueueLength;
        /// <summary>末端執行器位置 [X,Y,Z]（mm）。</summary>
        public float[] EndEffectorPosition => _motion.EndEffectorPosition;
        /// <summary>末端執行器姿態齊次矩陣。</summary>
        public Matrix4x4 EndEffectorPosture => _motion.EndEffectorPosture;
        /// <summary>目前目標姿態對應的六軸角度（mdeg）。</summary>
        public int[] TargetJointAngles => _motion.TargetJointAngles;
        /// <summary>最近一拍 continuous loop 的目標關節速度（mdeg/s）。</summary>
        public int[] TargetJointSpeedMdegPerSec => _motion.TargetJointSpeedMdegPerSec;
        /// <summary>最近一拍 continuous loop 真正送出的關節命令速度（mdeg/s）。</summary>
        public int[] CommandedJointSpeedMdegPerSec => _motion.CommandedJointSpeedMdegPerSec;
        /// <summary>最近一拍 continuous loop 依 target/current 計算出的預期限位端（mdeg）。</summary>
        public int[] ExpectedLimitTargetsMdeg => _motion.ExpectedLimitTargetsMdeg;
        /// <summary>最近一拍 continuous loop 實際追逐中的限位端（mdeg）。</summary>
        public int[] ActiveLimitTargetsMdeg => _motion.ActiveLimitTargetsMdeg;
        /// <summary>持續移動控制中的虛擬末端位置 [X,Y,Z]（mm）。</summary>
        public float[] VirtualEndEffectorPosition => _motion.VirtualEndEffectorPosition;
        /// <summary>最近一拍 continuous loop 的 tracking slowdown scale。</summary>
        public float ContinuousTrackingScale => _motion.ContinuousTrackingScale;
        /// <summary>最近一拍 continuous loop 的 singular slowdown scale。</summary>
        public float ContinuousSingularScale => _motion.ContinuousSingularScale;
        /// <summary>最近一拍 continuous loop 用於累積虛擬末端姿態的 cartesian slowdown scale。</summary>
        public float ContinuousCartesianSlowdownScale => _motion.ContinuousCartesianSlowdownScale;
        /// <summary>最近一拍 continuous loop 實際套用到虛擬末端設定點的平移速度 [X,Y,Z]（mm/s）。</summary>
        public float[] ContinuousAppliedLinearVelocity => _motion.ContinuousAppliedLinearVelocity;

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
            LogLevel logLevel = LogLevel.INFO,
            bool useOutOfProcess = true,
            string? commServicePath = null)
        {
            BackendMode = backendMode;
            _log = new RobotLogger(logDirectory, logPrefix, logLevel);

            bool useMock = backendMode == RobotBackendMode.Mock;
            _driver = AxisCardFactory.Create(
                _log,
                zeroConfigPath: zeroConfigPath,
                useMock: useMock,
                useOutOfProcess: useOutOfProcess,
                commServicePath: commServicePath,
                logLevel: logLevel);
            _motion = new MotionController(_driver, _log, toolLength);

            _log.Info($"RA605RobotApp 建立完成，後端模式：{backendMode}, OutOfProcess={useOutOfProcess}");
        }

        // ===== 生命週期 =====

        /// <summary>
        /// 建立與後端（Real/Mock）的連線。
        /// </summary>
        /// <returns>連線指令送出成功回傳 true，失敗回傳 false。</returns>
        public bool Connect() => _driver.Start();

        /// <summary>
        /// 初始化軸卡與各軸（零點、CSP、Servo ON）。
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
        /// 內部自動標記命令位置不可靠並清除 PV 狀態。
        /// </summary>
        /// <returns>成功回傳 true，失敗回傳 false。</returns>
        public bool Estop()
        {
            var result = _driver.Estop();
            _motion.OnEstop();
            return result;
        }

        /// <summary>
        /// 清除警報並嘗試回到可運轉狀態。
        /// 內部自動同步命令位置。
        /// </summary>
        /// <returns>成功回傳 true，失敗回傳 false。</returns>
        public bool Ralm()
        {
            var result = _driver.Ralm();
            if (result) _motion.OnAlarmCleared();
            return result;
        }

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
        /// 指定軸立即停止（無視該軸隊列，透過變速減速至 0 後停止）。
        /// 內部自動清除 PV 狀態並同步命令位置。
        /// </summary>
        /// <param name="axis">軸號（0-5）。</param>
        /// <param name="tDec">減速時間（秒），預設 0.5。</param>
        /// <returns>成功回傳 true，否則 false。</returns>
        public bool Stop(ushort axis, double tDec = 0.5)
            => _motion.StopAxis(axis, tDec);

        /// <summary>
        /// 停止末端持續相對移動。
        /// </summary>
        /// <returns>停止成功回傳 true，否則 false。</returns>
        public bool StopContinuousMove() => _motion.StopContinuousMove();

        /// <summary>
        /// 變更指定軸的移動速度。
        /// </summary>
        /// <param name="axis">軸號（0-5）。</param>
        /// <param name="newSpeed">新目標速度（mdeg/s）。</param>
        /// <param name="tSec">速度變化時間（秒）。</param>
        /// <returns>命令送出成功回傳 true，否則 false。</returns>
        public bool ChangeVelocity(ushort axis, int newSpeed, double tSec)
            => _driver.ChangeVelocity(axis, newSpeed, tSec);

        /// <summary>
        /// 變更指定軸在 CSP 模式下的目標位置。
        /// </summary>
        /// <param name="axis">軸號（0-5）。</param>
        /// <param name="newTargetMdeg">新的絕對目標位置（mdeg）。</param>
        /// <returns>命令送出成功回傳 true，否則 false。</returns>
        public bool ChangeTargetPosition(ushort axis, int newTargetMdeg)
            => _driver.ChangeTargetPosition(axis, newTargetMdeg);

        public bool TryGetAxisCommandTriplet(ushort axis, out int commandMdeg, out int actualCommandMdeg, out int targetCommandMdeg)
            => _driver.TryGetAxisCommandTriplet(axis, out commandMdeg, out actualCommandMdeg, out targetCommandMdeg);

        /// <summary>
        /// 單軸絕對角度移動的低階版本，可指定 endVel 以進入 CSP 追點狀態。
        /// </summary>
        public bool MoveAxisAbsoluteRaw(ushort axis, int angleMdeg, int strVel, int constVel,
                                        int endVel, double tAcc = 0.3, double tDec = 0.3)
            => _driver.MoveAbsolute(axis, angleMdeg, strVel, constVel, endVel, tAcc, tDec);

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

        /// <summary>
        /// 單軸等速持續移動（PV 模式）。
        /// 重複對同一軸呼叫時自動改用變速指令（ChangeVelocity）。
        /// 停止後內部自動同步命令位置。
        /// </summary>
        /// <param name="axis">軸號（0-5）。</param>
        /// <param name="strVel">起始速度（mdeg/s）。</param>
        /// <param name="constVel">等速速度（mdeg/s，正=正向/負=反向）。</param>
        /// <param name="tAcc">加速時間（秒）。</param>
        /// <returns>命令送出成功回傳 true，否則 false。</returns>
        public bool MovePV(ushort axis, int strVel, int constVel, double tAcc)
            => _motion.StartOrUpdatePV(axis, strVel, constVel, tAcc);

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
            _monitor = new MonitorServer(_motion, _log, port, resolved);
            _monitor.Start();

            return $"http://localhost:{port}";
        }

        /// <summary>停止 Web 監控伺服器並釋放相關資源。</summary>
        public void StopWebMonitor()
        {
            _monitor?.Dispose();
            _monitor = null;
        }

        /// <summary>依常見路徑尋找 monitor.html，找到回傳完整路徑，否則 null。</summary>
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

        /// <summary>釋放所有資源，並嘗試安全關閉監控與驅動。</summary>
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
