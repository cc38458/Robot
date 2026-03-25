using Robot.Core.Interfaces;
using Robot.Core.Logging;

namespace Robot.Driver.Delta
{
    /// <summary>
    /// 軸卡工廠：根據模式建立 IAxisCard 實例。
    ///   - useOutOfProcess=true（預設）：建立 PipeAxisCard，啟動獨立 CommService 行程
    ///   - useOutOfProcess=false：建立 DeltaDriver，在本行程內直接驅動 EtherCAT
    /// </summary>
    public static class AxisCardFactory
    {
        /// <summary>
        /// 建立 IAxisCard 實例。
        /// </summary>
        /// <param name="logger">日誌記錄器</param>
        /// <param name="zeroConfigPath">零點設定檔路徑</param>
        /// <param name="useMock">是否使用 Mock 後端</param>
        /// <param name="useOutOfProcess">
        /// true = 啟動獨立行程 CommService（生產環境建議）；
        /// false = 在本行程內建立 DeltaDriver（除錯或單行程模式）
        /// </param>
        /// <param name="commServicePath">
        /// CommService 執行檔路徑。預設為同目錄下的 Robot.CommService(.exe)。
        /// 僅 useOutOfProcess=true 時使用。
        /// </param>
        /// <returns>IAxisCard 實例</returns>
        public static IAxisCard Create(
            RobotLogger logger,
            string zeroConfigPath = "axis_zero_config.json",
            bool useMock = false,
            bool useOutOfProcess = true,
            string? commServicePath = null)
        {
            if (!useOutOfProcess)
            {
                logger.Info("AxisCardFactory：建立 In-Process DeltaDriver");
                return new DeltaDriver(logger, zeroConfigPath, useMock);
            }

            // 尋找 CommService 執行檔
            commServicePath ??= FindCommServicePath();
            logger.Info($"AxisCardFactory：建立 Out-of-Process PipeAxisCard（CommService={commServicePath}）");
            return new PipeAxisCard(logger, commServicePath, zeroConfigPath, useMock);
        }

        /// <summary>在常見位置尋找 CommService 執行檔。</summary>
        private static string FindCommServicePath()
        {
            var candidates = new[]
            {
                // 同一目錄
                Path.Combine(AppContext.BaseDirectory, "Robot.CommService.exe"),
                Path.Combine(AppContext.BaseDirectory, "Robot.CommService"),
                // dotnet run 模式
                Path.Combine(AppContext.BaseDirectory, "..", "Robot.CommService",
                    "bin", "Debug", "net8.0", "Robot.CommService.exe"),
                Path.Combine(AppContext.BaseDirectory, "..", "Robot.CommService",
                    "bin", "Debug", "net8.0", "Robot.CommService"),
            };

            foreach (var candidate in candidates)
            {
                var fullPath = Path.GetFullPath(candidate);
                if (File.Exists(fullPath))
                    return fullPath;
            }

            // 找不到就用 dotnet run 方式
            return "dotnet";
        }
    }
}
