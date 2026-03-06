using Robot.Motion.RA605;

namespace Demo.EndEffector;

internal static class Program
{
    static void Main()
    {
        Console.OutputEncoding = System.Text.Encoding.UTF8;
        Console.WriteLine("RA605 末端控制器");

        using var robot = new RA605RobotApp(
            backendMode: RobotBackendMode.Mock,
            zeroConfigPath: "axis_zero_config.json",
            toolLength: 60f,
            logDirectory: "logs",
            logPrefix: "EndEffector");

        Console.Write("連線中... ");
        if (!robot.Connect()) { Console.WriteLine("失敗"); return; }
        Console.WriteLine("成功");

        Console.Write("初始化中... ");
        if (!robot.Initialize()) { Console.WriteLine("失敗"); return; }
        Console.WriteLine("成功");

        // 唯讀 3D 監控（port 5850）
        string monitorUrl = robot.StartWebMonitor(5850);
        Console.WriteLine($"3D 監控：{monitorUrl}");

        // 控制伺服器（port 5851）
        string htmlPath = FindHtml("controller.html");
        using var ctrl = new ControlServer(robot, port: 5851, htmlPath: htmlPath);
        ctrl.Start();
        string ctrlUrl = "http://localhost:5851";
        Console.WriteLine($"控制頁面：{ctrlUrl}");

        // 嘗試開啟瀏覽器
        try
        {
            System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo
            {
                FileName = ctrlUrl,
                UseShellExecute = true,
            });
        }
        catch { }

        Console.WriteLine("按 Enter 結束...");
        Console.ReadLine();

        robot.StopContinuousMove();
        robot.End();
    }

    private static string FindHtml(string filename)
    {
        string baseDir = AppDomain.CurrentDomain.BaseDirectory;
        string[] candidates =
        {
            Path.Combine(baseDir, filename),
            Path.Combine(baseDir, "..", "..", "..", filename),
            filename,
        };
        foreach (var p in candidates)
        {
            try { var full = Path.GetFullPath(p); if (File.Exists(full)) return full; } catch { }
        }
        return filename;
    }
}
