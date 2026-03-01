using Robot.Core.Logging;
using Robot.Driver.Delta;

namespace Robot.MockConsole
{
    /// <summary>
    /// Mock 模式主控台程式
    /// 啟動流程：建立 Mock 驅動 → 連線/初始化 → 啟動 WebSocket 監控伺服器
    /// 瀏覽器開啟 http://localhost:5850 即可看到 3D 機械臂監控介面
    /// </summary>
    class Program
    {
        static void Main(string[] args)
        {
            Console.OutputEncoding = System.Text.Encoding.UTF8;
            Console.Title = "RA605 Mock Console";

            PrintBanner();

            // ── 日誌 ──
            var log = new RobotLogger("logs", "MockConsole", LogLevel.DEBUG);
            log.Info("Mock 主控台啟動");

            // ── Mock 驅動 ──
            var driver = new DeltaDriver(log, "axis_zero_config.json");

            // ── 連線/初始化 ──
            Console.ForegroundColor = ConsoleColor.Cyan;
            Console.WriteLine("\n[1/3] 建立 EtherCAT 連線 (Mock)...");
            Console.ResetColor();

            if (!driver.Start())
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine($"連線失敗！AxisCardState = {driver.AxisCardState}");
                Console.ResetColor();
                Console.WriteLine("請查看 logs/ 目錄下的日誌檔案以取得詳細資訊。");
                Console.WriteLine("按任意鍵結束...");
                Console.ReadKey();
                return;
            }
            Console.ForegroundColor = ConsoleColor.Green;
            Console.WriteLine("      連線成功 ✓");

            Console.ForegroundColor = ConsoleColor.Cyan;
            Console.WriteLine("[2/3] 初始化軸（齒輪比 + 零點 + Servo ON）...");
            Console.ResetColor();

            if (!driver.Initial())
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine("初始化失敗！");
                Console.ResetColor();
                return;
            }
            Console.ForegroundColor = ConsoleColor.Green;
            Console.WriteLine("      初始化完成 ✓  AxisCardState = READY");

            // ── 監控伺服器 ──
            Console.ForegroundColor = ConsoleColor.Cyan;
            Console.WriteLine("[3/3] 啟動 WebSocket 監控伺服器...");
            Console.ResetColor();

            // 尋找 monitor.html
            string? htmlPath = FindHtmlFile();
            var server = new MonitorServer(driver, log, port: 5850, htmlPath: htmlPath);
            server.Start();

            Console.ForegroundColor = ConsoleColor.Yellow;
            Console.WriteLine();
            Console.WriteLine("  ╔═══════════════════════════════════════════════╗");
            Console.WriteLine("  ║  瀏覽器開啟 http://localhost:5850            ║");
            Console.WriteLine("  ║  即可檢視 RA605 3D 即時監控介面              ║");
            Console.WriteLine("  ╚═══════════════════════════════════════════════╝");
            Console.ResetColor();
            Console.WriteLine();

            if (htmlPath != null)
                log.Info($"monitor.html 已找到：{htmlPath}");
            else
                log.Warn("monitor.html 未找到，瀏覽器僅顯示 WebSocket 端點提示");

            // ── 互動式主控台 ──
            PrintHelp();

            while (true)
            {
                Console.ForegroundColor = ConsoleColor.DarkGray;
                Console.Write("\n> ");
                Console.ResetColor();

                var input = Console.ReadLine()?.Trim().ToLower();
                if (string.IsNullOrEmpty(input)) continue;

                var parts = input.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                var cmd = parts[0];

                try
                {
                    switch (cmd)
                    {
                        case "q":
                        case "quit":
                        case "exit":
                            goto Shutdown;

                        case "help":
                        case "h":
                        case "?":
                            PrintHelp();
                            break;

                        case "status":
                        case "s":
                            PrintStatus(driver);
                            break;

                        case "home":
                            driver.MoveHome(20000, 0.5, 0.5);
                            Console.WriteLine("回原點指令已發送");
                            break;

                        case "estop":
                            driver.Estop();
                            Console.ForegroundColor = ConsoleColor.Red;
                            Console.WriteLine("🛑 緊急停止！");
                            Console.ResetColor();
                            break;

                        case "ralm":
                            driver.Ralm();
                            Console.WriteLine("警報已清除");
                            break;

                        case "move":
                            // move <axis> <angle_deg>
                            if (parts.Length >= 3 &&
                                ushort.TryParse(parts[1], out var axis) &&
                                double.TryParse(parts[2], out var deg))
                            {
                                int mdeg = (int)(deg * 1000);
                                driver.MoveAbsolute(axis, mdeg, 0, 30000, 0, 0.3, 0.3);
                                Console.WriteLine($"軸 {axis} → {deg:F1}° ({mdeg} mdeg)");
                            }
                            else
                            {
                                Console.WriteLine("用法：move <軸號0-5> <角度>");
                            }
                            break;

                        case "alarm":
                            // 模擬觸發警報（測試用）
                            if (parts.Length >= 2 && ushort.TryParse(parts[1], out var alarmAxis))
                            {
                                // 透過反射呼叫 Mock 方法，避免非 MOCK 編譯時報錯
                                var dllType = Type.GetType("EtherCAT_DLL_x64.CEtherCAT_DLL, Robot.Driver.Delta");
                                var method = dllType?.GetMethod("Mock_TriggerAlarm",
                                    System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static);
                                if (method != null)
                                {
                                    method.Invoke(null, new object[] { alarmAxis });
                                    Console.ForegroundColor = ConsoleColor.Yellow;
                                    Console.WriteLine($"已模擬軸 {alarmAxis} 警報");
                                    Console.ResetColor();
                                }
                                else
                                {
                                    Console.WriteLine("此指令僅在 MOCK 模式下可用（Mock_TriggerAlarm 不存在）");
                                }
                            }
                            else
                            {
                                Console.WriteLine("用法：alarm <軸號0-5>");
                            }
                            break;

                        default:
                            Console.ForegroundColor = ConsoleColor.DarkGray;
                            Console.WriteLine($"未知指令：{cmd}（輸入 help 查看指令清單）");
                            Console.ResetColor();
                            break;
                    }
                }
                catch (Exception ex)
                {
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine($"錯誤：{ex.Message}");
                    Console.ResetColor();
                }
            }

        Shutdown:
            Console.WriteLine("\n正在關閉...");
            server.Dispose();
            driver.End();
            driver.Dispose();
            log.Info("Mock 主控台已關閉");
            log.Dispose();
            Console.WriteLine("已關閉。");
        }

        static void PrintBanner()
        {
            Console.ForegroundColor = ConsoleColor.Cyan;
            Console.WriteLine(@"
  ╦═╗╔═╗╔═╗ ╔═╗╔═╗╔═╗   ╔╦╗╔═╗╔═╗╦╔═
  ╠╦╝╠═╣║ ╦ ║ ║║ ║╠╣    ║║║║ ║║  ╠╩╗
  ╩╚═╩ ╩╚═╝ ╚═╝╚═╝╚     ╩ ╩╚═╝╚═╝╩ ╩
  ─────────────────────────────────────
  六軸機械臂 Mock 模擬 + 3D 監控系統");
            Console.ResetColor();
        }

        static void PrintHelp()
        {
            Console.ForegroundColor = ConsoleColor.DarkCyan;
            Console.WriteLine(@"
  指令清單：
  ─────────────────────────────────
  status (s)          顯示各軸狀態
  move <軸> <角度>    移動指定軸
  home                所有軸回原點
  estop               緊急停止
  ralm                清除警報
  alarm <軸>          模擬觸發警報
  help (h/?)          顯示此清單
  quit (q)            關閉程式
  ─────────────────────────────────");
            Console.ResetColor();
        }

        static void PrintStatus(DeltaDriver driver)
        {
            var pos = driver.Pos;
            var spd = driver.Speed;
            var st = driver.State;
            var ql = driver.QueueLength;

            Console.ForegroundColor = ConsoleColor.White;
            Console.WriteLine($"  軸卡狀態：{driver.AxisCardState}");
            Console.WriteLine("  ─────────────────────────────────────");
            Console.WriteLine("  軸   角度(°)      速度(mdeg/s)  狀態    隊列");

            for (int i = 0; i < 6; i++)
            {
                var stateColor = st[i] switch
                {
                    Robot.Core.Enums.MotorState.MOVING => ConsoleColor.Cyan,
                    Robot.Core.Enums.MotorState.ALARM => ConsoleColor.Red,
                    Robot.Core.Enums.MotorState.STOP => ConsoleColor.Green,
                    _ => ConsoleColor.DarkGray,
                };
                Console.ForegroundColor = ConsoleColor.Gray;
                Console.Write($"  J{i + 1}  {pos[i] / 1000.0,10:F3}  {spd[i],12}  ");
                Console.ForegroundColor = stateColor;
                Console.Write($"{st[i],-8}");
                Console.ForegroundColor = ConsoleColor.Gray;
                Console.WriteLine($"{ql[i],4}");
            }
            Console.ResetColor();
        }

        static string? FindHtmlFile()
        {
            // 搜尋常見位置
            string[] candidates =
            {
                "monitor.html",
                Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "monitor.html"),
                Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "..", "..", "..", "monitor.html"),
                Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "..", "..", "..", "..", "Robot.MockConsole", "monitor.html"),
            };

            foreach (var path in candidates)
            {
                if (File.Exists(path))
                    return Path.GetFullPath(path);
            }
            return null;
        }
    }
}
