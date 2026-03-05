using Robot.Motion.RA605;
using Robot.Core.Enums;

namespace Demo.StepConsole;

internal static class Program
{
    static void Main()
    {
        Console.OutputEncoding = System.Text.Encoding.UTF8;
        Console.WriteLine("RA605 逐步控制範例（Mock + 唯讀 Web 監控）");
        Console.WriteLine("每個步驟執行完成後，按 Enter 進入下一步。\n");

        using var robot = new RA605RobotApp(
            backendMode: RobotBackendMode.Real,
            zeroConfigPath: "axis_zero_config.json",
            toolLength: 60f,
            logDirectory: "logs",
            logPrefix: "StepConsole");

        RunStep("1) 開啟手臂（Connect）", () =>
        {
            if (!robot.Connect())
                throw new InvalidOperationException($"Connect 失敗，狀態={robot.AxisCardState}");
            Console.WriteLine($"已連線，狀態：{robot.AxisCardState}");
        });

        RunStep("2) 初始化（Initialize）＋開啟 Web 監控頁", () =>
        {
            if (!robot.Initialize())
                throw new InvalidOperationException($"Initialize 失敗，狀態={robot.AxisCardState}");
            Console.WriteLine($"初始化完成，狀態：{robot.AxisCardState}");

            int[] pos = robot.Pos;
            Console.WriteLine("目前虛擬位置（校正後，單位 mdeg）：");
            for (int i = 0; i < pos.Length; i++)
                Console.WriteLine($"  J{i + 1}: {pos[i],10} mdeg  ({pos[i] / 1000.0,8:F3}°)");

            string url = robot.StartWebMonitor(5850);
            Console.WriteLine($"監控頁：{url}");
            Console.WriteLine("請用瀏覽器開啟網址查看手臂狀態（唯讀）。");
        });

        RunStep("3) 原點標定", () =>
        {
            if (robot.BackendMode != RobotBackendMode.Mock)
            {
                Console.WriteLine("請將手臂移動至機械原點，確認後按 Enter 繼續標定...");
                Console.ReadLine();
            }
            bool ok = robot.CalibrateZero();
            if (!ok) throw new InvalidOperationException("原點標定失敗");
            Console.WriteLine(robot.BackendMode == RobotBackendMode.Mock
                ? "Mock 模式：略過原點標定。"
                : "原點標定完成，已更新 axis_zero_config.json。");
        });

        RunStep("4) 第一軸移動到 30 度", () =>
        {
            bool ok = robot.MoveAxisAbsolute(axis: 0, angleMdeg: 30000, constVel: 30000);
            if (!ok) throw new InvalidOperationException("J1 移動命令送出失敗");
            Console.WriteLine("J1 移動命令已送出（30°）。");
            WaitUntilAllAxisStop(robot, 20);
        });

        RunStep("5) 回原點", () =>
        {
            bool ok = robot.MoveHome(constVel: 20000, tAcc: 0.5, tDec: 0.5);
            if (!ok) throw new InvalidOperationException("回原點命令送出失敗");
            Console.WriteLine("回原點命令已送出。");
            WaitUntilAllAxisStop(robot, 25);
        });

        RunStep("6) 多軸同動（末端相對移動）", () =>
        {
            bool ok = robot.MoveRelativeEndEffector(
                dx: 30, dy: 20, dz: -20,
                dYaw: 0, dPitch: 0, dRoll: 0,
                maxSpeed: 20000);
            if (!ok) throw new InvalidOperationException("多軸同動命令送出失敗");
            Console.WriteLine("多軸同動命令已送出（末端相對位移）。");
            WaitUntilAllAxisStop(robot, 25);
        });

        RunStep("7) 再次回原點", () =>
        {
            bool ok = robot.MoveHome(constVel: 20000, tAcc: 0.5, tDec: 0.5);
            if (!ok) throw new InvalidOperationException("回原點命令送出失敗");
            Console.WriteLine("回原點命令已送出。");
            WaitUntilAllAxisStop(robot, 25);
        });

        RunStep("8) 關閉手臂（End）", () =>
        {
            bool ok = robot.End();
            if (!ok) throw new InvalidOperationException("End 失敗（可能仍在 MOVING）");
            Console.WriteLine("手臂已關閉。\n");
        }, waitAfter: false);

        Console.WriteLine("流程完成。按 Enter 結束程式。");
        Console.ReadLine();
    }

    private static void RunStep(string title, Action action, bool waitAfter = true)
    {
        Console.WriteLine("====================================================");
        Console.WriteLine(title);
        Console.WriteLine("====================================================");

        try
        {
            action();
            Console.WriteLine("結果：成功");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"結果：失敗 - {ex.Message}");
            Console.WriteLine("按 Enter 結束。");
            Console.ReadLine();
            Environment.Exit(1);
        }

        if (waitAfter)
        {
            Console.WriteLine("\n按 Enter 進入下一步...");
            Console.ReadLine();
        }
    }

    private static void WaitUntilAllAxisStop(RA605RobotApp robot, int timeoutSec)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        while (sw.Elapsed < TimeSpan.FromSeconds(timeoutSec))
        {
            var states = robot.MotorState;
            bool allStop = true;
            for (int i = 0; i < states.Length; i++)
            {
                if (states[i] != MotorState.STOP)
                {
                    allStop = false;
                    break;
                }
            }

            if (allStop) return;
            Thread.Sleep(100);
        }

        Console.WriteLine("警告：等待停止逾時，將繼續下一步。");
    }
}
