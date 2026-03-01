# Robot RA605 開發指南（上位機專用）

本專案已整理為「上位機可直接引用一個專案」的模式：

- 你只需要引用：`Robot.Motion.RA605`
- 你會用到的高階入口類別：`RA605RobotApp`
- 可用參數決定後端：`Real`（真實 EtherCAT）或 `Mock`（虛擬手臂）
- 可呼叫方法啟動 Web 監控頁：唯讀，不可控制手臂

---

## 1. 你要引用哪個專案

請在你的上位機專案（WinForms / WPF / Console / ASP.NET）中引用：

- `Robot.Motion.RA605.csproj`

這個專案會自動帶入底層相依：

- `Robot.Driver.Delta`
- `Robot.Core`

---

## 2. 核心概念（你只要記這三件事）

1. `RA605RobotApp` 是唯一入口（高階控制）
2. 建立物件時指定 `RobotBackendMode.Real` 或 `RobotBackendMode.Mock`
3. `StartWebMonitor()` 只做監看，不會控制手臂

---

## 3. 最小可用範例（可直接複製）

```csharp
using Robot.Motion.RA605;

class Program
{
    static void Main()
    {
        // 1) 建立高階控制器：Mock / Real 二選一
        using var robot = new RA605RobotApp(
            backendMode: RobotBackendMode.Mock, // 改成 Real 就連真實手臂
            zeroConfigPath: "axis_zero_config.json",
            toolLength: 60f
        );

        // 2) 連線
        if (!robot.Connect())
        {
            Console.WriteLine($"Connect 失敗，狀態：{robot.AxisCardState}");
            return;
        }

        // 3) 初始化
        if (!robot.Initialize())
        {
            Console.WriteLine("Initialize 失敗");
            return;
        }

        // 4) 啟動唯讀 Web 監控
        //    monitor.html 預設會自動找常見位置，也可手動指定路徑
        string url = robot.StartWebMonitor(port: 5850);
        Console.WriteLine($"Web 監控：{url}");

        // 5) 下達控制命令（範例：J1 到 30 度）
        robot.MoveAxisAbsolute(axis: 0, angleMdeg: 30000, constVel: 30000);

        Console.WriteLine("按 Enter 結束");
        Console.ReadLine();

        robot.End();
    }
}
```

---

## 4. Real / Mock 切換方式

建立 `RA605RobotApp` 時設定：

- `RobotBackendMode.Real`：使用 `EtherCAT_DLL_x64`（真實手臂）
- `RobotBackendMode.Mock`：使用 `EtherCAT_DLL_Mock`（虛擬手臂）

也就是說：

- 你的上位機控制程式可以完全不改，只換一個參數就切後端

---

## 5. Web 監控的重要變更（已符合你的需求）

目前 Web 頁面已改為唯讀監看：

- 已移除拉桿
- 已移除按鈕（回原點 / 清警報 / 急停）
- 監控頁不會送控制指令到手臂
- 只顯示狀態（角度、速度、末端姿態）

因此現在架構是：

- 虛擬/真實手臂負責接收控制命令並更新姿態
- Web 監控只讀取當前姿態來展示

---

## 6. `RA605RobotApp` 常用方法

生命週期：

- `Connect()`
- `Initialize()`
- `End()`

安全：

- `Estop()`
- `Ralm()`

軸控制：

- `MoveAxisAbsolute(axis, angleMdeg, constVel, tAcc, tDec)`
- `MoveAxisRelative(axis, deltaAngleMdeg, constVel, tAcc, tDec)`
- `MoveHome(constVel, tAcc, tDec)`

末端控制：

- `MoveToPosture(targetPosture, moveTimeMs)`
- `MoveRelativeEndEffector(...)`
- `StartContinuousMove(...)`
- `UpdateContinuousMove(...)`
- `StopContinuousMove()`

監控：

- `StartWebMonitor(port, htmlPath)`
- `StopWebMonitor()`

狀態讀取：

- `AxisCardState`
- `Pos`（mdeg）
- `Speed`（mdeg/s）
- `MotorState`
- `QueueLength`
- `EndEffectorPosition`
- `EndEffectorPosture`

---

## 7. 單位說明（非常重要）

- 角度：`mdeg`（千分之一度）
  - `1° = 1000 mdeg`
- 角速度：`mdeg/s`
- 距離：`mm`
- 時間：`s`（例如 `tAcc = 0.3`）

例如：45 度要輸入 `45000`。

---

## 8. 新手建議開發流程

1. 先用 `Mock` 把流程跑通
2. 確認 UI、狀態、例外處理都正常
3. 再切 `Real` 做低速單軸測試
4. 最後才做多軸、連續運動

---

## 9. 主要檔案位置

- 高階入口：`Robot.Motion.RA605/RA605RobotApp.cs`
- 後端模式列舉：`Robot.Motion.RA605/RobotBackendMode.cs`
- 驅動執行緒：`Robot.Driver.Delta/CommThread.cs`
- Real/Mock 轉接：`Robot.Driver.Delta/EtherCatApiAdapter.cs`
- 唯讀監控伺服器：`Robot.Driver.Delta/MonitorServer.cs`
- Web 監控頁：`Robot.MockConsole/monitor.html`

