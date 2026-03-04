# Robot RA605

RA605 六軸機械手臂控制框架（C# / .NET 8）。

本專案提供統一的高階控制入口，支援執行期切換實體與模擬後端，並提供唯讀 Web 監控能力。

## 1. 目標與範圍

- 提供可重用之手臂控制元件，供上位應用程式整合。
- 提供統一介面管理連線、初始化、安全控制與運動控制。
- 支援 `Real`（EtherCAT 實體）與 `Mock`（虛擬）後端切換。
- 提供唯讀 Web 監控頁顯示手臂狀態與末端姿態。

## 2. 專案結構

```text
Robot.sln
├─ Robot.Core
├─ Robot.Driver.Delta
├─ Robot.Motion.RA605
├─ Robot.MockConsole
├─ Demo.StepConsole
└─ axis_zero_config.json
```

- `Robot.Core`：核心列舉、介面、資料模型、日誌元件。
- `Robot.Driver.Delta`：EtherCAT 驅動層，含 Real/Mock 後端與監控伺服器。
- `Robot.Motion.RA605`：高階控制入口。
- `Robot.MockConsole`：Web 監控。
- `Demo.StepConsole`：逐步操作範例程式。

## 3. 高階控制入口

建議上位應用程式直接引用 `Robot.Motion.RA605`，並實作 `RA605RobotApp`。

### 3.1 後端模式

- `RobotBackendMode.Real`：使用 `EtherCAT_DLL_x64`（實體控制）。
- `RobotBackendMode.Mock`：使用 `EtherCAT_DLL_Mock`（模擬控制）。

### 3.2 主要能力

- 生命週期：`Connect()`、`Initialize()`、`End()`
- 安全控制：`Estop()`、`Ralm()`
- 軸控制：`MoveAxisAbsolute()`、`MoveAxisRelative()`、`MoveHome()`
- 末端控制：`MoveToPosture()`、`MoveRelativeEndEffector()`、`Start/Update/StopContinuousMove()`
- 監控：`StartWebMonitor()`、`StopWebMonitor()`

## 4. Web 監控

監控頁為只讀模式，僅顯示狀態資訊，不提供控制能力。

- 預設 URL：`http://localhost:5850`
- 顯示內容：軸角度、速度、狀態、佇列長度、末端位置與姿態矩陣
- 不接受手臂控制指令

## 5. 最小整合範例

```csharp
using Robot.Motion.RA605;

using var robot = new RA605RobotApp(
    backendMode: RobotBackendMode.Mock,
    zeroConfigPath: "axis_zero_config.json",
    toolLength: 60f);

if (!robot.Connect()) return;
if (!robot.Initialize()) return;

string monitorUrl = robot.StartWebMonitor(5850);
robot.MoveAxisAbsolute(axis: 0, angleMdeg: 30000, constVel: 30000);
robot.MoveHome();

robot.End();
```

## 6. 執行範例程式

```bash
dotnet run --project Demo.StepConsole/Demo.StepConsole.csproj
```

執行流程：

1. 連線模擬手臂
2. 初始化
3. 啟動 Web 監控
4. 第一軸移動
5. 回原點
6. 多軸同動
7. 回原點
8. 關閉手臂

## 7. 單位規範

- 角度：`mdeg`（1° = 1000 mdeg）
- 角速度：`mdeg/s`
- 長度：`mm`
- 時間：`s`（部分 API 另有 `ms` 參數）

## 8. 環境需求

- .NET SDK 8.0 或以上
- Windows（Real 模式）
- 可執行 .NET 8 的環境（Mock 模式）

## 9. 主要檔案

- `Robot.Motion.RA605/RA605RobotApp.cs`
- `Robot.Motion.RA605/RobotBackendMode.cs`
- `Robot.Driver.Delta/CommThread.cs`
- `Robot.Driver.Delta/EtherCatApiAdapter.cs`
- `Robot.Driver.Delta/MonitorServer.cs`
- `Robot.MockConsole/monitor.html`
- `Demo.StepConsole/Program.cs`
