# Robot

本專案為 RA605 機械手臂控制解決方案，提供高階運動控制、正逆運動學、末端執行器移動規劃，以及獨立的通訊服務流程。專案同時支援實機與 Mock 模式，方便開發、測試與示範驗證。

## 專案組成

- `Robot.Motion.RA605`：RA605 高階控制入口，封裝運動控制、末端位姿計算與 Web 監控啟動能力。
- `Robot.CommService`：獨立行程通訊服務，負責 EtherCAT 驅動、命名管道收發與共享記憶體狀態推送。
- `Robot.Driver.Delta`：底層驅動層，提供對 Delta 軸卡的控制實作。
- `Robot.Core`：共用列舉、介面、IPC 結構與日誌元件。
- `Demo.StepConsole`：示範如何以逐步流程操作 RA605 手臂。

## 必要專案參考

若要在其他應用程式中使用本專案功能，請明確加入以下專案參考：

- `Robot.Motion.RA605`
- `Robot.CommService`

例如在 `.csproj` 中加入：

```xml
<ItemGroup>
  <ProjectReference Include="..\Robot.Motion.RA605\Robot.Motion.RA605.csproj" />
  <ProjectReference Include="..\Robot.CommService\Robot.CommService.csproj" />
</ItemGroup>
```

其中：

- `Robot.Motion.RA605` 提供上位程式直接使用的 `RA605RobotApp` 高階 API。
- `Robot.CommService` 提供獨立通訊服務，供控制流程以 out-of-process 方式與底層驅動互動。

## 使用方式

建議由 `RA605RobotApp` 作為主要入口，依序執行：

1. `Connect()`
2. `Initialize()`
3. 執行單軸或末端運動控制
4. `End()`

若要查看基本操作範例，可參考 `Demo.StepConsole`：

```bash
dotnet run --project Demo.StepConsole/Demo.StepConsole.csproj
```

## 開發環境

- .NET 8
- Visual Studio 2022 或相容的 .NET 開發工具

