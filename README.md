# Robot

RA605 六軸機械臂控制專案（C# / .NET 8），包含核心介面、Delta EtherCAT 驅動、運動控制，以及可直接啟動的 Mock 主控台與 3D 監控頁面。

## 專案結構

```text
Robot/
├── Robot.sln                                    方案檔（Debug / Release / Mock 三種組態）
├── axis_zero_config.json                        零點設定檔（各軸脈波偏移 + 齒輪比）
├── README.md                                    專案文件（繁中）
│
├── Robot.Core/                                  ◆ 核心介面層（零依賴）
│   ├── Robot.Core.csproj
│   ├── Enums/
│   │   ├── CardState.cs                            軸卡狀態 NULL→CONNING→CONNCET→READY→ALARM
│   │   ├── MotorState.cs                           單軸狀態 NULL/STOP/MOVING/ALARM
│   │   └── CommandType.cs                          指令類型 MoveAbsolute/Relative/PV/PT/Stop/MultiAxisPVT
│   ├── Interfaces/
│   │   ├── IAxisCard.cs                            軸卡驅動介面（連線/初始化/安全/運動）
│   │   └── IMotionController.cs                    高階運動控制介面（末端姿態/持續移動/軸控）
│   ├── Models/
│   │   ├── MotionCommand.cs                        指令隊列資料結構（含 Barrier 同步、多軸 PVT）
│   │   └── AxisZeroConfig.cs                       零點校正設定（JSON 序列化）
│   └── Logging/
│       └── RobotLogger.cs                          日誌系統（繁中、按日期分檔、非同步寫入）
│
├── Robot.Driver.Delta/                          ◆ 硬體驅動層（通訊線程 + 指令隊列 + 看門狗）
│   ├── Robot.Driver.Delta.csproj                   條件編譯：MOCK ↔ 正式 DLL 互斥
│   ├── EtherCAT_DLL_x64.cs                        原廠 DLL P/Invoke（578 API，1181 行）
│   ├── EtherCAT_DLL_Mock.cs                        模擬 DLL（Console 即時輸出，264 行）
│   ├── CommThread.cs                               通訊線程（輪詢/隊列消化/安全優先/看門狗，800 行）
│   ├── DeltaDriver.cs                              IAxisCard 實作（主線程端，354 行）
│   └── MonitorServer.cs                            WebSocket 監控伺服器（20Hz 推送 + 靜態檔案服務）
│
├── Robot.Motion.RA605/                          ◆ 高階運動控制層（正逆運動學 + 末端運動規劃）
│   ├── Robot.Motion.RA605.csproj
│   ├── RA605Kinematics.cs                          正逆運動學（DH 參數、Z-Y-Z 歐拉分解，216 行）
│   └── MotionController.cs                         IMotionController 實作（5 種運動模式，439 行）
│
└── Robot.MockConsole/                           ◆ Mock 模擬主控台（含 3D 瀏覽器監控）
    ├── Robot.MockConsole.csproj                    永遠定義 MOCK，自動複製 meshes/ 到輸出
    ├── Program.cs                                  入口點（連線→初始化→啟動 WS Server→互動 Console）
    ├── monitor.html                                3D 監控介面（Three.js + STL Loader + URDF 骨架）
    ├── RA605_SLDASM.urdf                           URDF 機械臂描述檔
    └── meshes/                                     SolidWorks 匯出 STL 模型（共 4.5MB）
        ├── base_link.STL                              基座（2.4MB）
        ├── Link1.STL                                  J1 連桿
        ├── Link2.STL                                  J2 大臂
        ├── Link3.STL                                  J3 小臂
        ├── Link4.STL                                  J4 手腕
        ├── Link5.STL                                  J5 手腕
        └── Link6.STL                                  J6 法蘭
```

## 相依關係

- `Robot.Driver.Delta -> Robot.Core`
- `Robot.Motion.RA605 -> Robot.Core`
- `Robot.MockConsole -> Robot.Core + Robot.Driver.Delta(UseMock=true)`

`Driver` 與 `Motion` 透過 `IAxisCard` 解耦，沒有直接專案相依。

## 環境需求

- .NET SDK 8.0+
- Windows（正式 EtherCAT 模式）
- 任意可跑 .NET 8 的環境（Mock 模式）

## 建置與執行

### 1. 建置整個方案

```bash
dotnet build Robot.sln
```

### 2. Mock 模式建置（不需硬體）

```bash
dotnet build Robot.sln -c Mock
```

### 3. 啟動 Mock 主控台 + 3D 監控

```bash
dotnet run --project Robot.MockConsole/Robot.MockConsole.csproj -c Mock
```

啟動後瀏覽器開啟：`http://localhost:5850`

## Mock 主控台指令

- `status` / `s`: 顯示各軸狀態
- `move <軸號0-5> <角度>`: 指定軸移動（角度單位：度）
- `home`: 全軸回原點
- `estop`: 緊急停止
- `ralm`: 清除警報
- `alarm <軸號0-5>`: 模擬觸發警報（僅 Mock）
- `quit` / `q`: 結束程式

## 單位與慣例

- 角度：`mdeg`（1 度 = 1000 mdeg）
- 角速度：`mdeg/s`
- 線位移：`mm`
- 線速度：`mm/s`
- 時間：`s` 或 `ms`（依 API）

## 主要檔案

- `Robot.Core/Interfaces/IAxisCard.cs`: 軸卡驅動抽象
- `Robot.Core/Interfaces/IMotionController.cs`: 運動控制抽象
- `Robot.Driver.Delta/DeltaDriver.cs`: 驅動實作入口
- `Robot.Driver.Delta/CommThread.cs`: 通訊執行緒、佇列、安全與看門狗
- `Robot.Motion.RA605/RA605Kinematics.cs`: 正逆運動學
- `Robot.Motion.RA605/MotionController.cs`: 高階運動控制
- `Robot.MockConsole/Program.cs`: Mock 測試入口
- `Robot.MockConsole/MonitorServer.cs`: 監控 WebSocket 伺服器

## 設定檔

- `axis_zero_config.json`: 軸零點脈波設定；`Initial()` 會載入套用。

## 已整理項目

- 舊版說明文件已備份為 `README_old`
- 已新增 `.gitignore`（忽略 `bin/`、`obj/`、`.vs/`、`logs/`）
- 已清理可刪除的建置產物與日誌（部分 `.vs` 索引檔案受系統權限保護，無法在目前權限下刪除）
