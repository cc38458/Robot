# Robot — RA605 六軸機械臂控制系統

基於 Delta EtherCAT 軸卡的 C# .NET 8.0 六軸機械臂控制類別庫。

---

## 方案架構

```
Robot.sln
├── Robot.Core                 介面、列舉、資料結構（零依賴）
├── Robot.Driver.Delta         EtherCAT 驅動層（通訊線程、指令隊列、看門狗）
├── Robot.Motion.RA605         高階運動控制（正逆運動學、末端運動規劃）
└── axis_zero_config.json      零點設定檔
```

### 依賴關係

```
Robot.Motion.RA605 ──→ Robot.Core
Robot.Driver.Delta ──→ Robot.Core
```

三個專案之間 **Driver 與 Motion 無直接依賴**，透過 `IAxisCard` 介面解耦。

---

## 快速上手

### 1. 正式版（需硬體）

```csharp
using Robot.Core.Logging;
using Robot.Driver.Delta;
using Robot.Motion.RA605;

var log = new RobotLogger("logs", "Robot");
var driver = new DeltaDriver(log, "axis_zero_config.json");
var motion = new MotionController(driver, log, toolLength: 60f);

// 連線 → 初始化
driver.Start();
driver.Initial();

// 軸移動
motion.MoveAxisAbsolute(0, 90000, 30000, 0.5, 0.5);  // 軸0 → 90°

// 末端移動（齊次矩陣）
var target = RA605Kinematics.PostureFromXYZYPR(300, 0, 400, 0, 0, 0);
motion.MoveToPosture(target, 2000);

// 回原點
motion.MoveHome(20000, 0.5, 0.5);

// 關閉
driver.End();
driver.Dispose();
log.Dispose();
```

### 2. 模擬版（Mock，無硬體）

```bash
dotnet build -c Mock
```

或在 Visual Studio 中將組態切換為 `Mock`。

Mock 模式下所有 DLL API 呼叫會即時輸出到 Console，方便開發調試。

---

## 核心機制

### 線程模型

| 線程 | 職責 | 所在專案 |
|------|------|---------|
| 主線程 | 呼叫 IAxisCard / MotionController，持有 AxisCardState | 使用者程式 |
| 通訊線程 | 獨佔所有 DLL 呼叫、輪詢、隊列消化、安全處理 | Robot.Driver.Delta |
| 持續移動線程 | 末端持續相對移動的 PVT 控制迴圈 | Robot.Motion.RA605 |

### 狀態機

```
軸卡狀態：NULL → CONNING → CONNCET → READY ⇄ ALARM
                              ↓
                           CONNERR

單軸狀態：NULL → STOP ⇄ MOVING → ALARM
```

### 指令隊列

- 主線程呼叫 `driver.MoveAbsolute()` 等方法
- 通訊線程內部自己維護每軸獨立 `Queue<MotionCommand>`
- 通訊線程在輪詢迴圈中 dequeue 並執行
- 安全指令（Estop/Ralm）不進隊列，透過 volatile flag 最高優先處理
- 硬體 buffer 餘量 < 15 時暫停出隊
- 最後指令 endVel ≠ 0 且無後續 → 自動補 Sd_Stop

### 雙向看門狗

- 主線程每 200ms 更新心跳
- 通訊線程每輪詢週期檢查主線程心跳，逾時 2s → Sd_Stop 全軸 → ALARM
- 主線程檢查通訊線程心跳，斷連 → 等待硬體看門狗介入
- 觸發後需 `Ralm()` 復歸

### 零點校正

- `axis_zero_config.json` 儲存各軸絕對脈波零點
- `Initial()` 時讀取，對每軸呼叫 `Virtual_Set_Enable` + `Virtual_Set_Command`
- 設定後上層看到的 0° = 實際機械零點

---

## 運動模式

| # | 模式 | 輸入 | 方法 |
|---|------|------|------|
| 1 | 末端絕對姿態 | 齊次矩陣 + 時間(ms) | `MoveToPosture()` |
| 2 | 末端持續相對 | 速度向量 (mm/s, mdeg/s) | `StartContinuousMove()` |
| 3 | 末端一次性相對 | 位移量 (mm, mdeg) + 最大速度 | `MoveRelativeEndEffector()` |
| 4 | 軸絕對角度 | 目標角度(mdeg) + 速度 | `MoveAxisAbsolute()` |
| 5 | 軸相對角度 | 增量角度(mdeg) + 速度 | `MoveAxisRelative()` |
| 6 | 回原點 | 速度 + 加減速時間 | `MoveHome()` |

---

## 單位規範

| 物理量 | 單位 | 說明 |
|--------|------|------|
| 角度 | mdeg（千分之一度） | 1° = 1000 mdeg |
| 角速度 | mdeg/s | |
| 位移 | mm | 末端空間 |
| 線速度 | mm/s | 末端空間 |
| 時間 | 秒(s) 或 毫秒(ms) | 視 API 而定 |

---

## 檔案說明

### Robot.Core

| 檔案 | 說明 |
|------|------|
| `Enums/CardState.cs` | 軸卡整體狀態列舉 |
| `Enums/MotorState.cs` | 單軸狀態列舉 |
| `Enums/CommandType.cs` | 指令類型列舉 |
| `Interfaces/IAxisCard.cs` | 軸卡驅動介面 |
| `Interfaces/IMotionController.cs` | 高階運動控制介面 |
| `Models/MotionCommand.cs` | 指令隊列資料結構 |
| `Models/AxisZeroConfig.cs` | 零點校正設定模型 |
| `Logging/RobotLogger.cs` | 日誌系統（按日期分檔、繁中、非同步寫入） |

### Robot.Driver.Delta

| 檔案 | 說明 |
|------|------|
| `EtherCAT_DLL_x64.cs` | 原始 DLL P/Invoke（正式版） |
| `EtherCAT_DLL_Mock.cs` | 模擬 DLL（Console 即時輸出） |
| `CommThread.cs` | 通訊線程（輪詢、隊列、安全、看門狗） |
| `DeltaDriver.cs` | IAxisCard 實作（主線程端） |

### Robot.Motion.RA605

| 檔案 | 說明 |
|------|------|
| `RA605Kinematics.cs` | 正逆運動學（DH 參數、Z-Y-Z 歐拉分解） |
| `MotionController.cs` | IMotionController 實作（5 種運動模式） |

---

## RA605 DH 參數

| 參數 | 值(mm) | 說明 |
|------|--------|------|
| D1 | 375 | 基座高度 |
| A1 | 30 | 第 1 軸偏移 |
| A2 | 340 | 大臂長度 |
| A3_X | 40 | 小臂 X 偏移 |
| A3_Z | 338 | 小臂 Z 偏移 |
| D6 | 86.5 | 法蘭距離 |
| Tool | 60 | 工具頭長度（可設定） |

### 齒輪比

```
pulse2ang = { 29049, 35959, 29479, 29479, 28889, 18194 }
CSP_Set_Gear(CardNo, NodeID, 0, pulse2ang[i], 1000, 1)
```

設定後 DLL 所有位置/速度參數的單位為 mdeg / mdeg/s。
