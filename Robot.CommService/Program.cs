using System.IO.Pipes;
using Robot.Core.Enums;
using Robot.Core.IPC;
using Robot.Core.Logging;
using Robot.Driver.Delta;

namespace Robot.CommService
{
    /// <summary>
    /// 獨立行程 EtherCAT 通訊服務。
    /// 職責：
    ///   1. 擁有 DeltaDriver（內含 CommThread），獨佔 EtherCAT 連線
    ///   2. 透過命名管道接收主程式的 IAxisCard 指令
    ///   3. 透過共享記憶體以 10ms 間隔推送狀態 + 心跳
    ///   4. 主程式心跳逾時（1s）→ 全軸停止 → Servo OFF → 關閉連線 → 終止
    ///   5. 管道斷線 → 同上安全關機程序
    /// </summary>
    internal class Program
    {
        private const int STATUS_PUSH_INTERVAL_MS = 10;
        private const int HEARTBEAT_CHECK_INTERVAL_MS = 500;
        private const double MAIN_HEARTBEAT_TIMEOUT_SEC = 1.0;
        private const int AXIS_COUNT = 6;

        private static volatile bool _running = true;
        private static DeltaDriver? _driver;
        private static SharedMemoryState? _shm;
        private static RobotLogger? _log;
        private static int _shutdownStarted;

        static int Main(string[] args)
        {
            // 解析引數：--mock --zero-config=path --shm-path=path
            bool useMock = args.Contains("--mock");
            string zeroConfig = GetArg(args, "--zero-config") ?? "axis_zero_config.json";
            string shmPath = GetArg(args, "--shm-path")
                ?? Path.Combine(Path.GetTempPath(), SharedMemoryState.FILE_NAME);

            _log = new RobotLogger("CommService");

            _log.Info("═══════════════════════════════════════════");
            _log.Info("  Robot CommService 啟動");
            _log.Info($"  Mock={useMock}, ZeroConfig={zeroConfig}");
            _log.Info($"  SHM={shmPath}");
            _log.Info("═══════════════════════════════════════════");

            AppDomain.CurrentDomain.ProcessExit += (_, _) =>
            {
                SafeShutdown("ProcessExit");
            };
            Console.CancelKeyPress += (_, e) =>
            {
                e.Cancel = true;
                _running = false;
                SafeShutdown("CancelKeyPress");
            };
            AppDomain.CurrentDomain.UnhandledException += (_, _) =>
            {
                SafeShutdown("UnhandledException");
            };

            try
            {
                _shm = new SharedMemoryState(shmPath);
                _driver = new DeltaDriver(_log, zeroConfig, useMock);

                // 啟動狀態推送線程
                var statusThread = new Thread(StatusPushLoop)
                { IsBackground = true, Name = "StatusPush" };
                statusThread.Start();

                // 啟動心跳監控線程
                var heartbeatThread = new Thread(HeartbeatMonitorLoop)
                { IsBackground = true, Name = "HeartbeatMonitor" };
                heartbeatThread.Start();

                // 主線程：管道伺服器迴圈
                PipeServerLoop();
            }
            catch (Exception ex)
            {
                _log?.Fatal("CommService 致命錯誤", ex);
            }
            finally
            {
                SafeShutdown("程式結束");
                _driver?.Dispose();
                _shm?.Dispose();
                _log?.Info("CommService 已終止");
            }

            return 0;
        }

        /// <summary>管道伺服器迴圈：等待連線 → 處理指令 → 斷線後重新等待。</summary>
        private static void PipeServerLoop()
        {
            while (_running)
            {
                _log!.Info("等待主程式透過命名管道連線...");

                using var pipeServer = new NamedPipeServerStream(
                    PipeProtocol.PIPE_NAME,
                    PipeDirection.InOut,
                    1, // 最多一個連線
                    PipeTransmissionMode.Byte,
                    PipeOptions.Asynchronous);

                try
                {
                    pipeServer.WaitForConnection();
                    _log.Info("主程式已連線");

                    using var reader = new StreamReader(pipeServer, leaveOpen: true);
                    using var writer = new StreamWriter(pipeServer, leaveOpen: true)
                    { AutoFlush = true };

                    while (_running && pipeServer.IsConnected)
                    {
                        string? line = reader.ReadLine();
                        if (line == null) break; // 管道斷線

                        PipeResponse response;
                        try
                        {
                            var request = PipeProtocol.Deserialize<PipeRequest>(line);
                            if (request == null)
                            {
                                response = new PipeResponse { Ok = false, Error = "無法解析請求" };
                            }
                            else
                            {
                                response = HandleRequest(request);
                            }
                        }
                        catch (Exception ex)
                        {
                            response = new PipeResponse { Ok = false, Error = ex.Message };
                        }

                        writer.WriteLine(PipeProtocol.Serialize(response));
                    }
                }
                catch (IOException)
                {
                    _log.Warn("管道 IO 錯誤（主程式可能已斷線）");
                }
                catch (Exception ex)
                {
                    if (_running)
                        _log.Error("管道伺服器錯誤", ex);
                }

                if (_running)
                {
                    _log.Warn("主程式已斷線，執行安全關機程序...");
                    SafeShutdown("管道斷線");

                    // 管道斷線後終止 CommService
                    _running = false;
                }
            }
        }

        /// <summary>處理來自管道的單一請求。</summary>
        private static PipeResponse HandleRequest(PipeRequest req)
        {
            var resp = new PipeResponse { Id = req.Id };

            try
            {
                bool result = req.Cmd switch
                {
                    "Start" => _driver!.Start(),
                    "End" => _driver!.End(),
                    "Initial" => _driver!.Initial(),
                    "Estop" => _driver!.Estop(),
                    "Ralm" => _driver!.Ralm(),
                    "CalibrateZero" => _driver!.CalibrateZero(),
                    "Stop" => _driver!.Stop(req.Axis ?? 0, req.TDec ?? 0.5),
                    "ChangeVelocity" => _driver!.ChangeVelocity(
                        req.Axis ?? 0, req.NewSpeed ?? 0, req.TSec ?? 1.0),
                    "MoveAbsolute" => _driver!.MoveAbsolute(
                        req.Axis ?? 0, req.Dist ?? 0, req.StrVel ?? 0,
                        req.ConstVel ?? 0, req.EndVel ?? 0,
                        req.TAcc ?? 0, req.TDec ?? 0),
                    "MoveRelative" => _driver!.MoveRelative(
                        req.Axis ?? 0, req.Dist ?? 0, req.StrVel ?? 0,
                        req.ConstVel ?? 0, req.EndVel ?? 0,
                        req.TAcc ?? 0, req.TDec ?? 0),
                    "MovePV" => _driver!.MovePV(
                        req.Axis ?? 0, req.StrVel ?? 0, req.ConstVel ?? 0, req.TAcc ?? 0),
                    "MovePT" => _driver!.MovePT(
                        req.Axis ?? 0, req.DataCnt ?? 0,
                        req.TargetPos ?? Array.Empty<int>(),
                        req.TargetTime ?? Array.Empty<int>(),
                        req.StrVel ?? 0, req.EndVel ?? 0),
                    "MoveMultiAxisPVT" => _driver!.MoveMultiAxisPVT(
                        req.MultiDataCount ?? Array.Empty<int>(),
                        req.MultiTargetPos ?? Array.Empty<int[]>(),
                        req.MultiTargetTime ?? Array.Empty<int[]>(),
                        req.MultiStrVel ?? Array.Empty<int>(),
                        req.MultiEndVel ?? Array.Empty<int>()),
                    "MoveHome" => _driver!.MoveHome(
                        req.ConstVel ?? 0, req.TAcc ?? 0, req.TDec ?? 0),
                    "AbortAndChangePosition" => _driver!.AbortAndChangePosition(
                        req.MultiDist ?? Array.Empty<int>(), req.TDec ?? 0.3),
                    _ => throw new ArgumentException($"未知指令：{req.Cmd}"),
                };

                resp.Ok = result;
                if (!result)
                    resp.Error = $"{req.Cmd} 返回 false（前置條件不符或執行失敗）";
            }
            catch (Exception ex)
            {
                resp.Ok = false;
                resp.Error = ex.Message;
                _log!.Error($"處理指令 {req.Cmd} 異常", ex);
            }

            return resp;
        }

        /// <summary>
        /// 狀態推送迴圈：每 10ms 將 DeltaDriver 狀態寫入共享記憶體 + 更新心跳。
        /// </summary>
        private static void StatusPushLoop()
        {
            var pos = new int[AXIS_COUNT];
            var speed = new int[AXIS_COUNT];
            var motorState = new MotorState[AXIS_COUNT];
            var queueLen = new int[AXIS_COUNT];

            while (_running)
            {
                try
                {
                    if (_driver != null && _shm != null)
                    {
                        var driverPos = _driver.Pos;
                        var driverSpeed = _driver.Speed;
                        var driverState = _driver.State;
                        var driverQueue = _driver.QueueLength;
                        var cardState = _driver.AxisCardState;

                        Array.Copy(driverPos, pos, AXIS_COUNT);
                        Array.Copy(driverSpeed, speed, AXIS_COUNT);
                        Array.Copy(driverQueue, queueLen, AXIS_COUNT);
                        for (int i = 0; i < AXIS_COUNT; i++)
                            motorState[i] = driverState[i];

                        _shm.WriteState(cardState, pos, speed, motorState, queueLen);
                        _shm.WriteCommHeartbeat();
                    }
                }
                catch (Exception ex)
                {
                    _log?.Error("狀態推送錯誤", ex);
                }

                Thread.Sleep(STATUS_PUSH_INTERVAL_MS);
            }
        }

        /// <summary>
        /// 心跳監控迴圈：每 500ms 檢查主程式心跳，逾時 1s 觸發安全關機。
        /// </summary>
        private static void HeartbeatMonitorLoop()
        {
            // 等待第一次心跳寫入
            Thread.Sleep(2000);

            while (_running)
            {
                try
                {
                    if (_shm != null && _shm.IsMainHeartbeatTimeout(MAIN_HEARTBEAT_TIMEOUT_SEC))
                    {
                        _log!.Fatal($"主程式心跳逾時（>{MAIN_HEARTBEAT_TIMEOUT_SEC}s），觸發安全關機！");
                        _running = false;
                        SafeShutdown("主程式心跳逾時");
                        break;
                    }
                }
                catch (Exception ex)
                {
                    _log?.Error("心跳監控錯誤", ex);
                }

                Thread.Sleep(HEARTBEAT_CHECK_INTERVAL_MS);
            }
        }

        /// <summary>
        /// 安全關機程序：全軸停止 → Servo OFF → 關閉 EtherCAT → 結束。
        /// </summary>
        private static void SafeShutdown(string reason)
        {
            if (Interlocked.Exchange(ref _shutdownStarted, 1) != 0) return;
            if (_driver == null) return;

            _log!.Warn($"安全關機啟動（原因：{reason}）");

            try
            {
                // Step 1: 緊急停止所有軸
                _log.Info("Step 1/3: 全軸緊急停止...");
                _driver.Estop();
                Thread.Sleep(500);

                // Step 2: Servo OFF + 關閉連線
                _log.Info("Step 2/3: Servo OFF + 關閉 EtherCAT 連線...");
                _driver.End();
                Thread.Sleep(300);

                // Step 3: 釋放資源
                _log.Info("Step 3/3: 釋放資源...");
                _driver.Dispose();
                _driver = null;

                _log.Info("安全關機完成");
            }
            catch (Exception ex)
            {
                _log.Error("安全關機過程中發生錯誤", ex);
            }
        }

        /// <summary>從命令列引數解析 --key=value 形式的值。</summary>
        private static string? GetArg(string[] args, string key)
        {
            var prefix = key + "=";
            var match = args.FirstOrDefault(a => a.StartsWith(prefix, StringComparison.OrdinalIgnoreCase));
            return match?.Substring(prefix.Length);
        }
    }
}
