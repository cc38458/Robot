using System.Net;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using Robot.Core.Enums;
using Robot.Core.Interfaces;
using Robot.Core.Logging;

namespace Robot.Driver.Delta
{
    /// <summary>
    /// 輕量 WebSocket 監控伺服器（嵌入式，零外部依賴）
    /// 功能：
    ///   1. 以 20Hz 頻率推送機械臂完整狀態（JSON）
    ///   2. 接收瀏覽器端指令（回原點、急停、單軸角度設定）
    ///   3. 提供 monitor.html 靜態檔案服務
    /// </summary>
    public class MonitorServer : IDisposable
    {
        private readonly IAxisCard _driver;
        private readonly RobotLogger _log;
        private readonly int _port;
        private readonly string? _htmlPath;

        private HttpListener? _listener;
        private CancellationTokenSource _cts = new();
        private readonly List<WebSocket> _clients = new();
        private readonly object _clientLock = new();
        private Thread? _broadcastThread;
        private bool _disposed;

        private const int BROADCAST_INTERVAL_MS = 50; // 20Hz

        /// <summary>
        /// 建立監控伺服器
        /// </summary>
        /// <param name="driver">軸卡驅動（通常為 Mock 模式的 DeltaDriver）</param>
        /// <param name="logger">日誌</param>
        /// <param name="port">HTTP/WebSocket 端口</param>
        /// <param name="htmlPath">monitor.html 檔案路徑（null 則不提供靜態檔案）</param>
        public MonitorServer(IAxisCard driver, RobotLogger logger,
                             int port = 5850, string? htmlPath = null)
        {
            _driver = driver;
            _log = logger;
            _port = port;
            _htmlPath = htmlPath;
        }

        /// <summary>啟動伺服器（非阻塞）</summary>
        public void Start()
        {
            _listener = new HttpListener();
            _listener.Prefixes.Add($"http://+:{_port}/");

            try
            {
                _listener.Start();
            }
            catch (HttpListenerException)
            {
                // 非管理員可能無法綁 +，改用 localhost
                _listener = new HttpListener();
                _listener.Prefixes.Add($"http://localhost:{_port}/");
                _listener.Start();
            }

            _log.Info($"監控伺服器啟動：http://localhost:{_port}/");
            _log.Info($"瀏覽器開啟上述網址即可檢視機械臂即時狀態");

            // 接收連線線程
            Task.Run(() => AcceptLoop(_cts.Token));

            // 廣播線程
            _broadcastThread = new Thread(BroadcastLoop)
            {
                Name = "MonitorBroadcast",
                IsBackground = true,
            };
            _broadcastThread.Start();
        }

        // ════════════════════════════════════════
        // HTTP 接收迴圈
        // ════════════════════════════════════════

        private async Task AcceptLoop(CancellationToken ct)
        {
            while (!ct.IsCancellationRequested && _listener!.IsListening)
            {
                try
                {
                    var ctx = await _listener.GetContextAsync();

                    if (ctx.Request.IsWebSocketRequest)
                    {
                        var wsCtx = await ctx.AcceptWebSocketAsync(null);
                        _log.Info($"WebSocket 客戶端已連線：{ctx.Request.RemoteEndPoint}");
                        _ = Task.Run(() => HandleWebSocket(wsCtx.WebSocket, ct));
                    }
                    else
                    {
                        // 靜態檔案服務
                        ServeHtml(ctx);
                    }
                }
                catch (ObjectDisposedException) { break; }
                catch (HttpListenerException) { break; }
                catch (Exception ex)
                {
                    _log.Error("監控伺服器接收錯誤", ex);
                }
            }
        }

        // ════════════════════════════════════════
        // 靜態 HTML 服務
        // ════════════════════════════════════════

        private void ServeHtml(HttpListenerContext ctx)
        {
            var resp = ctx.Response;
            var reqPath = ctx.Request.Url?.AbsolutePath ?? "/";
            if (reqPath == "/") reqPath = "/monitor.html";

            // 嘗試從 _htmlPath 的同級目錄提供靜態檔案
            string? baseDir = _htmlPath != null ? Path.GetDirectoryName(Path.GetFullPath(_htmlPath)) : null;

            // 正規化路徑分隔符（URL 用 /，Windows 用 \）
            var relativePath = reqPath.TrimStart('/').Replace('/', Path.DirectorySeparatorChar);
            string? filePath = baseDir != null ? Path.Combine(baseDir, relativePath) : null;

            _log.Debug($"[HTTP] 請求：{reqPath} → {filePath ?? "null"}");

            // 安全：禁止目錄遍歷
            if (filePath != null && baseDir != null &&
                Path.GetFullPath(filePath).StartsWith(Path.GetFullPath(baseDir)) &&
                File.Exists(filePath))
            {
                var ext = Path.GetExtension(filePath).ToLowerInvariant();
                resp.ContentType = ext switch
                {
                    ".html" => "text/html; charset=utf-8",
                    ".js" => "application/javascript",
                    ".css" => "text/css",
                    ".json" => "application/json",
                    ".stl" => "application/octet-stream",
                    ".urdf" => "application/xml",
                    ".png" => "image/png",
                    ".jpg" or ".jpeg" => "image/jpeg",
                    _ => "application/octet-stream",
                };

                // CORS 允許（開發用）
                resp.Headers.Add("Access-Control-Allow-Origin", "*");

                var data = File.ReadAllBytes(filePath);
                resp.ContentLength64 = data.Length;
                resp.StatusCode = 200;
                resp.OutputStream.Write(data, 0, data.Length);
                resp.OutputStream.Close();
            }
            else
            {
                // Fallback
                var msg = "<!DOCTYPE html><html><body style='background:#111;color:#eee;font-family:monospace;padding:40px'>"
                        + "<h2>RA605 Monitor Server — 404</h2>"
                        + "<p>WebSocket: ws://localhost:" + _port + "/</p>"
                        + $"<p>Requested: {reqPath}</p>"
                        + $"<p>Resolved: {filePath ?? "null"}</p>"
                        + $"<p>BaseDir: {baseDir ?? "null"}</p>"
                        + "<p>Place monitor.html + meshes/ in the same directory.</p>"
                        + "</body></html>";
                var data = Encoding.UTF8.GetBytes(msg);
                resp.ContentType = "text/html; charset=utf-8";
                resp.ContentLength64 = data.Length;
                resp.StatusCode = 404;
                resp.OutputStream.Write(data, 0, data.Length);
                resp.OutputStream.Close();
            }
        }

        // ════════════════════════════════════════
        // WebSocket 處理
        // ════════════════════════════════════════

        private async Task HandleWebSocket(WebSocket socket, CancellationToken ct)
        {
            lock (_clientLock) { _clients.Add(socket); }

            var buffer = new byte[4096];
            try
            {
                while (socket.State == WebSocketState.Open && !ct.IsCancellationRequested)
                {
                    var result = await socket.ReceiveAsync(
                        new ArraySegment<byte>(buffer), ct);

                    if (result.MessageType == WebSocketMessageType.Close)
                        break;

                    if (result.MessageType == WebSocketMessageType.Text)
                    {
                        var msg = Encoding.UTF8.GetString(buffer, 0, result.Count);
                        HandleCommand(msg);
                    }
                }
            }
            catch (WebSocketException) { }
            catch (OperationCanceledException) { }
            finally
            {
                lock (_clientLock) { _clients.Remove(socket); }
                try { socket.Dispose(); } catch { }
                _log.Info("WebSocket 客戶端已斷線");
            }
        }

        /// <summary>處理來自瀏覽器的指令</summary>
        private void HandleCommand(string json)
        {
            try
            {
                using var doc = JsonDocument.Parse(json);
                var root = doc.RootElement;
                var cmd = root.GetProperty("cmd").GetString();

                switch (cmd)
                {
                    case "home":
                        _log.Info("[監控] 收到回原點指令");
                        _driver.MoveHome(20000, 0.5, 0.5);
                        break;

                    case "estop":
                        _log.Warn("[監控] 收到緊急停止指令");
                        _driver.Estop();
                        break;

                    case "ralm":
                        _log.Info("[監控] 收到警報復歸指令");
                        _driver.Ralm();
                        break;

                    case "setAngle":
                        {
                            var axis = (ushort)root.GetProperty("axis").GetInt32();
                            var angle = root.GetProperty("angle").GetDouble();
                            var angleMdeg = (int)(angle * 1000);
                            _log.Debug($"[監控] 設定軸 {axis} → {angle:F1}°");
                            _driver.MoveAbsolute(axis, angleMdeg, 0, 30000, 0, 0.3, 0.3);
                        }
                        break;

                    case "moveRelative":
                        {
                            var axis = (ushort)root.GetProperty("axis").GetInt32();
                            var delta = root.GetProperty("delta").GetDouble();
                            var deltaMdeg = (int)(delta * 1000);
                            _driver.MoveRelative(axis, deltaMdeg, 0, 20000, 0, 0.3, 0.3);
                        }
                        break;
                }
            }
            catch (Exception ex)
            {
                _log.Error($"[監控] 指令解析失敗：{ex.Message}");
            }
        }

        // ════════════════════════════════════════
        // 狀態廣播
        // ════════════════════════════════════════

        private void BroadcastLoop()
        {
            while (!_cts.Token.IsCancellationRequested)
            {
                try
                {
                    var json = BuildStateJson();
                    var bytes = Encoding.UTF8.GetBytes(json);
                    var segment = new ArraySegment<byte>(bytes);

                    WebSocket[] snapshot;
                    lock (_clientLock) { snapshot = _clients.ToArray(); }

                    foreach (var ws in snapshot)
                    {
                        if (ws.State == WebSocketState.Open)
                        {
                            try
                            {
                                ws.SendAsync(segment, WebSocketMessageType.Text, true, _cts.Token)
                                  .GetAwaiter().GetResult();
                            }
                            catch
                            {
                                lock (_clientLock) { _clients.Remove(ws); }
                            }
                        }
                    }
                }
                catch (Exception ex)
                {
                    if (!_cts.Token.IsCancellationRequested)
                        _log.Error("廣播錯誤", ex);
                }

                Thread.Sleep(BROADCAST_INTERVAL_MS);
            }
        }

        private string BuildStateJson()
        {
            var pos = _driver.Pos;
            var speed = _driver.Speed;
            var motorState = _driver.State;
            var queueLen = _driver.QueueLength;

            return JsonSerializer.Serialize(new
            {
                type = "state",
                cardState = (int)_driver.AxisCardState,
                pos = pos,                                    // mdeg
                speed = speed,                                // mdeg/s
                motorState = motorState.Select(s => (int)s).ToArray(),
                queueLen = queueLen,
                timestamp = DateTimeOffset.Now.ToUnixTimeMilliseconds(),
            });
        }

        // ════════════════════════════════════════

        /// <summary>傳送日誌訊息到所有連線的瀏覽器</summary>
        public void PushLog(string level, string message)
        {
            var json = JsonSerializer.Serialize(new
            {
                type = "log",
                level,
                message,
                timestamp = DateTimeOffset.Now.ToUnixTimeMilliseconds(),
            });

            var bytes = Encoding.UTF8.GetBytes(json);
            var segment = new ArraySegment<byte>(bytes);

            WebSocket[] snapshot;
            lock (_clientLock) { snapshot = _clients.ToArray(); }

            foreach (var ws in snapshot)
            {
                if (ws.State == WebSocketState.Open)
                {
                    try { ws.SendAsync(segment, WebSocketMessageType.Text, true, CancellationToken.None).Wait(); }
                    catch { }
                }
            }
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            _cts.Cancel();
            _broadcastThread?.Join(2000);

            lock (_clientLock)
            {
                foreach (var ws in _clients)
                    try { ws.Dispose(); } catch { }
                _clients.Clear();
            }

            _listener?.Stop();
            _listener?.Close();
            _cts.Dispose();
        }
    }
}
