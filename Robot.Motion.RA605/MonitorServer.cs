using System.Net;
using System.Net.WebSockets;
using System.Reflection;
using System.Text;
using System.Text.Json;
using Robot.Core.Enums;
using Robot.Core.Interfaces;
using Robot.Core.Logging;

namespace Robot.Motion.RA605
{
    /// <summary>
    /// 唯讀 WebSocket 監控伺服器（嵌入式，零外部依賴）
    /// 功能：
    ///   1. 以 20Hz 頻率推送機械臂完整狀態（JSON），包含末端位姿
    ///   2. 提供 monitor.html 靜態檔案服務
    /// 注意：
    ///   - 本伺服器僅監看，不接受任何控制命令
    /// </summary>
    public class MonitorServer : IDisposable
    {
        private readonly IMotionController _controller;
        private readonly RobotLogger _log;
        private readonly int _port;
        private readonly string? _htmlPath;

        private HttpListener? _listener;
        private CancellationTokenSource _cts = new();
        private readonly List<WebSocket> _clients = new();
        private readonly object _clientLock = new();
        private Task? _broadcastTask;
        private Task? _acceptTask;
        private bool _disposed;

        private const int BROADCAST_INTERVAL_MS = 50; // 20Hz
        private const int SEND_TIMEOUT_MS = 1000;     // 單一客戶端發送逾時

        /// <summary>
        /// 建立監控伺服器實例。
        /// </summary>
        /// <param name="controller">運動控制器（提供狀態資料）。</param>
        /// <param name="logger">日誌記錄器。</param>
        /// <param name="port">HTTP 監聽端口。</param>
        /// <param name="htmlPath">monitor.html 磁碟路徑；null 時使用嵌入式資源。</param>
        public MonitorServer(IMotionController controller, RobotLogger logger,
                             int port = 5850, string? htmlPath = null)
        {
            _controller = controller;
            _log = logger;
            _port = port;
            _htmlPath = htmlPath;
        }

        /// <summary>啟動 HTTP 監聽、WebSocket 接收迴圈與狀態廣播迴圈。</summary>
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
                _listener = new HttpListener();
                _listener.Prefixes.Add($"http://localhost:{_port}/");
                _listener.Start();
            }

            _log.Info($"監控伺服器啟動：http://localhost:{_port}/");
            _log.Info("Web 監控為唯讀模式（不接受控制命令）");

            _acceptTask = Task.Run(() => AcceptLoop(_cts.Token));
            _broadcastTask = Task.Run(() => BroadcastLoopAsync());
        }

        /// <summary>HTTP 請求接收迴圈：分派 WebSocket 升級或靜態檔案服務。</summary>
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

        /// <summary>處理非 WebSocket 的 HTTP 請求：依路徑提供磁碟檔案或嵌入式資源。</summary>
        private void ServeHtml(HttpListenerContext ctx)
        {
            var resp = ctx.Response;
            var reqPath = ctx.Request.Url?.AbsolutePath ?? "/";
            if (reqPath == "/") reqPath = "/monitor.html";

            // 嘗試 1：從磁碟檔案提供（使用者有傳入 htmlPath 時）
            string? baseDir = _htmlPath != null ? Path.GetDirectoryName(Path.GetFullPath(_htmlPath)) : null;
            var relativePath = reqPath.TrimStart('/').Replace('/', Path.DirectorySeparatorChar);
            string? filePath = baseDir != null ? Path.Combine(baseDir, relativePath) : null;

            if (filePath != null && baseDir != null &&
                Path.GetFullPath(filePath).StartsWith(Path.GetFullPath(baseDir), StringComparison.OrdinalIgnoreCase) &&
                File.Exists(filePath))
            {
                ServeBytes(resp, File.ReadAllBytes(filePath), GetContentType(filePath));
                return;
            }

            // 嘗試 2：從 DLL 嵌入式資源提供
            var resourceName = reqPath.TrimStart('/');
            var stream = GetEmbeddedResource(resourceName);
            if (stream != null)
            {
                using (stream)
                {
                    var data = new byte[stream.Length];
                    stream.ReadExactly(data, 0, data.Length);
                    ServeBytes(resp, data, GetContentType(resourceName));
                }
                return;
            }

            // 404
            var msg = "<!DOCTYPE html><html><body style='background:#111;color:#eee;font-family:monospace;padding:40px'>"
                    + "<h2>RA605 Monitor Server — 404</h2>"
                    + "<p>WebSocket: ws://localhost:" + _port + "/</p>"
                    + $"<p>Requested: {reqPath}</p>"
                    + "</body></html>";
            var msgData = Encoding.UTF8.GetBytes(msg);
            resp.ContentType = "text/html; charset=utf-8";
            resp.ContentLength64 = msgData.Length;
            resp.StatusCode = 404;
            resp.OutputStream.Write(msgData, 0, msgData.Length);
            resp.OutputStream.Close();
        }

        /// <summary>將位元組陣列寫入 HTTP 回應。</summary>
        private static void ServeBytes(HttpListenerResponse resp, byte[] data, string contentType)
        {
            resp.ContentType = contentType;
            resp.Headers.Add("Access-Control-Allow-Origin", "*");
            resp.ContentLength64 = data.Length;
            resp.StatusCode = 200;
            resp.OutputStream.Write(data, 0, data.Length);
            resp.OutputStream.Close();
        }

        /// <summary>查詢嵌入式資源，先以短名稱查，備援以命名空間前綴查。</summary>
        private static Stream? GetEmbeddedResource(string name)
        {
            var asm = Assembly.GetExecutingAssembly();
            return asm.GetManifestResourceStream(name)
                ?? asm.GetManifestResourceStream($"Robot.Motion.RA605.{name}");
        }

        /// <summary>依副檔名回傳對應的 MIME Content-Type。</summary>
        private static string GetContentType(string path)
        {
            var ext = Path.GetExtension(path).ToLowerInvariant();
            return ext switch
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
        }

        /// <summary>處理單一 WebSocket 客戶端：維持連線直到關閉或取消（唯讀，忽略收到的訊息）。</summary>
        private async Task HandleWebSocket(WebSocket socket, CancellationToken ct)
        {
            lock (_clientLock) { _clients.Add(socket); }

            var buffer = new byte[1024];
            try
            {
                while (socket.State == WebSocketState.Open && !ct.IsCancellationRequested)
                {
                    var result = await socket.ReceiveAsync(new ArraySegment<byte>(buffer), ct);
                    if (result.MessageType == WebSocketMessageType.Close)
                        break;
                    // 唯讀模式：忽略來自瀏覽器的文字訊息
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

        /// <summary>
        /// 非同步廣播迴圈（取代原本的 sync-over-async 模式）。
        /// 使用 Task.WhenAll 並行發送至所有客戶端，每個客戶端附帶逾時保護。
        /// </summary>
        private async Task BroadcastLoopAsync()
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

                    var sendTasks = new List<Task>(snapshot.Length);
                    foreach (var ws in snapshot)
                    {
                        if (ws.State == WebSocketState.Open)
                            sendTasks.Add(SendWithTimeout(ws, segment, _cts.Token));
                    }

                    if (sendTasks.Count > 0)
                        await Task.WhenAll(sendTasks);
                }
                catch (Exception ex)
                {
                    if (!_cts.Token.IsCancellationRequested)
                        _log.Error("廣播錯誤", ex);
                }

                try
                {
                    await Task.Delay(BROADCAST_INTERVAL_MS, _cts.Token);
                }
                catch (OperationCanceledException) { break; }
            }
        }

        /// <summary>對單一客戶端發送資料，附帶逾時保護，失敗時自動移除。</summary>
        private async Task SendWithTimeout(WebSocket ws, ArraySegment<byte> data, CancellationToken ct)
        {
            try
            {
                using var timeoutCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
                timeoutCts.CancelAfter(SEND_TIMEOUT_MS);
                await ws.SendAsync(data, WebSocketMessageType.Text, true, timeoutCts.Token);
            }
            catch (Exception ex)
            {
                _log.Warn($"WebSocket 發送失敗，移除客戶端：{ex.Message}");
                lock (_clientLock) { _clients.Remove(ws); }
            }
        }

        /// <summary>
        /// 建構狀態 JSON。透過 IMotionController 存取低階軸資料與高階末端位姿，
        /// 每種資料只讀取一次以減少不必要的鎖操作。
        /// </summary>
        private string BuildStateJson()
        {
            // 從 IMotionController.AxisCard 讀取軸卡資料（每個屬性讀一次）
            var axisCard = _controller.AxisCard;
            var pos = axisCard.Pos;
            var speed = axisCard.Speed;
            var motorState = axisCard.State;
            var queueLen = axisCard.QueueLength;
            var cardState = axisCard.AxisCardState;

            // 從 IMotionController 讀取末端位姿
            var eePos = _controller.EndEffectorPosition;
            var eePosture = _controller.EndEffectorPosture;

            return JsonSerializer.Serialize(new
            {
                type = "state",
                cardState = (int)cardState,
                pos,
                speed,
                motorState = motorState.Select(s => (int)s).ToArray(),
                queueLen,
                endEffectorPosition = eePos,
                endEffectorPosture = new[]
                {
                    eePosture.M11, eePosture.M12, eePosture.M13, eePosture.M14,
                    eePosture.M21, eePosture.M22, eePosture.M23, eePosture.M24,
                    eePosture.M31, eePosture.M32, eePosture.M33, eePosture.M34,
                    eePosture.M41, eePosture.M42, eePosture.M43, eePosture.M44,
                },
                timestamp = DateTimeOffset.Now.ToUnixTimeMilliseconds(),
            });
        }

        /// <summary>停止廣播、關閉所有 WebSocket 客戶端並釋放 HTTP 監聽器。</summary>
        public void Dispose()
        {
            Dispose(disposing: true);
            GC.SuppressFinalize(this);
        }

        /// <summary>實際釋放資源的核心方法。</summary>
        protected virtual void Dispose(bool disposing)
        {
            if (_disposed) return;
            _disposed = true;

            if (disposing)
            {
                _cts.Cancel();

                var tasksToWait = new[] { _broadcastTask, _acceptTask }
                    .Where(t => t != null).Cast<Task>().ToArray();
                if (tasksToWait.Length > 0)
                    Task.WaitAll(tasksToWait, 2000);

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
}
