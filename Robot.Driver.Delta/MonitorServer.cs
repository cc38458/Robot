using System.Net;
using System.Net.WebSockets;
using System.Reflection;
using System.Text;
using System.Text.Json;
using Robot.Core.Interfaces;
using Robot.Core.Logging;

namespace Robot.Driver.Delta
{
    /// <summary>
    /// 唯讀 WebSocket 監控伺服器（嵌入式，零外部依賴）
    /// 功能：
    ///   1. 以 20Hz 頻率推送機械臂完整狀態（JSON）
    ///   2. 提供 monitor.html 靜態檔案服務
    /// 注意：
    ///   - 本伺服器僅監看，不接受任何控制命令
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

        public MonitorServer(IAxisCard driver, RobotLogger logger,
                             int port = 5850, string? htmlPath = null)
        {
            _driver = driver;
            _log = logger;
            _port = port;
            _htmlPath = htmlPath;
        }

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

            Task.Run(() => AcceptLoop(_cts.Token));

            _broadcastThread = new Thread(BroadcastLoop)
            {
                Name = "MonitorBroadcast",
                IsBackground = true,
            };
            _broadcastThread.Start();
        }

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
                Path.GetFullPath(filePath).StartsWith(Path.GetFullPath(baseDir)) &&
                File.Exists(filePath))
            {
                ServeBytes(resp, File.ReadAllBytes(filePath), GetContentType(filePath));
                return;
            }

            // 嘗試 2：從 DLL 嵌入式資源提供
            var resourceName = reqPath.TrimStart('/');
            var stream = Assembly.GetExecutingAssembly().GetManifestResourceStream(resourceName);
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

        private static void ServeBytes(HttpListenerResponse resp, byte[] data, string contentType)
        {
            resp.ContentType = contentType;
            resp.Headers.Add("Access-Control-Allow-Origin", "*");
            resp.ContentLength64 = data.Length;
            resp.StatusCode = 200;
            resp.OutputStream.Write(data, 0, data.Length);
            resp.OutputStream.Close();
        }

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
                pos,
                speed,
                motorState = motorState.Select(s => (int)s).ToArray(),
                queueLen,
                timestamp = DateTimeOffset.Now.ToUnixTimeMilliseconds(),
            });
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
