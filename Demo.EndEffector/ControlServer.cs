using System.Net;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using Robot.Motion.RA605;

namespace Demo.EndEffector;

/// <summary>
/// 末端控制 WebSocket 伺服器。
/// GET /  → controller.html
/// WS  /  → 雙向 JSON 控制協定
///
/// 客戶端 → 伺服器：
///   {"type":"move","dx":0,"dy":1,"dz":0,"dyaw":0,"dpitch":0,"droll":0}  // 各軸 -1~1
///   {"type":"stop"}
///   {"type":"home"}
///   {"type":"estop"}
///   {"type":"ralm"}
///   {"type":"speed","linear":30,"angular":10000}  // mm/s, mdeg/s
///
/// 伺服器 → 客戶端（10 Hz）：
///   {"type":"state","x":0,"y":0,"z":0,"joints":[...],"cardState":"READY","linearSpeed":30,"angularSpeed":10000}
/// </summary>
public sealed class ControlServer : IDisposable
{
    private readonly RA605RobotApp _robot;
    private readonly int _port;
    private readonly string _htmlPath;

    private HttpListener? _listener;
    private readonly CancellationTokenSource _cts = new();
    private readonly List<WebSocket> _clients = new();
    private readonly object _clientLock = new();

    private float _linearSpeed = 30f;      // mm/s
    private float _angularSpeed = 10000f;  // mdeg/s
    private readonly object _speedLock = new();

    private const int BROADCAST_MS = 100; // 10 Hz

    public ControlServer(RA605RobotApp robot, int port = 5851, string htmlPath = "controller.html")
    {
        _robot = robot;
        _port = port;
        _htmlPath = htmlPath;
    }

    public void Start()
    {
        _listener = new HttpListener();
        _listener.Prefixes.Add($"http://+:{_port}/");
        try { _listener.Start(); }
        catch
        {
            _listener = new HttpListener();
            _listener.Prefixes.Add($"http://localhost:{_port}/");
            _listener.Start();
        }
        Task.Run(() => AcceptLoop(_cts.Token));
        Task.Run(() => BroadcastLoop(_cts.Token));
    }

    // ── Accept ──────────────────────────────────────────────

    private async Task AcceptLoop(CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            try
            {
                var ctx = await _listener!.GetContextAsync();
                if (ctx.Request.IsWebSocketRequest)
                {
                    var ws = await ctx.AcceptWebSocketAsync(null);
                    _ = Task.Run(() => HandleClient(ws.WebSocket, ct));
                }
                else
                {
                    ServeFile(ctx);
                }
            }
            catch (ObjectDisposedException) { break; }
            catch (HttpListenerException) { break; }
        }
    }

    private void ServeFile(HttpListenerContext ctx)
    {
        var resp = ctx.Response;
        try
        {
            if (File.Exists(_htmlPath))
            {
                var data = File.ReadAllBytes(_htmlPath);
                resp.ContentType = "text/html; charset=utf-8";
                resp.ContentLength64 = data.Length;
                resp.StatusCode = 200;
                resp.OutputStream.Write(data, 0, data.Length);
            }
            else
            {
                var msg = Encoding.UTF8.GetBytes($"<h2>找不到 controller.html</h2><p>路徑：{_htmlPath}</p>");
                resp.ContentType = "text/html; charset=utf-8";
                resp.ContentLength64 = msg.Length;
                resp.StatusCode = 404;
                resp.OutputStream.Write(msg, 0, msg.Length);
            }
        }
        finally { try { resp.OutputStream.Close(); } catch { } }
    }

    // ── WebSocket ────────────────────────────────────────────

    private async Task HandleClient(WebSocket ws, CancellationToken ct)
    {
        lock (_clientLock) { _clients.Add(ws); }
        var buf = new byte[4096];
        try
        {
            while (ws.State == WebSocketState.Open && !ct.IsCancellationRequested)
            {
                var result = await ws.ReceiveAsync(new ArraySegment<byte>(buf), ct);
                if (result.MessageType == WebSocketMessageType.Close) break;
                if (result.MessageType == WebSocketMessageType.Text)
                    ProcessCommand(Encoding.UTF8.GetString(buf, 0, result.Count));
            }
        }
        catch (WebSocketException) { }
        catch (OperationCanceledException) { }
        finally
        {
            lock (_clientLock) { _clients.Remove(ws); }
            try { ws.Dispose(); } catch { }
            // 客戶端斷線時停止持續移動
            _robot.StopContinuousMove();
        }
    }

    private void ProcessCommand(string json)
    {
        try
        {
            using var doc = JsonDocument.Parse(json);
            var root = doc.RootElement;
            var type = root.GetProperty("type").GetString();

            switch (type)
            {
                case "move":
                {
                    float dx     = TryGetFloat(root, "dx");
                    float dy     = TryGetFloat(root, "dy");
                    float dz     = TryGetFloat(root, "dz");
                    float dyaw   = TryGetFloat(root, "dyaw");
                    float dpitch = TryGetFloat(root, "dpitch");
                    float droll  = TryGetFloat(root, "droll");

                    float lin, ang;
                    lock (_speedLock) { lin = _linearSpeed; ang = _angularSpeed; }

                    _robot.UpdateContinuousMove(
                        dx * lin, dy * lin, dz * lin,
                        dyaw * ang, dpitch * ang, droll * ang);
                    break;
                }
                case "stop":
                    _robot.StopContinuousMove();
                    break;

                case "home":
                    _robot.StopContinuousMove();
                    Thread.Sleep(100);
                    _robot.MoveHome();
                    break;

                case "estop":
                    _robot.StopContinuousMove();
                    _robot.Estop();
                    break;

                case "ralm":
                    _robot.Ralm();
                    break;

                case "speed":
                    lock (_speedLock)
                    {
                        if (root.TryGetProperty("linear",  out var l)) _linearSpeed  = l.GetSingle();
                        if (root.TryGetProperty("angular", out var a)) _angularSpeed = a.GetSingle();
                    }
                    break;
            }
        }
        catch { /* 忽略格式錯誤 */ }
    }

    // ── Broadcast ────────────────────────────────────────────

    private async Task BroadcastLoop(CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            try
            {
                var json = BuildStateJson();
                var seg = new ArraySegment<byte>(Encoding.UTF8.GetBytes(json));

                WebSocket[] snap;
                lock (_clientLock) { snap = _clients.ToArray(); }

                foreach (var ws in snap)
                {
                    if (ws.State != WebSocketState.Open) continue;
                    try { await ws.SendAsync(seg, WebSocketMessageType.Text, true, ct); }
                    catch { lock (_clientLock) { _clients.Remove(ws); } }
                }
            }
            catch { }

            await Task.Delay(BROADCAST_MS, ct).ConfigureAwait(false);
        }
    }

    private string BuildStateJson()
    {
        float lin, ang;
        lock (_speedLock) { lin = _linearSpeed; ang = _angularSpeed; }

        float[] pos;
        int[] joints;
        string cardState;
        try
        {
            pos = _robot.EndEffectorPosition;
            joints = _robot.Pos;
            cardState = _robot.AxisCardState.ToString();
        }
        catch
        {
            pos = new float[3];
            joints = new int[6];
            cardState = "NULL";
        }

        return JsonSerializer.Serialize(new
        {
            type = "state",
            x = MathF.Round(pos[0], 2),
            y = MathF.Round(pos[1], 2),
            z = MathF.Round(pos[2], 2),
            joints = joints.Select(j => MathF.Round(j / 1000f, 2)).ToArray(),
            cardState,
            linearSpeed  = lin,
            angularSpeed = ang,
        });
    }

    // ── Helpers ──────────────────────────────────────────────

    private static float TryGetFloat(JsonElement el, string key)
        => el.TryGetProperty(key, out var v) ? v.GetSingle() : 0f;

    public void Dispose()
    {
        _cts.Cancel();
        try { _listener?.Stop(); } catch { }
        try { _listener?.Close(); } catch { }
        _cts.Dispose();
    }
}
