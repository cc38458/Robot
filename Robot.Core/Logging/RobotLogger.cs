using System.Collections.Concurrent;
using System.Text;

namespace Robot.Core.Logging
{
    /// <summary>
    /// 日誌等級
    /// </summary>
    public enum LogLevel
    {
        DEBUG,
        INFO,
        WARN,
        ERROR,
        FATAL,
    }

    /// <summary>
    /// Robot 專案日誌系統
    /// 支援寫入帶時間戳的 .log 檔案（UTF-8 繁體中文），按日期分檔。
    /// 使用背景線程非同步寫入，避免阻塞呼叫端。
    /// </summary>
    public class RobotLogger : IDisposable
    {
        private readonly string _logDirectory;
        private readonly string _prefix;
        private readonly LogLevel _minLevel;
        private readonly ConcurrentQueue<string> _queue = new();
        private readonly Thread _writerThread;
        private readonly ManualResetEventSlim _hasEntries = new(false);
        private volatile bool _disposed;
        private StreamWriter? _currentWriter;
        private string _currentDate = "";

        /// <summary>
        /// 建立日誌系統
        /// </summary>
        /// <param name="logDirectory">日誌檔案目錄</param>
        /// <param name="prefix">檔名前綴（例如 "Robot"）</param>
        /// <param name="minLevel">最低記錄等級</param>
        public RobotLogger(string logDirectory = "logs", string prefix = "Robot", LogLevel minLevel = LogLevel.DEBUG)
        {
            _logDirectory = logDirectory;
            _prefix = prefix;
            _minLevel = minLevel;

            Directory.CreateDirectory(_logDirectory);

            _writerThread = new Thread(WriterLoop)
            {
                Name = "RobotLogger-Writer",
                IsBackground = true,
            };
            _writerThread.Start();
        }

        /// <summary>記錄 DEBUG 訊息</summary>
        public void Debug(string message) => Log(LogLevel.DEBUG, message);

        /// <summary>記錄 INFO 訊息</summary>
        public void Info(string message) => Log(LogLevel.INFO, message);

        /// <summary>記錄 WARN 訊息</summary>
        public void Warn(string message) => Log(LogLevel.WARN, message);

        /// <summary>記錄 ERROR 訊息</summary>
        public void Error(string message) => Log(LogLevel.ERROR, message);

        /// <summary>記錄 ERROR 訊息（附帶例外）</summary>
        public void Error(string message, Exception ex) =>
            Log(LogLevel.ERROR, $"{message} | 例外：{ex.GetType().Name}: {ex.Message}");

        /// <summary>記錄 FATAL 訊息</summary>
        public void Fatal(string message) => Log(LogLevel.FATAL, message);

        /// <summary>記錄 FATAL 訊息（附帶例外）</summary>
        public void Fatal(string message, Exception ex) =>
            Log(LogLevel.FATAL, $"{message} | 例外：{ex.GetType().Name}: {ex.Message}\n{ex.StackTrace}");

        /// <summary>
        /// 記錄 DLL 回傳碼
        /// </summary>
        /// <param name="apiName">API 名稱</param>
        /// <param name="retCode">回傳碼（0 = 成功）</param>
        /// <param name="extra">附加資訊</param>
        public void DllReturn(string apiName, ushort retCode, string extra = "")
        {
            if (retCode == 0)
                Debug($"[DLL] {apiName} → 成功{(extra.Length > 0 ? $" ({extra})" : "")}");
            else
                Warn($"[DLL] {apiName} → 錯誤碼 {retCode}{(extra.Length > 0 ? $" ({extra})" : "")}");
        }

        /// <summary>
        /// 將訊息加入寫入佇列（若等級低於最低設定則忽略）。
        /// </summary>
        private void Log(LogLevel level, string message)
        {
            if (_disposed || level < _minLevel) return;

            var timestamp = DateTime.Now.ToString("HH:mm:ss.fff");
            var threadId = Environment.CurrentManagedThreadId;
            var entry = $"[{timestamp}] [{level,-5}] [T{threadId:D3}] {message}";

            _queue.Enqueue(entry);
            _hasEntries.Set();
        }

        /// <summary>
        /// 背景寫入線程主迴圈：持續清空佇列並寫入檔案，直到 Dispose 被呼叫。
        /// </summary>
        private void WriterLoop()
        {
            while (!_disposed)
            {
                // 先清空佇列，再判斷是否需要等待
                FlushQueue();
                if (_queue.IsEmpty)
                {
                    _hasEntries.Wait(TimeSpan.FromMilliseconds(500));
                }
                _hasEntries.Reset();
            }
            // 最後清空
            FlushQueue();
            _currentWriter?.Dispose();
        }

        /// <summary>
        /// 將佇列中所有待寫入的日誌條目寫入檔案並刷新緩衝區。
        /// </summary>
        private void FlushQueue()
        {
            while (_queue.TryDequeue(out var entry))
            {
                EnsureWriter();
                _currentWriter?.WriteLine(entry);
            }
            _currentWriter?.Flush();
        }

        /// <summary>
        /// 確保 StreamWriter 已就緒且指向今日的日誌檔案（跨日自動切換）。
        /// </summary>
        private void EnsureWriter()
        {
            var today = DateTime.Now.ToString("yyyy-MM-dd");
            if (today == _currentDate && _currentWriter != null) return;

            _currentWriter?.Dispose();
            _currentDate = today;
            var filePath = Path.Combine(_logDirectory, $"{_prefix}_{_currentDate}.log");
            _currentWriter = new StreamWriter(filePath, append: true, encoding: Encoding.UTF8)
            {
                AutoFlush = false,
            };

            // 寫入日誌檔頭
            _currentWriter.WriteLine($"══════════════════════════════════════════");
            _currentWriter.WriteLine($"  {_prefix} 日誌 — {DateTime.Now:yyyy/MM/dd HH:mm:ss}");
            _currentWriter.WriteLine($"══════════════════════════════════════════");
        }

        /// <summary>
        /// 釋放資源，喚醒寫入線程以完成最後的佇列清空。
        /// </summary>
        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            _hasEntries.Set(); // 喚醒寫入線程
            _writerThread.Join(TimeSpan.FromSeconds(3));
            _hasEntries.Dispose();
            GC.SuppressFinalize(this);
        }
    }
}
