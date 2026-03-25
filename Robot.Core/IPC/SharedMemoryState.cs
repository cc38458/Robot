using System.IO.MemoryMappedFiles;
using System.Runtime.InteropServices;
using Robot.Core.Enums;

namespace Robot.Core.IPC
{
    /// <summary>
    /// 跨行程共享記憶體：用於狀態推送與心跳監控。
    /// 使用 file-backed MemoryMappedFile 以相容 Windows / WSL2。
    /// 採用 SeqLock 模式確保讀端在無鎖情況下取得一致快照。
    ///
    /// 記憶體佈局（共 128 bytes）：
    ///   [0..3]     int     Sequence（SeqLock 序號，寫端每次更新 +1 → 寫入 → +1）
    ///   [4..7]     int     CardState
    ///   [8..31]    int[6]  Pos
    ///   [32..55]   int[6]  Speed
    ///   [56..79]   int[6]  MotorState
    ///   [80..103]  int[6]  QueueLength
    ///   [104..111] long    CommService 心跳時戳（UTC ticks）
    ///   [112..119] long    MainApp 心跳時戳（UTC ticks）
    ///   [120..127] 保留
    /// </summary>
    public sealed class SharedMemoryState : IDisposable
    {
        public const string FILE_NAME = "robot_comm_shm.dat";
        public const int TOTAL_SIZE = 128;
        private const int AXIS_COUNT = 6;

        // 偏移量常數
        private const int OFF_SEQ = 0;
        private const int OFF_CARD_STATE = 4;
        private const int OFF_POS = 8;
        private const int OFF_SPEED = 32;
        private const int OFF_MOTOR_STATE = 56;
        private const int OFF_QUEUE_LEN = 80;
        private const int OFF_COMM_HEARTBEAT = 104;
        private const int OFF_MAIN_HEARTBEAT = 112;

        private readonly MemoryMappedFile _mmf;
        private readonly MemoryMappedViewAccessor _view;
        private bool _disposed;

        /// <summary>
        /// 建立或開啟共享記憶體。
        /// </summary>
        /// <param name="filePath">底層檔案路徑（建議放在 temp 目錄）</param>
        public SharedMemoryState(string filePath)
        {
            // 確保檔案存在且大小正確
            if (!File.Exists(filePath))
            {
                using var fs = File.Create(filePath);
                fs.SetLength(TOTAL_SIZE);
            }
            else
            {
                var info = new FileInfo(filePath);
                if (info.Length < TOTAL_SIZE)
                {
                    using var fs = File.OpenWrite(filePath);
                    fs.SetLength(TOTAL_SIZE);
                }
            }

            var fileStream = new FileStream(filePath, FileMode.Open, FileAccess.ReadWrite, FileShare.ReadWrite);
            _mmf = MemoryMappedFile.CreateFromFile(fileStream, null, TOTAL_SIZE,
                MemoryMappedFileAccess.ReadWrite, HandleInheritability.None, false);
            _view = _mmf.CreateViewAccessor(0, TOTAL_SIZE);
        }

        // ════════════════════════════════════════
        // 寫端（CommService 使用）
        // ════════════════════════════════════════

        /// <summary>
        /// 以 SeqLock 模式寫入完整狀態快照。
        /// </summary>
        public void WriteState(CardState cardState, int[] pos, int[] speed,
                               MotorState[] motorState, int[] queueLength)
        {
            // SeqLock: 序號先 +1（變為奇數，表示正在寫入）
            int seq = _view.ReadInt32(OFF_SEQ);
            _view.Write(OFF_SEQ, seq + 1);
            Thread.MemoryBarrier();

            _view.Write(OFF_CARD_STATE, (int)cardState);

            for (int i = 0; i < AXIS_COUNT; i++)
            {
                _view.Write(OFF_POS + i * 4, pos[i]);
                _view.Write(OFF_SPEED + i * 4, speed[i]);
                _view.Write(OFF_MOTOR_STATE + i * 4, (int)motorState[i]);
                _view.Write(OFF_QUEUE_LEN + i * 4, queueLength[i]);
            }

            // SeqLock: 序號再 +1（變為偶數，表示寫入完成）
            Thread.MemoryBarrier();
            _view.Write(OFF_SEQ, seq + 2);
        }

        /// <summary>寫入 CommService 心跳時戳。</summary>
        public void WriteCommHeartbeat()
        {
            _view.Write(OFF_COMM_HEARTBEAT, DateTime.UtcNow.Ticks);
        }

        /// <summary>寫入主程式心跳時戳。</summary>
        public void WriteMainHeartbeat()
        {
            _view.Write(OFF_MAIN_HEARTBEAT, DateTime.UtcNow.Ticks);
        }

        // ════════════════════════════════════════
        // 讀端（PipeAxisCard / 主程式使用）
        // ════════════════════════════════════════

        /// <summary>
        /// 以 SeqLock 模式讀取狀態快照。若偵測到寫入中衝突，最多重試 5 次。
        /// </summary>
        public bool TryReadState(out CardState cardState, int[] pos, int[] speed,
                                  MotorState[] motorState, int[] queueLength)
        {
            for (int attempt = 0; attempt < 5; attempt++)
            {
                int seqBefore = _view.ReadInt32(OFF_SEQ);
                Thread.MemoryBarrier();

                // 奇數表示寫端正在寫入
                if ((seqBefore & 1) != 0)
                {
                    Thread.SpinWait(50);
                    continue;
                }

                cardState = (CardState)_view.ReadInt32(OFF_CARD_STATE);
                for (int i = 0; i < AXIS_COUNT; i++)
                {
                    pos[i] = _view.ReadInt32(OFF_POS + i * 4);
                    speed[i] = _view.ReadInt32(OFF_SPEED + i * 4);
                    motorState[i] = (MotorState)_view.ReadInt32(OFF_MOTOR_STATE + i * 4);
                    queueLength[i] = _view.ReadInt32(OFF_QUEUE_LEN + i * 4);
                }

                Thread.MemoryBarrier();
                int seqAfter = _view.ReadInt32(OFF_SEQ);

                if (seqBefore == seqAfter)
                    return true;
            }

            // 讀取失敗，回傳預設值
            cardState = CardState.NULL;
            return false;
        }

        /// <summary>讀取 CommService 心跳時戳。</summary>
        public long ReadCommHeartbeatTicks() => _view.ReadInt64(OFF_COMM_HEARTBEAT);

        /// <summary>讀取主程式心跳時戳。</summary>
        public long ReadMainHeartbeatTicks() => _view.ReadInt64(OFF_MAIN_HEARTBEAT);

        /// <summary>檢查 CommService 心跳是否逾時。</summary>
        public bool IsCommHeartbeatTimeout(double timeoutSeconds)
        {
            long ticks = ReadCommHeartbeatTicks();
            if (ticks == 0) return false; // 尚未寫入過
            var elapsed = DateTime.UtcNow - new DateTime(ticks, DateTimeKind.Utc);
            return elapsed.TotalSeconds > timeoutSeconds;
        }

        /// <summary>檢查主程式心跳是否逾時。</summary>
        public bool IsMainHeartbeatTimeout(double timeoutSeconds)
        {
            long ticks = ReadMainHeartbeatTicks();
            if (ticks == 0) return false; // 尚未寫入過
            var elapsed = DateTime.UtcNow - new DateTime(ticks, DateTimeKind.Utc);
            return elapsed.TotalSeconds > timeoutSeconds;
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            _view.Dispose();
            _mmf.Dispose();
        }
    }
}
