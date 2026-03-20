// ═══════════════════════════════════════════════════════════════════
// EtherCAT_DLL_Mock.cs
// 模擬版 DLL — 取代 EtherCAT_DLL_x64.cs 進行無硬體測試
// 編譯符號 MOCK 時引用此檔，命名空間與類別名稱完全相同
// ═══════════════════════════════════════════════════════════════════

using System.Runtime.CompilerServices;
using System.Text;

namespace EtherCAT_DLL_Mock
{
    /// <summary>
    /// 模擬版 EtherCAT DLL — 在無硬體環境下以記憶體模擬所有軸卡操作，
    /// 支援 PVT 動畫插值以配合 Web 監控即時顯示。
    /// </summary>
    public class CEtherCAT_Mock
    {
        // ── 模擬狀態 ──
        private static readonly object _lock = new();
        private static bool _verbose = true;
        private static bool _showPolling = false;
        private static readonly int[] _position = new int[6];
        private static readonly int[] _speed = new int[6];
        private static readonly ushort[] _mdone = { 0, 0, 0, 0, 0, 0 };
        private static readonly ushort[] _statusWord = new ushort[6];
        private static readonly ushort[] _bufferLength = new ushort[6];
        private static bool _initialized = false;

        // ── PVTComplete Config 暫存（供 Sync_Move 使用）──
        private static readonly int[][] _pvtPos     = new int[6][];
        private static readonly int[][] _pvtTime    = new int[6][];
        private static readonly int[]   _pvtCnt     = new int[6];
        private static readonly int[]   _pvtEndVel  = new int[6];

        // ── 動畫取消令牌（每軸獨立，停止時取消正在執行的動畫） ──
        private static readonly CancellationTokenSource[] _animCts = new CancellationTokenSource[6];

        // ── 模擬控制方法（測試用） ──

        /// <summary>設定是否在 Console 印出 Mock 日誌。</summary>
        public static void Mock_SetVerbose(bool verbose) => _verbose = verbose;
        /// <summary>設定是否印出輪詢類 Mock 日誌（高頻）。</summary>
        public static void Mock_SetShowPolling(bool show) => _showPolling = show;
        /// <summary>模擬觸發指定軸的警報（設定 StatusWord bit3）。</summary>
        public static void Mock_TriggerAlarm(ushort axis)
        {
            if (axis < 6) _statusWord[axis] = (ushort)(_statusWord[axis] | 0x0008);
            MockLog($"⚠ 模擬觸發軸 {axis} 警報");
        }
        /// <summary>直接設定指定軸的模擬位置。</summary>
        public static void Mock_SetPosition(ushort axis, int pos) { if (axis < 6) _position[axis] = pos; }
        /// <summary>模擬指定軸運動完成（Mdone=0, Speed=0）。</summary>
        public static void Mock_SetMotionDone(ushort axis) { if (axis < 6) { _mdone[axis] = 0; _speed[axis] = 0; } }

        /// <summary>
        /// 依 PVTComplete 點位表逐步更新虛擬位置（供 web 監控顯示動態效果）。
        /// time[] 為累計時間戳（ms），pos[] 為各時間點的目標脈波位置。
        /// </summary>
        private const int INTERP_INTERVAL_MS = 100;

        /// <summary>取消指定軸的動畫並建立新的取消令牌</summary>
        private static CancellationToken CancelAndRenewAnimation(int axis)
        {
            var oldCts = _animCts[axis];
            oldCts?.Cancel();
            oldCts?.Dispose();
            var newCts = new CancellationTokenSource();
            _animCts[axis] = newCts;
            return newCts.Token;
        }

        /// <summary>停止指定軸的動畫，將位置凍結在當前值</summary>
        private static void StopAnimation(int axis)
        {
            var oldCts = _animCts[axis];
            oldCts?.Cancel();
            oldCts?.Dispose();
            _animCts[axis] = null!;
        }

        /// <summary>
        /// 依 PVT 點位表啟動非同步動畫，每 100ms 插值更新虛擬位置。
        /// </summary>
        private static void AnimatePvt(int axis, int[] pos, int[] time, int endVel)
        {
            // 動畫啟動前擷取目前位置作為插值起點，並取消該軸舊動畫
            int startPos;
            CancellationToken ct;
            lock (_lock)
            {
                startPos = _position[axis];
                ct = CancelAndRenewAnimation(axis);
            }

            Task.Run(async () =>
            {
                var startTime = DateTime.UtcNow;
                int totalMs = time[time.Length - 1];

                // 每 100ms 插幀一次，直到動畫結束或被取消
                for (int t = INTERP_INTERVAL_MS; t <= totalMs; t += INTERP_INTERVAL_MS)
                {
                    if (ct.IsCancellationRequested) return;

                    int waitMs = t - (int)(DateTime.UtcNow - startTime).TotalMilliseconds;
                    if (waitMs > 0)
                    {
                        try { await Task.Delay(waitMs, ct); }
                        catch (TaskCanceledException) { return; }
                    }

                    if (ct.IsCancellationRequested) return;

                    int elapsed = (int)(DateTime.UtcNow - startTime).TotalMilliseconds;
                    elapsed = Math.Clamp(elapsed, 0, totalMs);

                    // 插值點位表：[startPos@0] → [pos[0]@time[0]] → [pos[1]@time[1]] → ...
                    int interpPos;
                    if (elapsed <= time[0])
                    {
                        interpPos = time[0] == 0 ? pos[0]
                            : startPos + (int)((long)(pos[0] - startPos) * elapsed / time[0]);
                    }
                    else if (elapsed >= totalMs)
                    {
                        interpPos = pos[pos.Length - 1];
                    }
                    else
                    {
                        int lo = 0, hi = time.Length - 1;
                        while (hi - lo > 1)
                        {
                            int mid = (lo + hi) / 2;
                            if (time[mid] <= elapsed) lo = mid; else hi = mid;
                        }
                        int t0 = time[lo], t1 = time[hi];
                        int p0 = pos[lo],  p1 = pos[hi];
                        interpPos = t1 == t0 ? p1
                            : p0 + (int)((long)(p1 - p0) * (elapsed - t0) / (t1 - t0));
                    }

                    lock (_lock) { _position[axis] = interpPos; }
                }

                if (ct.IsCancellationRequested) return;

                // 確保最終位置精確
                lock (_lock)
                {
                    _position[axis] = pos[pos.Length - 1];
                    if (endVel == 0) { _mdone[axis] = 0; _speed[axis] = 0; }
                    if (_bufferLength[axis] > 0) _bufferLength[axis]--;
                }
            });
        }

        /// <summary>印出帶時間戳的 Mock 日誌（受 _verbose 控制）。</summary>
        private static void MockLog(string msg)
        {
            if (!_verbose) return;
            var ts = DateTime.Now.ToString("HH:mm:ss.fff");
            Console.ForegroundColor = ConsoleColor.Cyan;
            Console.Write($"[{ts}] ");
            Console.ForegroundColor = ConsoleColor.Yellow;
            Console.Write("[MOCK] ");
            Console.ResetColor();
            Console.WriteLine(msg);
        }
        /// <summary>印出輪詢類 Mock 日誌（受 _showPolling 控制）。</summary>
        private static void PollLog(string msg) { if (_showPolling) MockLog(msg); }

        // ═══════ 主站管理 ═══════

        /// <summary>模擬開啟主站，回傳 1 張軸卡。</summary>
        public static ushort CS_ECAT_Master_Open(ref ushort existcard)
        { existcard = 1; MockLog("Master_Open → 1 張軸卡"); return 0; }

        /// <summary>模擬取得軸卡序號（固定回傳 0）。</summary>
        public static ushort CS_ECAT_Master_Get_CardSeq(ushort CardNo_seq, ref ushort CardNo)
        { CardNo = 0; MockLog($"Master_Get_CardSeq → CardNo=0"); return 0; }

        /// <summary>模擬主站初始化。</summary>
        public static ushort CS_ECAT_Master_Initial(ushort CardNo)
        { MockLog($"Master_Initial(CardNo={CardNo})"); return 0; }

        /// <summary>模擬檢查初始化完成（第二次呼叫回傳完成）。</summary>
        public static ushort CS_ECAT_Master_Check_Initial_Done(ushort CardNo, ref ushort InitDone)
        {
            if (!_initialized) { _initialized = true; InitDone = 1; }
            else { InitDone = 0; }
            return 0;
        }

        /// <summary>模擬關閉主站。</summary>
        public static ushort CS_ECAT_Master_Close()
        { _initialized = false; MockLog("Master_Close"); return 0; }

        /// <summary>模擬主站重置。</summary>
        public static ushort CS_ECAT_Master_Reset(ushort CardNo) { return 0; }

        /// <summary>模擬取得從站數量（固定回傳 6）。</summary>
        public static ushort CS_ECAT_Master_Get_SlaveNum(ushort CardNo, ref ushort SlaveNum)
        { SlaveNum = 6; MockLog("Master_Get_SlaveNum → 6"); return 0; }

        /// <summary>模擬取得連線狀態（固定已連線）。</summary>
        public static ushort CS_ECAT_Master_Get_Connect_Status(ushort CardNo, ref ushort Status)
        { Status = 1; return 0; }

        /// <summary>模擬取得回傳碼訊息（空操作）。</summary>
        public static ushort CS_ECAT_Master_Get_Return_Code_Message(ushort ReturnCode, string Message) => 0;
        /// <summary>模擬取得硬體 ID。</summary>
        public static ushort CS_ECAT_Master_Get_Hardware_ID(ushort CardNo, ref ushort DeviceID) { DeviceID = 0x434; return 0; }
        /// <summary>模擬取得初始化錯誤碼（固定 0）。</summary>
        public static ushort CS_ECAT_Master_Get_Initial_ErrorCode(ushort CardNo) => 0;

        // ═══════ 伺服控制 ═══════

        /// <summary>模擬 Servo ON/OFF。</summary>
        public static ushort CS_ECAT_Slave_Motion_Set_Svon(ushort CardNo, ushort NodeID, ushort SlotNo, ushort On_Off)
        { MockLog($"Set_Svon(軸{NodeID}, {(On_Off == 1 ? "ON" : "OFF")})"); return 0; }

        /// <summary>模擬警報復歸（清除 StatusWord bit3）。</summary>
        public static ushort CS_ECAT_Slave_Motion_Ralm(ushort CardNo, ushort NodeID, ushort SlotNo)
        {
            if (NodeID < 6) _statusWord[NodeID] = (ushort)(_statusWord[NodeID] & ~0x0008);
            MockLog($"Ralm(軸{NodeID})");
            return 0;
        }

        /// <summary>模擬設定運動模式（CSP/CSV/CST/HOME）。</summary>
        public static ushort CS_ECAT_Slave_Motion_Set_MoveMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort OpMode)
        {
            string m = OpMode switch { 8 => "CSP", 9 => "CSV", 10 => "CST", 6 => "HOME", _ => $"{OpMode}" };
            MockLog($"Set_MoveMode(軸{NodeID}, {m})");
            return 0;
        }

        // ═══════ 停止命令 ═══════

        /// <summary>模擬緊急停止（立即停止動畫並重設狀態）。</summary>
        public static ushort CS_ECAT_Slave_Motion_Emg_Stop(ushort CardNo, ushort NodeID, ushort SlotNo)
        {
            if (NodeID < 6)
            {
                StopAnimation(NodeID);
                _mdone[NodeID] = 0; _speed[NodeID] = 0;
                if (_bufferLength[NodeID] > 0) _bufferLength[NodeID] = 0;
            }
            MockLog($"Emg_Stop(軸{NodeID})");
            return 0;
        }

        /// <summary>模擬減速停止（停止動畫並重設狀態）。</summary>
        public static ushort CS_ECAT_Slave_Motion_Sd_Stop(ushort CardNo, ushort NodeID, ushort SlotNo, double Tdec)
        {
            if (NodeID < 6)
            {
                StopAnimation(NodeID);
                _mdone[NodeID] = 0; _speed[NodeID] = 0;
                if (_bufferLength[NodeID] > 0) _bufferLength[NodeID] = 0;
            }
            MockLog($"Sd_Stop(軸{NodeID}, {Tdec}s)");
            return 0;
        }

        /// <summary>模擬變更速度（NewSpeed=0 時停止動畫）。</summary>
        public static ushort CS_ECAT_Slave_CSP_Velocity_Change(ushort CardNo, ushort NodeID, ushort SlotNo, int NewSpeed, double Tsec)
        {
            if (NodeID < 6)
            {
                if (NewSpeed == 0)
                {
                    // 減速至 0：取消動畫，凍結在當前位置
                    StopAnimation(NodeID);
                    _speed[NodeID] = 0;
                }
                else
                {
                    _speed[NodeID] = NewSpeed;
                }
            }
            MockLog($"CSP_Velocity_Change(軸{NodeID}, NewSpd={NewSpeed}, Tsec={Tsec})");
            return 0;
        }

        // ═══════ 狀態讀取 ═══════

        /// <summary>模擬讀取虛擬座標位置。</summary>
        public static ushort CS_ECAT_Slave_Motion_Get_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Position)
        { if (NodeID < 6) Position = _position[NodeID]; return 0; }

        /// <summary>模擬讀取實際編碼器位置（Mock 中與虛擬座標相同）。</summary>
        public static ushort CS_ECAT_Slave_Motion_Get_Actual_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int ActualPosition)
        { if (NodeID < 6) ActualPosition = _position[NodeID]; return 0; }

        /// <summary>模擬讀取目前速度。</summary>
        public static ushort CS_ECAT_Slave_Motion_Get_Current_Speed(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Speed)
        { if (NodeID < 6) Speed = _speed[NodeID]; return 0; }

        /// <summary>模擬讀取速度（同 Get_Current_Speed）。</summary>
        public static ushort CS_ECAT_Slave_Motion_Get_Speed(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Speed)
        { if (NodeID < 6) Speed = _speed[NodeID]; return 0; }

        /// <summary>模擬讀取運動完成旗標。</summary>
        public static ushort CS_ECAT_Slave_Motion_Get_Mdone(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Mdone)
        { if (NodeID < 6) Mdone = _mdone[NodeID]; return 0; }

        /// <summary>模擬讀取狀態字。</summary>
        public static ushort CS_ECAT_Slave_Motion_Get_StatusWord(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort StatusWord)
        { if (NodeID < 6) StatusWord = _statusWord[NodeID]; return 0; }

        /// <summary>模擬讀取硬體 buffer 長度。</summary>
        public static ushort CS_ECAT_Slave_Motion_Get_Buffer_Length(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort BufferLength)
        { if (NodeID < 6) BufferLength = _bufferLength[NodeID]; return 0; }

        /// <summary>模擬讀取命令位置。</summary>
        public static ushort CS_ECAT_Slave_Motion_Get_Command(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Command)
        { if (NodeID < 6) Command = _position[NodeID]; return 0; }

        /// <summary>模擬讀取實際命令位置。</summary>
        public static ushort CS_ECAT_Slave_Motion_Get_Actual_Command(ushort CardNo, ushort NodeID, ushort SlotNo, ref int ActualCommand)
        { if (NodeID < 6) ActualCommand = _position[NodeID]; return 0; }

        /// <summary>模擬讀取目標命令位置。</summary>
        public static ushort CS_ECAT_Slave_Motion_Get_Target_Command(ushort CardNo, ushort NodeID, ushort SlotNo, ref int TargetCommand)
        { if (NodeID < 6) TargetCommand = _position[NodeID]; return 0; }

        /// <summary>模擬讀取扭矩（固定 0）。</summary>
        public static ushort CS_ECAT_Slave_Motion_Get_Torque(ushort CardNo, ushort NodeID, ushort SlotNo, ref short Torque) { Torque = 0; return 0; }
        /// <summary>模擬讀取控制字（固定 0）。</summary>
        public static ushort CS_ECAT_Slave_Motion_Get_ControlWord(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort ControllWord) { ControllWord = 0; return 0; }
        /// <summary>模擬讀取運動模式（固定 CSP=8）。</summary>
        public static ushort CS_ECAT_Slave_Motion_Get_MoveMode(ushort CardNo, ushort NodeID, ushort SlotNo, ref byte Mode) { Mode = 8; return 0; }

        // ═══════ CSP 運動命令 ═══════

        /// <summary>模擬 CSP 位移運動（含動畫插值）。</summary>
        public static ushort CS_ECAT_Slave_CSP_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo,
            int Dist, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs)
        {
            if (NodeID >= 6) return 0;
            _mdone[NodeID] = 2; _speed[NodeID] = ConstVel; _bufferLength[NodeID]++;
            string mode = IsAbs == 1 ? "絕對" : "相對";
            MockLog($"CSP_Move(軸{NodeID}, {mode}, D={Dist}, V={ConstVel}, EndV={EndVel})");

            int startPos, targetPos;
            lock (_lock)
            {
                startPos = _position[NodeID];
                targetPos = IsAbs == 1 ? Dist : startPos + Dist;
            }
            long delta = Math.Abs((long)targetPos - startPos);
            int totalMs = ConstVel > 0 && delta > 0
                ? (int)(delta * 1000L / ConstVel) + (int)((Tacc + Tdec) * 500)
                : 200;
            totalMs = Math.Max(totalMs, 200);
            AnimatePvt(NodeID, new[] { targetPos }, new[] { totalMs }, EndVel);
            return 0;
        }

        /// <summary>模擬 CSP 等速運動。</summary>
        public static ushort CS_ECAT_Slave_CSP_Start_V_Move(ushort CardNo, ushort NodeID, ushort SlotNo,
            ushort Dir, int StrVel, int ConstVel, double Tacc, ushort SCurve)
        {
            if (NodeID < 6) { _mdone[NodeID] = 2; _speed[NodeID] = Dir == 0 ? ConstVel : -ConstVel; }
            MockLog($"CSP_V_Move(軸{NodeID}, {(Dir == 0 ? "正向" : "負向")}, V={ConstVel})");
            return 0;
        }

        /// <summary>模擬 PVT 完整運動（含動畫插值）。</summary>
        public static ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Move(ushort CardNo, ushort NodeID, ushort SlotID,
            int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs)
        {
            if (NodeID >= 6) return 0;
            _mdone[NodeID] = 2;
            _bufferLength[NodeID]++;

            var pos  = new int[DataCnt];
            var time = new int[DataCnt];
            var sb = new StringBuilder();
            sb.AppendLine($"CSP_PVTComplete_Move(軸{NodeID}, {DataCnt}筆, StrV={StrVel}, EndV={EndVel}, Abs={Abs})");
            for (int k = 0; k < DataCnt; k++)
            {
                pos[k]  = Unsafe.Add(ref TargetPos,  k);
                time[k] = Unsafe.Add(ref TargetTime, k);
                sb.AppendLine($"  [{k:D2}] Pos={pos[k],10} pulse  Time={time[k],8} ms");
            }
            MockLog(sb.ToString().TrimEnd());

            AnimatePvt(NodeID, pos, time, EndVel);
            return 0;
        }

        /// <summary>模擬 PVT 設定（暫存點位，待 Sync_Move 統一啟動）。</summary>
        public static ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Config(ushort CardNo, ushort NodeID, ushort SlotID,
            int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs)
        {
            if (NodeID >= 6) return 0;

            var pos  = new int[DataCnt];
            var time = new int[DataCnt];
            var sb = new StringBuilder();
            sb.AppendLine($"CSP_PVTComplete_Config(軸{NodeID}, {DataCnt}筆, StrV={StrVel}, EndV={EndVel}, Abs={Abs})");
            for (int k = 0; k < DataCnt; k++)
            {
                pos[k]  = Unsafe.Add(ref TargetPos,  k);
                time[k] = Unsafe.Add(ref TargetTime, k);
                sb.AppendLine($"  [{k:D2}] Pos={pos[k],10} pulse  Time={time[k],8} ms");
            }
            MockLog(sb.ToString().TrimEnd());

            // 暫存點位，等待 Sync_Move 統一啟動動畫
            _pvtPos[NodeID]    = pos;
            _pvtTime[NodeID]   = time;
            _pvtCnt[NodeID]    = DataCnt;
            _pvtEndVel[NodeID] = EndVel;
            return 0;
        }

        /// <summary>模擬 PVT 運動（簡化版，不含插值）。</summary>
        public static ushort CS_ECAT_Slave_CSP_Start_PVT_Move(ushort CardNo, ushort NodeID, ushort SlotID,
            int DataCnt, ref int TargetPos, ref int TargetTime, ref int TargetVel, ushort Abs)
        { if (NodeID < 6) _mdone[NodeID] = 2; MockLog($"CSP_PVT_Move(軸{NodeID}, {DataCnt}筆)"); return 0; }

        /// <summary>模擬 PVT 設定（簡化版）。</summary>
        public static ushort CS_ECAT_Slave_CSP_Start_PVT_Config(ushort CardNo, ushort NodeID, ushort SlotID,
            int DataCnt, ref int TargetPos, ref int TargetTime, ref int TargetVel, ushort Abs)
        { MockLog($"CSP_PVT_Config(軸{NodeID}, {DataCnt}筆)"); return 0; }

        /// <summary>模擬多軸同步啟動 PVT 運動（啟動各軸 Config 暫存的動畫）。</summary>
        public static ushort CS_ECAT_Slave_CSP_Start_PVT_Sync_Move(ushort CardNo, ushort AxisNum,
            ref ushort AxisArray, ref ushort SlotArray)
        {
            MockLog($"CSP_PVT_Sync_Move({AxisNum}軸同步)");
            int n = Math.Min((int)AxisNum, 6);
            for (int i = 0; i < n; i++)
            {
                int axis = Unsafe.Add(ref AxisArray, i);
                if (axis >= 6 || _pvtPos[axis] == null) continue;
                _mdone[axis] = 2;
                _bufferLength[axis]++;
                AnimatePvt(axis, _pvtPos[axis], _pvtTime[axis], _pvtEndVel[axis]);
                _pvtPos[axis] = null!;  // 消耗後清除
            }
            return 0;
        }

        // ═══════ CSP 設定 ═══════

        /// <summary>模擬設定齒輪比。</summary>
        public static ushort CS_ECAT_Slave_CSP_Set_Gear(ushort CardNo, ushort NodeID, ushort SlotNo,
            double Numerator, double Denominator, short Enable)
        { MockLog($"CSP_Set_Gear(軸{NodeID}, {Numerator}/{Denominator}, E={Enable})"); return 0; }

        /// <summary>模擬啟用/停用虛擬座標。</summary>
        public static ushort CS_ECAT_Slave_CSP_Virtual_Set_Enable(ushort CardNo, ushort NodeID, ushort SlotID, ushort Enable)
        { MockLog($"Virtual_Set_Enable(軸{NodeID}, {(Enable == 1 ? "ON" : "OFF")})"); return 0; }

        /// <summary>模擬設定虛擬座標命令值。</summary>
        public static ushort CS_ECAT_Slave_CSP_Virtual_Set_Command(ushort CardNo, ushort NodeID, ushort SlotID, int Command)
        { if (NodeID < 6) _position[NodeID] = Command; MockLog($"Virtual_Set_Command(軸{NodeID}, {Command})"); return 0; }

        /// <summary>模擬設定軟體極限（空操作）。</summary>
        public static ushort CS_ECAT_Slave_CSP_Set_Softlimit(ushort CardNo, ushort NodeID, ushort SlotNo, int PosiLimit, int NegaLimit, ushort Mode) => 0;
        /// <summary>模擬啟用速度銜接模式（空操作）。</summary>
        public static ushort CS_ECAT_Slave_CSP_Speed_Continue_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable) => 0;
        /// <summary>模擬設定速度銜接模式（空操作）。</summary>
        public static ushort CS_ECAT_Slave_CSP_Speed_Continue_Set_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode) => 0;
        /// <summary>模擬取得 PVT 目前運動計數（固定 0）。</summary>
        public static ushort CS_ECAT_Slave_CSP_PVT_Get_NowMotCount(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort NowMotCount) { NowMotCount = 0; return 0; }

        // ═══════ 其他 ═══════

        /// <summary>模擬直接設定軸位置。</summary>
        public static ushort CS_ECAT_Slave_Motion_Set_Position(ushort CardNo, ushort NodeID, ushort SlotNo, int Pos)
        { if (NodeID < 6) _position[NodeID] = Pos; return 0; }

        /// <summary>模擬設定命令（空操作）。</summary>
        public static ushort CS_ECAT_Slave_Motion_Set_Command(ushort CardNo, ushort NodeID, ushort SlotNo, int Cmd) => 0;
        /// <summary>模擬設定警報反應（空操作）。</summary>
        public static ushort CS_ECAT_Slave_Motion_Set_Alm_Reaction(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Fault_Type, ushort WR_Type) => 0;
        /// <summary>模擬設定命令等待到達（空操作）。</summary>
        public static ushort CS_ECAT_Slave_Motion_Set_Command_Wait_Target_Reach(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Wait) => 0;
        /// <summary>模擬忽略 Sd_Stop 錯誤值（空操作）。</summary>
        public static ushort CS_ECAT_Slave_Motion_Ignore_Sd_Stop_Wrong_Value(ushort CardNo, byte Ignore) => 0;
        /// <summary>模擬取得全部從站數量（固定 6）。</summary>
        public static ushort CS_ECAT_Master_Get_Total_SlaveNum(ushort CardNo, ref ushort SlaveNum) { SlaveNum = 6; return 0; }
        /// <summary>模擬取得本地從站數量（固定 0）。</summary>
        public static ushort CS_ECAT_Master_Get_Local_SlaveNum(ushort CardNo, ref ushort SlaveNum) { SlaveNum = 0; return 0; }
        /// <summary>模擬取得 Working Counter 錯誤計數（固定 0）。</summary>
        public static ushort CS_ECAT_Master_Get_Working_Counter_ErrorCounter(ushort CardNo, ref ushort Error_Cnt) { Error_Cnt = 0; return 0; }
        /// <summary>模擬檢查 Working Counter（固定正常）。</summary>
        public static ushort CS_ECAT_Master_Check_Working_Counter(ushort CardNo, ref ushort Abnormal_Flag, ref ushort Working_Slave_Cnt) { Abnormal_Flag = 0; Working_Slave_Cnt = 6; return 0; }

        /// <summary>Motion Buffer IO 資訊結構（模擬用）。</summary>
        public struct MotionBuffer_IO_Information { public ushort NodeID; public ushort SlotID; public ushort BitNo; }
        /// <summary>Motion Buffer IO 設定結構（模擬用）。</summary>
        public struct MotionBuffer_IO_Set { public MotionBuffer_IO_Information IO_Info; public ushort Value; }
    }
}
