// ═══════════════════════════════════════════════════════════════════
// EtherCAT_DLL_Mock.cs
// 模擬版 DLL — 取代 EtherCAT_DLL_x64.cs 進行無硬體測試
// 編譯符號 MOCK 時引用此檔，命名空間與類別名稱完全相同
// ═══════════════════════════════════════════════════════════════════

namespace EtherCAT_DLL_Mock
{
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

        // ── 模擬控制方法（測試用） ──
        public static void Mock_SetVerbose(bool verbose) => _verbose = verbose;
        public static void Mock_SetShowPolling(bool show) => _showPolling = show;
        public static void Mock_TriggerAlarm(ushort axis)
        {
            if (axis < 6) _statusWord[axis] = (ushort)(_statusWord[axis] | 0x0008);
            MockLog($"⚠ 模擬觸發軸 {axis} 警報");
        }
        public static void Mock_SetPosition(ushort axis, int pos) { if (axis < 6) _position[axis] = pos; }
        public static void Mock_SetMotionDone(ushort axis) { if (axis < 6) { _mdone[axis] = 0; _speed[axis] = 0; } }

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
        private static void PollLog(string msg) { if (_showPolling) MockLog(msg); }

        // ═══════ 主站管理 ═══════

        public static ushort CS_ECAT_Master_Open(ref ushort existcard)
        { existcard = 1; MockLog("Master_Open → 1 張軸卡"); return 0; }

        public static ushort CS_ECAT_Master_Get_CardSeq(ushort CardNo_seq, ref ushort CardNo)
        { CardNo = 0; MockLog($"Master_Get_CardSeq → CardNo=0"); return 0; }

        public static ushort CS_ECAT_Master_Initial(ushort CardNo)
        { MockLog($"Master_Initial(CardNo={CardNo})"); return 0; }

        public static ushort CS_ECAT_Master_Check_Initial_Done(ushort CardNo, ref ushort InitDone)
        {
            if (!_initialized) { _initialized = true; InitDone = 1; }
            else { InitDone = 0; }
            return 0;
        }

        public static ushort CS_ECAT_Master_Close()
        { _initialized = false; MockLog("Master_Close"); return 0; }

        public static ushort CS_ECAT_Master_Reset(ushort CardNo) { return 0; }

        public static ushort CS_ECAT_Master_Get_SlaveNum(ushort CardNo, ref ushort SlaveNum)
        { SlaveNum = 6; MockLog("Master_Get_SlaveNum → 6"); return 0; }

        public static ushort CS_ECAT_Master_Get_Connect_Status(ushort CardNo, ref ushort Status)
        { Status = 1; return 0; }

        public static ushort CS_ECAT_Master_Get_Return_Code_Message(ushort ReturnCode, string Message) => 0;
        public static ushort CS_ECAT_Master_Get_Hardware_ID(ushort CardNo, ref ushort DeviceID) { DeviceID = 0x434; return 0; }
        public static ushort CS_ECAT_Master_Get_Initial_ErrorCode(ushort CardNo) => 0;

        // ═══════ 伺服控制 ═══════

        public static ushort CS_ECAT_Slave_Motion_Set_Svon(ushort CardNo, ushort NodeID, ushort SlotNo, ushort On_Off)
        { MockLog($"Set_Svon(軸{NodeID}, {(On_Off == 1 ? "ON" : "OFF")})"); return 0; }

        public static ushort CS_ECAT_Slave_Motion_Ralm(ushort CardNo, ushort NodeID, ushort SlotNo)
        {
            if (NodeID < 6) _statusWord[NodeID] = (ushort)(_statusWord[NodeID] & ~0x0008);
            MockLog($"Ralm(軸{NodeID})");
            return 0;
        }

        public static ushort CS_ECAT_Slave_Motion_Set_MoveMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort OpMode)
        {
            string m = OpMode switch { 8 => "CSP", 9 => "CSV", 10 => "CST", 6 => "HOME", _ => $"{OpMode}" };
            MockLog($"Set_MoveMode(軸{NodeID}, {m})");
            return 0;
        }

        // ═══════ 停止命令 ═══════

        public static ushort CS_ECAT_Slave_Motion_Emg_Stop(ushort CardNo, ushort NodeID, ushort SlotNo)
        {
            if (NodeID < 6) { _mdone[NodeID] = 0; _speed[NodeID] = 0; }
            MockLog($"🛑 Emg_Stop(軸{NodeID})");
            return 0;
        }

        public static ushort CS_ECAT_Slave_Motion_Sd_Stop(ushort CardNo, ushort NodeID, ushort SlotNo, double Tdec)
        {
            if (NodeID < 6) { _mdone[NodeID] = 0; _speed[NodeID] = 0; }
            MockLog($"Sd_Stop(軸{NodeID}, {Tdec}s)");
            return 0;
        }

        // ═══════ 狀態讀取 ═══════

        public static ushort CS_ECAT_Slave_Motion_Get_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Position)
        { if (NodeID < 6) Position = _position[NodeID]; return 0; }

        public static ushort CS_ECAT_Slave_Motion_Get_Actual_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int ActualPosition)
        { if (NodeID < 6) ActualPosition = _position[NodeID]; return 0; }

        public static ushort CS_ECAT_Slave_Motion_Get_Current_Speed(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Speed)
        { if (NodeID < 6) Speed = _speed[NodeID]; return 0; }

        public static ushort CS_ECAT_Slave_Motion_Get_Speed(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Speed)
        { if (NodeID < 6) Speed = _speed[NodeID]; return 0; }

        public static ushort CS_ECAT_Slave_Motion_Get_Mdone(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Mdone)
        { if (NodeID < 6) Mdone = _mdone[NodeID]; return 0; }

        public static ushort CS_ECAT_Slave_Motion_Get_StatusWord(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort StatusWord)
        { if (NodeID < 6) StatusWord = _statusWord[NodeID]; return 0; }

        public static ushort CS_ECAT_Slave_Motion_Get_Buffer_Length(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort BufferLength)
        { if (NodeID < 6) BufferLength = _bufferLength[NodeID]; return 0; }

        public static ushort CS_ECAT_Slave_Motion_Get_Command(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Command)
        { if (NodeID < 6) Command = _position[NodeID]; return 0; }

        public static ushort CS_ECAT_Slave_Motion_Get_Actual_Command(ushort CardNo, ushort NodeID, ushort SlotNo, ref int ActualCommand)
        { if (NodeID < 6) ActualCommand = _position[NodeID]; return 0; }

        public static ushort CS_ECAT_Slave_Motion_Get_Target_Command(ushort CardNo, ushort NodeID, ushort SlotNo, ref int TargetCommand)
        { if (NodeID < 6) TargetCommand = _position[NodeID]; return 0; }

        public static ushort CS_ECAT_Slave_Motion_Get_Torque(ushort CardNo, ushort NodeID, ushort SlotNo, ref short Torque) { Torque = 0; return 0; }
        public static ushort CS_ECAT_Slave_Motion_Get_ControlWord(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort ControllWord) { ControllWord = 0; return 0; }
        public static ushort CS_ECAT_Slave_Motion_Get_MoveMode(ushort CardNo, ushort NodeID, ushort SlotNo, ref byte Mode) { Mode = 8; return 0; }

        // ═══════ CSP 運動命令 ═══════

        public static ushort CS_ECAT_Slave_CSP_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo,
            int Dist, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs)
        {
            if (NodeID < 6) { _mdone[NodeID] = 2; _speed[NodeID] = ConstVel; _bufferLength[NodeID]++; }
            string mode = IsAbs == 1 ? "絕對" : "相對";
            MockLog($"CSP_Move(軸{NodeID}, {mode}, D={Dist}, V={ConstVel}, EndV={EndVel})");
            // 模擬延遲後完成
            if (NodeID < 6)
            {
                var nid = NodeID;
                Task.Run(async () =>
                {
                    await Task.Delay(200);
                    lock (_lock)
                    {
                        if (EndVel == 0) { _mdone[nid] = 0; _speed[nid] = 0; }
                        _position[nid] = IsAbs == 1 ? Dist : _position[nid] + Dist;
                        if (_bufferLength[nid] > 0) _bufferLength[nid]--;
                    }
                });
            }
            return 0;
        }

        public static ushort CS_ECAT_Slave_CSP_Start_V_Move(ushort CardNo, ushort NodeID, ushort SlotNo,
            ushort Dir, int StrVel, int ConstVel, double Tacc, ushort SCurve)
        {
            if (NodeID < 6) { _mdone[NodeID] = 2; _speed[NodeID] = Dir == 0 ? ConstVel : -ConstVel; }
            MockLog($"CSP_V_Move(軸{NodeID}, {(Dir == 0 ? "正向" : "負向")}, V={ConstVel})");
            return 0;
        }

        public static ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Move(ushort CardNo, ushort NodeID, ushort SlotID,
            int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs)
        {
            if (NodeID < 6) { _mdone[NodeID] = 2; _bufferLength[NodeID]++; }
            MockLog($"CSP_PVTComplete_Move(軸{NodeID}, {DataCnt}筆, StrV={StrVel}, EndV={EndVel})");
            if (NodeID < 6)
            {
                var nid = NodeID;
                Task.Run(async () =>
                {
                    await Task.Delay(300);
                    lock (_lock)
                    {
                        if (EndVel == 0) { _mdone[nid] = 0; _speed[nid] = 0; }
                        if (_bufferLength[nid] > 0) _bufferLength[nid]--;
                    }
                });
            }
            return 0;
        }

        public static ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Config(ushort CardNo, ushort NodeID, ushort SlotID,
            int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs)
        { MockLog($"CSP_PVTComplete_Config(軸{NodeID}, {DataCnt}筆)"); return 0; }

        public static ushort CS_ECAT_Slave_CSP_Start_PVT_Move(ushort CardNo, ushort NodeID, ushort SlotID,
            int DataCnt, ref int TargetPos, ref int TargetTime, ref int TargetVel, ushort Abs)
        { if (NodeID < 6) _mdone[NodeID] = 2; MockLog($"CSP_PVT_Move(軸{NodeID}, {DataCnt}筆)"); return 0; }

        public static ushort CS_ECAT_Slave_CSP_Start_PVT_Config(ushort CardNo, ushort NodeID, ushort SlotID,
            int DataCnt, ref int TargetPos, ref int TargetTime, ref int TargetVel, ushort Abs)
        { MockLog($"CSP_PVT_Config(軸{NodeID}, {DataCnt}筆)"); return 0; }

        public static ushort CS_ECAT_Slave_CSP_Start_PVT_Sync_Move(ushort CardNo, ushort AxisNum,
            ref ushort AxisArray, ref ushort SlotArray)
        {
            MockLog($"CSP_PVT_Sync_Move({AxisNum}軸同步)");
            for (int i = 0; i < Math.Min((int)AxisNum, 6); i++) _mdone[i] = 2;
            Task.Run(async () =>
            {
                await Task.Delay(300);
                lock (_lock) { for (int i = 0; i < 6; i++) { _mdone[i] = 0; _speed[i] = 0; } }
            });
            return 0;
        }

        // ═══════ CSP 設定 ═══════

        public static ushort CS_ECAT_Slave_CSP_Set_Gear(ushort CardNo, ushort NodeID, ushort SlotNo,
            double Numerator, double Denominator, short Enable)
        { MockLog($"CSP_Set_Gear(軸{NodeID}, {Numerator}/{Denominator}, E={Enable})"); return 0; }

        public static ushort CS_ECAT_Slave_CSP_Virtual_Set_Enable(ushort CardNo, ushort NodeID, ushort SlotID, ushort Enable)
        { MockLog($"Virtual_Set_Enable(軸{NodeID}, {(Enable == 1 ? "ON" : "OFF")})"); return 0; }

        public static ushort CS_ECAT_Slave_CSP_Virtual_Set_Command(ushort CardNo, ushort NodeID, ushort SlotID, int Command)
        { if (NodeID < 6) _position[NodeID] = Command; MockLog($"Virtual_Set_Command(軸{NodeID}, {Command})"); return 0; }

        public static ushort CS_ECAT_Slave_CSP_Set_Softlimit(ushort CardNo, ushort NodeID, ushort SlotNo, int PosiLimit, int NegaLimit, ushort Mode) => 0;
        public static ushort CS_ECAT_Slave_CSP_Speed_Continue_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable) => 0;
        public static ushort CS_ECAT_Slave_CSP_Speed_Continue_Set_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode) => 0;
        public static ushort CS_ECAT_Slave_CSP_PVT_Get_NowMotCount(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort NowMotCount) { NowMotCount = 0; return 0; }

        // ═══════ 其他 ═══════

        public static ushort CS_ECAT_Slave_Motion_Set_Position(ushort CardNo, ushort NodeID, ushort SlotNo, int Pos)
        { if (NodeID < 6) _position[NodeID] = Pos; return 0; }

        public static ushort CS_ECAT_Slave_Motion_Set_Command(ushort CardNo, ushort NodeID, ushort SlotNo, int Cmd) => 0;
        public static ushort CS_ECAT_Slave_Motion_Set_Alm_Reaction(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Fault_Type, ushort WR_Type) => 0;
        public static ushort CS_ECAT_Slave_Motion_Set_Command_Wait_Target_Reach(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Wait) => 0;
        public static ushort CS_ECAT_Slave_Motion_Ignore_Sd_Stop_Wrong_Value(ushort CardNo, byte Ignore) => 0;
        public static ushort CS_ECAT_Master_Get_Total_SlaveNum(ushort CardNo, ref ushort SlaveNum) { SlaveNum = 6; return 0; }
        public static ushort CS_ECAT_Master_Get_Local_SlaveNum(ushort CardNo, ref ushort SlaveNum) { SlaveNum = 0; return 0; }
        public static ushort CS_ECAT_Master_Get_Working_Counter_ErrorCounter(ushort CardNo, ref ushort Error_Cnt) { Error_Cnt = 0; return 0; }
        public static ushort CS_ECAT_Master_Check_Working_Counter(ushort CardNo, ref ushort Abnormal_Flag, ref ushort Working_Slave_Cnt) { Abnormal_Flag = 0; Working_Slave_Cnt = 6; return 0; }

        public struct MotionBuffer_IO_Information { public ushort NodeID; public ushort SlotID; public ushort BitNo; }
        public struct MotionBuffer_IO_Set { public MotionBuffer_IO_Information IO_Info; public ushort Value; }
    }
}
