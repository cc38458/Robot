using EtherCAT_DLL_x64;
using EtherCAT_DLL_Mock;

namespace Robot.Driver.Delta
{
    /// <summary>
    /// EtherCAT DLL 抽象介面，統一 Real/Mock 後端的呼叫簽章。
    /// 所有方法回傳 ushort（0 = 成功，非 0 = 錯誤碼）。
    /// </summary>
    internal interface IEtherCatApi
    {
        // ── 主站管理 ──

        /// <summary>開啟主站並取得軸卡數量。</summary>
        ushort CS_ECAT_Master_Open(ref ushort existcard);
        /// <summary>取得指定序號的軸卡編號。</summary>
        ushort CS_ECAT_Master_Get_CardSeq(ushort CardNo_seq, ref ushort CardNo);
        /// <summary>初始化指定軸卡。</summary>
        ushort CS_ECAT_Master_Initial(ushort CardNo);
        /// <summary>檢查軸卡初始化是否完成（0=完成, 1=進行中, 99=失敗）。</summary>
        ushort CS_ECAT_Master_Check_Initial_Done(ushort CardNo, ref ushort InitDone);
        /// <summary>關閉主站。</summary>
        ushort CS_ECAT_Master_Close();
        /// <summary>取得指定軸卡的從站（軸）數量。</summary>
        ushort CS_ECAT_Master_Get_SlaveNum(ushort CardNo, ref ushort SlaveNum);

        // ── 軸設定 ──

        /// <summary>設定運動模式（例如 8 = CSP）。</summary>
        ushort CS_ECAT_Slave_Motion_Set_MoveMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort OpMode);
        /// <summary>設定齒輪比（Numerator/Denominator）。</summary>
        ushort CS_ECAT_Slave_CSP_Set_Gear(ushort CardNo, ushort NodeID, ushort SlotNo, double Numerator, double Denominator, short Enable);
        /// <summary>啟用/停用虛擬座標模式。</summary>
        ushort CS_ECAT_Slave_CSP_Virtual_Set_Enable(ushort CardNo, ushort NodeID, ushort SlotID, ushort Enable);
        /// <summary>設定虛擬座標命令值。</summary>
        ushort CS_ECAT_Slave_CSP_Virtual_Set_Command(ushort CardNo, ushort NodeID, ushort SlotID, int Command);
        /// <summary>設定 Servo ON/OFF（1=ON, 0=OFF）。</summary>
        ushort CS_ECAT_Slave_Motion_Set_Svon(ushort CardNo, ushort NodeID, ushort SlotNo, ushort On_Off);

        // ── 狀態讀取 ──

        /// <summary>讀取虛擬座標位置。</summary>
        ushort CS_ECAT_Slave_Motion_Get_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Position);
        /// <summary>讀取實際編碼器位置。</summary>
        ushort CS_ECAT_Slave_Motion_Get_Actual_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Position);
        /// <summary>讀取目前速度。</summary>
        ushort CS_ECAT_Slave_Motion_Get_Current_Speed(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Speed);
        /// <summary>讀取運動完成旗標（0=完成, 非0=運動中）。</summary>
        ushort CS_ECAT_Slave_Motion_Get_Mdone(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Mdone);
        /// <summary>讀取狀態字（bit3=警報）。</summary>
        ushort CS_ECAT_Slave_Motion_Get_StatusWord(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort StatusWord);
        /// <summary>讀取硬體 buffer 目前已排隊筆數。</summary>
        ushort CS_ECAT_Slave_Motion_Get_Buffer_Length(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort BufferLength);

        // ── 安全操作 ──

        /// <summary>緊急停止（立即斷電）。</summary>
        ushort CS_ECAT_Slave_Motion_Emg_Stop(ushort CardNo, ushort NodeID, ushort SlotNo);
        /// <summary>減速停止。</summary>
        ushort CS_ECAT_Slave_Motion_Sd_Stop(ushort CardNo, ushort NodeID, ushort SlotNo, double Tdec);
        /// <summary>警報復歸。</summary>
        ushort CS_ECAT_Slave_Motion_Ralm(ushort CardNo, ushort NodeID, ushort SlotNo);

        // ── 運動控制 ──

        /// <summary>變更目標速度。</summary>
        ushort CS_ECAT_Slave_CSP_Velocity_Change(ushort CardNo, ushort NodeID, ushort SlotNo, int NewSpeed, double Tsec);
        /// <summary>立即變更 CSP 目標位置。</summary>
        ushort CS_ECAT_Slave_CSP_TargetPos_Change(ushort CardNo, ushort NodeID, ushort SlotNo, int NewPos);

        /// <summary>CSP 位移運動（支援絕對/相對）。</summary>
        ushort CS_ECAT_Slave_CSP_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo,
            int Dist, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs);

        /// <summary>CSP 等速運動（指定方向與速度）。</summary>
        ushort CS_ECAT_Slave_CSP_Start_V_Move(ushort CardNo, ushort NodeID, ushort SlotNo,
            ushort Dir, int StrVel, int ConstVel, double Tacc, ushort SCurve);

        /// <summary>PVT 完整運動（單軸，一次下達全部點位）。</summary>
        ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Move(ushort CardNo, ushort NodeID, ushort SlotID,
            int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs);

        /// <summary>PVT 設定（多軸同步前的逐軸設定）。</summary>
        ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Config(ushort CardNo, ushort NodeID, ushort SlotID,
            int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs);

        /// <summary>多軸同步啟動 PVT 運動。</summary>
        ushort CS_ECAT_Slave_CSP_Start_PVT_Sync_Move(ushort CardNo, ushort AxisNum,
            ref ushort AxisArray, ref ushort SlotArray);

        /// <summary>多軸同步終止並切換至指定位置（CSP Abort and Change Position）。</summary>
        ushort CS_ECAT_Slave_CSP_Abort_and_Change_Position(ushort CardNo, ushort Axes,
            ref ushort NodeID, ref ushort SlotID, ref int Dist,
            int MaxVel, int EndVel, double Tacc, double Tdec, ushort CurveMode);
    }

    /// <summary>
    /// 實體 EtherCAT DLL 轉接器 — 將 IEtherCatApi 呼叫直接委派至 CEtherCAT_DLL（P/Invoke）。
    /// </summary>
    internal sealed class RealEtherCatApi : IEtherCatApi
    {
        /// <inheritdoc />
        public ushort CS_ECAT_Master_Open(ref ushort existcard) => CEtherCAT_DLL.CS_ECAT_Master_Open(ref existcard);
        /// <inheritdoc />
        public ushort CS_ECAT_Master_Get_CardSeq(ushort CardNo_seq, ref ushort CardNo) => CEtherCAT_DLL.CS_ECAT_Master_Get_CardSeq(CardNo_seq, ref CardNo);
        /// <inheritdoc />
        public ushort CS_ECAT_Master_Initial(ushort CardNo) => CEtherCAT_DLL.CS_ECAT_Master_Initial(CardNo);
        /// <inheritdoc />
        public ushort CS_ECAT_Master_Check_Initial_Done(ushort CardNo, ref ushort InitDone) => CEtherCAT_DLL.CS_ECAT_Master_Check_Initial_Done(CardNo, ref InitDone);
        /// <inheritdoc />
        public ushort CS_ECAT_Master_Close() => CEtherCAT_DLL.CS_ECAT_Master_Close();
        /// <inheritdoc />
        public ushort CS_ECAT_Master_Get_SlaveNum(ushort CardNo, ref ushort SlaveNum) => CEtherCAT_DLL.CS_ECAT_Master_Get_SlaveNum(CardNo, ref SlaveNum);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Set_MoveMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort OpMode) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Set_MoveMode(CardNo, NodeID, SlotNo, OpMode);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Set_Gear(ushort CardNo, ushort NodeID, ushort SlotNo, double Numerator, double Denominator, short Enable) => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Set_Gear(CardNo, NodeID, SlotNo, Numerator, Denominator, Enable);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Virtual_Set_Enable(ushort CardNo, ushort NodeID, ushort SlotID, ushort Enable) => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Virtual_Set_Enable(CardNo, NodeID, SlotID, Enable);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Virtual_Set_Command(ushort CardNo, ushort NodeID, ushort SlotID, int Command) => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Virtual_Set_Command(CardNo, NodeID, SlotID, Command);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Set_Svon(ushort CardNo, ushort NodeID, ushort SlotNo, ushort On_Off) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Set_Svon(CardNo, NodeID, SlotNo, On_Off);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Get_Actual_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Position) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Get_Actual_Position(CardNo, NodeID, SlotNo, ref Position);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Get_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Position) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Get_Position(CardNo, NodeID, SlotNo, ref Position);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Get_Current_Speed(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Speed) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Get_Current_Speed(CardNo, NodeID, SlotNo, ref Speed);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Get_Mdone(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Mdone) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Get_Mdone(CardNo, NodeID, SlotNo, ref Mdone);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Get_StatusWord(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort StatusWord) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Get_StatusWord(CardNo, NodeID, SlotNo, ref StatusWord);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Get_Buffer_Length(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort BufferLength) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Get_Buffer_Length(CardNo, NodeID, SlotNo, ref BufferLength);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Emg_Stop(ushort CardNo, ushort NodeID, ushort SlotNo) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Emg_Stop(CardNo, NodeID, SlotNo);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Sd_Stop(ushort CardNo, ushort NodeID, ushort SlotNo, double Tdec) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Sd_Stop(CardNo, NodeID, SlotNo, Tdec);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Ralm(ushort CardNo, ushort NodeID, ushort SlotNo) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Ralm(CardNo, NodeID, SlotNo);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Velocity_Change(ushort CardNo, ushort NodeID, ushort SlotNo, int NewSpeed, double Tsec) => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Velocity_Change(CardNo, NodeID, SlotNo, NewSpeed, Tsec);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_TargetPos_Change(ushort CardNo, ushort NodeID, ushort SlotNo, int NewPos) => CEtherCAT_DLL.CS_ECAT_Slave_CSP_TargetPos_Change(CardNo, NodeID, SlotNo, NewPos);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo, int Dist, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs)
            => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Start_Move(CardNo, NodeID, SlotNo, Dist, StrVel, ConstVel, EndVel, Tacc, Tdec, SCurve, IsAbs);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Start_V_Move(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Dir, int StrVel, int ConstVel, double Tacc, ushort SCurve)
            => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Start_V_Move(CardNo, NodeID, SlotNo, Dir, StrVel, ConstVel, Tacc, SCurve);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Move(ushort CardNo, ushort NodeID, ushort SlotID, int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs)
            => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Start_PVTComplete_Move(CardNo, NodeID, SlotID, DataCnt, ref TargetPos, ref TargetTime, StrVel, EndVel, Abs);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Config(ushort CardNo, ushort NodeID, ushort SlotID, int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs)
            => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Start_PVTComplete_Config(CardNo, NodeID, SlotID, DataCnt, ref TargetPos, ref TargetTime, StrVel, EndVel, Abs);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Start_PVT_Sync_Move(ushort CardNo, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray)
            => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Start_PVT_Sync_Move(CardNo, AxisNum, ref AxisArray, ref SlotArray);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Abort_and_Change_Position(ushort CardNo, ushort Axes,
            ref ushort NodeID, ref ushort SlotID, ref int Dist,
            int MaxVel, int EndVel, double Tacc, double Tdec, ushort CurveMode)
            => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Abort_and_Change_Position(CardNo, Axes, ref NodeID, ref SlotID, ref Dist, MaxVel, EndVel, Tacc, Tdec, CurveMode);
    }

    /// <summary>
    /// Mock EtherCAT DLL 轉接器 — 將 IEtherCatApi 呼叫委派至 CEtherCAT_Mock（記憶體模擬）。
    /// </summary>
    internal sealed class MockEtherCatApi : IEtherCatApi
    {
        /// <inheritdoc />
        public ushort CS_ECAT_Master_Open(ref ushort existcard) => CEtherCAT_Mock.CS_ECAT_Master_Open(ref existcard);
        /// <inheritdoc />
        public ushort CS_ECAT_Master_Get_CardSeq(ushort CardNo_seq, ref ushort CardNo) => CEtherCAT_Mock.CS_ECAT_Master_Get_CardSeq(CardNo_seq, ref CardNo);
        /// <inheritdoc />
        public ushort CS_ECAT_Master_Initial(ushort CardNo) => CEtherCAT_Mock.CS_ECAT_Master_Initial(CardNo);
        /// <inheritdoc />
        public ushort CS_ECAT_Master_Check_Initial_Done(ushort CardNo, ref ushort InitDone) => CEtherCAT_Mock.CS_ECAT_Master_Check_Initial_Done(CardNo, ref InitDone);
        /// <inheritdoc />
        public ushort CS_ECAT_Master_Close() => CEtherCAT_Mock.CS_ECAT_Master_Close();
        /// <inheritdoc />
        public ushort CS_ECAT_Master_Get_SlaveNum(ushort CardNo, ref ushort SlaveNum) => CEtherCAT_Mock.CS_ECAT_Master_Get_SlaveNum(CardNo, ref SlaveNum);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Set_MoveMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort OpMode) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Set_MoveMode(CardNo, NodeID, SlotNo, OpMode);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Set_Gear(ushort CardNo, ushort NodeID, ushort SlotNo, double Numerator, double Denominator, short Enable) => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Set_Gear(CardNo, NodeID, SlotNo, Numerator, Denominator, Enable);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Virtual_Set_Enable(ushort CardNo, ushort NodeID, ushort SlotID, ushort Enable) => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Virtual_Set_Enable(CardNo, NodeID, SlotID, Enable);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Virtual_Set_Command(ushort CardNo, ushort NodeID, ushort SlotID, int Command) => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Virtual_Set_Command(CardNo, NodeID, SlotID, Command);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Set_Svon(ushort CardNo, ushort NodeID, ushort SlotNo, ushort On_Off) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Set_Svon(CardNo, NodeID, SlotNo, On_Off);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Get_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Position) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Get_Position(CardNo, NodeID, SlotNo, ref Position);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Get_Actual_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Position) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Get_Actual_Position(CardNo, NodeID, SlotNo, ref Position);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Get_Current_Speed(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Speed) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Get_Current_Speed(CardNo, NodeID, SlotNo, ref Speed);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Get_Mdone(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Mdone) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Get_Mdone(CardNo, NodeID, SlotNo, ref Mdone);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Get_StatusWord(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort StatusWord) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Get_StatusWord(CardNo, NodeID, SlotNo, ref StatusWord);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Get_Buffer_Length(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort BufferLength) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Get_Buffer_Length(CardNo, NodeID, SlotNo, ref BufferLength);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Emg_Stop(ushort CardNo, ushort NodeID, ushort SlotNo) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Emg_Stop(CardNo, NodeID, SlotNo);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Sd_Stop(ushort CardNo, ushort NodeID, ushort SlotNo, double Tdec) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Sd_Stop(CardNo, NodeID, SlotNo, Tdec);
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_Motion_Ralm(ushort CardNo, ushort NodeID, ushort SlotNo) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Ralm(CardNo, NodeID, SlotNo);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Velocity_Change(ushort CardNo, ushort NodeID, ushort SlotNo, int NewSpeed, double Tsec) => 0; // Mock 直接回傳成功
        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_TargetPos_Change(ushort CardNo, ushort NodeID, ushort SlotNo, int NewPos) => 0; // Mock 直接回傳成功

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo, int Dist, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs)
            => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Start_Move(CardNo, NodeID, SlotNo, Dist, StrVel, ConstVel, EndVel, Tacc, Tdec, SCurve, IsAbs);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Start_V_Move(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Dir, int StrVel, int ConstVel, double Tacc, ushort SCurve)
            => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Start_V_Move(CardNo, NodeID, SlotNo, Dir, StrVel, ConstVel, Tacc, SCurve);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Move(ushort CardNo, ushort NodeID, ushort SlotID, int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs)
            => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Start_PVTComplete_Move(CardNo, NodeID, SlotID, DataCnt, ref TargetPos, ref TargetTime, StrVel, EndVel, Abs);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Config(ushort CardNo, ushort NodeID, ushort SlotID, int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs)
            => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Start_PVTComplete_Config(CardNo, NodeID, SlotID, DataCnt, ref TargetPos, ref TargetTime, StrVel, EndVel, Abs);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Start_PVT_Sync_Move(ushort CardNo, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray)
            => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Start_PVT_Sync_Move(CardNo, AxisNum, ref AxisArray, ref SlotArray);

        /// <inheritdoc />
        public ushort CS_ECAT_Slave_CSP_Abort_and_Change_Position(ushort CardNo, ushort Axes,
            ref ushort NodeID, ref ushort SlotID, ref int Dist,
            int MaxVel, int EndVel, double Tacc, double Tdec, ushort CurveMode)
            => 0; // Mock 直接回傳成功
    }
}
