using EtherCAT_DLL_x64;
using EtherCAT_DLL_Mock;

namespace Robot.Driver.Delta
{
    internal interface IEtherCatApi
    {
        ushort CS_ECAT_Master_Open(ref ushort existcard);
        ushort CS_ECAT_Master_Get_CardSeq(ushort CardNo_seq, ref ushort CardNo);
        ushort CS_ECAT_Master_Initial(ushort CardNo);
        ushort CS_ECAT_Master_Check_Initial_Done(ushort CardNo, ref ushort InitDone);
        ushort CS_ECAT_Master_Close();
        ushort CS_ECAT_Master_Get_SlaveNum(ushort CardNo, ref ushort SlaveNum);

        ushort CS_ECAT_Slave_Motion_Set_MoveMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort OpMode);
        ushort CS_ECAT_Slave_CSP_Set_Gear(ushort CardNo, ushort NodeID, ushort SlotNo, double Numerator, double Denominator, short Enable);
        ushort CS_ECAT_Slave_CSP_Virtual_Set_Enable(ushort CardNo, ushort NodeID, ushort SlotID, ushort Enable);
        ushort CS_ECAT_Slave_CSP_Virtual_Set_Command(ushort CardNo, ushort NodeID, ushort SlotID, int Command);
        ushort CS_ECAT_Slave_Motion_Set_Svon(ushort CardNo, ushort NodeID, ushort SlotNo, ushort On_Off);

        ushort CS_ECAT_Slave_Motion_Get_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Position);
        ushort CS_ECAT_Slave_Motion_Get_Current_Speed(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Speed);
        ushort CS_ECAT_Slave_Motion_Get_Mdone(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Mdone);
        ushort CS_ECAT_Slave_Motion_Get_StatusWord(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort StatusWord);
        ushort CS_ECAT_Slave_Motion_Get_Buffer_Length(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort BufferLength);

        ushort CS_ECAT_Slave_Motion_Emg_Stop(ushort CardNo, ushort NodeID, ushort SlotNo);
        ushort CS_ECAT_Slave_Motion_Sd_Stop(ushort CardNo, ushort NodeID, ushort SlotNo, double Tdec);
        ushort CS_ECAT_Slave_Motion_Ralm(ushort CardNo, ushort NodeID, ushort SlotNo);

        ushort CS_ECAT_Slave_CSP_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo,
            int Dist, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs);

        ushort CS_ECAT_Slave_CSP_Start_V_Move(ushort CardNo, ushort NodeID, ushort SlotNo,
            ushort Dir, int StrVel, int ConstVel, double Tacc, ushort SCurve);

        ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Move(ushort CardNo, ushort NodeID, ushort SlotID,
            int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs);

        ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Config(ushort CardNo, ushort NodeID, ushort SlotID,
            int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs);

        ushort CS_ECAT_Slave_CSP_Start_PVT_Sync_Move(ushort CardNo, ushort AxisNum,
            ref ushort AxisArray, ref ushort SlotArray);
    }

    internal sealed class RealEtherCatApi : IEtherCatApi
    {
        public ushort CS_ECAT_Master_Open(ref ushort existcard) => CEtherCAT_DLL.CS_ECAT_Master_Open(ref existcard);
        public ushort CS_ECAT_Master_Get_CardSeq(ushort CardNo_seq, ref ushort CardNo) => CEtherCAT_DLL.CS_ECAT_Master_Get_CardSeq(CardNo_seq, ref CardNo);
        public ushort CS_ECAT_Master_Initial(ushort CardNo) => CEtherCAT_DLL.CS_ECAT_Master_Initial(CardNo);
        public ushort CS_ECAT_Master_Check_Initial_Done(ushort CardNo, ref ushort InitDone) => CEtherCAT_DLL.CS_ECAT_Master_Check_Initial_Done(CardNo, ref InitDone);
        public ushort CS_ECAT_Master_Close() => CEtherCAT_DLL.CS_ECAT_Master_Close();
        public ushort CS_ECAT_Master_Get_SlaveNum(ushort CardNo, ref ushort SlaveNum) => CEtherCAT_DLL.CS_ECAT_Master_Get_SlaveNum(CardNo, ref SlaveNum);

        public ushort CS_ECAT_Slave_Motion_Set_MoveMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort OpMode) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Set_MoveMode(CardNo, NodeID, SlotNo, OpMode);
        public ushort CS_ECAT_Slave_CSP_Set_Gear(ushort CardNo, ushort NodeID, ushort SlotNo, double Numerator, double Denominator, short Enable) => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Set_Gear(CardNo, NodeID, SlotNo, Numerator, Denominator, Enable);
        public ushort CS_ECAT_Slave_CSP_Virtual_Set_Enable(ushort CardNo, ushort NodeID, ushort SlotID, ushort Enable) => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Virtual_Set_Enable(CardNo, NodeID, SlotID, Enable);
        public ushort CS_ECAT_Slave_CSP_Virtual_Set_Command(ushort CardNo, ushort NodeID, ushort SlotID, int Command) => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Virtual_Set_Command(CardNo, NodeID, SlotID, Command);
        public ushort CS_ECAT_Slave_Motion_Set_Svon(ushort CardNo, ushort NodeID, ushort SlotNo, ushort On_Off) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Set_Svon(CardNo, NodeID, SlotNo, On_Off);

        public ushort CS_ECAT_Slave_Motion_Get_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Position) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Get_Position(CardNo, NodeID, SlotNo, ref Position);
        public ushort CS_ECAT_Slave_Motion_Get_Current_Speed(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Speed) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Get_Current_Speed(CardNo, NodeID, SlotNo, ref Speed);
        public ushort CS_ECAT_Slave_Motion_Get_Mdone(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Mdone) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Get_Mdone(CardNo, NodeID, SlotNo, ref Mdone);
        public ushort CS_ECAT_Slave_Motion_Get_StatusWord(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort StatusWord) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Get_StatusWord(CardNo, NodeID, SlotNo, ref StatusWord);
        public ushort CS_ECAT_Slave_Motion_Get_Buffer_Length(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort BufferLength) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Get_Buffer_Length(CardNo, NodeID, SlotNo, ref BufferLength);

        public ushort CS_ECAT_Slave_Motion_Emg_Stop(ushort CardNo, ushort NodeID, ushort SlotNo) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Emg_Stop(CardNo, NodeID, SlotNo);
        public ushort CS_ECAT_Slave_Motion_Sd_Stop(ushort CardNo, ushort NodeID, ushort SlotNo, double Tdec) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Sd_Stop(CardNo, NodeID, SlotNo, Tdec);
        public ushort CS_ECAT_Slave_Motion_Ralm(ushort CardNo, ushort NodeID, ushort SlotNo) => CEtherCAT_DLL.CS_ECAT_Slave_Motion_Ralm(CardNo, NodeID, SlotNo);

        public ushort CS_ECAT_Slave_CSP_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo, int Dist, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs)
            => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Start_Move(CardNo, NodeID, SlotNo, Dist, StrVel, ConstVel, EndVel, Tacc, Tdec, SCurve, IsAbs);

        public ushort CS_ECAT_Slave_CSP_Start_V_Move(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Dir, int StrVel, int ConstVel, double Tacc, ushort SCurve)
            => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Start_V_Move(CardNo, NodeID, SlotNo, Dir, StrVel, ConstVel, Tacc, SCurve);

        public ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Move(ushort CardNo, ushort NodeID, ushort SlotID, int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs)
            => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Start_PVTComplete_Move(CardNo, NodeID, SlotID, DataCnt, ref TargetPos, ref TargetTime, StrVel, EndVel, Abs);

        public ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Config(ushort CardNo, ushort NodeID, ushort SlotID, int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs)
            => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Start_PVTComplete_Config(CardNo, NodeID, SlotID, DataCnt, ref TargetPos, ref TargetTime, StrVel, EndVel, Abs);

        public ushort CS_ECAT_Slave_CSP_Start_PVT_Sync_Move(ushort CardNo, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray)
            => CEtherCAT_DLL.CS_ECAT_Slave_CSP_Start_PVT_Sync_Move(CardNo, AxisNum, ref AxisArray, ref SlotArray);
    }

    internal sealed class MockEtherCatApi : IEtherCatApi
    {
        public ushort CS_ECAT_Master_Open(ref ushort existcard) => CEtherCAT_Mock.CS_ECAT_Master_Open(ref existcard);
        public ushort CS_ECAT_Master_Get_CardSeq(ushort CardNo_seq, ref ushort CardNo) => CEtherCAT_Mock.CS_ECAT_Master_Get_CardSeq(CardNo_seq, ref CardNo);
        public ushort CS_ECAT_Master_Initial(ushort CardNo) => CEtherCAT_Mock.CS_ECAT_Master_Initial(CardNo);
        public ushort CS_ECAT_Master_Check_Initial_Done(ushort CardNo, ref ushort InitDone) => CEtherCAT_Mock.CS_ECAT_Master_Check_Initial_Done(CardNo, ref InitDone);
        public ushort CS_ECAT_Master_Close() => CEtherCAT_Mock.CS_ECAT_Master_Close();
        public ushort CS_ECAT_Master_Get_SlaveNum(ushort CardNo, ref ushort SlaveNum) => CEtherCAT_Mock.CS_ECAT_Master_Get_SlaveNum(CardNo, ref SlaveNum);

        public ushort CS_ECAT_Slave_Motion_Set_MoveMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort OpMode) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Set_MoveMode(CardNo, NodeID, SlotNo, OpMode);
        public ushort CS_ECAT_Slave_CSP_Set_Gear(ushort CardNo, ushort NodeID, ushort SlotNo, double Numerator, double Denominator, short Enable) => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Set_Gear(CardNo, NodeID, SlotNo, Numerator, Denominator, Enable);
        public ushort CS_ECAT_Slave_CSP_Virtual_Set_Enable(ushort CardNo, ushort NodeID, ushort SlotID, ushort Enable) => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Virtual_Set_Enable(CardNo, NodeID, SlotID, Enable);
        public ushort CS_ECAT_Slave_CSP_Virtual_Set_Command(ushort CardNo, ushort NodeID, ushort SlotID, int Command) => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Virtual_Set_Command(CardNo, NodeID, SlotID, Command);
        public ushort CS_ECAT_Slave_Motion_Set_Svon(ushort CardNo, ushort NodeID, ushort SlotNo, ushort On_Off) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Set_Svon(CardNo, NodeID, SlotNo, On_Off);

        public ushort CS_ECAT_Slave_Motion_Get_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Position) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Get_Position(CardNo, NodeID, SlotNo, ref Position);
        public ushort CS_ECAT_Slave_Motion_Get_Current_Speed(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Speed) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Get_Current_Speed(CardNo, NodeID, SlotNo, ref Speed);
        public ushort CS_ECAT_Slave_Motion_Get_Mdone(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Mdone) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Get_Mdone(CardNo, NodeID, SlotNo, ref Mdone);
        public ushort CS_ECAT_Slave_Motion_Get_StatusWord(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort StatusWord) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Get_StatusWord(CardNo, NodeID, SlotNo, ref StatusWord);
        public ushort CS_ECAT_Slave_Motion_Get_Buffer_Length(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort BufferLength) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Get_Buffer_Length(CardNo, NodeID, SlotNo, ref BufferLength);

        public ushort CS_ECAT_Slave_Motion_Emg_Stop(ushort CardNo, ushort NodeID, ushort SlotNo) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Emg_Stop(CardNo, NodeID, SlotNo);
        public ushort CS_ECAT_Slave_Motion_Sd_Stop(ushort CardNo, ushort NodeID, ushort SlotNo, double Tdec) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Sd_Stop(CardNo, NodeID, SlotNo, Tdec);
        public ushort CS_ECAT_Slave_Motion_Ralm(ushort CardNo, ushort NodeID, ushort SlotNo) => CEtherCAT_Mock.CS_ECAT_Slave_Motion_Ralm(CardNo, NodeID, SlotNo);

        public ushort CS_ECAT_Slave_CSP_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo, int Dist, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs)
            => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Start_Move(CardNo, NodeID, SlotNo, Dist, StrVel, ConstVel, EndVel, Tacc, Tdec, SCurve, IsAbs);

        public ushort CS_ECAT_Slave_CSP_Start_V_Move(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Dir, int StrVel, int ConstVel, double Tacc, ushort SCurve)
            => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Start_V_Move(CardNo, NodeID, SlotNo, Dir, StrVel, ConstVel, Tacc, SCurve);

        public ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Move(ushort CardNo, ushort NodeID, ushort SlotID, int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs)
            => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Start_PVTComplete_Move(CardNo, NodeID, SlotID, DataCnt, ref TargetPos, ref TargetTime, StrVel, EndVel, Abs);

        public ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Config(ushort CardNo, ushort NodeID, ushort SlotID, int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs)
            => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Start_PVTComplete_Config(CardNo, NodeID, SlotID, DataCnt, ref TargetPos, ref TargetTime, StrVel, EndVel, Abs);

        public ushort CS_ECAT_Slave_CSP_Start_PVT_Sync_Move(ushort CardNo, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray)
            => CEtherCAT_Mock.CS_ECAT_Slave_CSP_Start_PVT_Sync_Move(CardNo, AxisNum, ref AxisArray, ref SlotArray);
    }
}
