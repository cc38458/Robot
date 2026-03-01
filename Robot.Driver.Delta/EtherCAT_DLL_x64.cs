using System;
using System.Text;
using System.Runtime.InteropServices;

namespace EtherCAT_DLL_x64
{
    public class CEtherCAT_DLL
  {
    public struct MotionBuffer_IO_Information
    {
      public ushort NodeID;
      public ushort SlotID;
      public ushort BitNo;
    };

    public struct MotionBuffer_IO_Set
    {
      public MotionBuffer_IO_Information IO_Info;
      public ushort Value;
    };
    const String strDLL_Path = "EtherCAT_DLL_x64.dll";
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Open")]
    public static extern ushort CS_ECAT_Master_Open(ref ushort existcard);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Initial")]
    public static extern ushort CS_ECAT_Master_Initial(ushort CardNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Reset")]
    public static extern ushort CS_ECAT_Master_Reset(ushort CardNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Close")]
    public static extern ushort CS_ECAT_Master_Close();
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_CardSeq")]
    public static extern ushort CS_ECAT_Master_Get_CardSeq(ushort CardNo_seq, ref ushort CardNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_SlaveNum")]
    public static extern ushort CS_ECAT_Master_Get_SlaveNum(ushort CardNo, ref ushort SlaveNum);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_Local_SlaveNum")]
    public static extern ushort CS_ECAT_Master_Get_Local_SlaveNum(ushort CardNo, ref ushort SlaveNum);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_Total_SlaveNum")]
    public static extern ushort CS_ECAT_Master_Get_Total_SlaveNum(ushort CardNo, ref ushort SlaveNum);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_Slave_Info")]
    public static extern ushort CS_ECAT_Master_Get_Slave_Info(ushort CardNo, ushort SeqID, ref ushort NodeID, ref uint VendorID, ref uint ProductCode, ref uint RevisionNo, ref uint DCTime);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_DC_Status")]
    public static extern ushort CS_ECAT_Master_Get_DC_Status(ushort CardNo, ref uint State, ref int DCTime, ref int OffsetTime);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_Connect_Status")]
    public static extern ushort CS_ECAT_Master_Get_Connect_Status(ushort CardNo, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_Api_BufferLength")]
    public static extern ushort CS_ECAT_Master_Get_Api_BufferLength(ushort CardNo, ushort NodeID, ref ushort Length);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_Cycle_SpendTime")]
    public static extern ushort CS_ECAT_Master_Get_Cycle_SpendTime(ushort CardNo, ref double Tx_Time, ref double Tx_MaxTime, ref double Rx_Time, ref double Rx_MaxTime);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Check_Initial_Done")]
    public static extern ushort CS_ECAT_Master_Check_Initial_Done(ushort CardNo, ref ushort InitDone);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_Initial_ErrorCode")]
    public static extern ushort CS_ECAT_Master_Get_Initial_ErrorCode(ushort CardNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Check_Working_Counter")]
    public static extern ushort CS_ECAT_Master_Check_Working_Counter(ushort CardNo, ref ushort Abnormal_Flag, ref ushort Working_Slave_Cnt);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Set_Working_Counter_Value")]
    public static extern ushort CS_ECAT_Master_Set_Working_Counter_Value(ushort CardNo, ushort Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_Return_Code_Message")]
    public static extern ushort CS_ECAT_Master_Get_Return_Code_Message(ushort ReturnCode, string Message);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_Hardware_ID")]
    public static extern ushort CS_ECAT_Master_Get_Hardware_ID(ushort CardNo, ref ushort DeviceID);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_Working_Counter_ErrorCounter")]
    public static extern ushort CS_ECAT_Master_Get_Working_Counter_ErrorCounter(ushort CardNo, ref ushort Error_Cnt);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_Frame_Counter")]
    public static extern ushort CS_ECAT_Master_Get_Frame_Counter(ushort CardNo, ref uint TxFrameCnt, ref uint RxFrameCnt, ref uint Frame_Error_Cnt);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Reference_System_Time")]
    public static extern ushort CS_ECAT_Master_Reference_System_Time(ushort CardNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Set_CycleTime")]
    public static extern ushort CS_ECAT_Master_Set_CycleTime(ushort CardNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_CycleTime")]
    public static extern ushort CS_ECAT_Master_Get_CycleTime(ushort CardNo, ref ushort CycleTime);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Set_DCSyncShiftTime")]
    public static extern ushort CS_ECAT_Master_Set_DCSyncShiftTime(ushort CardNo, ushort ShiftTime_Percent);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_DLL_SeqID")]
    public static extern ushort CS_ECAT_Master_Get_DLL_SeqID(ushort CardNo, ref ushort SeqID);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_SerialNo")]
    public static extern ushort CS_ECAT_Master_Get_SerialNo(ushort CardNo, ref uint SerialNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_NodeID_Alias_Enable")]
    public static extern ushort CS_ECAT_Master_NodeID_Alias_Enable(ushort CardNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Autoconfig_Open_File")]
    public static extern ushort CS_ECAT_Autoconfig_Open_File(ushort CardNo, string FilePath);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Autoconfig_Save_File")]
    public static extern ushort CS_ECAT_Autoconfig_Save_File(ushort CardNo, string FilePath);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Autoconfig_Set_Slave_DCTime")]
    public static extern ushort CS_ECAT_Autoconfig_Set_Slave_DCTime(ushort CardNo, ushort NodeID, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Autoconfig_Clear_ConfigFile")]
    public static extern ushort CS_ECAT_Autoconfig_Clear_ConfigFile(ushort CardNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Autoconfig_Set_NodeID_Alias")]
    public static extern ushort CS_ECAT_Autoconfig_Set_NodeID_Alias(ushort CardNo, ushort NodeID, ushort MapNodeID);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Autoconfig_Get_NodeID_Alias")]
    public static extern ushort CS_ECAT_Autoconfig_Get_NodeID_Alias(ushort CardNo, ushort RealNodeID, ref ushort MapNodeID);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Autoconfig_Save_NodeID_Alias")]
    public static extern ushort CS_ECAT_Autoconfig_Save_NodeID_Alias(ushort CardNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_SDO_Send_Message")]
    public static extern ushort CS_ECAT_Slave_SDO_Send_Message(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Index, ushort SubIndex, ushort Datasize, ref byte Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_SDO_Read_Message")]
    public static extern ushort CS_ECAT_Slave_SDO_Read_Message(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Index, ushort SubIndex, ushort Datasize, ref byte Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_SDO_Quick_Send_Message")]
    public static extern ushort CS_ECAT_Slave_SDO_Quick_Send_Message(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Index, ushort SubIndex, ushort Datasize, ref byte Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_SDO_Quick_Read_Message")]
    public static extern ushort CS_ECAT_Slave_SDO_Quick_Read_Message(ushort CardNo, ushort NodeID, ushort SlotID, ushort Index, ushort SubIndex, ushort DataSize);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_SDO_Read_Response")]
    public static extern ushort CS_ECAT_Slave_SDO_Read_Response(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort Done, ref byte Data, ref uint ErrorCode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_SDO_Wait_All_Done")]
    public static extern ushort CS_ECAT_Slave_SDO_Wait_All_Done(ushort CardNo, ushort AxisNum, ref ushort NodeID, ref ushort SlotNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_SDO_Check_Done")]
    public static extern ushort CS_ECAT_Slave_SDO_Check_Done(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Done);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_SDO_Get_ErrorCode")]
    public static extern ushort CS_ECAT_Slave_SDO_Get_ErrorCode(ushort CardNo, ushort NodeID, ushort SlotNo, ref uint ErrorCode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_PDO_Get_OD_Data")]
    public static extern ushort CS_ECAT_Slave_PDO_Get_OD_Data(ushort CardNo, ushort NodeID, ushort SlotNo, ushort IOType, ushort ODIndex, ushort ODSubIndex, ushort ByteSize, ref byte Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_PDO_Set_OD_Data")]
    public static extern ushort CS_ECAT_Slave_PDO_Set_OD_Data(ushort CardNo, ushort NodeID, ushort SlotNo, ushort ODIndex, ushort ODSubIndex, ref byte Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_PDO_Get_Information")]
    public static extern ushort CS_ECAT_Slave_PDO_Get_Information(ushort CardNo, ushort NodeID, ushort SlotNo, ushort IOType, ref ushort ODCnt, ref ushort StartIndex);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_PDO_Get_Detail_Mapping")]
    public static extern ushort CS_ECAT_Slave_PDO_Get_Detail_Mapping(ushort CardNo, ushort NodeID, ushort SlotNo, ushort IOType, ushort ODSeqID, ref ushort ODIndex, ref ushort ODsubIndex, ref ushort ODBitSize, ref ushort ODStartIndex);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_PDO_Get_Rx_Data")]
    public static extern ushort CS_ECAT_Slave_PDO_Get_Rx_Data(ushort CardNo, ref byte Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_PDO_Get_Tx_Data")]
    public static extern ushort CS_ECAT_Slave_PDO_Get_Tx_Data(ushort CardNo, ref byte Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_PDO_Set_Tx_Detail_Data")]
    public static extern ushort CS_ECAT_Slave_PDO_Set_Tx_Detail_Data(ushort CardNo, ushort NodeID, ushort SlotNo, ushort ODStartIndex, ushort Bytesize, ref byte Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_PDO_Set_Tx_Data")]
    public static extern ushort CS_ECAT_Slave_PDO_Set_Tx_Data(ushort CardNo, ref byte Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Get_EMCY_Data")]
    public static extern ushort CS_ECAT_Slave_Get_EMCY_Data(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort ErrorCode, ref byte ErrorRegister, ref byte[] Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Get_Connect_Status")]
    public static extern ushort CS_ECAT_Slave_Get_Connect_Status(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort OpState, ref ushort AlarmStatus);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Request_State")]
    public static extern ushort CS_ECAT_Slave_Request_State(ushort CardNo, ushort NodeID, ushort SlotNo, ushort OpState);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Get_ESC_Error_Counter")]
    public static extern ushort CS_ECAT_Slave_Get_ESC_Error_Counter(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort Processing_Unit_Error_Counter, ref ushort PDI_Error_Counter);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Get_ESC_0x130")]
    public static extern ushort CS_ECAT_Slave_Get_ESC_0x130(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort ModuleInformation);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Get_ESC_0x134")]
    public static extern ushort CS_ECAT_Slave_Get_ESC_0x134(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort ModuleInformation);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Get_ESC_Port_Error_Counter")]
    public static extern ushort CS_ECAT_Slave_Get_ESC_Port_Error_Counter(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort Port_Error_Counter);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Online_Generate_Dat")]
    public static extern ushort CS_ECAT_Slave_Online_Generate_Dat(ushort CardNo, string Folder);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_MoveMode")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_MoveMode(ushort CardNo, ushort NodeID, ushort SlotNo, ref byte Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_ControlWord")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_ControlWord(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort ControllWord);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_StatusWord")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_StatusWord(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort StatusWord);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_Command")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_Command(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Command);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_Position")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Position);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_Actual_Command")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_Actual_Command(ushort CardNo, ushort NodeID, ushort SlotNo, ref int ActualCommand);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_Actual_Position")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_Actual_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int ActualPosition);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_Mdone")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_Mdone(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Mdone);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_Speed")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_Speed(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Speed);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_Current_Speed")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_Current_Speed(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Speed);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_Torque")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_Torque(ushort CardNo, ushort NodeID, ushort SlotNo, ref short Torque);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_Target_Command")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_Target_Command(ushort CardNo, ushort NodeID, ushort SlotNo, ref int TargetCommand);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_Buffer_Length")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_Buffer_Length(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort BufferLength);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Set_MoveMode")]
    public static extern ushort CS_ECAT_Slave_Motion_Set_MoveMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort OpMode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Set_Svon")]
    public static extern ushort CS_ECAT_Slave_Motion_Set_Svon(ushort CardNo, ushort NodeID, ushort SlotNo, ushort On_Off);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Ralm")]
    public static extern ushort CS_ECAT_Slave_Motion_Ralm(ushort CardNo, ushort NodeID, ushort SlotNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Set_Position")]
    public static extern ushort CS_ECAT_Slave_Motion_Set_Position(ushort CardNo, ushort NodeID, ushort SlotNo, int Pos);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Set_Command")]
    public static extern ushort CS_ECAT_Slave_Motion_Set_Command(ushort CardNo, ushort NodeID, ushort SlotNo, int Cmd);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Emg_Stop")]
    public static extern ushort CS_ECAT_Slave_Motion_Emg_Stop(ushort CardNo, ushort NodeID, ushort SlotNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Sd_Stop")]
    public static extern ushort CS_ECAT_Slave_Motion_Sd_Stop(ushort CardNo, ushort NodeID, ushort SlotNo, double Tdec);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Set_Alm_Reaction")]
    public static extern ushort CS_ECAT_Slave_Motion_Set_Alm_Reaction(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Fault_Type, ushort WR_Type);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Set_Internal_Limit_Active_Reaction")]
    public static extern ushort CS_ECAT_Slave_Motion_Set_Internal_Limit_Active_Reaction(ushort CardNo, ushort NodeID, ushort SlotID, ushort Internal_Limit_Active_Type);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Set_Command_Wait_Target_Reach")]
    public static extern ushort CS_ECAT_Slave_Motion_Set_Command_Wait_Target_Reach(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Wait);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Ignore_Sd_Stop_Wrong_Value")]
    public static extern ushort CS_ECAT_Slave_Motion_Ignore_Sd_Stop_Wrong_Value(ushort CardNo, byte Ignore);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Set_TouchProbe_Config")]
    public static extern ushort CS_ECAT_Slave_Motion_Set_TouchProbe_Config(ushort CardNo, ushort NodeID, ushort SlotNo, ushort TriggerMode, ushort Signal_Source);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Set_TouchProbe_QuickStart")]
    public static extern ushort CS_ECAT_Slave_Motion_Set_TouchProbe_QuickStart(ushort CardNo, ushort NodeID, ushort SlotNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Set_TouchProbe_QuickDone")]
    public static extern ushort CS_ECAT_Slave_Motion_Set_TouchProbe_QuickDone(ushort CardNo, ushort NodeID, ushort SlotNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Set_TouchProbe_Disable")]
    public static extern ushort CS_ECAT_Slave_Motion_Set_TouchProbe_Disable(ushort CardNo, ushort NodeID, ushort SlotNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_TouchProbe_Status")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_TouchProbe_Status(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_TouchProbe_Position")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_TouchProbe_Position(ushort CardNo, ushort NodeID, ushort SlotNo, ref int LatchPosition);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Record_TxRxPacket_Enable")]
    public static extern ushort CS_ECAT_Master_Record_TxRxPacket_Enable(ushort CardNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Record_TxRxPacket_Close")]
    public static extern ushort CS_ECAT_Master_Record_TxRxPacket_Close(ushort CardNo, string FileName);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Get_Record_Axis_Unusual")]
    public static extern ushort CS_ECAT_Slave_Motion_Get_Record_Axis_Unusual(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Motion_Clear_Record_Axis_Unusual")]
    public static extern ushort CS_ECAT_Slave_Motion_Clear_Record_Axis_Unusual(ushort CardNo);
    
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo, int Dist, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_V_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_V_Move(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Dir, int StrVel, int ConstVel, double Tacc, ushort SCurve);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Arc_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Arc_Move(ushort CardNo, ref ushort NodeID, ref ushort SlotNo, ref int CenterPoint, double Angle, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Arc2_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Arc2_Move(ushort CardNo, ref ushort NodeID, ref ushort SlotNo, ref int EndPoint, double Angle, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Arc3_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Arc3_Move(ushort CardNo, ref ushort NodeID, ref ushort SlotNo, ref int CenterPoint, ref int EndPoint, ushort Dir, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Arc4_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Arc4_Move(ushort CardNo, ref ushort NodeID, ref ushort SlotNo, ref int MidPoint, ref int EndPoint, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort Abs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Spiral_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Spiral_Move(ushort CardNo, ref ushort NodeID, ref ushort SlotID, ref int CenterPoint, int Spiral_Interval, double Angle, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Spiral2_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Spiral2_Move(ushort CardNo, ref ushort NodeID, ref ushort SlotID, ref int CenterPoint, ref int EndPoint, ushort Dir, ushort CycleNum, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Sphere_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Sphere_Move(ushort CardNo, ref ushort NodeID, ref ushort SlotNo, ref int Target1Point, ref int Target2Point, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Heli_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Heli_Move(ushort CardNo, ref ushort NodeID, ref ushort SlotNo, ref int CenterPoint, int Depth, int Pitch, ushort Dir, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Msbrline_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Msbrline_Move(ushort CardNo, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray, ushort ArcNodeBit, ref int Target1Point, ref int Target2Point, ushort Mode, double Parameter, double ArcAngle1, double ArcAngle2, double SpdRatio, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Mabrline_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Mabrline_Move(ushort CardNo, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray, ref int Target1Point, ref int Target2Point, int StrVel, int FirstVel, int SecondVel, int EndVel, double Tacc_Step1, double Tacc_Step2, ushort Absflag);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Multiaxes_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Multiaxes_Move(ushort CardNo, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray, ref int Dist, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort IsAbs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_2Segment_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_2Segment_Move(ushort CardNo, ushort NodeID, ushort SlotID, ushort SegMode, int Dist, int Dist2, int StrVel, int MaxVel, int MaxVel2, int EndVel, double Tacc, double Tsec, double Tdec, ushort SCurve, ushort IsAbs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Helix_Multiaxes_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Helix_Multiaxes_Move(ushort CardNo, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray, ref int CenterPoint, ref int Dist, int Depth, int Pitch, ushort Dir, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort Abs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Spiral_Helix_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Spiral_Helix_Move(ushort CardNo, ref ushort NodeID, ref ushort SlotID, ref int CenterPoint, int Spiral_Interval, double Angle, int Pitch, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort Abs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Spiral_Helix2_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Spiral_Helix2_Move(ushort CardNo, ref ushort NodeID, ref ushort SlotID, ref int CenterPoint, ref int EndPoint, ushort Dir, ushort CycleNum, int Pitch, int StrVel, int ConstVel, int EndVel, double Tacc, double Tdec, ushort SCurve, ushort Abs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Tank_Helix_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Tank_Helix_Move(ushort CardNo, ref ushort NodeID, ref ushort SlotID, ref int Point1, ref int Point2, ref int Point3, ref int Point4, int Depth, int Pitch, int MaxVel, ushort FilterTime, ushort AMFNum, ushort Abs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Reference_Pos_Move_Config")]
    public static extern ushort CS_ECAT_Slave_CSP_Reference_Pos_Move_Config(ushort CardNo, ushort MasterNodeID, ushort MasterSlotID, ushort SlaveNodeID, ushort SlaveSlotID, int ReferencePos, short Dir);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_SuperImposed_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_SuperImposed_Move(ushort CardNo, ushort NodeID, ushort SlotID, int Dist, int VelocityDiff, double Tacc, double Tdec);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_HaltSuperImposed_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_HaltSuperImposed_Move(ushort CardNo, ushort NodeID, ushort SlotID, double Tdec);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Get_Superimposed_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Get_Superimposed_Status(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort State);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_GearIn_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_GearIn_Move(ushort CardNo, ushort NodeID, ushort SlotID, ushort MasterNodeID, ushort MasterSlotID, ushort SourceType, ushort SourceNo, int RatioNumerator, uint RatioDenominator, double Acceleration, double Deceleration, ushort GearIn_Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_GearOut_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_GearOut_Move(ushort CardNo, ushort NodeID, ushort SlotID);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Get_GearIn_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Get_GearIn_Status(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort State);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_PVT_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_PVT_Move(ushort CardNo, ushort NodeID, ushort SlotID, int DataCnt, ref int TargetPos, ref int TargetTime, ref int TargetVel, ushort Abs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_PVTComplete_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Move(ushort CardNo, ushort NodeID, ushort SlotID, int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_PVT_Config")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_PVT_Config(ushort CardNo, ushort NodeID, ushort SlotID, int DataCnt, ref int TargetPos, ref int TargetTime, ref int TargetVel, ushort Abs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_PVTComplete_Config")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_PVTComplete_Config(ushort CardNo, ushort NodeID, ushort SlotID, int DataCnt, ref int TargetPos, ref int TargetTime, int StrVel, int EndVel, ushort Abs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_PVT_Sync_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_PVT_Sync_Move(ushort CardNo, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_PVT_Get_NowMotCount")]
    public static extern ushort CS_ECAT_Slave_CSP_PVT_Get_NowMotCount(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort NowMotCount);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM_Set_Engage_Parameters")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM_Set_Engage_Parameters(ushort CardNo, ushort NodeID, ushort SlotID, ushort SourceType, ushort EngageNodeID, ushort EngageSlotID, ushort SourceNo, int Engage_PreLed, int Cycle_PreLead, ushort Engage_Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM_Set_Compensate_Parameters")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM_Set_Compensate_Parameters(ushort CardNo, ushort NodeID, ushort SlotID, ushort SourceType, ushort CompensateNodeID, ushort CompensateSlotID, ushort SourceNo, int TargetPos, int Max_Compensate_Ratio, int Active_Range_Ratio, ushort CompensateTime, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM_Set_DisEngage_Parameters")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM_Set_DisEngage_Parameters(ushort CardNo, ushort NodeID, ushort SlotID, ushort SourceType, ushort DisEngageNodeID, ushort DisEngageSlotID, ushort SourceNo, int DisEngage_Position, ushort DI_DisEngage, ushort DisEngage_TurnOff, ushort DisEngage_Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM_Set_SingleMove")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM_Set_SingleMove(ushort CardNo, ushort NodeID, ushort SlotID, int Dist, int ConstVel, int EndVel, double Tacc, double Tdec, ushort Abs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM_Disable")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM_Disable(ushort CardNo, ushort NodeID, ushort SlotID, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM_Get_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM_Get_Status(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM_Table_Data")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM_Table_Data(ushort CardNo, ushort ECAMNo, int RegionNum, ref int DataArray, int Master_Pulse, double Slave_mm_Pulse, double Master_mm_Pulse);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM_Velocity_Data")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM_Velocity_Data(ushort CardNo, ushort ECAMNo, int RegionNum, ref int[] Percent, int SCurveRegion, int TotalLength, int Master_Pulse_P, double Slave_mm_Pulse, double Master_mm_Pulse, ushort Construct_Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM_Flying_Shears_Data")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM_Flying_Shears_Data(ushort CardNo, ushort ECAMNo, double GearNum_A, double GearNum_B, int KnifeNum, double SlaveDiameter, double EncoderDiameter, int EncoderPulseRev, int SlavePUURev, int CutLength, int RegionNum, ref int[] Region, int Slave_Prelead, int Master_Engage_Prelead, ushort PreLeadMode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM_Intermittence_Print_Data")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM_Intermittence_Print_Data(ushort CardNo, ushort ECAMNo, int RegionNum, int PrintLength, int BlankLength, double GearNum_A, double GearNum_B, int SlavePUURev, double SlaveDiameter, int MasterPulseRev, double MasterDiameter, int HoldingAreaDeg, int SCurveDeg, int SynIncreaseDeg);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM_Move(ushort CardNo, ushort NodeID, ushort SlotID, ushort ECAMNo, ushort MasterNodeID, ushort MasterSlotID, ushort SourceType, ushort SourceNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM_Move_ChangeTable")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM_Move_ChangeTable(ushort CardNo, ushort NodeID, ushort SlotID, ushort ECAMNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM_Set_Scale")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM_Set_Scale(ushort CardNo, ushort NodeID, ushort SlotID, double Percentage);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_Set_Engage_Parameters")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_Set_Engage_Parameters(ushort CardNo, ushort NodeID, ushort SlotID, ushort SourceType, ushort EngageNodeID, ushort EngageSlotID, ushort SourceNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_Set_Compensate_Parameters")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_Set_Compensate_Parameters(ushort CardNo, ushort NodeID, ushort SlotID, ushort SourceType, ushort CompensateNodeID, ushort CompensateSlotID, ushort SourceNo, double TargetPos, double ActiveRangeRatio, double MaxVel, double Acceleration, double Deceleration, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_Set_DisEngage_Parameters")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_Set_DisEngage_Parameters(ushort CardNo, ushort NodeID, ushort SlotID, ushort SourceType, ushort DisEngageNodeID, ushort DisEngageSlotID, ushort SourceNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_RotaryCut_Initial")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_RotaryCut_Initial(ushort CardNo, ushort RotCutNo, double RotaryAxisRadius, ushort RotaryAxisKnifeNum, double FeedAxisRadius, double CutLenth, double SyncStartPos, double SyncStopPos);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_RotaryCut_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_RotaryCut_Move(ushort CardNo, ushort NodeID, ushort SlotID, ushort RotCutNo, ushort MasterNodeID, ushort MasterSlotID, ushort SourceType, ushort SourceNo, int RotaryAxis_PulseRev, int FeedAxis_PulseRev, short RotaryAxis_Dir, short FeedAxis_Dir);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_FlyingShear_Initial")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_FlyingShear_Initial(ushort CardNo, ushort FlyingShearNo, double MasterRadius, double SlaveRadius, double CutLength, double MasterStartPosition, double MasterSyncPosition, double SlaveSyncPosition, double SlaveEndPosition, double SlaveWaitPosition, double SlaveVelocity, double SlaveAcceleration, double SlaveDeceleration);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_FlyingShear_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_FlyingShear_Move(ushort CardNo, ushort NodeID, ushort SlotID, ushort FlyingShearNo, ushort MasterNodeID, ushort MasterSlotID, ushort SourceType, ushort SourceNo, int MasterAxis_PulseRev, int SlaveAxis_PulseRev, short MasterAxis_Dir, short SlaveAxis_Dir);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_Disable")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_Disable(ushort CardNo, ushort NodeID, ushort SlotID);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_Get_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_Get_Status(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_Phasing")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_Phasing(ushort CardNo, ushort NodeID, ushort SlotID, double Dist, double MaxVel, double Acceleration, double Deceleration);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_CamSwitch")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_CamSwitch(ushort CardNo, ushort CamNo, ushort SwitchNum, ref ushort Percent, ushort Cam_Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_CamSwitch_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_CamSwitch_Status(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_Profile_Data")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_Profile_Data(ushort CardNo, ushort CamNo, ushort DataNum, ref double MasterPos_mm, ref double SlavePos_mm, ref ushort Curve_Type, ref double Velocity_Scale);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_Delete_Profile_Data")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_Delete_Profile_Data(ushort CardNo, ushort CamNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_Profile_CamIn")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_Profile_CamIn(ushort CardNo, ushort NodeID, ushort SlotID, ushort MasterNodeID, ushort MasterSlotID, ushort SourceType, ushort SourceNo, ushort CamNo, ushort Periodic, ushort MasterAbsolute, ushort SlaveAbsolute, double MasterOffset, double SlaveOffset, double MasterScaling, double SlaveScaling, ushort StartMode, int MasterAxis_mm_Pulse, int SlaveAxis_mm_Pulse, double Slave_MaxVel_mm, double Slave_Acc_mm, double Slave_Dec_mm, short MasterAxis_Dir, short SlaveAxis_Dir, ushort BufferMode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_Profile_CamOut")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_Profile_CamOut(ushort CardNo, ushort NodeID, ushort SlotID);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_Profile_Modify")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_Profile_Modify(ushort CardNo, ushort CamNo, ushort PointNo, ref double Parameters);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_Profile_CamSet")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_Profile_CamSet(ushort CardNo, ushort NodeID, ushort SlotID);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_ECAM2_Get_Slave_Info")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_ECAM2_Get_Slave_Info(ushort CardNo, ushort NodeID, ushort SlotID,ref short pi16_CurrentCamNo, ref int pi32_CurrentTablePos);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TableMotion_CycleMode_Data")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TableMotion_CycleMode_Data(ushort CardNo, ushort TableNo, ref int PositionData, int DataSize, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray, int MaxVel, ushort FilterTime, ushort AMFNum, ushort CycleNum, ref int IOControl);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TableMotion_CycleMode_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TableMotion_CycleMode_Move(ushort CardNo, ushort TableNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TableMotion_BufferMode_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TableMotion_BufferMode_Move(ushort CardNo, ushort TableNo, ref int PositionData, int DataSize, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray, int MaxVel, ushort FilterTime, ushort AMFNum, ref int IOControl);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TableMotion_Get_Length")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TableMotion_Get_Length(ushort CardNo, ushort TableNo, ref int cnt);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TableMotion_Get_CycleNum")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TableMotion_Get_CycleNum(ushort CardNo, ushort TableNo, ref int cycle);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TableMotion_Feedrate_Overwrite")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TableMotion_Feedrate_Overwrite(ushort CardNo, ushort TableNo, int ratio, double sec);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TableMotion_Feedhold")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TableMotion_Feedhold(ushort CardNo, ushort TableNo, ushort On_Off, double sec);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TableMotion_Get_VectorSpeed")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TableMotion_Get_VectorSpeed(ushort CardNo, ushort TableNo, ref double vectorspeed);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TableMotion_Set_Blending_Ratio")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TableMotion_Set_Blending_Ratio(ushort CardNo, ushort TableNo, uint Ratio);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TableMotion_ProfileMode_Parameter")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TableMotion_ProfileMode_Parameter(ushort CardNo, ushort TableNo, int NanoSmoothDecectionLength, int RefTurnOnSpd, ushort CurveDecectionFlag, int CurveDecectionLength, double CurveDecectionAngle, int ArcRefSpeed, int ArcRefRadius);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TableMotion_ProfileMode_Data")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TableMotion_ProfileMode_Data(ushort CardNo, ushort TableNo, ref int PositionData, int DataSize, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray, ref int MaxVel, ref int AccDec, double InitialVel_Ratio, ref int IOControl);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TableMotion_ProfileMode_Data_Flying_Photography")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TableMotion_ProfileMode_Data_Flying_Photography(ushort CardNo, ushort TableNo, ref int PositionData, int DataSize, ref ushort AxisArray, ref ushort SlotArray, ref int MaxVel, ref int AccDecLimit, int Remain_Dist, int Stage2_Velocity, double InitialVel_Ratio, ref int IOControl);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TableMotion_ProfileMode_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TableMotion_ProfileMode_Move(ushort CardNo, ushort TableNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TableMotion_IOControl_Mapping")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TableMotion_IOControl_Mapping(ushort CardNo, ushort TableNo, ushort IOControlNum, ushort NodeID, ushort SlotID, ushort BitNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Set_Crd_Parameters")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Set_Crd_Parameters(ushort CardNo, ushort TableNo, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray, int SycMaxVel, int SycMaxAcc, ushort S_Curve_Time, ushort Max_IO_Advance_Time, ushort SetOriginFlag, ref int OriginPos);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Get_Crd_Parameters")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Get_Crd_Parameters(ushort CardNo, ushort TableNo, ref ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray, ref int SycMaxVel, ref int SycMaxAcc, ref ushort S_Curve_Time, ref ushort Max_IO_Advance_Time, ref ushort SetOriginFlag, ref int OriginPos);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Clear_Crd")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Clear_Crd(ushort CardNo, ushort TableNo, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Line_MultiAxis")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Line_MultiAxis(ushort CardNo, ushort TableNo, ref int TargetPosition, int SynTargetVel, int SynEndVel, int SynAcc, ushort AbsMode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Line_MultiAxis_G00")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Line_MultiAxis_G00(ushort CardNo, ushort TableNo, ref int TargetPosition, int SynTargetVel, int SynAcc, ushort AbsMode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Arc_Radius")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Arc_Radius(ushort CardNo, ushort TableNo, ref ushort Arc_Axis, ref int TargetPosition, int Radius, ushort CircleDir, int SynTargetVel, int SynEndVel, int SynAcc, ushort AbsMode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Arc_CenterPos")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Arc_CenterPos(ushort CardNo, ushort TableNo, ref ushort Arc_Axis, ref int TargetPosition, ref int CenterPos, ushort CircleDir, int SynTargetVel, int SynEndVel, int SynAcc, ushort AbsMode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_IOControl")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_IOControl(ushort CardNo, ushort TableNo, ushort IO_Control_Num, ref MotionBuffer_IO_Set IO_Array, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Advanced_IOControl")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Advanced_IOControl(ushort CardNo, ushort TableNo, ushort IO_Control_Num, ref MotionBuffer_IO_Set IO_Array, ushort Time, ushort Mode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_SoftLimit_On")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_SoftLimit_On(ushort CardNo, ushort TableNo, ushort AxisNo, short LimitType, ushort Mode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_SoftLimit_Off")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_SoftLimit_Off(ushort CardNo, ushort TableNo, ushort AxisNo, short LimitType, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Delay")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Delay(ushort CardNo, ushort TableNo, double DelayTime, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Single_Step")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Single_Step(ushort CardNo, ushort TableNo, ushort FifoNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Get_Remain_Buffer_Size")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Get_Remain_Buffer_Size(ushort CardNo, ushort TableNo, ref int RemainBufSize, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Clear_Buffer")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Clear_Buffer(ushort CardNo, ushort TableNo, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Start")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Start(ushort CardNo, ushort TableNo, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Stop")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Stop(ushort CardNo, ushort TableNo, ushort FifoNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Feedrate_Override")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Feedrate_Override(ushort CardNo, ushort TableNo, double SynVelRatio);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Get_Motion_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Get_Motion_Status(ushort CardNo, ushort TableNo, ref ushort Status, ref int BlockNum, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Set_UserSegNum")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Set_UserSegNum(ushort CardNo, ushort TableNo, int SegNum, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Get_UserSegNum")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Get_UserSegNum(ushort CardNo, ushort TableNo, ref int SegNum, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Set_Stop_Deceleration")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Set_Stop_Deceleration(ushort CardNo, ushort TableNo, int DecSmoothStop, int DecAbruptStop);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Get_Stop_Deceleration")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Get_Stop_Deceleration(ushort CardNo, ushort TableNo, ref int DecSmoothStop, ref int DecAbruptStop);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Get_Position")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Get_Position(ushort CardNo, ushort TableNo, ref int Now_Position, ref int Now_Position_Org);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Get_Velocity")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Get_Velocity(ushort CardNo, ushort TableNo, ref int CurrentSpeed);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_BufGear")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_BufGear(ushort CardNo, ushort TableNo, ushort AxisNo, ushort SlotNo, int Distance, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_BufMove")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_BufMove(ushort CardNo, ushort TableNo, ushort AxisNo, ushort SlotNo, int Distance, int TargetVel, int Acceleration, ushort AbsMode, ushort BufMove_Mode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Set_LookAhead_Parameters")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Set_LookAhead_Parameters(ushort CardNo, ushort TableNo, int RefSwerveSpd, ushort ArcSpdModify, int ArcRefSpeed, int ArcRefRadius, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Disable")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Disable(ushort CardNo, ushort TableNo, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MoitonBuffer_Set_OD_Data")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MoitonBuffer_Set_OD_Data(ushort CardNo, ushort TableNo, ushort NodeID, ushort SlotID, ushort Index, ushort SubIndex, int Data, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Helix")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Helix(ushort CardNo, ushort TableNo, ref ushort Helix_Axis, ref int CenterPos, int Depth, int Pitch, ushort HelixDir, int SynTargetVel, int SynEndVel, int SynAcc, ushort AbsMode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Arc_MidPos")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Arc_MidPos(ushort CardNo, ushort TableNo, ref ushort Arc_Axis, ref int Arc_TargetPosition, ref int Arc_MidTargetPosition, int SynTargetVel, int SynEndVel, int SynAcc, ushort AbsMode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Arc1")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Arc1(ushort CardNo, ushort TableNo, ushort FollowAxisNum, ref ushort Axis_Array, ref int CenterPos, double Angle, ushort CircleDir, ref int Follow_TargetPosition, int SynTargetVel, int SynEndVel, int SynAcc, ushort SpeedMode, ushort AbsMode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Arc2")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Arc2(ushort CardNo, ushort TableNo, ushort FollowAxisNum, ref ushort Axis_Array, ref int Arc_TargetPosition, double Angle, ushort CircleDir, ref int Follow_TargetPosition, int SynTargetVel, int SynEndVel, int SynAcc, ushort SpeedMode, ushort AbsMode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Arc3")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Arc3(ushort CardNo, ushort TableNo, ushort FollowAxisNum, ref ushort Axis_Array, ref int Arc_TargetPosition, ref int CenterPos, ushort CircleDir, ref int Follow_TargetPosition, int SynTargetVel, int SynEndVel, int SynAcc, ushort SpeedMode, ushort AbsMode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Arc4")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Arc4(ushort CardNo, ushort TableNo, ushort FollowAxisNum, ref ushort Axis_Array, ref int Arc_TargetPosition, ref int Arc_MidPosition, ref int Follow_TargetPosition, int SynTargetVel, int SynEndVel, int SynAcc, ushort SpeedMode, ushort AbsMode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Arc_follow")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Arc_follow(ushort CardNo, ushort TableNo, ushort AxesNum, ref ushort Axis_Array, ushort ArcMode, ref ushort Arc_Axis_Index, ushort Master_Axis_Index, ref int CenterPoint, ref int EndPoint, double Angle, ushort Dir, int SynTargetVel, int SynEndVel, int SynAcc, ushort AbsMode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Line_MultiAxis2")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Line_MultiAxis2(ushort CardNo, ushort TableNo, ushort AxesNum, ref ushort Line_Axis, ref int TargetPosition, int SynTargetVel, int SynEndVel, int SynAcc, ushort AbsMode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Line_MultiAxis2_G00")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Line_MultiAxis2_G00(ushort CardNo, ushort TableNo, ushort AxesNum, ref ushort Line_Axis, ref int TargetPosition, int SynTargetVel, int SynAcc, ushort AbsMode, ushort FifoNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Set_G00_Check")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Set_G00_Check(ushort CardNo, ushort TableNo, ushort CheckTime, ref int ErrorRange, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Get_G00_Check")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Get_G00_Check(ushort CardNo, ushort TableNo, ref ushort CheckTime, ref int ErrorRange, ref ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Get_G00_Check_Flag")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Get_G00_Check_Flag(ushort CardNo, ushort TableNo, ref ushort CheckG00Flag);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_MotionBuffer_Get_G00_Check_Match")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_MotionBuffer_Get_G00_Check_Match(ushort CardNo, ushort TableNo, ref int Feedback);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Group_Info")]
    public static extern ushort CS_ECAT_Slave_CSP_Group_Info(ushort CardNo, ushort GroupNo, ushort Axes, ref ushort NodeID, ref ushort SlotID);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Group_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Group_Enable(ushort CardNo, ushort GroupNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Group_Motion_Stop")]
    public static extern ushort CS_ECAT_Slave_CSP_Group_Motion_Stop(ushort CardNo, ushort GroupNo, double Tdec, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Knife_Tool_Measure")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Knife_Tool_Measure(ushort CardNo, ushort MeasureNo, ushort NodeID, ushort SlotID, ushort TriggerEdge, int FirstVel, int SecondVel, double Tacc, double Tdec, int SafetyPosition, int LimitDistance, ushort Abs_Rel, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Get_Knife_Tool_Measure_Latch_Position")]
    public static extern ushort CS_ECAT_Slave_CSP_Get_Knife_Tool_Measure_Latch_Position(ushort CardNo, ushort MeasureNo, ref int LatchPosition);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Get_Knife_Tool_Measure_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Get_Knife_Tool_Measure_Status(ushort CardNo, ushort MeasureNo, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TouchProb_Home_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TouchProb_Home_Move(ushort CardNo, ushort NodeID, ushort SlotID, ushort TriggerSource, ushort TriggerEdge, ushort Direction, int Offset, int StrVel, int FirstVel, int SecondVel, int EndVel, double Tacc, double Tdec, ushort SCurve);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_TouchProb_Home_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_TouchProb_Home_Status(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Start_Gantry_TouchProb_Home_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Start_Gantry_TouchProb_Home_Move(ushort CardNo, ushort NodeID, ushort SlotID, ushort TriggerSource, ushort TriggerEdge, ushort Direction, int StrVel, int FirstVel, int SecondVel, int EndVel, double Tacc, double Tdec, ushort SCurve);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSV_Start_Move")]
    public static extern ushort CS_ECAT_Slave_CSV_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo, int Target_Velocity, double Acceleration, ushort Curve_Mode, ushort Acc_Type);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSV_Multi_Start_Move")]
    public static extern ushort CS_ECAT_Slave_CSV_Multi_Start_Move(ushort CardNo, ushort AxisNum, ref ushort NodeID, ref ushort SlotNo, ref int Target_Velocity, ref double Acceleration, ushort Curve_Mode, ushort Acc_Type);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CST_Start_Move")]
    public static extern ushort CS_ECAT_Slave_CST_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo, short Target_Torque, uint Slope, ushort Curve_Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CST_Multi_Start_Move")]
    public static extern ushort CS_ECAT_Slave_CST_Multi_Start_Move(ushort CardNo, ushort AxisNum, ref ushort NodeID, ref ushort SlotNo, ref short Target_Torque, ref uint Slope, ushort Curve_Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Set_Gear")]
    public static extern ushort CS_ECAT_Slave_CSP_Set_Gear(ushort CardNo, ushort NodeID, ushort SlotNo, double Numerator, double Denominator, short Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Set_Softlimit")]
    public static extern ushort CS_ECAT_Slave_CSP_Set_Softlimit(ushort CardNo, ushort NodeID, ushort SlotNo, int PosiLimit, int NegaLimit, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_TargetPos_Change")]
    public static extern ushort CS_ECAT_Slave_CSP_TargetPos_Change(ushort CardNo, ushort NodeID, ushort SlotNo, int NewPos);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Velocity_Change")]
    public static extern ushort CS_ECAT_Slave_CSP_Velocity_Change(ushort CardNo, ushort NodeID, ushort SlotNo, int NewSpeed, double Tsec);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_TargetPos_Velocity_Change")]
    public static extern ushort CS_ECAT_Slave_CSP_TargetPos_Velocity_Change(ushort CardNo, ushort NodeID, ushort SlotID, int NewPos, int NewSpeed, double Tsec);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Feedrate_Overwrite")]
    public static extern ushort CS_ECAT_Slave_CSP_Feedrate_Overwrite(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode, int NewSpeed, double Tsec);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Speed_Continue_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Speed_Continue_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Speed_Continue_Set_Mode")]
    public static extern ushort CS_ECAT_Slave_CSP_Speed_Continue_Set_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Speed_Continue_Set_Combine_Ratio")]
    public static extern ushort CS_ECAT_Slave_CSP_Speed_Continue_Set_Combine_Ratio(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Ratio);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Scurve_Rate")]
    public static extern ushort CS_ECAT_Slave_CSP_Scurve_Rate(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Scurve_Rate);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Linear_Speed_Master")]
    public static extern ushort CS_ECAT_Slave_CSP_Linear_Speed_Master(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Mask_Axis")]
    public static extern ushort CS_ECAT_Slave_CSP_Mask_Axis(ushort CardNo, ushort AxisNum, ref ushort AxisArray, ref ushort SlotNoArray);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Sync_Config")]
    public static extern ushort CS_ECAT_Slave_CSP_Sync_Config(ushort CardNo, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Sync_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Sync_Move(ushort CardNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Virtual_Set_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Virtual_Set_Enable(ushort CardNo, ushort NodeID, ushort SlotID, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Virtual_Set_Command")]
    public static extern ushort CS_ECAT_Slave_CSP_Virtual_Set_Command(ushort CardNo, ushort NodeID, ushort SlotID, int Command);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Abort_and_Change_Position")]
    public static extern ushort CS_ECAT_Slave_CSP_Abort_and_Change_Position(ushort CardNo, ushort Axes, ref ushort NodeID, ref ushort SlotID, ref int Dist, int MaxVel, int EndVel, double Tacc, double Tdec, ushort CurveMode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Flying_Compensate_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Flying_Compensate_Move(ushort CardNo, ushort NodeID, ushort SlotID, int Dist, int StrVel, int MaxVel, int EndVel, double Tacc, double Tdec, ushort AbsRel, ushort CompareChannel, int CompensateMaxVel, int CompensateOffset, double f64_Ratio);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Get_Flying_Compensate_Data")]
    public static extern ushort CS_ECAT_Slave_CSP_Get_Flying_Compensate_Data(ushort CardNo, ref int Velocity, ref int Offset, ushort DataNum);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Set_SingleAxis_NormalAMF_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Set_SingleAxis_NormalAMF_Enable(ushort CardNo, ushort NodeID, ushort SlotID, ushort FilterTime, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Set_NormalAMF_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Set_NormalAMF_Enable(ushort CardNo, ushort FilterNum, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Set_DITrigger_Info")]
    public static extern ushort CS_ECAT_Slave_CSP_Set_DITrigger_Info(ushort CardNo, ushort NodeID, ushort SlotID, ushort Polarity, ushort FilterTime);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Get_DITrigger_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Get_DITrigger_Status(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort TriggerStatus, ref uint TriggerCount);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Set_SLD_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Set_SLD_Enable(ushort CardNo, ushort NodeID, ushort SlotID, double Tdec, ushort SourceType, ushort DINodeID, ushort DISlotID, ushort SourceNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Set_TrSeg2_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Set_TrSeg2_Enable(ushort CardNo, ushort NodeID, ushort SlotID, int Dist, int ConstVel, int EndVel, double Tacc, double Tdec, ushort Scurve, ushort SourceType, ushort DINodeID, ushort DISlotID, ushort SourceNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Set_Velocity_Change_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Set_Velocity_Change_Enable(ushort CardNo, ushort NodeID, ushort SlotID, int NewSpeed, double Tsec, ushort SourceType, ushort DINodeID, ushort DISlotID, ushort SourceNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Set_DITrigger_OD")]
    public static extern ushort CS_ECAT_Slave_CSP_Set_DITrigger_OD(ushort CardNo, ushort NodeID, ushort SlotID, ushort OdIndex, ushort OdSubIndex);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Get_SoftLimit_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Get_SoftLimit_Status(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Set_SoftTargetReach")]
    public static extern ushort CS_ECAT_Slave_CSP_Set_SoftTargetReach(ushort CardNo, ushort NodeID, ushort SlotID, ushort Window_Time, uint Position_Window, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Get_SoftTargetReach_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Get_SoftTargetReach_Status(ushort CardNo, ushort NodeID, ushort SlotID, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Pitch_Set_InterVal")]
    public static extern ushort CS_ECAT_Slave_CSP_Pitch_Set_InterVal(ushort CardNo, ushort NodeID, ushort SlotID, uint InterVal);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Pitch_Set_Mode")]
    public static extern ushort CS_ECAT_Slave_CSP_Pitch_Set_Mode(ushort CardNo, ushort NodeID, ushort SlotID, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Pitch_Set_Org")]
    public static extern ushort CS_ECAT_Slave_CSP_Pitch_Set_Org(ushort CardNo, ushort NodeID, ushort SlotID, ushort Dir, int OrgPos);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Pitch_Set_Rel_Table")]
    public static extern ushort CS_ECAT_Slave_CSP_Pitch_Set_Rel_Table(ushort CardNo, ushort NodeID, ushort SlotID, ushort Dir, ref int Table, ushort Num);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Pitch_Set_Abs_Table")]
    public static extern ushort CS_ECAT_Slave_CSP_Pitch_Set_Abs_Table(ushort CardNo, ushort NodeID, ushort SlotID, ushort Dir, ref int Table, ushort Num);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Pitch_Set_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Pitch_Set_Enable(ushort CardNo, ushort NodeID, ushort SlotID, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Pitch_Reverse_Position")]
    public static extern ushort CS_ECAT_Slave_CSP_Pitch_Reverse_Position(ushort CardNo, ushort NodeID, ushort SlotID, int Pos_After_Compensate, ref int Pos_Before_Compensate);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Backlash_Set_Info")]
    public static extern ushort CS_ECAT_Slave_CSP_Backlash_Set_Info(ushort CardNo, ushort NodeID, ushort SlotID, short Backlash, ushort AccStep, ushort ConstStep, ushort DecStep);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Backlash_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Backlash_Enable(ushort CardNo, ushort NodeID, ushort SlotID, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Follow_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Follow_Enable(ushort CardNo, ushort GantryNo, ushort MasterNodeID, ushort MasterSlotID, ushort SlaveNodeID, ushort SlaveSlotID, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Gantry_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Gantry_Enable(ushort CardNo, ushort GantryNo, ushort MasterNodeID, ushort MasterSlotID, ushort SlaveNodeID, ushort SlaveSlotID, int Motor_Pulse_Per_Turn, double Motor_MaxTorque, double K_I, double K_VI, double Filter_Frequency, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Velocity_Gantry_Info")]
    public static extern ushort CS_ECAT_Slave_CSP_Velocity_Gantry_Info(ushort CardNo, ushort GantryNo, ushort MasterNodeID, ushort MasterSlotID, ushort SlaveNodeID, ushort SlaveSlotID, int Velcoity_Offset_Normalize, int Pulse_Normalize, int Podition_Threshold, double PID_Kp, double PID_Ki, double PID_Kd);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Velocity_Gantry_LPF_Info")]
    public static extern ushort CS_ECAT_Slave_CSP_Velocity_Gantry_LPF_Info(ushort CardNo, ushort GantryNo, ushort LPF_Mode, double LPF_Time);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Velocity_Gantry_Forward_Info")]
    public static extern ushort CS_ECAT_Slave_CSP_Velocity_Gantry_Forward_Info(ushort CardNo, ushort GantryNo, ushort Forward_Mode, ushort Forward_Time, double Forward_Kp);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Velocity_Gantry_PIDAdv_Info")]
    public static extern ushort CS_ECAT_Slave_CSP_Velocity_Gantry_PIDAdv_Info(ushort CardNo, ushort GantryNo, double PID_Output_Max, double PID_Output_Min, double PID_Ki_Max, double PID_Ki_Min);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Velocity_Gantry_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Velocity_Gantry_Enable(ushort CardNo, ushort GantryNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Gantry_Enable_PositionError")]
    public static extern ushort CS_ECAT_Slave_CSP_Gantry_Enable_PositionError(ushort CardNo, ushort GantryNo, ushort PositionErrorMode, int PositionError, ushort PassingCount, double Tdec, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Gantry_Get_PositionError")]
    public static extern ushort CS_ECAT_Slave_CSP_Gantry_Get_PositionError(ushort CardNo, ushort GantryNo, ref int PositionError);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Gantry_Get_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Gantry_Get_Status(ushort CardNo, ushort GantryNo, ref ushort Status, ref ushort PassingLevel);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Gantry_Set_Position")]
    public static extern ushort CS_ECAT_Slave_CSP_Gantry_Set_Position(ushort CardNo, ushort GantryNo, int NewPosition);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Gantry_Ralm")]
    public static extern ushort CS_ECAT_Slave_CSP_Gantry_Ralm(ushort CardNo, ushort GantryNo, int Velocity, double Tacc, double Tdec);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Gantry_Ralm_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Gantry_Ralm_Status(ushort CardNo, ushort GantryNo, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Gantry_Set_Home_Config")]
    public static extern ushort CS_ECAT_Slave_CSP_Gantry_Set_Home_Config(ushort CardNo, ushort GantryNo, ushort Mode, int Offset, uint FirstVel, uint SecondVel, double Tacc, ushort Dir, int Shift);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Gantry_Home_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Gantry_Home_Move(ushort CardNo, ushort GantryNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Gantry_Disable_Home_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Gantry_Disable_Home_Move(ushort CardNo, ushort GantryNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Gantry_Home_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Gantry_Home_Status(ushort CardNo, ushort GantryNo, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Gantry_Set_Home_Edge_Trigger_Level")]
    public static extern ushort CS_ECAT_Slave_CSP_Gantry_Set_Home_Edge_Trigger_Level(ushort CardNo, ushort GantryNo, ushort PositiveLimt, ushort NegativeLimit, ushort HomeSensor);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Velocity_Compensate_Info")]
    public static extern ushort CS_ECAT_Slave_CSP_Velocity_Compensate_Info(ushort CardNo, ushort NodeID, ushort SlotID, int Velcoity_Offset_Normalize, int Pulse_Normalize, int Podition_Threshold, double PID_Kp, double PID_Ki, double PID_Kd);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Velocity_Compensate_LPF_Info")]
    public static extern ushort CS_ECAT_Slave_CSP_Velocity_Compensate_LPF_Info(ushort CardNo, ushort NodeID, ushort SlotID, ushort LPF_Mode, double LPF_Time);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Velocity_Compensate_Forward_Info")]
    public static extern ushort CS_ECAT_Slave_CSP_Velocity_Compensate_Forward_Info(ushort CardNo, ushort NodeID, ushort SlotID, ushort Forward_Mode, ushort Forward_Time, double Forward_Kp);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Velocity_Compensate_PIDAdv_Info")]
    public static extern ushort CS_ECAT_Slave_CSP_Velocity_Compensate_PIDAdv_Info(ushort CardNo, ushort NodeID, ushort SlotID, double PID_Output_Max, double PID_Output_Min, double PID_Ki_Max, double PID_Ki_Min);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Velocity_Compensate_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Velocity_Compensate_Enable(ushort CardNo, ushort NodeID, ushort SlotID, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Remote_Limit_Config")]
    public static extern ushort CS_ECAT_Slave_CSP_Remote_Limit_Config(ushort CardNo, ushort LimitNo, ushort AxisNodeID, ushort AxisSlotID, ushort DINodeID, ushort DISlotID, ushort PELBitNo, ushort NELBitNo, ushort ORGBitNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Remote_Limit_Edge_Trigger_Level")]
    public static extern ushort CS_ECAT_Slave_CSP_Remote_Limit_Edge_Trigger_Level(ushort CardNo, ushort LimitNo, ushort PositiveLimt, ushort NegativeLimit, ushort HomeSensor);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Remote_Limit_Enable")]
    public static extern ushort CS_ECAT_Slave_CSP_Remote_Limit_Enable(ushort CardNo, ushort LimitNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Home_Axis_Config")]
    public static extern ushort CS_ECAT_Slave_CSP_Home_Axis_Config(ushort CardNo, ushort HomeNo, ushort NodeID, ushort SlotID, ushort Mode, int Offset, uint FirstVel, uint SecondVel, double Tacc, ushort Dir, int Shift);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Home_Move")]
    public static extern ushort CS_ECAT_Slave_CSP_Home_Move(ushort CardNo, ushort HomeNo, ushort u16_LimitNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_CSP_Home_Status")]
    public static extern ushort CS_ECAT_Slave_CSP_Home_Status(ushort CardNo, ushort HomeNo, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Home_Config")]
    public static extern ushort CS_ECAT_Slave_Home_Config(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode, int Offset, uint FirstVel, uint SecondVel, uint Acceleration);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Home_Move")]
    public static extern ushort CS_ECAT_Slave_Home_Move(ushort CardNo, ushort NodeID, ushort SlotNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Home_Status")]
    public static extern ushort CS_ECAT_Slave_Home_Status(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_User_Motion_Control_Set_Type")]
    public static extern ushort CS_ECAT_Slave_User_Motion_Control_Set_Type(ushort CardNo, ushort GroupNo, ushort AxisNum, ref ushort AxisNo, ref ushort SlotNo, ushort MotionType);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_User_Motion_Control_Set_Enable_Mode")]
    public static extern ushort CS_ECAT_Slave_User_Motion_Control_Set_Enable_Mode(ushort CardNo, ushort GroupNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_User_Motion_Control_Get_Enable_Mode")]
    public static extern ushort CS_ECAT_Slave_User_Motion_Control_Get_Enable_Mode(ushort CardNo, ushort GroupNo, ref ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_User_Motion_Control_Set_Data")]
    public static extern ushort CS_ECAT_Slave_User_Motion_Control_Set_Data(ushort CardNo, ushort GroupNo, ref int CmdPulse);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_User_Motion_Control_Get_DataCnt")]
    public static extern ushort CS_ECAT_Slave_User_Motion_Control_Get_DataCnt(ushort CardNo, ushort GroupNo, ref ushort Counter);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_User_Motion_Control_Clear_Data")]
    public static extern ushort CS_ECAT_Slave_User_Motion_Control_Clear_Data(ushort CardNo, ushort GroupNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_User_Motion_Control_Ralm")]
    public static extern ushort CS_ECAT_Slave_User_Motion_Control_Ralm(ushort CardNo, ushort GroupNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_User_Motion_Control_Svon")]
    public static extern ushort CS_ECAT_Slave_User_Motion_Control_Svon(ushort CardNo, ushort GroupNo, ushort ON_OFF);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_User_Motion_Control_Get_Alm")]
    public static extern ushort CS_ECAT_Slave_User_Motion_Control_Get_Alm(ushort CardNo, ushort GroupNo, ref ushort Alm);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Record_Set_Type")]
    public static extern ushort CS_ECAT_Slave_Record_Set_Type(ushort CardNo, ushort NodeID, ushort SlotNo, ushort MonitorIndex, ushort IOType, ushort OD_Index, ushort OD_SubIndex);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Record_Set_Enable")]
    public static extern ushort CS_ECAT_Slave_Record_Set_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Record_Get_Data_Cnt")]
    public static extern ushort CS_ECAT_Slave_Record_Get_Data_Cnt(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Cnt);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Record_Read_Data")]
    public static extern ushort CS_ECAT_Slave_Record_Read_Data(ushort CardNo, ushort NodeID, ushort SlotNo, ref uint Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Record_Clear_Data")]
    public static extern ushort CS_ECAT_Slave_Record_Clear_Data(ushort CardNo, ushort NodeID, ushort SlotNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Record_Multi_Set_Enable")]
    public static extern ushort CS_ECAT_Slave_Record_Multi_Set_Enable(ushort CardNo, ushort NodeNum, ref ushort NodeIDArray, ref ushort SlotNoArray, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_Record_Multi_Clear_Data")]
    public static extern ushort CS_ECAT_Slave_Record_Multi_Clear_Data(ushort CardNo, ushort NodeNum, ref ushort NodeIDArray, ref ushort SlotNoArray);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_MR_J4_ABS_Set_Mode")]
    public static extern ushort CS_ECAT_Slave_MR_J4_ABS_Set_Mode(ushort CardNo, ushort GroupNo, ushort AxisNum, ushort DI_NodeID, ushort DI_SlotID, ushort DO_NodeID, ushort DO_SlotID);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_MR_J4_ABS_Set_Enable")]
    public static extern ushort CS_ECAT_Slave_MR_J4_ABS_Set_Enable(ushort CardNo, ushort GroupNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_MR_J4_ABS_Get_State")]
    public static extern ushort CS_ECAT_Slave_MR_J4_ABS_Get_State(ushort CardNo, ushort GroupNo, ref ushort State);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_MR_J4_ABS_Get_Data")]
    public static extern ushort CS_ECAT_Slave_MR_J4_ABS_Get_Data(ushort CardNo, ushort GroupNo, ushort AxisIndex, ref int Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_MR_J4_ABS_Reset")]
    public static extern ushort CS_ECAT_Slave_MR_J4_ABS_Reset(ushort CardNo, ushort GroupNo, ushort AxisIndex);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_PP_Start_Move")]
    public static extern ushort CS_ECAT_Slave_PP_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo, int TargetPos, uint ConstVel, uint Acceleration, uint Deceleration, ushort Abs_Rel);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_PP_Advance_Config")]
    public static extern ushort CS_ECAT_Slave_PP_Advance_Config(ushort CardNo, ushort NodeID, ushort SlotNo, ushort SetBit, uint End_Vel, int Min_Range_Limit, int Max_Range_Limit, int Min_Soft_Limit, int Max_Soft_Limit);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_PV_Start_Move")]
    public static extern ushort CS_ECAT_Slave_PV_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo, int TargetVel, uint Acceleration, uint Deceleration);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_PV_Advance_Config")]
    public static extern ushort CS_ECAT_Slave_PV_Advance_Config(ushort CardNo, ushort NodeID, ushort SlotNo, ushort SetBit, ushort Max_Torque, ushort Velocity_Window, ushort Velocity_Window_Time, ushort Velocity_Threshold, ushort Velocity_Threshold_Time);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_VL_Start_Move")]
    public static extern ushort CS_ECAT_Slave_VL_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo, int TargetVel, uint Acceleration, uint Deceleration);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_PT_Start_Move")]
    public static extern ushort CS_ECAT_Slave_PT_Start_Move(ushort CardNo, ushort NodeID, ushort SlotNo, short Target_Torque, uint Slope);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_PT_Advance_Config")]
    public static extern ushort CS_ECAT_Slave_PT_Advance_Config(ushort CardNo, ushort NodeID, ushort SlotNo, ushort SetBit, ushort Max_Current, short Torque_Profile);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DIO_Get_Input_Value")]
    public static extern ushort CS_ECAT_Slave_DIO_Get_Input_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DIO_Get_Output_Value")]
    public static extern ushort CS_ECAT_Slave_DIO_Get_Output_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DIO_Set_Output_Value")]
    public static extern ushort CS_ECAT_Slave_DIO_Set_Output_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ushort value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DIO_Get_Single_Input_Value")]
    public static extern ushort CS_ECAT_Slave_DIO_Get_Single_Input_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ushort BitNum, ref ushort value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DIO_Get_Single_Output_Value")]
    public static extern ushort CS_ECAT_Slave_DIO_Get_Single_Output_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ushort BitNum, ref ushort value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DIO_Set_Single_Output_Value")]
    public static extern ushort CS_ECAT_Slave_DIO_Set_Single_Output_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ushort BitNum, ushort value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DIO_Set_Output_Error_Mode")]
    public static extern ushort CS_ECAT_Slave_DIO_Set_Output_Error_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort BitMode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DIO_Set_Output_Error_Value")]
    public static extern ushort CS_ECAT_Slave_DIO_Set_Output_Error_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ushort BitMode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC0902_Get_Input_Value")]
    public static extern ushort CS_ECAT_Slave_ESC0902_Get_Input_Value(ushort u16_CardNo, ushort u16_NodeID, ushort u16_PortNo, ref ushort pu16_Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC0902_Get_Single_Input_Value")]
    public static extern ushort CS_ECAT_Slave_ESC0902_Get_Single_Input_Value(ushort u16_CardNo, ushort u16_NodeID, ushort u16_PortNo, ushort u16_BitNo, ref ushort pu16_Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC0902_Set_Output_Enable")]
    public static extern ushort CS_ECAT_Slave_ESC0902_Set_Output_Enable(ushort u16_CardNo, ushort u16_NodeID, ushort u16_PortNo, ushort u16_Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC0902_Get_Output_Value")]
    public static extern ushort CS_ECAT_Slave_ESC0902_Get_Output_Value(ushort u16_CardNo, ushort u16_NodeID, ushort u16_PortNo, ref ushort pu16_Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC0902_Set_Output_Value")]
    public static extern ushort CS_ECAT_Slave_ESC0902_Set_Output_Value(ushort u16_CardNo, ushort u16_NodeID, ushort u16_PortNo, ushort u16_Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC0902_Get_Single_Output_Value")]
    public static extern ushort CS_ECAT_Slave_ESC0902_Get_Single_Output_Value(ushort u16_CardNo, ushort u16_NodeID, ushort u16_PortNo, ushort u16_BitNo, ref ushort pu16_Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC0902_Set_Single_Output_Value")]
    public static extern ushort CS_ECAT_Slave_ESC0902_Set_Single_Output_Value(ushort u16_CardNo, ushort u16_NodeID , ushort u16_PortNo, ushort u16_BitNo, ushort u16_Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC0902_Set_Output_Error_Mode")]
    public static extern ushort CS_ECAT_Slave_ESC0902_Set_Output_Error_Mode(ushort u16_CardNo, ushort u16_NodeID, ushort u16_PortNo, ushort u16_BitMode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC0902_Set_Output_Error_Value")]
    public static extern ushort CS_ECAT_Slave_ESC0902_Set_Output_Error_Value(ushort u16_CardNo, ushort u16_NodeID, ushort u16_PortNo, ushort u16_Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_AIO_Get_Input_Value")]
    public static extern ushort CS_ECAT_Slave_AIO_Get_Input_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_AIO_Set_Output_Value")]
    public static extern ushort CS_ECAT_Slave_AIO_Set_Output_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_AIO_Get_Output_Value")]
    public static extern ushort CS_ECAT_Slave_AIO_Get_Output_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_AIO_Set_Output_Error_Mode")]
    public static extern ushort CS_ECAT_Slave_AIO_Set_Output_Error_Mode(ushort CardNo, ushort NodeID, ushort SlotID, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_AIO_Set_Output_Error_Value")]
    public static extern ushort CS_ECAT_Slave_AIO_Set_Output_Error_Value(ushort CardNo, ushort NodeID, ushort SlotID, ushort Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Set_Counter_Mode")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Set_Counter_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Get_Counter_Value")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Get_Counter_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ref uint Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Reset_Counter_Value")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Reset_Counter_Value(ushort CardNo, ushort NodeID, ushort SlotNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Get_Overflow_Times")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Get_Overflow_Times(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Times);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Get_Underflow_Times")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Get_Underflow_Times(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Times);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Reset_Overflow_Times")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Reset_Overflow_Times(ushort CardNo, ushort NodeID, ushort SlotNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Set_Output_Mode")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Set_Output_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Set_Output_Enable")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Set_Output_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Set_Output_Pulse_Width")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Set_Output_Pulse_Width(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Width);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Set_Force_Value")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Set_Force_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Set_Method0_To_2_Compare_Value")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Set_Method0_To_2_Compare_Value(ushort CardNo, ushort NodeID, ushort SlotNo, uint Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Set_Method3_Compare_Value")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Set_Method3_Compare_Value(ushort CardNo, ushort NodeID, ushort SlotNo, uint FirstValue, uint SecondValue, uint ReloadValue);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Set_PWM_Frequency")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Set_PWM_Frequency(ushort CardNo, ushort NodeID, ushort SlotNo, uint Frequency);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Set_PWM_Duty_Value")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Set_PWM_Duty_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Duty);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5604_Set_Output_Error_Handle_Enable")]
    public static extern ushort CS_ECAT_Slave_R1_EC5604_Set_Output_Error_Handle_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5614_Set_MJ_Config")]
    public static extern ushort CS_ECAT_Slave_R1_EC5614_Set_MJ_Config(ushort CardNo, ushort MJNo, ushort MJType, ushort NodeID, ushort SlotNo, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray, ref int MaxSpeed, ref double TAcc, ref double Ratio);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5614_Set_MJ_Enable")]
    public static extern ushort CS_ECAT_Slave_R1_EC5614_Set_MJ_Enable(ushort CardNo, ushort MJNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5614_Get_IO_Status")]
    public static extern ushort CS_ECAT_Slave_R1_EC5614_Get_IO_Status(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5614_Get_MPG_Counter")]
    public static extern ushort CS_ECAT_Slave_R1_EC5614_Get_MPG_Counter(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Counter);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5614_Set_Mpg_Mode")]
    public static extern ushort CS_ECAT_Slave_R1_EC5614_Set_Mpg_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5614_Set_MPG_AMF")]
    public static extern ushort CS_ECAT_Slave_R1_EC5614_Set_MPG_AMF(ushort CardNo, ushort AMFNo, ushort AMFNum, ushort FilterNum, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5614_Clear_MPG_Counter")]
    public static extern ushort CS_ECAT_Slave_R1_EC5614_Clear_MPG_Counter(ushort u16_CardNo, ushort u16_NodeID, ushort u16_SlotID);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5621_Set_Output_Mode")]
    public static extern ushort CS_ECAT_Slave_R1_EC5621_Set_Output_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5621_Set_Input_Mode")]
    public static extern ushort CS_ECAT_Slave_R1_EC5621_Set_Input_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5621_Set_ORG_Inverse")]
    public static extern ushort CS_ECAT_Slave_R1_EC5621_Set_ORG_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5621_Set_QZ_Inverse")]
    public static extern ushort CS_ECAT_Slave_R1_EC5621_Set_QZ_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5621_Set_Home_SpMode")]
    public static extern ushort CS_ECAT_Slave_R1_EC5621_Set_Home_SpMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5621_Set_MEL_Inverse")]
    public static extern ushort CS_ECAT_Slave_R1_EC5621_Set_MEL_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5621_Set_PEL_Inverse")]
    public static extern ushort CS_ECAT_Slave_R1_EC5621_Set_PEL_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5621_Set_Svon_Inverse")]
    public static extern ushort CS_ECAT_Slave_R1_EC5621_Set_Svon_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5621_Set_Home_Slow_Down")]
    public static extern ushort CS_ECAT_Slave_R1_EC5621_Set_Home_Slow_Down(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable, ushort SlowDoneTime, ushort WaitTime);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5621_Get_IO_Status")]
    public static extern ushort CS_ECAT_Slave_R1_EC5621_Get_IO_Status(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort IOStatus);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5621_Get_Single_IO_Status")]
    public static extern ushort CS_ECAT_Slave_R1_EC5621_Get_Single_IO_Status(ushort CardNo, ushort NodeID, ushort SlotNo, ushort BitNo, ref ushort IOStatus);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC5621_Set_ALM_Clean_Time")]
    public static extern ushort CS_ECAT_Slave_R1_EC5621_Set_ALM_Clean_Time(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Ms);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Set_Latch_Enable")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Set_Latch_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Set_Latch_Source")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Set_Latch_Source(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Source);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Set_Latch_Trigger_Edge")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Set_Latch_Trigger_Edge(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Edge);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Set_Latch_External_IO")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Set_Latch_External_IO(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Select);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Get_Latch_Latest_Value")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Get_Latch_Latest_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Get_Latch_FIFO_Counter")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Get_Latch_FIFO_Counter(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Count);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Get_Latch_FIFO_Value")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Get_Latch_FIFO_Value(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Get_Latch_FIFO_Source")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Get_Latch_FIFO_Source(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Source);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Reset_Latch_FIFO")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Reset_Latch_FIFO(ushort CardNo, ushort NodeID, ushort SlotNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Set_Compare_Output_Mode")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Set_Compare_Output_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Set_Compare_GPIO_Output_Enable")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Set_Compare_GPIO_Output_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Set_Compare_IO_Inverse")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Set_Compare_IO_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Inverse);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Set_Compare_Interval_Setting")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Set_Compare_Interval_Setting(ushort CardNo, ushort NodeID, ushort SlotNo, int StartPos, ushort Dir, uint Interval, uint TriggetCount);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Reset_Compare_Trigger_Counter")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Reset_Compare_Trigger_Counter(ushort CardNo, ushort NodeID, ushort SlotNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Set_Compare_One_Shot")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Set_Compare_One_Shot(ushort CardNo, ushort NodeID, ushort SlotNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Set_Trigger_Time")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Set_Trigger_Time(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Set_Compare_Source")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Set_Compare_Source(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Source);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Set_Compare_Enable")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Set_Compare_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Set_Table_Compare_Level_Setting")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Set_Table_Compare_Level_Setting(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Level);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Set_Table_Compare_Data")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Set_Table_Compare_Data(ushort CardNo, ushort NodeID, ushort SlotNo, ushort DataNum, ref int TableData);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Get_Table_Compare_FIFO_Counter")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Get_Table_Compare_FIFO_Counter(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Count);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC0624_Reset_Table_Compare_Counter")]
    public static extern ushort CS_ECAT_Slave_R1_EC0624_Reset_Table_Compare_Counter(ushort CardNo, ushort NodeID, ushort SlotNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_ECx62x_Set_Output_Mode")]
    public static extern ushort CS_ECAT_Slave_R1_ECx62x_Set_Output_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_ECx62x_Set_Input_Mode")]
    public static extern ushort CS_ECAT_Slave_R1_ECx62x_Set_Input_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_ECx62x_Set_ORG_Inverse")]
    public static extern ushort CS_ECAT_Slave_R1_ECx62x_Set_ORG_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_ECx62x_Set_QZ_Inverse")]
    public static extern ushort CS_ECAT_Slave_R1_ECx62x_Set_QZ_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_ECx62x_Set_Home_SpMode")]
    public static extern ushort CS_ECAT_Slave_R1_ECx62x_Set_Home_SpMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_ECx62x_Set_MEL_Inverse")]
    public static extern ushort CS_ECAT_Slave_R1_ECx62x_Set_MEL_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_ECx62x_Set_PEL_Inverse")]
    public static extern ushort CS_ECAT_Slave_R1_ECx62x_Set_PEL_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_ECx62x_Set_Svon_Inverse")]
    public static extern ushort CS_ECAT_Slave_R1_ECx62x_Set_Svon_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_ECx62x_Set_Home_Slow_Down")]
    public static extern ushort CS_ECAT_Slave_R1_ECx62x_Set_Home_Slow_Down(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable, ushort SlowDoneTime, ushort WaitTime);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_ECx62x_Get_IO_Status")]
    public static extern ushort CS_ECAT_Slave_R1_ECx62x_Get_IO_Status(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort IOStatus);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_ECx62x_Get_Single_IO_Status")]
    public static extern ushort CS_ECAT_Slave_R1_ECx62x_Get_Single_IO_Status(ushort CardNo, ushort NodeID, ushort SlotNo, ushort BitNo, ref ushort IOStatus);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_ECx62x_Set_ALM_Inverse")]
    public static extern ushort CS_ECAT_Slave_R1_ECx62x_Set_ALM_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC70E2_Set_Output_Enable")]
    public static extern ushort CS_ECAT_Slave_R1_EC70E2_Set_Output_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Eanble);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC70X2_Set_Output_Enable")]
    public static extern ushort CS_ECAT_Slave_R1_EC70X2_Set_Output_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Eanble);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC8124_Set_Input_RangeMode")]
    public static extern ushort CS_ECAT_Slave_R1_EC8124_Set_Input_RangeMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC8124_Set_Input_ConvstFreq_Mode")]
    public static extern ushort CS_ECAT_Slave_R1_EC8124_Set_Input_ConvstFreq_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC8124_Set_Input_Enable")]
    public static extern ushort CS_ECAT_Slave_R1_EC8124_Set_Input_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC8124_Get_Input_RangeMode")]
    public static extern ushort CS_ECAT_Slave_R1_EC8124_Get_Input_RangeMode(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC8124_Set_Input_AverageMode")]
    public static extern ushort CS_ECAT_Slave_R1_EC8124_Set_Input_AverageMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Avg_Times);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC8124_Get_Input_Value")]
    public static extern ushort CS_ECAT_Slave_R1_EC8124_Get_Input_Value(ushort CardNo, ushort NodeID, ushort SlotID, ushort AIType, ref double Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC9144_Set_Output_RangeMode")]
    public static extern ushort CS_ECAT_Slave_R1_EC9144_Set_Output_RangeMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort RangeMode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC9144_Set_Output_Enable")]
    public static extern ushort CS_ECAT_Slave_R1_EC9144_Set_Output_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC9144_Get_Output_ReturnCode")]
    public static extern ushort CS_ECAT_Slave_R1_EC9144_Get_Output_ReturnCode(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort ReturnCode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC9144_Set_Output_Value")]
    public static extern ushort CS_ECAT_Slave_R1_EC9144_Set_Output_Value(ushort CardNo, ushort NodeID, ushort SlotID, double Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_R1_EC9144_Get_Output_Value")]
    public static extern ushort CS_ECAT_Slave_R1_EC9144_Get_Output_Value(ushort CardNo, ushort NodeID, ushort SlotID, ref double Value);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DeltaServo_Write_Parameter")]
    public static extern ushort CS_ECAT_Slave_DeltaServo_Write_Parameter(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Page, ushort Index, int WriteData);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DeltaServo_Read_Parameter")]
    public static extern ushort CS_ECAT_Slave_DeltaServo_Read_Parameter(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Page, ushort Index, ref int ReadData);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DeltaServo_Read_Parameter_Info")]
    public static extern ushort CS_ECAT_Slave_DeltaServo_Read_Parameter_Info(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Page, ushort Index, ref ushort ParaType, ref ushort DataSize, ref ushort DataType);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DeltaServo_Set_Velocity_Limit")]
    public static extern ushort CS_ECAT_Slave_DeltaServo_Set_Velocity_Limit(ushort CardNo, ushort NodeID, ushort SlotNo, uint LimitValue);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DeltaServo_Set_Compare_Enable")]
    public static extern ushort CS_ECAT_Slave_DeltaServo_Set_Compare_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable, ushort CompareSource, ushort SignalLength, ushort SignalPolarity);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DeltaServo_Get_Compare_Enable")]
    public static extern ushort CS_ECAT_Slave_DeltaServo_Get_Compare_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Enable, ref ushort CompareSource, ref ushort SignalLength, ref ushort SignalPolarity);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_DeltaServo_Set_Compare_Config")]
    public static extern ushort CS_ECAT_Slave_DeltaServo_Set_Compare_Config(ushort CardNo, ushort NodeID, ushort SlotNo, ushort CompareNum, ref int ComparePos);
    
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Security_Check_Verifykey")]
    public static extern ushort CS_ECAT_Security_Check_Verifykey(ushort CardNo, ref uint VerifyKey);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Security_Get_Check_Verifykey_State")]
    public static extern ushort CS_ECAT_Security_Get_Check_Verifykey_State(ushort CardNo, ref ushort State);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Security_Write_Verifykey")]
    public static extern ushort CS_ECAT_Security_Write_Verifykey(ushort CardNo, ref uint VerifyKey);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Security_Get_Write_Verifykey_State")]
    public static extern ushort CS_ECAT_Security_Get_Write_Verifykey_State(ushort CardNo, ref ushort State);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Security_Check_UserPassword")]
    public static extern ushort CS_ECAT_Security_Check_UserPassword(ushort CardNo, ref uint UserPassword);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Security_Get_Check_UserPassword_State")]
    public static extern ushort CS_ECAT_Security_Get_Check_UserPassword_State(ushort CardNo, ref ushort State);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Security_Write_UserPassword")]
    public static extern ushort CS_ECAT_Security_Write_UserPassword(ushort CardNo, ref uint UserPassword);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Security_Get_Write_UserPassword_State")]
    public static extern ushort CS_ECAT_Security_Get_Write_UserPassword_State(ushort CardNo, ref ushort State);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_MRAM_Write_Word_Data")]
    public static extern ushort CS_ECAT_Master_MRAM_Write_Word_Data(ushort CardNo, uint Index, uint DataNum, ref ushort Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_MRAM_Read_Word_Data")]
    public static extern ushort CS_ECAT_Master_MRAM_Read_Word_Data(ushort CardNo, uint Index, uint DataNum, ref ushort Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_MRAM_Write_DWord_Data")]
    public static extern ushort CS_ECAT_Master_MRAM_Write_DWord_Data(ushort CardNo, uint Index, uint DataNum, ref uint Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_MRAM_Read_DWord_Data")]
    public static extern ushort CS_ECAT_Master_MRAM_Read_DWord_Data(ushort CardNo, uint Index, uint DataNum, ref uint Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Set_Output")]
    public static extern ushort CS_ECAT_GPIO_Set_Output(ushort CardNo, ushort OnOff);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Get_Output")]
    public static extern ushort CS_ECAT_GPIO_Get_Output(ushort CardNo, ref ushort OnOff);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Get_Input")]
    public static extern ushort CS_ECAT_GPIO_Get_Input(ushort CardNo, ref ushort OnOff);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Set_Digital_Filter")]
    public static extern ushort CS_ECAT_GPIO_Set_Digital_Filter(ushort CardNo, ushort Filter);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Get_Digital_Filter")]
    public static extern ushort CS_ECAT_GPIO_Get_Digital_Filter(ushort CardNo, ref ushort Filter);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Set_Single_Output")]
    public static extern ushort CS_ECAT_GPIO_Set_Single_Output(ushort CardNo, ushort BitNo, ushort OnOff);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Get_Single_Output")]
    public static extern ushort CS_ECAT_GPIO_Get_Single_Output(ushort CardNo, ushort BitNo, ref ushort OnOff);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Get_Single_Input")]
    public static extern ushort CS_ECAT_GPIO_Get_Single_Input(ushort CardNo, ushort BitNo, ref ushort OnOff);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Set_Output_Control_Mode")]
    public static extern ushort CS_ECAT_GPIO_Set_Output_Control_Mode(ushort CardNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Emg_Enable")]
    public static extern ushort CS_ECAT_GPIO_Emg_Enable(ushort CardNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Emg_Set_Input_Trigger_Level")]
    public static extern ushort CS_ECAT_GPIO_Emg_Set_Input_Trigger_Level(ushort CardNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Emg_Soft_Clear")]
    public static extern ushort CS_ECAT_GPIO_Emg_Soft_Clear(ushort CardNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Emg_Get_Status")]
    public static extern ushort CS_ECAT_GPIO_Emg_Get_Status(ushort CardNo, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Set_Output32")]
    public static extern ushort CS_ECAT_GPIO_Set_Output32(ushort CardNo, uint OnOff);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Get_Output32")]
    public static extern ushort CS_ECAT_GPIO_Get_Output32(ushort CardNo, ref uint OnOff);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Get_Input32")]
    public static extern ushort CS_ECAT_GPIO_Get_Input32(ushort CardNo, ref uint OnOff);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Set_Digital_Filter32")]
    public static extern ushort CS_ECAT_GPIO_Set_Digital_Filter32(ushort CardNo, ushort Filter);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Get_Digital_Filter32")]
    public static extern ushort CS_ECAT_GPIO_Get_Digital_Filter32(ushort CardNo, ref ushort Filter);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Set_High_Speed_Mode")]
    public static extern ushort CS_ECAT_GPIO_Set_High_Speed_Mode(ushort CardNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Get_High_Speed_Mode")]
    public static extern ushort CS_ECAT_GPIO_Get_High_Speed_Mode(ushort CardNo, ref ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Set_Output_Mode_Select")]
    public static extern ushort CS_ECAT_GPIO_Set_Output_Mode_Select(ushort CardNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Get_Output_Mode_Select")]
    public static extern ushort CS_ECAT_GPIO_Get_Output_Mode_Select(ushort CardNo, ref ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Set_Single_Output_Mode_Select")]
    public static extern ushort CS_ECAT_GPIO_Set_Single_Output_Mode_Select(ushort CardNo, ushort GPIONo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_GPIO_Get_Single_Output_Mode_Select")]
    public static extern ushort CS_ECAT_GPIO_Get_Single_Output_Mode_Select(ushort CardNo, ushort GPIONo, ref ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel_Position")]
    public static extern ushort CS_ECAT_Compare_Set_Channel_Position(ushort CardNo, ushort CompareChannel, int Position);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Get_Channel_Position")]
    public static extern ushort CS_ECAT_Compare_Get_Channel_Position(ushort CardNo, ushort CompareChannel, ref int Position);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Ipulser_Mode")]
    public static extern ushort CS_ECAT_Compare_Set_Ipulser_Mode(ushort CardNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel_Direction")]
    public static extern ushort CS_ECAT_Compare_Set_Channel_Direction(ushort CardNo, ushort CompareChannel, ushort Dir);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel_Trigger_Time")]
    public static extern ushort CS_ECAT_Compare_Set_Channel_Trigger_Time(ushort CardNo, ushort CompareChannel, uint TimeUs);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel_Trigger_Time_Multiple")]
    public static extern ushort CS_ECAT_Compare_Set_Channel_Trigger_Time_Multiple(ushort CardNo, ushort CompareChannel, ushort Multiple);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel_One_Shot")]
    public static extern ushort CS_ECAT_Compare_Set_Channel_One_Shot(ushort CardNo, ushort CompareChannel);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel_Source")]
    public static extern ushort CS_ECAT_Compare_Set_Channel_Source(ushort CardNo, ushort CompareChannel, ushort Source);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel_Enable")]
    public static extern ushort CS_ECAT_Compare_Set_Channel_Enable(ushort CardNo, ushort CompareChannel, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Channel0_Position")]
    public static extern ushort CS_ECAT_Compare_Channel0_Position(ushort CardNo, int Start, ushort Dir, ushort Interval, uint TriggerCount);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel0_Trigger_By_GPIO")]
    public static extern ushort CS_ECAT_Compare_Set_Channel0_Trigger_By_GPIO(ushort CardNo, ushort Dir, ushort Interval, int TriggerCount);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel1_Output_Enable")]
    public static extern ushort CS_ECAT_Compare_Set_Channel1_Output_Enable(ushort CardNo, ushort OnOff);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel1_Output_Mode")]
    public static extern ushort CS_ECAT_Compare_Set_Channel1_Output_Mode(ushort CardNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Get_Channel1_IO_Status")]
    public static extern ushort CS_ECAT_Compare_Get_Channel1_IO_Status(ushort CardNo, ref ushort IOStatus);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel1_GPIO_Out")]
    public static extern ushort CS_ECAT_Compare_Set_Channel1_GPIO_Out(ushort CardNo, ushort OnOff);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel1_Position_Table")]
    public static extern ushort CS_ECAT_Compare_Set_Channel1_Position_Table(ushort CardNo, ref int PosTable, uint TableSize);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel1_Position_Table_Level")]
    public static extern ushort CS_ECAT_Compare_Set_Channel1_Position_Table_Level(ushort CardNo, ref int PosTable, ref uint LevelTable, uint TableSize);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Get_Channel1_Position_Table_Count")]
    public static extern ushort CS_ECAT_Compare_Get_Channel1_Position_Table_Count(ushort CardNo, ref uint pCount);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel_Polarity")]
    public static extern ushort CS_ECAT_Compare_Set_Channel_Polarity(ushort CardNo, ushort Inverse);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Reuse_Channel1_Position_Table")]
    public static extern ushort CS_ECAT_Compare_Reuse_Channel1_Position_Table(ushort CardNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Reuse_Channel1_Position_Table_Level")]
    public static extern ushort CS_ECAT_Compare_Reuse_Channel1_Position_Table_Level(ushort CardNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Channel0_Position_32Bit")]
    public static extern ushort CS_ECAT_Compare_Channel0_Position_32Bit(ushort CardNo, int Start, ushort Dir, uint Interval, uint TriggerCount);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel0_Trigger_By_GPIO_32Bit")]
    public static extern ushort CS_ECAT_Compare_Set_Channel0_Trigger_By_GPIO_32Bit(ushort CardNo, ushort Dir, uint Interval, int TriggerCount);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel_Position_Table")]
    public static extern ushort CS_ECAT_Compare_Set_Channel_Position_Table(ushort CardNo, ushort CompareChannel, ref int PosTable, uint TableSize);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Channel_Position_Table_Level")]
    public static extern ushort CS_ECAT_Compare_Set_Channel_Position_Table_Level(ushort CardNo, ushort CompareChannel, ref int PosTable, ref uint LevelTable, uint TableSize);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Get_Channel_Position_Table_Count")]
    public static extern ushort CS_ECAT_Compare_Get_Channel_Position_Table_Count(ushort CardNo, ushort CompareChannel, ref uint Count);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Reuse_Channel_Position_Table")]
    public static extern ushort CS_ECAT_Compare_Reuse_Channel_Position_Table(ushort CardNo, ushort CompareChannel);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Reuse_Channel_Position_Table_Level")]
    public static extern ushort CS_ECAT_Compare_Reuse_Channel_Position_Table_Level(ushort CardNo, ushort CompareChannel);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Set_Control_Mode")]
    public static extern ushort CS_ECAT_Compare_Set_Control_Mode(ushort CardNo, ushort CompareChannel, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_2D_Set_Range")]
    public static extern ushort CS_ECAT_Compare_2D_Set_Range(ushort CardNo, ushort CompareChannel, int Range);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_2D_Set_Table")]
    public static extern ushort CS_ECAT_Compare_2D_Set_Table(ushort CardNo, ushort Group, ref int PosX, ref int PosY, uint TableSize);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_2D_Set_Table_Level")]
    public static extern ushort CS_ECAT_Compare_2D_Set_Table_Level(ushort CardNo, ushort Group, ref int PosX, ref int PosY, ref uint LevelTable, uint TableSize);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_2D_Set_Channel_Range")]
    public static extern ushort CS_ECAT_Compare_2D_Set_Channel_Range(ushort CardNo, ushort CompareChannel, int XRange, int YRange);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_2D_Set_Channel_Table")]
    public static extern ushort CS_ECAT_Compare_2D_Set_Channel_Table(ushort CardNo, ushort CompareChannel, ushort Group, ref int PosX, ref int PosY, uint TableSize);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_2D_Set_Channel_Table_Level")]
    public static extern ushort CS_ECAT_Compare_2D_Set_Channel_Table_Level(ushort CardNo, ushort CompareChannel, ushort Group, ref int PosX, ref int PosY, ref uint LevelTable, uint TableSize);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Get_Channel_Count")]
    public static extern ushort CS_ECAT_Compare_Get_Channel_Count(ushort CardNo, ushort CompareChannel, ref uint Count);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Compare_Reset_Channel_Count")]
    public static extern ushort CS_ECAT_Compare_Reset_Channel_Count(ushort CardNo, ushort CompareChannel);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_PWM_Set_Frequency")]
    public static extern ushort CS_ECAT_PWM_Set_Frequency(ushort CardNo, uint Freq);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_PWM_Set_Duty")]
    public static extern ushort CS_ECAT_PWM_Set_Duty(ushort CardNo, uint Duty);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_PWM_Set_Control_Mode")]
    public static extern ushort CS_ECAT_PWM_Set_Control_Mode(ushort CardNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_PWM_Soft_Enable")]
    public static extern ushort CS_ECAT_PWM_Soft_Enable(ushort CardNo, ushort OnOff);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_PWM_VCTL_Config")]
    public static extern ushort CS_ECAT_PWM_VCTL_Config(ushort CardNo, ushort TableNo, ushort NodeID, ushort SlotID, ushort Mode, uint InitData);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_PWM_VCTL_Set_DDA_Table")]
    public static extern ushort CS_ECAT_PWM_VCTL_Set_DDA_Table(ushort CardNo, ushort TableNo, ushort Type, ushort Interpolation, ref int pi32_PWMTable, double f64_MaxVelocity);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_PWM_VCTL_Follow_DDA")]
    public static extern ushort CS_ECAT_PWM_VCTL_Follow_DDA(ushort CardNo, ushort TableNo, ushort DdaGroupNo, ref double pf64_Gear, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Card_CE_Operate_Enable")]
    public static extern ushort CS_ECAT_Card_CE_Operate_Enable(ushort CardNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Card_CE_Write_PC_Data")]
    public static extern ushort CS_ECAT_Card_CE_Write_PC_Data(ushort CardNo, ushort Index, uint Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Card_CE_Read_PC_Data")]
    public static extern ushort CS_ECAT_Card_CE_Read_PC_Data(ushort CardNo, ushort Index, ref uint Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Card_CE_Write_Device_Data")]
    public static extern ushort CS_ECAT_Card_CE_Write_Device_Data(ushort CardNo, ushort Index, uint Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Card_CE_Read_Device_Data")]
    public static extern ushort CS_ECAT_Card_CE_Read_Device_Data(ushort CardNo, ushort Index, ref uint Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Card_CE_Write_PC_Struct")]
    public static extern ushort CS_ECAT_Card_CE_Write_PC_Struct(ushort CardNo, ref uint AddrOfStruct, uint Size);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Card_CE_Read_PC_Struct")]
    public static extern ushort CS_ECAT_Card_CE_Read_PC_Struct(ushort CardNo, ref uint AddrOfStruct, uint Size);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Card_CE_Write_Device_Struct")]
    public static extern ushort CS_ECAT_Card_CE_Write_Device_Struct(ushort CardNo, ref uint AddrOfStruct, uint Size);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Card_CE_Read_Device_Struct")]
    public static extern ushort CS_ECAT_Card_CE_Read_Device_Struct(ushort CardNo, ref uint AddrOfStruct, uint Size);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Card_Get_OS_Type")]
    public static extern ushort CS_ECAT_Card_Get_OS_Type(ushort CardNo, ref byte[] pu8_Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Card_Get_Mac_Address")]
    public static extern ushort CS_ECAT_Card_Get_Mac_Address(ushort CardNo, ushort Index, ref byte[] MacAddr);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_RTX_Open_Rtss")]
    public static extern ushort CS_ECAT_Master_RTX_Open_Rtss(ushort CardNo, string FilePath, ref ushort Flag);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_RTX_Create_Shared_Memory")]
    public static extern ushort CS_ECAT_Master_RTX_Create_Shared_Memory(ushort CardNo, uint MaxSize, string SpaceName, ref IntPtr location, ref IntPtr Sharedmemory_Handle);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_RTX_Open_Shared_Memory")]
    public static extern ushort CS_ECAT_Master_RTX_Open_Shared_Memory(ushort CardNo, string SpaceName, ref IntPtr location, ref IntPtr Sharedmemory_Handle);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_RTX_Create_Event")]
    public static extern ushort CS_ECAT_Master_RTX_Create_Event(ushort CardNo, ushort ManualReset, ushort InitialState, string EventName, ref IntPtr Event_Handle);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_RTX_Open_Event")]
    public static extern ushort CS_ECAT_Master_RTX_Open_Event(ushort CardNo, string EventName, ref IntPtr Event_Handle);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_RTX_Set_Event")]
    public static extern ushort CS_ECAT_Master_RTX_Set_Event(ushort CardNo, IntPtr Event_Handle);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_RTX_Close_Handle")]
    public static extern ushort CS_ECAT_Master_RTX_Close_Handle(ushort CardNo, IntPtr A_Handle);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Errorlog_API_Set_Enable")]
    public static extern ushort CS_ECAT_Master_Errorlog_API_Set_Enable(ushort CardNo, ushort Enable, ushort Mode, ushort Level);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Errorlog_API_Get_Enable")]
    public static extern ushort CS_ECAT_Master_Errorlog_API_Get_Enable(ushort CardNo, ref ushort Enable, ref ushort Mode, ref ushort Level);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Errorlog_API_Get_FunIDToString")]
    public static extern ushort CS_ECAT_Master_Errorlog_API_Get_FunIDToString(ushort CardNo, ushort FunID, string FunName);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Errorlog_API_Clear")]
    public static extern ushort CS_ECAT_Master_Errorlog_API_Clear(ushort CardNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Errorlog_API_Get_ErrorCnt")]
    public static extern ushort CS_ECAT_Master_Errorlog_API_Get_ErrorCnt(ushort CardNo, ref ushort ErrorCnt);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Errorlog_API_Get_ErrorData")]
    public static extern ushort CS_ECAT_Master_Errorlog_API_Get_ErrorData(ushort CardNo, ref ushort NodeID, ref ushort SlotID, ref ushort FunID, ref ushort ErrorCode, ref double ErrorTime);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Errorlog_API_Get_Single_ErrorData")]
    public static extern ushort CS_ECAT_Master_Errorlog_API_Get_Single_ErrorData(ushort CardNo, ushort Count, ref ushort NodeID, ref ushort SlotID, ref ushort FunID, ref ushort ErrorCode, ref double ErrorTime);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Errorlog_API_DumpData")]
    public static extern ushort CS_ECAT_Master_Errorlog_API_DumpData(ushort CardNo, string FilePath);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Errorlog_Status_Set_Enable")]
    public static extern ushort CS_ECAT_Master_Errorlog_Status_Set_Enable(ushort CardNo, ushort Enable, ushort RecordMask);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Errorlog_Status_Get_Enable")]
    public static extern ushort CS_ECAT_Master_Errorlog_Status_Get_Enable(ushort CardNo, ref ushort Enable, ref ushort RecordMask);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Errorlog_Status_Clear")]
    public static extern ushort CS_ECAT_Master_Errorlog_Status_Clear(ushort CardNo);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Errorlog_Status_Get_ErrorCnt")]
    public static extern ushort CS_ECAT_Master_Errorlog_Status_Get_ErrorCnt(ushort CardNo, ref ushort ErrorCnt);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Errorlog_Status_Get_ErrorData")]
    public static extern ushort CS_ECAT_Master_Errorlog_Status_Get_ErrorData(ushort CardNo, ref ushort ErrorCode, ref ushort ErrorData, ref double ErrorTime);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Errorlog_Status_Get_Single_ErrorData")]
    public static extern ushort CS_ECAT_Master_Errorlog_Status_Get_Single_ErrorData(ushort CardNo, ushort Count, ref ushort ErrorCode, ref ushort ErrorData, ref double ErrorTime);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_DLL_Path")]
    public static extern ushort CS_ECAT_Master_Get_DLL_Path(string lpFilePath, uint nSize, ref uint nLength);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_DLL_Version")]
    public static extern ushort CS_ECAT_Master_Get_DLL_Version(string lpBuf, uint nSize, ref uint nLength);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_DLL_ProductVersion")]
    public static extern ushort CS_ECAT_Master_Get_DLL_ProductVersion(string lpBuf, uint nSize, ref uint nLength);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_DLL_Path_Single")]
    public static extern ushort CS_ECAT_Master_Get_DLL_Path_Single(ushort CardNo, string lpFilePath, uint nSize, ref uint nLength);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_DLL_Version_Single")]
    public static extern ushort CS_ECAT_Master_Get_DLL_Version_Single(ushort CardNo, string lpBuf, uint nSize, ref uint nLength);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_Get_DLL_ProductVersion_Single")]
    public static extern ushort CS_ECAT_Master_Get_DLL_ProductVersion_Single(ushort CardNo, string lpBuf, uint nSize, ref uint nLength);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC5614_Set_MJ_Config")]
    public static extern ushort CS_ECAT_Slave_ESC5614_Set_MJ_Config(ushort CardNo, ushort MJNo, ushort MJType, ushort NodeID, ushort SlotNo, ushort AxisNum, ref ushort AxisArray, ref ushort SlotArray, ref int MaxSpeed, ref double TAcc, ref double Ratio);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC5614_Set_MJ_Enable")]
    public static extern ushort CS_ECAT_Slave_ESC5614_Set_MJ_Enable(ushort CardNo, ushort MJNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC5614_Get_IO_Status")]
    public static extern ushort CS_ECAT_Slave_ESC5614_Get_IO_Status(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Status);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC5614_Get_MPG_Counter")]
    public static extern ushort CS_ECAT_Slave_ESC5614_Get_MPG_Counter(ushort CardNo, ushort NodeID, ushort SlotNo, ref int Counter);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC5621_Set_Output_Mode")]
    public static extern ushort CS_ECAT_Slave_ESC5621_Set_Output_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC5621_Set_Input_Mode")]
    public static extern ushort CS_ECAT_Slave_ESC5621_Set_Input_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC5621_Set_ORG_Inverse")]
    public static extern ushort CS_ECAT_Slave_ESC5621_Set_ORG_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC5621_Set_QZ_Inverse")]
    public static extern ushort CS_ECAT_Slave_ESC5621_Set_QZ_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC5621_Set_Home_SpMode")]
    public static extern ushort CS_ECAT_Slave_ESC5621_Set_Home_SpMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC5621_Set_MEL_Inverse")]
    public static extern ushort CS_ECAT_Slave_ESC5621_Set_MEL_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC5621_Set_PEL_Inverse")]
    public static extern ushort CS_ECAT_Slave_ESC5621_Set_PEL_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC5621_Set_Svon_Inverse")]
    public static extern ushort CS_ECAT_Slave_ESC5621_Set_Svon_Inverse(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC70E2_Set_Output_Enable")]
    public static extern ushort CS_ECAT_Slave_ESC70E2_Set_Output_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Eanble);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC8124_Set_Input_RangeMode")]
    public static extern ushort CS_ECAT_Slave_ESC8124_Set_Input_RangeMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC8124_Set_Input_ConvstFreq_Mode")]
    public static extern ushort CS_ECAT_Slave_ESC8124_Set_Input_ConvstFreq_Mode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC8124_Set_Input_Enable")]
    public static extern ushort CS_ECAT_Slave_ESC8124_Set_Input_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC8124_Get_Input_RangeMode")]
    public static extern ushort CS_ECAT_Slave_ESC8124_Get_Input_RangeMode(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort Mode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC8124_Set_Input_AverageMode")]
    public static extern ushort CS_ECAT_Slave_ESC8124_Set_Input_AverageMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Avg_Times);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC9144_Set_Output_RangeMode")]
    public static extern ushort CS_ECAT_Slave_ESC9144_Set_Output_RangeMode(ushort CardNo, ushort NodeID, ushort SlotNo, ushort RangeMode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC9144_Set_Output_Enable")]
    public static extern ushort CS_ECAT_Slave_ESC9144_Set_Output_Enable(ushort CardNo, ushort NodeID, ushort SlotNo, ushort Enable);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Slave_ESC9144_Get_Output_ReturnCode")]
    public static extern ushort CS_ECAT_Slave_ESC9144_Get_Output_ReturnCode(ushort CardNo, ushort NodeID, ushort SlotNo, ref ushort ReturnCode);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_GPIO_Set_Type")]
    public static extern ushort CS_ECAT_Master_GPIO_Set_Type(ushort CardNo, ushort PortNo, ushort Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_GPIO_Set_OutputValue")]
    public static extern ushort CS_ECAT_Master_GPIO_Set_OutputValue(ushort CardNo, ushort Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_GPIO_Get_OutputValue")]
    public static extern ushort CS_ECAT_Master_GPIO_Get_OutputValue(ushort CardNo, ref ushort Data);
    [DllImport(strDLL_Path, EntryPoint = "_ECAT_Master_GPIO_Get_InputValue")]
    public static extern ushort CS_ECAT_Master_GPIO_Get_InputValue(ushort CardNo, ref ushort Data);
  }
}
