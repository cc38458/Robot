namespace Robot.Core.Enums
{
    /// <summary>
    /// 單軸狀態
    /// </summary>
    public enum MotorState
    {
        /// <summary>Servo OFF，尚未啟動</summary>
        NULL = 0,

        /// <summary>Servo ON，靜止待命</summary>
        STOP = 1,

        /// <summary>正在執行運動命令</summary>
        MOVING = 2,

        /// <summary>該軸處於警報狀態</summary>
        ALARM = 3,
    }
}
