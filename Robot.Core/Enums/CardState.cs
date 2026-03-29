using System.ComponentModel;
using System.Reflection;

namespace Robot.Core.Enums
{
    /// <summary>
    /// 軸卡整體狀態
    /// </summary>
    public enum CardState
    {
        /// <summary>尚未呼叫 Start()，軸卡未連線</summary>
        NULL = 0,

        /// <summary>Start() 已呼叫，正在建立 EtherCAT 連線</summary>
        CONNING = 1,

        /// <summary>EtherCAT 連線成功，但尚未初始化軸（Servo 仍 OFF）</summary>
        CONNCET = 2,

        /// <summary>連線過程發生錯誤（含逾時）</summary>
        CONNERR = 3,

        /// <summary>所有軸初始化完成且 Servo ON，可接收運動命令</summary>
        READY = 4,

        /// <summary>一或多軸發生警報，或看門狗觸發</summary>
        ALARM = 5,
    }
}
