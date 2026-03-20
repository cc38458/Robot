namespace Robot.Core.Enums
{
    /// <summary>
    /// 運動指令類型（用於指令隊列）
    /// </summary>
    public enum CommandType
    {
        /// <summary>絕對位置移動（單軸 CSP）</summary>
        MoveAbsolute,

        /// <summary>相對位置移動（單軸 CSP）</summary>
        MoveRelative,

        /// <summary>等速持續移動（單軸 V_Move）</summary>
        MovePV,

        /// <summary>多點路徑移動（單軸 PVTComplete）</summary>
        MovePT,

        /// <summary>軟減速停止（單軸）</summary>
        Stop,

        /// <summary>多軸同步 PVT（末端移動用，barrier 同步發送）</summary>
        MultiAxisPVT,

        /// <summary>變更移動速度（單軸 CSP Velocity Change）</summary>
        VelocityChange,
    }
}
