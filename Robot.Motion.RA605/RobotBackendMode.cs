namespace Robot.Motion.RA605
{
    /// <summary>
    /// 手臂後端模式
    /// </summary>
    public enum RobotBackendMode
    {
        /// <summary>使用實體 EtherCAT 硬體。</summary>
        Real,
        /// <summary>使用記憶體模擬後端（無需硬體）。</summary>
        Mock,
    }
}
