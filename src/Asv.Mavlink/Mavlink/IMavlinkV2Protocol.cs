using System;

namespace Asv.Mavlink
{
    /// <summary>
    /// 
    /// https://mavlink.io/en/services/
    /// </summary>
    public interface IMavlinkV2Protocol:IDisposable
    {
        int SystemId { get; }
        int ComponentId { get; }
        int TargetComponentId { get; }
        int TargetSystemId { get; }
        IMavlinkTelemetry Rtt { get; }
        IMavlinkParameterMicroservice Params { get; }
        IMavlinkCommandMicroservice Commands { get; }
        IMavlinkMissionMicroservice Mission { get; }
        IMavlinkOffboardMode Offboard { get; }
        IMavlinkCommon Common { get; }
    }
}
