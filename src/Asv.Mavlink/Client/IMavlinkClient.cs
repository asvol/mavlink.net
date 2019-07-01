using System;

namespace Asv.Mavlink
{
    /// <summary>
    /// 
    /// https://mavlink.io/en/services/
    /// </summary>
    public interface IMavlinkClient:IDisposable
    {
        MavlinkClientIdentity Identity { get; }
        IMavlinkTelemetry Rtt { get; }
        IMavlinkParameterMicroservice Params { get; }
        IMavlinkCommandMicroservice Commands { get; }
        IMavlinkMissionMicroservice Mission { get; }
        IMavlinkOffboardMode Offboard { get; }
        IMavlinkCommon Common { get; }
    }
}
