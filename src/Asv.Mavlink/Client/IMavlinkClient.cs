using System;
using Asv.Mavlink.Client;

namespace Asv.Mavlink
{
    /// <summary>
    /// 
    /// https://mavlink.io/en/services/
    /// </summary>
    public interface IMavlinkClient:IDisposable
    {
        MavlinkClientIdentity Identity { get; }
        IHeartbeatClient Heartbeat { get; }
        IMavlinkTelemetry Rtt { get; }
        IMavlinkParameterMicroservice Params { get; }
        IMavlinkCommandMicroservice Commands { get; }
        IMavlinkMissionMicroservice Mission { get; }
        IMavlinkOffboardMode Offboard { get; }
        IMavlinkCommon Common { get; }
        IDebugClient Debug { get; }
    }
}
