using System;
using Asv.Mavlink.V2.Common;
using Asv.Mavlink.V2.Minimal;

namespace Asv.Mavlink.Client
{
    public interface IHeartbeatClient:IDisposable
    {
        IRxValue<HeartbeatPayload> RawHeartbeat { get; }
        IRxValue<int> PacketRateHz { get; }
        IRxValue<double> LinkQuality { get; }
        IRxValue<LinkState> Link { get; }
    }
}
