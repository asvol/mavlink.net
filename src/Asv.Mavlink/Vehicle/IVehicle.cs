using System;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public interface IVehicle:IDisposable
    {
        IRxValue<LinkState> Link { get; }
        IRxValue<int> PacketRateHz { get; }
        IRxValue<HeartbeatPayload> Heartbeat { get; }
        IRxValue<SysStatusPayload> SysStatus { get; }
    }
}
