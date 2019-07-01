using System;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink.Server
{
    public interface IMavlinkHeartbeatServer:IDisposable
    {
        void Set(Action<HeartbeatPayload> changeCallback);
    }
}