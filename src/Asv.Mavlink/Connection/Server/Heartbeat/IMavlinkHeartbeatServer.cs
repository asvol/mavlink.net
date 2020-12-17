using System;
using Asv.Mavlink.V2.Common;
using Asv.Mavlink.V2.Minimal;

namespace Asv.Mavlink.Server
{
    public interface IMavlinkHeartbeatServer:IDisposable
    {
        void Start();
        void Set(Action<HeartbeatPayload> changeCallback);
    }
}
