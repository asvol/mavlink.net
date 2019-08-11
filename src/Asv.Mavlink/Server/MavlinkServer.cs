using System;

namespace Asv.Mavlink.Server
{
    public interface IMavlinkServer:IDisposable
    {
        IMavlinkHeartbeatServer Heartbeat { get; }
        IStatusTextServer StatusText { get; }
        IDebugServer Debug { get; }
        ICommandLongServer CommandLong { get; }
        ILoggingServer Logging { get; }
    }
}
