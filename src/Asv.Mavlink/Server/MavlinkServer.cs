using System;

namespace Asv.Mavlink.Server
{
    public interface IMavlinkServer:IDisposable
    {
        IMavlinkHeartbeatServer Heartbeat { get; }
        IStatusTextLogger StatusText { get; }
        INamedValueServer NamedValue { get; }
        ICommandLongServer CommandLong { get; }
    }

    public class MavlinkServerBase:IMavlinkServer
    {
        private readonly IPacketSequenceCalculator _seq = new PacketSequenceCalculator();

        public MavlinkServerBase(IMavlinkV2Connection connection, MavlinkServerIdentity identity)
        {
            Heartbeat = new MavlinkHeartbeatServer(connection, _seq, identity, new MavlinkHeartbeatServerConfig
            {
                HeartbeatRateMs = 1000
            });
            StatusText = new StatusTextLogger(connection,_seq, identity,new StatusTextLoggerConfig
            {
                MaxQueueSize = 100,
                MaxSendRateHz = 10
            });
            NamedValue = new NamedValueServer(connection,_seq,identity);
            CommandLong = new CommandLongServer(connection,_seq,identity);
        }

        public IMavlinkHeartbeatServer Heartbeat { get; }
        public IStatusTextLogger StatusText { get; }
        public INamedValueServer NamedValue { get; }
        public ICommandLongServer CommandLong { get; }

        public void Dispose()
        {
            StatusText?.Dispose();
        }
    }
}
