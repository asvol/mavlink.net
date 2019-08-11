namespace Asv.Mavlink.Server
{
    public class MavlinkServerBase:IMavlinkServer
    {
        private readonly IPacketSequenceCalculator _seq = new PacketSequenceCalculator();

        public MavlinkServerBase(IMavlinkV2Connection connection, MavlinkServerIdentity identity)
        {
            Heartbeat = new MavlinkHeartbeatServer(connection, _seq, identity, new MavlinkHeartbeatServerConfig
            {
                HeartbeatRateMs = 1000
            });
            StatusText = new StatusTextServer(connection,_seq, identity,new StatusTextLoggerConfig
            {
                MaxQueueSize = 100,
                MaxSendRateHz = 10
            });
            CommandLong = new CommandLongServer(connection,_seq,identity);
            Debug = new DebugServer(connection,_seq,identity);
            Logging = new LoggingServer(connection, _seq, identity);
            V2Extension = new V2ExtensionServer(connection,_seq,identity);
        }

        public IMavlinkHeartbeatServer Heartbeat { get; }
        public IStatusTextServer StatusText { get; }
        public IDebugServer Debug { get; }
        public ICommandLongServer CommandLong { get; }
        public ILoggingServer Logging { get; }
        public IV2ExtensionServer V2Extension { get; }


        public void Dispose()
        {
            StatusText?.Dispose();
        }
    }
}