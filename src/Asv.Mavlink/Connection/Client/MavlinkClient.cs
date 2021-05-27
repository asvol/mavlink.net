using System;
using System.Threading;
using NLog;

namespace Asv.Mavlink.Client
{
    public class MavlinkClientIdentity
    {
        public byte SystemId { get; set; } = 254;
        public byte ComponentId { get; set; } = 254;
        public byte TargetSystemId { get; set; } = 1;
        public byte TargetComponentId { get; set; } = 1;

        public override string ToString()
        {
            return $"SYS:{SystemId};COM:{ComponentId};Target SYS:{TargetSystemId};COM:{TargetComponentId}";
        }
    }

    public class MavlinkClientConfig
    {
        public int CommandTimeoutMs { get; set; } = 5000;
        public int TimeoutToReadAllParamsMs { get; set; } = 60*60*1000;
        public int ReadParamTimeoutMs { get; set; } = 10000;
    }

    public class MavlinkClient : IMavlinkClient
    {
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        private readonly IMavlinkV2Connection _mavlinkConnection;
        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();
        private readonly MavlinkTelemetry _rtt;
        private readonly MavlinkParameterClient _params;
        private readonly MavlinkCommandClient _mavlinkCommands;
        private readonly MissionClient _mission;
        private readonly MavlinkOffboardMode _mavlinkOffboard;
        private readonly MavlinkCommon _mode;
        private readonly DebugClient _debugs;
        private readonly HeartbeatClient _heartbeat;
        private readonly LoggingClient _logging;
        private readonly DgpsClient _rtk;
        private IV2ExtensionClient _v2Ext;
        private readonly IPacketSequenceCalculator _seq;
        private int _isDisposed;


        public MavlinkClient(IMavlinkV2Connection connection, MavlinkClientIdentity identity, MavlinkClientConfig config, IPacketSequenceCalculator _sequence = null)
        {
            if (connection == null) throw new ArgumentNullException(nameof(connection));
            if (config == null) throw new ArgumentNullException(nameof(config));
            _seq = _sequence ?? new PacketSequenceCalculator();
            Identity = identity;
            _mavlinkConnection = connection;

            _rtt = new MavlinkTelemetry(_mavlinkConnection, identity);
            _disposeCancel.Token.Register(() => _rtt.Dispose());

            _params = new MavlinkParameterClient(_mavlinkConnection, identity,new VehicleParameterProtocolConfig {ReadWriteTimeoutMs = config.ReadParamTimeoutMs,TimeoutToReadAllParamsMs = config.TimeoutToReadAllParamsMs});
            _disposeCancel.Token.Register(() => _params.Dispose());

            _mavlinkCommands = new MavlinkCommandClient(_mavlinkConnection, identity, _seq,new CommandProtocolConfig { CommandTimeoutMs = config.CommandTimeoutMs});
            _disposeCancel.Token.Register(() => _mavlinkCommands.Dispose());

            _mission = new MissionClient(_mavlinkConnection,_seq, identity, new MissionClientConfig{ CommandTimeoutMs = config.CommandTimeoutMs});
            _disposeCancel.Token.Register(() => _mission.Dispose());

            _mavlinkOffboard = new MavlinkOffboardMode(_mavlinkConnection,identity);
            _disposeCancel.Token.Register(() => _mavlinkOffboard.Dispose());

            _mode = new MavlinkCommon(_mavlinkConnection,identity);
            _disposeCancel.Token.Register(() => _mode.Dispose());

            _debugs = new DebugClient(_mavlinkConnection, identity);
            _disposeCancel.Token.Register(() => _debugs.Dispose());

            _heartbeat = new HeartbeatClient(_mavlinkConnection,identity);
            _disposeCancel.Token.Register(() => _heartbeat.Dispose());

            _logging = new LoggingClient(_mavlinkConnection,identity);
            _disposeCancel.Token.Register(() => _logging.Dispose());

            _v2Ext = new V2ExtensionClient(_mavlinkConnection, _seq, identity);
            _disposeCancel.Token.Register(() => _v2Ext.Dispose());

            _rtk = new DgpsClient(_mavlinkConnection, _seq, identity);
            _disposeCancel.Token.Register(() => _rtt.Dispose());

            

        }

        protected IMavlinkV2Connection Connection => _mavlinkConnection;
        public MavlinkClientIdentity Identity { get; }
        public IHeartbeatClient Heartbeat => _heartbeat;
        public IMavlinkTelemetry Rtt => _rtt;
        public IMavlinkParameterClient Params => _params;
        public IMavlinkCommandClient Commands => _mavlinkCommands;
        public IMissionClient Mission => _mission;
        public IMavlinkOffboardMode Offboard => _mavlinkOffboard;
        public IMavlinkCommon Common => _mode;
        public IDebugClient Debug => _debugs;
        public ILoggingClient Logging => _logging;
        public IV2ExtensionClient V2Extension => _v2Ext;
        public IDgpsClient Rtk => _rtk;

        public void Dispose()
        {
            if (Interlocked.CompareExchange(ref _isDisposed,1,0) !=0) return;
            try
            {
                _disposeCancel?.Cancel(false);
                _disposeCancel?.Dispose();
            }
            catch (Exception e)
            {
                Logger.Error(e, $"Exeption occured disposing vehicle:{e.Message}");
            }   
        }

        
    }
}
