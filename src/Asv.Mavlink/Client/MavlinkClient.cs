using System;
using NLog;

namespace Asv.Mavlink
{
    public class MavlinkClientIdentity
    {
        public byte SystemId { get; set; } = 254;
        public byte ComponentId { get; set; } = 254;
        public byte TargetSystemId { get; set; } = 1;
        public byte TargetComponentId { get; set; } = 1;
    }

    public class MavlinkClientConfig
    {
        public int CommandTimeoutMs { get; set; } = 5000;
        public int TimeoutToReadAllParamsMs { get; set; } = 10000;
        public int ReadParamTimeoutMs { get; set; } = 10000;
    }

    public class MavlinkClient : IMavlinkClient
    {
        private readonly IMavlinkV2Connection _mavlinkConnection;
        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();
        private readonly MavlinkTelemetry _rtt;
        private readonly MavlinkParameterMicroservice _params;
        private readonly MavlinkCommandMicroservice _mavlinkCommands;
        private readonly MavlinkMissionMicroservice _mission;
        private readonly MavlinkOffboardMode _mavlinkOffboard;
        private readonly MavlinkCommon _mode;
        
        public MavlinkClient(IMavlinkV2Connection connection, MavlinkClientIdentity identity, MavlinkClientConfig config)
        {
            if (connection == null) throw new ArgumentNullException(nameof(connection));
            if (config == null) throw new ArgumentNullException(nameof(config));
            Identity = identity;
            _mavlinkConnection = connection;
            _rtt = new MavlinkTelemetry(_mavlinkConnection, identity);
            _params = new MavlinkParameterMicroservice(_mavlinkConnection, identity,new VehicleParameterProtocolConfig {ReadWriteTimeoutMs = config.ReadParamTimeoutMs,TimeoutToReadAllParamsMs = config.TimeoutToReadAllParamsMs});
            _mavlinkCommands = new MavlinkCommandMicroservice(_mavlinkConnection, identity,new CommandProtocolConfig { CommandTimeoutMs = config.CommandTimeoutMs});
            _mission = new MavlinkMissionMicroservice(_mavlinkConnection,identity);
            _mavlinkOffboard = new MavlinkOffboardMode(_mavlinkConnection,identity);
            _mode = new MavlinkCommon(_mavlinkConnection,identity);
        }

        protected IMavlinkV2Connection Connection => _mavlinkConnection;
        public MavlinkClientIdentity Identity { get; }
        public IMavlinkTelemetry Rtt => _rtt;
        public IMavlinkParameterMicroservice Params => _params;
        public IMavlinkCommandMicroservice Commands => _mavlinkCommands;
        public IMavlinkMissionMicroservice Mission => _mission;
        public IMavlinkOffboardMode Offboard => _mavlinkOffboard;
        public IMavlinkCommon Common => _mode;

        public bool IsDisposed { get; private set; }

        public void Dispose()
        {
            if (IsDisposed) return;
            IsDisposed = true;
            try
            {
                _rtt.Dispose();
                _params.Dispose();
                _mavlinkCommands.Dispose();
                _mission.Dispose();
                _mavlinkOffboard?.Dispose();
                _mode?.Dispose();
            }
            catch (Exception e)
            {
                Logger.Error(e, $"Exeption occured disposing vehicle:{e.Message}");
            }   
        }

        
    }
}