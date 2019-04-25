using System;
using NLog;

namespace Asv.Mavlink
{
    public class VehicleConfig
    {
        public byte SystemId { get; set; } = 254;
        public byte ComponentId { get; set; } = 254;
        public byte TargetSystemId { get; } = 1;
        public byte TargetComponentId { get; } = 1;
        public int CommandTimeoutMs { get; set; } = 5000;
        public int TimeoutToReadAllParamsMs { get; set; } = 10000;
        public int ReadParamTimeoutMs { get; set; } = 10000;
    }

    public class MavlinkV2Protocol : IMavlinkV2Protocol
    {
        private readonly IMavlinkV2Connection _mavlinkConnection;
        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();
        private readonly MavlinkTelemetry _rtt;
        private readonly MavlinkParameterMicroservice _params;
        private readonly MavlinkCommandMicroservice _mavlinkCommands;
        private readonly MavlinkMissionMicroservice _mission;
        private readonly MavlinkOffboardMode _mavlinkOffboard;
        private readonly ModeProtocol _mode;

        public MavlinkV2Protocol(IMavlinkV2Connection connection, VehicleConfig config)
        {
            if (connection == null) throw new ArgumentNullException(nameof(connection));
            if (config == null) throw new ArgumentNullException(nameof(config));
            _mavlinkConnection = connection;
            _rtt = new MavlinkTelemetry(_mavlinkConnection,new RawTelemetryConfig { ComponentId = config.ComponentId , SystemId = config.SystemId});
            _params = new MavlinkParameterMicroservice(_mavlinkConnection,new VehicleParameterProtocolConfig { ComponentId = config.ComponentId,  SystemId = config.SystemId, ReadWriteTimeoutMs = config.ReadParamTimeoutMs,TimeoutToReadAllParamsMs = config.TimeoutToReadAllParamsMs});
            _mavlinkCommands = new MavlinkCommandMicroservice(_mavlinkConnection,new CommandProtocolConfig { ComponentId = config.ComponentId,CommandTimeoutMs = config.CommandTimeoutMs,SystemId = config.SystemId});
            _mission = new MavlinkMissionMicroservice(_mavlinkConnection,
                new VehicleMissionProtocolConfig {ComponentId = config.ComponentId, SystemId = config.SystemId});
            _mavlinkOffboard = new MavlinkOffboardMode(_mavlinkConnection, new OffboardModeConfig { ComponentId = config.ComponentId,SystemId = config.SystemId});
            _mode = new ModeProtocol(_mavlinkConnection,new MicroserviceConfigBase { ComponentId = config.ComponentId, SystemId = config.SystemId});
        }

        protected IMavlinkV2Connection Connection => _mavlinkConnection;

        public IMavlinkTelemetry Rtt => _rtt;
        public IMavlinkParameterMicroservice Params => _params;
        public IMavlinkCommandMicroservice Commands => _mavlinkCommands;
        public IMavlinkMissionMicroservice Mission => _mission;
        public IMavlinkOffboardMode Offboard => _mavlinkOffboard;
        public IMavlinkModeProtocol Mode => _mode;

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