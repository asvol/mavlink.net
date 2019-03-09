using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Reactive.Threading.Tasks;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Port;
using Asv.Mavlink.V2.Ardupilotmega;
using Asv.Mavlink.V2.Common;
using Asv.Mavlink.V2.Icarous;
using Asv.Mavlink.V2.Uavionix;
using Nito.AsyncEx;
using NLog;
using MavCmd = Asv.Mavlink.V2.Common.MavCmd;

namespace Asv.Mavlink
{
    public class VehicleConfig
    {
        public string ConnectionString { get; set; } = "0.0.0.0:14560";
        public int HeartbeatTimeoutMs { get; set; } = 2000;
        public byte SystemId { get; set; } = 254;
        public byte ComponentId { get; set; } = 254;
        public byte TargetSystemId { get; } = 1;
        public byte TargetComponentId { get; } = 1;
        public int CommandTimeoutMs { get; set; } = 1000;
        public int TimeoutToReadAllParamsMs { get; set; } = 10000;
        public int ReadParamTimeoutMs { get; set; } = 10000;
    }

    public class Vehicle : IVehicle
    {
        private readonly VehicleConfig _config;
        private readonly MavlinkV2Connection _mavlinkConnection;
        private readonly IPort _port;
        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();
        private readonly RawTelemetry _rtt;
        private readonly VehicleParameterProtocol _params;
        private readonly VehicleCommandProtocol _vehicleCommands;
        private readonly VehicleMissionProtocol _mission;
        private readonly OffboardMode _offboard;
        private readonly RxValue<DeserizliaePackageException> _packetErrors = new RxValue<DeserizliaePackageException>();
        private readonly IDisposable _packetErrorSubscribe;

        public Vehicle(VehicleConfig config)
        {
            if (config == null) throw new ArgumentNullException(nameof(config));

            _config = config;
            _port = PortFactory.Create(config.ConnectionString);
            _mavlinkConnection = new MavlinkV2Connection(_port, _ =>
            {
                _.RegisterCommonDialect();
                _.RegisterArdupilotmegaDialect();
                _.RegisterIcarousDialect();
                _.RegisterUavionixDialect();
            });
            _mavlinkConnection.Port.Enable();
            _rtt = new RawTelemetry(_mavlinkConnection,new RawTelemetryConfig { ComponentId = _config.ComponentId ,HeartbeatTimeoutMs = _config.HeartbeatTimeoutMs,SystemId = _config.SystemId});
            _params = new VehicleParameterProtocol(_mavlinkConnection,new VehicleParameterProtocolConfig { ComponentId = _config.ComponentId,  SystemId = _config.SystemId, ReadWriteTimeoutMs = _config.ReadParamTimeoutMs,TimeoutToReadAllParamsMs = _config.TimeoutToReadAllParamsMs});
            _vehicleCommands = new VehicleCommandProtocol(_mavlinkConnection,new CommandProtocolConfig { ComponentId = _config.ComponentId,CommandTimeoutMs = _config.CommandTimeoutMs,SystemId = _config.SystemId});
            _mission = new VehicleMissionProtocol(_mavlinkConnection,
                new VehicleMissionProtocolConfig {ComponentId = _config.ComponentId, SystemId = _config.SystemId});
            _offboard = new OffboardMode(_mavlinkConnection, new OffboardModeConfig());
            _packetErrorSubscribe = _packetErrors.Subscribe(_packetErrors);
        }

        protected IMavlinkV2Connection Connection => _mavlinkConnection;


        public IRxValue<Exception> PortError => _port.Error;
        public IRxValue<DeserizliaePackageException> OnPacketErrors => _packetErrors;

        public IRawTelemetry Rtt => _rtt;
        public IVehicleParameterProtocol Params => _params;
        public IVehicleCommandProtocol Commands => _vehicleCommands;
        public IVehicleMissionProtocol Mission => _mission;
        public IOffboardMode Offboard => _offboard;


        public bool IsDisposed { get; private set; }

        public void Dispose()
        {
            if (IsDisposed) return;
            IsDisposed = true;
            try
            {
                _packetErrorSubscribe?.Dispose();
                _port?.Dispose();
                _mavlinkConnection?.Dispose();
                _rtt.Dispose();
                _params.Dispose();
                _vehicleCommands.Dispose();
                _mission.Dispose();
            }
            catch (Exception e)
            {
                Logger.Error(e, $"Exeption occured disposing vehicle:{e.Message}");
            }   
        }

        
    }
}