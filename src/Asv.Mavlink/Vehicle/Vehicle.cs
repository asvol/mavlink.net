﻿using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Reactive.Threading.Tasks;
using System.Threading;
using System.Threading.Tasks;
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
        private readonly IMavlinkV2Connection _mavlinkConnection;
        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();
        private readonly RawTelemetry _rtt;
        private readonly VehicleParameterProtocol _params;
        private readonly VehicleCommandProtocol _vehicleCommands;
        private readonly VehicleMissionProtocol _mission;
        private readonly OffboardMode _offboard;

        public Vehicle(IMavlinkV2Connection connection, VehicleConfig config)
        {
            if (connection == null) throw new ArgumentNullException(nameof(connection));
            if (config == null) throw new ArgumentNullException(nameof(config));
            _mavlinkConnection = connection;
            var config1 = config;
            _rtt = new RawTelemetry(_mavlinkConnection,new RawTelemetryConfig { ComponentId = config1.ComponentId ,HeartbeatTimeoutMs = config1.HeartbeatTimeoutMs,SystemId = config1.SystemId});
            _params = new VehicleParameterProtocol(_mavlinkConnection,new VehicleParameterProtocolConfig { ComponentId = config1.ComponentId,  SystemId = config1.SystemId, ReadWriteTimeoutMs = config1.ReadParamTimeoutMs,TimeoutToReadAllParamsMs = config1.TimeoutToReadAllParamsMs});
            _vehicleCommands = new VehicleCommandProtocol(_mavlinkConnection,new CommandProtocolConfig { ComponentId = config1.ComponentId,CommandTimeoutMs = config1.CommandTimeoutMs,SystemId = config1.SystemId});
            _mission = new VehicleMissionProtocol(_mavlinkConnection,
                new VehicleMissionProtocolConfig {ComponentId = config1.ComponentId, SystemId = config1.SystemId});
            _offboard = new OffboardMode(_mavlinkConnection, new OffboardModeConfig());
        }

        protected IMavlinkV2Connection Connection => _mavlinkConnection;

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