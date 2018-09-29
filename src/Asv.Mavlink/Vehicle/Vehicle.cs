using System;
using System.Linq;
using System.Reactive.Linq;
using System.Reactive.Threading.Tasks;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Port;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public class VehicleConfig
    {
        public string ConnectionString { get; set; } = "0.0.0.0:14560";
        public int HeartbeatTimeoutMs { get; set; } = 2000;
        public byte SystemId { get; set; } = 254;
        public byte ComponentId { get; set; } = 254;
        public byte TargetSystemId { get; } = 1;
        public byte TargetComponenId { get; } = 1;
        public int CommandTimeoutMs { get; set; } = 30000;
    }

    public class Vehicle : IVehicle
    {
        private readonly VehicleConfig _config;
        private readonly SingleThreadTaskScheduler _ts;
        
        private DateTime _lastHearteat;
        private readonly IPort _port;
        private readonly LinkIndicator _link = new LinkIndicator();
        private readonly RxValue<int> _packetRate = new RxValue<int>();
        private readonly RxValue<HeartbeatPayload> _heartBeat = new RxValue<HeartbeatPayload>();
        private readonly RxValue<SysStatusPayload> _sysStatus = new RxValue<SysStatusPayload>();
        private readonly RxValue<GpsRawIntPayload> _gpsRawInt = new RxValue<GpsRawIntPayload>();
        private readonly RxValue<HighresImuPayload> _highresImu = new RxValue<HighresImuPayload>();

        private readonly MavlinkV2Connection _mavlinkConnection;
        protected readonly IObservable<IPacketV2<IPayload>> InputPackets;
        protected readonly TaskFactory TaskFactory;
        protected readonly CancellationTokenSource DisposeCancel = new CancellationTokenSource();
        private readonly RxValue<VfrHudPayload> _vfrHud = new RxValue<VfrHudPayload>();
        private readonly RxValue<AttitudePayload> _attitude = new RxValue<AttitudePayload>();
        private readonly RxValue<BatteryStatusPayload> _batteryStatus = new RxValue<BatteryStatusPayload>();
        private readonly RxValue<AltitudePayload> _altitude = new RxValue<AltitudePayload>();
        private readonly RxValue<ExtendedSysStatePayload> _extendedSysState = new RxValue<ExtendedSysStatePayload>();

        public Vehicle(VehicleConfig config)
        {
            if (config == null) throw new ArgumentNullException(nameof(config));
            _config = config;
            _ts = new SingleThreadTaskScheduler("vehicle");
            TaskFactory = new TaskFactory(_ts);
            _port = PortFactory.Create(config.ConnectionString);
            _mavlinkConnection = new MavlinkV2Connection(_port, _ => _.RegisterCommonDialect());
            _mavlinkConnection.Port.Enable();
            InputPackets = _mavlinkConnection.Where(FilterVehicle);


            HandleStatistic();
            HandleHeartbeat(config);
            HandleSystemStatus();
            HandleGps();
            HandleHighresImu();
            HandleVfrHud();
            HandleAttitude();
            HandleBatteryStatus();
            HandleAltitude();
            HandleExtendedSysState();
        }

        private void HandleExtendedSysState()
        {
            InputPackets
                .Where(_ => _.MessageId == ExtendedSysStatePacket.PacketMessageId)
                .Cast<ExtendedSysStatePacket>()
                .Select(_ => _.Payload)
                .Subscribe(_extendedSysState, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _extendedSysState.Dispose());
        }

        private void HandleAltitude()
        {
            InputPackets
                .Where(_ => _.MessageId == AltitudePacket.PacketMessageId)
                .Cast<AltitudePacket>()
                .Select(_ => _.Payload)
                .Subscribe(_altitude, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _altitude.Dispose());
        }

        private void HandleBatteryStatus()
        {
            InputPackets
                .Where(_ => _.MessageId == BatteryStatusPacket.PacketMessageId)
                .Cast<BatteryStatusPacket>()
                .Select(_ => _.Payload)
                .Subscribe(_batteryStatus, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _batteryStatus.Dispose());
        }

        private void HandleAttitude()
        {
            InputPackets
                .Where(_ => _.MessageId == AttitudePacket.PacketMessageId)
                .Cast<AttitudePacket>()
                .Select(_ => _.Payload)
                .Subscribe(_attitude, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _attitude.Dispose());
        }

        private void HandleVfrHud()
        {
            InputPackets
                .Where(_ => _.MessageId == VfrHudPacket.PacketMessageId)
                .Cast<VfrHudPacket>()
                .Select(_ => _.Payload)
                .Subscribe(_vfrHud, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _vfrHud.Dispose());
        }

        private void HandleHighresImu()
        {
            InputPackets
                .Where(_ => _.MessageId == HighresImuPacket.PacketMessageId)
                .Cast<HighresImuPacket>()
                .Select(_ => _.Payload)
                .Subscribe(_highresImu, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _highresImu.Dispose());
        }

        private void HandleGps()
        {
            InputPackets
                .Where(_ => _.MessageId == GpsRawIntPacket.PacketMessageId)
                .Cast<GpsRawIntPacket>()
                .Select(_ => _.Payload)
                .Subscribe(_gpsRawInt, DisposeCancel.Token);

            DisposeCancel.Token.Register(() => _gpsRawInt.Dispose());
        }

        private void HandleSystemStatus()
        {
            InputPackets
                .Where(_ => _.MessageId == SysStatusPacket.PacketMessageId)
                .Cast<SysStatusPacket>()
                .Select(_ => _.Payload)
                .Subscribe(_sysStatus, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _sysStatus.Dispose());
        }

        private void HandleStatistic()
        {
            InputPackets
                .Select(_ => 1)
                .Buffer(TimeSpan.FromSeconds(1))
                .Select(_ => _.Sum()).Subscribe(_packetRate, DisposeCancel.Token);
        }

        private void HandleHeartbeat(VehicleConfig config)
        {
            InputPackets
                .Where(_ => _.MessageId == HeartbeatPacket.PacketMessageId)
                .Cast<HeartbeatPacket>()
                .Subscribe(_ => TaskFactory.StartNew(() => OnHeartBeat(_)), DisposeCancel.Token);
            Observable
                .Timer(TimeSpan.FromMilliseconds(config.HeartbeatTimeoutMs), TimeSpan.FromMilliseconds(config.HeartbeatTimeoutMs))
                .Subscribe(_ => TaskFactory.StartNew(() => CheckConnection(_)), DisposeCancel.Token);
        }

        private bool FilterVehicle(IPacketV2<IPayload> packetV2)
        {
            if (_config.TargetSystemId != 0 && _config.TargetSystemId != packetV2.SystemId) return false;
            if (_config.TargetComponenId != 0 && _config.TargetComponenId != packetV2.ComponenId) return false;
            return true;
        }

        private void CheckConnection(long value)
        {
            _ts.VerifyAccess();
            if (DateTime.Now - _lastHearteat > TimeSpan.FromMilliseconds(_config.HeartbeatTimeoutMs))
            {
                _link.Downgrade();
            }
        }

        private void OnHeartBeat(HeartbeatPacket heartbeatPacket)
        {
            _ts.VerifyAccess();
            _lastHearteat = DateTime.Now;
            _link.Upgrade();
            _heartBeat.OnNext(heartbeatPacket.Payload);
        }

        public IRxValue<LinkState> Link => _link;
        public IRxValue<int> PacketRateHz => _packetRate;
        public IRxValue<HeartbeatPayload> Heartbeat => _heartBeat;
        public IRxValue<SysStatusPayload> SysStatus => _sysStatus;
        public IRxValue<GpsRawIntPayload> GpsRawInt => _gpsRawInt;
        public IRxValue<HighresImuPayload> HighresImu => _highresImu;
        public IRxValue<ExtendedSysStatePayload> ExtendedSysState => _extendedSysState;
        public IRxValue<AltitudePayload> Altitude => _altitude;
        public IRxValue<BatteryStatusPayload> BatteryStatus => _batteryStatus;
        public IRxValue<AttitudePayload> Attitude => _attitude;
        public IRxValue<VfrHudPayload> VfrHud => _vfrHud;

        public async Task<CommandAckPayload> SendCommand(MavCmd command, float param1, float param2, float param3, float param4, float param5, float param6, float param7, int atteptCount, CancellationToken cancel)
        {
            var packet = new CommandLongPacket
            {
                ComponenId = _config.ComponentId,
                SystemId = _config.SystemId,
                Payload =
                {
                    Command = command,
                    TargetComponent = _config.TargetComponenId,
                    TargetSystem = _config.TargetSystemId,
                    Confirmation = 0,
                    Param1 = param1,
                    Param2 = param2,
                    Param3 = param3,
                    Param4 = param4,
                    Param5 = param5,
                    Param6 = param6,
                    Param7 = param7,
                }
            };
            byte currentAttept = 0;
            CommandAckPacket result = null;
            while (currentAttept < atteptCount)
            {
                packet.Payload.Confirmation = currentAttept;
                ++currentAttept;
                
                var timeoutCancel = new CancellationTokenSource(_config.CommandTimeoutMs);
                var linkedCancel = CancellationTokenSource.CreateLinkedTokenSource(cancel, timeoutCancel.Token);
                try
                {
                    var resultTask = InputPackets
                        .Where(_ => _.MessageId == CommandAckPacket.PacketMessageId)
                        .Where(_ => _config.TargetComponenId == 0 ||
                                    _config.TargetComponenId == _.ComponenId)
                        .Where(_=> _config.TargetSystemId == 0 ||
                                   _config.TargetSystemId == _.SystemId)
                        .Cast<CommandAckPacket>()
                        .Where(_ => _.Payload.TargetComponent == _config.ComponentId)
                        .Where(_ => _.Payload.TargetSystem == _config.SystemId)
                        .Take(1)
                        .ToTask(linkedCancel.Token);
                    await _mavlinkConnection.Send(packet, cancel).ConfigureAwait(false);
                    result = await resultTask.ConfigureAwait(false);
                    break;
                }
                catch (TaskCanceledException)
                {
                    if (!timeoutCancel.IsCancellationRequested)
                    {
                        throw;
                    }
                }
            }
            return result?.Payload;
        }

        public void Dispose()
        {
            
            _heartBeat.Dispose();
            _packetRate.Dispose();
            _link.Dispose();
            _port?.Dispose();
            _ts?.Dispose();
            _mavlinkConnection?.Dispose();
            DisposeCancel.Cancel(false);
        }
    }
}