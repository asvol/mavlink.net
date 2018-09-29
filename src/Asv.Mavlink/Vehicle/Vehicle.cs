using System;
using System.Linq;
using System.Reactive.Linq;
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
        public int SystemId { get; set; } = 254;
        public int ComponentId { get; set; } = 254;
        public int TargetSystemId { get; } = 0;
        public int TargetComponenId { get; } = 0;
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

        private readonly MavlinkV2Connection _mavlinkConnection;
        protected readonly IObservable<IPacketV2<IPayload>> InputPackets;
        protected readonly TaskFactory TaskFactory;
        protected readonly CancellationTokenSource DisposeCancel = new CancellationTokenSource();

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
        }

        private void HandleSystemStatus()
        {
            InputPackets
                .Where(_ => _.MessageId == SysStatusPacket.PacketMessageId)
                .Cast<SysStatusPacket>()
                .Select(_ => _.Payload)
                .Subscribe(_sysStatus, DisposeCancel.Token);
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

        public void Dispose()
        {
            _sysStatus.Dispose();
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