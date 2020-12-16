using System;
using System.Collections.Generic;
using System.Linq;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Threading;
using Asv.Mavlink.V2.Ardupilotmega;
using Asv.Mavlink.V2.Common;
using Asv.Mavlink.V2.Icarous;
using Asv.Mavlink.V2.Uavionix;
using Newtonsoft.Json;
using NLog;

namespace Asv.Mavlink
{
    public class GroundControlStationIdentity
    {
        public byte SystemId { get; set; } = 254;
        public byte ComponentId { get; set; } = 254;
    }

    public class GroundControlStation : IGroundControlStation
    {
        private readonly GroundControlStationIdentity _config;
        private readonly TimeSpan _linkTimeout = TimeSpan.FromSeconds(3);
        private readonly CancellationTokenSource _cancel = new CancellationTokenSource();
        private readonly List<MavlinkDevice> _info = new List<MavlinkDevice>();
        private readonly ReaderWriterLockSlim _deviceListLock = new ReaderWriterLockSlim();
        private readonly Subject<IMavlinkDeviceInfo> _foundDeviceSubject = new Subject<IMavlinkDeviceInfo>();
        private readonly Subject<IMavlinkDeviceInfo> _lostDeviceSubject = new Subject<IMavlinkDeviceInfo>();
        private int _isDisposed;
        private static readonly Logger _logger = LogManager.GetCurrentClassLogger();

        public class MavlinkDeviceInfo : IMavlinkDeviceInfo
        {
            private readonly HeartbeatPacket _packet;

            public MavlinkDeviceInfo(HeartbeatPacket packet)
            {
                _packet = packet;
            }

            public int SystemId => _packet.SystemId;
            public int ComponentId => _packet.ComponenId;
            public uint CustomMode => _packet.Payload.CustomMode;
            public MavType Type => _packet.Payload.Type;
            public MavAutopilot Autopilot => _packet.Payload.Autopilot;
            public MavModeFlag BaseMode => _packet.Payload.BaseMode;
            public MavState SystemStatus => _packet.Payload.SystemStatus;
            public byte MavlinkVersion => _packet.Payload.MavlinkVersion;
        }

        class MavlinkDevice
        {
            private long _lastHit;
            public HeartbeatPacket Packet { get; }

            public MavlinkDevice(HeartbeatPacket packet)
            {
                Packet = packet;
                Touch();
            }

            public DateTime GetLastHit()
            {
                var lastHit = Interlocked.CompareExchange(ref _lastHit, 0, 0);
                return DateTime.FromBinary(lastHit);
            }

            public void Touch()
            {
                Interlocked.Exchange(ref _lastHit, DateTime.Now.ToBinary());
            }

            public IMavlinkDeviceInfo GetInfo()
            {
                return new MavlinkDeviceInfo(Packet);
            }
        }

        public GroundControlStation(GroundControlStationIdentity config)
        {
            if (config == null) throw new ArgumentNullException(nameof(config));
            _config = config;

            MavlinkV2 = new MavlinkV2Connection(Ports, _ =>
            {
                _.RegisterCommonDialect();
                _.RegisterArdupilotmegaDialect();
                _.RegisterIcarousDialect();
                _.RegisterUavionixDialect();
            });
            _cancel.Token.Register(() =>
            {
                _foundDeviceSubject.OnCompleted();
                _foundDeviceSubject.Dispose();
            });
            _cancel.Token.Register(() =>
            {
                _lostDeviceSubject.OnCompleted();
                _lostDeviceSubject.Dispose();
            });
            MavlinkV2.Where(_ => _.MessageId == HeartbeatPacket.PacketMessageId).Cast<HeartbeatPacket>().Subscribe(DeviceFounder, _cancel.Token);
            Observable.Timer(TimeSpan.FromSeconds(3), TimeSpan.FromSeconds(3)).Subscribe(_ => RemoveOldDevice(), _cancel.Token);
        }

       
        private void RemoveOldDevice()
        {
            _deviceListLock.EnterUpgradeableReadLock();
            var now = DateTime.Now;
            var deviceToRemove = _info.Where(_ => (now - _.GetLastHit()) > _linkTimeout).ToArray();
            if (deviceToRemove.Length != 0)
            {
                _deviceListLock.EnterWriteLock();
                foreach (var device in deviceToRemove)
                {
                    _info.Remove(device);
                    _logger.Info($"Delete device {JsonConvert.SerializeObject(device.GetInfo())}");
                }
                _deviceListLock.ExitWriteLock();
            }
            _deviceListLock.ExitUpgradeableReadLock();
            foreach (var dev in deviceToRemove)
            {
                _lostDeviceSubject.OnNext(dev.GetInfo());
            }

        }

        private void DeviceFounder(HeartbeatPacket packet)
        {
            MavlinkDevice newItem = null;
            _deviceListLock.EnterUpgradeableReadLock();
            var founded = _info.Find(_ => _.Packet.SystemId == packet.SystemId && _.Packet.ComponenId == packet.ComponenId);
            if (founded != null)
            {
                founded.Touch();
            }
            else
            {
                _deviceListLock.EnterWriteLock();
                newItem = new MavlinkDevice(packet);
                _info.Add(newItem);
                _deviceListLock.ExitWriteLock();
                _logger.Info($"Found new device {JsonConvert.SerializeObject(newItem.GetInfo())}");
            }
            _deviceListLock.ExitUpgradeableReadLock();

            if (newItem != null) _foundDeviceSubject.OnNext(newItem.GetInfo());
        }

        public GroundControlStationIdentity Identity => _config;
        public IPortManager Ports { get; } = new PortManager();
        public IMavlinkV2Connection MavlinkV2 { get; }
        public IObservable<IMavlinkDeviceInfo> OnFoundNewDevice => _foundDeviceSubject;
        public IObservable<IMavlinkDeviceInfo> OnLostDevice => _lostDeviceSubject;

        public IMavlinkDeviceInfo[] Devices
        {
            get
            {
                _deviceListLock.EnterReadLock();
                var items = _info.Select(_ => _.GetInfo()).ToArray();
                _deviceListLock.ExitReadLock();
                return items;
            }
        }

        public void Dispose()
        {
            if (Interlocked.CompareExchange(ref _isDisposed, 1, 0) != 0) return;
            _cancel.Cancel(false);
            _cancel.Dispose();
            Ports?.Dispose();
            MavlinkV2?.Dispose();
        }
    }
}
