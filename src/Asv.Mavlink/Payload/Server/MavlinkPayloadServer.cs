using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reactive.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Server;
using Asv.Mavlink.V2.Common;
using NLog;

namespace Asv.Mavlink
{
    public class MavlinkPayloadIdentity: MavlinkServerIdentity
    {
        public byte NetworkId { get; set; } = 0;
    }

    public class MavlinkPayloadServerConfig
    {
        public MavlinkPayloadIdentity Identity { get; set; } = new MavlinkPayloadIdentity { ComponenId = 13,SystemId = 13, NetworkId = 0};
        public string ConnectionString { get; set; } = "serial:/dev/modem?br=115200";
    }
   
    public class MavlinkPayloadServer : IMavlinkPayloadServer
    {
        private readonly MavlinkPayloadIdentity _identity;
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();
        private readonly IDictionary<string, Action<DeviceIdentity,Stream>> _dataCallbacks= new ConcurrentDictionary<string, Action<DeviceIdentity, Stream>>();
        private readonly MavlinkV2Connection _conn;
        private readonly MavlinkServerBase _srv;
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        private volatile int _isDisposed;
        private readonly ConcurrentQueue<ushort> _packetIdCache = new ConcurrentQueue<ushort>();
        private int _maxPacketIdCacheSize = 15;
        private int _doublePacketsCount;
        private int _packetCounter;

        public MavlinkPayloadServer(MavlinkPayloadIdentity identity, IDataStream dataStream)
        {
            _identity = identity;
            _logger.Info($"Create mavlink payload server: dataStream:{dataStream}, comId:{identity.ComponenId}, sysId:{identity.SystemId}, netId:{identity.NetworkId}");
            _conn = new MavlinkV2Connection(dataStream, _ => _.RegisterCommonDialect());
            _srv = new MavlinkServerBase(_conn, identity);
            _srv.Heartbeat.Set(_ =>
            {
                _.Autopilot = MavAutopilot.MavAutopilotGeneric;
                _.Type = MavType.MavTypeGeneric;
                _.SystemStatus = MavState.MavStateActive;
                _.MavlinkVersion = 3;
                _.CustomMode = PayloadHelper.HeartbeatMagicDigit;
            });
            _srv.Heartbeat.Start();
            _srv.V2Extension.OnData.Where(CheckPacketTarget).Subscribe(OnData, _disposeCancel.Token);
        }

        public MavlinkPayloadServer(MavlinkPayloadServerConfig cfg)
        {
            _logger.Info($"Create mavlink payload server: cs:{cfg.ConnectionString}, comId:{cfg.Identity.ComponenId}, sysId:{cfg.Identity.SystemId}, netId:{cfg.Identity.NetworkId}");
            _conn = new MavlinkV2Connection(cfg.ConnectionString, _ => _.RegisterCommonDialect());
            _srv = new MavlinkServerBase(_conn, cfg.Identity);
            _identity = cfg.Identity;
            _srv.Heartbeat.Set(_ =>
            {
                _.Autopilot = MavAutopilot.MavAutopilotGeneric;
                _.Type = MavType.MavTypeGeneric;
                _.SystemStatus = MavState.MavStateActive;
                _.MavlinkVersion = 3;
                _.CustomMode = PayloadHelper.HeartbeatMagicDigit;
            });
            _srv.Heartbeat.Start();
            _srv.V2Extension.OnData.Where(CheckPacketTarget).Subscribe(OnData, _disposeCancel.Token);
        }

        public IMavlinkV2Connection Connection => _conn;

        private bool CheckPacketTarget(V2ExtensionPacket packet)
        {
            var network = (packet.Payload.TargetNetwork == 0 || _identity.NetworkId == 0 || packet.Payload.TargetNetwork == _identity.NetworkId );
            var system = (packet.Payload.TargetSystem == 0 || _identity.SystemId == 0 || packet.Payload.TargetSystem == _identity.SystemId);
            var component = (packet.Payload.TargetComponent == 0 || _identity.ComponenId == 0 || packet.Payload.TargetComponent == _identity.ComponenId);
            return network && system && component;
        }

        private void OnData(V2ExtensionPacket v2ExtensionPacket)
        {
            try
            {
                PayloadPacketHeader header = null;

                using (var ms = new MemoryStream(v2ExtensionPacket.Payload.Payload))
                {
                    header = PayloadHelper.ReadHeader(ms);
                    if (FilterDoublePackets(header) == false)
                    {
                        Interlocked.Increment(ref _doublePacketsCount);
                        return;
                    }

                    if (!_dataCallbacks.TryGetValue(header.Path, out var callback))
                    {
                        //_logger.Warn($"Receive unsupported data '{packetInfo.Path}'");
                        return;
                    }
                    callback(new DeviceIdentity { ComponentId = v2ExtensionPacket.ComponenId, SystemId = v2ExtensionPacket.SystemId }, ms);
                }
            }
            catch (Exception e)
            {
                _logger.Warn($"Error execute data:{e.Message}");
            }
        }

        private bool FilterDoublePackets(PayloadPacketHeader header)
        {
            if (_packetIdCache.Contains(header.PacketId)) return false;
            _packetIdCache.Enqueue(header.PacketId);
            while (_packetIdCache.Count > _maxPacketIdCacheSize)
            {
                _packetIdCache.TryDequeue(out var id);
            }
            return true;
        }

        public void Dispose()
        {
            if (Interlocked.CompareExchange(ref _isDisposed,1,0) !=0) return;
            _disposeCancel?.Cancel(false);
            _disposeCancel?.Dispose();
            _conn?.Dispose();
            _srv?.Dispose();
            _dataCallbacks.Clear();
        }

        public void Register<TIn,TOut>(string path, DataDelegate<TIn,TOut> callback)
        {
            _dataCallbacks[path] = async (devId, strm) =>
            {
                TIn data;
                try
                {
                    data = PayloadHelper.ReadData<TIn>(strm);
                }
                catch (Exception e)
                {
                    _logger.Warn(e, $"Error to deserialize input data '{path}'. Payload type {typeof(TIn).Name}");
                    await SendError(devId, path, ErrorType.ArgsError,"Args error", CancellationToken.None);
                    return;
                }

                try
                {
                    var result = await callback(devId, data);
                    await SendResult(devId, path, result, CancellationToken.None);
                }
                catch (Exception e)
                {
                    Status.Error($"Execute {path}:{e.Message}");
                    _logger.Warn(e, $"Error to execute '{path}':{e.Message}");
                    await SendError(devId, path, ErrorType.InternalError, "Execute error", CancellationToken.None);
                }
            };
        }

        public async Task SendResult<T>(DeviceIdentity devId, string path, T data, CancellationToken cancel)
        {
            using (var strm = new MemoryStream())
            {
                PayloadHelper.WriteHeader(strm, new PayloadPacketHeader
                {
                    PacketId = GetPacketId(),
                    Path = path,
                });
                PayloadHelper.WriteData(strm,data);
                strm.Position = 0;
                await SendData(devId.SystemId, devId.ComponentId, PayloadHelper.DefaultNetworkId, PayloadHelper.DefaultSuccessMessageType,strm,cancel);
            }
        }
        private ushort GetPacketId()
        {
            return (ushort)(Interlocked.Increment(ref _packetCounter) % ushort.MaxValue);
        }
    


    public async Task SendError(DeviceIdentity devId, string path, ErrorType errorType, string message, CancellationToken cancel)
        {
            try
            {
                using (var strm = new MemoryStream())
                {
                    PayloadHelper.WriteHeader(strm, new PayloadPacketHeader
                    {
                        PacketId = GetPacketId(),
                        Path = path,
                    });
                    PayloadHelper.WriteData(strm, new ErrorCode { Msg = message, Res = errorType });
                    strm.Position = 0;
                    await SendData(devId.SystemId, devId.ComponentId, PayloadHelper.DefaultNetworkId, PayloadHelper.DefaultErrorMessageType, strm, cancel);
                }
            }
            catch (Exception e)
            {
                _logger.Error(e, $"Exception occured to send error. Type:{errorType:G}, Message:{message}");
            }
        }

        private async Task SendData(byte targetSystemId, byte targetComponentId, byte targetNetworkId, ushort messageType, MemoryStream strm, CancellationToken cancel)
        {
            if (strm.Length > PayloadHelper.V2ExtensionMaxDataSize)
                throw new Exception(
                    $"Packet size ({strm.Length}) too large to send. Max available size: {PayloadHelper.V2ExtensionMaxDataSize}");
            var data = new byte[strm.Length];
            await strm.ReadAsync(data, 0, data.Length, cancel);
            await _srv.V2Extension.SendData(targetSystemId,targetComponentId, targetNetworkId, messageType, data, cancel);
        }

        public IStatusTextServer Status => _srv.StatusText;
    }

    
}
