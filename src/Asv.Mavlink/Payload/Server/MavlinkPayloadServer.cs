using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.ComponentModel.Composition;
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
    public class MavlinkPayloadIdentity:MavlinkServerIdentity
    {
        public byte NetworkId { get; set; } = 0;
    }

    public class MavlinkPayloadServerConfig
    {
        public MavlinkPayloadIdentity Identity { get; set; } = new MavlinkPayloadIdentity { ComponenId = 13, SystemId = 13, NetworkId = 0};
        public byte DefaultPacketSendCount { get; set; } = 2;
        public string ConnectionString { get; set; } = "serial:/dev/modem?br=115200";
    }

    public class MavlinkPayloadServer : IMavlinkPayloadServer
    {
        private readonly MavlinkPayloadIdentity _identity;
        private readonly byte _defaultPacketSendCount;
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();
        private readonly IDictionary<string, Action<DeviceIdentity,Stream>> _dataCallbacks= new ConcurrentDictionary<string, Action<DeviceIdentity, Stream>>();
        private readonly MavlinkV2Connection _conn;
        private readonly MavlinkServerBase _srv;
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        private volatile int _isDisposed;
        private readonly RxValue<int> _txPacketMaxSize = new RxValue<int>();
        private readonly RxValue<int> _rxPacketMaxSize = new RxValue<int>();
        private readonly RxValue<int> _rxPacketCount = new RxValue<int>();
        private readonly RxValue<int> _txPacketCount = new RxValue<int>();
        private readonly RxValue<int> _rxDoublePacketCount = new RxValue<int>();
        private int _packetCounter;
        private readonly ConcurrentQueue<ushort> _packetIdCache = new ConcurrentQueue<ushort>();
        private int _maxPacketIdCacheSize = 15;

        public MavlinkPayloadServer(MavlinkPayloadIdentity identity, IDataStream dataStream, byte defaultPacketSendCount = 2)
        {
            _identity = identity;
            _defaultPacketSendCount = defaultPacketSendCount;
            _logger.Info($"Create mavlink payload server: dataStream:{dataStream}, comId:{identity.ComponenId}, sysId:{identity.SystemId}");
            _conn = new MavlinkV2Connection(dataStream, _ => _.RegisterCommonDialect());
            _srv = new MavlinkServerBase(_conn, identity);
            _srv.Heartbeat.Set(_ =>
            {
                _.Autopilot = MavAutopilot.MavAutopilotGeneric;
                _.Type = MavType.MavTypeGeneric;
                _.SystemStatus = MavState.MavStateActive;
                _.MavlinkVersion = 3;
                _.CustomMode = PayloadSerializerV2.HeartbeatMagicDigit;
            });
            _srv.Heartbeat.Start();
            _srv.V2Extension.OnData.Subscribe(OnData, _disposeCancel.Token);
        }

        public MavlinkPayloadServer(MavlinkPayloadServerConfig cfg)
        {
            _logger.Info($"Create mavlink payload server: cs:{cfg.ConnectionString}, comId:{cfg.Identity.ComponenId}, sysId:{cfg.Identity.SystemId}");
            _conn = new MavlinkV2Connection(cfg.ConnectionString, _ => _.RegisterCommonDialect());
            _srv = new MavlinkServerBase(_conn, cfg.Identity);
            _defaultPacketSendCount = cfg.DefaultPacketSendCount;
            _srv.Heartbeat.Set(_ =>
            {
                _.Autopilot = MavAutopilot.MavAutopilotGeneric;
                _.Type = MavType.MavTypeGeneric;
                _.SystemStatus = MavState.MavStateActive;
                _.MavlinkVersion = 3;
                _.CustomMode = PayloadSerializerV2.HeartbeatMagicDigit;
            });
            _srv.Heartbeat.Start();
            _srv.V2Extension.OnData.Where(CheckPacketTarget).Where(_=>_.Payload.MessageType == PayloadSerializerV2.SuccessMessageTypeId).Subscribe(OnData, _disposeCancel.Token);
            _srv.V2Extension.OnData.Where(CheckPacketTarget).Where(_=>_.Payload.MessageType == PayloadSerializerV2.ErrorMessageTypeId).Subscribe(OnError,_disposeCancel.Token);
        }

        public IMavlinkV2Connection Connection => _conn;

        private void OnError(V2ExtensionPacket v2ExtensionPacket)
        {
            try
            {
                using (var ms = new MemoryStream(v2ExtensionPacket.Payload.Payload))
                {
                    _rxPacketCount.Value++;
                    var header = PayloadSerializerV2.ReadHeader(ms);
                    if (FilterDoublePackets(header) == false)
                    {
                        _rxDoublePacketCount.Value++;
                        return;
                    }
                    var err = PayloadSerializerV2.ReadData<PayloadError>(ms);
                    _logger.Error($"Recv error packet ID:{header.PacketId}; {header.Path}: {err.ErrorMessage}");
                    if (_rxPacketMaxSize.Value < ms.Position)
                    {
                        _rxPacketMaxSize.OnNext((int)ms.Position);
                    }
                }
            }
            catch (Exception e)
            {
                _logger.Warn($"Error deserialize data:{e.Message}");
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

        private void OnData(V2ExtensionPacket v2ExtensionPacket)
        {
            try
            {
                using (var ms = new MemoryStream(v2ExtensionPacket.Payload.Payload))
                {
                    _rxPacketCount.Value++;
                    var header = PayloadSerializerV2.ReadHeader(ms);
                    if (FilterDoublePackets(header) == false)
                    {
                        _rxDoublePacketCount.Value++;
                        return;
                    }

                    if (!_dataCallbacks.TryGetValue(header.Path, out var callback))
                    {
                        _logger.Warn($"Receive unsupported data '{header.Path}'");
                        return;
                    }
                    callback(new DeviceIdentity { ComponentId = v2ExtensionPacket.ComponenId, SystemId = v2ExtensionPacket.SystemId}, ms);
                    if (_rxPacketMaxSize.Value < ms.Position)
                    {
                        _rxPacketMaxSize.OnNext((int)ms.Position);
                    }
                }
            }
            catch (Exception e)
            {
                _logger.Warn($"Error execute data:{e.Message}");
            }
        }

        

        private bool CheckPacketTarget(V2ExtensionPacket packet)
        {
            var network =  (packet.Payload.TargetNetwork == 0 || packet.Payload.TargetNetwork == _identity.NetworkId || _identity.NetworkId == 0);
            var system = (packet.Payload.TargetSystem == 0 || _identity.SystemId == 0 || packet.Payload.TargetSystem == _identity.SystemId);
            var component = (packet.Payload.TargetComponent == 0 || _identity.ComponenId == 0 || packet.Payload.TargetComponent == _identity.ComponenId);
            return network && system && component;
        }

        public void Register<TIn,TOut>(string path, DataDelegate<TIn,TOut> callback)
        {
            _dataCallbacks[path] = async (devId, strm) =>
            {
                TIn data;
                try
                {
                    data = PayloadSerializerV2.ReadData<TIn>(strm);
                }
                catch (Exception e)
                {
                    _logger.Warn(e, $"Error to deserialize input data '{path}'. Payload argument: {typeof(TIn).Name}");
                    await SendError(devId, path, e.Message, CancellationToken.None, _defaultPacketSendCount);
                    return;
                }

                try
                {
                    var result = await callback(devId, data);
                    await SendResult(devId, path, result, CancellationToken.None, _defaultPacketSendCount);
                }
                catch (CompositionException e)
                {
                    var err = GetStringFromException(e);
                    Status.Error($"{path}:{err}");
                    _logger.Warn(e, $"Error to execute '{path}':{e.Message}");
                    await SendError(devId, path, err, CancellationToken.None, _defaultPacketSendCount);
                }
                catch (Exception e)
                {
                    Status.Error($"{path}:{e.Message}");
                    _logger.Warn(e, $"Error to execute '{path}':{e.Message}");
                    await SendError(devId, path, e.Message, CancellationToken.None, _defaultPacketSendCount);
                }
            };
        }

        private string GetStringFromException(CompositionException e)
        {
            var ex = e.Errors.FirstOrDefault()?.Exception;
            if (ex == null) return e.Message;
            while (true)
            {
                if (ex is CompositionException exception)
                {
                    return GetStringFromException(exception);
                }
                if (ex.InnerException == null) return ex.Message;
                ex = ex.InnerException;
            }
        }

        public async Task SendResult<T>(DeviceIdentity devId, string path, T data, CancellationToken cancel = default, byte sendPacketCount = 1)
        {
            using (var strm = new MemoryStream())
            {
                PayloadSerializerV2.WriteHeader(strm, new PayloadPacketHeader
                {
                    PacketId = GetPacketId(),
                    Path = path,
                });
                PayloadSerializerV2.WriteData(strm, data);
                
                if (_txPacketMaxSize.Value < strm.Length)
                {
                    _txPacketMaxSize.OnNext((int)strm.Length);
                }

                strm.Position = 0;

                await SendData(devId.SystemId, devId.ComponentId, _identity.NetworkId, PayloadSerializerV2.SuccessMessageTypeId, strm, cancel, sendPacketCount);
            }
        }

        private ushort GetPacketId()
        {
            return (ushort) (Interlocked.Increment(ref _packetCounter) % ushort.MaxValue);
        }


        public async Task SendError(DeviceIdentity devId, string path,  string message, CancellationToken cancel = default, byte sendPacketCount = 1)
        {
            try
            {
                var msg = message.TrimToMaxLength(PayloadSerializerV2.MaxErrorMessageSize);
                using (var strm = new MemoryStream())
                {
                    PayloadSerializerV2.WriteHeader(strm, new PayloadPacketHeader
                    {
                        Path = path,
                        PacketId = GetPacketId(),
                    });
                    
                    PayloadSerializerV2.WriteData(strm, new PayloadError { ErrorMessage = msg });
                    strm.Position = 0;
                    await SendData(devId.SystemId, devId.ComponentId, _identity.NetworkId, PayloadSerializerV2.ErrorMessageTypeId, strm, cancel, sendPacketCount);
                }
            }
            catch (Exception e)
            {
                _logger.Error(e, $"Exception occured to send error. Message:{message}");
            }
        }

        private async Task SendData(byte targetSystemId, byte targetComponentId, byte targetNetworkId, ushort messageType, MemoryStream strm, CancellationToken cancel = default, int sendPacketCount = 1)
        {
            if (strm.Length > PayloadSerializerV2.V2ExtensionMaxDataSize) throw new Exception($"Packet size ({strm.Length}) too large to send. Max available size: {PayloadSerializerV2.V2ExtensionMaxDataSize} bytes");
            var data = new byte[strm.Length];
            await strm.ReadAsync(data, 0, data.Length,cancel);
            for (var i = 0; i < sendPacketCount; i++)
            {
                _txPacketCount.Value++;
                await _srv.V2Extension.SendData(targetSystemId, targetComponentId, targetNetworkId, messageType, data, cancel);
            }
        }

        public IRxValue<int> TxPacketMaxSize => _txPacketMaxSize;
        public IRxValue<int> RxPacketCount => _rxPacketCount;
        public IRxValue<int> TxPacketCount => _txPacketCount;
        public IRxValue<int> RxDoubleCount => _rxDoublePacketCount;
        public IRxValue<int> RxPacketMaxSize => _rxPacketMaxSize;

        public IStatusTextServer Status => _srv.StatusText;

        public void Dispose()
        {
            if (Interlocked.CompareExchange(ref _isDisposed, 1, 0) != 0) return;
            _disposeCancel?.Cancel(false);
            _disposeCancel?.Dispose();
            _conn?.Dispose();
            _srv?.Dispose();
            _dataCallbacks.Clear();
        }
    }

    
}
