using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Server;
using Asv.Mavlink.V2.Common;
using Nito.AsyncEx;
using NLog;

namespace Asv.Mavlink
{
    public class Result<T>
    {
        public bool IsError { get; set; }
        public string ErrorMessage { get; set; }
        public T Value { get; set; }
    }

    public class MavlinkPayloadClient : IMavlinkPayloadClient
    {
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();
        private readonly IMavlinkClient _client;
        private readonly byte _networkId;
        private readonly RxValue<VehicleStatusMessage> _logMessage = new RxValue<VehicleStatusMessage>();
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        private volatile int _isDisposed;
        private int _packetId;
        private readonly object _sync = new object();
        private readonly Subject<V2Packet> _onData = new Subject<V2Packet>();
        private readonly RxValue<int> _rxPacketCount = new RxValue<int>();
        private readonly RxValue<int> _txPacketCount = new RxValue<int>();
        private readonly RxValue<int> _rxDoublePacketCount = new RxValue<int>();
        private readonly ConcurrentQueue<ushort> _packetIdCache = new ConcurrentQueue<ushort>();
        private int _maxPacketIdCacheSize = 15;
        private int _packetCounter;

        public MavlinkPayloadClient(IMavlinkClient client, byte networkId = 0)
        {
            _client = client;
            _networkId = networkId;
            client.Rtt.RawStatusText.Select(ConvertLog).Subscribe(_logMessage,_disposeCancel.Token);
            client.V2Extension.OnData.Where(CheckPacketTarget).Subscribe(OnData, _disposeCancel.Token);
        }

        public IMavlinkClient Client => _client;

        public IRxValue<LinkState> Link => _client.Heartbeat.Link;
        public IRxValue<int> PacketRateHz => _client.Heartbeat.PacketRateHz;
        public IRxValue<int> RxPacketCount => _rxPacketCount;
        public IRxValue<int> TxPacketCount => _txPacketCount;
        public IRxValue<int> RxDoublePacketCount => _rxDoublePacketCount;

        public IRxValue<VehicleStatusMessage> OnLogMessage => _logMessage;

        private VehicleStatusMessage ConvertLog(StatustextPayload payload)
        {
            var message = new string(payload.Text.TakeWhile(_ => _ != (char)0).ToArray());
            return new VehicleStatusMessage {Type = payload.Severity, Text = message};
        }

        private bool CheckPacketTarget(V2ExtensionPacket packet)
        {
            var network = (packet.Payload.TargetNetwork == 0 || packet.Payload.TargetNetwork == _networkId || _networkId == 0);
            var system = (packet.Payload.TargetSystem == 0 || _client.Identity.SystemId == 0 || packet.Payload.TargetSystem == _client.Identity.SystemId);
            var component = (packet.Payload.TargetComponent == 0 || _client.Identity.ComponentId == 0 || packet.Payload.TargetComponent == _client.Identity.ComponentId);
            return network && system && component;
        }

        private void OnData(V2ExtensionPacket v2ExtensionPacket)
        {
            try
            {
                PayloadPacketHeader header;

                using (var ms = new MemoryStream(v2ExtensionPacket.Payload.Payload))
                {
                    _rxPacketCount.Value++;
                    header = PayloadSerializerV2.ReadHeader(ms);
                    if (FilterDoublePackets(header) == false)
                    {
                        _rxDoublePacketCount.Value++;
                        return;
                    }

                    var data = new byte[ms.Length - ms.Position];
                    ms.Read(data, 0, data.Length);
                    var pkt = new V2Packet
                    {
                        Device = new DeviceIdentity { ComponentId = v2ExtensionPacket.ComponenId, SystemId = v2ExtensionPacket.SystemId },
                        Path = header.Path,
                        Data = data,
                        MessageType = v2ExtensionPacket.Payload.MessageType,
                    };
                    _onData.OnNext(pkt);
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

        private ushort GetPacketId()
        {
            return (ushort)(Interlocked.Increment(ref _packetCounter) % ushort.MaxValue);
        }

        public async Task<TOut> Send<TIn, TOut>(string path, TIn data, CancellationToken cancel = default, byte sendPacketCount = 1)
        {
            using (var strm = new MemoryStream())
            {
                PayloadSerializerV2.WriteHeader(strm, new PayloadPacketHeader
                {
                    PacketId = GetPacketId(),
                    Path = path,
                });
                PayloadSerializerV2.WriteData(strm, data);
                var eve = new AsyncAutoResetEvent(false);

                var result = default(Result<TOut>);
                var dispose = Register<TOut>(path).FirstAsync(_ => true).Subscribe(_ =>
                {
                    result = _;
                    eve.Set();
                });
                try
                {
                    strm.Position = 0;
                    await SendData((byte)_networkId, PayloadSerializerV2.SuccessMessageTypeId, strm, cancel, sendPacketCount);
                    await eve.WaitAsync(cancel);
                    if (result.IsError)
                    {
                        throw new PayloadClientException(path, result.ErrorMessage);
                    }
                    Debug.Assert(result.Value != null);
                    return result.Value;
                }
                catch (Exception e)
                {
                    _logger.Error(e, $"Error to get interface implementations:{e.Message}");
                    throw;
                }
                finally
                {
                    dispose.Dispose();
                }
            }
        }

        private async Task SendData(byte defaultNetworkId, ushort defaultSuccessMessageType, MemoryStream strm, CancellationToken cancel = default, byte sendPacketCount = 1)
        {
            if (strm.Length > PayloadSerializerV2.V2ExtensionMaxDataSize) throw new Exception($"Packet size ({strm.Length}) too large to send. Max available size: {PayloadSerializerV2.V2ExtensionMaxDataSize} bytes");
            var data = new byte[strm.Length];
            await strm.ReadAsync(data, 0, data.Length, cancel);
            for (var i = 0; i < sendPacketCount; i++)
            {
                _txPacketCount.Value++;
                await _client.V2Extension.SendData(defaultNetworkId, defaultSuccessMessageType, data, cancel);
            }
        }

        public class V2Packet
        {
            public DeviceIdentity Device { get; set; }
            public string Path { get; set; }
            public byte[] Data { get; set; }
            public int MessageType { get; set; }
        }

        public IObservable<Result<TOut>> Register<TOut>(string path)
        {
            return _onData.Where(_=>_.Path == path).Select(_ =>
            {
                using (var strm = new MemoryStream(_.Data))
                {
                    try
                    {
                        if (!path.Equals(_.Path)) return default(Result<TOut>);
                        if (_.MessageType == PayloadSerializerV2.ErrorMessageTypeId)
                        {
                            var err = PayloadSerializerV2.ReadData<PayloadError>(strm);
                            return new Result<TOut>
                            {
                                ErrorMessage = err.ErrorMessage,
                                IsError = true,
                            };
                        }

                        if (_.MessageType == PayloadSerializerV2.SuccessMessageTypeId)
                        {
                            var data = PayloadSerializerV2.ReadData<TOut>(strm);
                            return new Result<TOut>
                            {
                                IsError = false,
                                Value = data,
                            };
                        }

                        return default(Result<TOut>);
                    }
                    catch (Exception e)
                    {
                        _logger.Warn(e, $"Error to deserialize input data '{path}'. Payload type {typeof(TOut).Name}");
                        return default(Result<TOut>);
                    }
                }
            }).Where(_=>!object.Equals(_,default(TOut)));
        }

        public void Dispose()
        {
            if (Interlocked.CompareExchange(ref _isDisposed,1,0) != 0) return;
            _disposeCancel?.Cancel(false);
            _disposeCancel?.Dispose();
        }
    }

   
}
