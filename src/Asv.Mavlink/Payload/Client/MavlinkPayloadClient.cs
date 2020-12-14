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
        public ErrorCode Error { get; set; }
        public string ErrorMessage { get; set; }
        public T Value { get; set; }
    }

    public class MavlinkPayloadClient : IMavlinkPayloadClient
    {
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();
        private readonly byte _networkId;
        private readonly RxValue<VehicleStatusMessage> _logMessage = new RxValue<VehicleStatusMessage>();
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        private volatile int _isDisposed;
        private readonly Subject<V2Packet> _onData = new Subject<V2Packet>();
        private int _packetCounter;
        private readonly ConcurrentQueue<ushort> _packetIdCache = new ConcurrentQueue<ushort>();
        private int _maxPacketIdCacheSize = 15;
        private int _txPacketsCounter;
        private int _doublePacketsCounter;
        private readonly RxValue<double> _linkQualitySubject = new RxValue<double>();

        public MavlinkPayloadClient(IMavlinkClient client, byte networkId = 0)
        {
            Client = client;
            _networkId = networkId;
            client.Rtt.RawStatusText.Select(ConvertLog).Subscribe(_logMessage,_disposeCancel.Token);
            client.V2Extension.OnData.Where(CheckPacketTarget).Subscribe(OnData, _disposeCancel.Token);
            _onData.Select(_=>_.Header.PacketId).Buffer(TimeSpan.FromSeconds(1)).Where(_=>_.Count!=0 && (_.Last() -_.First())>=0).Select(_=>(_.Last() - _.First())/(double)_.Count).Subscribe(_linkQualitySubject, _disposeCancel.Token);
        }




        public IMavlinkClient Client { get; }

        public IRxValue<LinkState> Link => Client.Heartbeat.Link;
        public IRxValue<int> PacketRateHz => Client.Heartbeat.PacketRateHz;
        public IRxValue<VehicleStatusMessage> OnLogMessage => _logMessage;
        public IRxValue<double> LinkQuality => _linkQualitySubject;

        private VehicleStatusMessage ConvertLog(StatustextPayload payload)
        {
            var message = new string(payload.Text.TakeWhile(_ => _ != (char)0).ToArray());
            return new VehicleStatusMessage {Type = payload.Severity, Text = message};
        }

        private void OnData(V2ExtensionPacket v2ExtensionPacket)
        {
            try
            {
                Interlocked.Increment(ref _txPacketsCounter);
                
                PayloadPacketHeader packetInfo = null;

                using (var ms = new MemoryStream(v2ExtensionPacket.Payload.Payload))
                {
                    packetInfo = PayloadHelper.ReadHeader(ms);

                    if (FilterDoublePackets(packetInfo) == false)
                    {
                        Interlocked.Increment(ref _doublePacketsCounter);
                        return;
                    }


                    var data = new byte[ms.Length - ms.Position];
                    ms.Read(data, 0, data.Length);
                    var pkt = new V2Packet
                    {
                        Device = new DeviceIdentity { ComponentId = v2ExtensionPacket.ComponenId, SystemId = v2ExtensionPacket.SystemId },
                        Header = packetInfo,
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

        private bool CheckPacketTarget(V2ExtensionPacket packet)
        {
            var network = (packet.Payload.TargetNetwork == 0 || packet.Payload.TargetNetwork == _networkId || _networkId == 0);
            var system = (packet.Payload.TargetSystem == 0 || Client.Identity.SystemId == 0 || packet.Payload.TargetSystem == Client.Identity.SystemId);
            var component = (packet.Payload.TargetComponent == 0 || Client.Identity.ComponentId == 0 || packet.Payload.TargetComponent == Client.Identity.ComponentId);
            return network && system && component;
        }

        private ushort GetPacketId()
        {
            return (ushort) (Interlocked.Increment(ref _packetCounter) % ushort.MaxValue);
        }

        public async Task<TOut> Send<TIn, TOut>(string path, TIn data, CancellationToken cancel)
        {
            using (var strm = new MemoryStream())
            {
                PayloadHelper.WriteHeader(strm, new PayloadPacketHeader
                {
                    PacketId = GetPacketId(),
                    Path = path,
                });
                PayloadHelper.WriteData(strm, data);
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
                    await SendData(PayloadHelper.DefaultNetworkId, PayloadHelper.DefaultSuccessMessageType, strm, cancel);

                    await eve.WaitAsync(cancel);
                    if (result.IsError)
                    {
                        throw new PayloadClientException(path, result.Error, result.ErrorMessage);
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

        private async Task SendData(byte defaultNetworkId, ushort defaultSuccessMessageType, MemoryStream strm, CancellationToken cancel = default, int sendPacketCount = 1)
        {
            if (strm.Length > PayloadHelper.V2ExtensionMaxDataSize) throw new PayloadOversizeException($"Packet size ({strm.Length}) too large to send. Max available size: {PayloadHelper.V2ExtensionMaxDataSize}");
            var data = new byte[strm.Length];
            await strm.ReadAsync(data, 0, data.Length, cancel);
            for (int i = 0; i < sendPacketCount; i++)
            {
                await Client.V2Extension.SendData(defaultNetworkId, defaultSuccessMessageType, data, cancel);
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

        private class V2Packet
        {
            public DeviceIdentity Device { get; set; }
            public byte[] Data { get; set; }
            public int MessageType { get; set; }
            public PayloadPacketHeader Header { get; set; }
        }

        public IObservable<Result<TOut>> Register<TOut>(string path)
        {
            return _onData.Where(_=>_.Header.Path == path).Select(_ =>
            {
                
                using (var strm = new MemoryStream(_.Data))
                {
                    try
                    {
                        if (!path.Equals(_.Header.Path)) return default(Result<TOut>);
                        if (_.MessageType == PayloadHelper.DefaultErrorMessageType)
                        {
                            var err = PayloadHelper.ReadData<ErrorCode>(strm);
                            return new Result<TOut>
                            {
                                Error = err,
                                
                                IsError = true,
                            };
                        }

                        if (_.MessageType == PayloadHelper.DefaultSuccessMessageType)
                        {
                            var data = PayloadHelper.ReadData<TOut>(strm);
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
            _linkQualitySubject.Dispose();
            _logMessage.Dispose();
        }
    }

   
}
