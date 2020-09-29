using System;
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
        public T Value { get; set; }
    }

    public class MavlinkPayloadClient : IMavlinkPayloadClient
    {
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();
        private readonly IMavlinkClient _client;
        private readonly RxValue<VehicleStatusMessage> _logMessage = new RxValue<VehicleStatusMessage>();
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        private volatile int _isDisposed;
        private int _packetId;
        private readonly object _sync = new object();
        private readonly Dictionary<ushort, SortedList<ushort, PacketInfo>> _dict = new Dictionary<ushort, SortedList<ushort, PacketInfo>>();
        private readonly Subject<V2Packet> _onData = new Subject<V2Packet>();

        public MavlinkPayloadClient(IMavlinkClient client)
        {
            _client = client;
            client.Rtt.RawStatusText.Select(ConvertLog).Subscribe(_logMessage,_disposeCancel.Token);
            client.V2Extension.OnData.Subscribe(OnData, _disposeCancel.Token);
        }

        public IMavlinkClient Client => _client;

        public IRxValue<LinkState> Link => _client.Heartbeat.Link;
        public IRxValue<int> PacketRateHz => _client.Heartbeat.PacketRateHz;
        public IRxValue<VehicleStatusMessage> OnLogMessage => _logMessage;

        private VehicleStatusMessage ConvertLog(StatustextPayload payload)
        {
            var message = new string(payload.Text.TakeWhile(_ => _ != (char)0).ToArray());
            return new VehicleStatusMessage {Type = payload.Severity, Text = message};
        }

        private void OnData(V2ExtensionPacket v2ExtensionPacket)
        {
            try
            {
                PacketInfo packetInfo = null;

                using (var ms = new MemoryStream(v2ExtensionPacket.Payload.Payload))
                {
                    using (var rdr = new BinaryReader(ms))
                    {
                        packetInfo = PayloadHelper.GetInfo(rdr);
                    }
                }

                var memStream = CheckNewPacketId(packetInfo);
                if (memStream == null) return;

                using (var ms = memStream)
                {
                    string path;
                    PayloadHelper.ReadHeader(ms, out path);
                    var data = new byte[ms.Length - ms.Position];
                    ms.Read(data, 0, data.Length);
                    var pkt = new V2Packet
                    {
                        Device = new DeviceIdentity { ComponentId = v2ExtensionPacket.ComponenId, SystemId = v2ExtensionPacket.SystemId },
                        Path = path,
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

        private MemoryStream CheckNewPacketId(PacketInfo packetInfo)
        {
            if (packetInfo.PacketCount == 1) return new MemoryStream(packetInfo.Data);
            lock (_sync)
            {
                SortedList<ushort, PacketInfo> res;
                if (_dict.TryGetValue(packetInfo.PacketId, out res))
                {
                    res.Add(packetInfo.PacketIndex, packetInfo);
                    if (res.Count != packetInfo.PacketCount) return null;

                    var mem = new MemoryStream(res.Sum(_ => _.Value.DataCount));

                    foreach (var info in res)
                    {
                        mem.Write(info.Value.Data, 0, info.Value.DataCount);
                    }

                    mem.Position = 0;
                    return mem;
                }

                var list = new SortedList<ushort, PacketInfo> { { packetInfo.PacketIndex, packetInfo } };
                _dict.Add(packetInfo.PacketId, list);
                return null;
            }

        }

        public async Task<TOut> Send<TIn, TOut>(string path, TIn data, CancellationToken cancel)
        {
            using (var strm = new MemoryStream())
            {
                PayloadHelper.WriteHeader(strm, path);
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
                        throw new PayloadClientException(path, result.Error);
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

        private async Task SendData(byte defaultNetworkId, ushort defaultSuccessMessageType, MemoryStream strm, CancellationToken cancel)
        {
            var maxDataSize = (PayloadHelper.V2ExtensionMaxDataSize - PacketInfo.PacketInfoSize);
            var fullPacketCount = strm.Length / maxDataSize;
            var lastPartSize = strm.Length % maxDataSize;
            var packetCount = fullPacketCount + (lastPartSize == 0 ? 0 : 1);
            var packetId = (ushort)(Interlocked.Increment(ref _packetId) % ushort.MaxValue);

            using (var wrtStream = new MemoryStream(new byte[PayloadHelper.V2ExtensionMaxDataSize]))
            {
                using (var wrt = new BinaryWriter(wrtStream))
                {
                    var buffer = new byte[maxDataSize];
                    for (ushort i = 0; i < packetCount; i++)
                    {
                        var count = strm.Read(buffer, 0, buffer.Length);

                        var info = new PacketInfo
                        {
                            PacketId = packetId,
                            PacketIndex = i,
                            PacketCount = (ushort)packetCount,
                            DataCount = (byte) count,
                            Data = buffer
                        };
                        PayloadHelper.SetInfo(wrt, info);
                        var length = wrtStream.Position;
                        var data = new  byte[length];
                        wrtStream.Position = 0;
                        wrtStream.Read(data,0,data.Length);
                        await _client.V2Extension.SendData(defaultNetworkId, defaultSuccessMessageType, data, cancel);
                        wrtStream.Position = 0;
                    }
                }
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
        }
    }

   
}
