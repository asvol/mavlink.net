using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Server;
using Asv.Mavlink.V2.Common;
using NLog;

namespace Asv.Mavlink
{
    public class MavlinkPayloadServerConfig
    {
        public MavlinkServerIdentity Identity { get; set; } = new MavlinkServerIdentity { ComponenId = 13,SystemId = 13};
        public string ConnectionString { get; set; } = "udp://127.0.0.1:55555?rhost=127.0.0.1&rport=55554"; //"serial:COM5"
    }

    public class MessagePart
    {
        public MessagePart(PacketInfo packetInfo)
        {
            Parts = new SortedList<ushort, PacketInfo>(packetInfo.PacketCount);
            Parts.Add(packetInfo.PacketIndex,packetInfo);
            LastMessage = DateTime.Now;
        }

        public SortedList<ushort,PacketInfo> Parts { get; }
        public DateTime LastMessage { get; set; }

        public bool AddAndCheck(PacketInfo packetInfo)
        {
            // if double packet 
            if (Parts.ContainsKey(packetInfo.PacketIndex)) return false;

            Parts.Add(packetInfo.PacketIndex, packetInfo);
            LastMessage = DateTime.Now;
            // not enough
            return Parts.Count == packetInfo.PacketCount;
        }
    }

   
    public class MavlinkPayloadServer : IMavlinkPayloadServer
    {
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();
        private readonly IDictionary<string, Action<DeviceIdentity,Stream>> _dataCallbacks= new ConcurrentDictionary<string, Action<DeviceIdentity, Stream>>();
        private readonly MavlinkV2Connection _conn;
        private readonly MavlinkServerBase _srv;
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        private volatile int _isDisposed;
        private readonly RxValue<int> _txPacketMaxSize = new RxValue<int>();
        private readonly RxValue<int> _rxPacketMaxSize = new RxValue<int>();
        private readonly object _sync = new object();
        private readonly TimeSpan _dropOldPacketsTime = TimeSpan.FromSeconds(10);
        private readonly Dictionary<ushort, MessagePart> _dict = new Dictionary<ushort, MessagePart>();
        private int _packetId;

       

        public MavlinkPayloadServer(MavlinkPayloadServerConfig cfg)
        {
            _logger.Info($"Create mavlink payload server: cs:{cfg.ConnectionString}, comId:{cfg.Identity.ComponenId}, sysId:{cfg.Identity.SystemId}");
            _conn = new MavlinkV2Connection(cfg.ConnectionString, _ => _.RegisterCommonDialect());
            _srv = new MavlinkServerBase(_conn, cfg.Identity);
            _srv.Heartbeat.Set(_ =>
            {
                _.Autopilot = MavAutopilot.MavAutopilotGeneric;
                _.Type = MavType.MavTypeGeneric;
                _.SystemStatus = MavState.MavStateActive;
                _.MavlinkVersion = 3;
                _.CustomMode = PayloadHelper.HeartbeatMagicDigit;
            });
            _srv.Heartbeat.Start();
            _srv.V2Extension.OnData.Subscribe(OnData, _disposeCancel.Token);
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
                    Action<DeviceIdentity, Stream> callback;
                    if (!_dataCallbacks.TryGetValue(path, out callback))
                    {
                        _logger.Warn($"Receive unsuported data '{path}'");
                        return;
                    }
                    callback(new DeviceIdentity { ComponentId = v2ExtensionPacket.ComponenId, SystemId = v2ExtensionPacket.SystemId }, ms);
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

        private MemoryStream CheckNewPacketId(PacketInfo packetInfo)
        {
            // if single packet
            if (packetInfo.PacketCount == 1) return new MemoryStream(packetInfo.Data);
            lock (_sync)
            {
                MessagePart res;
                if (_dict.TryGetValue(packetInfo.PacketId, out res))
                {
                    if (res.AddAndCheck(packetInfo) == false) return null;
                    

                    var mem = new MemoryStream(res.Parts.Sum(_ => _.Value.DataCount));

                    foreach (var info in res.Parts)
                    {
                        mem.Write(info.Value.Data, 0, info.Value.DataCount);
                    }

                    mem.Position = 0;
                    _dict.Remove(packetInfo.PacketId);
                    return mem;
                }

                var item = new MessagePart(packetInfo);
                _dict.Add(packetInfo.PacketId, item);

                // Check old parts and remove them
                var toDelete = _dict.Where(_ => (DateTime.Now - _.Value.LastMessage) > _dropOldPacketsTime).Select(_=>_.Key).ToArray();
                foreach (var key in toDelete)
                {
                    _dict.Remove(key);
                }

                return null;
            }

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
                    await Send(devId, path, result, CancellationToken.None);
                }
                catch (Exception e)
                {
                    _logger.Warn(e, $"Error to execute '{path}':{e.Message}");
                    await SendError(devId, path, ErrorType.ArgsError, "Execute error", CancellationToken.None);
                }
            };
        }

        public async Task Send<T>(DeviceIdentity devId, string path, T data, CancellationToken cancel)
        {
            using (var strm = new MemoryStream())
            {
                PayloadHelper.WriteHeader(strm, path);
                PayloadHelper.WriteData(strm,data);
                
                if (_txPacketMaxSize.Value < strm.Length)
                {
                    _txPacketMaxSize.OnNext((int)strm.Length);
                }

                strm.Position = 0;
                await SendData(devId.SystemId, devId.ComponentId, PayloadHelper.DefaultNetworkId, PayloadHelper.DefaultSuccessMessageType,strm,cancel);
            }
        }

        private async Task SendData(byte targetSystemId, byte targetComponentId, byte targetNetworkId, ushort messageType, MemoryStream strm, CancellationToken cancel)
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
                            DataCount = (byte)count,
                            Data = buffer
                        };
                        PayloadHelper.SetInfo(wrt, info);
                        var length = wrtStream.Position;
                        var data = new byte[length];
                        wrtStream.Position = 0;
                        wrtStream.Read(data, 0, data.Length);
                        await _srv.V2Extension.SendData(targetSystemId,targetComponentId,targetNetworkId,messageType, data, cancel);
                        wrtStream.Position = 0;
                    }
                }
            }





        }


        public IRxValue<int> TxPacketMaxSize => _txPacketMaxSize;
        public IRxValue<int> RxPacketMaxSize => _rxPacketMaxSize;

        public async Task SendError(DeviceIdentity devId, string path, ErrorType errorType, string message, CancellationToken cancel)
        {
            try
            {
                using (var strm = new MemoryStream())
                {
                    PayloadHelper.WriteHeader(strm, path);
                    PayloadHelper.WriteData(strm, new ErrorCode { Msg = message, Res = errorType });
                    await _srv.V2Extension.SendData(devId.SystemId, devId.ComponentId, PayloadHelper.DefaultNetworkId, PayloadHelper.DefaultErrorMessageType, strm.ToArray(), cancel);
                }
            }
            catch (Exception e)
            {
                _logger.Error(e, $"Exception occured to send error. Type:{errorType:G}, Message:{message}");
            }
        }

        public IStatusTextServer Status => _srv.StatusText;
    }

    
}