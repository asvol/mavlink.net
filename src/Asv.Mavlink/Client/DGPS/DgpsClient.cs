using System;
using System.Reactive.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;
using NLog;

namespace Asv.Mavlink.Client
{
    public class DgpsClient : IDgpsClient
    {

        private readonly int MaxMessageLength = new GpsRtcmDataPayload().Data.Length;
        private readonly IMavlinkV2Connection _connection;
        private readonly IPacketSequenceCalculator _seq;
        private readonly MavlinkClientIdentity _identity;
        private readonly RxValue<V2ExtensionPacket> _onData = new RxValue<V2ExtensionPacket>();
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();
        private int _seqNumber;

        public DgpsClient(IMavlinkV2Connection connection, IPacketSequenceCalculator seq, MavlinkClientIdentity identity)
        {
            _connection = connection;
            _seq = seq;
            _identity = identity;
            
        }

        public void Dispose()
        {
            _disposeCancel.Cancel(false);
            _disposeCancel.Dispose();
        }

        public async Task SendRtcmData(byte[] data, int length, CancellationToken cancel)
        {
            if (length > MaxMessageLength * 4)
                _logger.Error($"RTCM message for DGPS is too large '{length}'");

            // number of packets we need, including a termination packet if needed
            var pktCount = length / MaxMessageLength + 1;
            if (pktCount >= 4)
            {
                pktCount = 4;
            }
                

            using (var linked = CancellationTokenSource.CreateLinkedTokenSource(_disposeCancel.Token, cancel))
            {
                for (var i = 0; i < pktCount; i++)
                {
                    var pkt = new GpsRtcmDataPacket()
                    {
                        ComponenId = _identity.ComponentId,
                        SystemId = _identity.SystemId,
                        CompatFlags = 0,
                        IncompatFlags = 0,
                        Sequence = _seq.GetNextSequenceNumber(),
                    };

                    // 1 means message is fragmented
                    pkt.Payload.Flags = (byte) (pktCount > 1 ? 1 : 0);
                    //  next 2 bits are the fragment ID
                    pkt.Payload.Flags += (byte)((i & 0x3) << 1);
                    // the remaining 5 bits are used for the sequence ID
                    pkt.Payload.Flags += (byte)((Interlocked.Increment(ref _seqNumber) & 0x1f) << 3);

                    var dataLength = Math.Min(length - i * MaxMessageLength, MaxMessageLength);
                    Array.Copy(data, i * MaxMessageLength, pkt.Payload.Data, 0, dataLength);

                    pkt.Payload.Len = (byte) dataLength;
                    await _connection.Send(pkt, linked.Token);
                }
                
            }
        }
    }
}
