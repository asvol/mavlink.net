using System;
using System.Reactive.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink.Client
{
    public class V2ExtensionClient : IV2ExtensionClient
    {
        private readonly IMavlinkV2Connection _connection;
        private readonly IPacketSequenceCalculator _seq;
        private readonly MavlinkClientIdentity _identity;
        private readonly RxValue<V2ExtensionPacket> _onData = new RxValue<V2ExtensionPacket>();
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();

        public V2ExtensionClient(IMavlinkV2Connection connection, IPacketSequenceCalculator seq, MavlinkClientIdentity identity)
        {
            _connection = connection;
            _seq = seq;
            _identity = identity;
            connection
                .Where(_ => _.MessageId == V2ExtensionPacket.PacketMessageId)
                .Cast<V2ExtensionPacket>().Where(_ =>
                    (_.Payload.TargetSystem == 0 || _.Payload.TargetSystem == _identity.SystemId) &&
                    (_.Payload.TargetComponent == 0 || _.Payload.TargetComponent == _identity.ComponentId))
                .Subscribe(_onData,_disposeCancel.Token);
        }

        public void Dispose()
        {
            _disposeCancel.Dispose();
        }

        public IRxValue<V2ExtensionPacket> OnData => _onData;

        public Task SendData(byte targetNetworkId,ushort messageType, byte[] data, CancellationToken cancel)
        {
            var linked = CancellationTokenSource.CreateLinkedTokenSource(_disposeCancel.Token, cancel);
            return _connection.Send(new V2ExtensionPacket
            {
                ComponenId = _identity.ComponentId,
                SystemId = _identity.SystemId,
                CompatFlags = 0,
                IncompatFlags = 0,
                Sequence = _seq.GetNextSequenceNumber(),
                Payload =
                {
                    MessageType = messageType,
                    Payload = data,
                    TargetComponent = _identity.TargetComponentId,
                    TargetSystem = _identity.TargetSystemId,
                    TargetNetwork = targetNetworkId,
                }
            }, linked.Token);

        }
    }

    
}
