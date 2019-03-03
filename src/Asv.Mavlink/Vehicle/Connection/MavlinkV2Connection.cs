using System;
using System.Reactive.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Decoder;
using Asv.Mavlink.Port;

namespace Asv.Mavlink
{
    public class MavlinkV2Connection : IMavlinkV2Connection
    {
        private readonly PacketV2Decoder _decoder = new PacketV2Decoder();
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        public MavlinkV2Connection(string connectionString, Action<IPacketDecoder<IPacketV2<IPayload>>> register):this(PortFactory.Create(connectionString),register)
        {

        }

        public MavlinkV2Connection(IPort port, Action<IPacketDecoder<IPacketV2<IPayload>>> register)
        {
            Port = port;
            register(_decoder);
            Port.SelectMany(_ => _).Subscribe(_decoder, _disposeCancel.Token);
            Port.Enable();
        }

        public void Dispose()
        {
            _decoder.Dispose();
        }

        public IObservable<DeserizliaePackageException> DeserializePackageErrors => _decoder.OutError;
        public IPort Port { get; }

        public Task Send(IPacketV2<IPayload> packet, CancellationToken cancel)
        {
            var buffer = new byte[packet.GetMaxByteSize()];
            var size = packet.Serialize(buffer, 0);
            return Port.Send(buffer,size, cancel);
        }

        public IDisposable Subscribe(IObserver<IPacketV2<IPayload>> observer)
        {
            return _decoder.Subscribe(observer);
        }
    }
}
