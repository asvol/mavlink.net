using System;
using System.Reactive.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Decoder;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    

    public interface IMavlinkV2Connection:IObserver<IPacketV2<IPayload>>,IObservable<IPacketV2<IPayload>>, IDisposable
    {
        Task Connect(string connectionString);
    }

    public class MavlinkV2Connection : IMavlinkV2Connection
    {
        private readonly PacketEncoder<IPacketV2<IPayload>> _encoder = new PacketEncoder<IPacketV2<IPayload>>(PacketV2Helper.PacketV2MaxSize);
        private readonly PacketV2Decoder _decoder = new PacketV2Decoder();
        private IRemoteStream _strm;
        private CancellationTokenSource _strmCancel;

        public MavlinkV2Connection(Action<IPacketDecoder<IPacketV2<IPayload>>> register)
        {
            register(_decoder);
        }

        public void Dispose()
        {
            _encoder.Dispose();
            _decoder.Dispose();
        }

        public Task Connect(string connectionString)
        {
            _strm = RemoteStreamFactory.CreateStream(connectionString);
            _strmCancel = new CancellationTokenSource();
            _strm.SelectMany(_=>_).Subscribe(_decoder, _strmCancel.Token);
            _encoder.Subscribe(_strm,_strmCancel.Token);
            return _strm.Start(CancellationToken.None);
        }

        public void OnNext(IPacketV2<IPayload> value)
        {
            _encoder.OnNext(value);
        }

        public void OnError(Exception error)
        {
            _encoder.OnError(error);
        }

        public void OnCompleted()
        {
            _encoder.OnCompleted();
        }

        public IDisposable Subscribe(IObserver<IPacketV2<IPayload>> observer)
        {
            return _decoder.Subscribe(observer);
        }
    }
}
