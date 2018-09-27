using System;
using System.Reactive.Subjects;

namespace Asv.Mavlink
{
    public class PacketEncoder<TFrame> : IPacketEncoder<TFrame> where TFrame : IPacket<IPayload>
    {
        private readonly byte[] _buffer;
        private readonly object _sync = new object();
        private readonly Subject<byte[]> _onData = new Subject<byte[]>();
        private readonly Subject<Exception> _outError = new Subject<Exception>();

        public PacketEncoder(int maxPacketLength)
        {
            _buffer = new byte[maxPacketLength];
        }

        public void Dispose()
        {
            _onData.Dispose();
            _outError.Dispose();
        }

        public IDisposable Subscribe(IObserver<byte[]> observer)
        {
            return _onData.Subscribe(observer);
        }

        public void OnNext(TFrame value)
        {
            byte[] data = null;
            Exception err = null;
            lock (_sync)
            {
                try
                {
                    var size = value.Serialize(_buffer, 0);
                    data = new byte[size];
                    Buffer.BlockCopy(_buffer, 0,data,0,data.Length);
                }
                catch (Exception e)
                {
                    err = e;
                }
               
            }
            if (err != null)
            {
                _outError.OnNext(err);
            }
            else
            {
                _onData.OnNext(data);
            }
        }

        public void OnError(Exception error)
        {
            throw new NotImplementedException();
        }

        public void OnCompleted()
        {
            _onData.OnCompleted();
            _outError.OnCompleted();
            Dispose();
        }

        public IObservable<Exception> OutError => _outError;
    }
}