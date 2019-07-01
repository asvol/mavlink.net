using System.Collections.Concurrent;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink.Decoder
{
    public class PacketV2DecoderSeparteThread : PacketV2Decoder
    {
        private readonly CancellationTokenSource _cancel = new CancellationTokenSource();
        private readonly BlockingCollection<byte> _fifo = new BlockingCollection<byte>();

        public PacketV2DecoderSeparteThread()
        {
            Task.Factory.StartNew(_ => SeparateThreadTick(), TaskCreationOptions.LongRunning);
        }

        private void SeparateThreadTick()
        {
            foreach (var value in _fifo.GetConsumingEnumerable(_cancel.Token))
            {
                base.OnNext(value);
            }
        }

        public override void OnNext(byte value)
        {
            _fifo.Add(value);
        }

        public override void Dispose()
        {
            _cancel?.Dispose();
            _fifo?.Dispose();
            base.Dispose();
        }
    }
}