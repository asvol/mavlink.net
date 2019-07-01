using System.Threading;

namespace Asv.Mavlink.Server
{
    public interface IPacketSequenceCalculator
    {
        byte GetNextSequenceNumber();
    }

    public class PacketSequenceCalculator : IPacketSequenceCalculator
    {
        private int _seq;

        public byte GetNextSequenceNumber()
        {
            return (byte)(Interlocked.Increment(ref _seq) % 255);
        }
    }
}
