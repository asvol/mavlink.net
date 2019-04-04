using System;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink
{
    

    public interface IMavlinkV2Connection:IObservable<IPacketV2<IPayload>>, IDisposable
    {
        long RxPackets { get; }
        long TxPackets { get; }
        long SkipPackets { get; }
        IObservable<DeserizliaePackageException> DeserializePackageErrors { get; }
        Task Send(IPacketV2<IPayload> packet, CancellationToken cancel);
    }
}