using System;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink
{
    public interface IMavlinkV2Connection:IObservable<IPacketV2<IPayload>>, IDisposable
    {
        IObservable<DeserizliaePackageException> DeserializePackageErrors { get; }
        IPort Port { get; }
        Task Send(IPacketV2<IPayload> packet, CancellationToken cancel);
    }
}