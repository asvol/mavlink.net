using System;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Port;

namespace Asv.Mavlink
{
    public interface IMavlinkV2Connection:IObservable<IPacketV2<IPayload>>, IDisposable
    {
        IObservable<DeserizliaePackageException> DeserizliaePackageErrors { get; }
        IPort Port { get; }
        Task Send(IPacketV2<IPayload> packet, CancellationToken cancel);
    }
}