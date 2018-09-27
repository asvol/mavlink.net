using System;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink
{
    public interface IRemoteStream : IObservable<byte[]>,IObserver<byte[]>, IDisposable
    {
        Task Start(CancellationToken cancel);
    }
}