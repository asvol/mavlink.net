using System;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink
{
    public interface IRemoteStream : IObservable<byte[]>, IDisposable
    {
        Task Start(CancellationToken cancel);
        Task Send(byte[] buffer, int count, CancellationToken cancel);
    }
}