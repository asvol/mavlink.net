using System;

namespace Asv.Mavlink
{
    public interface IPacketDecoder<TFrame>: IDisposable, IObserver<byte>, IObservable<TFrame>
    {
        IObservable<DeserizliaePackageException> OutError { get; }
        void Register(Func<TFrame> factory);
    }
}