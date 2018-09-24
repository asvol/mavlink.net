using System;

namespace Asv.Mavlink
{
    public interface IPacketDecoder<TFrame, TPayload>: IDisposable, IObserver<byte>, IObservable<TFrame>
        where TFrame:IPacket<TPayload> where TPayload : IPayload
    {
        IObservable<DeserizliaePackageException> OutError { get; }
        void Register(Func<TFrame> factory);
    }
}