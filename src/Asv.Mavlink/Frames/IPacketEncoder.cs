using System;

namespace Asv.Mavlink
{
    public interface IPacketEncoder<in TPacket> : IDisposable, IObservable<byte[]>, IObserver<TPacket>
        where TPacket:IPacket<IPayload>
    {
        IObservable<Exception> OutError { get; }
    }
}