﻿using System;

namespace Asv.Mavlink
{
    public interface IPacketDecoder<TFrame>: IDisposable, IObserver<byte>, IObservable<TFrame>
        where TFrame:ISerializable
    {
        IObservable<DeserializePackageException> OutError { get; }
        void Register(Func<TFrame> factory);
    }
}