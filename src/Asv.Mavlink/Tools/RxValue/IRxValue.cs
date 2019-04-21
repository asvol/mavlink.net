using System;

namespace Asv.Mavlink
{
    public interface IRxValue<out TValue> : IObservable<TValue>
    {
        TValue Value { get; }
    }

    public interface IRxEditableValue<TValue> : IRxValue<TValue>,IObserver<TValue>
    {
        
    }
}