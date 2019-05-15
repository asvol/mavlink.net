using System;
using System.Threading;

namespace Asv.Mavlink
{
    public interface IRxValue<out TValue> : IObservable<TValue>
    {
        TValue Value { get; }
    }

    public interface IRxEditableValue<TValue> : IRxValue<TValue>,IObserver<TValue>
    {
        
    }

    public static class RxValueHelper
    {
        public static IDisposable CreateAutocanceledWrapper<T>(this IRxValue<T> src,
            Action<T, CancellationToken> onConnectedCallback)
        {
            return new CancellationWrapper<T>(src, onConnectedCallback);
        }

        public static IDisposable CreateAutocanceledWrapper<T1,T2>(this IRxValue<T1> src, IRxValue<T2> src2,
            Action<T1,T2, CancellationToken> onConnectedCallback)
        {
            return new CancellationWrapper<T1,T2>(src,src2, onConnectedCallback);
        }
    }
}