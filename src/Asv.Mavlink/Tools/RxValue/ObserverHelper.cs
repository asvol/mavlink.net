using System;
using System.Diagnostics;
using System.Reactive.Linq;

namespace Asv.Mavlink
{
    public static class ObserverHelper
    {
        public static IObservable<T> IgnoreObserverExceptions<T, TException>(
            this IObservable<T> source
        ) where TException : Exception
        {
            return Observable.Create<T>(
                o => source.Subscribe(
                    v => {
                        try { o.OnNext(v); }
                        catch (TException) { }
                    },
                    ex => o.OnError(ex),
                    () => o.OnCompleted()
                ));
        }

        public static IObservable<T> IgnoreObserverExceptions<T>(
            this IObservable<T> source
        )
        {
            return Observable.Create<T>(
                o => source.Subscribe(
                    v => {
                        try { o.OnNext(v); }
                        catch
                        {
                            Debug.Assert(false,"Exception ignored");
                            // ignored
                        }
                    },
                    ex => o.OnError(ex),
                    () => o.OnCompleted()
                ));
        }
    }
}
