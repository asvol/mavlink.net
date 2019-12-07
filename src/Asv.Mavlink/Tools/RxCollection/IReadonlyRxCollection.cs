using System;
using System.Collections.Generic;

namespace Asv.Avialab.Core
{
    public interface IReadonlyRxCollection<out TModel>:IEnumerable<TModel>,IDisposable
    {
        IObservable<TModel> OnAdd { get; }
        IObservable<TModel> OnRemove { get; }
    }
}