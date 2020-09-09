using System;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;

namespace Asv.Mavlink.Streams
{
    public interface IJsonStream : IDisposable, IObservable<JObject>
    {
        IObservable<Exception> OnError { get; }

        Task Send<T>(T data, CancellationToken cancel);

        Task<JObject> Request<TRequest>(TRequest request, Func<JObject, bool> responseFilter, CancellationToken cancel);
    }
}