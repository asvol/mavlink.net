using System;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink
{
    public class MavlinkPayloadClientInterfaceBase : IMavlinkPayloadClientInterface
    {
        private string _name;
        private volatile int _isDisposed;
        private IMavlinkPayloadClient _client;

        public MavlinkPayloadClientInterfaceBase(string name)
        {
            _name = name;
            PayloadHelper.ValidateName(name);
        }

        public string Name => _name;

        public virtual void Init(IMavlinkPayloadClient client)
        {
            _client = client ?? throw new ArgumentNullException(nameof(client));
        }

        protected IObservable<Result<TOut>> Register<TOut>(string path)
        {
            var absolutePath = PayloadHelper.PathJoin(_name, path);
            return _client.Register<TOut>(absolutePath);
        }

        protected Task<TOut> Send<TIn, TOut>(string path, TIn data, TimeSpan attemptTimeout, int attemptsCount, CancellationToken cancel, Action<int> progressCallback)
        {
            return _client.Send<TIn, TOut>(PayloadHelper.PathJoin(_name, path), data, attemptTimeout, attemptsCount, cancel, progressCallback);
        }

        public void Dispose()
        {
            if (Interlocked.CompareExchange(ref _isDisposed, 1, 0) != 0) return;
            InternalDisposeOnce();
        }

        protected virtual void InternalDisposeOnce()
        {

        }
    }
}
