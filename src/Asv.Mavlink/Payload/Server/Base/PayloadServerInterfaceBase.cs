using System;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Server;

namespace Asv.Mavlink
{
    

    public class PayloadServerInterfaceBase : IPayloadServerInterface
    {
        private readonly string _name;
        private IMavlinkPayloadServer _server;
        private volatile int _isDisposed;

        protected PayloadServerInterfaceBase(string name)
        {
            _name = name;
            PayloadHelper.ValidateName(name);
        }

        public string Name => _name;

        public virtual void Init(IMavlinkPayloadServer server)
        {
            _server = server;
        }

        protected void Register<TIn,TOut>(string path, DataDelegate<TIn,TOut> callback)
        {
            if (!path.StartsWith(_name, StringComparison.InvariantCultureIgnoreCase))
            {
                var absolutePath = PayloadHelper.PathJoin(_name, path);
                _server.Register(absolutePath,callback);
            }
        }

        protected Task Send<T>(DeviceIdentity devId, string path, T data, CancellationToken cancel)
        {
            return _server.Send(devId, PayloadHelper.PathJoin(_name, path), data, cancel);
        }

        protected Task SendError(DeviceIdentity devId, string path, ErrorType errorCode, string message, CancellationToken cancel)
        {
            return _server.SendError(devId, PayloadHelper.PathJoin(_name, path), errorCode, message, cancel);
        }

        protected IStatusTextServer Status => _server.Status;

        public void Dispose()
        {
            if (Interlocked.CompareExchange(ref _isDisposed,1,0) != 0) return;
            InternalDisposeOnce();
        }

        protected virtual void InternalDisposeOnce()
        {
            
        }
    }
}