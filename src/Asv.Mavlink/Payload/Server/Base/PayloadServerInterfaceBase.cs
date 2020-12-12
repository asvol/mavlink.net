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
            PayloadSerializerV2.ValidateName(name);
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
                var absolutePath = PayloadSerializerV2.PathJoin(_name, path);
                _server.Register(absolutePath,callback);
            }
        }

        protected Task Send<T>(DeviceIdentity devId, string path, T data, CancellationToken cancel = default)
        {
            return _server.SendResult(devId, PayloadSerializerV2.PathJoin(_name, path), data, cancel);
        }

        protected Task SendError(DeviceIdentity devId, string path, string message, CancellationToken cancel = default, byte sendPacketCount = 1)
        {
            return _server.SendError(devId, PayloadSerializerV2.PathJoin(_name, path), message, cancel, sendPacketCount);
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
