using System;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Server;

namespace Asv.Mavlink
{
    public interface IMavlinkPayloadServer:IDisposable
    {
        IMavlinkV2Connection Connection { get; }
        void Register<TIn, TOut>(string path, DataDelegate<TIn,TOut> callback) where TIn : new();
        Task SendResult<T>(DeviceIdentity devId, string path, T data, CancellationToken cancel);
        Task SendError(DeviceIdentity devId, string path, ErrorType errorType, string message, CancellationToken cancel);
        IStatusTextServer Status { get; }
    }

    
}
