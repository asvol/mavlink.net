using System;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Server;

namespace Asv.Mavlink
{
    public interface IMavlinkPayloadServer:IDisposable
    {
        void Register<TIn, TOut>(string path, DataDelegate<TIn,TOut> callback);
        Task Send<T>(DeviceIdentity devId, string path, T data, CancellationToken cancel);
        Task SendError(DeviceIdentity devId, string path, ErrorType errorType, string message, CancellationToken cancel);
        IStatusTextServer Status { get; }
        IRxValue<int> RxPacketMaxSize { get; }
        IRxValue<int> TxPacketMaxSize { get; }
    }

    
}