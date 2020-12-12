using System;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Server;

namespace Asv.Mavlink
{
    public interface IMavlinkPayloadServer:IDisposable
    {
        IMavlinkV2Connection Connection { get; }

        void Register<TIn, TOut>(string path, DataDelegate<TIn,TOut> callback);
        Task SendResult<T>(DeviceIdentity devId, string path, T data, CancellationToken cancel = default, byte sendPacketCount = 1);
        Task SendError(DeviceIdentity devId, string path, string message, CancellationToken cancel = default, byte sendPacketCount = 1);
        IStatusTextServer Status { get; }
        IRxValue<int> RxPacketMaxSize { get; }
        IRxValue<int> TxPacketMaxSize { get; }
        IRxValue<int> RxPacketCount { get; }
        IRxValue<int> TxPacketCount { get; }
        IRxValue<int> RxDoubleCount { get; }
    }

    
}
