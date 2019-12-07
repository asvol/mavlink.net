using System;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink
{
   

    public interface IMavlinkPayloadClient:IDisposable
    {
        IMavlinkClient Client { get; }
        IRxValue<LinkState> Link { get; }
        IRxValue<int> PacketRateHz { get; }
        IRxValue<VehicleStatusMessage> OnLogMessage { get; }
        IObservable<Result<TOut>> Register<TOut>(string path);
        Task<TOut> Send<TIn,TOut>(string path, TIn data, CancellationToken cancel);
    }
}