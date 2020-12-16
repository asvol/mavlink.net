using System;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json;
using NLog;

namespace Asv.Mavlink
{
   

    public interface IMavlinkPayloadClient:IDisposable
    {
        IMavlinkClient Client { get; }
        IRxValue<LinkState> Link { get; }
        IRxValue<int> PacketRateHz { get; }
        IRxValue<double> LinkQuality { get; }
        IRxValue<VehicleStatusMessage> OnLogMessage { get; }
        IObservable<Result<TOut>> Register<TOut>(string path) where TOut : new();
        Task<TOut> Send<TIn,TOut>(string path, TIn data, CancellationToken cancel) where TOut : new();
    }

    public static class MavlinkPayloadClientHelper
    {
        private static readonly Logger _logger = LogManager.GetCurrentClassLogger();

        public static async Task<TOut> Send<TIn, TOut>(this IMavlinkPayloadClient src, string path, TIn data, TimeSpan timeout , int attemptsCount, CancellationToken cancel, Action<int> progressCallback) where TOut : new()
        {
            progressCallback = progressCallback ?? ((i) => { });
            var isSuccessfully = false;
            Exception lastError = null;
            var result = default(TOut);
            for (var i = 0; i < attemptsCount; i++)
            {
                progressCallback(i+1);
                cancel.ThrowIfCancellationRequested();
                _logger.Debug($"Call {path}. Attempt {i + 1} of {attemptsCount} with {timeout:g} timeout. Args {JsonConvert.SerializeObject(data)}");
                var tokenWithTimeout = new CancellationTokenSource(timeout);
                var linkedToken = CancellationTokenSource.CreateLinkedTokenSource(cancel, tokenWithTimeout.Token);
                try
                {
                    result = await src.Send<TIn, TOut>(path, data, linkedToken.Token);
                    isSuccessfully = true;
                    return result;
                }
                catch (TimeoutException e)
                {
                    lastError = e;
                    _logger.Warn(e, $"Timeout to call {path}. Attept {i + 1} of {attemptsCount}");
                }
                catch (Exception e)
                {
                    lastError = e;
                    _logger.Warn(e,$"Error to call {path}. Attept {i +1} of {attemptsCount}");
                }
                finally
                {
                    linkedToken?.Dispose();
                    tokenWithTimeout?.Dispose();
                }
            }

            throw new Exception($"Call {path} failed.", lastError);
        }
    }
}
