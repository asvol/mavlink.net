using System;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink.Port
{
    public abstract class PortBase:IPort
    {
        private readonly CancellationTokenSource _disposedCancel = new CancellationTokenSource();
        private int _isEvaluating;
        private readonly RxValue<Exception> _portErrorStream = new RxValue<Exception>();
        private readonly RxValue<PortState> _portStateStream = new RxValue<PortState>();
        private readonly RxValue<bool> _enableStream = new RxValue<bool>();
        private readonly Subject<byte[]> _outputData = new Subject<byte[]>();

        public IRxValue<bool> IsEnabled => _enableStream;
        public TimeSpan ReconnectTimeout { get; set; } = TimeSpan.FromSeconds(5);
        public IRxValue<PortState> State => _portStateStream;

        public async Task Send(byte[] data, int count, CancellationToken cancel)
        {
            try
            {
                await InternalSend(data,count, cancel).ConfigureAwait(false);
            }
            catch (Exception exception)
            {
                InternalOnError(exception);
                throw;
            }
        }

        public IRxValue<Exception> Error => _portErrorStream;

        protected PortBase()
        {
            _enableStream.Where(_ => _).Subscribe(_=>TryConnect(), _disposedCancel.Token);
        }

        public void Enable()
        {
            _enableStream.OnNext(true);
        }

        public void Disable()
        {
            _enableStream.OnNext(false);
            _portStateStream.OnNext(PortState.Disabled);
            Stop();
        }

        private void Stop()
        {
            try
            {
                InternalStop();
            }
            finally
            {
                // ignore
            }
        }


        private void TryConnect()
        {
            if (Interlocked.CompareExchange(ref _isEvaluating, 1, 0) != 0) return;
            if (!_enableStream.Value) return;
            if (_disposedCancel.IsCancellationRequested) return;
            try
            {
                _portStateStream.OnNext(PortState.Connecting);
                InternalStart();
                _portStateStream.OnNext(PortState.Connected);
            }
            catch (Exception e)
            {
                InternalOnError(e);
            }
        }

        protected abstract Task InternalSend(byte[] data, int count, CancellationToken cancel);

        protected abstract void InternalStop();

        protected abstract void InternalStart();

        protected void InternalOnData(byte[] data)
        {
            try
            {
                _outputData.OnNext(data);
            }
            catch (Exception ex)
            {
                
            }
            finally
            {
                // ignore
            }
        }

        protected void InternalOnError(Exception exception)
        {
            _portStateStream.OnNext(PortState.Error);
            _portErrorStream.OnNext(exception);
            Observable.Timer(ReconnectTimeout).Subscribe(_ => TryConnect(), _disposedCancel.Token);
            Stop();
        }


        public IDisposable Subscribe(IObserver<byte[]> observer)
        {
            return _outputData.Subscribe(observer);
        }

        public void Dispose()
        {
            Disable();
            _portErrorStream.Dispose();
            _portStateStream.Dispose();
            _enableStream.Dispose();
            _outputData.Dispose();
        }
    }
}