using System;
using System.Diagnostics;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink
{
    public abstract class PortBase : IPort
    {
        private readonly CancellationTokenSource _disposedCancel = new CancellationTokenSource();
        private int _isEvaluating;
        private readonly RxValue<Exception> _portErrorStream = new RxValue<Exception>();
        private readonly RxValue<PortState> _portStateStream = new RxValue<PortState>();
        private readonly RxValue<bool> _enableStream = new RxValue<bool>();
        private readonly Subject<byte[]> _outputData = new Subject<byte[]>();
        private long _rxBytes;
        private long _txBytes;

        public IRxValue<bool> IsEnabled => _enableStream;
        public long RxBytes => Interlocked.Read(ref _rxBytes);
        public long TxBytes => Interlocked.Read(ref _txBytes);
        public abstract PortType PortType { get; }
        public TimeSpan ReconnectTimeout { get; set; } = TimeSpan.FromSeconds(5);
        public IRxValue<PortState> State => _portStateStream;

        protected PortBase()
        {
            _enableStream.Where(_ => _).Subscribe(_ => Task.Factory.StartNew(TryConnect), _disposedCancel.Token);
        }

        public async Task Send(byte[] data, int count, CancellationToken cancel)
        {
            if (!IsEnabled.Value) return;
            if (_portStateStream.Value != PortState.Connected) return;
            try
            {
                await InternalSend(data, count, cancel).ConfigureAwait(false);
                Interlocked.Add(ref _txBytes, count);
            }
            catch (Exception exception)
            {
                InternalOnError(exception);
            }
        }

        public IRxValue<Exception> Error => _portErrorStream;

        public void Enable()
        {
            _enableStream.OnNext(true);
        }

        public void Disable()
        {
            _enableStream.OnNext(false);
            _portStateStream.OnNext(PortState.Disabled);
            Task.Factory.StartNew(Stop);
        }

        private void Stop()
        {
            try
            {
                InternalStop();
            }
            catch (Exception ex)
            {
                Debug.Assert(true,ex.Message);
            }
            finally
            {
                // ignore
            }
        }




        private void TryConnect()
        {
            if (Interlocked.CompareExchange(ref _isEvaluating, 1, 0) != 0) return;
            try
            {
                if (!_enableStream.Value) return;
                if (_disposedCancel.IsCancellationRequested) return;
                _portStateStream.OnNext(PortState.Connecting);
                InternalStart();
                _portStateStream.OnNext(PortState.Connected);
            }
            catch (Exception e)
            {
                InternalOnError(e);
            }
            finally
            {
                Interlocked.Exchange(ref _isEvaluating, 0);
            }
        }

        protected abstract Task InternalSend(byte[] data, int count, CancellationToken cancel);

        protected abstract void InternalStop();

        protected abstract void InternalStart();

        protected void InternalOnData(byte[] data)
        {
            try
            {
                Interlocked.Add(ref _rxBytes, data.Length);
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
            _outputData.OnCompleted();
            _outputData.Dispose();
        }
    }
}