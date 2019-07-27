using System;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Threading;
using System.Threading.Tasks;
using NLog;

namespace Asv.Mavlink
{
    public abstract class PortBaseV2:IPort
    {
        private readonly string _name;
        private readonly CancellationTokenSource _disposedCancel = new CancellationTokenSource();
        private readonly TaskFactory _taskFactory;
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();
        private readonly RxValue<Exception> _portErrorStream = new RxValue<Exception>();
        private readonly RxValue<PortState> _portStateStream = new RxValue<PortState>();
        private readonly RxValue<bool> _enableStream = new RxValue<bool>();
        private readonly Subject<byte[]> _outputData = new Subject<byte[]>();
        private long _rxBytes;
        private long _txBytes;
        private int _isDisposed;
        private readonly SingleThreadTaskScheduler _portScheduler;

        public PortBaseV2(string name)
        {
            _name = name;
            _portScheduler = new SingleThreadTaskScheduler(name);
            _taskFactory = new TaskFactory(_portScheduler);
            _enableStream.Subscribe(_ => TryReconnect(),_disposedCancel.Token);
            _disposedCancel.Token.Register(() => _portScheduler.Dispose());
            _disposedCancel.Token.Register(() => _portErrorStream.Dispose());
            _disposedCancel.Token.Register(() => _portStateStream.Dispose());
            _disposedCancel.Token.Register(() => _enableStream.Dispose());
            _disposedCancel.Token.Register(() => _outputData.Dispose());
        }


        public IDisposable Subscribe(IObserver<byte[]> observer)
        {
            return _outputData.Subscribe(observer);
        }

        public void Dispose()
        {
            if (Interlocked.CompareExchange(ref _isDisposed, 1, 0) == 1) return;
            Disable();
            _disposedCancel.Cancel(false);
            _disposedCancel.Dispose();
        }

        public IRxValue<bool> IsEnabled => _enableStream;
        public long RxBytes => Interlocked.Read(ref _rxBytes);
        public long TxBytes => Interlocked.Read(ref _txBytes);
        public IRxValue<Exception> Error => _portErrorStream;
        public abstract PortType PortType { get; }
        public TimeSpan ReconnectTimeout { get; set; } = TimeSpan.FromSeconds(5);
        public IRxValue<PortState> State => _portStateStream;

        public void Enable()
        {
            _taskFactory.StartNew(() => _enableStream.OnNext(true),_disposedCancel.Token);
        }

        public void Disable()
        {
            _taskFactory.StartNew(() => _enableStream.OnNext(false), _disposedCancel.Token);
        }

        private void TryReconnect()
        {
            _portScheduler.VerifyAccess();
            if (_portStateStream.Value == PortState.Disabled && _enableStream.Value == true)
            {
                InternalEnable();
            }

            if (_portStateStream.Value == PortState.Connected && _enableStream.Value == false)
            {
                InternalDisable();
            }

            if (_portStateStream.Value == PortState.Error && _enableStream.Value == true)
            {
                InternalDisable();
                InternalEnable();
            }
        }

        private void InternalEnable()
        {
            _portScheduler.VerifyAccess();
            if (_disposedCancel.IsCancellationRequested) return;

            try
            {
                _portStateStream.OnNext(PortState.Connecting);
                _logger.Trace("Try connect to the port {0}", _name);
                InternalStart();
                _portStateStream.OnNext(PortState.Connected);
                _portErrorStream.OnNext(null);
            }
            catch (Exception e)
            {
                _logger.Trace(e,$"Error to connect to the port {_name}:{e.Message}. Reconnect after {ReconnectTimeout:g}");
                _portErrorStream.OnNext(e);
                _portStateStream.OnNext(PortState.Error);
                Observable.Timer(ReconnectTimeout).Subscribe(_ => _taskFactory.StartNew(TryReconnect, _disposedCancel.Token));
            }
        }

        private void InternalDisable()
        {
            _portScheduler.VerifyAccess();
            if (_disposedCancel.IsCancellationRequested) return;
            try
            {
                _logger.Trace("Try disconnect port {0}", _name);
                InternalStop();
            }
            catch (Exception e)
            {
                _logger.Trace(e, $"Error to disconnect port {_name}:{e.Message}. Do nothing...");
            }

            _portStateStream.OnNext(PortState.Disabled);
        }

        protected abstract void InternalStart();
        protected abstract void InternalStop();
        protected abstract void InternalSend(byte[] data, int count);



        public async Task Send(byte[] data, int count, CancellationToken cancel)
        {
            if (_portScheduler.CheckAccess() == true)
            {

            }
            if (!IsEnabled.Value) return;
            if (_portStateStream.Value == PortState.Error || _portStateStream.Value == PortState.Disabled) return;
            CancellationTokenSource linkedCancel = null;
            try
            {
                linkedCancel = CancellationTokenSource.CreateLinkedTokenSource(cancel, _disposedCancel.Token);
                await _taskFactory.StartNew(() => InternalSend(data, count), linkedCancel.Token);
                Interlocked.Add(ref _txBytes, count);
            }
            catch (Exception e)
            {
                _logger.Trace(e, $"Error to send data to the port {_name}:{e.Message}. Reconnect after {ReconnectTimeout:g}");
                _portErrorStream.OnNext(e);
                _portStateStream.OnNext(PortState.Error);
                Observable.Timer(ReconnectTimeout).Subscribe(_ => _taskFactory.StartNew(TryReconnect, _disposedCancel.Token));
            }
            finally
            {
                linkedCancel?.Dispose();
            }
        }

        protected void InternalOnError(Exception ex)
        {
            _taskFactory.StartNew(() => PrivateOnError(ex));
        }

        private void PrivateOnError(Exception e)
        {
            _logger.Trace(e, $"Error occured from the port {_name}:{e.Message}. Reconnect after {ReconnectTimeout:g}");
            _portErrorStream.OnNext(e);
            _portStateStream.OnNext(PortState.Error);
            Observable.Timer(ReconnectTimeout).Subscribe(_ => _taskFactory.StartNew(TryReconnect, _disposedCancel.Token));
        }

        protected void InternalOnData(byte[] data)
        {
            try
            {
                Interlocked.Add(ref _rxBytes, data.Length);
                _outputData.OnNext(data);
            }
            catch (Exception ex)
            {
                _logger.Warn(ex,$"External error when ouput data from port:{ex.Message}");
            }
            finally
            {
                // ignore
            }
        }

    }
}
