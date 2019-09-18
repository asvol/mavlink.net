using System;
using System.Collections.Generic;
using System.Linq;
using System.Reactive;
using System.Reactive.Subjects;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink
{
    public class PortWrapper : IDisposable
    {
        private readonly Action<PortWrapper, byte[], CancellationToken> _onRecv;
        private readonly CancellationTokenSource _cancel = new CancellationTokenSource();

        public PortWrapper(IPort port, string id, PortSettings settings, Action<PortWrapper, byte[], CancellationToken> onRecv)
        {
            _onRecv = onRecv;
            Port = port;
            Id = id;
            Settings = settings;
            port.Subscribe(OnNext, _cancel.Token);
        }

        private void OnNext(byte[] bytes)
        {
            _onRecv(this, bytes, _cancel.Token);
        }

        public IPort Port { get; }
        public string Id { get; }

        public PortSettings Settings { get; }

        public void Dispose()
        {
            _cancel.Cancel(false);
            _cancel.Dispose();
            Port.Dispose();
        }
    }

    public class PortInfo : IPortInfo
    {
        public string Id { get; }
        public PortSettings Settings { get; }
        public long RxAcc { get; }
        public long TxAcc { get; }
        public PortState State { get; set; }
        public PortType Type { get; set; }
        public Exception LastException { get; set; }

        public PortInfo(PortWrapper wraper)
        {
            Id = wraper.Id;
            Settings = wraper.Settings;
            RxAcc = wraper.Port.RxBytes;
            TxAcc = wraper.Port.TxBytes;
            LastException = wraper.Port.Error.Value;
            Type = wraper.Port.PortType;
            State = wraper.Port.State.Value;
        }
    }

    public class SubjectA<T> : ISubject<T>,IDisposable
    {
        private Subject<T> _s = new Subject<T>();
        public SubjectA()
        {
            
        }

        public void OnNext(T value)
        {
            _s.OnNext(value);
        }

        public void OnError(Exception error)
        {
            _s.OnError(error);
        }

        public void OnCompleted()
        {
            _s.OnCompleted();
        }

        public IDisposable Subscribe(IObserver<T> observer)
        {
            return _s.Subscribe(observer);
        }

        public void Dispose()
        {
            _s?.Dispose();
        }
    }

    public class PortManager:IPortManager
    {
        private readonly object _sync = new object();
        private readonly List<PortWrapper> _ports = new List<PortWrapper>();
        private readonly Subject<Unit> _configChangedSubject = new Subject<Unit>();
        private readonly SubjectA<byte[]> _onRecv = new SubjectA<byte[]>();

        public PortManager()
        {
            
        }

        public IPortInfo[] Ports => GetPortsInfo();

        private IPortInfo[] GetPortsInfo()
        {
            lock (_sync)
            {
                return _ports.Select(_ => new PortInfo(_)).Cast<IPortInfo>().ToArray();
            }
        }

        public void Add(PortSettings settings)
        {
            lock (_sync)
            {
                var port = PortFactory.Create(settings.ConnectionString);
                if (settings.IsEnabled)
                {
                    port.Enable();
                }
                else
                {
                    port.Disable();
                }
                _ports.Add(new PortWrapper(port, Guid.NewGuid().ToString(), settings, OnRecv));
            }
            _configChangedSubject.OnNext(Unit.Default);
        }

        private void OnRecv(PortWrapper sender, byte[] data,CancellationToken cancel)
        {
            PortWrapper[] ports;
            lock (_sync)
            {
                // repeat
                ports = _ports
                    // exclude self
                    .Where(_ => _.Id != sender.Id)
                    // exclude disabled
                    .Where(_ => _.Port.IsEnabled.Value)
                    // only connected
                    .Where(_ => _.Port.State.Value == PortState.Connected)
                    .ToArray();
            }
            _onRecv.OnNext(data);
            Task.WaitAll(ports.Select(_ => _.Port.Send(data, data.Length, cancel)).ToArray(), cancel);
        }

        public PortManagerSettings Save()
        {
            lock (_sync)
            {
                return new PortManagerSettings
                {
                    Ports = _ports.Select(_=>_.Settings).ToArray()
                };
            }
        }

        public void Enable(string portId)
        {
            lock (_sync)
            {
                var item = _ports.FirstOrDefault(_ => _.Id == portId);
                if (item == null) return;
                item.Port.Enable();
                item.Settings.IsEnabled = true;
            }
            _configChangedSubject.OnNext(Unit.Default);
        }

        public void Disable(string portId)
        {
            lock (_sync)
            {
                var item =_ports.FirstOrDefault(_ => _.Id == portId);
                if (item == null) return;
                item.Port.Disable();
                item.Settings.IsEnabled = false;
            }
            _configChangedSubject.OnNext(Unit.Default);
        }

        public void Load(PortManagerSettings settings)
        {
            lock (_sync)
            {
                foreach (var port in settings.Ports)
                {
                    Add(port);
                }
            }
        }

        public bool Remove(string portId)
        {
            lock (_sync)
            {
                var item = _ports.Find(_ => _.Id == portId);
                if (item == null) return false;
                item.Dispose();
                _ports.Remove(item);
                return true;
            }
        }

        public IObservable<Unit> OnConfigChanged => _configChangedSubject;

        public void Dispose()
        {
            PortWrapper[] ports;
            lock (_sync)
            {
                ports = _ports.Where(_ => _.Port.IsEnabled.Value).ToArray();
                _ports.Clear();
            }

            foreach (var port in ports)
            {
                port.Dispose();
            }
            
            _configChangedSubject?.Dispose();
            _onRecv?.Dispose();
        }

        public IDisposable Subscribe(IObserver<byte[]> observer)
        {
            return _onRecv.Subscribe(observer);
        }

        public Task Send(byte[] data, int count, CancellationToken cancel)
        {
            PortWrapper[] ports;
            lock (_sync)
            {
                ports = _ports.Where(_ => _.Port.IsEnabled.Value).ToArray();
            }
            return Task.WhenAll(ports.Select(_ => _.Port.Send(data, count, cancel)));
        }
    }
}