using System;
using System.Collections.Generic;
using System.Linq;
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

    public class PortManager:IPortManager
    {
        private readonly object _sync = new object();
        private readonly List<PortWrapper> _ports = new List<PortWrapper>();

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
                _ports.Add(new PortWrapper(port, _ports.Count.ToString(), settings, OnRecv));
            }
        }

        private void OnRecv(PortWrapper sender, byte[] data,CancellationToken cancel)
        {
            lock (_sync)
            {
                Task.WaitAll(_ports.Where(_ => _ != sender).Select(_ => _.Port.Send(data, data.Length, cancel)).ToArray(),cancel);
            }
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
    }
}