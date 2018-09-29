using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink.Port
{
    public class UdpPortConfig
    {
        public string IpAddress { get; set; }
        public int Port { get; set; }

        public static bool TryParseFromUri(Uri uri, out UdpPortConfig opt)
        {
            if (!"udp".Equals(uri.Scheme, StringComparison.InvariantCultureIgnoreCase))
            {
                opt = null;
                return false;
            }

            opt = new UdpPortConfig
            {
                IpAddress = IPAddress.Parse(uri.Host).ToString(),
                Port = uri.Port,
            };
            return true;
        }
    }

    public class UdpPort : PortBase
    {
        private readonly IPEndPoint _endPoint;
        private UdpClient _udp;
        private IPEndPoint _lastRecvEndpoint;
        private CancellationTokenSource _stop;

        public UdpPort(UdpPortConfig config)
        {
            _endPoint = new IPEndPoint(IPAddress.Parse(config.IpAddress),config.Port);
        }

        protected override Task InternalSend(byte[] data, int count, CancellationToken cancel)
        {
            return _udp.SendAsync(data, count);
        }

        protected override void InternalStop()
        {
            _stop?.Cancel(false);
            _udp?.Dispose();
        }

        protected override void InternalStart()
        {
            _udp = new UdpClient(_endPoint);
            _stop = new CancellationTokenSource();
            Task.Factory.StartNew(ListenAsync, _stop.Token, TaskCreationOptions.LongRunning);
        }

        private void ListenAsync(object obj)
        {
            try
            {
                var anyEp = new IPEndPoint(IPAddress.Any, _endPoint.Port);
                while (true)
                {
                    var bytes = _udp.Receive(ref anyEp);
                    if (Equals(_endPoint.Address, IPAddress.Any))
                    {
                        _lastRecvEndpoint = anyEp;
                        _udp.Connect(_lastRecvEndpoint);
                    }
                    InternalOnData(bytes);
                }
            }
            catch (Exception e)
            {
                InternalOnError(e);
            }
        }
    }
}