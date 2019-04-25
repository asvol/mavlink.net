using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using System.Web;

namespace Asv.Mavlink
{
    public class UdpPortConfig
    {
        public string LocalHost { get; set; }
        public int LocalPort { get; set; }
        public string RemoteHost { get; set; }
        public int RemotePort { get; set; }

        public static bool TryParseFromUri(Uri uri, out UdpPortConfig opt)
        {
            if (!"udp".Equals(uri.Scheme, StringComparison.InvariantCultureIgnoreCase))
            {
                opt = null;
                return false;
            }

            var coll = HttpUtility.ParseQueryString(uri.Query);
            
            opt = new UdpPortConfig
            {
                LocalHost = IPAddress.Parse(uri.Host).ToString(),
                LocalPort = uri.Port,
            };

            var rhost = coll["rhost"];
            if (!rhost.IsNullOrWhiteSpace())
            {
                opt.RemoteHost = IPAddress.Parse(rhost).ToString();
            }

            var rport = coll["rport"];
            if (!rport.IsNullOrWhiteSpace())
            {
                opt.RemotePort = int.Parse(rport);
            }
            return true;
        }
    }

    public class UdpPort : PortBase
    {
        private readonly IPEndPoint _recvEndPoint;
        private UdpClient _udp;
        private IPEndPoint _lastRecvEndpoint;
        private CancellationTokenSource _stop;
        private IPEndPoint _sendEndPoint;

        public UdpPort(UdpPortConfig config)
        {
            _recvEndPoint = new IPEndPoint(IPAddress.Parse(config.LocalHost),config.LocalPort);
            if (!config.RemoteHost.IsNullOrWhiteSpace() && config.RemotePort != 0)
            {
                _sendEndPoint = new IPEndPoint(IPAddress.Parse(config.RemoteHost), config.RemotePort);
            }
            
        }

        public override PortType PortType => PortType.Udp;

        protected override Task InternalSend(byte[] data, int count, CancellationToken cancel)
        {
            if (_udp?.Client == null || _udp.Client.Connected == false) return Task.CompletedTask;
            return _udp.SendAsync(data, count);
        }

        protected override void InternalStop()
        {
            _stop?.Cancel(false);
            _udp?.Dispose();
        }

        protected override void InternalStart()
        {
            _udp = new UdpClient(_recvEndPoint);
            if (_sendEndPoint != null)
            {
                _udp.Connect(_sendEndPoint);
            }
            _stop = new CancellationTokenSource();
            Task.Factory.StartNew(ListenAsync, _stop.Token, TaskCreationOptions.LongRunning);
        }

        private void ListenAsync(object obj)
        {
            try
            {
                var anyEp = new IPEndPoint(IPAddress.Any, _recvEndPoint.Port);
                while (true)
                {
                    var bytes = _udp.Receive(ref anyEp);
                    if (_lastRecvEndpoint == null )
                    {
                        _lastRecvEndpoint = anyEp;
                        _udp.Connect(_lastRecvEndpoint);
                    }

                    InternalOnData(bytes);
                }
            }
            catch (SocketException ex)
            {
                if (ex.SocketErrorCode == SocketError.Interrupted) return;
                InternalOnError(ex);
            }
            catch (Exception e)
            {
                InternalOnError(e);
            }
        }
    }
}