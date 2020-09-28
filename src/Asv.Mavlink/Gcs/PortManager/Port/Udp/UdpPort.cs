using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using NLog;

namespace Asv.Mavlink
{
    public class UdpPort : PortBase
    {
        private readonly UdpPortConfig _config;
        private readonly IPEndPoint _recvEndPoint;
        private UdpClient _udp;
        private IPEndPoint _lastRecvEndpoint;
        private CancellationTokenSource _stop;
        private IPEndPoint _sendEndPoint;
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();

        public UdpPort(UdpPortConfig config)
        {
            _config = config;
            _recvEndPoint = new IPEndPoint(IPAddress.Parse(config.LocalHost), config.LocalPort);
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
            var recvThread = new Thread(ListenAsync) { IsBackground = true, Priority = ThreadPriority.Lowest };
            _stop.Token.Register(() =>
            {
                try
                {
                    recvThread.Abort();
                }
                catch (Exception e)
                {
                    // ignore
                }
            });
            recvThread.Start();
            
        }

        private void ListenAsync(object obj)
        {
            try
            {
                var anyEp = new IPEndPoint(IPAddress.Any, _recvEndPoint.Port);
                while (true)
                {
                    var bytes = _udp.Receive(ref anyEp);
                    if (_lastRecvEndpoint == null || _udp.Client.Connected == false)
                    {
                        _lastRecvEndpoint = anyEp;
                        _udp.Connect(_lastRecvEndpoint);
                        _logger.Info($"Recieve new UDP client {_lastRecvEndpoint}");
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

        public override string ToString()
        {
            return _config.ToString();
        }
    }
}
