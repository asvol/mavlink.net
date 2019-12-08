using System;
using System.Collections.Concurrent;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using NLog;

namespace Asv.Mavlink
{
    public class TcpServerPort : PortBase
    {
        private readonly TcpPortConfig _cfg;
        private TcpListener _tcp;
        private CancellationTokenSource _stop;
        private TcpClient _client;
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();

        public TcpServerPort(TcpPortConfig cfg)
        {
            _cfg = cfg;
        }

        public override PortType PortType { get; } = PortType.Tcp;

        protected override Task InternalSend(byte[] data, int count, CancellationToken cancel)
        {
            if (_tcp == null || _client == null || _client.Connected == false) return Task.CompletedTask;
            return _client.GetStream().WriteAsync(data, 0, count, cancel);
        }

        protected override void InternalStop()
        {
            _stop?.Cancel(false);
            _stop?.Dispose();
            _stop = null;

        }

        protected override void InternalStart()
        {
            var tcp = new TcpListener(IPAddress.Parse(_cfg.Host),_cfg.Port);
            tcp.Start();
            _tcp = tcp;
            _stop = new CancellationTokenSource();
            var recvThread = new Thread(RecvThreadCallback) { IsBackground = true, Priority = ThreadPriority.Lowest };
            _stop.Token.Register(() =>
            {
                try
                {
                    recvThread.Abort();
                    _client.Dispose();
                    _client = null;
                    _tcp.Stop();
                }
                catch (Exception e)
                {
                    // ignore
                }
            });
            recvThread.Start();

        }

        private void RecvThreadCallback(object obj)
        {
            try
            {
                while (true)
                {
                    if (_client == null)
                    {
                        _client = _tcp.AcceptTcpClient();
                        _logger.Info($"Accept tcp client {_client.Client.RemoteEndPoint}");
                    }
                    else
                    {
                        if (_client.Available != 0)
                        {
                            var buff = new byte[_client.Available];
                            _client.GetStream().Read(buff, 0, buff.Length);
                            InternalOnData(buff);
                        }
                    }
                    Thread.Sleep(100);
                }
            }
            catch (SocketException ex)
            {
                _client = null;
                if (ex.SocketErrorCode == SocketError.Interrupted) return;
                InternalOnError(ex);
            }
            catch (Exception e)
            {
                _client = null;
                InternalOnError(e);
            }


        }
    }
}