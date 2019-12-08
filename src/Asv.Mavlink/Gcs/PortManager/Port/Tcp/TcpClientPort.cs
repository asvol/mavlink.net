using System;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using System.Web;

namespace Asv.Mavlink
{
    public class TcpClientPort:PortBase
    {
        private TcpPortConfig _cfg;
        private TcpClient _tcp;
        private CancellationTokenSource _stop;

        public TcpClientPort(TcpPortConfig cfg)
        {
            _cfg = cfg;
        }

        public override PortType PortType { get; } = PortType.Tcp;

        protected override Task InternalSend(byte[] data, int count, CancellationToken cancel)
        {
            if (_tcp == null || _tcp.Connected == false) return Task.CompletedTask;
            return _tcp.GetStream().WriteAsync(data, 0, count, cancel);
        }

        protected override void InternalStop()
        {
            _stop?.Cancel(false);
            _stop?.Dispose();
            _stop = null;

        }

        protected override void InternalStart()
        {
            var tcp = new TcpClient();
            tcp.Connect(_cfg.Host,_cfg.Port);
            _tcp = tcp;
            _stop = new CancellationTokenSource();
            var recvThread = new Thread(ListenAsync) { IsBackground = true, Priority = ThreadPriority.Lowest };
            _stop.Token.Register(() =>
            {
                try
                {
                    recvThread.Abort();
                    _tcp.Close();
                    _tcp.Dispose();
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
                while (true)
                {
                    if (_tcp.Available != 0)
                    {
                        var buff = new byte[_tcp.Available];
                        _tcp.GetStream().Read(buff, 0, buff.Length);
                        InternalOnData(buff);
                    }
                    else
                    {
                        Thread.Sleep(100);
                    }
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
            return _cfg.ToString();
        }

    }
}
