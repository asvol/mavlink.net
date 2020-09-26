using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reactive.Linq;
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
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();
        private readonly List<TcpClient> _clients = new List<TcpClient>();
        private ReaderWriterLockSlim _rw = new ReaderWriterLockSlim();

        public TcpServerPort(TcpPortConfig cfg)
        {
            _cfg = cfg;
            Observable.Timer(TimeSpan.FromSeconds(1), TimeSpan.FromSeconds(3)).Where(_=>IsEnabled.Value).Subscribe(DeleteClients, DisposeCancel);
            DisposeCancel.Register(InternalStop);
        }

        private void DeleteClients(long l)
        {
            _rw.EnterUpgradeableReadLock();
            try
            {

                var itemsToDelete = _clients.Where(_ => _.Connected == false).ToArray();
                if (itemsToDelete.Length != 0)
                {
                    _rw.EnterWriteLock();
                    try
                    {
                        foreach (var tcpClient in itemsToDelete)
                        {
                            _clients.Remove(tcpClient);
                            _logger.Info($"Remove TCP client {tcpClient?.Client?.RemoteEndPoint}");
                        }
                    }
                    finally
                    {
                        _rw.ExitWriteLock();
                    }
                }
            }
            catch (Exception e)
            {
                _logger.Error(e, $"Error to delete TCP client:{e.Message}");
                Debug.Assert(false);
            }
            finally
            {
                _rw.ExitUpgradeableReadLock();
            }
            
        }

        public override PortType PortType { get; } = PortType.Tcp;

        protected override Task InternalSend(byte[] data, int count, CancellationToken cancel)
        {
            _rw.EnterReadLock();
            var clients = _clients.ToArray();
            _rw.ExitReadLock();
            return Task.WhenAll(clients.Select(_ => SendAsync(_, data, count, cancel)));
        }

        private async Task SendAsync(TcpClient client, byte[] data, int count, CancellationToken cancel)
        {
            if (_tcp == null || client == null || client.Connected == false) return;
            try
            {
                await client.GetStream().WriteAsync(data, 0, count, cancel);
            }
            catch (Exception e)
            {
                Debug.Assert(false);    
            }
        }

        protected override void InternalStop()
        {
            _stop?.Cancel(false);
            _stop?.Dispose();
            _stop = null;

        }

        protected override void InternalStart()
        {
            _rw.EnterWriteLock();
            _clients.Clear();
            _rw.ExitWriteLock();

            var tcp = new TcpListener(IPAddress.Parse(_cfg.Host),_cfg.Port);
            tcp.Start();
            _tcp = tcp;
            _stop = new CancellationTokenSource();
            var recvConnectionThread = new Thread(RecvConnectionCallback) { IsBackground = true, Priority = ThreadPriority.Lowest };
            var recvDataThread = new Thread(RecvDataCallback) { IsBackground = true, Priority = ThreadPriority.Lowest };
            _stop.Token.Register(() =>
            {
                try
                {
                    _tcp.Stop();
                    _rw.EnterWriteLock();
                    foreach (var client in _clients.ToArray())
                    {
                        client.Dispose();
                    }
                    _clients.Clear();
                    _rw.ExitWriteLock();

                    recvDataThread.Abort();
                    recvConnectionThread.Abort();
                }
                catch (Exception e)
                {
                    Debug.Assert(false);
                    // ignore
                }
            });
            recvDataThread.Start();
            recvConnectionThread.Start();

        }

        private void RecvConnectionCallback(object obj)
        {
            try
            {
                while (true)
                {
                    try
                    {
                        var newClient = _tcp.AcceptTcpClient();
                        _rw.EnterWriteLock();
                        _clients.Add(newClient);
                        _rw.ExitWriteLock();
                        _logger.Info($"Accept tcp client {newClient.Client.RemoteEndPoint}");
                    }
                    catch (ThreadAbortException e)
                    {
                        // ignore
                    }
                    catch (Exception e)
                    {
                        Debug.Assert(false);
                        // ignore
                    }

                }
            }
            catch (ThreadAbortException e)
            {
                // ignore
            }

        }

        private void RecvDataCallback(object obj)
        {
            try
            {
                while (true)
                {
                    _rw.EnterReadLock();
                    var clients = _clients.ToArray();
                    _rw.ExitReadLock();

                    foreach (var tcpClient in clients)
                    {
                        var data = RecvClientData(tcpClient);
                        if (data != null)
                        {
                            foreach (var otherClients in clients)
                            {
                                SendData(otherClients,data);
                            }
                            InternalOnData(data);
                        }
                        
                    }
                    Thread.Sleep(30);
                }
            }
            catch (SocketException ex)
            {
                if (ex.SocketErrorCode == SocketError.Interrupted) return;
                InternalOnError(ex);
            }
            catch (ThreadAbortException e)
            {
                // ignore
            }
            catch (Exception e)
            {
                InternalOnError(e);
            }


        }

        private void SendData(TcpClient tcpClient, byte[] data)
        {
            if (tcpClient.Available == 0) return;
            if (tcpClient.Connected == false) return;
            if (tcpClient.Client.Connected == false) return;
            try
            {
                tcpClient.GetStream().Write(data, 0, data.Length);
            }
            catch (Exception e)
            {
                // ignore
            }
        }

        private byte[] RecvClientData(TcpClient tcpClient)
        {
            if (tcpClient.Available == 0) return null;
            if (tcpClient.Connected == false) return null;
            if (tcpClient.Client.Connected == false) return null;
            var buff = new byte[tcpClient.Available];
            try
            {
                tcpClient.GetStream().Read(buff, 0, buff.Length);
            }
            catch (ThreadAbortException e)
            {
                // ignore
            }
            
            return buff;
        }

        

        public override string ToString()
        {
            return _cfg.ToString();
        }
    }
}
