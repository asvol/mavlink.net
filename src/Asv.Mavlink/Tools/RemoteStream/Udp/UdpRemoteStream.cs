using System;
using System.Net;
using System.Net.Sockets;
using System.Reactive.Subjects;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink
{
    public class UdpRemoteStream : IRemoteStream
    {
        private readonly UdpClient _client;
        private readonly CancellationTokenSource _cancel = new CancellationTokenSource();
        private readonly Subject<byte[]> _dataSubject = new Subject<byte[]>();
        private IPEndPoint _sendEndpoint;

        public UdpRemoteStream(UdpRemoteStreamConfig cfg)
        {
            _client = new UdpClient(cfg.EndPoint);
        }

        private void AsyncRead()
        {
            while (!_cancel.IsCancellationRequested)
            {
                var remoteEp = new IPEndPoint(IPAddress.Any,0);
                Interlocked.CompareExchange(ref _sendEndpoint, remoteEp, _sendEndpoint);
                var data = _client.Receive(ref remoteEp);
                _dataSubject.OnNext(data);
            }
        }

        public IDisposable Subscribe(IObserver<byte[]> observer)
        {
            return _dataSubject.Subscribe(observer);
        }

        public void Dispose()
        {
            _cancel?.Cancel(false);
        }

        public Task Start(CancellationToken cancel)
        {
            Task.Factory.StartNew(AsyncRead, _cancel.Token);
            return Task.CompletedTask;
        }

        public Task Send(byte[] buffer, int count, CancellationToken cancel)
        {
            return _sendEndpoint == null ? Task.FromResult(count) : _client.SendAsync(buffer, count, _sendEndpoint);
        }
    }
}
