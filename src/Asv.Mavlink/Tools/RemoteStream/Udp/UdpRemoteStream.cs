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
                var data = _client.Receive(ref remoteEp);
                if (_sendEndpoint == null)
                {
                    _client.Connect(remoteEp);
                    _sendEndpoint = remoteEp;
                }
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
            Task.Factory.StartNew(AsyncRead, _cancel.Token, TaskCreationOptions.LongRunning, TaskScheduler.Default);
            return Task.CompletedTask;
        }

        public void OnNext(byte[] value)
        {
            if (_sendEndpoint == null) return;
            _client.Send(value, value.Length);
        }

        public void OnError(Exception error)
        {
            throw new NotImplementedException();
        }

        public void OnCompleted()
        {
            Dispose();
        }
    }
}
