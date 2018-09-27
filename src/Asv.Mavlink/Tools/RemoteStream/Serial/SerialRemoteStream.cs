using System;
using System.IO.Ports;
using System.Reactive.Subjects;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink
{
    public class SerialRemoteStream : IRemoteStream
    {
        private readonly SerialRemoteStreamConfig _cfg;
        private SerialPort _serial;
        private readonly CancellationTokenSource _cancel = new CancellationTokenSource();
        private readonly Subject<byte[]> _dataSubject = new Subject<byte[]>();
        private SingleThreadTaskScheduler _ts;
        private TaskFactory _taskFactory;

        public SerialRemoteStream(SerialRemoteStreamConfig cfg)
        {
            this._cfg = cfg;
            _ts = new SingleThreadTaskScheduler("SERIAL");
            _taskFactory = new TaskFactory(_ts);
        }

        public IDisposable Subscribe(IObserver<byte[]> observer)
        {
            return _dataSubject.Subscribe(observer);
        }

        public void Dispose()
        {
            _taskFactory.StartNew(() =>
            {
                _serial.Dispose();
            }).Wait();
            _ts.Dispose();
            _cancel?.Cancel(false);
        }

        public Task Start(CancellationToken cancel)
        {
            return _taskFactory.StartNew(InternalStart, cancel);
        }

        private void InternalStart()
        {
            _serial = new SerialPort(_cfg.PortName, _cfg.BoundRate, _cfg.Parity, _cfg.DataBits, _cfg.StopBits);
            _serial.Open();
            _ts = new SingleThreadTaskScheduler("SERIAL");
            _taskFactory = new TaskFactory(_ts);
            _serial.DataReceived += (sender, args) => InternalRead() ;
        }

        private void InternalRead()
        {
            var data = new byte[_serial.BytesToRead];
            _serial.Read(data, 0, data.Length);
            _dataSubject.OnNext(data);
        }

        public Task Send(byte[] buffer, int count, CancellationToken cancel)
        {
            return _serial.BaseStream.WriteAsync(buffer, 0, count,cancel);
        }
    }
}