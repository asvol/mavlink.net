using System;
using System.Diagnostics;
using System.Globalization;
using System.IO.Ports;
using System.Reactive.Linq;
using System.Threading;
using System.Threading.Tasks;
using NLog;

namespace Asv.Mavlink
{
    public class CustomSerialPort : PortBase
    {
        private readonly SerialPortConfig _config;
        private SerialPort _serial;
        private readonly object _sync = new object();
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();
        private int _isReading;
        private IDisposable _readingTimer;

        public CustomSerialPort(SerialPortConfig config)
        {
            _config = config;
        }

        public override PortType PortType => PortType.Serial;

        protected override Task InternalSend(byte[] data, int count, CancellationToken cancel)
        {
            if (_serial == null) return Task.CompletedTask;
            lock (_sync)
            {
                if (!_serial.IsOpen) return Task.CompletedTask;
                return _serial.BaseStream.WriteAsync(data, 0, count, cancel);
            }
        }

        protected override void InternalStop()
        {
            if (_serial == null) return;
            lock (_sync)
            {
                if (_serial == null) return;
                try
                {
                    _readingTimer.Dispose();
                    if (_serial.IsOpen == true)
                        _serial.Close();

                }
                catch (Exception e)
                {
                    // ignore close errors
                }
                _serial.Dispose();
                _serial = null;
            }
        }

        protected override void InternalStart()
        {
            lock (_sync)
            {
                _serial = new SerialPort(_config.PortName, _config.BoundRate, _config.Parity, _config.DataBits, _config.StopBits);
                _serial.Open();
                _readingTimer = Observable.Timer(TimeSpan.FromMilliseconds(10),TimeSpan.FromMilliseconds(10)).Subscribe(TryReadData);
            }
        }

        private void TryReadData(long l)
        {
            if (Interlocked.CompareExchange(ref _isReading,1,0) !=0) return;
            byte[] data;
            try
            {
                lock (_sync)
                {
                    if (_serial.BytesToRead == 0) goto exit;
                    data = new byte[_serial.BytesToRead];
                    _serial.Read(data, 0, data.Length);
                }
                InternalOnData(data);
            }
            catch (Exception e)
            {
                _logger.Error(e, $"Error to read and push data from serial port:{e.Message}");
                Debug.Assert(false, e.Message);
            }
            
            exit:
            Interlocked.Exchange(ref _isReading, 0);
        }

    }
}