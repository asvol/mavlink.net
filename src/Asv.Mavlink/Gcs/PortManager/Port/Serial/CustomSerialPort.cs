using System;
using System.Diagnostics;
using System.Globalization;
using System.IO.Ports;
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
                    _serial.DataReceived -= _serial_DataReceived;
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
                _serial.DataReceived += _serial_DataReceived;
            }
        }

        private void _serial_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            byte[] data;
            lock (_sync)
            {
                data = new byte[_serial.BytesToRead];
                _serial.Read(data, 0, data.Length);
            }

            try
            {
                InternalOnData(data);
            }
            catch (Exception ex)
            {
                _logger.Error(ex,$"Error to push data:{ex.Message}");
                Debug.Assert(false,ex.Message);
            }

            
            
        }
    }
}