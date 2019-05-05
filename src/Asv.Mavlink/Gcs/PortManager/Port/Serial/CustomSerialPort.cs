using System;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink
{
    public class CustomSerialPort : PortBase
    {
        private readonly SerialPortConfig _config;
        private SerialPort _serial;

        public CustomSerialPort(SerialPortConfig config)
        {
            _config = config;
        }

        public override PortType PortType => PortType.Serial;

        protected override Task InternalSend(byte[] data, int count, CancellationToken cancel)
        {
            if (!_serial.IsOpen) return Task.CompletedTask;
            return _serial.BaseStream.WriteAsync(data, 0, count, cancel);
        }

        protected override void InternalStop()
        {
            if (_serial != null)
            {
                
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
            _serial = new SerialPort(_config.PortName, _config.BoundRate, _config.Parity, _config.DataBits, _config.StopBits);
            _serial.Open();
            _serial.DataReceived += _serial_DataReceived;
        }

        private void _serial_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            var data = new byte[_serial.BytesToRead];
            _serial.Read(data, 0, data.Length);
            InternalOnData(data);
        }
    }
}