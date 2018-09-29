using System;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink.Port
{
    public enum PortState
    {
        Disabled,
        Connecting,
        Error,
        Connected
    }

    public interface IPort:IObservable<byte[]>,IDisposable
    {
        TimeSpan ReconnectTimeout { get; set; }
        IRxValue<bool> IsEnabled { get; }
        IRxValue<PortState> State { get; }
        IRxValue<Exception> Error { get; }
        void Enable();
        void Disable();
        Task Send(byte[] data, int count,CancellationToken cancel);
    }

    public static class PortFactory
    {
        public static IPort Create(string connectionString)
        {
            var uri = new Uri(connectionString);

            UdpPortConfig udp;
            if (UdpPortConfig.TryParseFromUri(uri, out udp)) return new UdpPort(udp);

            SerialPortConfig ser;
            if (SerialPortConfig.TryParseFromUri(uri, out ser)) return new CustomSerialPort(ser);

            throw new Exception(string.Format(RS.RemoteStreamFactory_CreateStream_Connection_string_is_invalid, connectionString));
        }
    }
}
