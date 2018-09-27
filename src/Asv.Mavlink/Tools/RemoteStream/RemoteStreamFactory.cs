using System;

namespace Asv.Mavlink
{
    public static class RemoteStreamFactory
    {
        public static IRemoteStream CreateStream(string connectionString)
        {
            var uri = new Uri(connectionString);

            UdpRemoteStreamConfig udp;
            if (UdpRemoteStreamConfig.TryGetOptions(uri, out udp)) return new UdpRemoteStream(udp);

            SerialRemoteStreamConfig serialOpt;
            if (SerialRemoteStreamConfig.TryGetOptions(uri, out serialOpt)) return new SerialRemoteStream(serialOpt);

            throw new MavlinkException(string.Format(RS.RemoteStreamFactory_CreateStream_Connection_string_is_invalid, connectionString));
        }
    }
}