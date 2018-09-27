using System;
using System.Net;

namespace Asv.Mavlink
{
    public class UdpRemoteStreamConfig
    {
        public IPEndPoint EndPoint { get; set; }

        public static bool TryGetOptions(Uri uri, out UdpRemoteStreamConfig opt)
        {
            if (!"udp".Equals(uri.Scheme, StringComparison.InvariantCultureIgnoreCase))
            {
                opt = null;
                return false;
            }

            opt = new UdpRemoteStreamConfig
            {
                EndPoint = new IPEndPoint(IPAddress.Parse(uri.Host), uri.Port),
            };
            return true;
        }
    }
}