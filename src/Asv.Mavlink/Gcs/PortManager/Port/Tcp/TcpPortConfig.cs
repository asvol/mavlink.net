using System;
using System.Net;
using System.Web;

namespace Asv.Mavlink
{
    public class TcpPortConfig
    {
        public string Host { get; set; }
        public int Port { get; set; }
        public bool IsServer { get; set; }

        public static bool TryParseFromUri(Uri uri, out TcpPortConfig opt)
        {
            if (!"tcp".Equals(uri.Scheme, StringComparison.InvariantCultureIgnoreCase))
            {
                opt = null;
                return false;
            }
            var coll = HttpUtility.ParseQueryString(uri.Query);
            opt = new TcpPortConfig
            {
                IsServer = bool.Parse(coll["srv"] ?? bool.FalseString),
                Host = IPAddress.Parse(uri.Host).ToString(),
                Port = uri.Port,
            };

            return true;
        }

        public override string ToString()
        {
            if (IsServer)
            {
                return $"TCP_S {Host}:{Port}";
            }
            else
            {
                return $"TCP_C {Host}:{Port}";
            }
            
        }
    }
}
