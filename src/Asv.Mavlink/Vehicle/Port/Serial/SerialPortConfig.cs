using System;
using System.IO.Ports;
using System.Web;

namespace Asv.Mavlink
{
    public class SerialPortConfig
    {

        public int DataBits { get; set; } = 8;
        public int BoundRate { get; set; } = 115200;
        public Parity Parity { get; set; } = Parity.None;
        public StopBits StopBits { get; set; } = StopBits.One;
        public string PortName { get; set; }

        public static bool TryParseFromUri(Uri uri, out SerialPortConfig opt)
        {
            if (!"serial".Equals(uri.Scheme, StringComparison.InvariantCultureIgnoreCase))
            {
                opt = null;
                return false;
            }

            var coll = HttpUtility.ParseQueryString(uri.Query);
            opt = new SerialPortConfig
            {
                PortName = uri.LocalPath,
                BoundRate = int.Parse(coll["boundRate"]),
                Parity = (Parity)Enum.Parse(typeof(Parity), coll["parity"] ?? Parity.None.ToString()),
                DataBits = int.Parse(coll["dataBits"] ?? "8"),
                StopBits = (StopBits)Enum.Parse(typeof(StopBits), coll["stopBits"] ?? StopBits.One.ToString()),
            };
            return true;
        }

    }
}