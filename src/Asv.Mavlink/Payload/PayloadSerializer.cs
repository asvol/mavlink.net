using System;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Server;
using Asv.Mavlink.V2.Common;
using MessagePack;


namespace Asv.Mavlink
{
    [MessagePackObject]
    public class PayloadPacketHeader
    {
        [Key(0)]
        public ushort PacketId { get; set; }
        [Key(1)]
        public string Path { get; set; }
    }

    [MessagePackObject]
    public class PayloadError
    {
        [Key(0)]
        public string ErrorMessage { get; set; }
    }

    public delegate Task<TOut> DataDelegate<in TIn, TOut>(DeviceIdentity devId, TIn data);

    public static class PayloadSerializerV2
    {
        private static Regex NameRegex = new Regex("[A-Za-z][a-zA-Z][^#&<>\"~;$^%{}?]{1,10}$", RegexOptions.Compiled);
        public static char PathSeparator = '.';

        public static void ValidateName(string name)
        {
            if (!NameRegex.IsMatch(name))
            {
                throw new Exception($"Name validation error '{name}'");
            }
        }
        
        public static string PathJoin(string name1, string name2)
        {
            return string.Concat(name1, PathSeparator, name2);
        }
        
        public static void PathSplit(string path, out string name1, out string name2)
        {
            var paths = path.Split(PathSeparator);
            name1 = paths.FirstOrDefault();
            name2 =  string.Join(PathSeparator.ToString(), paths.Skip(1));
        }

        public const uint HeartbeatMagicDigit = 734654897;
        public const int SuccessMessageTypeId = 32770;
        public const int ErrorMessageTypeId = 32771;
        public static readonly int V2ExtensionMaxDataSize = new V2ExtensionPayload().GetPayloadMaxItemsCount();
        public static int MaxErrorMessageSize = 200;

        public static void WriteData<TData>(Stream stream, TData data)
        {
            MessagePackSerializer.Serialize(stream, data);
        }

        public static TData ReadData<TData>(Stream stream)
        {
            return MessagePackSerializer.Deserialize<TData>(stream);
        }

        public static void WriteHeader(Stream stream, PayloadPacketHeader header)
        {
            MessagePackSerializer.Serialize(stream, header);
        }
        
        public static PayloadPacketHeader ReadHeader(Stream stream)
        {
           return MessagePackSerializer.Deserialize<PayloadPacketHeader>(stream);
        }

        
    }
}
