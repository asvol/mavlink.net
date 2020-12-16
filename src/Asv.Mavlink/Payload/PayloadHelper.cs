using System;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using Asv.Mavlink.Server;
using Asv.Mavlink.V2.Common;
using Newtonsoft.Json;
using Newtonsoft.Json.Bson;
using Newtonsoft.Json.Linq;

namespace Asv.Mavlink
{
    public class ErrorCode
    {
        public ErrorType Res { get; set; }
        public string Msg { get; set; }
    }

    public enum ErrorType
    {
        ArgsError,
        InternalError
    }

    public class PayloadVoid
    {
        public static PayloadVoid Default = new PayloadVoid();
    }

    public delegate Task<TOut> DataDelegate<in TIn, TOut>(DeviceIdentity devId, TIn data);

    public class PayloadPacketHeader:ISerializablePayloadData
    {
        public string Path { get; set; }
        public byte PacketId { get; set; }
        public void Serialize(BsonDataWriter wrt)
        {
            wrt.WriteStartArray();
            wrt.WriteValue(Path);
            wrt.WriteValue(PacketId);
            wrt.WriteEndArray();
        }

        public void Deserialize(BsonDataReader rdr)
        {
            var arr = JArray.ReadFrom(rdr);
            Path = (string) ((JValue) arr.First).Value;
            PacketId = (byte)((JValue)arr.Last).Value;
        }
    }

    public interface ISerializablePayloadData
    {
        void Serialize(BsonDataWriter wrt);
        void Deserialize(BsonDataReader rdr);
    }

    public static class PayloadHelper
    {
        private static Regex NameRegex = new Regex("[A-Za-z][a-zA-Z][^#&<>\"~;$^%{}?]{1,10}$", RegexOptions.Compiled);
        public static char PathSeparator = '.';

        public const uint HeartbeatMagicDigit = 734654897;
        public const byte DefaultNetworkId = 0;
        /// <summary>
        /// Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
        /// </summary>
        public const ushort DefaultSuccessMessageType = 32770;
        public const ushort DefaultErrorMessageType = 32771;
        public static Encoding DefaultEncoding => Encoding.UTF8;
        private static readonly JsonSerializer Serializer = JsonSerializer.CreateDefault();

        public static void WriteData<T>(Stream strm, T data)
        {
            using (var writer = new BsonDataWriter(new BinaryWriter(strm, DefaultEncoding, true)))
            {
                if (data is ISerializablePayloadData)
                {
                    ((ISerializablePayloadData)data).Serialize(writer);
                }
                else
                {
                    Serializer.Serialize(writer, data);
                }
            }
            // var serializer = MessagePackSerializer.Get<T>();
            // serializer.Pack(strm, data);
        }

        public static T ReadData<T>(Stream strm) where T : new()
        {
            var data = new T();
            using (var rdr = new BsonDataReader(new BinaryReader(strm, DefaultEncoding, true)))
            {
                if (data is ISerializablePayloadData)
                {
                    ((ISerializablePayloadData)data).Deserialize(rdr);
                }
                else
                {
                    Serializer.Populate(rdr, data);
                }
                return data;
            }

            // var serializer = MessagePackSerializer.Get<T>();
            // return serializer.Unpack(strm);
        }

        

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

        public static readonly int V2ExtensionMaxDataSize = new V2ExtensionPayload().GetPayloadMaxItemsCount();
        

        public static string PrintData(MemoryStream strm)
        {
            using (var rdr = new BsonDataReader(new BinaryReader(strm, DefaultEncoding, true)))
            {
                return Serializer.Deserialize<JToken>(rdr)?.ToString(Formatting.Indented);
            }
            // var obj = MessagePackSerializer.UnpackMessagePackObject(ms);
            // return obj.ToString();
        }
    }

}
