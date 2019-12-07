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

    public class NameWrapper
    {
        public string N { get; set; }
    }

    public class Void
    {

    }

    public delegate Task<TOut> DataDelegate<in TIn, TOut>(DeviceIdentity devId, TIn data);

    public static class WellKnownDiag
    {
        #region DIAG

        public const string Diag = "DIAG";
        public const string DiagStringsValueName = "SVAL";
        public const string DiagDigitValueName = "DVAL";
        public const string DiagSettingsValueName = "CONF";
        public const string DiagSettingsSetMethodName = "SET";

        #endregion
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
        public const ushort DefaultSuccessMessageType = 32768;
        public const ushort DefaultErrorMessageType = 32769;
        public static Encoding DefaultEncoding => Encoding.UTF8;


        public static void WriteData<T>(Stream strm, T data)
        {
            using (var writer = new BsonDataWriter(new BinaryWriter(strm, DefaultEncoding, true)))
            {
                var serializer = new JsonSerializer();
                serializer.Serialize(writer, data);
            }
        }

        public static T ReadData<T>(Stream strm)
        {
            using (var rdr = new BsonDataReader(new BinaryReader(strm, DefaultEncoding, true)))
            {
                var serializer = new JsonSerializer();
                return serializer.Deserialize<T>(rdr);
            }
        }

        public static void ReadHeader(Stream strm, out string path)
        {
            using (var rdr = new BsonDataReader(new BinaryReader(strm, DefaultEncoding, true)))
            {
                var serializer = new JsonSerializer();
                var res = serializer.Deserialize<NameWrapper>(rdr);
                path = res.N;
            }
        }

        public static void WriteHeader(Stream strm, string path)
        {
            using (var writer = new BsonDataWriter(new BinaryWriter(strm, DefaultEncoding, true)))
            {
                var serializer = new JsonSerializer();
                serializer.Serialize(writer, new NameWrapper { N = path });
            }
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


        public static PacketInfo GetInfo(BinaryReader rdr)
        {
            var info = new PacketInfo
            {
                PacketId = rdr.ReadUInt16(),
                PacketIndex = rdr.ReadUInt16(),
                PacketCount = rdr.ReadUInt16(),
                DataCount = rdr.ReadByte(),
            };

            info.Data = new byte[info.DataCount];
            rdr.Read(info.Data, 0, info.Data.Length);
            return info;
        }

        public static void SetInfo(BinaryWriter wrt, PacketInfo info)
        {
            wrt.Write(info.PacketId);
            wrt.Write(info.PacketIndex);
            wrt.Write(info.PacketCount);
            wrt.Write(info.DataCount);
            wrt.Write(info.Data, 0, info.DataCount);
        }
    }

    public class PacketInfo
    {
        public static int PacketInfoSize = sizeof(ushort) * 4;

        public ushort PacketId { get; set; }
        public ushort PacketIndex { get; set; }
        public ushort PacketCount { get; set; }
        public byte DataCount { get; set; }
        public byte[] Data { get; set; }
    }
}