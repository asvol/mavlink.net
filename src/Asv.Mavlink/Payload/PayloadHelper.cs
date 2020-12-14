using System;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using Asv.Mavlink.Server;
using Asv.Mavlink.V2.Common;
using MsgPack.Serialization;

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

    public class Void
    {
        public bool N { get; set; }
        public static Void Default = new Void();
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

    public class PayloadPacketHeader
    {
        public string Path { get; set; }
        public ushort PacketId { get; set; }
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


        public static void WriteData<T>(Stream strm, T data)
        {
            var serializer = MessagePackSerializer.Get<T>();
            serializer.Pack(strm, data);
        }

        public static T ReadData<T>(Stream strm)
        {
            var serializer = MessagePackSerializer.Get<T>();
            return serializer.Unpack(strm);
        }

        public static PayloadPacketHeader ReadHeader(Stream strm)
        {
            var serializer = MessagePackSerializer.Get<PayloadPacketHeader>();
            return serializer.Unpack(strm);
        }

        public static void WriteHeader(Stream strm, PayloadPacketHeader header)
        {
            var serializer = MessagePackSerializer.Get<PayloadPacketHeader>();
            serializer.Pack(strm, header);
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

        public static string PrintData(MemoryStream ms)
        {
            var obj = MessagePackSerializer.UnpackMessagePackObject(ms);
            return obj.ToString();
        }
    }

}
