using Newtonsoft.Json.Bson;

namespace Asv.Mavlink
{
    public static class WellKnownDiag
    {
        #region DIAG

        public const string Diag = "DIAG";
        public const string DiagStringsValueName = "SVAL";
        public const string DiagDigitValueName = "DVAL";
        public const string DiagSettingsValueName = "CONF";
        public const string DiagSettingsSetMethodName = "SET";
        public const string DiagGetAll = "GET";

        #endregion
    }

    public class KeyValueData : ISerializablePayloadData
    {
        public string Key { get; set; }
        public string Value { get; set; }

        public void Serialize(BsonDataWriter wrt)
        {
            wrt.WriteStartArray();
            wrt.WriteValue(Key);
            wrt.WriteValue(Value);
            wrt.WriteEndArray();
        }

        public void Deserialize(BsonDataReader rdr)
        {
            rdr.Read();

            rdr.Read();
            Key = rdr.ReadAsString();

            rdr.Read();
            Value = rdr.ReadAsString();

            rdr.Read();
        }
    }
}
