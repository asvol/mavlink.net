namespace Asv.Mavlink
{
    public class Signature : ISignature
    {
        private bool _isPresent;

        public bool IsPresent
        {
            get { return _isPresent; }
            set
            {
                _isPresent = value;
                ByteSize = value ? PacketV2Helper.SignatureByteSize : 0;
            }
        }

        public int ByteSize { get; private set; }
        public byte LinkId { get; set; }
        public ulong Timestamp { get; set; }
        public ulong Sign { get; set; }

        public int Deserialize(byte[] buffer, int inx)
        {
            IsPresent = true;
            LinkId = buffer[inx];
            Timestamp = buffer[inx + 1];
            Timestamp |= ((ulong)buffer[inx + 2]) << 8;
            Timestamp |= ((ulong)buffer[inx + 3]) << 16;
            Timestamp |= ((ulong)buffer[inx + 4]) << 24;
            Timestamp |= ((ulong)buffer[inx + 5]) << 32;
            Timestamp |= ((ulong)buffer[inx + 6]) << 40;

            Sign = buffer[inx + 7];
            Sign |= ((ulong)buffer[inx + 8]) << 8;
            Sign |= ((ulong)buffer[inx + 9]) << 16;
            Sign |= ((ulong)buffer[inx + 10]) << 24;
            Sign |= ((ulong)buffer[inx + 11]) << 32;
            Sign |= ((ulong)buffer[inx + 12]) << 40;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int inx)
        {
            if (!IsPresent) return 0;
            buffer[inx] = LinkId;
            buffer[inx + 1] = (byte) (Timestamp & 0xFF);
            buffer[inx + 2] = (byte)(Timestamp >> 8 & 0xFF);
            buffer[inx + 3] = (byte)(Timestamp >> 16 & 0xFF);
            buffer[inx + 4] = (byte)(Timestamp >> 24 & 0xFF);
            buffer[inx + 5] = (byte)(Timestamp >> 32 & 0xFF);
            buffer[inx + 6] = (byte)(Timestamp >> 40 & 0xFF);


            buffer[inx + 7] = (byte)(Sign & 0xFF);
            buffer[inx + 8] = (byte)(Sign >> 8 & 0xFF);
            buffer[inx + 9] = (byte)(Sign >> 16 & 0xFF);
            buffer[inx + 10] = (byte)(Sign >> 24 & 0xFF);
            buffer[inx + 11] = (byte)(Sign >> 32 & 0xFF);
            buffer[inx + 12] = (byte)(Sign >> 40 & 0xFF);
            return PacketV2Helper.SignatureByteSize;
        }
    }
}