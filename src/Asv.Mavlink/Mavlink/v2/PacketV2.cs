namespace Asv.Mavlink
{
    public abstract class PacketV2<TPayload> : IPacketV2<TPayload> where TPayload : IPayload
    {
        public byte Magic => PacketV2Helper.MagicMarkerV2;
        public int ByteSize => Payload.ByteSize + Signature.ByteSize + PacketV2Helper.PacketV2FrameSize;
        public abstract byte CrcEtra { get; }
        public abstract int MessageId { get; }
        public byte IncompatFlags { get; set; }
        public byte CompatFlags { get; set; }
        public byte Sequence { get; set; }
        public byte SystemId { get; set; }
        public byte ComponenId { get; set; }
        public abstract TPayload Payload { get; }
        public ISignature Signature { get; } = new Signature();

        public int Serialize(byte[] buffer, int inx)
        {
            PacketV2Helper.SetStx(buffer, inx);
            PacketV2Helper.SetPayloadSize(buffer,inx,Payload.ByteSize);
            PacketV2Helper.SetIncompatFlags(buffer, inx, IncompatFlags);
            PacketV2Helper.SetCompatFlags(buffer, inx, CompatFlags);
            PacketV2Helper.SetSequence(buffer, inx, Sequence);
            PacketV2Helper.SetSystemId(buffer, inx, SystemId);
            PacketV2Helper.SetComponenId(buffer, inx, ComponenId);
            PacketV2Helper.SetMessageId(buffer, inx, MessageId);
            Payload.Serialize(buffer, inx + PacketV2Helper.PaylodStartIndexInFrame);
            PacketV2Helper.SetCrc(buffer, inx, CrcEtra);
            if (Signature.IsPresent)
            {
                Signature.Serialize(buffer, inx + Payload.ByteSize + PacketV2Helper.PacketV2FrameSize);
            }
            return ByteSize;
        }

        public int Deserialize(byte[] buffer, int inx)
        {
            PacketV2Helper.VerifyStx(buffer,inx);
            var payloadSize = PacketV2Helper.GetPayloadSize(buffer, inx);
            if (payloadSize != Payload.ByteSize)
                throw new MavlinkException(string.Format(RS.PacketV2_Deserialize_Packet_payload_length_error, Payload.ByteSize, payloadSize));
            IncompatFlags = PacketV2Helper.GetIncompatFlags(buffer, inx);
            CompatFlags = PacketV2Helper.GetCompatFlags(buffer, inx);
            Sequence = PacketV2Helper.GetSequence(buffer, inx);
            SystemId = PacketV2Helper.GetSystemId(buffer, inx);
            ComponenId = PacketV2Helper.GetComponenId(buffer, inx);
            var messageId = PacketV2Helper.GetMessageId(buffer, inx);
            if (messageId != MessageId)
                throw new MavlinkException(string.Format(RS.PacketV2_Deserialize_Error_message_id_type, MessageId, messageId));
            Payload.Deserialize(buffer, inx + PacketV2Helper.PaylodStartIndexInFrame);
            PacketV2Helper.VerifyCrc(buffer,inx, CrcEtra);
            if (PacketV2Helper.CheckSignaturePresent(buffer, inx))
            {
                Signature.Deserialize(buffer, PacketV2Helper.GetSignatureStartIndex(buffer, inx));
            }
            return ByteSize;
        }

        
    }
}