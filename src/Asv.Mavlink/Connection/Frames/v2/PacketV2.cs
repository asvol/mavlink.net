using System;

namespace Asv.Mavlink
{
    public abstract class PacketV2<TPayload> : IPacketV2<TPayload> where TPayload : IPayload
    {
        public byte Magic => PacketV2Helper.MagicMarkerV2;
        public abstract byte GetCrcEtra();
        public abstract int MessageId { get; }
        public byte IncompatFlags { get; set; }
        public byte CompatFlags { get; set; }
        public byte Sequence { get; set; }
        ///
        public byte SystemId { get; set; }
        public byte ComponenId { get; set; }
        public abstract TPayload Payload { get; }
        public abstract string Name { get; }
        public ISignature Signature { get; } = new Signature();

        public int GetMaxByteSize() => Payload.GetMaxByteSize() + Signature.GetMaxByteSize() + PacketV2Helper.PacketV2FrameSize;

        public int Serialize(byte[] buffer, int offset)
        {
            PacketV2Helper.SetStx(buffer, offset);
            PacketV2Helper.SetIncompatFlags(buffer, offset, IncompatFlags);
            PacketV2Helper.SetCompatFlags(buffer, offset, CompatFlags);
            PacketV2Helper.SetSequence(buffer, offset, Sequence);
            PacketV2Helper.SetSystemId(buffer, offset, SystemId);
            PacketV2Helper.SetComponenId(buffer, offset, ComponenId);
            PacketV2Helper.SetMessageId(buffer, offset, MessageId);
            var payloadSize = Payload.Serialize(buffer, offset + PacketV2Helper.PaylodStartIndexInFrame);
            PacketV2Helper.SetPayloadSize(buffer, offset, (byte) payloadSize);
            PacketV2Helper.SetCrc(buffer, offset, GetCrcEtra());
            if (Signature.IsPresent)
            {
                Signature.Serialize(buffer, offset + payloadSize + PacketV2Helper.PacketV2FrameSize);
            }
            return payloadSize + Signature.ByteSize + PacketV2Helper.PacketV2FrameSize;
        }

        public int Deserialize(byte[] buffer, int offset)
        {
            PacketV2Helper.VerifyStx(buffer,offset);
            var payloadSize = PacketV2Helper.GetPayloadSize(buffer, offset);
            IncompatFlags = PacketV2Helper.GetIncompatFlags(buffer, offset);
            CompatFlags = PacketV2Helper.GetCompatFlags(buffer, offset);
            Sequence = PacketV2Helper.GetSequence(buffer, offset);
            SystemId = PacketV2Helper.GetSystemId(buffer, offset);
            ComponenId = PacketV2Helper.GetComponenId(buffer, offset);
            var messageId = PacketV2Helper.GetMessageId(buffer, offset);
            if (messageId != MessageId)
                throw new MavlinkException(string.Format(RS.PacketV2_Deserialize_Error_message_id_type, MessageId, messageId));
            Payload.Deserialize(buffer, offset + PacketV2Helper.PaylodStartIndexInFrame, payloadSize);
            PacketV2Helper.VerifyCrc(buffer,offset, GetCrcEtra());
            if (PacketV2Helper.CheckSignaturePresent(buffer, offset))
            {
                Signature.Deserialize(buffer, PacketV2Helper.GetSignatureStartIndex(buffer, offset));
            }
            return payloadSize + Signature.ByteSize + PacketV2Helper.PacketV2FrameSize;
        }

        public override string ToString()
        {
            return $"{Name.PadRight(30)}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }

    }
}