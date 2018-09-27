namespace Asv.Mavlink.Decoder
{
    public class PacketV2Encoder : PacketEncoder<IPacketV2<IPayload>>
    {
        public PacketV2Encoder() : base(PacketV2Helper.PacketV2MaxSize)
        {
        }
    }
}