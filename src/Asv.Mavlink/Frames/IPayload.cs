namespace Asv.Mavlink
{
    public interface IPayload: ISerializable
    {
        /// <summary>
        /// ID of link on which packet is sent. Normally this is the same as the channel.
        /// </summary>
        /// 
        byte ByteSize { get; }
    }
}