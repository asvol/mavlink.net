namespace Asv.Mavlink
{
    public interface IPacket<out TPayload>: ISerializable
        where TPayload:IPayload
    {
        /// <summary>
        /// Protocol magic marker
        /// </summary>
        byte Magic { get; }
        /// <summary>
        /// Sequence of packet
        /// </summary>
        byte Sequence { get; set; }
        /// <summary>
        /// ID of message sender system/aircraft
        /// </summary>
        byte SystemId { get; set; }
        /// <summary>
        /// ID of the message sender component
        /// </summary>
        byte ComponenId { get; set; }
        /// <summary>
        /// Message Id
        /// </summary>
        int MessageId { get; }
        /// <summary>
        /// Message payload
        /// </summary>
        TPayload Payload { get; }
        /// <summary>
        /// Size in bytes. Used when packet where deserialized
        /// </summary>
        int Size { get; set; }
    }
}
