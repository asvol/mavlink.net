namespace Asv.Mavlink
{
    public interface IPayload
    {
        /// <summary>
        /// Maximum size of payload
        /// </summary>
        byte GetMaxByteSize();
        /// <summary>
        /// Serialize payload to buffer
        /// </summary>
        /// <param name="buffer"></param>
        /// <param name="offset"></param>
        /// <returns>writed bytes</returns>
        int Serialize(byte[] buffer, int offset);

        /// <summary>
        /// Deserialize object from buffer
        /// </summary>
        /// <param name="buffer"></param>
        /// <param name="offset"></param>
        /// <param name="payloadSize"></param>
        /// <returns></returns>
        void Deserialize(byte[] buffer, int offset, int payloadSize);
    }
}