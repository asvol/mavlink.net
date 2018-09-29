namespace Asv.Mavlink
{
    public interface ISerializable
    {
        /// <summary>
        /// Maximum size in bytes
        /// </summary>
        int MaxByteSize { get; }
        /// <summary>
        /// Serialize object to buffer
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
        /// <returns></returns>
        int Deserialize(byte[] buffer, int offset);
    }
}
