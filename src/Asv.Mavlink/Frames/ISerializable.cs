namespace Asv.Mavlink
{
    public interface ISerializable
    {
        /// <summary>
        /// Serialize object to buffer
        /// </summary>
        /// <param name="buffer"></param>
        /// <param name="inx"></param>
        /// <returns>writed bytes</returns>
        int Serialize(byte[] buffer, int inx);
        /// <summary>
        /// Deserialize object from buffer
        /// </summary>
        /// <param name="buffer"></param>
        /// <param name="inx"></param>
        /// <returns></returns>
        int Deserialize(byte[] buffer, int inx);
    }
}