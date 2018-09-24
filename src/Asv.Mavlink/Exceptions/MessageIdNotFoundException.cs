namespace Asv.Mavlink
{
    public class MessageIdNotFoundException : DeserizliaePackageException
    {
        public MessageIdNotFoundException(int messageId) : base(messageId, string.Format(RS.MessageIdNotFoundException_MessageIdNotFoundException, messageId))
        {
        }
    }
}