using System;

namespace Asv.Mavlink
{
    public class DeserizliaePackageException:MavlinkException
    {
        public int MessageId { get; }

        public DeserizliaePackageException(int messageId,string message, Exception innerException):base(message,innerException)
        {
            MessageId = messageId;
        }

        public DeserizliaePackageException(int messageId, string message):base(message)
        {
            MessageId = messageId;
        }
    }
}