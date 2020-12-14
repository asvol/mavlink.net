using System;
using System.Runtime.Serialization;

namespace Asv.Mavlink
{
    [Serializable]
    public class PayloadOversizeException : Exception
    {
        public PayloadOversizeException()
        {
        }

        public PayloadOversizeException(string message) : base(message)
        {
        }

        public PayloadOversizeException(string message, Exception inner) : base(message, inner)
        {
        }

        protected PayloadOversizeException(
            SerializationInfo info,
            StreamingContext context) : base(info, context)
        {
        }
    }
}
