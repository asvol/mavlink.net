using System;

namespace Asv.Mavlink
{
    public class PayloadClientException : Exception
    {
        public string Path { get; }
        public string ErrorMessage { get; }

        public PayloadClientException(string path, string errorMessage):base($"Remote error to execute '{path}'. Error code:'{errorMessage:G}'")
        {
            Path = path;
            ErrorMessage = errorMessage;
        }
    }
}
