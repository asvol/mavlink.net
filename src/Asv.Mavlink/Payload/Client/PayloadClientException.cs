using System;

namespace Asv.Mavlink
{
    public class PayloadClientException : Exception
    {
        public string Path { get; }
        public ErrorCode Code { get; }
        public string Message { get; }

        public PayloadClientException(string path, ErrorCode code, string message):base($"Remote error to execute '{path}' {code:G}:'{message}'")
        {
            Path = path;
            Code = code;
            Message = message;
        }
    }
}
