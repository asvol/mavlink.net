using System;

namespace Asv.Mavlink
{
    public class PayloadClientException : Exception
    {
        public string Path { get; }
        public ErrorCode ResultError { get; }

        public PayloadClientException(string path, ErrorCode resultError):base($"Remote error to execute '{path}'. Error code:'{resultError:G}'")
        {
            Path = path;
            ResultError = resultError;
        }
    }
}