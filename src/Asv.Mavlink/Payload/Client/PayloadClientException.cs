using System;

namespace Asv.Mavlink
{
    public class PayloadClientException : Exception
    {
        public string Path { get; }
        public ErrorCode RemoteError { get; }

        public PayloadClientException(string path, ErrorCode remoteError):base($"Remote error to execute '{path}' {remoteError.Res:G}:'{remoteError.Msg}'")
        {
            Path = path;
            RemoteError = remoteError;
        }
    }
}
