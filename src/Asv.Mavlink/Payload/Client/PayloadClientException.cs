using System;

namespace Asv.Mavlink
{
    public class PayloadClientException : Exception
    {
        public string Path { get; }
        public ErrorCode Code { get; }
        public string RemoteMessage { get; }

        public PayloadClientException(string path, ErrorCode code, string remoteMessage):base($"Remote error to execute '{path}' {code:G}:'{remoteMessage}'")
        {
            Path = path;
            Code = code;
            RemoteMessage = remoteMessage;
        }
    }
}
