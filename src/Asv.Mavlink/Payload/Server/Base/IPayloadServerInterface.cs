using System;

namespace Asv.Mavlink
{
    public interface IPayloadServerInterface:IDisposable
    {
        string Name { get; }
        void Init(IMavlinkPayloadServer server);
    }
}