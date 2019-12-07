using System;

namespace Asv.Mavlink
{
    public interface IMavlinkPayloadClientInterface:IDisposable
    {
        string Name { get; }
        void Init(IMavlinkPayloadClient client);
    }
}