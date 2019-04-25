using System;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public interface IMavlinkModeProtocol:IDisposable
    {
        Task SetMode(uint baseMode, uint customMode, CancellationToken cancel);
    }
}