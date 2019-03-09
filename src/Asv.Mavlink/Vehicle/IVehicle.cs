using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;
using NLog;

namespace Asv.Mavlink
{
    public interface IVehicle:IDisposable
    {
        IRxValue<Exception> PortError {get; }
        IRxValue<DeserizliaePackageException> OnPacketErrors { get; }
        IRawTelemetry Rtt { get; }
        IVehicleParameterProtocol Params { get; }
        IVehicleCommandProtocol Commands { get; }
        IVehicleMissionProtocol Mission { get; }
        IOffboardMode Offboard { get; }
       
    }
}
