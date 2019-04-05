using System;

namespace Asv.Mavlink
{
    public interface IVehicle:IDisposable
    {
        IRawTelemetry Rtt { get; }
        IVehicleParameterProtocol Params { get; }
        IVehicleCommandProtocol Commands { get; }
        IVehicleMissionProtocol Mission { get; }
        IOffboardMode Offboard { get; }
       
    }
}
