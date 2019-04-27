using System;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink
{
    


    public interface IVehicle:IDisposable
    {
        Task Init(CancellationToken cancel);

        IRxValue<LinkState> Link { get; }
        IRxValue<int> PacketRateHz { get; }
        IRxValue<GeoPoint?> RelGps { get; }
        IRxValue<GeoPoint> GlobGps { get; }
        IRxValue<GeoPoint?> Home { get; }
        
        IRxValue<bool> IsArmed { get; }
        Task ArmDisarm(bool isArming, CancellationToken cancel);

        IRxValue<double?> BatteryCharge { get; }

        IRxValue<double> Pitch { get; }
        IRxValue<double> Roll { get; }
        IRxValue<double> Yaw { get; }
        IRxValue<double> PitchSpeed { get; }
        IRxValue<double> RollSpeed { get; }
        IRxValue<double> YawSpeed { get; }

        Task TakeOff(double altitude, CancellationToken cancel);
        Task GoTo(GeoPoint location, CancellationToken cancel);


    }

    
}
