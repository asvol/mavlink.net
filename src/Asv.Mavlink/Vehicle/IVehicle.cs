using System;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public enum VehicleInitState
    {
        WaitConnection,
        Failed,
        InProgress,
        Complete
    }


    public interface IVehicle:IDisposable
    {
        IRxValue<VehicleInitState> InitState { get; }

        void StartListen();

        IRxValue<LinkState> Link { get; }
        IRxValue<int> PacketRateHz { get; }

        IRxValue<GeoPoint> GpsLocation { get; }
        IRxValue<GpsInfo> GpsInfo { get; }

        IRxValue<double> AltitudeAboveHome { get; }
        IRxValue<GeoPoint> Home { get; }
        
        IRxValue<bool> IsArmed { get; }
        Task ArmDisarm(bool isArming, CancellationToken cancel);

        IRxValue<double?> CurrentBattery { get; }
        IRxValue<double> VoltageBattery { get; }
        IRxValue<double?> BatteryCharge { get; }

        IRxValue<double> Pitch { get; }
        IRxValue<double> Roll { get; }
        IRxValue<double> Yaw { get; }
        IRxValue<double> PitchSpeed { get; }
        IRxValue<double> RollSpeed { get; }
        IRxValue<double> YawSpeed { get; }

        IRxValue<VehicleStatusMessage> TextStatus { get; }
        IRxValue<double> CpuLoad { get; }
        IRxValue<double> DropRateCommunication { get; }

        IRxValue<double> GroundVelocity { get; }


        IRxValue<VehicleMode> Mode { get; }

        Task TakeOff(double altitude, CancellationToken cancel);

        Task GoToRel(GeoPoint location, CancellationToken cancel, double? yawDeg = null);
        Task GoToGlob(GeoPoint location, CancellationToken cancel, double? yawDeg = null);


        Task DoRtl(CancellationToken cancel);
    }

    public class VehicleStatusMessage
    {
        public string Text { get; set; }
        public MavSeverity Type { get; set; }
    }
}
