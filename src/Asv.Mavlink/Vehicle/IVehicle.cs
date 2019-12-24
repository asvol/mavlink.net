using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Client;
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

    public enum VehicleClass
    {
        Unknown,
        Plane,
        Copter,
    }

    public interface IVehicle
    {
        MavlinkClientIdentity Identity { get; }

        IMavlinkClient Mavlink { get; }

        IRxValue<VehicleInitState> InitState { get; }

        void StartListen();

        IRxValue<LinkState> Link { get; }
        IRxValue<int> PacketRateHz { get; }

        IRxValue<VehicleClass> Class { get; }

        IRxValue<GeoPoint> GpsLocation { get; }
        IRxValue<GpsInfo> GpsInfo { get; }

        IRxValue<double> AltitudeAboveHome { get; }

        IRxValue<GeoPoint?> Home { get; }
        IRxValue<double?> HomeDistance { get; }
        Task RequestHome(CancellationToken cancel);

        IRxValue<bool> IsArmed { get; }
        IRxValue<TimeSpan> ArmedTime { get; }
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
        
        IEnumerable<VehicleCustomMode> AvailableModes { get; }
        IRxValue<VehicleMode> Mode { get; }
        Task SetMode(VehicleMode mode, CancellationToken cancel);

        IMavlinkParameterClient Params { get; }

        Task TakeOff(double altitude, CancellationToken cancel);

        IRxValue<GeoPoint?> GoToTarget { get; }
        Task GoToGlob(GeoPoint location, CancellationToken cancel, double? yawDeg = null);
        Task GoToGlobAndWait(GeoPoint location, IProgress<double> progress, double precisionMet, CancellationToken cancel);
        Task GoToGlobAndWaitWithoutAltitude(GeoPoint location, IProgress<double> progress, double precisionMet, CancellationToken cancel);

        Task FlyByLineGlob(GeoPoint start, GeoPoint stop, double precisionMet, CancellationToken cancel, Action firstPointComplete = null);

        Task DoRtl(CancellationToken cancel);

        Task SetRoi(GeoPoint location, CancellationToken cancel);
        IRxValue<GeoPoint?> Roi { get; }
        
        Task ClearRoi(CancellationToken cancel);
        /// <summary>
        /// Request the reboot or shutdown of system components.
        /// </summary>
        /// <param name="ardupilot"></param>
        /// <param name="companion"></param>
        /// <param name="cancel"></param>
        /// <returns></returns>
        Task PreflightRebootShutdown(AutopilotRebootShutdown ardupilot, CompanionRebootShutdown companion, CancellationToken cancel);
    }

    public class VehicleStatusMessage
    {
        public string Text { get; set; }
        public MavSeverity Type { get; set; }
    }


    public static class VehicleFactory
    {
        public static IVehicle CreateVehicle(MavlinkClient client, IMavlinkDeviceInfo info)
        {
            switch (info.Autopilot)
            {
               
                case MavAutopilot.MavAutopilotArdupilotmega:
                    return SelectArdupilotmega(client,info);
                case MavAutopilot.MavAutopilotGeneric:
                case MavAutopilot.MavAutopilotReserved:
                case MavAutopilot.MavAutopilotSlugs:
                case MavAutopilot.MavAutopilotOpenpilot:
                case MavAutopilot.MavAutopilotGenericWaypointsOnly:
                case MavAutopilot.MavAutopilotGenericWaypointsAndSimpleNavigationOnly:
                case MavAutopilot.MavAutopilotGenericMissionFull:
                case MavAutopilot.MavAutopilotInvalid:
                case MavAutopilot.MavAutopilotPpz:
                case MavAutopilot.MavAutopilotUdb:
                case MavAutopilot.MavAutopilotFp:
                case MavAutopilot.MavAutopilotPx4:
                case MavAutopilot.MavAutopilotSmaccmpilot:
                case MavAutopilot.MavAutopilotAutoquad:
                case MavAutopilot.MavAutopilotArmazila:
                case MavAutopilot.MavAutopilotAerob:
                case MavAutopilot.MavAutopilotAsluav:
                case MavAutopilot.MavAutopilotSmartap:
                case MavAutopilot.MavAutopilotAirrails:
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        public static VehicleArdupilot SelectArdupilotmega(MavlinkClient client, IMavlinkDeviceInfo info)
        {
            switch (info.Type)
            {
                case MavType.MavTypeFixedWing:
                    return new VehicleArdupilotPlane(client,new VehicleBaseConfig());
                case MavType.MavTypeQuadrotor:
                case MavType.MavTypeTricopter:
                case MavType.MavTypeHexarotor:
                case MavType.MavTypeOctorotor:
                case MavType.MavTypeHelicopter:
                    return new VehicleArdupilotCopter(client, new VehicleBaseConfig());
                case MavType.MavTypeSurfaceBoat:
                case MavType.MavTypeGroundRover:
                case MavType.MavTypeAntennaTracker:
                case MavType.MavTypeSubmarine:
                default:
                    return null;
            }
        }
    }
}
