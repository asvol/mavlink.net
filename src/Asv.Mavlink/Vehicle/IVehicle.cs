using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Diagnostics;
using System.Globalization;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Client;
using Asv.Mavlink.V2.Common;
using Asv.Mavlink.V2.Minimal;
using NLog;

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

    public class RadioStatusInfo
    {
        public RadioStatusInfo()
        {
        }

        public RadioStatusInfo(byte rssi, byte noise)
        {
            RSSI = rssi;
            if (rssi == 255)
            {
                RSSIPersent = 0;
                RSSIdBm = Double.NaN;
                RSSImW = Double.NaN;
            }
            else
            {
                RSSIPersent = 100.0 / 254.0 * rssi;
                RSSIdBm = rssi / 1.9 - 127;
                RSSImW = Math.Pow(10, (RSSIdBm - 30) / 10) * 1000;
            }

            Noise = noise;
            if (noise == 255)
            {
                NoisePercent = 0;
                NoisedBm = Double.NaN;
                NoisemW = Double.NaN;
            }
            else
            {
                NoisePercent = 100.0 / 254.0 * noise;
                NoisedBm = noise / 1.9 - 127;
                NoisemW = Math.Pow(10, (NoisedBm - 30) / 10) * 1000;
            }
        }


        public double RSSI { get; set; }
        public double RSSIPersent { get; set; }
        public double RSSIdBm { get; set; }
        public double RSSImW { get; set; }

        public double Noise { get; set; }
        public double NoisedBm { get; set; }
        public double NoisemW { get; set; }
        public double NoisePercent { get; set; }
    }

    public class RadioLinkStatus
    {
        public RadioLinkStatus()
        {
        }
        public RadioLinkStatus(RadioStatusPayload value)
        {
            Local = new RadioStatusInfo(value.Rssi,value.Noise);
            Remote = new RadioStatusInfo(value.Remrssi,value.Remnoise);
            CorrectedPackets = value.Fixed;
            FreeTxBufferSpace = value.Txbuf;
            ReceiveErrors = value.Rxerrors;
        }

        public ushort ReceiveErrors { get; set; }
        public byte FreeTxBufferSpace { get; set; }

        public ushort CorrectedPackets { get; set; }

        public RadioStatusInfo Local { get; set; }
        public RadioStatusInfo Remote { get; set; }
        
    }

    

    public interface IVehicle
    {
        MavlinkClientIdentity Identity { get; }

        IMavlinkClient Mavlink { get; }

        IRxValue<VehicleInitState> InitState { get; }

        void StartListen();

        IRxValue<LinkState> Link { get; }
        IRxValue<int> PacketRateHz { get; }
        IRxValue<double> LinkQuality { get; }
        IRxValue<RadioLinkStatus> RadioStatus { get; }

        IRxValue<VehicleClass> Class { get; }

        /// <summary>
        /// The filtered global position (e.g. fused GPS and accelerometers). 
        /// </summary>
        IRxValue<GeoPoint> GlobalPosition { get; }

        IRxValue<GpsInfo> GpsInfo { get; }
        IRxValue<double> GpsGroundVelocity { get; }

        IRxValue<GpsInfo> Gps2Info { get; }
        IRxValue<double> Gps2GroundVelocity { get; }

        IRxValue<double> AltitudeAboveHome { get; }

        IRxValue<GeoPoint?> Home { get; }
        IRxValue<double?> HomeDistance { get; }
        Task RequestHome(CancellationToken cancel);

        IRxValue<bool> IsArmed { get; }
        IRxValue<TimeSpan> ArmedTime { get; }

        Task ArmDisarm(bool isArming, CancellationToken cancel, bool force = false);

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
        
        IEnumerable<VehicleCustomMode> AvailableModes { get; }
        IRxValue<VehicleMode> Mode { get; }
        Task SetMode(VehicleMode mode, CancellationToken cancel);

        IMavlinkParameterClient Params { get; }

        IEnumerable<VehicleParamDescription> GetParamDescription();

        Task TakeOff(double altitude, CancellationToken cancel);

        IRxValue<GeoPoint?> GoToTarget { get; }
        Task GoToGlob(GeoPoint location, CancellationToken cancel, double? yawDeg = null);
        Task GoToGlobAndWait(GeoPoint location, IProgress<double> progress, double precisionMet, CancellationToken cancel);
        Task GoToGlobAndWaitWithoutAltitude(GeoPoint location, IProgress<double> progress, double precisionMet, CancellationToken cancel);

        Task FlyByLineGlob(GeoPoint start, GeoPoint stop, double precisionMet, CancellationToken cancel, Action firstPointComplete = null);

        Task DoRtl(CancellationToken cancel);

        #region Region of Interest

        Task SetRoi(GeoPoint location, CancellationToken cancel);
        IRxValue<GeoPoint?> Roi { get; }
        Task ClearRoi(CancellationToken cancel);

        #endregion

        /// <summary>
        /// Request the reboot or shutdown of system components.
        /// </summary>
        /// <param name="ardupilot"></param>
        /// <param name="companion"></param>
        /// <param name="cancel"></param>
        /// <returns></returns>
        Task PreflightRebootShutdown(AutopilotRebootShutdown ardupilot, CompanionRebootShutdown companion, CancellationToken cancel);

        #region Flight time statistic

        /// <summary>
        /// Flight time recorder statistic
        /// </summary>
        /// <param name="cancel"></param>
        /// <returns></returns>
        Task<FlightTimeStatistic> GetFlightTimeStatistic(CancellationToken cancel = default);
        /// <summary>
        /// Reset flight time stat
        /// </summary>
        /// <param name="cancel"></param>
        /// <returns></returns>
        Task ResetFlightStatistic(CancellationToken cancel = default);

        #endregion

        #region

        IEnumerable<FailSafeInfo> AvailableFailSafe { get; }
        Task<FailSafeState[]> ReadFailSafe(CancellationToken cancel = default);
        Task WriteFailSafe(IReadOnlyDictionary<string,bool> values,CancellationToken cancel = default);

        #endregion

        #region Missions

        

        #endregion
    }

    
    public class FailSafeInfo
    {
        public object Tag { get; set; }
        public string Name { get; set; }
        public string DisplayName { get; set; }
        public string Description { get; set; }
    }

    public class FailSafeState
    {
        public FailSafeInfo Info { get; set; }
        public bool Value { get; set; }
    }

    public class FlightTimeStatistic
    {
        /// <summary>
        /// Holds the number of times the board has been booted
        /// </summary>
        public long BootCount { get; set; }
        /// <summary>
        /// Holds the total number of seconds that the board/vehicle has been flying (including all previous flights)
        /// </summary>
        public TimeSpan FlightTime { get; set; }
        /// <summary>
        /// Holds the total number of seconds that the board has been powered up (including all previous flights)
        /// </summary>
        public TimeSpan RunTime { get; set; }
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
                    return null;
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
