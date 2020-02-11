using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Client;
using Asv.Mavlink.V2.Common;
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

    public static class VehicleHalper
    {
        /// <summary>
        /// Число попыток вызвать метод GoToGlob()
        /// </summary>
        private const int DefaultAttemptsCount = 10;
        /// <summary>
        /// Расстоение в метрах, которое будет являться признаком движения судна
        /// </summary>
        private const int FlightSignDistance = 5;
        /// <summary>
        /// Частота проверки начала движения судна
        /// </summary>
        private static readonly TimeSpan FlightCheckFrequency = TimeSpan.FromSeconds(3);


        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        private static async Task GoToGlobAndWaitWithConfirmBase(this IVehicle vehicle, GeoPoint location,
            IProgress<double> progress, double precisionMet, CancellationToken cancel, bool isWithoutAltitude = false,
            int attemptsCount = DefaultAttemptsCount, Action<string> logger = null)
        {
            logger = logger ?? (_ => { });
            await vehicle.GoToGlob(location, cancel);
            progress = progress ?? new Progress<double>();
            var startLocation = vehicle.GpsLocation.Value;
            var startDistance = isWithoutAltitude
                ? Math.Abs(GeoMath.Distance(location.SetAltitude(0), startLocation.SetAltitude(0)))
                : Math.Abs(GeoMath.Distance(location, startLocation));

            Logger.Info("GoToGlobAndWaitWithConfirm {0} with precision {2:F1} m. Distance to target {3:F1}", location, precisionMet, Math.Abs(GeoMath.Distance(location, startLocation)));
            logger($"GoToGlobAndWaitWithConfirm {location} with precision {precisionMet:F1} m. Distance to target {Math.Abs(GeoMath.Distance(location, startLocation)):F1}");

            progress.Report(0);

            if (startDistance <= precisionMet)
            {
                Logger.Debug("Already in target, nothing to do", startLocation);
                logger($"Already in target, nothing to do {startLocation}");
                progress.Report(1);
                return;
            }

            var sw = new Stopwatch();
            sw.Start();

            double dist = 0;
            var isFlying = false;
            for (var i = 0; i < attemptsCount; i++)
            {
                if (cancel.IsCancellationRequested) break;
                Logger.Debug($"Send command GoTo to vehicle. Attempt number {i + 1}", startLocation);
                logger($"Send command GoTo to vehicle . Attempt number {i + 1}. {startLocation}");
                await vehicle.GoToGlob(location, cancel).ConfigureAwait(false);
                await Task.Delay(FlightCheckFrequency, cancel).ConfigureAwait(false);
                var loc = vehicle.GpsLocation.Value;
                dist = isWithoutAltitude
                    ? Math.Abs(GeoMath.Distance(location.SetAltitude(0), loc.SetAltitude(0)))
                    : Math.Abs(GeoMath.Distance(location, loc));

                if (startDistance - dist > FlightSignDistance || dist <= precisionMet)
                {
                    isFlying = true;
                    break;
                }
            }

            if (!isFlying)
            {
                Logger.Info($"GoToGlobAndWaitWithConfirm. Command {nameof(vehicle.GoToGlob)} did not reach to UAV");
                logger($"GoToGlobAndWaitWithConfirm. Command {nameof(vehicle.GoToGlob)} did not reach to UAV");
                throw new Exception($"GoToGlobAndWaitWithConfirm. Command {nameof(vehicle.GoToGlob)} did not reach to UAV");
            }

            while (!cancel.IsCancellationRequested)
            {
                var loc = vehicle.GpsLocation.Value;
                dist = isWithoutAltitude
                    ? Math.Abs(GeoMath.Distance(location.SetAltitude(0), loc.SetAltitude(0)))
                    : Math.Abs(GeoMath.Distance(location, loc));

                var prog = 1 - dist / startDistance;
                Logger.Trace("Distance to target {0:F1}, location: {1}, progress {2:P2}", dist, loc, prog);
                logger($"Distance to target {dist:F1}, location: {loc}, progress {prog:P2}");
                progress.Report(prog);
                if (dist <= precisionMet) break;
                await Task.Delay(TimeSpan.FromSeconds(1), cancel).ConfigureAwait(false);
            }
            sw.Stop();
            Logger.Info($"Complete {sw.Elapsed:hh\\:mm\\:ss} location error {dist:F1} m");
            logger($"Complete {sw.Elapsed:hh\\:mm\\:ss} location error {dist:F1} m");
            progress.Report(1);
        }

        public static Task GoToGlobAndWaitWithConfirm(this IVehicle vehicle, GeoPoint location, IProgress<double> progress, double precisionMet, CancellationToken cancel, int attemptsCount = DefaultAttemptsCount, Action<string> logger = null)
        {
            return GoToGlobAndWaitWithConfirmBase(vehicle, location, progress, precisionMet, cancel, false,
                attemptsCount, logger);
        }

        public static Task GoToGlobAndWaitWithoutAltitudeWithConfirm(this IVehicle vehicle, GeoPoint location, IProgress<double> progress, double precisionMet, CancellationToken cancel, int attemptsCount = DefaultAttemptsCount, Action<string> logger = null)
        {
            return GoToGlobAndWaitWithConfirmBase(vehicle, location, progress, precisionMet, cancel, true,
                attemptsCount, logger);
        }
    }

    public interface IVehicle
    {
        MavlinkClientIdentity Identity { get; }

        IMavlinkClient Mavlink { get; }

        IRxValue<VehicleInitState> InitState { get; }

        void StartListen();

        IRxValue<LinkState> Link { get; }
        IRxValue<int> PacketRateHz { get; }
        IRxValue<RadioLinkStatus> RadioStatus { get; }

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
