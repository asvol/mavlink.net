using System;
using System.Collections.Generic;
using System.Reactive.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Ardupilotmega;
using Asv.Mavlink.V2.Common;
using NLog;
using MavCmd = Asv.Mavlink.V2.Common.MavCmd;

namespace Asv.Mavlink
{

    public enum FirmwareType
    {
        ArduPlane,
        ArduCopter2,
        ArduRover,
        ArduSub,
        ArduTracker,
        Unknown
    }





    public class VehicleArdupilot : VehicleBase
    {
        private readonly IMavlinkClient _mavlink;
        private readonly VehicleBaseConfig _config;
        

        public VehicleArdupilot(IMavlinkClient mavlink, VehicleBaseConfig config) : base(mavlink, config)
        {
            _mavlink = mavlink;
            _config = config;
        }

        public override void StartListen()
        {
            base.StartListen();
            InitFirmware();
        }

        private readonly RxValue<FirmwareType> _firmware = new RxValue<FirmwareType>();
        public IRxValue<FirmwareType> Firmware => _firmware;

        private void InitFirmware()
        {
            _mavlink.Rtt.RawHeartbeat.Select(SelectFirmware).Subscribe(_firmware, DisposeCancel.Token);
        }

        private FirmwareType SelectFirmware(HeartbeatPayload heartbeatPayload)
        {
            if (heartbeatPayload.Autopilot == MavAutopilot.MavAutopilotArdupilotmega)
            {
                switch (heartbeatPayload.Type)
                {
                    case MavType.MavTypeFixedWing:
                        return FirmwareType.ArduPlane;
                    case MavType.MavTypeQuadrotor:
                    case MavType.MavTypeTricopter:
                    case MavType.MavTypeHexarotor:
                    case MavType.MavTypeOctorotor:
                    case MavType.MavTypeHelicopter:
                        return FirmwareType.ArduCopter2;
                    case MavType.MavTypeSurfaceBoat:
                    case MavType.MavTypeGroundRover:
                        return FirmwareType.ArduRover;
                    case MavType.MavTypeAntennaTracker:
                        return FirmwareType.ArduTracker;
                    case MavType.MavTypeSubmarine:
                        return FirmwareType.ArduSub;
                    default:
                        return FirmwareType.Unknown;
                }
            }

            return FirmwareType.Unknown;

        }


        protected override VehicleMode InterpretateMode(HeartbeatPayload heartbeat)
        {
            switch (_firmware.Value)
            {
                case FirmwareType.ArduPlane:
                    return new VehicleMode
                    {
                        BaseMode = heartbeat.BaseMode,
                        CustomMode = heartbeat.CustomMode,
                        Name = ((PlaneMode)(heartbeat.CustomMode)).ToString("G").Replace(nameof(PlaneMode),string.Empty),
                    };
                case FirmwareType.ArduCopter2:
                    return new VehicleMode
                    {
                        BaseMode = heartbeat.BaseMode,
                        CustomMode = heartbeat.CustomMode,
                        Name = ((CopterMode)(heartbeat.CustomMode)).ToString("G").Replace(nameof(CopterMode), string.Empty),
                    };
                case FirmwareType.ArduRover:
                    return new VehicleMode
                    {
                        BaseMode = heartbeat.BaseMode,
                        CustomMode = heartbeat.CustomMode,
                        Name = ((RoverMode)(heartbeat.CustomMode)).ToString("G").Replace(nameof(RoverMode), string.Empty),
                    };
                case FirmwareType.ArduSub:
                    return new VehicleMode
                    {
                        BaseMode = heartbeat.BaseMode,
                        CustomMode = heartbeat.CustomMode,
                        Name = ((SubMode)(heartbeat.CustomMode)).ToString("G").Replace(nameof(SubMode), string.Empty),
                    };
                
                case FirmwareType.ArduTracker:
                    return new VehicleMode
                    {
                        BaseMode = heartbeat.BaseMode,
                        CustomMode = heartbeat.CustomMode,
                        Name = ((TrackerMode)(heartbeat.CustomMode)).ToString("G").Replace(nameof(TrackerMode), string.Empty),
                    };
                case FirmwareType.Unknown:
                    return new VehicleMode
                    {
                        BaseMode = heartbeat.BaseMode,
                        CustomMode = heartbeat.CustomMode,
                        Name = "UNKNOWN",
                    };
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        protected override  Task<bool> CheckGuidedMode(CancellationToken cancel)
        {
            return Task.FromResult(
                _mavlink.Rtt.RawHeartbeat.Value.BaseMode.HasFlag(MavModeFlag.MavModeFlagCustomModeEnabled) &&
                _mavlink.Rtt.RawHeartbeat.Value.CustomMode == 4);
        }

        protected override async Task EnsureInGuidedMode(CancellationToken cancel)
        {
            if (!await CheckGuidedMode(cancel))
            {
                await _mavlink.Common.SetMode(1, 4, cancel);
            }
        }

        public override Task DoRtl(CancellationToken cancel)
        {
            return _mavlink.Common.SetMode(1, 6, cancel);
        }

        public override async Task SetRoi(GeoPoint location, CancellationToken cancel)
        {
            _roi.OnNext(location);
            try
            {
                // just send, because AEDUPILOT does not send mavcmd ack
                await _mavlink.Commands.SendCommandLong(MavCmd.MavCmdDoSetRoiLocation, 0, 0, 0, 0, 0, 0, 0, 1, CancellationToken.None);
                await _mavlink.Commands.SendCommandLong(MavCmd.MavCmdDoSetRoi, (int)MavRoi.MavRoiLocation, 0, 0, 0, (float)location.Latitude, (float)location.Longitude, (float)location.Altitude, 3, CancellationToken.None);
            }
            catch (Exception)
            {
                _roi.OnNext(null);
                throw;
            }
            
        }

        public override async Task ClearRoi(CancellationToken cancel)
        {
            var old = _roi.Value;
            _roi.OnNext(null);
            try
            {
                // just send, because ARDUPILOT does not send mavcmd ack
                await _mavlink.Commands.SendCommandLong(MavCmd.MavCmdDoSetRoiNone, 0, 0, 0, 0, 0, 0, 0, 1, CancellationToken.None);
                //await _mavlink.Commands.SendCommandLong(MavCmd.MavCmdDoSetRoiNone, (int)MavRoi.MavRoiLocation, 0, 0, 0, 0, 0, 0, 1, CancellationToken.None);
            }
            catch (Exception)
            {
                _roi.OnNext(old);
                throw;
            }

        }

        public override async Task TakeOff(double altitude, CancellationToken cancel)
        {
            try
            {
                await base.TakeOff(altitude, cancel);
            }
            catch (CommandException e)
            {
                if (e.Result.Result == MavResult.MavResultFailed)
                {
                    // IGNORE this. I don't know why, but the Ardupilot always send failed
                    return;
                }

                throw;
            }
            
        }
    }
}
