using System;
using System.Collections.Generic;
using System.Linq;
using System.Reactive.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Ardupilotmega;
using Asv.Mavlink.V2.Common;
using NLog;
using MavCmd = Asv.Mavlink.V2.Common.MavCmd;

namespace Asv.Mavlink
{

   


    public abstract class VehicleArdupilot : VehicleBase
    {
        private readonly IMavlinkClient _mavlink;
        private readonly VehicleBaseConfig _config;
        private const int DefaultAttemptToReadCount = 3;


        protected VehicleArdupilot(IMavlinkClient mavlink, VehicleBaseConfig config) : base(mavlink, config)
        {
            _mavlink = mavlink;
            _config = config;
        }

        public override async Task SetRoi(GeoPoint location, CancellationToken cancel)
        {
            _roi.OnNext(location);
            try
            {
                // just send, because ARDUPILOT does not send mavcmd ack
                await _mavlink.Commands.SendCommandLong(MavCmd.MavCmdDoSetRoiLocation, 0, 0, 0, 0, 0, 0, 0, 1, CancellationToken.None);
                await _mavlink.Commands.SendCommandLong(MavCmd.MavCmdDoSetRoi, (int)MavRoi.MavRoiLocation, 0, 0, 0, (float)location.Latitude, (float)location.Longitude, (float)location.Altitude, 3, CancellationToken.None);
            }
            catch (Exception)
            {
                _roi.OnNext(null);
                throw;
            }
            
        }

        public override IEnumerable<VehicleParamDescription> GetParamDescription()
        {
            return ArdupilotParamParser.Parse(Files.apm_pdef);
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

        public override async Task ArmDisarm(bool isArming, CancellationToken cancel, bool force = false)
        {
            const float magic_force_arm_value = 2989.0f;
            const float magic_force_disarm_value = 21196.0f;

            if (force)
            {
                var result = await _mavlink.Commands.CommandLong(MavCmd.MavCmdComponentArmDisarm, isArming ? 1 : 0, isArming? magic_force_arm_value : magic_force_disarm_value, float.NaN, float.NaN, float.NaN, float.NaN, float.NaN, 3, cancel);
                Logger.Info($"=> Ardupilot force ArmDisarm(isArming:{isArming}, force={force})): '{result.Result}'(porgress:{result.Progress};resultParam2:{result.ResultParam2})");
                ValidateCommandResult(result);
                await WaitCompleteWithDefaultTimeout(() => IsArmed.Value, "Arm/Disarm", cancel);
            }
            else
            {
                await base.ArmDisarm(isArming, cancel);
            }
        }
        /// <summary>
        /// https://ardupilot.org/copter/docs/common-flight-time-recorder.html
        /// ArduPilot includes a flight time recorder which records the board’s total flight time, total run time and number of times the board has been rebooted. These are stored in user resettable parameters meaning they are not protected from being tampered with.
        /// </summary>
        /// <param name="cancel"></param>
        /// <returns></returns>
        public override async Task<FlightTimeStatistic> GetFlightTimeStatistic(CancellationToken cancel = default(CancellationToken))
        {
            var stat = new FlightTimeStatistic
            {
                BootCount = (await Params.ReadParam("STAT_BOOTCNT", DefaultAttemptToReadCount,cancel)).IntegerValue.Value,
                FlightTime = TimeSpan.FromSeconds((double) (await Params.ReadParam("STAT_FLTTIME", DefaultAttemptToReadCount, cancel)).IntegerValue),
                RunTime = TimeSpan.FromSeconds((double) (await Params.ReadParam("STAT_RUNTIME", DefaultAttemptToReadCount, cancel)).IntegerValue),
            };
            return stat;
        }

        public override async Task ResetFlightStatistic(CancellationToken cancel)
        {
            var result = await Params.ReadParam("STAT_BOOTCNT", DefaultAttemptToReadCount, cancel);
            result.IntegerValue = 0;
            await Params.WriteParam(result, DefaultAttemptToReadCount, cancel);
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
