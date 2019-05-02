﻿using System;
using System.Collections.Generic;
using System.Reactive.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;
using NLog;

namespace Asv.Mavlink
{
    public class VehicleBaseConfig
    {
        public int HeartbeatTimeoutMs { get; set; } = 2000;
        public int CommandTimeoutMs { get; set; } = 10000;
    }

    public abstract class VehicleBase : IVehicle
    {
        public static Logger Logger = LogManager.GetCurrentClassLogger();
        private AutopilotVersionPayload _autopilotVersion;

        private readonly IMavlinkV2Protocol _mavlink;
        private readonly VehicleBaseConfig _config;


        protected readonly CancellationTokenSource DisposeCancel = new CancellationTokenSource();
        private int _isDisposed;


        protected VehicleBase(IMavlinkV2Protocol mavlink, VehicleBaseConfig config )
        {
            _mavlink = mavlink;
            _config = config;
        }

        public IRxValue<int> PacketRateHz => _mavlink.Rtt.PacketRateHz;


        public virtual Task Init(CancellationToken cancel)
        {
            InitAttitude();
            InitLink();
            InitGps();
            InitHome();
            InitArmed();
            InitBattery();
            InitAltitude();
            InitStatus();
            InitMode();
            return Task.CompletedTask;
        }


        public virtual void Dispose()
        {
            if (Interlocked.CompareExchange(ref _isDisposed,1,0) == 1) return;
            DisposeCancel?.Cancel(false);
            DisposeCancel?.Dispose();
            _mavlink?.Dispose();
        }

        protected async Task WaitCompleteWithDefaultTimeout(Func<bool> condition, string actionName, CancellationToken cancel)
        {
            var started = DateTime.Now;
            while (!condition())
            {
                await Task.Delay(500, cancel);
                if ((DateTime.Now - started).Milliseconds > _config.CommandTimeoutMs)
                {
                    throw new TimeoutException(string.Format(RS.VehicleBase_WaitCompleteWithDefaultTimeout_Timeout_to_execute, actionName, TimeSpan.FromMilliseconds(_config.CommandTimeoutMs))); ;
                }
            }
        }

        protected void ValidateCommandResult(CommandAckPayload result)
        {
            switch (result.Result)
            {
                case MavResult.MavResultTemporarilyRejected:
                case MavResult.MavResultDenied:
                case MavResult.MavResultUnsupported:
                case MavResult.MavResultFailed:
                    throw new CommandException(result);
            }
        }

        protected Task<AutopilotVersionPayload> GetAutopilotVersion(CancellationToken cancel)
        {
            return _autopilotVersion != null ? Task.FromResult(_autopilotVersion) : _mavlink.Commands.GetAutopilotVersion(cancel);
        }

        #region Mode

        private readonly RxValue<VehicleMode> _mode = new RxValue<VehicleMode>();
        public IRxValue<VehicleMode> Mode => _mode;

        protected abstract VehicleMode InterpretateMode(HeartbeatPayload heartbeat);

        private void InitMode()
        {
            _mavlink.Rtt.RawHeartbeat.Select(InterpretateMode).Subscribe(_mode, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _dropRateComm.Dispose());
        }

        /// <summary>
        /// Check is current mode can used for remote control from GCS with GPS positioning. It depend from vehicle types 
        /// </summary>
        /// <param name="cancel"></param>
        /// <returns></returns>
        protected abstract Task<bool> CheckGuidedMode(CancellationToken cancel);

        /// <summary>
        /// CheckGuidedMode and switch, if it not in guided mode. It depend from vehicle types 
        /// </summary>
        /// <param name="cancel"></param>
        /// <returns></returns>
        protected abstract Task EnsureInGuidedMode(CancellationToken cancel);

        #endregion


        

        #region Attitude



        private readonly RxValue<double> _pitch = new RxValue<double>();
        private readonly RxValue<double> _roll = new RxValue<double>();
        private readonly RxValue<double> _yaw = new RxValue<double>();
        private readonly RxValue<double> _pitchSpeed = new RxValue<double>();
        private readonly RxValue<double> _rollSpeed = new RxValue<double>();
        private readonly RxValue<double> _yawSpeed = new RxValue<double>();


        public IRxValue<double> Pitch => _pitch;
        public IRxValue<double> Roll => _roll;
        public IRxValue<double> Yaw => _yaw;
        public IRxValue<double> PitchSpeed => _pitchSpeed;
        public IRxValue<double> RollSpeed => _rollSpeed;
        public IRxValue<double> YawSpeed => _yawSpeed;


        protected virtual void InitAttitude()
        {
            _mavlink.Rtt.RawAttitude.Select(_ => (double)_.Pitch).Subscribe(_pitch, DisposeCancel.Token);
            _mavlink.Rtt.RawAttitude.Select(_ => (double)_.Roll).Subscribe(_roll, DisposeCancel.Token);
            _mavlink.Rtt.RawAttitude.Select(_ => (double)_.Yaw).Subscribe(_yaw, DisposeCancel.Token);
            _mavlink.Rtt.RawAttitude.Select(_ => (double)_.Pitchspeed).Subscribe(_pitchSpeed, DisposeCancel.Token);
            _mavlink.Rtt.RawAttitude.Select(_ => (double)_.Rollspeed).Subscribe(_rollSpeed, DisposeCancel.Token);
            _mavlink.Rtt.RawAttitude.Select(_ => (double)_.Yawspeed).Subscribe(_yawSpeed, DisposeCancel.Token);

            DisposeCancel.Token.Register(() => _pitch.Dispose());
            DisposeCancel.Token.Register(() => _roll.Dispose());
            DisposeCancel.Token.Register(() => _yaw.Dispose());
            DisposeCancel.Token.Register(() => _pitchSpeed.Dispose());
            DisposeCancel.Token.Register(() => _rollSpeed.Dispose());
            DisposeCancel.Token.Register(() => _yawSpeed.Dispose());


        }

        #endregion

        #region Link

        private DateTime _lastHeartbeat;
        private readonly LinkIndicator _link = new LinkIndicator(3);
        public IRxValue<LinkState> Link => _link;

        private void InitLink()
        {
            Observable.Timer(TimeSpan.FromSeconds(1),TimeSpan.FromSeconds(1)).Subscribe(CheckConnection,DisposeCancel.Token);
            _mavlink.Rtt.RawHeartbeat.Subscribe(_ =>
            {
                _lastHeartbeat = DateTime.Now;
                _link.Upgrade();
            }, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _link.Dispose());
        }

        private void CheckConnection(long value)
        {
            if (DateTime.Now - _lastHeartbeat > TimeSpan.FromMilliseconds(_config.HeartbeatTimeoutMs))
            {
                _link.Downgrade();
            }
        }

        #endregion

        #region Altitude

        private readonly RxValue<double> _altitudeRelative = new RxValue<double>();
        public IRxValue<double> AltitudeAboveHome => _altitudeRelative;

        private void InitAltitude()
        {
            _mavlink.Rtt.RawGlobalPositionInt.Select(_=>_.RelativeAlt/1000D).Subscribe(_altitudeRelative, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _altitudeRelative.Dispose());
        }

       

        #endregion

        #region GPS

        private readonly RxValue<GpsInfo>  _gpsInfo = new RxValue<GpsInfo>();
        private readonly RxValue<double> _gVelocity = new RxValue<double>();
        private readonly RxValue<GeoPoint> _globGps = new RxValue<GeoPoint>();

        public IRxValue<GeoPoint> GpsLocation => _globGps;
        public IRxValue<GpsInfo> GpsInfo => _gpsInfo;
        public IRxValue<double> GroundVelocity => _gVelocity;
        

        protected virtual void InitGps()
        {
            
            _mavlink.Rtt.RawGpsRawInt.Select(_ => new GeoPoint(_.Lat / 10000000D, _.Lon / 10000000D, _.Alt / 1000D)).Subscribe(_globGps, DisposeCancel.Token);
            _mavlink.Rtt.RawGpsRawInt.Select(_ => new GpsInfo(_)).Subscribe(_gpsInfo, DisposeCancel.Token);
            _mavlink.Rtt.RawGpsRawInt.Select(_ => _.Vel / 100D).Subscribe(_gVelocity, DisposeCancel.Token);

            DisposeCancel.Token.Register(() => _gVelocity.Dispose());
            DisposeCancel.Token.Register(() => _gpsInfo.Dispose());
            DisposeCancel.Token.Register(() => _globGps.Dispose());
        }

        #endregion

        #region Home

        private readonly RxValue<GeoPoint> _home = new RxValue<GeoPoint>();
        public IRxValue<GeoPoint> Home => _home;
        protected virtual void InitHome()
        {
            _mavlink.Rtt.RawHome
                .Select(_ => new GeoPoint(_.Latitude / 10000000D, _.Longitude / 10000000D, _.Altitude / 1000D));
            DisposeCancel.Token.Register(() => _home.Dispose());
        }

        #endregion

        #region Arm

        private readonly RxValue<bool> _isArmed = new RxValue<bool>();
        public IRxValue<bool> IsArmed => _isArmed;
        protected virtual void InitArmed()
        {
            _mavlink.Rtt.RawHeartbeat.Select(_ => _.BaseMode.HasFlag(MavModeFlag.MavModeFlagSafetyArmed)).Subscribe(_isArmed, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _isArmed.Dispose());
        }

        public virtual async Task ArmDisarm(bool isArming,CancellationToken cancel)
        {
            if (_isArmed.Value == isArming) return;

            var result = await _mavlink.Commands.CommandLong(MavCmd.MavCmdComponentArmDisarm, isArming ? 1 : 0, float.NaN, float.NaN, float.NaN, float.NaN, float.NaN, float.NaN, 3, cancel);
            ValidateCommandResult(result);
            await WaitCompleteWithDefaultTimeout(() => _isArmed.Value, "Arm/Disarm", cancel);
        }

        #endregion

        #region Battery

        private readonly RxValue<double?> _batteryCharge = new RxValue<double?>();
        public IRxValue<double?> BatteryCharge => _batteryCharge;

        private readonly RxValue<double?> _currentBattery = new RxValue<double?>();
        public IRxValue<double?> CurrentBattery => _currentBattery;

        private readonly RxValue<double> _voltageBattery = new RxValue<double>();
        public IRxValue<double> VoltageBattery => _voltageBattery;

        protected virtual void InitBattery()
        {
            /// TODO: add _mavlink.Rtt.RawBatteryStatus

            _mavlink.Rtt.RawSysStatus.Select(_ => _.BatteryRemaining < 0 ? default(double?) : (_.BatteryRemaining / 100.0d)).Subscribe(_batteryCharge,DisposeCancel.Token);
            _mavlink.Rtt.RawSysStatus.Select(_ => _.CurrentBattery < 0 ? default(double?) : (_.CurrentBattery / 1000.0d)).Subscribe(_currentBattery, DisposeCancel.Token);
            _mavlink.Rtt.RawSysStatus.Select(_ => _.VoltageBattery / 1000.0d).Subscribe(_voltageBattery, DisposeCancel.Token);

            DisposeCancel.Token.Register(() => _batteryCharge.Dispose());
            DisposeCancel.Token.Register(() => _currentBattery.Dispose());
            DisposeCancel.Token.Register(() => _voltageBattery.Dispose());

        }

        #endregion

        #region Takeoff

        public virtual async Task TakeOff(double altitude, CancellationToken cancel)
        {
            await EnsureInGuidedMode(cancel);
            await ArmDisarm(true, cancel);
            var res = await _mavlink.Commands.CommandLong(MavCmd.MavCmdNavTakeoff, float.NaN, float.NaN, float.NaN, float.NaN, (float)_globGps.Value.Latitude, (float)_globGps.Value.Longitude, (float)altitude, 3, cancel);
            ValidateCommandResult(res);
        }

        #endregion

        #region GoTo

        public Task GoToGlob(GeoPoint location, CancellationToken cancel, double? yawDeg = null)
        {
            return GoTo(location, MavFrame.MavFrameGlobalInt, cancel, yawDeg);
        }

        public virtual Task DoRtl(CancellationToken cancel)
        {
            throw new NotImplementedException();
        }
        
        public Task GoToRel(GeoPoint location, CancellationToken cancel, double? yawDeg = null)
        {
             return GoTo(location, MavFrame.MavFrameGlobalRelativeAltInt, cancel, yawDeg);
        }

        private async Task GoTo(GeoPoint location, MavFrame frame, CancellationToken cancel, double? yawDeg = null, double? vx = null, double? vy = null, double? vz = null)
        {
            await EnsureInGuidedMode(cancel);
            if (location.Altitude.HasValue == false)
            {
                throw new MavlinkException(RS.VehicleArdupilot_GoTo_Altitude_of_end_point_is_null);
            }

            var yaw = yawDeg.HasValue ? (float?) GeoMath.DegreesToRadians(yawDeg.Value) : null;
            await _mavlink.Common.SetPositionTargetGlobalInt(0, frame,cancel, (int)(location.Latitude * 10000000), (int)(location.Longitude * 10000000), (float)location.Altitude.Value, yaw: yaw);
        }

        #endregion


        

        #region Status

        private readonly RxValue<VehicleStatusMessage> _textStatus = new RxValue<VehicleStatusMessage>();
        public IRxValue<VehicleStatusMessage> TextStatus => _textStatus;

        private readonly RxValue<double> _cpuLoad = new RxValue<double>();
        public IRxValue<double> CpuLoad => _cpuLoad;

        private readonly RxValue<double> _dropRateComm = new RxValue<double>();
        public IRxValue<double> DropRateCommunication => _dropRateComm;

        private void InitStatus()
        {
            _mavlink.Rtt.RawStatusText.Select(_ => new VehicleStatusMessage { Text = new string(_.Text), Type = _.Severity  }).Subscribe(_textStatus,DisposeCancel.Token);
            _mavlink.Rtt.RawSysStatus.Select(_=>_.Load/1000D).Subscribe(_cpuLoad, DisposeCancel.Token);
            _mavlink.Rtt.RawSysStatus.Select(_ => _.DropRateComm / 1000D).Subscribe(_dropRateComm, DisposeCancel.Token);
            

            DisposeCancel.Token.Register(() => _textStatus.Dispose());
            DisposeCancel.Token.Register(() => _cpuLoad.Dispose());
            DisposeCancel.Token.Register(() => _dropRateComm.Dispose());
        }

        #endregion
    }

    public class VehicleMode
    {
        public MavModeFlag BaseMode { get; set; }
        public uint CustomMode { get; set; }
        public string Name { get; set; }
    }
}