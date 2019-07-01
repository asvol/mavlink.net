using System;
using System.Collections.Generic;
using System.Diagnostics;
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
        public int RequestInitDataDelayAfterFailMs { get; set; } = 5000;
    }

    public abstract class VehicleBase : IVehicle
    {
        public static readonly Logger Logger = LogManager.GetCurrentClassLogger();
        private AutopilotVersionPayload _autopilotVersion;

        private readonly IMavlinkClient _mavlink;
        private readonly VehicleBaseConfig _config;


        protected readonly CancellationTokenSource DisposeCancel = new CancellationTokenSource();
        private int _isDisposed;


        protected VehicleBase(IMavlinkClient mavlink, VehicleBaseConfig config )
        {
            _mavlink = mavlink;
            _config = config;
        }

        public IRxValue<int> PacketRateHz => _mavlink.Rtt.PacketRateHz;

      



        public virtual void StartListen()
        {
            InitRequestVehicleInfo();
            InitAttitude();
            InitLink();
            InitRoi();
            InitGps();
            InitHome();
            InitArmed();
            InitBattery();
            InitAltitude();
            InitGoTo();
            InitStatus();
            InitMode();
        }

       


        #region Request init info

        private readonly RxValue<VehicleInitState> _initState = new RxValue<VehicleInitState>();


        public MavlinkClientIdentity Identity => _mavlink.Identity;
        public IRxValue<VehicleInitState> InitState => _initState;

        private int _isRequestInfoIsInProgressOrAlreadySuccess;
        private bool _needToRequestAgain = true;
        private void InitRequestVehicleInfo()
        {
            _initState.OnNext(VehicleInitState.WaitConnection);
            _link.DistinctUntilChanged().Where(_ => _ == LinkState.Disconnected).Subscribe(_ => _needToRequestAgain = true,DisposeCancel.Token);
            _link.DistinctUntilChanged().Where(_ => _needToRequestAgain).Where(_ => _ == LinkState.Connected)
                // only one time
                .Subscribe(_ => Task.Factory.StartNew(TryToRequestData,TaskCreationOptions.LongRunning), DisposeCancel.Token);
        }



        private async void TryToRequestData()
        {
            if (Interlocked.CompareExchange(ref _isRequestInfoIsInProgressOrAlreadySuccess,1,0) == 1) return;
            _initState.OnNext(VehicleInitState.InProgress);
            try
            {
                Logger.Info($"Request ALL data stream from vehicle");
                await _mavlink.Common.RequestDataStream(0, 2, true, DisposeCancel.Token);
                Logger.Info($"Request home position");
                await _mavlink.Commands.GetHomePosition(CancellationToken.None);
                //await _mavlink.Params.ReadAllParams(DisposeCancel.Token,);
                _initState.OnNext(VehicleInitState.Complete);
                _needToRequestAgain = false;
            }
            catch (Exception e)
            {
                Logger.Error(e, $"Error to read all vehicle info:{e.Message}");
                _initState.OnNext(VehicleInitState.Failed);
                Observable.Timer(TimeSpan.FromMilliseconds(_config.RequestInitDataDelayAfterFailMs))
                    .Subscribe(_ => TryToRequestData(), DisposeCancel.Token);
            }
            finally
            {
                Interlocked.Exchange(ref _isRequestInfoIsInProgressOrAlreadySuccess, 0);
            }
            
        }

        #endregion

        public virtual void Dispose()
        {
            Logger.Trace("Dispose vehicle");
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
            _mavlink.Rtt.RawAttitude.Select(_ => (double)GeoMath.RadiansToDegrees(_.Pitch)).Subscribe(_pitch, DisposeCancel.Token);
            _mavlink.Rtt.RawAttitude.Select(_ => (double) GeoMath.RadiansToDegrees(_.Roll)).Subscribe(_roll, DisposeCancel.Token);
            _mavlink.Rtt.RawAttitude.Select(_ => (double)GeoMath.RadiansToDegrees(_.Yaw)).Subscribe(_yaw, DisposeCancel.Token);
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

        #region ROI

        public virtual async Task SetRoi(GeoPoint location, CancellationToken cancel)
        {
            Logger.Info($"=> SetRoi(location:{location.ToString()})");
            var res = await _mavlink.Commands.CommandLong(MavCmd.MavCmdDoSetRoi, (int)MavRoi.MavRoiLocation, 0, 0, 0, (float)location.Latitude, (float)location.Longitude, (float)location.Altitude, 3, CancellationToken.None);
            Logger.Info($"<= SetRoi(location:{location.ToString()}): '{res.Result}'(porgress:{res.Progress};resultParam2:{res.ResultParam2})");
            ValidateCommandResult(res);
            _roi.OnNext(location);
        }

        protected virtual void InitRoi()
        {
            DisposeCancel.Token.Register(() => _roi.Dispose());
        }

        protected readonly RxValue<GeoPoint?> _roi = new RxValue<GeoPoint?>();
        public IRxValue<GeoPoint?> Roi => _roi;

        public virtual async Task ClearRoi(CancellationToken cancel)
        {
            Logger.Info($"=> ClearRoi()");
            var res = await _mavlink.Commands.CommandLong(MavCmd.MavCmdDoSetRoiNone, (int)MavRoi.MavRoiLocation, 0, 0, 0, 0, 0, 0, 3, CancellationToken.None);
            Logger.Info($"<= ClearRoi(): '{res.Result}'(porgress:{res.Progress};resultParam2:{res.ResultParam2})");
            ValidateCommandResult(res);
            _roi.OnNext(null);
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
                if (DisposeCancel.IsCancellationRequested) return;
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

        private readonly RxValue<GeoPoint?> _home = new RxValue<GeoPoint?>();
        private readonly RxValue<double?> _homeDistance = new RxValue<double?>();
        public IRxValue<GeoPoint?> Home => _home;
        public IRxValue<double?> HomeDistance => _homeDistance;
        protected virtual void InitHome()
        {
            _mavlink.Rtt.RawHome
                .Select(_ => (GeoPoint?) new GeoPoint(_.Latitude / 10000000D, _.Longitude / 10000000D, _.Altitude / 1000D))
                .Subscribe(_home,DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _homeDistance.Dispose());

            this.GpsLocation
                .Where(_=>_home.Value.HasValue)
                // ReSharper disable once PossibleInvalidOperationException
                .Select(_ => (double?)GeoMath.Distance(_home.Value.Value,_))
                .Subscribe(_homeDistance, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _home.Dispose());
        }

        #endregion

        #region Arm

        private readonly RxValue<bool> _isArmed = new RxValue<bool>();
        private readonly RxValue<TimeSpan> _armedTime = new RxValue<TimeSpan>();
        public Task RequestHome(CancellationToken cancel)
        {
            return _mavlink.Commands.GetHomePosition(cancel);
        }

        public IRxValue<bool> IsArmed => _isArmed;
        public IRxValue<TimeSpan> ArmedTime => _armedTime;
        private long _lastArmedTime;
        protected virtual void InitArmed()
        {
            var timer = Observable.Timer(TimeSpan.FromSeconds(1), TimeSpan.FromSeconds(1)).Where(_=>IsArmed.Value).Subscribe(_ =>
            {
                var lastBin = Interlocked.Read(ref _lastArmedTime);
                if (lastBin == 0)
                {
                    _armedTime.OnNext(TimeSpan.Zero);
                    return;
                }
                var last = DateTime.FromBinary(lastBin);
                var now = DateTime.Now;
                var delay = (now - last);
                _armedTime.OnNext(delay);
            });
            _isArmed.DistinctUntilChanged().Where(_ => _isArmed.Value).Subscribe(_ => Interlocked.Exchange(ref _lastArmedTime,DateTime.Now.ToBinary()) ,DisposeCancel.Token);
            DisposeCancel.Token.Register(() => timer.Dispose());

            _mavlink.Rtt.RawHeartbeat.Select(_ => _.BaseMode.HasFlag(MavModeFlag.MavModeFlagSafetyArmed)).Subscribe(_isArmed, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _isArmed.Dispose());
        }

        public virtual async Task ArmDisarm(bool isArming,CancellationToken cancel)
        {
            if (_isArmed.Value == isArming) return;
            Logger.Info($"=> ArmDisarm(isArming:{isArming})");
            var result = await _mavlink.Commands.CommandLong(MavCmd.MavCmdComponentArmDisarm, isArming ? 1 : 0, float.NaN, float.NaN, float.NaN, float.NaN, float.NaN, float.NaN, 3, cancel);
            Logger.Info($"=> ArmDisarm(isArming:{isArming}): '{result.Result}'(porgress:{result.Progress};resultParam2:{result.ResultParam2})");
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
            Logger.Info($"=> TakeOff(altitude:{altitude:F2})");
            await EnsureInGuidedMode(cancel);
            await ArmDisarm(true, cancel);
            var res = await _mavlink.Commands.CommandLong(MavCmd.MavCmdNavTakeoff, float.NaN, float.NaN, float.NaN, float.NaN, (float)_globGps.Value.Latitude, (float)_globGps.Value.Longitude, (float)altitude, 3, cancel);
            Logger.Info($"<= TakeOff(altitude:{altitude:F2}): '{res.Result}'(porgress:{res.Progress};resultParam2:{res.ResultParam2})");
            ValidateCommandResult(res);
        }

        #endregion

        #region GoTo

        public Task GoToGlob(GeoPoint location, CancellationToken cancel, double? yawDeg = null)
        {
            return GoTo(location, MavFrame.MavFrameGlobalInt, cancel, yawDeg);
        }

        public async Task GoToGlobAndWait(GeoPoint location, IProgress<double> progress, double precision, CancellationToken cancel)
        {
            await GoToGlob(location, cancel);
            progress = progress ?? new Progress<double>();
            var startLocation = this.GpsLocation.Value;
            var startDistance = Math.Abs(GeoMath.Distance(location, startLocation));

            Logger.Info("GoToAndWait {0} with precision {2:F1} m. Distance to target {3:F1}", location, precision, startDistance);
            progress.Report(0);
            if (startDistance <= precision)
            {
                Logger.Debug("Already in target, nothing to do", startLocation);
                progress.Report(1);
                return;
            }

            var sw = new Stopwatch();
            sw.Start();
            Logger.Debug("Send command GoTo to vehicle", startLocation);
            await this.GoToGlob(location, cancel).ConfigureAwait(false);
            double dist = 0;
            while (!cancel.IsCancellationRequested)
            {
                var loc = this.GpsLocation.Value;
                dist = Math.Abs(GeoMath.Distance(location, loc));
                var prog = 1 - dist / startDistance;
                Logger.Trace("Distance to target {0:F1}, location: {1}, progress {2:P2}", dist, loc, prog);
                progress.Report(prog);
                if (dist <= precision) break;
                await Task.Delay(TimeSpan.FromSeconds(1), cancel).ConfigureAwait(false);
            }
            sw.Stop();
            Logger.Info($"Complete {sw.Elapsed:hh\\:mm\\:ss} location error {dist:F1} m");
            progress.Report(1);
        }

        public virtual Task DoRtl(CancellationToken cancel)
        {
            throw new NotImplementedException();
        }
        
        public Task GoToRel(GeoPoint location, CancellationToken cancel, double? yawDeg = null)
        {
             return GoTo(location, MavFrame.MavFrameGlobalRelativeAltInt, cancel, yawDeg);
        }

        private readonly RxValue<GeoPoint?> _goToTarget = new RxValue<GeoPoint?>();
        public IRxValue<GeoPoint?> GoToTarget => _goToTarget;

        private void InitGoTo()
        {
            DisposeCancel.Token.Register(() => _goToTarget.Dispose());
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
            _goToTarget.OnNext(location);
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