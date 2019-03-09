﻿using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Reactive.Threading.Tasks;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Port;
using Asv.Mavlink.V2.Ardupilotmega;
using Asv.Mavlink.V2.Common;
using Asv.Mavlink.V2.Icarous;
using Asv.Mavlink.V2.Uavionix;
using Nito.AsyncEx;
using NLog;
using MavCmd = Asv.Mavlink.V2.Common.MavCmd;

namespace Asv.Mavlink
{
    public class VehicleConfig
    {
        public string ConnectionString { get; set; } = "0.0.0.0:14560";
        public int HeartbeatTimeoutMs { get; set; } = 2000;
        public byte SystemId { get; set; } = 254;
        public byte ComponentId { get; set; } = 254;
        public byte TargetSystemId { get; } = 1;
        public byte TargetComponentId { get; } = 1;
        public int CommandTimeoutMs { get; set; } = 1000;
        public int TimeoutToReadAllParamsMs { get; set; } = 10000;
        public int ReadParamTimeoutMs { get; set; } = 10000;
    }

    public class Vehicle : IVehicle
    {
        private readonly VehicleConfig _config;
        private readonly MavlinkV2Connection _mavlinkConnection;
        private readonly IPort _port;
        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();
        private readonly RawTelemetry _rtt;
        private readonly VehicleParameterProtocol _params;
        private readonly VehicleCommandProtocol _vehicleCommands;
        private readonly VehicleMissionProtocol _mission;
        private readonly OffboardMode _offboard;
        private readonly RxValue<DeserizliaePackageException> _packetErrors = new RxValue<DeserizliaePackageException>();
        private readonly IDisposable _packetErrorSubscribe;

        public Vehicle(VehicleConfig config)
        {
            if (config == null) throw new ArgumentNullException(nameof(config));

            _config = config;
            _port = PortFactory.Create(config.ConnectionString);
            _mavlinkConnection = new MavlinkV2Connection(_port, _ =>
<<<<<<< HEAD
=======
            {
                _.RegisterCommonDialect();
                _.RegisterArdupilotmegaDialect();
            });
            _mavlinkConnection.Port.Enable();
            InputPackets = _mavlinkConnection.Where(FilterVehicle);
            HandleStatistic();
            HandleHeartbeat(config);
            HandleSystemStatus();
            HandleGps();
            HandleHighresImu();
            HandleVfrHud();
            HandleAttitude();
            HandleBatteryStatus();
            HandleAltitude();
            HandleExtendedSysState();
            HandleParams();
            HandleHome();
        }

        public IRxValue<Exception> PortError => _port.Error;
        public IObservable<DeserizliaePackageException> OnPacketErrors => _mavlinkConnection.DeserializePackageErrors;

        public IRxValue<LinkState> Link => _link;
        public IRxValue<int> PacketRateHz => _packetRate;
        public IRxValue<GeoPoint> Home => _homePos;
        public IRxValue<bool> Armed => _armed;
        public IRxValue<HeartbeatPayload> RawHeartbeat => _heartBeat;
        public IRxValue<SysStatusPayload> RawSysStatus => _sysStatus;
        public IRxValue<GpsRawIntPayload> RawGpsRawInt => _gpsRawInt;
        public IRxValue<HighresImuPayload> RawHighresImu => _highresImu;
        public IRxValue<ExtendedSysStatePayload> RawExtendedSysState => _extendedSysState;
        public IRxValue<AltitudePayload> RawAltitude => _altitude;
        public IRxValue<BatteryStatusPayload> RawBatteryStatus => _batteryStatus;
        public IRxValue<AttitudePayload> RawAttitude => _attitude;
        public IRxValue<VfrHudPayload> RawVfrHud => _vfrHud;
        public IRxValue<GeoPoint> RelGps => _relGps;
        public IRxValue<GeoPoint> GlobGps => _globGps;
        public IRxValue<HomePositionPayload> RawHome => _home;
        public IRxValue<StatustextPayload> RawStatusText => _statusText;
        public IReadOnlyDictionary<string, MavParam> Params => _params;
        public IRxValue<int?> ParamsCount => _paramsCount;
        public IObservable<MavParam> OnParamUpdated => _paramUpdated;

        private void HandleParams()
        {
            InputPackets
                .Where(_ => _.MessageId == ParamValuePacket.PacketMessageId)
                .Cast<ParamValuePacket>().Subscribe(UpdateParam,DisposeCancel.Token);

            DisposeCancel.Token.Register(() => _paramUpdated.Dispose());
            DisposeCancel.Token.Register(() => _paramsCount.Dispose());
        }

        private void UpdateParam(ParamValuePacket p)
        {
            var name = GetParamName(p.Payload);

            float? floatVal;
            long? longVal;
            ConvertFromMavlinkUnionToParamValue(p.Payload.ParamValue, p.Payload.ParamType, out floatVal, out longVal);
            var mavParam = new MavParam(p.Payload.ParamIndex, name, p.Payload.ParamType,floatVal,longVal);
            _params.AddOrUpdate(name, mavParam, (s, param) => mavParam);
            _paramUpdated.OnNext(mavParam);
            _paramsCount.OnNext(p.Payload.ParamCount);
        }

        private float ConvertToMavlinkUnionToParamValue(MavParam param)
        {
            var arr = BitConverter.GetBytes(0.0F);
            switch (param.Type)
            {
                case MavParamType.MavParamTypeUint8:
                    if (!param.IntegerValue.HasValue)
                        throw new Exception(string.Format(RS.Vehicle_ConvertToMavlinkUnionToParamValue_Integer_value_not_assigned_for_param, param.Name, param.Type));
                    arr[0] = (byte)(param.IntegerValue & 0xFF);
                    break;
                case MavParamType.MavParamTypeInt8:
                    if (!param.IntegerValue.HasValue)
                        throw new Exception(string.Format(RS.Vehicle_ConvertToMavlinkUnionToParamValue_Integer_value_not_assigned_for_param, param.Name, param.Type));
                    arr[0] = (byte)(param.IntegerValue & 0xFF);
                    break;
                case MavParamType.MavParamTypeUint16:
                    if (!param.IntegerValue.HasValue)
                        throw new Exception(string.Format(RS.Vehicle_ConvertToMavlinkUnionToParamValue_Integer_value_not_assigned_for_param, param.Name, param.Type));
                    arr[0] = (byte)(param.IntegerValue & 0xFF);
                    arr[1] = (byte)((param.IntegerValue >> 8) & 0xFF);
                    break;
                case MavParamType.MavParamTypeInt16:
                    if (!param.IntegerValue.HasValue)
                        throw new Exception(string.Format(RS.Vehicle_ConvertToMavlinkUnionToParamValue_Integer_value_not_assigned_for_param, param.Name, param.Type));
                    arr[0] = (byte)(param.IntegerValue & 0xFF);
                    arr[1] = (byte)((param.IntegerValue >> 8) & 0xFF);
                    break;
                case MavParamType.MavParamTypeUint32:
                    if (!param.IntegerValue.HasValue)
                        throw new Exception(string.Format(RS.Vehicle_ConvertToMavlinkUnionToParamValue_Integer_value_not_assigned_for_param, param.Name, param.Type));
                    arr[0] = (byte)(param.IntegerValue & 0xFF);
                    arr[1] = (byte)((param.IntegerValue >> 8) & 0xFF);
                    arr[2] = (byte)((param.IntegerValue >> 16) & 0xFF);
                    arr[3] = (byte)((param.IntegerValue >> 24) & 0xFF);
                    break;
                case MavParamType.MavParamTypeInt32:
                    if (!param.IntegerValue.HasValue)
                        throw new Exception(string.Format(RS.Vehicle_ConvertToMavlinkUnionToParamValue_Integer_value_not_assigned_for_param, param.Name, param.Type));
                    arr[0] = (byte)(param.IntegerValue & 0xFF);
                    arr[1] = (byte)((param.IntegerValue >> 8) & 0xFF);
                    arr[2] = (byte)((param.IntegerValue >> 16) & 0xFF);
                    arr[3] = (byte)((param.IntegerValue >> 24) & 0xFF);
                    break;
                case MavParamType.MavParamTypeUint64:
                    throw new MavlinkException(RS.Vehicle_ConvertToMavlinkUnionToParamValue_NeedMoreByte);
                case MavParamType.MavParamTypeInt64:
                    throw new MavlinkException(RS.Vehicle_ConvertToMavlinkUnionToParamValue_NeedMoreByte);
                case MavParamType.MavParamTypeReal32:
                    if (!param.RealValue.HasValue)
                        throw new Exception(string.Format(RS.Vehicle_ConvertToMavlinkUnionToParamValue_Real_value_not_assigned_for_param, param.Name, param.Type));
                    return param.RealValue.Value;
                case MavParamType.MavParamTypeReal64:
                    throw new MavlinkException(RS.Vehicle_ConvertToMavlinkUnionToParamValue_NeedMoreByte);
                default:
                    throw new ArgumentOutOfRangeException(nameof(param.Type), param.Type, null);
            }
            return BitConverter.ToSingle(arr, 0);
        }

        private void ConvertFromMavlinkUnionToParamValue(float payloadParamValue, MavParamType payloadParamType, out float? floatVal,out long? longVal)
        {

            // MAVLink (v1.0, v2.0) supports these data types:
            // uint32_t - 32bit unsigned integer(use the ENUM value MAV_PARAM_TYPE_UINT32)
            // int32_t - 32bit signed integer(use the ENUM value MAV_PARAM_TYPE_INT32)
            // float - IEEE754 single precision floating point number(use the ENUM value MAV_PARAM_TYPE_FLOAT)
            // All parameters are send as the float value of mavlink_param_union_t, which means that a parameter 
            // should be byte-wise converted with this union to a byte-wise float (no type conversion). 
            // This is necessary in order to not limit the maximum precision for scaled integer params. 
            // E.g. GPS coordinates can only be expressed with single float precision up to a few meters, while GPS coordinates in 1E7 scaled integers 
            // provide very high accuracy.

            switch (payloadParamType)
            {
                case MavParamType.MavParamTypeUint8:
                    longVal = BitConverter.GetBytes(payloadParamValue)[0];
                    floatVal = null;
                    break;
                case MavParamType.MavParamTypeInt8:
                    longVal = (sbyte)BitConverter.GetBytes(payloadParamValue)[0];
                    floatVal = null;
                    break;
                case MavParamType.MavParamTypeUint16:
                    longVal = BitConverter.ToUInt16(BitConverter.GetBytes(payloadParamValue),0);
                    floatVal = null;
                    break;
                case MavParamType.MavParamTypeInt16:
                    longVal = BitConverter.ToInt16(BitConverter.GetBytes(payloadParamValue), 0);
                    floatVal = null;
                    break;
                case MavParamType.MavParamTypeUint32:
                    longVal = BitConverter.ToUInt32(BitConverter.GetBytes(payloadParamValue), 0);
                    floatVal = null;
                    break;
                case MavParamType.MavParamTypeInt32:
                    longVal = BitConverter.ToInt32(BitConverter.GetBytes(payloadParamValue), 0);
                    floatVal = null;
                    break;
                case MavParamType.MavParamTypeUint64:
                    throw new MavlinkException(RS.Vehicle_ConvertToMavlinkUnionToParamValue_NeedMoreByte);
                case MavParamType.MavParamTypeInt64:
                    throw new MavlinkException(RS.Vehicle_ConvertToMavlinkUnionToParamValue_NeedMoreByte);
                case MavParamType.MavParamTypeReal32:
                    floatVal = payloadParamValue;
                    longVal = null;
                    break;
                case MavParamType.MavParamTypeReal64:
                    throw new MavlinkException(RS.Vehicle_ConvertToMavlinkUnionToParamValue_NeedMoreByte);
                default:
                    throw new ArgumentOutOfRangeException(nameof(payloadParamType), payloadParamType, null);
            }
        }

        private void HandleHome()
        {
            InputPackets
                .Where(_ => _.MessageId == HomePositionPacket.PacketMessageId)
                .Cast<HomePositionPacket>()
                .Select(_ => _.Payload)
                .Subscribe(_home, DisposeCancel.Token);
            _home
                .Select(_ => new GeoPoint(_.Latitude / 10000000D, _.Longitude / 10000000D, _.Altitude / 1000D))
                .Subscribe(_homePos, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _homePos.Dispose());
            DisposeCancel.Token.Register(() => _home.Dispose());
        }

        private void HandleExtendedSysState()
        {
            InputPackets
                .Where(_ => _.MessageId == ExtendedSysStatePacket.PacketMessageId)
                .Cast<ExtendedSysStatePacket>()
                .Select(_ => _.Payload)
                .Subscribe(_extendedSysState, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _extendedSysState.Dispose());
        }

        private void HandleAltitude()
        {
            InputPackets
                .Where(_ => _.MessageId == AltitudePacket.PacketMessageId)
                .Cast<AltitudePacket>()
                .Select(_ => _.Payload)
                .Subscribe(_altitude, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _altitude.Dispose());
        }

        private void HandleBatteryStatus()
        {
            InputPackets
                .Where(_ => _.MessageId == BatteryStatusPacket.PacketMessageId)
                .Cast<BatteryStatusPacket>()
                .Select(_ => _.Payload)
                .Subscribe(_batteryStatus, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _batteryStatus.Dispose());
        }

        private void HandleAttitude()
        {
            InputPackets
                .Where(_ => _.MessageId == AttitudePacket.PacketMessageId)
                .Cast<AttitudePacket>()
                .Select(_ => _.Payload)
                .Subscribe(_attitude, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _attitude.Dispose());
        }

        private void HandleVfrHud()
        {
            InputPackets
                .Where(_ => _.MessageId == VfrHudPacket.PacketMessageId)
                .Cast<VfrHudPacket>()
                .Select(_ => _.Payload)
                .Subscribe(_vfrHud, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _vfrHud.Dispose());
        }

        private void HandleHighresImu()
        {
            InputPackets
                .Where(_ => _.MessageId == HighresImuPacket.PacketMessageId)
                .Cast<HighresImuPacket>()
                .Select(_ => _.Payload)
                .Subscribe(_highresImu, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _highresImu.Dispose());
        }

        private void HandleGps()
        {
            var s =InputPackets
                .Where(_ => _.MessageId == GpsRawIntPacket.PacketMessageId)
                .Cast<GpsRawIntPacket>()
                .Select(_ => _.Payload);
            s.Subscribe(_gpsRawInt, DisposeCancel.Token);
            s.Select(_ => new GeoPoint(_.Lat / 10000000D, _.Lon / 10000000D, (_.Alt / 1000D) - Home.Value.Altitude ?? 0)).Subscribe(_relGps,DisposeCancel.Token);
            s.Select(_ => new GeoPoint(_.Lat / 10000000D, _.Lon / 10000000D, _.Alt / 1000D)).Subscribe(_globGps, DisposeCancel.Token);

            DisposeCancel.Token.Register(() => _gpsRawInt.Dispose());
            DisposeCancel.Token.Register(() => _relGps.Dispose());
            DisposeCancel.Token.Register(() => _globGps.Dispose());

        }

        private void HandleSystemStatus()
        {
            InputPackets
                .Where(_ => _.MessageId == SysStatusPacket.PacketMessageId)
                .Cast<SysStatusPacket>()
                .Select(_ => _.Payload)
                .Subscribe(_sysStatus, DisposeCancel.Token);
            InputPackets
                .Where(_ => _.MessageId == StatustextPacket.PacketMessageId)
                .Cast<StatustextPacket>()
                .Select(_ => _.Payload)
                .Subscribe(_statusText, DisposeCancel.Token);

            DisposeCancel.Token.Register(() => _sysStatus.Dispose());
            DisposeCancel.Token.Register(() => _statusText.Dispose());
        }

        private void HandleStatistic()
        {
            InputPackets
                .Select(_ => 1)
                .Buffer(TimeSpan.FromSeconds(1))
                .Select(_ => _.Sum()).Subscribe(_packetRate, DisposeCancel.Token);
            DisposeCancel.Token.Register(() => _packetRate.Dispose());
        }

        private void HandleHeartbeat(VehicleConfig config)
        {
            InputPackets
                .Where(_ => _.MessageId == HeartbeatPacket.PacketMessageId)
                .Cast<HeartbeatPacket>()
                .Subscribe(_ => TaskFactory.StartNew(() => OnHeartBeat(_)), DisposeCancel.Token);
            Observable
                .Timer(TimeSpan.FromMilliseconds(config.HeartbeatTimeoutMs), TimeSpan.FromMilliseconds(config.HeartbeatTimeoutMs))
                .Subscribe(_ => TaskFactory.StartNew(() => CheckConnection(_)), DisposeCancel.Token);
        }

        private bool FilterVehicle(IPacketV2<IPayload> packetV2)
        {
            if (_config.TargetSystemId != 0 && _config.TargetSystemId != packetV2.SystemId) return false;
            if (_config.TargetComponenId != 0 && _config.TargetComponenId != packetV2.ComponenId) return false;
            return true;
        }

        private void CheckConnection(long value)
        {
            _ts.VerifyAccess();
            if (DateTime.Now - _lastHearteat > TimeSpan.FromMilliseconds(_config.HeartbeatTimeoutMs))
            {
                _link.Downgrade();
            }
        }

        private void OnHeartBeat(HeartbeatPacket heartbeatPacket)
        {
            _ts.VerifyAccess();
            _lastHearteat = DateTime.Now;
            _link.Upgrade();
            _heartBeat.OnNext(heartbeatPacket.Payload);
            _armed.OnNext(heartbeatPacket.Payload.BaseMode.HasFlag(MavModeFlag.MavModeFlagSafetyArmed));

            DisposeCancel.Token.Register(() => _armed.Dispose());
            DisposeCancel.Token.Register(() => _heartBeat.Dispose());
        }

        public async Task ReadAllParams(CancellationToken cancel, IProgress<double> progress = null)
        {
            progress = progress ?? new Progress<double>();
            var packet = new ParamRequestListPacket
            {
                ComponenId = _config.ComponentId,
                SystemId = _config.SystemId,
                Payload =
                {
                    TargetComponent = _config.TargetComponenId,
                    TargetSystem = _config.TargetSystemId,
                }
            };

            var t = Task.Factory.StartNew(_=>InternalReadParams(progress) , cancel, TaskCreationOptions.LongRunning);
            // we need start listen before send request
            while (t.Status != TaskStatus.Running)
            {
                await Task.Delay(TaskStartDelayMs, cancel).ConfigureAwait(false);
            }
            await _mavlinkConnection.Send(packet, cancel).ConfigureAwait(false);
            await t.ConfigureAwait(false);
        }

        public async Task<MavParam> ReadParam(string name,int attemptCount, CancellationToken cancel)
        {
            var packet = new ParamRequestReadPacket
            {
                ComponenId = _config.ComponentId,
                SystemId = _config.SystemId,
                Payload =
                {
                    TargetComponent = _config.TargetComponenId,
                    TargetSystem = _config.TargetSystemId,
                    ParamId = SetParamName(name),
                    ParamIndex = -1,
                }
            };

            byte currentAttempt = 0;
            MavParam result = null;
            while (currentAttempt < attemptCount)
            {
                ++currentAttempt;

                using (var timeoutCancel = new CancellationTokenSource(_config.ReadParamTimeoutMs))
                using (var linkedCancel = CancellationTokenSource.CreateLinkedTokenSource(cancel, timeoutCancel.Token))
                {
                    var eve = new AsyncAutoResetEvent(false);
                    IDisposable subscribe = null;
                    try
                    {
                        subscribe = OnParamUpdated.FirstAsync(_ => _.Name == name)
                            .Subscribe(_ =>
                            {
                                result = _;
                                eve.Set();
                            });
                        await _mavlinkConnection.Send(packet, linkedCancel.Token).ConfigureAwait(false);
                        await eve.WaitAsync(linkedCancel.Token);
                    }
                    catch (TaskCanceledException)
                    {
                        if (!timeoutCancel.IsCancellationRequested)
                        {
                            throw;
                        }
                    }
                    finally
                    {
                        subscribe?.Dispose();
                    }
                    return result;
                }
            }
            if (result == null) throw new TimeoutException(string.Format("Timeout to read param '{0}' with '{1}' attempts (timeout {1} times by {2:g} )", name, currentAttempt, TimeSpan.FromMilliseconds(_config.CommandTimeoutMs)));
            return result;
        }

        public async Task<MavParam> ReadParam(short index, int attemptCount, CancellationToken cancel)
        {
            var packet = new ParamRequestReadPacket
            {
                ComponenId = _config.ComponentId,
                SystemId = _config.SystemId,
                Payload =
                {
                    TargetComponent = _config.TargetComponenId,
                    TargetSystem = _config.TargetSystemId,
                    ParamId = SetParamName(string.Empty),
                    ParamIndex = index,
                }
            };
            byte currentAttempt = 0;
            MavParam result = null;
            while (currentAttempt < attemptCount)
            {
                ++currentAttempt;

                using (var timeoutCancel = new CancellationTokenSource(_config.ReadParamTimeoutMs))
                using (var linkedCancel = CancellationTokenSource.CreateLinkedTokenSource(cancel, timeoutCancel.Token))
                {
                    var eve = new AsyncAutoResetEvent(false);
                    IDisposable subscribe = null;
                    try
                    {
                        subscribe = OnParamUpdated.FirstAsync(_ => _.Index == index)
                            .Subscribe(_ =>
                            {
                                result = _;
                                eve.Set();
                            } );
                        await _mavlinkConnection.Send(packet, linkedCancel.Token).ConfigureAwait(false);
                        await eve.WaitAsync(linkedCancel.Token);
                    }
                    catch (TaskCanceledException)
                    {
                        if (!timeoutCancel.IsCancellationRequested)
                        {
                            throw;
                        }
                    }
                    finally
                    {
                        subscribe?.Dispose();
                    }
                    return result;
                }
            }
            if (result == null) throw new TimeoutException(string.Format("Timeout to read param with index '{0}' with '{1}' attempts (timeout {1} times by {2:g} )", index, currentAttempt, TimeSpan.FromMilliseconds(_config.CommandTimeoutMs)));
            return result;
        }


        public async Task<MavParam> WriteParam(MavParam param, int attemptCount, CancellationToken cancel)
        {
            var packet = new ParamSetPacket()
            {
                ComponenId = _config.ComponentId,
                SystemId = _config.SystemId,
                Payload =
                {
                    TargetComponent = _config.TargetComponenId,
                    TargetSystem = _config.TargetSystemId,
                    ParamId = SetParamName(param.Name),
                    ParamType = param.Type,
                    ParamValue = ConvertToMavlinkUnionToParamValue(param)
                }
            };

            byte currentAttempt = 0;
            MavParam result = null;
            while (currentAttempt < attemptCount)
>>>>>>> 35246ed271febdb2117d29a0ca4f6b0d14722dcf
            {
                _.RegisterCommonDialect();
                _.RegisterArdupilotmegaDialect();
                _.RegisterIcarousDialect();
                _.RegisterUavionixDialect();
            });
            _mavlinkConnection.Port.Enable();
            _rtt = new RawTelemetry(_mavlinkConnection,new RawTelemetryConfig { ComponentId = _config.ComponentId ,HeartbeatTimeoutMs = _config.HeartbeatTimeoutMs,SystemId = _config.SystemId});
            _params = new VehicleParameterProtocol(_mavlinkConnection,new VehicleParameterProtocolConfig { ComponentId = _config.ComponentId,  SystemId = _config.SystemId, ReadWriteTimeoutMs = _config.ReadParamTimeoutMs,TimeoutToReadAllParamsMs = _config.TimeoutToReadAllParamsMs});
            _vehicleCommands = new VehicleCommandProtocol(_mavlinkConnection,new CommandProtocolConfig { ComponentId = _config.ComponentId,CommandTimeoutMs = _config.CommandTimeoutMs,SystemId = _config.SystemId});
            _mission = new VehicleMissionProtocol(_mavlinkConnection,
                new VehicleMissionProtocolConfig {ComponentId = _config.ComponentId, SystemId = _config.SystemId});
            _offboard = new OffboardMode(_mavlinkConnection, new OffboardModeConfig());
            _packetErrorSubscribe = _packetErrors.Subscribe(_packetErrors);
        }

        protected IMavlinkV2Connection Connection => _mavlinkConnection;


        public IRxValue<Exception> PortError => _port.Error;
        public IRxValue<DeserizliaePackageException> OnPacketErrors => _packetErrors;

        public IRawTelemetry Rtt => _rtt;
        public IVehicleParameterProtocol Params => _params;
        public IVehicleCommandProtocol Commands => _vehicleCommands;
        public IVehicleMissionProtocol Mission => _mission;
        public IOffboardMode Offboard => _offboard;


        public bool IsDisposed { get; private set; }

        public void Dispose()
        {
            if (IsDisposed) return;
            IsDisposed = true;
            try
            {
                _packetErrorSubscribe?.Dispose();
                _port?.Dispose();
                _mavlinkConnection?.Dispose();
                _rtt.Dispose();
                _params.Dispose();
                _vehicleCommands.Dispose();
                _mission.Dispose();
            }
            catch (Exception e)
            {
                Logger.Error(e, $"Exeption occured disposing vehicle:{e.Message}");
            }   
        }

        
    }
}