using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;
using Nito.AsyncEx;

namespace Asv.Mavlink.Client
{

    public class VehicleParameterProtocolConfig
    {
        public int TimeoutToReadAllParamsMs { get; set; } = (int) TimeSpan.FromSeconds(60).TotalMilliseconds;
        public int ReadWriteTimeoutMs { get; set; } = 10000;
    }

    public class MavlinkParameterClient : IMavlinkParameterClient,IDisposable
    {
        private readonly IMavlinkV2Connection _connection;
        private readonly MavlinkClientIdentity _identity;
        private readonly VehicleParameterProtocolConfig _config;
        private readonly ConcurrentDictionary<string, MavParam> _params = new ConcurrentDictionary<string, MavParam>();
        private readonly Subject<MavParam> _paramUpdated = new Subject<MavParam>();
        private readonly RxValue<int?> _paramsCount = new RxValue<int?>();
        private IDisposable _paramsSubscribe;

        public MavlinkParameterClient(IMavlinkV2Connection connection, MavlinkClientIdentity identity, VehicleParameterProtocolConfig config)
        {
            if (connection == null) throw new ArgumentNullException(nameof(connection));
            if (config == null) throw new ArgumentNullException(nameof(config));
            _connection = connection;
            _identity = identity;
            _config = config;
            HandleParams();
            Converter = new MavParamArdupilotValueConverter();
        }

        private bool FilterVehicle(IPacketV2<IPayload> packetV2)
        {
            if (_identity.TargetSystemId != 0 && _identity.TargetSystemId != packetV2.SystemId) return false;
            if (_identity.TargetComponentId != 0 && _identity.TargetComponentId != packetV2.ComponenId) return false;
            return true;
        }

        public IReadOnlyDictionary<string, MavParam> Params => _params;
        public IRxValue<int?> ParamsCount => _paramsCount;
        public IObservable<MavParam> OnParamUpdated => _paramUpdated;

        private void HandleParams()
        {
            _paramsSubscribe = _connection
                .Where(FilterVehicle)
                .Where(_ => _.MessageId == ParamValuePacket.PacketMessageId)
                .Cast<ParamValuePacket>().Subscribe(UpdateParam);
        }

        public Task RequestAllParams(CancellationToken cancel)
        {
            var packet = new ParamRequestListPacket
            {
                ComponenId = _identity.ComponentId,
                SystemId = _identity.SystemId,
                Payload =
                {
                    TargetComponent = _identity.TargetComponentId,
                    TargetSystem = _identity.TargetSystemId,
                }
            };
            return _connection.Send(packet, cancel);
        }

        public async Task ReadAllParams(CancellationToken cancel, IProgress<double> progress = null)
        {
            progress = progress ?? new Progress<double>();
            var packet = new ParamRequestListPacket
            {
                ComponenId = _identity.ComponentId,
                SystemId = _identity.SystemId,
                Payload =
                {
                    TargetComponent = _identity.TargetComponentId,
                    TargetSystem = _identity.TargetSystemId,
                }
            };

            var samplesBySecond = _connection
                .Where(FilterVehicle)
                .Where(_ => _.MessageId == ParamValuePacket.PacketMessageId)
                .Cast<ParamValuePacket>().Buffer(TimeSpan.FromSeconds(1)).Next();


            await _connection.Send(packet, cancel).ConfigureAwait(false);


            var timeout = DateTime.Now + TimeSpan.FromMilliseconds(_config.TimeoutToReadAllParamsMs);

            int? totalCnt = null;
            progress.Report(0);
            var paramsNames = new HashSet<string>();
            foreach (var paramsPart in samplesBySecond)
            {
                if (DateTime.Now >= timeout)
                {
                    throw new TimeoutException(string.Format(RS.Vehicle_ReadAllParams_Timeout_to_read_all_params_from_Vehicle, _config.TimeoutToReadAllParamsMs));
                }
                foreach (var p in paramsPart)
                {
                    totalCnt = totalCnt ?? p.Payload.ParamCount;
                    var name = GetParamName(p.Payload);
                    paramsNames.Add(name);
                }
                if (totalCnt.HasValue && totalCnt.Value <= paramsNames.Count) break;
                progress.Report(totalCnt == null ? 0 : Math.Min(1d, paramsNames.Count / (double)totalCnt));
            }
            progress.Report(1);
        }

        public async Task<MavParam> ReadParam(string name, int attemptCount, CancellationToken cancel)
        {
            var packet = new ParamRequestReadPacket
            {
                ComponenId = _identity.ComponentId,
                SystemId = _identity.SystemId,
                Payload =
                {
                    TargetComponent = _identity.TargetComponentId,
                    TargetSystem = _identity.TargetSystemId,
                    ParamId = SetParamName(name),
                    ParamIndex = -1,
                }
            };

            byte currentAttempt = 0;
            MavParam result = null;
            while (currentAttempt < attemptCount)
            {
                ++currentAttempt;

                using (var timeoutCancel = new CancellationTokenSource(_config.ReadWriteTimeoutMs))
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
                        await _connection.Send(packet, linkedCancel.Token).ConfigureAwait(false);
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
            if (result == null) throw new TimeoutException(string.Format("Timeout to read param '{0}' with '{1}' attempts (timeout {1} times by {2:g} )", name, currentAttempt, TimeSpan.FromMilliseconds(_config.ReadWriteTimeoutMs)));
            return result;
        }

        public async Task<MavParam> ReadParam(short index, int attemptCount, CancellationToken cancel)
        {
            var packet = new ParamRequestReadPacket
            {
                ComponenId = _identity.ComponentId,
                SystemId = _identity.SystemId,
                Payload =
                {
                    TargetComponent = _identity.TargetComponentId,
                    TargetSystem = _identity.TargetSystemId,
                    ParamId = SetParamName(string.Empty),
                    ParamIndex = index,
                }
            };
            byte currentAttempt = 0;
            MavParam result = null;
            while (currentAttempt < attemptCount)
            {
                ++currentAttempt;

                using (var timeoutCancel = new CancellationTokenSource(_config.ReadWriteTimeoutMs))
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
                            });
                        await _connection.Send(packet, linkedCancel.Token).ConfigureAwait(false);
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
            if (result == null) throw new TimeoutException(string.Format("Timeout to read param with index '{0}' with '{1}' attempts (timeout {1} times by {2:g} )", index, currentAttempt, TimeSpan.FromMilliseconds(_config.ReadWriteTimeoutMs)));
            return result;
        }

        public IMavParamValueConverter Converter { get; set; }

        public async Task<MavParam> WriteParam(MavParam param, int attemptCount, CancellationToken cancel)
        {
            var packet = new ParamSetPacket()
            {
                ComponenId = _identity.ComponentId,
                SystemId = _identity.SystemId,
                Payload =
                {
                    TargetComponent = _identity.TargetComponentId,
                    TargetSystem = _identity.TargetSystemId,
                    ParamId = SetParamName(param.Name),
                    ParamType = param.Type,
                    ParamValue = Converter.ConvertToMavlinkUnionToParamValue(param)
                }
            };

            byte currentAttempt = 0;
            MavParam result = null;
            while (currentAttempt < attemptCount)
            {
                ++currentAttempt;
                using (var timeoutCancel = new CancellationTokenSource(_config.ReadWriteTimeoutMs))
                using (var linkedCancel = CancellationTokenSource.CreateLinkedTokenSource(cancel, timeoutCancel.Token))
                {
                    var eve = new AsyncAutoResetEvent(false);
                    IDisposable subscribe = null;
                    try
                    {
                        subscribe = OnParamUpdated.FirstAsync(_ => _.Name == param.Name).Subscribe(_ =>
                        {
                            result = _;
                            eve.Set();
                        });
                        await _connection.Send(packet, linkedCancel.Token).ConfigureAwait(false);
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

            if (result == null) throw new TimeoutException(string.Format("Timeout to write param '{0}' with '{1}' attempts (timeout {1} times by {2:g} )", param.Name, currentAttempt, TimeSpan.FromMilliseconds(_config.ReadWriteTimeoutMs)));
            return result;


        }

        

       

        private char[] SetParamName(string name)
        {
            return name.PadRight(16, '\0').ToCharArray();
        }

        private void UpdateParam(ParamValuePacket p)
        {
            var name = GetParamName(p.Payload);

            float? floatVal;
            long? longVal;
            Converter.ConvertFromMavlinkUnionToParamValue(p.Payload.ParamValue, p.Payload.ParamType, out floatVal, out longVal);
            var mavParam = new MavParam(p.Payload.ParamIndex, name, p.Payload.ParamType, floatVal, longVal);
            _params.AddOrUpdate(name, mavParam, (s, param) => mavParam);
            _paramUpdated.OnNext(mavParam);
            _paramsCount.OnNext(p.Payload.ParamCount);
        }

        

        

        private string GetParamName(ParamValuePayload payload)
        {
            return new string(payload.ParamId.Where(_ => _ != '\0').ToArray());
        }

        public void Dispose()
        {
            _paramUpdated?.Dispose();
            _paramsCount?.Dispose();
            _params.Clear();
            _paramsSubscribe?.Dispose();
        }
    }
}