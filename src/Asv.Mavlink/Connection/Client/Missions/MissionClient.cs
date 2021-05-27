using System;
using System.Reactive.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Client;
using Asv.Mavlink.V2.Common;
using Nito.AsyncEx;
using NLog;

namespace Asv.Mavlink
{

    public class MissionClientConfig
    {
        public int CommandTimeoutMs { get; set; } = 1000;
    }

    public class MissionClient : IMissionClient,IDisposable
    {
        private readonly IMavlinkV2Connection _mavlink;
        private readonly IPacketSequenceCalculator _seq;
        private readonly MavlinkClientIdentity _identity;
        private readonly MissionClientConfig _config;
        private static readonly Logger _logger = LogManager.GetCurrentClassLogger();

        public MissionClient(IMavlinkV2Connection mavlink, IPacketSequenceCalculator seq,
            MavlinkClientIdentity identity, MissionClientConfig config)
        {
            _mavlink = mavlink ?? throw new ArgumentNullException(nameof(mavlink));
            _seq = seq ?? throw new ArgumentNullException(nameof(seq));
            _identity = identity ?? throw new ArgumentNullException(nameof(identity));
            _config = config;
        }

        private bool FilterVehicle(IPacketV2<IPayload> packetV2)
        {
            if (_identity.TargetSystemId != 0 && _identity.TargetSystemId != packetV2.SystemId) return false;
            if (_identity.TargetComponentId != 0 && _identity.TargetComponentId != packetV2.ComponenId) return false;
            return true;
        }

        public async Task<int> MissionRequestCount(int attemptCount, CancellationToken cancel)
        {
            _logger.Trace($"[MISSION]=> Begin request items count with {attemptCount} attempts");
            var packet = new MissionRequestListPacket()
            {
                ComponenId = _identity.ComponentId,
                SystemId = _identity.SystemId,
                Sequence = _seq.GetNextSequenceNumber(),
                Payload =
                {
                    TargetComponent = _identity.TargetComponentId,
                    TargetSystem = _identity.TargetSystemId,
                }
            };
            byte currentAttept = 0;
            MissionCountPacket result = null;
            while (currentAttept < attemptCount)
            {
                ++currentAttept;

                using (var timeoutCancel = new CancellationTokenSource(_config.CommandTimeoutMs))
                using (var linkedCancel = CancellationTokenSource.CreateLinkedTokenSource(cancel, timeoutCancel.Token))
                {
                    IDisposable subscribe = null;
                    try
                    {
                        var eve = new AsyncAutoResetEvent(false);
                        subscribe = _mavlink.Where(FilterVehicle).Where(_ => _.MessageId == MissionCountPacket.PacketMessageId)
                            .Cast<MissionCountPacket>()
                            .FirstAsync()
                            .Subscribe(_ =>
                            {
                                result = _;
                                eve.Set();
                            });
                        await _mavlink.Send(packet, linkedCancel.Token).ConfigureAwait(false);
                        await eve.WaitAsync(linkedCancel.Token);
                        break;
                    }
                    catch (TaskCanceledException e)
                    {
                        _logger.Warn(e,$"[MISSION]=> Request {currentAttept} of {attemptCount} items count error:{e.Message}");
                        if (!timeoutCancel.IsCancellationRequested)
                        {
                            throw;
                        }
                    }
                    finally
                    {
                        subscribe?.Dispose();
                    }
                }
            }
            if (result == null) throw new TimeoutException(string.Format("Timeout to request mission items with '{0}' attempts (timeout {0} times by {1:g} )", currentAttept, TimeSpan.FromMilliseconds(_config.CommandTimeoutMs)));

            _logger.Info($"[MISSION]<== Mission item count: {result.Payload.Count} items");

            return result.Payload.Count;
        }

        public async Task<MissionItemIntPayload> MissionRequestItem(ushort index, int attemptCount, CancellationToken cancel)
        {
            // MISSION_REQUEST_INT
            _logger.Trace($"[MISSION]=> Begin request mission item {index} with {attemptCount} attempts");
            var packet = new MissionRequestIntPacket
            {
                ComponenId = _identity.ComponentId,
                SystemId = _identity.SystemId,
                Sequence = _seq.GetNextSequenceNumber(),
                Payload =
                {
                    TargetComponent = _identity.TargetComponentId,
                    TargetSystem = _identity.TargetSystemId,
                }
            };
            byte currentAttept = 0;
            MissionItemIntPacket result = null;
            while (currentAttept < attemptCount)
            {
                ++currentAttept;

                using (var timeoutCancel = new CancellationTokenSource(_config.CommandTimeoutMs))
                using (var linkedCancel = CancellationTokenSource.CreateLinkedTokenSource(cancel, timeoutCancel.Token))
                {
                    IDisposable subscribe = null;
                    try
                    {
                        var eve = new AsyncAutoResetEvent(false);
                        subscribe = _mavlink.Where(FilterVehicle).Where(_ => _.MessageId == MissionItemIntPacket.PacketMessageId)
                            .Cast<MissionItemIntPacket>()
                            .Where(_=>_.Payload.Seq == index)
                            .FirstAsync()
                            .Subscribe(_ =>
                            {
                                result = _;
                                eve.Set();
                            });
                        await _mavlink.Send(packet, linkedCancel.Token).ConfigureAwait(false);
                        await eve.WaitAsync(linkedCancel.Token);
                        break;
                    }
                    catch (TaskCanceledException e)
                    {
                        _logger.Warn(e, $"[MISSION]=> Request {currentAttept} of {attemptCount} mission item {index} error:{e.Message}");
                        if (!timeoutCancel.IsCancellationRequested)
                        {
                            throw;
                        }
                    }
                    finally
                    {
                        subscribe?.Dispose();
                    }
                }
            }
            if (result == null) throw new TimeoutException(
                $"Timeout to request mission item #{index} with '{currentAttept}' attempts (timeout {currentAttept} times by {TimeSpan.FromMilliseconds(_config.CommandTimeoutMs):g} )");
            _logger.Info($"[MISSION]<== Mission item {index} recieved: {result.Payload}");
            return result.Payload;

        }

        public async Task<MissionItemIntPayload[]> MissionRequest(int attemptCount, CancellationToken cancel)
        {
            _logger.Trace($"[MISSION]=> Begin request items {attemptCount} attempts");
            var count = await MissionRequestCount(attemptCount, cancel);
            var result = new MissionItemIntPayload[count];
            for (ushort i = 0; i < count; i++)
            {
                var item = await MissionRequestItem(i, attemptCount, cancel);
                result[i] = item;
            }
            _logger.Trace($"[MISSION]=> Request items complete");
            return result;
        }

        public Task WriteMissionItem(MavFrame frame, MavCmd cmd, bool current, bool autoContinue, float param1, float param2, float param3,
            float param4, float x, float y, float z, MavMissionType missionType, int attemptCount, CancellationToken cancel)
        {
            var packet = new MissionItemPacket()
            {
                ComponenId = _identity.ComponentId,
                SystemId = _identity.SystemId,
                Sequence = _seq.GetNextSequenceNumber(),
                Payload =
                {
                    TargetComponent = _identity.TargetComponentId,
                    TargetSystem = _identity.TargetSystemId,
                    Seq = 0,
                    Frame = frame,
                    Command = cmd,
                    Current = (byte) (current ? 2:0),
                    Autocontinue = (byte) (autoContinue? 1:0),
                    Param1 = param1,
                    Param2 = param2,
                    Param3 = param3,
                    Param4 = param4,
                    X = x,
                    Y = y,
                    Z = z,
                    MissionType = missionType
                }
            };
            return _mavlink.Send(packet, cancel);
        }

        public void Dispose()
        {
            _mavlink?.Dispose();
        }
    }
}
