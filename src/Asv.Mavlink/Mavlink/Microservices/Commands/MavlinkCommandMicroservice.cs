﻿using System;
using System.Reactive.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;
using Nito.AsyncEx;

namespace Asv.Mavlink
{
    public class CommandProtocolConfig: MicroserviceConfigBase
    {
        public int CommandTimeoutMs { get; set; } = 5000;
    }

    public class MavlinkCommandMicroservice : IMavlinkCommandMicroservice,IDisposable
    {
        private readonly IMavlinkV2Connection _connection;
        private readonly CommandProtocolConfig _config;

        public MavlinkCommandMicroservice(IMavlinkV2Connection connection, CommandProtocolConfig config)
        {
            if (connection == null) throw new ArgumentNullException(nameof(connection));
            if (config == null) throw new ArgumentNullException(nameof(config));
            _connection = connection;
            _config = config;
        }


        private bool FilterVehicle(IPacketV2<IPayload> packetV2)
        {
            if (_config.TargetSystemId != 0 && _config.TargetSystemId != packetV2.SystemId) return false;
            if (_config.TargetComponenId != 0 && _config.TargetComponenId != packetV2.ComponenId) return false;
            return true;
        }

        public async Task<CommandAckPayload> CommandInt(MavCmd command, MavFrame frame, bool current, bool autocontinue, float param1, float param2,
            float param3, float param4, int x, int y, float z, int attemptCount, CancellationToken cancel)
        {
            var packet = new CommandIntPacket()
            {
                ComponenId = _config.ComponentId,
                SystemId = _config.SystemId,
                Payload =
                {
                    Command = command,
                    TargetComponent = _config.TargetComponenId,
                    TargetSystem = _config.TargetSystemId,
                    Frame = frame,
                    Param1 = param1,
                    Param2 = param2,
                    Param3 = param3,
                    Param4 = param4,
                    Current = (byte) (current ? 1:0),
                    Autocontinue = (byte) (autocontinue ? 1:0),
                    X = x,
                    Y = y,
                    Z = z,
                }
            };
            byte currentAttept = 0;
            CommandAckPacket result = null;
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
                        subscribe = _connection.Where(FilterVehicle).Where(_ => _.MessageId == CommandAckPacket.PacketMessageId)
                            .Cast<CommandAckPacket>()
                            .FirstAsync(_ => _.Payload.Command == command)
                            //   21.04.2019 comment  this filter, because work in progress https://mavlink.io/en/messages/common.html#COMMAND_ACK
                            //  .FirstAsync(_ => _.Payload.TargetComponent == _config.ComponentId &&
                            //  _.Payload.TargetSystem == _config.SystemId)
                            .Subscribe(_ =>
                                             {
                                                 result = _;
                                                 eve.Set();
                                             });
                        await _connection.Send(packet, linkedCancel.Token).ConfigureAwait(false);
                        await eve.WaitAsync(linkedCancel.Token);

                        break;
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
                }
            }
            if (result == null) throw new TimeoutException(string.Format("Timeout to execute command '{0:G}' with '{1}' attempts (timeout {1} times by {2:g} )", command, currentAttept, TimeSpan.FromMilliseconds(_config.CommandTimeoutMs)));
            return result.Payload;
        }

        public async Task<CommandAckPayload> CommandLong(MavCmd command, float param1, float param2, float param3, float param4, float param5, float param6, float param7, int attemptCount, CancellationToken cancel)
        {
            var packet = new CommandLongPacket
            {
                ComponenId = _config.ComponentId,
                SystemId = _config.SystemId,
                Payload =
                {
                    Command = command,
                    TargetComponent = _config.TargetComponenId,
                    TargetSystem = _config.TargetSystemId,
                    Confirmation = 0,
                    Param1 = param1,
                    Param2 = param2,
                    Param3 = param3,
                    Param4 = param4,
                    Param5 = param5,
                    Param6 = param6,
                    Param7 = param7,
                }
            };
            byte currentAttept = 0;
            CommandAckPacket result = null;
            while (currentAttept < attemptCount)
            {
                packet.Payload.Confirmation = currentAttept;
                ++currentAttept;

                using (var timeoutCancel = new CancellationTokenSource(_config.CommandTimeoutMs))
                using (var linkedCancel = CancellationTokenSource.CreateLinkedTokenSource(cancel, timeoutCancel.Token))
                {
                    IDisposable subscribe = null;
                    try
                    {
                        var eve = new AsyncAutoResetEvent(false);
                        subscribe = _connection.Where(FilterVehicle).Where(_ => _.MessageId == CommandAckPacket.PacketMessageId)
                            .Cast<CommandAckPacket>()
                            .FirstAsync(_=>_.Payload.Command == command)
                            //   21.04.2019 comment  this filter, because work in progress https://mavlink.io/en/messages/common.html#COMMAND_ACK
                            //  .FirstAsync(_ => _.Payload.TargetComponent == _config.ComponentId &&
                            //  _.Payload.TargetSystem == _config.SystemId)
                            .Subscribe(_ =>
                                             {
                                                 result = _;
                                                 eve.Set();
                                             });
                        await _connection.Send(packet, linkedCancel.Token).ConfigureAwait(false);
                        await eve.WaitAsync(linkedCancel.Token);

                        break;
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
                }
            }
            if (result == null) throw new TimeoutException(string.Format("Timeout to execute command '{0:G}' with '{1}' attempts (timeout {1} times by {2:g} )", command, currentAttept, TimeSpan.FromMilliseconds(_config.CommandTimeoutMs)));
            return result.Payload;
        }

        public void Dispose()
        {
            
        }
    }
}