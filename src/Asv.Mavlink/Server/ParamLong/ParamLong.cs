using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Reactive.Concurrency;
using System.Reactive.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;
using NLog;

namespace Asv.Mavlink.Server
{
    public delegate Task<MavResult> CommandLongDelegate(float param1, float param2, float param3, float param4, float param5, float param6, float param7,CancellationToken cancel);

    public interface ICommandLongServer:IDisposable
    {
        CommandLongDelegate this[MavCmd cmd] { set; }
    }

    
    

    public class CommandLongServer: ICommandLongServer
    {
        private readonly IMavlinkV2Connection _connection;
        private readonly IPacketSequenceCalculator _seq;
        private readonly MavlinkServerIdentity _identity;
        private readonly ConcurrentDictionary<MavCmd, CommandLongDelegate> _registry = new ConcurrentDictionary<MavCmd, CommandLongDelegate>();
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();
        private int _isBusy;

        public CommandLongServer(IMavlinkV2Connection connection, IPacketSequenceCalculator seq,MavlinkServerIdentity identity)
        {
            _connection = connection;
            _seq = seq;
            _identity = identity;
            connection
                .Where(_ => _.MessageId == CommandLongPacket.PacketMessageId)
                .Cast<CommandLongPacket>()
                .Where(_ => _.Payload.TargetComponent == identity.ComponenId && _.Payload.TargetSystem == identity.SystemId)
                .ObserveOn(TaskPoolScheduler.Default)
                .Subscribe(OnRequest, _disposeCancel.Token);
        }

        private async void OnRequest(CommandLongPacket obj)
        {
            // wait until prev been executed
            if (Interlocked.CompareExchange(ref _isBusy, 1, 0) == 1 && obj.Payload.Confirmation == 0)
            {
                _logger.Warn($"Reject command {obj.Payload.Command}(Param1:{obj.Payload.Param1},Param2:{obj.Payload.Param2},Param3:{obj.Payload.Param3},Param4:{obj.Payload.Param4},Param5:{obj.Payload.Param5},Param6:{obj.Payload.Param6},Param7:{obj.Payload.Param7}): too busy now");
                SafeSendCommandAck(obj.Payload.Command, MavResult.MavResultTemporarilyRejected, obj.SystemId, obj.ComponenId);
                return;
            }
            CommandLongDelegate callback;
            if (_registry.TryGetValue(obj.Payload.Command, out callback) == false)
            {
                _logger.Warn($"Reject unknown command {obj.Payload.Command}(Param1:{obj.Payload.Param1},Param2:{obj.Payload.Param2},Param3:{obj.Payload.Param3},Param4:{obj.Payload.Param4},Param5:{obj.Payload.Param5},Param6:{obj.Payload.Param6},Param7:{obj.Payload.Param7})");
                SafeSendCommandAck(obj.Payload.Command, MavResult.MavResultUnsupported, obj.SystemId, obj.ComponenId);
                return;
            }

            try
            {
                _logger.Info($"Command {obj.Payload.Command}(Param1:{obj.Payload.Param1},Param2:{obj.Payload.Param2},Param3:{obj.Payload.Param3},Param4:{obj.Payload.Param4},Param5:{obj.Payload.Param5},Param6:{obj.Payload.Param6},Param7:{obj.Payload.Param7})");

                var result = await callback(obj.Payload.Param1, obj.Payload.Param2, obj.Payload.Param3,
                    obj.Payload.Param4, obj.Payload.Param5, obj.Payload.Param6, obj.Payload.Param7,_disposeCancel.Token);
                SafeSendCommandAck(obj.Payload.Command, result, obj.SystemId, obj.ComponenId);
                
            }
            catch (Exception e)
            {
                _logger.Error(
                    $"Error to execute command {obj.Payload.Command}(Param1:{obj.Payload.Param1},Param2:{obj.Payload.Param2},Param3:{obj.Payload.Param3},Param4:{obj.Payload.Param4},Param5:{obj.Payload.Param5},Param6:{obj.Payload.Param6},Param7:{obj.Payload.Param7}):{e.Message}");
                SafeSendCommandAck(obj.Payload.Command, MavResult.MavResultFailed, obj.SystemId, obj.ComponenId);
                
            }
            finally
            {
                Interlocked.Exchange(ref _isBusy, 1);
            }
        }

        private void SafeSendCommandAck(MavCmd cmd, MavResult result, byte systemId, byte componenId)
        {
            try
            {
                _connection.Send(new CommandAckPacket
                {
                    ComponenId = _identity.ComponenId,
                    SystemId = _identity.SystemId,
                    Sequence = _seq.GetNextSequenceNumber(),
                    CompatFlags = 0,
                    IncompatFlags = 0,
                    Payload =
                    {
                        Command = cmd,
                        Result = result,
                        TargetSystem = systemId,
                        TargetComponent = componenId
                    }
                }, _disposeCancel.Token);
            }
            catch (Exception e)
            {
                _logger.Error(e, string.Format("Error to send CommandAckPacket. Command: {0:G}. Result: {1:G}. TargetSystemId: {2}. TargetComponentId: {3}. {4}", cmd, result, systemId, componenId, e.Message));
            }
        }

        public CommandLongDelegate this[MavCmd cmd]
        {
            set { _registry.AddOrUpdate(cmd, value, (mavCmd, del) =>value); }
        }


        public void Dispose()
        {
            _disposeCancel?.Cancel(false);
            _disposeCancel?.Dispose();
        }
    }
}
