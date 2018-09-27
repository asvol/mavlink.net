﻿using System;
using System.Reactive.Linq;
using System.Reactive.Threading.Tasks;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public interface IMavlinkCommandProtocol:IDisposable
    {
        
    }

    public class MavlinkCommandProtocol : IMavlinkCommandProtocol
    {
        private readonly IObservable<IPacketV2<IPayload>> _inputPackets;
        private readonly IObserver<IPacketV2<IPayload>> _outputPackets;

        public MavlinkCommandProtocol(IObservable<IPacketV2<IPayload>> inputPackets, IObserver<IPacketV2<IPayload>> outputPackets)
        {
            _inputPackets = inputPackets;
            _outputPackets = outputPackets;
        }

        public Task Execute(byte sequence, byte systemId, byte componentId,MavCmd command, byte targetComponent, CancellationToken cancel, float param1, float param2, float param3, float param4, float param5, float param6)
        {
            return Task.Factory.StartNew(() =>
            {
                var c = new ManualResetEventSlim();
                var packet = new CommandLongPacket
                {
                    ComponenId = componentId,
                    SystemId = systemId,
                    Sequence = sequence
                };
                packet.Payload.Command = command;
                packet.Payload.Confirmation = 0;
                packet.Payload.TargetComponent = targetComponent;
                packet.Payload.Param1 = param1;
                packet.Payload.Param2 = param2;
                packet.Payload.Param3 = param3;
                packet.Payload.Param4 = param4;
                packet.Payload.Param5 = param5;
                packet.Payload.Param6 = param6;
                CommandAckPacket pck;
                _inputPackets
                    .Where(_ => _.MessageId == CommandAckPacket.PacketMessageId)
                    .Cast<CommandAckPacket>()
                    .Where(_ => _.Payload.Command == command).Subscribe(_ =>
                    {
                        pck = _;
                        c.Set();
                    });
                _outputPackets.OnNext(packet);
                c.Wait(cancel);
                
            }, cancel);
            
        }

        public void Dispose()
        {
        }
    }
}
