using System;
using System.Collections.Generic;
using System.Linq;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Runtime.CompilerServices;
using System.Threading;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public interface IMavlinkNamedValues:IDisposable
    {
        IObservable<KeyValuePair<string,float>> OnFloatValue { get; }
        IObservable<KeyValuePair<string, int>> OnIntValue { get; }
    }

    public class NamedValueClient: IMavlinkNamedValues
    {
        private readonly MavlinkClientIdentity _identity;
        private readonly Subject<KeyValuePair<string, float>> _onFloatSubject = new Subject<KeyValuePair<string, float>>();
        private readonly Subject<KeyValuePair<string, int>> _onIntSubject = new Subject<KeyValuePair<string, int>>();
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();

        public NamedValueClient(IMavlinkV2Connection connection, MavlinkClientIdentity identity)
        {
            _identity = identity;
            var inputPackets = connection.Where(FilterVehicle);
            inputPackets
                .Where(_ => _.MessageId == NamedValueFloatPacket.PacketMessageId)
                .Cast<NamedValueFloatPacket>()
                .Select(_ => new KeyValuePair<string, float>(ConvertToKey(_.Payload.Name), _.Payload.Value))
                .Subscribe(_onFloatSubject,_disposeCancel.Token);

            inputPackets
                .Where(_ => _.MessageId == NamedValueIntPacket.PacketMessageId)
                .Cast<NamedValueIntPacket>()
                .Select(_ => new KeyValuePair<string, int>(ConvertToKey(_.Payload.Name), _.Payload.Value))
                .Subscribe(_onIntSubject, _disposeCancel.Token);
        }

        private string ConvertToKey(char[] payloadName)
        {
            return new string(payloadName.Where(_=>_ != 0).ToArray());
        }

        private bool FilterVehicle(IPacketV2<IPayload> packetV2)
        {
            if (_identity.TargetSystemId != 0 && _identity.TargetSystemId != packetV2.SystemId) return false;
            if (_identity.TargetComponentId != 0 && _identity.TargetComponentId != packetV2.ComponenId) return false;
            return true;
        }

        public IObservable<KeyValuePair<string, float>> OnFloatValue =>_onFloatSubject;
        public IObservable<KeyValuePair<string, int>> OnIntValue => _onIntSubject;

        public void Dispose()
        {
            _disposeCancel?.Cancel(false);
            _disposeCancel?.Dispose();
            _onFloatSubject?.Dispose();
            _onIntSubject?.Dispose();
        }
    }
}