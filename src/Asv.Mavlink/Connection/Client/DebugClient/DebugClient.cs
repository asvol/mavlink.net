﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Threading;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink.Client
{
    public class DebugClient: IDebugClient
    {
        private readonly MavlinkClientIdentity _identity;
        private readonly Subject<KeyValuePair<string, float>> _onFloatSubject = new Subject<KeyValuePair<string, float>>();
        private readonly Subject<KeyValuePair<string, int>> _onIntSubject = new Subject<KeyValuePair<string, int>>();
        private readonly RxValue<DebugFloatArrayPayload> _debugFloatArray = new RxValue<DebugFloatArrayPayload>();
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();

        public DebugClient(IMavlinkV2Connection connection, MavlinkClientIdentity identity)
        {
            _identity = identity;
            var inputPackets = connection.FilterVehicle(identity);
            inputPackets
                .Where(_ => _.MessageId == NamedValueFloatPacket.PacketMessageId)
                .Cast<NamedValueFloatPacket>()
                .Select(_ => new KeyValuePair<string, float>(ConvertToKey(_.Payload.Name), _.Payload.Value))
                .Subscribe(_onFloatSubject,_disposeCancel.Token);

            _disposeCancel.Token.Register(() =>
            {
                _onFloatSubject.OnCompleted();
                _onFloatSubject.Dispose();
            });

            inputPackets
                .Where(_ => _.MessageId == NamedValueIntPacket.PacketMessageId)
                .Cast<NamedValueIntPacket>()
                .Select(_ => new KeyValuePair<string, int>(ConvertToKey(_.Payload.Name), _.Payload.Value))
                .Subscribe(_onIntSubject, _disposeCancel.Token);

            _disposeCancel.Token.Register(() =>
            {
                _onIntSubject.OnCompleted();
                _onIntSubject.Dispose();
            });

            inputPackets
                .Where(_ => _.MessageId == DebugFloatArrayPacket.PacketMessageId)
                .Cast<DebugFloatArrayPacket>()
                .Select(_ => _.Payload)
                .Subscribe(_debugFloatArray, _disposeCancel.Token);
            _disposeCancel.Token.Register(() => _debugFloatArray.Dispose());
        }

        private string ConvertToKey(char[] payloadName)
        {
            return new string(payloadName.Where(_=>_ != 0).ToArray());
        }

        public IObservable<KeyValuePair<string, float>> NamedFloatValue =>_onFloatSubject;
        public IObservable<KeyValuePair<string, int>> NamedIntValue => _onIntSubject;
        public IRxValue<DebugFloatArrayPayload> DebugFloatArray => _debugFloatArray;

        public void Dispose()
        {
            _disposeCancel.Cancel(false);
            _disposeCancel.Dispose();

            
        }
    }
}