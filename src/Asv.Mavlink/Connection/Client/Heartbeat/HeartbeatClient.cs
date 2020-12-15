using System;
using System.Collections.Concurrent;
using System.Linq;
using System.Reactive.Linq;
using System.Threading;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink.Client
{
    public class HeartbeatClient : IHeartbeatClient
    {
        private readonly int _heartBeatTimeoutMs;
        private readonly RxValue<HeartbeatPayload> _heartBeat = new RxValue<HeartbeatPayload>();
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        private readonly RxValue<int> _packetRate = new RxValue<int>();
        private readonly RxValue<double> _linkQuality = new RxValue<double>();
        private readonly LinkIndicator _link = new LinkIndicator(3);
        private readonly ConcurrentQueue<byte> _sequence = new ConcurrentQueue<byte>(new []{ (byte)1, (byte)2, (byte)3, (byte)4, (byte)5, (byte)6, (byte)7, (byte)8, (byte)9, (byte)10 });

        private DateTime _lastHeartbeat;

        public HeartbeatClient(IMavlinkV2Connection connection, MavlinkClientIdentity config, int heartBeatTimeoutMs = 2000)
        {
            _heartBeatTimeoutMs = heartBeatTimeoutMs;
            connection
                .FilterVehicle(config)
                .Select(_ => _.Sequence)
                .Subscribe(_ =>
                {
                    while (_sequence.TryDequeue(out var val) == false) { }
                    _sequence.Enqueue(_);
                }, _disposeCancel.Token);


            connection
                .FilterVehicle(config)
                .Where(_ => _.MessageId == HeartbeatPacket.PacketMessageId)
                .Cast<HeartbeatPacket>()
                .Select(_ => _.Payload)
                .Subscribe(_heartBeat);
            _disposeCancel.Token.Register(() => _heartBeat.Dispose());

            connection
                .FilterVehicle(config)
                .Select(_ => 1)
                .Buffer(TimeSpan.FromSeconds(1))
                .Select(_ => _.Sum()).Subscribe(_packetRate, _disposeCancel.Token);
            _disposeCancel.Token.Register(() => _packetRate.Dispose());

            Observable.Timer(TimeSpan.FromSeconds(1), TimeSpan.FromSeconds(1)).Subscribe(CheckConnection, _disposeCancel.Token);
            RawHeartbeat.Subscribe(_ =>
            {
                if (_disposeCancel.IsCancellationRequested) return;
                _lastHeartbeat = DateTime.Now;
                var last = _sequence.Last();
                var first = _sequence.First();
                var seq = last - first + 1;
                if (seq < 0) seq = last + byte.MaxValue - first + 1;
                _linkQuality.OnNext(((double)_sequence.Count)/ seq);
                _link.Upgrade();
            }, _disposeCancel.Token);
            _disposeCancel.Token.Register(() => _link.Dispose());
        }

        public IRxValue<HeartbeatPayload> RawHeartbeat => _heartBeat;
        public IRxValue<int> PacketRateHz => _packetRate;
        public IRxValue<double> LinkQuality => _linkQuality;
        public IRxValue<LinkState> Link => _link;

        private void CheckConnection(long value)
        {
            if (DateTime.Now - _lastHeartbeat > TimeSpan.FromMilliseconds(_heartBeatTimeoutMs))
            {
                _link.Downgrade();
            }
        }

        public void Dispose()
        {
            _disposeCancel?.Cancel(false);
            _disposeCancel?.Dispose();
        }
    }
}
