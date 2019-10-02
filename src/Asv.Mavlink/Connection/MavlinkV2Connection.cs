﻿using System;
using System.Reactive.Concurrency;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Decoder;

namespace Asv.Mavlink
{
    public class MavlinkV2Connection : IMavlinkV2Connection
    {
        private readonly PacketV2Decoder _decoder = new PacketV2Decoder();
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        private int _disposed;
        private long _txPackets;
        private long _rxPackets;
        private long _skipPackets;
        private readonly Subject<IPacketV2<IPayload>> _sendPacketSubject = new Subject<IPacketV2<IPayload>>();

        public MavlinkV2Connection(string connectionString, Action<IPacketDecoder<IPacketV2<IPayload>>> register):this(ConnectionStringConvert(connectionString),register)
        {

        }

        private static IDataStream ConnectionStringConvert(string connString)
        {
            var p =PortFactory.Create(connString);
            p.Enable();
            return p;
        }

        public MavlinkV2Connection(IDataStream dataStream, Action<IPacketDecoder<IPacketV2<IPayload>>> register)
        {
            DataStream = dataStream;
            register(_decoder);
            DataStream.SelectMany(_ => _).Subscribe(_decoder, _disposeCancel.Token);
            _decoder.Subscribe(_ => Interlocked.Increment(ref _rxPackets), _disposeCancel.Token);
            _decoder.OutError.Subscribe(_ => Interlocked.Increment(ref _skipPackets), _disposeCancel.Token);
        }

        public void Dispose()
        {
            if (Interlocked.CompareExchange(ref _disposed, 1,0) == 1) return;
            _sendPacketSubject?.Dispose();
            _disposeCancel?.Cancel(false);
            _disposeCancel?.Dispose();
            _decoder.Dispose();
        }

        public long RxPackets => Interlocked.Read(ref _rxPackets);
        public long TxPackets => Interlocked.Read(ref _txPackets);
        public long SkipPackets => Interlocked.Read(ref _skipPackets);
        public IObservable<DeserializePackageException> DeserializePackageErrors => _decoder.OutError;
        public IObservable<IPacketV2<IPayload>> OnSendPacket => _sendPacketSubject;
        public IDataStream DataStream { get; }

        public Task Send(IPacketV2<IPayload> packet, CancellationToken cancel)
        {
            Interlocked.Increment(ref _txPackets);
            var buffer = new byte[packet.GetMaxByteSize()];
            var size = packet.Serialize(buffer, 0);
            _sendPacketSubject.OnNext(packet);
            return DataStream.Send(buffer,size, cancel);
        }

        public IDisposable Subscribe(IObserver<IPacketV2<IPayload>> observer)
        {
            return _decoder.Subscribe(observer);
        }
    }
}
