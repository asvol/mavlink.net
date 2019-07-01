using System;
using System.Collections.Generic;
using System.Reactive.Subjects;

namespace Asv.Mavlink.Decoder
{
    public class PacketV2Decoder : IPacketDecoder<IPacketV2<IPayload>>
    {
        private readonly byte[] _buffer = new byte[PacketV2Helper.PacketV2MaxSize];
        private DecodeStep _decodeStep;
        private int _bufferIndex;
        private int _bufferStopIndex;
        private readonly Dictionary<int, Func<IPacketV2<IPayload>>> _dict = new Dictionary<int, Func<IPacketV2<IPayload>>>();
        private readonly Subject<DeserializePackageException> _decodeErrorSubject = new Subject<DeserializePackageException>();
        private readonly Subject<IPacketV2<IPayload>> _packetSubject = new Subject<IPacketV2<IPayload>>();

        private enum DecodeStep
        {
            Sync,
            Length,
            FillBuffer,
            FillSignature
        }

        public PacketV2Decoder()
        {
            // the first byte is always STX
            _buffer[0] = PacketV2Helper.MagicMarkerV2;
        }

        public virtual void OnNext(byte value)
        {
            switch (_decodeStep)
            {
                case DecodeStep.Sync:
                    _decodeStep = SyncStep(value);
                    break;
                case DecodeStep.Length:
                    _decodeStep = GetLengthStep(value);
                    break;
                case DecodeStep.FillBuffer:
                    _decodeStep = FillBufferStep(value);
                    break;
                case DecodeStep.FillSignature:
                    _decodeStep = SignatureStep(value);
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        public void OnError(Exception error)
        {
            _packetSubject.OnError(error);
            _decodeErrorSubject.OnError(error);
        }

        public void OnCompleted()
        {
            _packetSubject.OnCompleted();
            _decodeErrorSubject.OnCompleted();
            Dispose();
        }

        private DecodeStep SignatureStep(byte value)
        {
            _buffer[_bufferIndex] = value;
            if (_bufferIndex <= (_bufferStopIndex + PacketV2Helper.SignatureByteSize))
            {
                TryDecodePacket();
                return DecodeStep.Sync;
            }
            return DecodeStep.FillSignature;
        }

        private DecodeStep SyncStep(byte value)
        {
            // if found STX => next step is geting length
            return value != PacketV2Helper.MagicMarkerV2 ? DecodeStep.Sync : DecodeStep.Length;
        }

        private DecodeStep GetLengthStep(byte value)
        {
            _buffer[1] = value;

            _bufferStopIndex = PacketV2Helper.PaylodStartIndexInFrame + /*payload length*/value + 2 /*crc*/;
            _bufferIndex = 2;
            return DecodeStep.FillBuffer;
        }

        private DecodeStep FillBufferStep(byte value)
        {
            _buffer[_bufferIndex] = value;
            ++_bufferIndex;
            if (_bufferIndex < _bufferStopIndex) return DecodeStep.FillBuffer;
            
            // buffer with frames,payload and checksumm filled => check if signature present
            // https://mavlink.io/en/guide/message_signing.html
            if (PacketV2Helper.CheckSignaturePresent(_buffer,0))
            {
                return DecodeStep.FillSignature;
            }
            // packet without sync
            TryDecodePacket();
            return DecodeStep.Sync;
        }

        private IPacketV2<IPayload> CreatePacket(int messageId)
        {
            Func<IPacketV2<IPayload>> func;
            return _dict.TryGetValue(messageId, out func) ? func() : null;
        }

        private void TryDecodePacket()
        {
            var messageId = PacketV2Helper.GetMessageId(_buffer,0);

            var packet = CreatePacket(messageId);
            if (packet == null)
            {
                _decodeErrorSubject.OnNext(new MessageIdNotFoundException(messageId));
                return;
            }
            try
            {
                packet.Deserialize(_buffer, 0);
            }
            catch (Exception exception)
            {
                _decodeErrorSubject.OnNext(new DeserializePackageException(messageId, string.Format(RS.DecoderV2_TryDecodePacket_Error_for_deserialize_mavlink_V2, messageId,exception.Message) , exception) );
            }
            _packetSubject.OnNext(packet);
        }



        public IObservable<DeserializePackageException> OutError => _decodeErrorSubject;

        public void Register(Func<IPacketV2<IPayload>> factory)
        {
            var testPckt = factory();
            _dict.Add(testPckt.MessageId,factory);
        }

        public IDisposable Subscribe(IObserver<IPacketV2<IPayload>> observer)
        {
            return _packetSubject.Subscribe(observer);
        }

        public virtual void Dispose()
        {
            _decodeErrorSubject?.Dispose();
            _packetSubject?.Dispose();
        }
    }
}