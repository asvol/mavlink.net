using System;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink.Server
{
    public interface INamedValueServer:IDisposable
    {
        Task SendFloat(string name, float value);
        Task SendInteger(string name, int value);
    }

    public class NamedValueServer:INamedValueServer
    {
        private readonly int MaxKeyLength = new NamedValueFloatPayload().Name.Length;
        private readonly IMavlinkV2Connection _connection;
        private readonly IPacketSequenceCalculator _seq;
        private readonly MavlinkServerIdentity _identity;
        private readonly DateTime _bootTime;
        private readonly CancellationTokenSource _disposableCancel = new CancellationTokenSource();

        public NamedValueServer(IMavlinkV2Connection connection,IPacketSequenceCalculator seq,MavlinkServerIdentity identity)
        {
            _connection = connection;
            _seq = seq;
            _identity = identity;
            _bootTime = DateTime.Now;
        }

        public Task SendFloat(string name, float value)
        {
            if (name.Length > MaxKeyLength)
            {
                throw new ArgumentException($"Name '{name}' is too long for parameter name (max size {MaxKeyLength})",nameof(name));
            }

            var packet = new NamedValueFloatPacket
            {
                ComponenId = _identity.ComponenId,
                SystemId = _identity.SystemId,
                CompatFlags = 0,
                IncompatFlags = 0,
                Sequence = _seq.GetNextSequenceNumber(),
                Payload =
                {
                    Name = name.ToCharArray(),
                    TimeBootMs = (uint)(DateTime.Now - _bootTime).TotalMilliseconds,
                    Value = value,
                }
            };
            return _connection.Send(packet, _disposableCancel.Token);
        }

        public Task SendInteger(string name, int value)
        {
            if (name.Length > MaxKeyLength)
            {
                throw new ArgumentException($"Name '{name}' is too long for parameter name (max size {MaxKeyLength})", nameof(name));
            }

            var packet = new NamedValueIntPacket()
            {
                ComponenId = _identity.ComponenId,
                SystemId = _identity.SystemId,
                CompatFlags = 0,
                IncompatFlags = 0,
                Sequence = _seq.GetNextSequenceNumber(),
                Payload =
                {
                    Name = name.ToCharArray(),
                    TimeBootMs = (uint)(DateTime.Now - _bootTime).TotalMilliseconds,
                    Value = value,
                }
            };
            return _connection.Send(packet, _disposableCancel.Token);
        }

        public void Dispose()
        {
            _disposableCancel?.Cancel(false);
            _disposableCancel?.Dispose();
        }
    }

    
}
