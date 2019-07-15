using System;
using System.Collections.Generic;
using System.Reactive.Linq;
using System.Threading;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink.Client
{
    public class LoggingClient:ILoggingClient
    {
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        private readonly RxValue<LoggingDataPayload> _loggingData = new RxValue<LoggingDataPayload>();

        public LoggingClient(IMavlinkV2Connection connection, MavlinkClientIdentity identity)
        {
            if (connection == null) throw new ArgumentNullException(nameof(connection));
            if (identity == null) throw new ArgumentNullException(nameof(identity));

            connection
                .FilterVehicle(identity)
                .Where(_ => _.MessageId == LoggingDataPacket.PacketMessageId)
                .Cast<LoggingDataPacket>()
                .Where(_=>_.Payload.TargetSystem == identity.SystemId && _.Payload.TargetComponent == identity.ComponentId)
                .Select(_=>_.Payload)
                .Subscribe(_loggingData, _disposeCancel.Token);

            _disposeCancel.Token.Register(() => _loggingData.Dispose());
        }

        public IRxValue<LoggingDataPayload> RawLoggingData => _loggingData;
    

        public void Dispose()
        {
            _disposeCancel?.Cancel(false);
            _disposeCancel?.Dispose();
        }
    }
}
