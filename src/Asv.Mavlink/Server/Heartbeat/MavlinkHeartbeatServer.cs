﻿using System;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink.Server
{
    public class MavlinkHeartbeatServerConfig
    {
        public int HeartbeatRateMs { get; set; } = 1000;
    }

    public class MavlinkHeartbeatServer: IMavlinkHeartbeatServer,IDisposable
    {
        private readonly MavlinkHeartbeatServerConfig _config;
        private readonly MavlinkPacketTransponder<HeartbeatPacket, HeartbeatPayload> _transponder;
        public MavlinkHeartbeatServer(IMavlinkV2Connection connection, IPacketSequenceCalculator seq, MavlinkServerIdentity identity, MavlinkHeartbeatServerConfig config)
        {
            _config = config;
            _transponder = new MavlinkPacketTransponder<HeartbeatPacket,HeartbeatPayload>(connection, identity, seq);
        }

        public void Set(Action<HeartbeatPayload> changeCallback)
        {
            _transponder.Set(changeCallback);
        }

        public void Start()
        {
            _transponder.Start(TimeSpan.FromMilliseconds(_config.HeartbeatRateMs));
        }


        public void Dispose()
        {
            _transponder.Dispose();
        }
    }
}