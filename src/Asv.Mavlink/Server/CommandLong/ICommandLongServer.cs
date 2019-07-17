using System;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink.Server
{
    public class CommandLongResult
    {
        public CommandLongResult(MavResult resultCode, int resultValue = 0)
        {
            ResultCode = resultCode;
            ResultValue = resultValue;
        }

        public MavResult ResultCode { get; }
        public int ResultValue { get; }
    }

    public class DeviceIdentity
    {
        public byte SystemId { get; set; }
        public byte ComponentId { get; set; }
    }


    public class CommandArgs
    {
        public DeviceIdentity Subject { get; set; }
        public float Param1 { get; set; }
        public float Param2 { get; set; }
        public float Param3 { get; set; }
        public float Param4 { get; set; }
        public float Param5 { get; set; }
        public float Param6 { get; set; }
        public float Param7 { get; set; }
    }

    public delegate Task<CommandLongResult> CommandLongDelegate(CommandArgs args, CancellationToken cancel);

    public interface ICommandLongServer:IDisposable
    {
        CommandLongDelegate this[MavCmd cmd] { set; }
    }
}