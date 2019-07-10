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


    public delegate Task<CommandLongResult> CommandLongDelegate(float param1, float param2, float param3, float param4, float param5, float param6, float param7, CancellationToken cancel);

    public interface ICommandLongServer:IDisposable
    {
        CommandLongDelegate this[MavCmd cmd] { set; }
    }
}