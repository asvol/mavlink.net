using System;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink.Server
{
    public interface IStatusTextLogger:IDisposable
    {
        bool Log(MavSeverity severity, string message);
    }

    public static class StatusTextLoggerHelper
    {
        public static bool Alert(this IStatusTextLogger src, string message)
        {
            return src.Log(MavSeverity.MavSeverityAlert, message);
        }
        public static bool Critical(this IStatusTextLogger src, string message)
        {
            return src.Log(MavSeverity.MavSeverityCritical, message);
        }
        public static bool Debug(this IStatusTextLogger src, string message)
        {
            return src.Log(MavSeverity.MavSeverityDebug, message);
        }
        public static bool Emergency(this IStatusTextLogger src, string message)
        {
            return src.Log(MavSeverity.MavSeverityEmergency, message);
        }
        public static bool Error(this IStatusTextLogger src, string message)
        {
            return src.Log(MavSeverity.MavSeverityError, message);
        }
        public static bool Info(this IStatusTextLogger src, string message)
        {
            return src.Log(MavSeverity.MavSeverityInfo, message);
        }
        public static bool Notice(this IStatusTextLogger src, string message)
        {
            return src.Log(MavSeverity.MavSeverityNotice, message);
        }
        public static bool Warning(this IStatusTextLogger src, string message)
        {
            return src.Log(MavSeverity.MavSeverityWarning, message);
        }
    }
}