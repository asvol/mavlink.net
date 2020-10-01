using System;
using System.IO;
using NLog;

namespace Asv.Mavlink
{
    public static class PlatformHelper
    {
        private static readonly Logger _logger = LogManager.GetCurrentClassLogger();

        public enum OperatingSystem
        {
            Undefined,
            Windows,
            Linux,
            MacOsX
        }

        private static OperatingSystem DetectPlatform()
        {
            var windir = Environment.GetEnvironmentVariable("windir");
            if (!string.IsNullOrEmpty(windir) && windir.Contains(@"\") && Directory.Exists(windir)) return OperatingSystem.Windows;

            if (File.Exists(@"/proc/sys/kernel/ostype"))
            {
                var osType = File.ReadAllText(@"/proc/sys/kernel/ostype");
                return osType.StartsWith("Linux", StringComparison.OrdinalIgnoreCase)
                    ? OperatingSystem.Linux
                    : OperatingSystem.Undefined;
            }

            return File.Exists(@"/System/Library/CoreServices/SystemVersion.plist")
                ? OperatingSystem.MacOsX
                : OperatingSystem.Undefined;
        }

      
    }
}
