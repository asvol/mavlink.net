using System;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public class GpsInfo
    {
        private static readonly DateTime Epoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

        public GpsInfo(GpsRawIntPayload rawGps)
        {
            
            Vdop = rawGps.Epv == ushort.MaxValue ? (double?)null : rawGps.Epv / 100D;
            Hdop = rawGps.Eph == ushort.MaxValue ? (double?)null : rawGps.Eph / 100D;
            Pdop = Hdop.HasValue && Vdop.HasValue
                ? Math.Sqrt(Hdop.Value * Hdop.Value + Vdop.Value * Vdop.Value)
                : default(double?);
            AltitudeMsl = rawGps.Alt / 1000D;
            AltitudeEllipsoid = rawGps.AltEllipsoid / 1000D;
            CourseOverGround = rawGps.Cog / 100D;
            FixType = rawGps.FixType;
            SatellitesVisible = rawGps.SatellitesVisible;
            Time = Epoch.AddSeconds(rawGps.TimeUsec);
        }

        public GpsInfo(Gps2RawPayload rawGps)
        {
            Vdop = rawGps.Epv == ushort.MaxValue ? (double?)null : rawGps.Epv / 100D;
            Hdop = rawGps.Eph == ushort.MaxValue ? (double?)null : rawGps.Eph / 100D;
            Pdop = Hdop.HasValue && Vdop.HasValue
                ? Math.Sqrt(Hdop.Value * Hdop.Value + Vdop.Value * Vdop.Value)
                : default(double?);
            AltitudeMsl = rawGps.Alt / 1000D;
            
            CourseOverGround = rawGps.Cog / 100D;
            FixType = rawGps.FixType;
            SatellitesVisible = rawGps.SatellitesVisible;
            Time = Epoch.AddSeconds(rawGps.TimeUsec);
        }

        /// <summary>
        /// 
        /// </summary>
        public double CourseOverGround { get; set; }

        /// <summary>
        /// Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
        /// </summary>
        public double? AltitudeEllipsoid { get; }

        /// <summary>
        /// Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
        /// </summary>
        public double AltitudeMsl { get; }

        public GpsFixType FixType { get; }
        public int SatellitesVisible { get; }
        public DateTime Time { get; }
        /// <summary>
        /// HDOP – horizontal dilution of precision
        /// </summary>
        public double? Hdop { get; }
        public DopStatusEnum HdopStatus => GpsInfoHelper.GetDopStatus(Hdop);

        /// <summary>
        /// position (3D) dilution of precision
        /// </summary>
        public double? Pdop { get; }
        public DopStatusEnum PdopStatus => GpsInfoHelper.GetDopStatus(Pdop);

        /// <summary>
        ///  vertical dilution of precision
        /// </summary>
        public double? Vdop { get; }
        public DopStatusEnum VdopStatus => GpsInfoHelper.GetDopStatus(Vdop);
    }

    /// <summary>
    /// https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
    /// </summary>
    public enum DopStatusEnum
    {
        Unknown,
        Ideal,
        Excellent,
        Good,
        Moderate,
        Fair,
        Poor,
    }

    public static class GpsInfoHelper
    {
        public static DopStatusEnum GetDopStatus(double? value)
        {
            if (!value.HasValue) return DopStatusEnum.Unknown;
            if (value <= 1) return DopStatusEnum.Ideal;
            if (value <= 2 ) return DopStatusEnum.Excellent;
            if (value <= 5) return DopStatusEnum.Good;
            if (value <= 10) return DopStatusEnum.Moderate;
            if (value <= 20) return DopStatusEnum.Fair;
            return DopStatusEnum.Poor;
        }

        public static string GetDescription(this DopStatusEnum src)
        {
            switch (src)
            {
                case DopStatusEnum.Ideal:
                    return RS.GpsInfoHelper_GetDescription_Ideal;
                case DopStatusEnum.Excellent:
                    return RS.GpsInfoHelper_GetDescription_Excellent;
                case DopStatusEnum.Good:
                    return RS.GpsInfoHelper_GetDescription_Good;
                case DopStatusEnum.Moderate:
                    return RS.GpsInfoHelper_GetDescription_Moderate;
                case DopStatusEnum.Fair:
                    return RS.GpsInfoHelper_GetDescription_Fair;
                case DopStatusEnum.Poor:
                    return RS.GpsInfoHelper_GetDescription_Poor;
                case DopStatusEnum.Unknown:
                default:
                    return RS.GpsInfoHelper_GetDescription_Unknown;
            }
        }

        public static string GetDisplayName(this DopStatusEnum src)
        {
            switch (src)
            {
                case DopStatusEnum.Ideal:
                    return RS.GpsInfoHelper_GetDisplayName_Ideal;
                case DopStatusEnum.Excellent:
                    return RS.GpsInfoHelper_GetDisplayName_Excellent;
                case DopStatusEnum.Good:
                    return RS.GpsInfoHelper_GetDisplayName_Good;
                case DopStatusEnum.Moderate:
                    return RS.GpsInfoHelper_GetDisplayName_Moderate;
                case DopStatusEnum.Fair:
                    return RS.GpsInfoHelper_GetDisplayName_Fair;
                case DopStatusEnum.Poor:
                    return RS.GpsInfoHelper_GetDisplayName_Poor;
                case DopStatusEnum.Unknown:
                default:
                    return RS.GpsInfoHelper_GetDescription_Unknown;
            }
        }

        public static string GetShortDisplayName(this GpsFixType fixType)
        {
            switch (fixType)
            {
                case GpsFixType.GpsFixTypeNoGps:
                    return "No GPS";
                case GpsFixType.GpsFixTypeNoFix:
                    return "No Fix";
                case GpsFixType.GpsFixType2dFix:
                    return "2D Fix";
                case GpsFixType.GpsFixType3dFix:
                    return "3D Fix";
                case GpsFixType.GpsFixTypeDgps:
                    return "Dgps";
                case GpsFixType.GpsFixTypeRtkFloat:
                    return "RTK Float";
                case GpsFixType.GpsFixTypeRtkFixed:
                    return "RTK Fix";
                case GpsFixType.GpsFixTypeStatic:
                    return "Static";
                case GpsFixType.GpsFixTypePpp:
                    return "Ppp";
                default:
                    return RS.GpsInfoHelper_GetDescription_Unknown;
            }
        }
    }
}
