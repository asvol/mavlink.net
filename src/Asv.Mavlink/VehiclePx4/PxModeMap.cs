using System;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public enum CustomMainMode
    {
        Empty = 0,
        Px4CustomMainModeManual = 1,
        Px4CustomMainModeAltctl = 2,
        Px4CustomMainModePosctl = 3,
        Px4CustomMainModeAuto = 4,
        Px4CustomMainModeAcro = 5,
        Px4CustomMainModeOffboard = 6,
        Px4CustomMainModeStabilized = 7,
        Px4CustomMainModeRattitude = 8,
    }

    public enum CustomSubMode
    {
        Empty = 0,
        Px4CustomSubModeAutoReady = 1,
        Px4CustomSubModeAutoTakeoff = 2,
        Px4CustomSubModeAutoLoiter = 3,
        Px4CustomSubModeAutoMission = 4,
        Px4CustomSubModeAutoRTL = 5,
        Px4CustomSubModeAutoLand = 6,
        Px4CustomSubModeAutoRtgs = 7,
        Px4CustomSubModeAutoFollowTarget = 8,
    }

    public enum Px4CustomMode
    {
        Manual,
        Stabilized,
        Acro,
        Rattitude,
        Altctl,
        Posctl,
        Loiter,
        Mission,
        RTL,
        Followme,
        Offboard,
        Unknwon,
    }

    public static class Px4ModeHelper
    {
        public static Px4VehicleMode Create(this Px4CustomMode mode)
        {
            switch (mode)
            {
                case Px4CustomMode.Manual: return Manual;
                case Px4CustomMode.Stabilized: return Stabilized;
                case Px4CustomMode.Acro: return Acro;
                case Px4CustomMode.Rattitude: return Rattitude;
                case Px4CustomMode.Altctl: return Altctl;
                case Px4CustomMode.Posctl: return Posctl;
                case Px4CustomMode.Loiter: return Loiter;
                case Px4CustomMode.Mission: return Mission;
                case Px4CustomMode.RTL: return RTL;
                case Px4CustomMode.Followme: return Followme;
                case Px4CustomMode.Offboard: return Offboard;
                default:
                    throw new ArgumentOutOfRangeException(nameof(mode), mode, null);
            }
        }

        public static Px4CustomMode GetMode(this Px4VehicleMode mode)
        {
            if (mode.Equals(Manual)) return Px4CustomMode.Manual;
            if (mode.Equals(Stabilized)) return Px4CustomMode.Stabilized;
            if (mode.Equals(Acro)) return Px4CustomMode.Acro;
            if (mode.Equals(Rattitude)) return Px4CustomMode.Rattitude;
            if (mode.Equals(Altctl)) return Px4CustomMode.Altctl;
            if (mode.Equals(Posctl)) return Px4CustomMode.Posctl;
            if (mode.Equals(Loiter)) return Px4CustomMode.Loiter;
            if (mode.Equals(Mission)) return Px4CustomMode.Mission;
            if (mode.Equals(RTL)) return Px4CustomMode.RTL;
            if (mode.Equals(Followme)) return Px4CustomMode.Followme;
            if (mode.Equals(Offboard)) return Px4CustomMode.Offboard;

            return Px4CustomMode.Unknwon;

        }

        public static Px4VehicleMode Manual = new Px4VehicleMode
        {
            ModeFlag = MavModeFlag.MavModeFlagCustomModeEnabled | MavModeFlag.MavModeFlagStabilizeEnabled | MavModeFlag.MavModeFlagManualInputEnabled,
            CustomMainMode = CustomMainMode.Px4CustomMainModeManual,
            CustomSubMode = CustomSubMode.Empty,
        };
        public static Px4VehicleMode Stabilized = new Px4VehicleMode
        {
            ModeFlag = MavModeFlag.MavModeFlagCustomModeEnabled | MavModeFlag.MavModeFlagStabilizeEnabled | MavModeFlag.MavModeFlagManualInputEnabled,
            CustomMainMode = CustomMainMode.Px4CustomMainModeStabilized,
            CustomSubMode = CustomSubMode.Empty,
        };
        public static Px4VehicleMode Acro = new Px4VehicleMode
        {
            ModeFlag = MavModeFlag.MavModeFlagCustomModeEnabled | MavModeFlag.MavModeFlagManualInputEnabled,
            CustomMainMode = CustomMainMode.Px4CustomMainModeAcro,
            CustomSubMode = CustomSubMode.Empty,
        };
        public static Px4VehicleMode Rattitude = new Px4VehicleMode
        {
            ModeFlag = MavModeFlag.MavModeFlagCustomModeEnabled | MavModeFlag.MavModeFlagManualInputEnabled,
            CustomMainMode = CustomMainMode.Px4CustomMainModeRattitude,
            CustomSubMode = CustomSubMode.Empty,
        };
        public static Px4VehicleMode Altctl = new Px4VehicleMode
        {
            ModeFlag = MavModeFlag.MavModeFlagCustomModeEnabled | MavModeFlag.MavModeFlagStabilizeEnabled | MavModeFlag.MavModeFlagManualInputEnabled,
            CustomMainMode = CustomMainMode.Px4CustomMainModeAltctl,
            CustomSubMode = CustomSubMode.Empty,
        };
        public static Px4VehicleMode Posctl = new Px4VehicleMode
        {
            ModeFlag = MavModeFlag.MavModeFlagCustomModeEnabled | MavModeFlag.MavModeFlagStabilizeEnabled | MavModeFlag.MavModeFlagManualInputEnabled,
            CustomMainMode = CustomMainMode.Px4CustomMainModePosctl,
            CustomSubMode = CustomSubMode.Empty,
        };
        public static Px4VehicleMode Loiter = new Px4VehicleMode
        {
            ModeFlag = MavModeFlag.MavModeFlagCustomModeEnabled | MavModeFlag.MavModeFlagAutoEnabled | MavModeFlag.MavModeFlagStabilizeEnabled | MavModeFlag.MavModeFlagGuidedEnabled,
            CustomMainMode = CustomMainMode.Px4CustomMainModeAuto,
            CustomSubMode = CustomSubMode.Px4CustomSubModeAutoLoiter,
        };
        public static Px4VehicleMode Mission = new Px4VehicleMode
        {
            ModeFlag = MavModeFlag.MavModeFlagCustomModeEnabled | MavModeFlag.MavModeFlagAutoEnabled | MavModeFlag.MavModeFlagStabilizeEnabled | MavModeFlag.MavModeFlagGuidedEnabled,
            CustomMainMode = CustomMainMode.Px4CustomMainModeAuto,
            CustomSubMode = CustomSubMode.Px4CustomSubModeAutoMission,
        };
        public static Px4VehicleMode RTL = new Px4VehicleMode
        {
            ModeFlag = MavModeFlag.MavModeFlagCustomModeEnabled | MavModeFlag.MavModeFlagAutoEnabled | MavModeFlag.MavModeFlagStabilizeEnabled | MavModeFlag.MavModeFlagGuidedEnabled,
            CustomMainMode = CustomMainMode.Px4CustomMainModeAuto,
            CustomSubMode = CustomSubMode.Px4CustomSubModeAutoRTL,
        };
        public static Px4VehicleMode Followme = new Px4VehicleMode
        {
            ModeFlag = MavModeFlag.MavModeFlagCustomModeEnabled | MavModeFlag.MavModeFlagAutoEnabled | MavModeFlag.MavModeFlagStabilizeEnabled | MavModeFlag.MavModeFlagGuidedEnabled,
            CustomMainMode = CustomMainMode.Px4CustomMainModeAuto,
            CustomSubMode = CustomSubMode.Px4CustomSubModeAutoFollowTarget,
        };
        public static Px4VehicleMode Offboard = new Px4VehicleMode
        {
            ModeFlag = MavModeFlag.MavModeFlagCustomModeEnabled | MavModeFlag.MavModeFlagAutoEnabled | MavModeFlag.MavModeFlagStabilizeEnabled | MavModeFlag.MavModeFlagGuidedEnabled,
            CustomMainMode = CustomMainMode.Px4CustomMainModeOffboard,
            CustomSubMode = CustomSubMode.Empty,
        };


    }

    public class Px4VehicleMode:IEquatable<Px4VehicleMode>
    {
        public Px4VehicleMode()
        {
            
        }

        public Px4VehicleMode(HeartbeatPayload payload)
        {
            CustomMainMode = (CustomMainMode) ((payload.CustomMode & 0xFF0000) >> 16);
            CustomSubMode = (CustomSubMode) ((payload.CustomMode & 0xFF000000) >> 24);
            ModeFlag = payload.BaseMode;
        }
        public Px4CustomMode Mode => this.GetMode();
        public MavModeFlag ModeFlag { get; set; }
        public CustomMainMode CustomMainMode { get; set; }
        public CustomSubMode CustomSubMode { get; set; }

        public bool Equals(Px4VehicleMode other)
        {
            if (ReferenceEquals(null, other)) return false;
            if (ReferenceEquals(this, other)) return true;
            return ModeFlag == other.ModeFlag && CustomMainMode == other.CustomMainMode && CustomSubMode == other.CustomSubMode;
        }

        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj)) return false;
            if (ReferenceEquals(this, obj)) return true;
            if (obj.GetType() != this.GetType()) return false;
            return Equals((Px4VehicleMode) obj);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = (int) ModeFlag;
                hashCode = (hashCode * 397) ^ (int) CustomMainMode;
                hashCode = (hashCode * 397) ^ (int) CustomSubMode;
                return hashCode;
            }
        }
    }

    
}
