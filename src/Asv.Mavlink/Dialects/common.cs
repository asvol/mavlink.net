// MIT License
//
// Copyright (c) 2018 Alexey Voloshkevich Cursir ltd. (https://github.com/asvol)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// This code was generate by tool Asv.Mavlink.Gen version v0.1.0-6-g9101a23

using System;
using System.Text;

namespace Asv.Mavlink.V2.Common
{

    public static class CommonHelper
    {
        public static void RegisterCommonDialect(this IPacketDecoder<IPacketV2<IPayload>> src)
        {
            src.Register(()=>new HeartbeatPacket());
            src.Register(()=>new SysStatusPacket());
            src.Register(()=>new SystemTimePacket());
            src.Register(()=>new PingPacket());
            src.Register(()=>new ChangeOperatorControlPacket());
            src.Register(()=>new ChangeOperatorControlAckPacket());
            src.Register(()=>new AuthKeyPacket());
            src.Register(()=>new SetModePacket());
            src.Register(()=>new ParamRequestReadPacket());
            src.Register(()=>new ParamRequestListPacket());
            src.Register(()=>new ParamValuePacket());
            src.Register(()=>new ParamSetPacket());
            src.Register(()=>new GpsRawIntPacket());
            src.Register(()=>new GpsStatusPacket());
            src.Register(()=>new ScaledImuPacket());
            src.Register(()=>new RawImuPacket());
            src.Register(()=>new RawPressurePacket());
            src.Register(()=>new ScaledPressurePacket());
            src.Register(()=>new AttitudePacket());
            src.Register(()=>new AttitudeQuaternionPacket());
            src.Register(()=>new LocalPositionNedPacket());
            src.Register(()=>new GlobalPositionIntPacket());
            src.Register(()=>new RcChannelsScaledPacket());
            src.Register(()=>new RcChannelsRawPacket());
            src.Register(()=>new ServoOutputRawPacket());
            src.Register(()=>new MissionRequestPartialListPacket());
            src.Register(()=>new MissionWritePartialListPacket());
            src.Register(()=>new MissionItemPacket());
            src.Register(()=>new MissionRequestPacket());
            src.Register(()=>new MissionSetCurrentPacket());
            src.Register(()=>new MissionCurrentPacket());
            src.Register(()=>new MissionRequestListPacket());
            src.Register(()=>new MissionCountPacket());
            src.Register(()=>new MissionClearAllPacket());
            src.Register(()=>new MissionItemReachedPacket());
            src.Register(()=>new MissionAckPacket());
            src.Register(()=>new SetGpsGlobalOriginPacket());
            src.Register(()=>new GpsGlobalOriginPacket());
            src.Register(()=>new ParamMapRcPacket());
            src.Register(()=>new MissionRequestIntPacket());
            src.Register(()=>new SafetySetAllowedAreaPacket());
            src.Register(()=>new SafetyAllowedAreaPacket());
            src.Register(()=>new AttitudeQuaternionCovPacket());
            src.Register(()=>new NavControllerOutputPacket());
            src.Register(()=>new GlobalPositionIntCovPacket());
            src.Register(()=>new LocalPositionNedCovPacket());
            src.Register(()=>new RcChannelsPacket());
            src.Register(()=>new RequestDataStreamPacket());
            src.Register(()=>new DataStreamPacket());
            src.Register(()=>new ManualControlPacket());
            src.Register(()=>new RcChannelsOverridePacket());
            src.Register(()=>new MissionItemIntPacket());
            src.Register(()=>new VfrHudPacket());
            src.Register(()=>new CommandIntPacket());
            src.Register(()=>new CommandLongPacket());
            src.Register(()=>new CommandAckPacket());
            src.Register(()=>new ManualSetpointPacket());
            src.Register(()=>new SetAttitudeTargetPacket());
            src.Register(()=>new AttitudeTargetPacket());
            src.Register(()=>new SetPositionTargetLocalNedPacket());
            src.Register(()=>new PositionTargetLocalNedPacket());
            src.Register(()=>new SetPositionTargetGlobalIntPacket());
            src.Register(()=>new PositionTargetGlobalIntPacket());
            src.Register(()=>new LocalPositionNedSystemGlobalOffsetPacket());
            src.Register(()=>new HilStatePacket());
            src.Register(()=>new HilControlsPacket());
            src.Register(()=>new HilRcInputsRawPacket());
            src.Register(()=>new HilActuatorControlsPacket());
            src.Register(()=>new OpticalFlowPacket());
            src.Register(()=>new GlobalVisionPositionEstimatePacket());
            src.Register(()=>new VisionPositionEstimatePacket());
            src.Register(()=>new VisionSpeedEstimatePacket());
            src.Register(()=>new ViconPositionEstimatePacket());
            src.Register(()=>new HighresImuPacket());
            src.Register(()=>new OpticalFlowRadPacket());
            src.Register(()=>new HilSensorPacket());
            src.Register(()=>new SimStatePacket());
            src.Register(()=>new RadioStatusPacket());
            src.Register(()=>new FileTransferProtocolPacket());
            src.Register(()=>new TimesyncPacket());
            src.Register(()=>new CameraTriggerPacket());
            src.Register(()=>new HilGpsPacket());
            src.Register(()=>new HilOpticalFlowPacket());
            src.Register(()=>new HilStateQuaternionPacket());
            src.Register(()=>new ScaledImu2Packet());
            src.Register(()=>new LogRequestListPacket());
            src.Register(()=>new LogEntryPacket());
            src.Register(()=>new LogRequestDataPacket());
            src.Register(()=>new LogDataPacket());
            src.Register(()=>new LogErasePacket());
            src.Register(()=>new LogRequestEndPacket());
            src.Register(()=>new GpsInjectDataPacket());
            src.Register(()=>new Gps2RawPacket());
            src.Register(()=>new PowerStatusPacket());
            src.Register(()=>new SerialControlPacket());
            src.Register(()=>new GpsRtkPacket());
            src.Register(()=>new Gps2RtkPacket());
            src.Register(()=>new ScaledImu3Packet());
            src.Register(()=>new DataTransmissionHandshakePacket());
            src.Register(()=>new EncapsulatedDataPacket());
            src.Register(()=>new DistanceSensorPacket());
            src.Register(()=>new TerrainRequestPacket());
            src.Register(()=>new TerrainDataPacket());
            src.Register(()=>new TerrainCheckPacket());
            src.Register(()=>new TerrainReportPacket());
            src.Register(()=>new ScaledPressure2Packet());
            src.Register(()=>new AttPosMocapPacket());
            src.Register(()=>new SetActuatorControlTargetPacket());
            src.Register(()=>new ActuatorControlTargetPacket());
            src.Register(()=>new AltitudePacket());
            src.Register(()=>new ResourceRequestPacket());
            src.Register(()=>new ScaledPressure3Packet());
            src.Register(()=>new FollowTargetPacket());
            src.Register(()=>new ControlSystemStatePacket());
            src.Register(()=>new BatteryStatusPacket());
            src.Register(()=>new AutopilotVersionPacket());
            src.Register(()=>new LandingTargetPacket());
            src.Register(()=>new EstimatorStatusPacket());
            src.Register(()=>new WindCovPacket());
            src.Register(()=>new GpsInputPacket());
            src.Register(()=>new GpsRtcmDataPacket());
            src.Register(()=>new HighLatencyPacket());
            src.Register(()=>new HighLatency2Packet());
            src.Register(()=>new VibrationPacket());
            src.Register(()=>new HomePositionPacket());
            src.Register(()=>new SetHomePositionPacket());
            src.Register(()=>new MessageIntervalPacket());
            src.Register(()=>new ExtendedSysStatePacket());
            src.Register(()=>new AdsbVehiclePacket());
            src.Register(()=>new CollisionPacket());
            src.Register(()=>new V2ExtensionPacket());
            src.Register(()=>new MemoryVectPacket());
            src.Register(()=>new DebugVectPacket());
            src.Register(()=>new NamedValueFloatPacket());
            src.Register(()=>new NamedValueIntPacket());
            src.Register(()=>new StatustextPacket());
            src.Register(()=>new DebugPacket());
            src.Register(()=>new SetupSigningPacket());
            src.Register(()=>new ButtonChangePacket());
            src.Register(()=>new PlayTunePacket());
            src.Register(()=>new CameraInformationPacket());
            src.Register(()=>new CameraSettingsPacket());
            src.Register(()=>new StorageInformationPacket());
            src.Register(()=>new CameraCaptureStatusPacket());
            src.Register(()=>new CameraImageCapturedPacket());
            src.Register(()=>new FlightInformationPacket());
            src.Register(()=>new MountOrientationPacket());
            src.Register(()=>new LoggingDataPacket());
            src.Register(()=>new LoggingDataAckedPacket());
            src.Register(()=>new LoggingAckPacket());
            src.Register(()=>new VideoStreamInformationPacket());
            src.Register(()=>new SetVideoStreamSettingsPacket());
            src.Register(()=>new WifiConfigApPacket());
            src.Register(()=>new ProtocolVersionPacket());
            src.Register(()=>new UavcanNodeStatusPacket());
            src.Register(()=>new UavcanNodeInfoPacket());
            src.Register(()=>new ParamExtRequestReadPacket());
            src.Register(()=>new ParamExtRequestListPacket());
            src.Register(()=>new ParamExtValuePacket());
            src.Register(()=>new ParamExtSetPacket());
            src.Register(()=>new ParamExtAckPacket());
            src.Register(()=>new ObstacleDistancePacket());
            src.Register(()=>new OdometryPacket());
            src.Register(()=>new TrajectoryPacket());
        }
    }

#region Enums

    /// <summary>
    /// Micro air vehicle / autopilot classes. This identifies the individual model.
    ///  MAV_AUTOPILOT
    /// </summary>
    public enum MavAutopilot
    {
        /// <summary>
        /// Generic autopilot, full support for everything
        /// MAV_AUTOPILOT_GENERIC
        /// </summary>
        MavAutopilotGeneric = 0,
        /// <summary>
        /// Reserved for future use.
        /// MAV_AUTOPILOT_RESERVED
        /// </summary>
        MavAutopilotReserved = 1,
        /// <summary>
        /// SLUGS autopilot, http://slugsuav.soe.ucsc.edu
        /// MAV_AUTOPILOT_SLUGS
        /// </summary>
        MavAutopilotSlugs = 2,
        /// <summary>
        /// ArduPilotMega / ArduCopter, http://diydrones.com
        /// MAV_AUTOPILOT_ARDUPILOTMEGA
        /// </summary>
        MavAutopilotArdupilotmega = 3,
        /// <summary>
        /// OpenPilot, http://openpilot.org
        /// MAV_AUTOPILOT_OPENPILOT
        /// </summary>
        MavAutopilotOpenpilot = 4,
        /// <summary>
        /// Generic autopilot only supporting simple waypoints
        /// MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY
        /// </summary>
        MavAutopilotGenericWaypointsOnly = 5,
        /// <summary>
        /// Generic autopilot supporting waypoints and other simple navigation commands
        /// MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY
        /// </summary>
        MavAutopilotGenericWaypointsAndSimpleNavigationOnly = 6,
        /// <summary>
        /// Generic autopilot supporting the full mission command set
        /// MAV_AUTOPILOT_GENERIC_MISSION_FULL
        /// </summary>
        MavAutopilotGenericMissionFull = 7,
        /// <summary>
        /// No valid autopilot, e.g. a GCS or other MAVLink component
        /// MAV_AUTOPILOT_INVALID
        /// </summary>
        MavAutopilotInvalid = 8,
        /// <summary>
        /// PPZ UAV - http://nongnu.org/paparazzi
        /// MAV_AUTOPILOT_PPZ
        /// </summary>
        MavAutopilotPpz = 9,
        /// <summary>
        /// UAV Dev Board
        /// MAV_AUTOPILOT_UDB
        /// </summary>
        MavAutopilotUdb = 10,
        /// <summary>
        /// FlexiPilot
        /// MAV_AUTOPILOT_FP
        /// </summary>
        MavAutopilotFp = 11,
        /// <summary>
        /// PX4 Autopilot - http://pixhawk.ethz.ch/px4/
        /// MAV_AUTOPILOT_PX4
        /// </summary>
        MavAutopilotPx4 = 12,
        /// <summary>
        /// SMACCMPilot - http://smaccmpilot.org
        /// MAV_AUTOPILOT_SMACCMPILOT
        /// </summary>
        MavAutopilotSmaccmpilot = 13,
        /// <summary>
        /// AutoQuad -- http://autoquad.org
        /// MAV_AUTOPILOT_AUTOQUAD
        /// </summary>
        MavAutopilotAutoquad = 14,
        /// <summary>
        /// Armazila -- http://armazila.com
        /// MAV_AUTOPILOT_ARMAZILA
        /// </summary>
        MavAutopilotArmazila = 15,
        /// <summary>
        /// Aerob -- http://aerob.ru
        /// MAV_AUTOPILOT_AEROB
        /// </summary>
        MavAutopilotAerob = 16,
        /// <summary>
        /// ASLUAV autopilot -- http://www.asl.ethz.ch
        /// MAV_AUTOPILOT_ASLUAV
        /// </summary>
        MavAutopilotAsluav = 17,
        /// <summary>
        /// SmartAP Autopilot - http://sky-drones.com
        /// MAV_AUTOPILOT_SMARTAP
        /// </summary>
        MavAutopilotSmartap = 18,
        /// <summary>
        /// AirRails - http://uaventure.com
        /// MAV_AUTOPILOT_AIRRAILS
        /// </summary>
        MavAutopilotAirrails = 19,
    }

    /// <summary>
    ///  MAV_TYPE
    /// </summary>
    public enum MavType
    {
        /// <summary>
        /// Generic micro air vehicle.
        /// MAV_TYPE_GENERIC
        /// </summary>
        MavTypeGeneric = 0,
        /// <summary>
        /// Fixed wing aircraft.
        /// MAV_TYPE_FIXED_WING
        /// </summary>
        MavTypeFixedWing = 1,
        /// <summary>
        /// Quadrotor
        /// MAV_TYPE_QUADROTOR
        /// </summary>
        MavTypeQuadrotor = 2,
        /// <summary>
        /// Coaxial helicopter
        /// MAV_TYPE_COAXIAL
        /// </summary>
        MavTypeCoaxial = 3,
        /// <summary>
        /// Normal helicopter with tail rotor.
        /// MAV_TYPE_HELICOPTER
        /// </summary>
        MavTypeHelicopter = 4,
        /// <summary>
        /// Ground installation
        /// MAV_TYPE_ANTENNA_TRACKER
        /// </summary>
        MavTypeAntennaTracker = 5,
        /// <summary>
        /// Operator control unit / ground control station
        /// MAV_TYPE_GCS
        /// </summary>
        MavTypeGcs = 6,
        /// <summary>
        /// Airship, controlled
        /// MAV_TYPE_AIRSHIP
        /// </summary>
        MavTypeAirship = 7,
        /// <summary>
        /// Free balloon, uncontrolled
        /// MAV_TYPE_FREE_BALLOON
        /// </summary>
        MavTypeFreeBalloon = 8,
        /// <summary>
        /// Rocket
        /// MAV_TYPE_ROCKET
        /// </summary>
        MavTypeRocket = 9,
        /// <summary>
        /// Ground rover
        /// MAV_TYPE_GROUND_ROVER
        /// </summary>
        MavTypeGroundRover = 10,
        /// <summary>
        /// Surface vessel, boat, ship
        /// MAV_TYPE_SURFACE_BOAT
        /// </summary>
        MavTypeSurfaceBoat = 11,
        /// <summary>
        /// Submarine
        /// MAV_TYPE_SUBMARINE
        /// </summary>
        MavTypeSubmarine = 12,
        /// <summary>
        /// Hexarotor
        /// MAV_TYPE_HEXAROTOR
        /// </summary>
        MavTypeHexarotor = 13,
        /// <summary>
        /// Octorotor
        /// MAV_TYPE_OCTOROTOR
        /// </summary>
        MavTypeOctorotor = 14,
        /// <summary>
        /// Tricopter
        /// MAV_TYPE_TRICOPTER
        /// </summary>
        MavTypeTricopter = 15,
        /// <summary>
        /// Flapping wing
        /// MAV_TYPE_FLAPPING_WING
        /// </summary>
        MavTypeFlappingWing = 16,
        /// <summary>
        /// Kite
        /// MAV_TYPE_KITE
        /// </summary>
        MavTypeKite = 17,
        /// <summary>
        /// Onboard companion controller
        /// MAV_TYPE_ONBOARD_CONTROLLER
        /// </summary>
        MavTypeOnboardController = 18,
        /// <summary>
        /// Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
        /// MAV_TYPE_VTOL_DUOROTOR
        /// </summary>
        MavTypeVtolDuorotor = 19,
        /// <summary>
        /// Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
        /// MAV_TYPE_VTOL_QUADROTOR
        /// </summary>
        MavTypeVtolQuadrotor = 20,
        /// <summary>
        /// Tiltrotor VTOL
        /// MAV_TYPE_VTOL_TILTROTOR
        /// </summary>
        MavTypeVtolTiltrotor = 21,
        /// <summary>
        /// VTOL reserved 2
        /// MAV_TYPE_VTOL_RESERVED2
        /// </summary>
        MavTypeVtolReserved2 = 22,
        /// <summary>
        /// VTOL reserved 3
        /// MAV_TYPE_VTOL_RESERVED3
        /// </summary>
        MavTypeVtolReserved3 = 23,
        /// <summary>
        /// VTOL reserved 4
        /// MAV_TYPE_VTOL_RESERVED4
        /// </summary>
        MavTypeVtolReserved4 = 24,
        /// <summary>
        /// VTOL reserved 5
        /// MAV_TYPE_VTOL_RESERVED5
        /// </summary>
        MavTypeVtolReserved5 = 25,
        /// <summary>
        /// Onboard gimbal
        /// MAV_TYPE_GIMBAL
        /// </summary>
        MavTypeGimbal = 26,
        /// <summary>
        /// Onboard ADSB peripheral
        /// MAV_TYPE_ADSB
        /// </summary>
        MavTypeAdsb = 27,
        /// <summary>
        /// Steerable, nonrigid airfoil
        /// MAV_TYPE_PARAFOIL
        /// </summary>
        MavTypeParafoil = 28,
        /// <summary>
        /// Dodecarotor
        /// MAV_TYPE_DODECAROTOR
        /// </summary>
        MavTypeDodecarotor = 29,
        /// <summary>
        /// Camera
        /// MAV_TYPE_CAMERA
        /// </summary>
        MavTypeCamera = 30,
        /// <summary>
        /// Charging station
        /// MAV_TYPE_CHARGING_STATION
        /// </summary>
        MavTypeChargingStation = 31,
        /// <summary>
        /// Onboard FLARM collision avoidance system
        /// MAV_TYPE_FLARM
        /// </summary>
        MavTypeFlarm = 32,
    }

    /// <summary>
    /// These values define the type of firmware release.  These values indicate the first version or release of this type.  For example the first alpha release would be 64, the second would be 65.
    ///  FIRMWARE_VERSION_TYPE
    /// </summary>
    public enum FirmwareVersionType
    {
        /// <summary>
        /// development release
        /// FIRMWARE_VERSION_TYPE_DEV
        /// </summary>
        FirmwareVersionTypeDev = 0,
        /// <summary>
        /// alpha release
        /// FIRMWARE_VERSION_TYPE_ALPHA
        /// </summary>
        FirmwareVersionTypeAlpha = 64,
        /// <summary>
        /// beta release
        /// FIRMWARE_VERSION_TYPE_BETA
        /// </summary>
        FirmwareVersionTypeBeta = 128,
        /// <summary>
        /// release candidate
        /// FIRMWARE_VERSION_TYPE_RC
        /// </summary>
        FirmwareVersionTypeRc = 192,
        /// <summary>
        /// official stable release
        /// FIRMWARE_VERSION_TYPE_OFFICIAL
        /// </summary>
        FirmwareVersionTypeOfficial = 255,
    }

    /// <summary>
    /// Flags to report failure cases over the high latency telemtry.
    ///  HL_FAILURE_FLAG
    /// </summary>
    public enum HlFailureFlag
    {
        /// <summary>
        /// GPS failure.
        /// HL_FAILURE_FLAG_GPS
        /// </summary>
        HlFailureFlagGps = 1,
        /// <summary>
        /// Differential pressure sensor failure.
        /// HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE
        /// </summary>
        HlFailureFlagDifferentialPressure = 2,
        /// <summary>
        /// Absolute pressure sensor failure.
        /// HL_FAILURE_FLAG_ABSOLUTE_PRESSURE
        /// </summary>
        HlFailureFlagAbsolutePressure = 4,
        /// <summary>
        /// Accelerometer sensor failure.
        /// HL_FAILURE_FLAG_3D_ACCEL
        /// </summary>
        HlFailureFlag3dAccel = 8,
        /// <summary>
        /// Gyroscope sensor failure.
        /// HL_FAILURE_FLAG_3D_GYRO
        /// </summary>
        HlFailureFlag3dGyro = 16,
        /// <summary>
        /// Magnetometer sensor failure.
        /// HL_FAILURE_FLAG_3D_MAG
        /// </summary>
        HlFailureFlag3dMag = 32,
        /// <summary>
        /// Terrain subsystem failure.
        /// HL_FAILURE_FLAG_TERRAIN
        /// </summary>
        HlFailureFlagTerrain = 64,
        /// <summary>
        /// Battery failure/critical low battery.
        /// HL_FAILURE_FLAG_BATTERY
        /// </summary>
        HlFailureFlagBattery = 128,
        /// <summary>
        /// RC receiver failure/no rc connection.
        /// HL_FAILURE_FLAG_RC_RECEIVER
        /// </summary>
        HlFailureFlagRcReceiver = 256,
        /// <summary>
        /// Offboard link failure.
        /// HL_FAILURE_FLAG_OFFBOARD_LINK
        /// </summary>
        HlFailureFlagOffboardLink = 512,
        /// <summary>
        /// Engine failure.
        /// HL_FAILURE_FLAG_ENGINE
        /// </summary>
        HlFailureFlagEngine = 1024,
        /// <summary>
        /// Geofence violation.
        /// HL_FAILURE_FLAG_GEOFENCE
        /// </summary>
        HlFailureFlagGeofence = 2048,
        /// <summary>
        /// Estimator failure, for example measurement rejection or large variances.
        /// HL_FAILURE_FLAG_ESTIMATOR
        /// </summary>
        HlFailureFlagEstimator = 4096,
        /// <summary>
        /// Mission failure.
        /// HL_FAILURE_FLAG_MISSION
        /// </summary>
        HlFailureFlagMission = 8192,
    }

    /// <summary>
    /// These flags encode the MAV mode.
    ///  MAV_MODE_FLAG
    /// </summary>
    public enum MavModeFlag
    {
        /// <summary>
        /// 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state.
        /// MAV_MODE_FLAG_SAFETY_ARMED
        /// </summary>
        MavModeFlagSafetyArmed = 128,
        /// <summary>
        /// 0b01000000 remote control input is enabled.
        /// MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
        /// </summary>
        MavModeFlagManualInputEnabled = 64,
        /// <summary>
        /// 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
        /// MAV_MODE_FLAG_HIL_ENABLED
        /// </summary>
        MavModeFlagHilEnabled = 32,
        /// <summary>
        /// 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
        /// MAV_MODE_FLAG_STABILIZE_ENABLED
        /// </summary>
        MavModeFlagStabilizeEnabled = 16,
        /// <summary>
        /// 0b00001000 guided mode enabled, system flies waypoints / mission items.
        /// MAV_MODE_FLAG_GUIDED_ENABLED
        /// </summary>
        MavModeFlagGuidedEnabled = 8,
        /// <summary>
        /// 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
        /// MAV_MODE_FLAG_AUTO_ENABLED
        /// </summary>
        MavModeFlagAutoEnabled = 4,
        /// <summary>
        /// 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
        /// MAV_MODE_FLAG_TEST_ENABLED
        /// </summary>
        MavModeFlagTestEnabled = 2,
        /// <summary>
        /// 0b00000001 Reserved for future use.
        /// MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        /// </summary>
        MavModeFlagCustomModeEnabled = 1,
    }

    /// <summary>
    /// These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
    ///  MAV_MODE_FLAG_DECODE_POSITION
    /// </summary>
    public enum MavModeFlagDecodePosition
    {
        /// <summary>
        /// First bit:  10000000
        /// MAV_MODE_FLAG_DECODE_POSITION_SAFETY
        /// </summary>
        MavModeFlagDecodePositionSafety = 128,
        /// <summary>
        /// Second bit: 01000000
        /// MAV_MODE_FLAG_DECODE_POSITION_MANUAL
        /// </summary>
        MavModeFlagDecodePositionManual = 64,
        /// <summary>
        /// Third bit:  00100000
        /// MAV_MODE_FLAG_DECODE_POSITION_HIL
        /// </summary>
        MavModeFlagDecodePositionHil = 32,
        /// <summary>
        /// Fourth bit: 00010000
        /// MAV_MODE_FLAG_DECODE_POSITION_STABILIZE
        /// </summary>
        MavModeFlagDecodePositionStabilize = 16,
        /// <summary>
        /// Fifth bit:  00001000
        /// MAV_MODE_FLAG_DECODE_POSITION_GUIDED
        /// </summary>
        MavModeFlagDecodePositionGuided = 8,
        /// <summary>
        /// Sixt bit:   00000100
        /// MAV_MODE_FLAG_DECODE_POSITION_AUTO
        /// </summary>
        MavModeFlagDecodePositionAuto = 4,
        /// <summary>
        /// Seventh bit: 00000010
        /// MAV_MODE_FLAG_DECODE_POSITION_TEST
        /// </summary>
        MavModeFlagDecodePositionTest = 2,
        /// <summary>
        /// Eighth bit: 00000001
        /// MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE
        /// </summary>
        MavModeFlagDecodePositionCustomMode = 1,
    }

    /// <summary>
    /// Override command, pauses current mission execution and moves immediately to a position
    ///  MAV_GOTO
    /// </summary>
    public enum MavGoto
    {
        /// <summary>
        /// Hold at the current position.
        /// MAV_GOTO_DO_HOLD
        /// </summary>
        MavGotoDoHold = 0,
        /// <summary>
        /// Continue with the next item in mission execution.
        /// MAV_GOTO_DO_CONTINUE
        /// </summary>
        MavGotoDoContinue = 1,
        /// <summary>
        /// Hold at the current position of the system
        /// MAV_GOTO_HOLD_AT_CURRENT_POSITION
        /// </summary>
        MavGotoHoldAtCurrentPosition = 2,
        /// <summary>
        /// Hold at the position specified in the parameters of the DO_HOLD action
        /// MAV_GOTO_HOLD_AT_SPECIFIED_POSITION
        /// </summary>
        MavGotoHoldAtSpecifiedPosition = 3,
    }

    /// <summary>
    /// These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
    ///                simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.
    ///  MAV_MODE
    /// </summary>
    public enum MavMode
    {
        /// <summary>
        /// System is not ready to fly, booting, calibrating, etc. No flag is set.
        /// MAV_MODE_PREFLIGHT
        /// </summary>
        MavModePreflight = 0,
        /// <summary>
        /// System is allowed to be active, under assisted RC control.
        /// MAV_MODE_STABILIZE_DISARMED
        /// </summary>
        MavModeStabilizeDisarmed = 80,
        /// <summary>
        /// System is allowed to be active, under assisted RC control.
        /// MAV_MODE_STABILIZE_ARMED
        /// </summary>
        MavModeStabilizeArmed = 208,
        /// <summary>
        /// System is allowed to be active, under manual (RC) control, no stabilization
        /// MAV_MODE_MANUAL_DISARMED
        /// </summary>
        MavModeManualDisarmed = 64,
        /// <summary>
        /// System is allowed to be active, under manual (RC) control, no stabilization
        /// MAV_MODE_MANUAL_ARMED
        /// </summary>
        MavModeManualArmed = 192,
        /// <summary>
        /// System is allowed to be active, under autonomous control, manual setpoint
        /// MAV_MODE_GUIDED_DISARMED
        /// </summary>
        MavModeGuidedDisarmed = 88,
        /// <summary>
        /// System is allowed to be active, under autonomous control, manual setpoint
        /// MAV_MODE_GUIDED_ARMED
        /// </summary>
        MavModeGuidedArmed = 216,
        /// <summary>
        /// System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
        /// MAV_MODE_AUTO_DISARMED
        /// </summary>
        MavModeAutoDisarmed = 92,
        /// <summary>
        /// System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
        /// MAV_MODE_AUTO_ARMED
        /// </summary>
        MavModeAutoArmed = 220,
        /// <summary>
        /// UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
        /// MAV_MODE_TEST_DISARMED
        /// </summary>
        MavModeTestDisarmed = 66,
        /// <summary>
        /// UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
        /// MAV_MODE_TEST_ARMED
        /// </summary>
        MavModeTestArmed = 194,
    }

    /// <summary>
    ///  MAV_STATE
    /// </summary>
    public enum MavState
    {
        /// <summary>
        /// Uninitialized system, state is unknown.
        /// MAV_STATE_UNINIT
        /// </summary>
        MavStateUninit = 0,
        /// <summary>
        /// System is booting up.
        /// MAV_STATE_BOOT
        /// </summary>
        MavStateBoot = 1,
        /// <summary>
        /// System is calibrating and not flight-ready.
        /// MAV_STATE_CALIBRATING
        /// </summary>
        MavStateCalibrating = 2,
        /// <summary>
        /// System is grounded and on standby. It can be launched any time.
        /// MAV_STATE_STANDBY
        /// </summary>
        MavStateStandby = 3,
        /// <summary>
        /// System is active and might be already airborne. Motors are engaged.
        /// MAV_STATE_ACTIVE
        /// </summary>
        MavStateActive = 4,
        /// <summary>
        /// System is in a non-normal flight mode. It can however still navigate.
        /// MAV_STATE_CRITICAL
        /// </summary>
        MavStateCritical = 5,
        /// <summary>
        /// System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
        /// MAV_STATE_EMERGENCY
        /// </summary>
        MavStateEmergency = 6,
        /// <summary>
        /// System just initialized its power-down sequence, will shut down now.
        /// MAV_STATE_POWEROFF
        /// </summary>
        MavStatePoweroff = 7,
        /// <summary>
        /// System is terminating itself.
        /// MAV_STATE_FLIGHT_TERMINATION
        /// </summary>
        MavStateFlightTermination = 8,
    }

    /// <summary>
    ///  MAV_COMPONENT
    /// </summary>
    public enum MavComponent
    {
        /// <summary>
        /// 
        /// MAV_COMP_ID_ALL
        /// </summary>
        MavCompIdAll = 0,
        /// <summary>
        /// 
        /// MAV_COMP_ID_AUTOPILOT1
        /// </summary>
        MavCompIdAutopilot1 = 1,
        /// <summary>
        /// 
        /// MAV_COMP_ID_CAMERA
        /// </summary>
        MavCompIdCamera = 100,
        /// <summary>
        /// 
        /// MAV_COMP_ID_CAMERA2
        /// </summary>
        MavCompIdCamera2 = 101,
        /// <summary>
        /// 
        /// MAV_COMP_ID_CAMERA3
        /// </summary>
        MavCompIdCamera3 = 102,
        /// <summary>
        /// 
        /// MAV_COMP_ID_CAMERA4
        /// </summary>
        MavCompIdCamera4 = 103,
        /// <summary>
        /// 
        /// MAV_COMP_ID_CAMERA5
        /// </summary>
        MavCompIdCamera5 = 104,
        /// <summary>
        /// 
        /// MAV_COMP_ID_CAMERA6
        /// </summary>
        MavCompIdCamera6 = 105,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SERVO1
        /// </summary>
        MavCompIdServo1 = 140,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SERVO2
        /// </summary>
        MavCompIdServo2 = 141,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SERVO3
        /// </summary>
        MavCompIdServo3 = 142,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SERVO4
        /// </summary>
        MavCompIdServo4 = 143,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SERVO5
        /// </summary>
        MavCompIdServo5 = 144,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SERVO6
        /// </summary>
        MavCompIdServo6 = 145,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SERVO7
        /// </summary>
        MavCompIdServo7 = 146,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SERVO8
        /// </summary>
        MavCompIdServo8 = 147,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SERVO9
        /// </summary>
        MavCompIdServo9 = 148,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SERVO10
        /// </summary>
        MavCompIdServo10 = 149,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SERVO11
        /// </summary>
        MavCompIdServo11 = 150,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SERVO12
        /// </summary>
        MavCompIdServo12 = 151,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SERVO13
        /// </summary>
        MavCompIdServo13 = 152,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SERVO14
        /// </summary>
        MavCompIdServo14 = 153,
        /// <summary>
        /// 
        /// MAV_COMP_ID_GIMBAL
        /// </summary>
        MavCompIdGimbal = 154,
        /// <summary>
        /// 
        /// MAV_COMP_ID_LOG
        /// </summary>
        MavCompIdLog = 155,
        /// <summary>
        /// 
        /// MAV_COMP_ID_ADSB
        /// </summary>
        MavCompIdAdsb = 156,
        /// <summary>
        /// On Screen Display (OSD) devices for video links
        /// MAV_COMP_ID_OSD
        /// </summary>
        MavCompIdOsd = 157,
        /// <summary>
        /// Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter sub-protocol
        /// MAV_COMP_ID_PERIPHERAL
        /// </summary>
        MavCompIdPeripheral = 158,
        /// <summary>
        /// 
        /// MAV_COMP_ID_QX1_GIMBAL
        /// </summary>
        MavCompIdQx1Gimbal = 159,
        /// <summary>
        /// 
        /// MAV_COMP_ID_FLARM
        /// </summary>
        MavCompIdFlarm = 160,
        /// <summary>
        /// 
        /// MAV_COMP_ID_MAPPER
        /// </summary>
        MavCompIdMapper = 180,
        /// <summary>
        /// 
        /// MAV_COMP_ID_MISSIONPLANNER
        /// </summary>
        MavCompIdMissionplanner = 190,
        /// <summary>
        /// 
        /// MAV_COMP_ID_PATHPLANNER
        /// </summary>
        MavCompIdPathplanner = 195,
        /// <summary>
        /// 
        /// MAV_COMP_ID_IMU
        /// </summary>
        MavCompIdImu = 200,
        /// <summary>
        /// 
        /// MAV_COMP_ID_IMU_2
        /// </summary>
        MavCompIdImu2 = 201,
        /// <summary>
        /// 
        /// MAV_COMP_ID_IMU_3
        /// </summary>
        MavCompIdImu3 = 202,
        /// <summary>
        /// 
        /// MAV_COMP_ID_GPS
        /// </summary>
        MavCompIdGps = 220,
        /// <summary>
        /// 
        /// MAV_COMP_ID_GPS2
        /// </summary>
        MavCompIdGps2 = 221,
        /// <summary>
        /// 
        /// MAV_COMP_ID_UDP_BRIDGE
        /// </summary>
        MavCompIdUdpBridge = 240,
        /// <summary>
        /// 
        /// MAV_COMP_ID_UART_BRIDGE
        /// </summary>
        MavCompIdUartBridge = 241,
        /// <summary>
        /// 
        /// MAV_COMP_ID_SYSTEM_CONTROL
        /// </summary>
        MavCompIdSystemControl = 250,
    }

    /// <summary>
    /// These encode the sensors whose status is sent as part of the SYS_STATUS message.
    ///  MAV_SYS_STATUS_SENSOR
    /// </summary>
    public enum MavSysStatusSensor
    {
        /// <summary>
        /// 0x01 3D gyro
        /// MAV_SYS_STATUS_SENSOR_3D_GYRO
        /// </summary>
        MavSysStatusSensor3dGyro = 1,
        /// <summary>
        /// 0x02 3D accelerometer
        /// MAV_SYS_STATUS_SENSOR_3D_ACCEL
        /// </summary>
        MavSysStatusSensor3dAccel = 2,
        /// <summary>
        /// 0x04 3D magnetometer
        /// MAV_SYS_STATUS_SENSOR_3D_MAG
        /// </summary>
        MavSysStatusSensor3dMag = 4,
        /// <summary>
        /// 0x08 absolute pressure
        /// MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
        /// </summary>
        MavSysStatusSensorAbsolutePressure = 8,
        /// <summary>
        /// 0x10 differential pressure
        /// MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE
        /// </summary>
        MavSysStatusSensorDifferentialPressure = 16,
        /// <summary>
        /// 0x20 GPS
        /// MAV_SYS_STATUS_SENSOR_GPS
        /// </summary>
        MavSysStatusSensorGps = 32,
        /// <summary>
        /// 0x40 optical flow
        /// MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW
        /// </summary>
        MavSysStatusSensorOpticalFlow = 64,
        /// <summary>
        /// 0x80 computer vision position
        /// MAV_SYS_STATUS_SENSOR_VISION_POSITION
        /// </summary>
        MavSysStatusSensorVisionPosition = 128,
        /// <summary>
        /// 0x100 laser based position
        /// MAV_SYS_STATUS_SENSOR_LASER_POSITION
        /// </summary>
        MavSysStatusSensorLaserPosition = 256,
        /// <summary>
        /// 0x200 external ground truth (Vicon or Leica)
        /// MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH
        /// </summary>
        MavSysStatusSensorExternalGroundTruth = 512,
        /// <summary>
        /// 0x400 3D angular rate control
        /// MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL
        /// </summary>
        MavSysStatusSensorAngularRateControl = 1024,
        /// <summary>
        /// 0x800 attitude stabilization
        /// MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION
        /// </summary>
        MavSysStatusSensorAttitudeStabilization = 2048,
        /// <summary>
        /// 0x1000 yaw position
        /// MAV_SYS_STATUS_SENSOR_YAW_POSITION
        /// </summary>
        MavSysStatusSensorYawPosition = 4096,
        /// <summary>
        /// 0x2000 z/altitude control
        /// MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL
        /// </summary>
        MavSysStatusSensorZAltitudeControl = 8192,
        /// <summary>
        /// 0x4000 x/y position control
        /// MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL
        /// </summary>
        MavSysStatusSensorXyPositionControl = 16384,
        /// <summary>
        /// 0x8000 motor outputs / control
        /// MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS
        /// </summary>
        MavSysStatusSensorMotorOutputs = 32768,
        /// <summary>
        /// 0x10000 rc receiver
        /// MAV_SYS_STATUS_SENSOR_RC_RECEIVER
        /// </summary>
        MavSysStatusSensorRcReceiver = 65536,
        /// <summary>
        /// 0x20000 2nd 3D gyro
        /// MAV_SYS_STATUS_SENSOR_3D_GYRO2
        /// </summary>
        MavSysStatusSensor3dGyro2 = 131072,
        /// <summary>
        /// 0x40000 2nd 3D accelerometer
        /// MAV_SYS_STATUS_SENSOR_3D_ACCEL2
        /// </summary>
        MavSysStatusSensor3dAccel2 = 262144,
        /// <summary>
        /// 0x80000 2nd 3D magnetometer
        /// MAV_SYS_STATUS_SENSOR_3D_MAG2
        /// </summary>
        MavSysStatusSensor3dMag2 = 524288,
        /// <summary>
        /// 0x100000 geofence
        /// MAV_SYS_STATUS_GEOFENCE
        /// </summary>
        MavSysStatusGeofence = 1048576,
        /// <summary>
        /// 0x200000 AHRS subsystem health
        /// MAV_SYS_STATUS_AHRS
        /// </summary>
        MavSysStatusAhrs = 2097152,
        /// <summary>
        /// 0x400000 Terrain subsystem health
        /// MAV_SYS_STATUS_TERRAIN
        /// </summary>
        MavSysStatusTerrain = 4194304,
        /// <summary>
        /// 0x800000 Motors are reversed
        /// MAV_SYS_STATUS_REVERSE_MOTOR
        /// </summary>
        MavSysStatusReverseMotor = 8388608,
        /// <summary>
        /// 0x1000000 Logging
        /// MAV_SYS_STATUS_LOGGING
        /// </summary>
        MavSysStatusLogging = 16777216,
        /// <summary>
        /// 0x2000000 Battery
        /// MAV_SYS_STATUS_SENSOR_BATTERY
        /// </summary>
        MavSysStatusSensorBattery = 33554432,
        /// <summary>
        /// 0x4000000 Proximity
        /// MAV_SYS_STATUS_SENSOR_PROXIMITY
        /// </summary>
        MavSysStatusSensorProximity = 67108864,
    }

    /// <summary>
    ///  MAV_FRAME
    /// </summary>
    public enum MavFrame
    {
        /// <summary>
        /// Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL).
        /// MAV_FRAME_GLOBAL
        /// </summary>
        MavFrameGlobal = 0,
        /// <summary>
        /// Local coordinate frame, Z-down (x: north, y: east, z: down).
        /// MAV_FRAME_LOCAL_NED
        /// </summary>
        MavFrameLocalNed = 1,
        /// <summary>
        /// NOT a coordinate frame, indicates a mission command.
        /// MAV_FRAME_MISSION
        /// </summary>
        MavFrameMission = 2,
        /// <summary>
        /// Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
        /// MAV_FRAME_GLOBAL_RELATIVE_ALT
        /// </summary>
        MavFrameGlobalRelativeAlt = 3,
        /// <summary>
        /// Local coordinate frame, Z-up (x: east, y: north, z: up).
        /// MAV_FRAME_LOCAL_ENU
        /// </summary>
        MavFrameLocalEnu = 4,
        /// <summary>
        /// Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL).
        /// MAV_FRAME_GLOBAL_INT
        /// </summary>
        MavFrameGlobalInt = 5,
        /// <summary>
        /// Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location.
        /// MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        /// </summary>
        MavFrameGlobalRelativeAltInt = 6,
        /// <summary>
        /// Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.
        /// MAV_FRAME_LOCAL_OFFSET_NED
        /// </summary>
        MavFrameLocalOffsetNed = 7,
        /// <summary>
        /// Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.
        /// MAV_FRAME_BODY_NED
        /// </summary>
        MavFrameBodyNed = 8,
        /// <summary>
        /// Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.
        /// MAV_FRAME_BODY_OFFSET_NED
        /// </summary>
        MavFrameBodyOffsetNed = 9,
        /// <summary>
        /// Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
        /// MAV_FRAME_GLOBAL_TERRAIN_ALT
        /// </summary>
        MavFrameGlobalTerrainAlt = 10,
        /// <summary>
        /// Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
        /// MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
        /// </summary>
        MavFrameGlobalTerrainAltInt = 11,
        /// <summary>
        /// Body fixed frame of reference, Z-down (x: forward, y: right, z: down).
        /// MAV_FRAME_BODY_FRD
        /// </summary>
        MavFrameBodyFrd = 12,
        /// <summary>
        /// Body fixed frame of reference, Z-up (x: forward, y: left, z: up).
        /// MAV_FRAME_BODY_FLU
        /// </summary>
        MavFrameBodyFlu = 13,
        /// <summary>
        /// Odometry local coordinate frame of data given by a motion capture system, Z-down (x: north, y: east, z: down).
        /// MAV_FRAME_MOCAP_NED
        /// </summary>
        MavFrameMocapNed = 14,
        /// <summary>
        /// Odometry local coordinate frame of data given by a motion capture system, Z-up (x: east, y: north, z: up).
        /// MAV_FRAME_MOCAP_ENU
        /// </summary>
        MavFrameMocapEnu = 15,
        /// <summary>
        /// Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: north, y: east, z: down).
        /// MAV_FRAME_VISION_NED
        /// </summary>
        MavFrameVisionNed = 16,
        /// <summary>
        /// Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: east, y: north, z: up).
        /// MAV_FRAME_VISION_ENU
        /// </summary>
        MavFrameVisionEnu = 17,
        /// <summary>
        /// Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: north, y: east, z: down).
        /// MAV_FRAME_ESTIM_NED
        /// </summary>
        MavFrameEstimNed = 18,
        /// <summary>
        /// Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: east, y: noth, z: up).
        /// MAV_FRAME_ESTIM_ENU
        /// </summary>
        MavFrameEstimEnu = 19,
    }

    /// <summary>
    ///  MAVLINK_DATA_STREAM_TYPE
    /// </summary>
    public enum MavlinkDataStreamType
    {
        /// <summary>
        /// 
        /// MAVLINK_DATA_STREAM_IMG_JPEG
        /// </summary>
        MavlinkDataStreamImgJpeg = 0,
        /// <summary>
        /// 
        /// MAVLINK_DATA_STREAM_IMG_BMP
        /// </summary>
        MavlinkDataStreamImgBmp = 1,
        /// <summary>
        /// 
        /// MAVLINK_DATA_STREAM_IMG_RAW8U
        /// </summary>
        MavlinkDataStreamImgRaw8u = 2,
        /// <summary>
        /// 
        /// MAVLINK_DATA_STREAM_IMG_RAW32U
        /// </summary>
        MavlinkDataStreamImgRaw32u = 3,
        /// <summary>
        /// 
        /// MAVLINK_DATA_STREAM_IMG_PGM
        /// </summary>
        MavlinkDataStreamImgPgm = 4,
        /// <summary>
        /// 
        /// MAVLINK_DATA_STREAM_IMG_PNG
        /// </summary>
        MavlinkDataStreamImgPng = 5,
    }

    /// <summary>
    ///  FENCE_ACTION
    /// </summary>
    public enum FenceAction
    {
        /// <summary>
        /// Disable fenced mode
        /// FENCE_ACTION_NONE
        /// </summary>
        FenceActionNone = 0,
        /// <summary>
        /// Switched to guided mode to return point (fence point 0)
        /// FENCE_ACTION_GUIDED
        /// </summary>
        FenceActionGuided = 1,
        /// <summary>
        /// Report fence breach, but don't take action
        /// FENCE_ACTION_REPORT
        /// </summary>
        FenceActionReport = 2,
        /// <summary>
        /// Switched to guided mode to return point (fence point 0) with manual throttle control
        /// FENCE_ACTION_GUIDED_THR_PASS
        /// </summary>
        FenceActionGuidedThrPass = 3,
        /// <summary>
        /// Switch to RTL (return to launch) mode and head for the return point.
        /// FENCE_ACTION_RTL
        /// </summary>
        FenceActionRtl = 4,
    }

    /// <summary>
    ///  FENCE_BREACH
    /// </summary>
    public enum FenceBreach
    {
        /// <summary>
        /// No last fence breach
        /// FENCE_BREACH_NONE
        /// </summary>
        FenceBreachNone = 0,
        /// <summary>
        /// Breached minimum altitude
        /// FENCE_BREACH_MINALT
        /// </summary>
        FenceBreachMinalt = 1,
        /// <summary>
        /// Breached maximum altitude
        /// FENCE_BREACH_MAXALT
        /// </summary>
        FenceBreachMaxalt = 2,
        /// <summary>
        /// Breached fence boundary
        /// FENCE_BREACH_BOUNDARY
        /// </summary>
        FenceBreachBoundary = 3,
    }

    /// <summary>
    /// Enumeration of possible mount operation modes
    ///  MAV_MOUNT_MODE
    /// </summary>
    public enum MavMountMode
    {
        /// <summary>
        /// Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
        /// MAV_MOUNT_MODE_RETRACT
        /// </summary>
        MavMountModeRetract = 0,
        /// <summary>
        /// Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
        /// MAV_MOUNT_MODE_NEUTRAL
        /// </summary>
        MavMountModeNeutral = 1,
        /// <summary>
        /// Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
        /// MAV_MOUNT_MODE_MAVLINK_TARGETING
        /// </summary>
        MavMountModeMavlinkTargeting = 2,
        /// <summary>
        /// Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
        /// MAV_MOUNT_MODE_RC_TARGETING
        /// </summary>
        MavMountModeRcTargeting = 3,
        /// <summary>
        /// Load neutral position and start to point to Lat,Lon,Alt
        /// MAV_MOUNT_MODE_GPS_POINT
        /// </summary>
        MavMountModeGpsPoint = 4,
    }

    /// <summary>
    /// Generalized UAVCAN node health
    ///  UAVCAN_NODE_HEALTH
    /// </summary>
    public enum UavcanNodeHealth
    {
        /// <summary>
        /// The node is functioning properly.
        /// UAVCAN_NODE_HEALTH_OK
        /// </summary>
        UavcanNodeHealthOk = 0,
        /// <summary>
        /// A critical parameter went out of range or the node has encountered a minor failure.
        /// UAVCAN_NODE_HEALTH_WARNING
        /// </summary>
        UavcanNodeHealthWarning = 1,
        /// <summary>
        /// The node has encountered a major failure.
        /// UAVCAN_NODE_HEALTH_ERROR
        /// </summary>
        UavcanNodeHealthError = 2,
        /// <summary>
        /// The node has suffered a fatal malfunction.
        /// UAVCAN_NODE_HEALTH_CRITICAL
        /// </summary>
        UavcanNodeHealthCritical = 3,
    }

    /// <summary>
    /// Generalized UAVCAN node mode
    ///  UAVCAN_NODE_MODE
    /// </summary>
    public enum UavcanNodeMode
    {
        /// <summary>
        /// The node is performing its primary functions.
        /// UAVCAN_NODE_MODE_OPERATIONAL
        /// </summary>
        UavcanNodeModeOperational = 0,
        /// <summary>
        /// The node is initializing; this mode is entered immediately after startup.
        /// UAVCAN_NODE_MODE_INITIALIZATION
        /// </summary>
        UavcanNodeModeInitialization = 1,
        /// <summary>
        /// The node is under maintenance.
        /// UAVCAN_NODE_MODE_MAINTENANCE
        /// </summary>
        UavcanNodeModeMaintenance = 2,
        /// <summary>
        /// The node is in the process of updating its software.
        /// UAVCAN_NODE_MODE_SOFTWARE_UPDATE
        /// </summary>
        UavcanNodeModeSoftwareUpdate = 3,
        /// <summary>
        /// The node is no longer available online.
        /// UAVCAN_NODE_MODE_OFFLINE
        /// </summary>
        UavcanNodeModeOffline = 7,
    }

    /// <summary>
    /// Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data.
    ///  MAV_CMD
    /// </summary>
    public enum MavCmd
    {
        /// <summary>
        /// Navigate to waypoint.
        /// Param 1 - Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
        /// Param 2 - Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
        /// Param 3 - 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
        /// Param 4 - Desired yaw angle at waypoint (rotary wing). NaN for unchanged.
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude
        /// MAV_CMD_NAV_WAYPOINT
        /// </summary>
        MavCmdNavWaypoint = 16,
        /// <summary>
        /// Loiter around this waypoint an unlimited amount of time
        /// Param 1 - Empty
        /// Param 2 - Empty
        /// Param 3 - Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
        /// Param 4 - Desired yaw angle.
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude
        /// MAV_CMD_NAV_LOITER_UNLIM
        /// </summary>
        MavCmdNavLoiterUnlim = 17,
        /// <summary>
        /// Loiter around this waypoint for X turns
        /// Param 1 - Turns
        /// Param 2 - Empty
        /// Param 3 - Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
        /// Param 4 - Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude
        /// MAV_CMD_NAV_LOITER_TURNS
        /// </summary>
        MavCmdNavLoiterTurns = 18,
        /// <summary>
        /// Loiter around this waypoint for X seconds
        /// Param 1 - Seconds (decimal)
        /// Param 2 - Empty
        /// Param 3 - Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
        /// Param 4 - Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude
        /// MAV_CMD_NAV_LOITER_TIME
        /// </summary>
        MavCmdNavLoiterTime = 19,
        /// <summary>
        /// Return to launch location
        /// Param 1 - Empty
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_NAV_RETURN_TO_LAUNCH
        /// </summary>
        MavCmdNavReturnToLaunch = 20,
        /// <summary>
        /// Land at location
        /// Param 1 - Abort Alt
        /// Param 2 - Precision land mode. (0 = normal landing, 1 = opportunistic precision landing, 2 = required precsion landing)
        /// Param 3 - Empty
        /// Param 4 - Desired yaw angle. NaN for unchanged.
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude (ground level)
        /// MAV_CMD_NAV_LAND
        /// </summary>
        MavCmdNavLand = 21,
        /// <summary>
        /// Takeoff from ground / hand
        /// Param 1 - Minimum pitch (if airspeed sensor present), desired pitch without sensor
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude
        /// MAV_CMD_NAV_TAKEOFF
        /// </summary>
        MavCmdNavTakeoff = 22,
        /// <summary>
        /// Land at local position (local frame only)
        /// Param 1 - Landing target number (if available)
        /// Param 2 - Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land
        /// Param 3 - Landing descend rate [ms^-1]
        /// Param 4 - Desired yaw angle [rad]
        /// Param 5 - Y-axis position [m]
        /// Param 6 - X-axis position [m]
        /// Param 7 - Z-axis / ground level position [m]
        /// MAV_CMD_NAV_LAND_LOCAL
        /// </summary>
        MavCmdNavLandLocal = 23,
        /// <summary>
        /// Takeoff from local position (local frame only)
        /// Param 1 - Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]
        /// Param 2 - Empty
        /// Param 3 - Takeoff ascend rate [ms^-1]
        /// Param 4 - Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these
        /// Param 5 - Y-axis position [m]
        /// Param 6 - X-axis position [m]
        /// Param 7 - Z-axis position [m]
        /// MAV_CMD_NAV_TAKEOFF_LOCAL
        /// </summary>
        MavCmdNavTakeoffLocal = 24,
        /// <summary>
        /// Vehicle following, i.e. this waypoint represents the position of a moving vehicle
        /// Param 1 - Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation
        /// Param 2 - Ground speed of vehicle to be followed
        /// Param 3 - Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
        /// Param 4 - Desired yaw angle.
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude
        /// MAV_CMD_NAV_FOLLOW
        /// </summary>
        MavCmdNavFollow = 25,
        /// <summary>
        /// Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached.
        /// Param 1 - Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude. 
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Desired altitude in meters
        /// MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT
        /// </summary>
        MavCmdNavContinueAndChangeAlt = 30,
        /// <summary>
        /// Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint. 
        /// Param 1 - Heading Required (0 = False)
        /// Param 2 - Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
        /// Param 3 - Empty
        /// Param 4 - Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude
        /// MAV_CMD_NAV_LOITER_TO_ALT
        /// </summary>
        MavCmdNavLoiterToAlt = 31,
        /// <summary>
        /// Being following a target
        /// Param 1 - System ID (the system ID of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode
        /// Param 2 - RESERVED
        /// Param 3 - RESERVED
        /// Param 4 - altitude flag: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home
        /// Param 5 - altitude
        /// Param 6 - RESERVED
        /// Param 7 - TTL in seconds in which the MAV should go to the default position hold mode after a message rx timeout
        /// MAV_CMD_DO_FOLLOW
        /// </summary>
        MavCmdDoFollow = 32,
        /// <summary>
        /// Reposition the MAV after a follow target command has been sent
        /// Param 1 - Camera q1 (where 0 is on the ray from the camera to the tracking device)
        /// Param 2 - Camera q2
        /// Param 3 - Camera q3
        /// Param 4 - Camera q4
        /// Param 5 - altitude offset from target (m)
        /// Param 6 - X offset from target (m)
        /// Param 7 - Y offset from target (m)
        /// MAV_CMD_DO_FOLLOW_REPOSITION
        /// </summary>
        MavCmdDoFollowReposition = 33,
        /// <summary>
        /// WIP: Start orbiting on the circumference of a circle defined by the parameters. Setting any value NaN results in using defaults.
        /// Param 1 - Radius of the circle in meters. positive: Orbit clockwise. negative: Orbit counter-clockwise. 
        /// Param 2 - Velocity tangential in m/s. NaN: Vehicle configuration default.
        /// Param 3 - Yaw behaviour of the vehicle. 0: vehicle front points to the center (default). 1: Hold last heading. 2: Leave yaw uncontrolled.
        /// Param 4 - Reserved (e.g. for dynamic center beacon options)
        /// Param 5 - Center point latitude / X coordinate. NaN: Use current vehicle position or current center if already orbiting.
        /// Param 6 - Center point longitude / Y coordinate.
        /// Param 7 - Center point altitude / Z coordinate.
        /// MAV_CMD_DO_ORBIT
        /// </summary>
        MavCmdDoOrbit = 34,
        /// <summary>
        /// THIS INTERFACE IS DEPRECATED AS OF JANUARY 2018. Please use MAV_CMD_DO_SET_ROI_* messages instead. Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
        /// Param 1 - Region of intereset mode. (see MAV_ROI enum)
        /// Param 2 - Waypoint index/ target ID. (see MAV_ROI enum)
        /// Param 3 - ROI index (allows a vehicle to manage multiple ROI's)
        /// Param 4 - Empty
        /// Param 5 - x the location of the fixed ROI (see MAV_FRAME)
        /// Param 6 - y
        /// Param 7 - z
        /// MAV_CMD_NAV_ROI
        /// </summary>
        MavCmdNavRoi = 80,
        /// <summary>
        /// Control autonomous path planning on the MAV.
        /// Param 1 - 0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning
        /// Param 2 - 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid
        /// Param 3 - Empty
        /// Param 4 - Yaw angle at goal, in compass degrees, [0..360]
        /// Param 5 - Latitude/X of goal
        /// Param 6 - Longitude/Y of goal
        /// Param 7 - Altitude/Z of goal
        /// MAV_CMD_NAV_PATHPLANNING
        /// </summary>
        MavCmdNavPathplanning = 81,
        /// <summary>
        /// Navigate to waypoint using a spline path.
        /// Param 1 - Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Latitude/X of goal
        /// Param 6 - Longitude/Y of goal
        /// Param 7 - Altitude/Z of goal
        /// MAV_CMD_NAV_SPLINE_WAYPOINT
        /// </summary>
        MavCmdNavSplineWaypoint = 82,
        /// <summary>
        /// Takeoff from ground using VTOL mode
        /// Param 1 - Empty
        /// Param 2 - Front transition heading, see VTOL_TRANSITION_HEADING enum.
        /// Param 3 - Empty
        /// Param 4 - Yaw angle in degrees. NaN for unchanged.
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude
        /// MAV_CMD_NAV_VTOL_TAKEOFF
        /// </summary>
        MavCmdNavVtolTakeoff = 84,
        /// <summary>
        /// Land using VTOL mode
        /// Param 1 - Empty
        /// Param 2 - Empty
        /// Param 3 - Approach altitude (with the same reference as the Altitude field). NaN if unspecified.
        /// Param 4 - Yaw angle in degrees. NaN for unchanged.
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude (ground level)
        /// MAV_CMD_NAV_VTOL_LAND
        /// </summary>
        MavCmdNavVtolLand = 85,
        /// <summary>
        /// hand control over to an external controller
        /// Param 1 - On / Off (> 0.5f on)
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_NAV_GUIDED_ENABLE
        /// </summary>
        MavCmdNavGuidedEnable = 92,
        /// <summary>
        /// Delay the next navigation command a number of seconds or until a specified time
        /// Param 1 - Delay in seconds (decimal, -1 to enable time-of-day fields)
        /// Param 2 - hour (24h format, UTC, -1 to ignore)
        /// Param 3 - minute (24h format, UTC, -1 to ignore)
        /// Param 4 - second (24h format, UTC)
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_NAV_DELAY
        /// </summary>
        MavCmdNavDelay = 93,
        /// <summary>
        /// Descend and place payload.  Vehicle descends until it detects a hanging payload has reached the ground, the gripper is opened to release the payload
        /// Param 1 - Maximum distance to descend (meters)
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Latitude (deg * 1E7)
        /// Param 6 - Longitude (deg * 1E7)
        /// Param 7 - Altitude (meters)
        /// MAV_CMD_NAV_PAYLOAD_PLACE
        /// </summary>
        MavCmdNavPayloadPlace = 94,
        /// <summary>
        /// NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
        /// Param 1 - Empty
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_NAV_LAST
        /// </summary>
        MavCmdNavLast = 95,
        /// <summary>
        /// Delay mission state machine.
        /// Param 1 - Delay in seconds (decimal)
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_CONDITION_DELAY
        /// </summary>
        MavCmdConditionDelay = 112,
        /// <summary>
        /// Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
        /// Param 1 - Descent / Ascend rate (m/s)
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Finish Altitude
        /// MAV_CMD_CONDITION_CHANGE_ALT
        /// </summary>
        MavCmdConditionChangeAlt = 113,
        /// <summary>
        /// Delay mission state machine until within desired distance of next NAV point.
        /// Param 1 - Distance (meters)
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_CONDITION_DISTANCE
        /// </summary>
        MavCmdConditionDistance = 114,
        /// <summary>
        /// Reach a certain target angle.
        /// Param 1 - target angle: [0-360], 0 is north
        /// Param 2 - speed during yaw change:[deg per second]
        /// Param 3 - direction: negative: counter clockwise, positive: clockwise [-1,1]
        /// Param 4 - relative offset or absolute angle: [ 1,0]
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_CONDITION_YAW
        /// </summary>
        MavCmdConditionYaw = 115,
        /// <summary>
        /// NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
        /// Param 1 - Empty
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_CONDITION_LAST
        /// </summary>
        MavCmdConditionLast = 159,
        /// <summary>
        /// Set system mode.
        /// Param 1 - Mode, as defined by ENUM MAV_MODE
        /// Param 2 - Custom mode - this is system specific, please refer to the individual autopilot specifications for details.
        /// Param 3 - Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_SET_MODE
        /// </summary>
        MavCmdDoSetMode = 176,
        /// <summary>
        /// Jump to the desired command in the mission list.  Repeat this action only the specified number of times
        /// Param 1 - Sequence number
        /// Param 2 - Repeat count
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_JUMP
        /// </summary>
        MavCmdDoJump = 177,
        /// <summary>
        /// Change speed and/or throttle set points.
        /// Param 1 - Speed type (0=Airspeed, 1=Ground Speed)
        /// Param 2 - Speed  (m/s, -1 indicates no change)
        /// Param 3 - Throttle  ( Percent, -1 indicates no change)
        /// Param 4 - absolute or relative [0,1]
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_CHANGE_SPEED
        /// </summary>
        MavCmdDoChangeSpeed = 178,
        /// <summary>
        /// Changes the home location either to the current location or a specified location.
        /// Param 1 - Use current (1=use current location, 0=use specified location)
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude
        /// MAV_CMD_DO_SET_HOME
        /// </summary>
        MavCmdDoSetHome = 179,
        /// <summary>
        /// Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.
        /// Param 1 - Parameter number
        /// Param 2 - Parameter value
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_SET_PARAMETER
        /// </summary>
        MavCmdDoSetParameter = 180,
        /// <summary>
        /// Set a relay to a condition.
        /// Param 1 - Relay number
        /// Param 2 - Setting (1=on, 0=off, others possible depending on system hardware)
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_SET_RELAY
        /// </summary>
        MavCmdDoSetRelay = 181,
        /// <summary>
        /// Cycle a relay on and off for a desired number of cyles with a desired period.
        /// Param 1 - Relay number
        /// Param 2 - Cycle count
        /// Param 3 - Cycle time (seconds, decimal)
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_REPEAT_RELAY
        /// </summary>
        MavCmdDoRepeatRelay = 182,
        /// <summary>
        /// Set a servo to a desired PWM value.
        /// Param 1 - Servo number
        /// Param 2 - PWM (microseconds, 1000 to 2000 typical)
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_SET_SERVO
        /// </summary>
        MavCmdDoSetServo = 183,
        /// <summary>
        /// Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
        /// Param 1 - Servo number
        /// Param 2 - PWM (microseconds, 1000 to 2000 typical)
        /// Param 3 - Cycle count
        /// Param 4 - Cycle time (seconds)
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_REPEAT_SERVO
        /// </summary>
        MavCmdDoRepeatServo = 184,
        /// <summary>
        /// Terminate flight immediately
        /// Param 1 - Flight termination activated if > 0.5
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_FLIGHTTERMINATION
        /// </summary>
        MavCmdDoFlighttermination = 185,
        /// <summary>
        /// Change altitude set point.
        /// Param 1 - Altitude in meters
        /// Param 2 - Mav frame of new altitude (see MAV_FRAME)
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_CHANGE_ALTITUDE
        /// </summary>
        MavCmdDoChangeAltitude = 186,
        /// <summary>
        /// Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence.
        /// Param 1 - Empty
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Empty
        /// MAV_CMD_DO_LAND_START
        /// </summary>
        MavCmdDoLandStart = 189,
        /// <summary>
        /// Mission command to perform a landing from a rally point.
        /// Param 1 - Break altitude (meters)
        /// Param 2 - Landing speed (m/s)
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_RALLY_LAND
        /// </summary>
        MavCmdDoRallyLand = 190,
        /// <summary>
        /// Mission command to safely abort an autonmous landing.
        /// Param 1 - Altitude (meters)
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_GO_AROUND
        /// </summary>
        MavCmdDoGoAround = 191,
        /// <summary>
        /// Reposition the vehicle to a specific WGS84 global position.
        /// Param 1 - Ground speed, less than 0 (-1) for default
        /// Param 2 - Bitmask of option flags, see the MAV_DO_REPOSITION_FLAGS enum.
        /// Param 3 - Reserved
        /// Param 4 - Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)
        /// Param 5 - Latitude (deg * 1E7)
        /// Param 6 - Longitude (deg * 1E7)
        /// Param 7 - Altitude (meters)
        /// MAV_CMD_DO_REPOSITION
        /// </summary>
        MavCmdDoReposition = 192,
        /// <summary>
        /// If in a GPS controlled position mode, hold the current position or continue.
        /// Param 1 - 0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.
        /// Param 2 - Reserved
        /// Param 3 - Reserved
        /// Param 4 - Reserved
        /// Param 5 - Reserved
        /// Param 6 - Reserved
        /// Param 7 - Reserved
        /// MAV_CMD_DO_PAUSE_CONTINUE
        /// </summary>
        MavCmdDoPauseContinue = 193,
        /// <summary>
        /// Set moving direction to forward or reverse.
        /// Param 1 - Direction (0=Forward, 1=Reverse)
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_SET_REVERSE
        /// </summary>
        MavCmdDoSetReverse = 194,
        /// <summary>
        /// Sets the region of interest (ROI) to a location. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
        /// Param 1 - Empty
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude
        /// MAV_CMD_DO_SET_ROI_LOCATION
        /// </summary>
        MavCmdDoSetRoiLocation = 195,
        /// <summary>
        /// Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
        /// Param 1 - Empty
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - pitch offset from next waypoint
        /// Param 6 - roll offset from next waypoint
        /// Param 7 - yaw offset from next waypoint
        /// MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET
        /// </summary>
        MavCmdDoSetRoiWpnextOffset = 196,
        /// <summary>
        /// Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
        /// Param 1 - Empty
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_SET_ROI_NONE
        /// </summary>
        MavCmdDoSetRoiNone = 197,
        /// <summary>
        /// Control onboard camera system.
        /// Param 1 - Camera ID (-1 for all)
        /// Param 2 - Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw
        /// Param 3 - Transmission mode: 0: video stream, >0: single images every n seconds (decimal)
        /// Param 4 - Recording: 0: disabled, 1: enabled compressed, 2: enabled raw
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_CONTROL_VIDEO
        /// </summary>
        MavCmdDoControlVideo = 200,
        /// <summary>
        /// THIS INTERFACE IS DEPRECATED AS OF JANUARY 2018. Please use MAV_CMD_DO_SET_ROI_* messages instead. Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
        /// Param 1 - Region of intereset mode. (see MAV_ROI enum)
        /// Param 2 - Waypoint index/ target ID. (see MAV_ROI enum)
        /// Param 3 - ROI index (allows a vehicle to manage multiple ROI's)
        /// Param 4 - Empty
        /// Param 5 - MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude
        /// Param 6 - MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude
        /// Param 7 - MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude
        /// MAV_CMD_DO_SET_ROI
        /// </summary>
        MavCmdDoSetRoi = 201,
        /// <summary>
        /// THIS INTERFACE IS DEPRECATED since 2018-01. Please use PARAM_EXT_XXX messages and the camera definition format described in https://mavlink.io/en/protocol/camera_def.html.
        /// Param 1 - Modes: P, TV, AV, M, Etc
        /// Param 2 - Shutter speed: Divisor number for one second
        /// Param 3 - Aperture: F stop number
        /// Param 4 - ISO number e.g. 80, 100, 200, Etc
        /// Param 5 - Exposure type enumerator
        /// Param 6 - Command Identity
        /// Param 7 - Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
        /// MAV_CMD_DO_DIGICAM_CONFIGURE
        /// </summary>
        MavCmdDoDigicamConfigure = 202,
        /// <summary>
        /// THIS INTERFACE IS DEPRECATED since 2018-01. Please use PARAM_EXT_XXX messages and the camera definition format described in https://mavlink.io/en/protocol/camera_def.html.
        /// Param 1 - Session control e.g. show/hide lens
        /// Param 2 - Zoom's absolute position
        /// Param 3 - Zooming step value to offset zoom from the current position
        /// Param 4 - Focus Locking, Unlocking or Re-locking
        /// Param 5 - Shooting Command
        /// Param 6 - Command Identity
        /// Param 7 - Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.
        /// MAV_CMD_DO_DIGICAM_CONTROL
        /// </summary>
        MavCmdDoDigicamControl = 203,
        /// <summary>
        /// Mission command to configure a camera or antenna mount
        /// Param 1 - Mount operation mode (see MAV_MOUNT_MODE enum)
        /// Param 2 - stabilize roll? (1 = yes, 0 = no)
        /// Param 3 - stabilize pitch? (1 = yes, 0 = no)
        /// Param 4 - stabilize yaw? (1 = yes, 0 = no)
        /// Param 5 - roll input (0 = angle, 1 = angular rate)
        /// Param 6 - pitch input (0 = angle, 1 = angular rate)
        /// Param 7 - yaw input (0 = angle, 1 = angular rate)
        /// MAV_CMD_DO_MOUNT_CONFIGURE
        /// </summary>
        MavCmdDoMountConfigure = 204,
        /// <summary>
        /// Mission command to control a camera or antenna mount
        /// Param 1 - pitch depending on mount mode (degrees or degrees/second depending on pitch input).
        /// Param 2 - roll depending on mount mode (degrees or degrees/second depending on roll input).
        /// Param 3 - yaw depending on mount mode (degrees or degrees/second depending on yaw input).
        /// Param 4 - alt in meters depending on mount mode.
        /// Param 5 - latitude in degrees * 1E7, set if appropriate mount mode.
        /// Param 6 - longitude in degrees * 1E7, set if appropriate mount mode.
        /// Param 7 - MAV_MOUNT_MODE enum value
        /// MAV_CMD_DO_MOUNT_CONTROL
        /// </summary>
        MavCmdDoMountControl = 205,
        /// <summary>
        /// Mission command to set camera trigger distance for this flight. The camera is trigerred each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera.
        /// Param 1 - Camera trigger distance (meters). 0 to stop triggering.
        /// Param 2 - Camera shutter integration time (milliseconds). -1 or 0 to ignore
        /// Param 3 - Trigger camera once immediately. (0 = no trigger, 1 = trigger)
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_SET_CAM_TRIGG_DIST
        /// </summary>
        MavCmdDoSetCamTriggDist = 206,
        /// <summary>
        /// Mission command to enable the geofence
        /// Param 1 - enable? (0=disable, 1=enable, 2=disable_floor_only)
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_FENCE_ENABLE
        /// </summary>
        MavCmdDoFenceEnable = 207,
        /// <summary>
        /// Mission command to trigger a parachute
        /// Param 1 - action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_PARACHUTE
        /// </summary>
        MavCmdDoParachute = 208,
        /// <summary>
        /// Mission command to perform motor test
        /// Param 1 - motor number (a number from 1 to max number of motors on the vehicle)
        /// Param 2 - throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
        /// Param 3 - throttle
        /// Param 4 - timeout (in seconds)
        /// Param 5 - motor count (number of motors to test to test in sequence, waiting for the timeout above between them; 0=1 motor, 1=1 motor, 2=2 motors...)
        /// Param 6 - motor test order (See MOTOR_TEST_ORDER enum)
        /// Param 7 - Empty
        /// MAV_CMD_DO_MOTOR_TEST
        /// </summary>
        MavCmdDoMotorTest = 209,
        /// <summary>
        /// Change to/from inverted flight
        /// Param 1 - inverted (0=normal, 1=inverted)
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_INVERTED_FLIGHT
        /// </summary>
        MavCmdDoInvertedFlight = 210,
        /// <summary>
        /// Sets a desired vehicle turn angle and speed change
        /// Param 1 - yaw angle to adjust steering by in centidegress
        /// Param 2 - speed - normalized to 0 .. 1
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_NAV_SET_YAW_SPEED
        /// </summary>
        MavCmdNavSetYawSpeed = 213,
        /// <summary>
        /// Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera.
        /// Param 1 - Camera trigger cycle time (milliseconds). -1 or 0 to ignore.
        /// Param 2 - Camera shutter integration time (milliseconds). Should be less than trigger cycle time. -1 or 0 to ignore.
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL
        /// </summary>
        MavCmdDoSetCamTriggInterval = 214,
        /// <summary>
        /// Mission command to control a camera or antenna mount, using a quaternion as reference.
        /// Param 1 - q1 - quaternion param #1, w (1 in null-rotation)
        /// Param 2 - q2 - quaternion param #2, x (0 in null-rotation)
        /// Param 3 - q3 - quaternion param #3, y (0 in null-rotation)
        /// Param 4 - q4 - quaternion param #4, z (0 in null-rotation)
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_MOUNT_CONTROL_QUAT
        /// </summary>
        MavCmdDoMountControlQuat = 220,
        /// <summary>
        /// set id of master controller
        /// Param 1 - System ID
        /// Param 2 - Component ID
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_GUIDED_MASTER
        /// </summary>
        MavCmdDoGuidedMaster = 221,
        /// <summary>
        /// set limits for external control
        /// Param 1 - timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout
        /// Param 2 - absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit
        /// Param 3 - absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit
        /// Param 4 - horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_GUIDED_LIMITS
        /// </summary>
        MavCmdDoGuidedLimits = 222,
        /// <summary>
        /// Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines
        /// Param 1 - 0: Stop engine, 1:Start Engine
        /// Param 2 - 0: Warm start, 1:Cold start. Controls use of choke where applicable
        /// Param 3 - Height delay (meters). This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_ENGINE_CONTROL
        /// </summary>
        MavCmdDoEngineControl = 223,
        /// <summary>
        /// NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
        /// Param 1 - Empty
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_LAST
        /// </summary>
        MavCmdDoLast = 240,
        /// <summary>
        /// Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero.
        /// Param 1 - 1: gyro calibration, 3: gyro temperature calibration
        /// Param 2 - 1: magnetometer calibration
        /// Param 3 - 1: ground pressure calibration
        /// Param 4 - 1: radio RC calibration, 2: RC trim calibration
        /// Param 5 - 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
        /// Param 6 - 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration
        /// Param 7 - 1: ESC calibration, 3: barometer temperature calibration
        /// MAV_CMD_PREFLIGHT_CALIBRATION
        /// </summary>
        MavCmdPreflightCalibration = 241,
        /// <summary>
        /// Set sensor offsets. This command will be only accepted if in pre-flight mode.
        /// Param 1 - Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer
        /// Param 2 - X axis offset (or generic dimension 1), in the sensor's raw units
        /// Param 3 - Y axis offset (or generic dimension 2), in the sensor's raw units
        /// Param 4 - Z axis offset (or generic dimension 3), in the sensor's raw units
        /// Param 5 - Generic dimension 4, in the sensor's raw units
        /// Param 6 - Generic dimension 5, in the sensor's raw units
        /// Param 7 - Generic dimension 6, in the sensor's raw units
        /// MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS
        /// </summary>
        MavCmdPreflightSetSensorOffsets = 242,
        /// <summary>
        /// Trigger UAVCAN config. This command will be only accepted if in pre-flight mode.
        /// Param 1 - 1: Trigger actuator ID assignment and direction mapping.
        /// Param 2 - Reserved
        /// Param 3 - Reserved
        /// Param 4 - Reserved
        /// Param 5 - Reserved
        /// Param 6 - Reserved
        /// Param 7 - Reserved
        /// MAV_CMD_PREFLIGHT_UAVCAN
        /// </summary>
        MavCmdPreflightUavcan = 243,
        /// <summary>
        /// Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
        /// Param 1 - Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
        /// Param 2 - Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
        /// Param 3 - Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz logging)
        /// Param 4 - Reserved
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_PREFLIGHT_STORAGE
        /// </summary>
        MavCmdPreflightStorage = 245,
        /// <summary>
        /// Request the reboot or shutdown of system components.
        /// Param 1 - 0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.
        /// Param 2 - 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.
        /// Param 3 - WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded
        /// Param 4 - WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded
        /// Param 5 - Reserved, send 0
        /// Param 6 - Reserved, send 0
        /// Param 7 - WIP: ID (e.g. camera ID -1 for all IDs)
        /// MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
        /// </summary>
        MavCmdPreflightRebootShutdown = 246,
        /// <summary>
        /// Hold / continue the current action
        /// Param 1 - MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan
        /// Param 2 - MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position
        /// Param 3 - MAV_FRAME coordinate frame of hold point
        /// Param 4 - Desired yaw angle in degrees
        /// Param 5 - Latitude / X position
        /// Param 6 - Longitude / Y position
        /// Param 7 - Altitude / Z position
        /// MAV_CMD_OVERRIDE_GOTO
        /// </summary>
        MavCmdOverrideGoto = 252,
        /// <summary>
        /// start running a mission
        /// Param 1 - first_item: the first mission item to run
        /// Param 2 - last_item:  the last mission item to run (after this item is run, the mission ends)
        /// MAV_CMD_MISSION_START
        /// </summary>
        MavCmdMissionStart = 300,
        /// <summary>
        /// Arms / Disarms a component
        /// Param 1 - 1 to arm, 0 to disarm
        /// MAV_CMD_COMPONENT_ARM_DISARM
        /// </summary>
        MavCmdComponentArmDisarm = 400,
        /// <summary>
        /// Request the home position from the vehicle.
        /// Param 1 - Reserved
        /// Param 2 - Reserved
        /// Param 3 - Reserved
        /// Param 4 - Reserved
        /// Param 5 - Reserved
        /// Param 6 - Reserved
        /// Param 7 - Reserved
        /// MAV_CMD_GET_HOME_POSITION
        /// </summary>
        MavCmdGetHomePosition = 410,
        /// <summary>
        /// Starts receiver pairing
        /// Param 1 - 0:Spektrum
        /// Param 2 - RC type (see RC_TYPE enum)
        /// MAV_CMD_START_RX_PAIR
        /// </summary>
        MavCmdStartRxPair = 500,
        /// <summary>
        /// Request the interval between messages for a particular MAVLink message ID
        /// Param 1 - The MAVLink message ID
        /// MAV_CMD_GET_MESSAGE_INTERVAL
        /// </summary>
        MavCmdGetMessageInterval = 510,
        /// <summary>
        /// Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM
        /// Param 1 - The MAVLink message ID
        /// Param 2 - The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate.
        /// MAV_CMD_SET_MESSAGE_INTERVAL
        /// </summary>
        MavCmdSetMessageInterval = 511,
        /// <summary>
        /// Request MAVLink protocol version compatibility
        /// Param 1 - 1: Request supported protocol versions by all nodes on the network
        /// Param 2 - Reserved (all remaining params)
        /// MAV_CMD_REQUEST_PROTOCOL_VERSION
        /// </summary>
        MavCmdRequestProtocolVersion = 519,
        /// <summary>
        /// Request autopilot capabilities
        /// Param 1 - 1: Request autopilot version
        /// Param 2 - Reserved (all remaining params)
        /// MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
        /// </summary>
        MavCmdRequestAutopilotCapabilities = 520,
        /// <summary>
        /// Request camera information (CAMERA_INFORMATION).
        /// Param 1 - 0: No action 1: Request camera capabilities
        /// Param 2 - Reserved (all remaining params)
        /// MAV_CMD_REQUEST_CAMERA_INFORMATION
        /// </summary>
        MavCmdRequestCameraInformation = 521,
        /// <summary>
        /// Request camera settings (CAMERA_SETTINGS).
        /// Param 1 - 0: No Action 1: Request camera settings
        /// Param 2 - Reserved (all remaining params)
        /// MAV_CMD_REQUEST_CAMERA_SETTINGS
        /// </summary>
        MavCmdRequestCameraSettings = 522,
        /// <summary>
        /// WIP: Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage.
        /// Param 1 - Storage ID (0 for all, 1 for first, 2 for second, etc.)
        /// Param 2 - 0: No Action 1: Request storage information
        /// Param 3 - Reserved (all remaining params)
        /// MAV_CMD_REQUEST_STORAGE_INFORMATION
        /// </summary>
        MavCmdRequestStorageInformation = 525,
        /// <summary>
        /// WIP: Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage.
        /// Param 1 - Storage ID (1 for first, 2 for second, etc.)
        /// Param 2 - 0: No action 1: Format storage
        /// Param 3 - Reserved (all remaining params)
        /// MAV_CMD_STORAGE_FORMAT
        /// </summary>
        MavCmdStorageFormat = 526,
        /// <summary>
        /// Request camera capture status (CAMERA_CAPTURE_STATUS)
        /// Param 1 - 0: No Action 1: Request camera capture status
        /// Param 2 - Reserved (all remaining params)
        /// MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS
        /// </summary>
        MavCmdRequestCameraCaptureStatus = 527,
        /// <summary>
        /// WIP: Request flight information (FLIGHT_INFORMATION)
        /// Param 1 - 1: Request flight information
        /// Param 2 - Reserved (all remaining params)
        /// MAV_CMD_REQUEST_FLIGHT_INFORMATION
        /// </summary>
        MavCmdRequestFlightInformation = 528,
        /// <summary>
        /// Reset all camera settings to Factory Default
        /// Param 1 - 0: No Action 1: Reset all settings
        /// Param 2 - Reserved (all remaining params)
        /// MAV_CMD_RESET_CAMERA_SETTINGS
        /// </summary>
        MavCmdResetCameraSettings = 529,
        /// <summary>
        /// Set camera running mode. Use NAN for reserved values.
        /// Param 1 - Reserved (Set to 0)
        /// Param 2 - Camera mode (see CAMERA_MODE enum)
        /// Param 3 - Reserved (all remaining params)
        /// MAV_CMD_SET_CAMERA_MODE
        /// </summary>
        MavCmdSetCameraMode = 530,
        /// <summary>
        /// Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NAN for reserved values.
        /// Param 1 - Reserved (Set to 0)
        /// Param 2 - Duration between two consecutive pictures (in seconds)
        /// Param 3 - Number of images to capture total - 0 for unlimited capture
        /// Param 4 - Capture sequence (ID to prevent double captures when a command is retransmitted, 0: unused, >= 1: used)
        /// Param 5 - Reserved (all remaining params)
        /// MAV_CMD_IMAGE_START_CAPTURE
        /// </summary>
        MavCmdImageStartCapture = 2000,
        /// <summary>
        /// Stop image capture sequence Use NAN for reserved values.
        /// Param 1 - Reserved (Set to 0)
        /// Param 2 - Reserved (all remaining params)
        /// MAV_CMD_IMAGE_STOP_CAPTURE
        /// </summary>
        MavCmdImageStopCapture = 2001,
        /// <summary>
        /// WIP: Re-request a CAMERA_IMAGE_CAPTURE packet. Use NAN for reserved values.
        /// Param 1 - Sequence number for missing CAMERA_IMAGE_CAPTURE packet
        /// Param 2 - Reserved (all remaining params)
        /// MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE
        /// </summary>
        MavCmdRequestCameraImageCapture = 2002,
        /// <summary>
        /// Enable or disable on-board camera triggering system.
        /// Param 1 - Trigger enable/disable (0 for disable, 1 for start), -1 to ignore
        /// Param 2 - 1 to reset the trigger sequence, -1 or 0 to ignore
        /// Param 3 - 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore
        /// MAV_CMD_DO_TRIGGER_CONTROL
        /// </summary>
        MavCmdDoTriggerControl = 2003,
        /// <summary>
        /// Starts video capture (recording). Use NAN for reserved values.
        /// Param 1 - Reserved (Set to 0)
        /// Param 2 - Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency in Hz)
        /// Param 3 - Reserved (all remaining params)
        /// MAV_CMD_VIDEO_START_CAPTURE
        /// </summary>
        MavCmdVideoStartCapture = 2500,
        /// <summary>
        /// Stop the current video capture (recording). Use NAN for reserved values.
        /// Param 1 - Reserved (Set to 0)
        /// Param 2 - Reserved (all remaining params)
        /// MAV_CMD_VIDEO_STOP_CAPTURE
        /// </summary>
        MavCmdVideoStopCapture = 2501,
        /// <summary>
        /// WIP: Start video streaming
        /// Param 1 - Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
        /// Param 2 - Reserved
        /// MAV_CMD_VIDEO_START_STREAMING
        /// </summary>
        MavCmdVideoStartStreaming = 2502,
        /// <summary>
        /// WIP: Stop the current video streaming
        /// Param 1 - Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
        /// Param 2 - Reserved
        /// MAV_CMD_VIDEO_STOP_STREAMING
        /// </summary>
        MavCmdVideoStopStreaming = 2503,
        /// <summary>
        /// WIP: Request video stream information (VIDEO_STREAM_INFORMATION)
        /// Param 1 - Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
        /// Param 2 - 0: No Action 1: Request video stream information
        /// Param 3 - Reserved (all remaining params)
        /// MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION
        /// </summary>
        MavCmdRequestVideoStreamInformation = 2504,
        /// <summary>
        /// Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
        /// Param 1 - Format: 0: ULog
        /// Param 2 - Reserved (set to 0)
        /// Param 3 - Reserved (set to 0)
        /// Param 4 - Reserved (set to 0)
        /// Param 5 - Reserved (set to 0)
        /// Param 6 - Reserved (set to 0)
        /// Param 7 - Reserved (set to 0)
        /// MAV_CMD_LOGGING_START
        /// </summary>
        MavCmdLoggingStart = 2510,
        /// <summary>
        /// Request to stop streaming log data over MAVLink
        /// Param 1 - Reserved (set to 0)
        /// Param 2 - Reserved (set to 0)
        /// Param 3 - Reserved (set to 0)
        /// Param 4 - Reserved (set to 0)
        /// Param 5 - Reserved (set to 0)
        /// Param 6 - Reserved (set to 0)
        /// Param 7 - Reserved (set to 0)
        /// MAV_CMD_LOGGING_STOP
        /// </summary>
        MavCmdLoggingStop = 2511,
        /// <summary>
        /// 
        /// Param 1 - Landing gear ID (default: 0, -1 for all)
        /// Param 2 - Landing gear position (Down: 0, Up: 1, NAN for no change)
        /// Param 3 - Reserved, set to NAN
        /// Param 4 - Reserved, set to NAN
        /// Param 5 - Reserved, set to NAN
        /// Param 6 - Reserved, set to NAN
        /// Param 7 - Reserved, set to NAN
        /// MAV_CMD_AIRFRAME_CONFIGURATION
        /// </summary>
        MavCmdAirframeConfiguration = 2520,
        /// <summary>
        /// Request to start/stop transmitting over the high latency telemetry
        /// Param 1 - Control transmittion over high latency telemetry (0: stop, 1: start)
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_CONTROL_HIGH_LATENCY
        /// </summary>
        MavCmdControlHighLatency = 2600,
        /// <summary>
        /// Create a panorama at the current position
        /// Param 1 - Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)
        /// Param 2 - Viewing angle vertical of panorama (in degrees)
        /// Param 3 - Speed of the horizontal rotation (in degrees per second)
        /// Param 4 - Speed of the vertical rotation (in degrees per second)
        /// MAV_CMD_PANORAMA_CREATE
        /// </summary>
        MavCmdPanoramaCreate = 2800,
        /// <summary>
        /// Request VTOL transition
        /// Param 1 - The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.
        /// MAV_CMD_DO_VTOL_TRANSITION
        /// </summary>
        MavCmdDoVtolTransition = 3000,
        /// <summary>
        /// This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocites along all three axes.
        ///                   
        /// MAV_CMD_SET_GUIDED_SUBMODE_STANDARD
        /// </summary>
        MavCmdSetGuidedSubmodeStandard = 4000,
        /// <summary>
        /// This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
        ///                   
        /// Param 1 - Radius of desired circle in CIRCLE_MODE
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - Unscaled target latitude of center of circle in CIRCLE_MODE
        /// Param 6 - Unscaled target longitude of center of circle in CIRCLE_MODE
        /// MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE
        /// </summary>
        MavCmdSetGuidedSubmodeCircle = 4001,
        /// <summary>
        /// WIP: Delay mission state machine until gate has been reached.
        /// Param 1 - Geometry: 0: orthogonal to path between previous and next waypoint.
        /// Param 2 - Altitude: 0: ignore altitude
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude
        /// MAV_CMD_CONDITION_GATE
        /// </summary>
        MavCmdConditionGate = 4501,
        /// <summary>
        /// Request authorization to arm the vehicle to a external entity, the arm authorizer is resposible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
        ///         
        /// Param 1 - Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle
        /// MAV_CMD_ARM_AUTHORIZATION_REQUEST
        /// </summary>
        MavCmdArmAuthorizationRequest = 3001,
        /// <summary>
        /// Fence return point. There can only be one fence return point.
        ///         
        /// Param 1 - Reserved
        /// Param 2 - Reserved
        /// Param 3 - Reserved
        /// Param 4 - Reserved
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude
        /// MAV_CMD_NAV_FENCE_RETURN_POINT
        /// </summary>
        MavCmdNavFenceReturnPoint = 5000,
        /// <summary>
        /// Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
        ///         
        /// Param 1 - Polygon vertex count
        /// Param 2 - Reserved
        /// Param 3 - Reserved
        /// Param 4 - Reserved
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Reserved
        /// MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        /// </summary>
        MavCmdNavFencePolygonVertexInclusion = 5001,
        /// <summary>
        /// Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
        ///         
        /// Param 1 - Polygon vertex count
        /// Param 2 - Reserved
        /// Param 3 - Reserved
        /// Param 4 - Reserved
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Reserved
        /// MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
        /// </summary>
        MavCmdNavFencePolygonVertexExclusion = 5002,
        /// <summary>
        /// Circular fence area. The vehicle must stay inside this area.
        ///         
        /// Param 1 - radius in meters
        /// Param 2 - Reserved
        /// Param 3 - Reserved
        /// Param 4 - Reserved
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Reserved
        /// MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
        /// </summary>
        MavCmdNavFenceCircleInclusion = 5003,
        /// <summary>
        /// Circular fence area. The vehicle must stay outside this area.
        ///         
        /// Param 1 - radius in meters
        /// Param 2 - Reserved
        /// Param 3 - Reserved
        /// Param 4 - Reserved
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Reserved
        /// MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
        /// </summary>
        MavCmdNavFenceCircleExclusion = 5004,
        /// <summary>
        /// Rally point. You can have multiple rally points defined.
        ///         
        /// Param 1 - Reserved
        /// Param 2 - Reserved
        /// Param 3 - Reserved
        /// Param 4 - Reserved
        /// Param 5 - Latitude
        /// Param 6 - Longitude
        /// Param 7 - Altitude
        /// MAV_CMD_NAV_RALLY_POINT
        /// </summary>
        MavCmdNavRallyPoint = 5100,
        /// <summary>
        /// Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages.
        /// Param 1 - Reserved (set to 0)
        /// Param 2 - Reserved (set to 0)
        /// Param 3 - Reserved (set to 0)
        /// Param 4 - Reserved (set to 0)
        /// Param 5 - Reserved (set to 0)
        /// Param 6 - Reserved (set to 0)
        /// Param 7 - Reserved (set to 0)
        /// MAV_CMD_UAVCAN_GET_NODE_INFO
        /// </summary>
        MavCmdUavcanGetNodeInfo = 5200,
        /// <summary>
        /// Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity.
        /// Param 1 - Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.
        /// Param 2 - Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.
        /// Param 3 - Desired ground speed at release time. This can be overriden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.
        /// Param 4 - Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.
        /// Param 5 - Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
        /// Param 6 - Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
        /// Param 7 - Altitude, in meters AMSL
        /// MAV_CMD_PAYLOAD_PREPARE_DEPLOY
        /// </summary>
        MavCmdPayloadPrepareDeploy = 30001,
        /// <summary>
        /// Control the payload deployment.
        /// Param 1 - Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.
        /// Param 2 - Reserved
        /// Param 3 - Reserved
        /// Param 4 - Reserved
        /// Param 5 - Reserved
        /// Param 6 - Reserved
        /// Param 7 - Reserved
        /// MAV_CMD_PAYLOAD_CONTROL_DEPLOY
        /// </summary>
        MavCmdPayloadControlDeploy = 30002,
        /// <summary>
        /// User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - Latitude unscaled
        /// Param 6 - Longitude unscaled
        /// Param 7 - Altitude, in meters AMSL
        /// MAV_CMD_WAYPOINT_USER_1
        /// </summary>
        MavCmdWaypointUser1 = 31000,
        /// <summary>
        /// User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - Latitude unscaled
        /// Param 6 - Longitude unscaled
        /// Param 7 - Altitude, in meters AMSL
        /// MAV_CMD_WAYPOINT_USER_2
        /// </summary>
        MavCmdWaypointUser2 = 31001,
        /// <summary>
        /// User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - Latitude unscaled
        /// Param 6 - Longitude unscaled
        /// Param 7 - Altitude, in meters AMSL
        /// MAV_CMD_WAYPOINT_USER_3
        /// </summary>
        MavCmdWaypointUser3 = 31002,
        /// <summary>
        /// User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - Latitude unscaled
        /// Param 6 - Longitude unscaled
        /// Param 7 - Altitude, in meters AMSL
        /// MAV_CMD_WAYPOINT_USER_4
        /// </summary>
        MavCmdWaypointUser4 = 31003,
        /// <summary>
        /// User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - Latitude unscaled
        /// Param 6 - Longitude unscaled
        /// Param 7 - Altitude, in meters AMSL
        /// MAV_CMD_WAYPOINT_USER_5
        /// </summary>
        MavCmdWaypointUser5 = 31004,
        /// <summary>
        /// User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - Latitude unscaled
        /// Param 6 - Longitude unscaled
        /// Param 7 - Altitude, in meters AMSL
        /// MAV_CMD_SPATIAL_USER_1
        /// </summary>
        MavCmdSpatialUser1 = 31005,
        /// <summary>
        /// User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - Latitude unscaled
        /// Param 6 - Longitude unscaled
        /// Param 7 - Altitude, in meters AMSL
        /// MAV_CMD_SPATIAL_USER_2
        /// </summary>
        MavCmdSpatialUser2 = 31006,
        /// <summary>
        /// User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - Latitude unscaled
        /// Param 6 - Longitude unscaled
        /// Param 7 - Altitude, in meters AMSL
        /// MAV_CMD_SPATIAL_USER_3
        /// </summary>
        MavCmdSpatialUser3 = 31007,
        /// <summary>
        /// User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - Latitude unscaled
        /// Param 6 - Longitude unscaled
        /// Param 7 - Altitude, in meters AMSL
        /// MAV_CMD_SPATIAL_USER_4
        /// </summary>
        MavCmdSpatialUser4 = 31008,
        /// <summary>
        /// User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - Latitude unscaled
        /// Param 6 - Longitude unscaled
        /// Param 7 - Altitude, in meters AMSL
        /// MAV_CMD_SPATIAL_USER_5
        /// </summary>
        MavCmdSpatialUser5 = 31009,
        /// <summary>
        /// User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - User defined
        /// Param 6 - User defined
        /// Param 7 - User defined
        /// MAV_CMD_USER_1
        /// </summary>
        MavCmdUser1 = 31010,
        /// <summary>
        /// User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - User defined
        /// Param 6 - User defined
        /// Param 7 - User defined
        /// MAV_CMD_USER_2
        /// </summary>
        MavCmdUser2 = 31011,
        /// <summary>
        /// User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - User defined
        /// Param 6 - User defined
        /// Param 7 - User defined
        /// MAV_CMD_USER_3
        /// </summary>
        MavCmdUser3 = 31012,
        /// <summary>
        /// User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - User defined
        /// Param 6 - User defined
        /// Param 7 - User defined
        /// MAV_CMD_USER_4
        /// </summary>
        MavCmdUser4 = 31013,
        /// <summary>
        /// User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
        /// Param 1 - User defined
        /// Param 2 - User defined
        /// Param 3 - User defined
        /// Param 4 - User defined
        /// Param 5 - User defined
        /// Param 6 - User defined
        /// Param 7 - User defined
        /// MAV_CMD_USER_5
        /// </summary>
        MavCmdUser5 = 31014,
    }

    /// <summary>
    /// THIS INTERFACE IS DEPRECATED AS OF JULY 2015. Please use MESSAGE_INTERVAL instead. A data stream is not a fixed set of messages, but rather a
    ///      recommendation to the autopilot software. Individual autopilots may or may not obey
    ///      the recommended messages.
    ///  MAV_DATA_STREAM
    /// </summary>
    public enum MavDataStream
    {
        /// <summary>
        /// Enable all data streams
        /// MAV_DATA_STREAM_ALL
        /// </summary>
        MavDataStreamAll = 0,
        /// <summary>
        /// Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
        /// MAV_DATA_STREAM_RAW_SENSORS
        /// </summary>
        MavDataStreamRawSensors = 1,
        /// <summary>
        /// Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
        /// MAV_DATA_STREAM_EXTENDED_STATUS
        /// </summary>
        MavDataStreamExtendedStatus = 2,
        /// <summary>
        /// Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
        /// MAV_DATA_STREAM_RC_CHANNELS
        /// </summary>
        MavDataStreamRcChannels = 3,
        /// <summary>
        /// Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
        /// MAV_DATA_STREAM_RAW_CONTROLLER
        /// </summary>
        MavDataStreamRawController = 4,
        /// <summary>
        /// Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
        /// MAV_DATA_STREAM_POSITION
        /// </summary>
        MavDataStreamPosition = 6,
        /// <summary>
        /// Dependent on the autopilot
        /// MAV_DATA_STREAM_EXTRA1
        /// </summary>
        MavDataStreamExtra1 = 10,
        /// <summary>
        /// Dependent on the autopilot
        /// MAV_DATA_STREAM_EXTRA2
        /// </summary>
        MavDataStreamExtra2 = 11,
        /// <summary>
        /// Dependent on the autopilot
        /// MAV_DATA_STREAM_EXTRA3
        /// </summary>
        MavDataStreamExtra3 = 12,
    }

    /// <summary>
    /// THIS INTERFACE IS DEPRECATED AS OF JANUARY 2018. Please use MAV_CMD_DO_SET_ROI_* messages instead. The ROI (region of interest) for the vehicle. This can be
    ///                 be used by the vehicle for camera/vehicle attitude alignment (see
    ///                 MAV_CMD_NAV_ROI).
    ///  MAV_ROI
    /// </summary>
    public enum MavRoi
    {
        /// <summary>
        /// No region of interest.
        /// MAV_ROI_NONE
        /// </summary>
        MavRoiNone = 0,
        /// <summary>
        /// Point toward next waypoint, with optional pitch/roll/yaw offset.
        /// MAV_ROI_WPNEXT
        /// </summary>
        MavRoiWpnext = 1,
        /// <summary>
        /// Point toward given waypoint.
        /// MAV_ROI_WPINDEX
        /// </summary>
        MavRoiWpindex = 2,
        /// <summary>
        /// Point toward fixed location.
        /// MAV_ROI_LOCATION
        /// </summary>
        MavRoiLocation = 3,
        /// <summary>
        /// Point toward of given id.
        /// MAV_ROI_TARGET
        /// </summary>
        MavRoiTarget = 4,
    }

    /// <summary>
    /// ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
    ///  MAV_CMD_ACK
    /// </summary>
    public enum MavCmdAck
    {
        /// <summary>
        /// Command / mission item is ok.
        /// MAV_CMD_ACK_OK
        /// </summary>
        MavCmdAckOk = 0,
        /// <summary>
        /// Generic error message if none of the other reasons fails or if no detailed error reporting is implemented.
        /// MAV_CMD_ACK_ERR_FAIL
        /// </summary>
        MavCmdAckErrFail = 1,
        /// <summary>
        /// The system is refusing to accept this command from this source / communication partner.
        /// MAV_CMD_ACK_ERR_ACCESS_DENIED
        /// </summary>
        MavCmdAckErrAccessDenied = 2,
        /// <summary>
        /// Command or mission item is not supported, other commands would be accepted.
        /// MAV_CMD_ACK_ERR_NOT_SUPPORTED
        /// </summary>
        MavCmdAckErrNotSupported = 3,
        /// <summary>
        /// The coordinate frame of this command / mission item is not supported.
        /// MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED
        /// </summary>
        MavCmdAckErrCoordinateFrameNotSupported = 4,
        /// <summary>
        /// The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible.
        /// MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE
        /// </summary>
        MavCmdAckErrCoordinatesOutOfRange = 5,
        /// <summary>
        /// The X or latitude value is out of range.
        /// MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE
        /// </summary>
        MavCmdAckErrXLatOutOfRange = 6,
        /// <summary>
        /// The Y or longitude value is out of range.
        /// MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE
        /// </summary>
        MavCmdAckErrYLonOutOfRange = 7,
        /// <summary>
        /// The Z or altitude value is out of range.
        /// MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE
        /// </summary>
        MavCmdAckErrZAltOutOfRange = 8,
    }

    /// <summary>
    /// Specifies the datatype of a MAVLink parameter.
    ///  MAV_PARAM_TYPE
    /// </summary>
    public enum MavParamType
    {
        /// <summary>
        /// 8-bit unsigned integer
        /// MAV_PARAM_TYPE_UINT8
        /// </summary>
        MavParamTypeUint8 = 1,
        /// <summary>
        /// 8-bit signed integer
        /// MAV_PARAM_TYPE_INT8
        /// </summary>
        MavParamTypeInt8 = 2,
        /// <summary>
        /// 16-bit unsigned integer
        /// MAV_PARAM_TYPE_UINT16
        /// </summary>
        MavParamTypeUint16 = 3,
        /// <summary>
        /// 16-bit signed integer
        /// MAV_PARAM_TYPE_INT16
        /// </summary>
        MavParamTypeInt16 = 4,
        /// <summary>
        /// 32-bit unsigned integer
        /// MAV_PARAM_TYPE_UINT32
        /// </summary>
        MavParamTypeUint32 = 5,
        /// <summary>
        /// 32-bit signed integer
        /// MAV_PARAM_TYPE_INT32
        /// </summary>
        MavParamTypeInt32 = 6,
        /// <summary>
        /// 64-bit unsigned integer
        /// MAV_PARAM_TYPE_UINT64
        /// </summary>
        MavParamTypeUint64 = 7,
        /// <summary>
        /// 64-bit signed integer
        /// MAV_PARAM_TYPE_INT64
        /// </summary>
        MavParamTypeInt64 = 8,
        /// <summary>
        /// 32-bit floating-point
        /// MAV_PARAM_TYPE_REAL32
        /// </summary>
        MavParamTypeReal32 = 9,
        /// <summary>
        /// 64-bit floating-point
        /// MAV_PARAM_TYPE_REAL64
        /// </summary>
        MavParamTypeReal64 = 10,
    }

    /// <summary>
    /// Specifies the datatype of a MAVLink extended parameter.
    ///  MAV_PARAM_EXT_TYPE
    /// </summary>
    public enum MavParamExtType
    {
        /// <summary>
        /// 8-bit unsigned integer
        /// MAV_PARAM_EXT_TYPE_UINT8
        /// </summary>
        MavParamExtTypeUint8 = 1,
        /// <summary>
        /// 8-bit signed integer
        /// MAV_PARAM_EXT_TYPE_INT8
        /// </summary>
        MavParamExtTypeInt8 = 2,
        /// <summary>
        /// 16-bit unsigned integer
        /// MAV_PARAM_EXT_TYPE_UINT16
        /// </summary>
        MavParamExtTypeUint16 = 3,
        /// <summary>
        /// 16-bit signed integer
        /// MAV_PARAM_EXT_TYPE_INT16
        /// </summary>
        MavParamExtTypeInt16 = 4,
        /// <summary>
        /// 32-bit unsigned integer
        /// MAV_PARAM_EXT_TYPE_UINT32
        /// </summary>
        MavParamExtTypeUint32 = 5,
        /// <summary>
        /// 32-bit signed integer
        /// MAV_PARAM_EXT_TYPE_INT32
        /// </summary>
        MavParamExtTypeInt32 = 6,
        /// <summary>
        /// 64-bit unsigned integer
        /// MAV_PARAM_EXT_TYPE_UINT64
        /// </summary>
        MavParamExtTypeUint64 = 7,
        /// <summary>
        /// 64-bit signed integer
        /// MAV_PARAM_EXT_TYPE_INT64
        /// </summary>
        MavParamExtTypeInt64 = 8,
        /// <summary>
        /// 32-bit floating-point
        /// MAV_PARAM_EXT_TYPE_REAL32
        /// </summary>
        MavParamExtTypeReal32 = 9,
        /// <summary>
        /// 64-bit floating-point
        /// MAV_PARAM_EXT_TYPE_REAL64
        /// </summary>
        MavParamExtTypeReal64 = 10,
        /// <summary>
        /// Custom Type
        /// MAV_PARAM_EXT_TYPE_CUSTOM
        /// </summary>
        MavParamExtTypeCustom = 11,
    }

    /// <summary>
    /// result from a mavlink command
    ///  MAV_RESULT
    /// </summary>
    public enum MavResult
    {
        /// <summary>
        /// Command ACCEPTED and EXECUTED
        /// MAV_RESULT_ACCEPTED
        /// </summary>
        MavResultAccepted = 0,
        /// <summary>
        /// Command TEMPORARY REJECTED/DENIED
        /// MAV_RESULT_TEMPORARILY_REJECTED
        /// </summary>
        MavResultTemporarilyRejected = 1,
        /// <summary>
        /// Command PERMANENTLY DENIED
        /// MAV_RESULT_DENIED
        /// </summary>
        MavResultDenied = 2,
        /// <summary>
        /// Command UNKNOWN/UNSUPPORTED
        /// MAV_RESULT_UNSUPPORTED
        /// </summary>
        MavResultUnsupported = 3,
        /// <summary>
        /// Command executed, but failed
        /// MAV_RESULT_FAILED
        /// </summary>
        MavResultFailed = 4,
        /// <summary>
        /// WIP: Command being executed
        /// MAV_RESULT_IN_PROGRESS
        /// </summary>
        MavResultInProgress = 5,
    }

    /// <summary>
    /// result in a mavlink mission ack
    ///  MAV_MISSION_RESULT
    /// </summary>
    public enum MavMissionResult
    {
        /// <summary>
        /// mission accepted OK
        /// MAV_MISSION_ACCEPTED
        /// </summary>
        MavMissionAccepted = 0,
        /// <summary>
        /// generic error / not accepting mission commands at all right now
        /// MAV_MISSION_ERROR
        /// </summary>
        MavMissionError = 1,
        /// <summary>
        /// coordinate frame is not supported
        /// MAV_MISSION_UNSUPPORTED_FRAME
        /// </summary>
        MavMissionUnsupportedFrame = 2,
        /// <summary>
        /// command is not supported
        /// MAV_MISSION_UNSUPPORTED
        /// </summary>
        MavMissionUnsupported = 3,
        /// <summary>
        /// mission item exceeds storage space
        /// MAV_MISSION_NO_SPACE
        /// </summary>
        MavMissionNoSpace = 4,
        /// <summary>
        /// one of the parameters has an invalid value
        /// MAV_MISSION_INVALID
        /// </summary>
        MavMissionInvalid = 5,
        /// <summary>
        /// param1 has an invalid value
        /// MAV_MISSION_INVALID_PARAM1
        /// </summary>
        MavMissionInvalidParam1 = 6,
        /// <summary>
        /// param2 has an invalid value
        /// MAV_MISSION_INVALID_PARAM2
        /// </summary>
        MavMissionInvalidParam2 = 7,
        /// <summary>
        /// param3 has an invalid value
        /// MAV_MISSION_INVALID_PARAM3
        /// </summary>
        MavMissionInvalidParam3 = 8,
        /// <summary>
        /// param4 has an invalid value
        /// MAV_MISSION_INVALID_PARAM4
        /// </summary>
        MavMissionInvalidParam4 = 9,
        /// <summary>
        /// x/param5 has an invalid value
        /// MAV_MISSION_INVALID_PARAM5_X
        /// </summary>
        MavMissionInvalidParam5X = 10,
        /// <summary>
        /// y/param6 has an invalid value
        /// MAV_MISSION_INVALID_PARAM6_Y
        /// </summary>
        MavMissionInvalidParam6Y = 11,
        /// <summary>
        /// param7 has an invalid value
        /// MAV_MISSION_INVALID_PARAM7
        /// </summary>
        MavMissionInvalidParam7 = 12,
        /// <summary>
        /// received waypoint out of sequence
        /// MAV_MISSION_INVALID_SEQUENCE
        /// </summary>
        MavMissionInvalidSequence = 13,
        /// <summary>
        /// not accepting any mission commands from this communication partner
        /// MAV_MISSION_DENIED
        /// </summary>
        MavMissionDenied = 14,
    }

    /// <summary>
    /// Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
    ///  MAV_SEVERITY
    /// </summary>
    public enum MavSeverity
    {
        /// <summary>
        /// System is unusable. This is a "panic" condition.
        /// MAV_SEVERITY_EMERGENCY
        /// </summary>
        MavSeverityEmergency = 0,
        /// <summary>
        /// Action should be taken immediately. Indicates error in non-critical systems.
        /// MAV_SEVERITY_ALERT
        /// </summary>
        MavSeverityAlert = 1,
        /// <summary>
        /// Action must be taken immediately. Indicates failure in a primary system.
        /// MAV_SEVERITY_CRITICAL
        /// </summary>
        MavSeverityCritical = 2,
        /// <summary>
        /// Indicates an error in secondary/redundant systems.
        /// MAV_SEVERITY_ERROR
        /// </summary>
        MavSeverityError = 3,
        /// <summary>
        /// Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
        /// MAV_SEVERITY_WARNING
        /// </summary>
        MavSeverityWarning = 4,
        /// <summary>
        /// An unusual event has occured, though not an error condition. This should be investigated for the root cause.
        /// MAV_SEVERITY_NOTICE
        /// </summary>
        MavSeverityNotice = 5,
        /// <summary>
        /// Normal operational messages. Useful for logging. No action is required for these messages.
        /// MAV_SEVERITY_INFO
        /// </summary>
        MavSeverityInfo = 6,
        /// <summary>
        /// Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
        /// MAV_SEVERITY_DEBUG
        /// </summary>
        MavSeverityDebug = 7,
    }

    /// <summary>
    /// Power supply status flags (bitmask)
    ///  MAV_POWER_STATUS
    /// </summary>
    public enum MavPowerStatus
    {
        /// <summary>
        /// main brick power supply valid
        /// MAV_POWER_STATUS_BRICK_VALID
        /// </summary>
        MavPowerStatusBrickValid = 1,
        /// <summary>
        /// main servo power supply valid for FMU
        /// MAV_POWER_STATUS_SERVO_VALID
        /// </summary>
        MavPowerStatusServoValid = 2,
        /// <summary>
        /// USB power is connected
        /// MAV_POWER_STATUS_USB_CONNECTED
        /// </summary>
        MavPowerStatusUsbConnected = 4,
        /// <summary>
        /// peripheral supply is in over-current state
        /// MAV_POWER_STATUS_PERIPH_OVERCURRENT
        /// </summary>
        MavPowerStatusPeriphOvercurrent = 8,
        /// <summary>
        /// hi-power peripheral supply is in over-current state
        /// MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT
        /// </summary>
        MavPowerStatusPeriphHipowerOvercurrent = 16,
        /// <summary>
        /// Power status has changed since boot
        /// MAV_POWER_STATUS_CHANGED
        /// </summary>
        MavPowerStatusChanged = 32,
    }

    /// <summary>
    /// SERIAL_CONTROL device types
    ///  SERIAL_CONTROL_DEV
    /// </summary>
    public enum SerialControlDev
    {
        /// <summary>
        /// First telemetry port
        /// SERIAL_CONTROL_DEV_TELEM1
        /// </summary>
        SerialControlDevTelem1 = 0,
        /// <summary>
        /// Second telemetry port
        /// SERIAL_CONTROL_DEV_TELEM2
        /// </summary>
        SerialControlDevTelem2 = 1,
        /// <summary>
        /// First GPS port
        /// SERIAL_CONTROL_DEV_GPS1
        /// </summary>
        SerialControlDevGps1 = 2,
        /// <summary>
        /// Second GPS port
        /// SERIAL_CONTROL_DEV_GPS2
        /// </summary>
        SerialControlDevGps2 = 3,
        /// <summary>
        /// system shell
        /// SERIAL_CONTROL_DEV_SHELL
        /// </summary>
        SerialControlDevShell = 10,
    }

    /// <summary>
    /// SERIAL_CONTROL flags (bitmask)
    ///  SERIAL_CONTROL_FLAG
    /// </summary>
    public enum SerialControlFlag
    {
        /// <summary>
        /// Set if this is a reply
        /// SERIAL_CONTROL_FLAG_REPLY
        /// </summary>
        SerialControlFlagReply = 1,
        /// <summary>
        /// Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
        /// SERIAL_CONTROL_FLAG_RESPOND
        /// </summary>
        SerialControlFlagRespond = 2,
        /// <summary>
        /// Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set
        /// SERIAL_CONTROL_FLAG_EXCLUSIVE
        /// </summary>
        SerialControlFlagExclusive = 4,
        /// <summary>
        /// Block on writes to the serial port
        /// SERIAL_CONTROL_FLAG_BLOCKING
        /// </summary>
        SerialControlFlagBlocking = 8,
        /// <summary>
        /// Send multiple replies until port is drained
        /// SERIAL_CONTROL_FLAG_MULTI
        /// </summary>
        SerialControlFlagMulti = 16,
    }

    /// <summary>
    /// Enumeration of distance sensor types
    ///  MAV_DISTANCE_SENSOR
    /// </summary>
    public enum MavDistanceSensor
    {
        /// <summary>
        /// Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
        /// MAV_DISTANCE_SENSOR_LASER
        /// </summary>
        MavDistanceSensorLaser = 0,
        /// <summary>
        /// Ultrasound rangefinder, e.g. MaxBotix units
        /// MAV_DISTANCE_SENSOR_ULTRASOUND
        /// </summary>
        MavDistanceSensorUltrasound = 1,
        /// <summary>
        /// Infrared rangefinder, e.g. Sharp units
        /// MAV_DISTANCE_SENSOR_INFRARED
        /// </summary>
        MavDistanceSensorInfrared = 2,
        /// <summary>
        /// Radar type, e.g. uLanding units
        /// MAV_DISTANCE_SENSOR_RADAR
        /// </summary>
        MavDistanceSensorRadar = 3,
        /// <summary>
        /// Broken or unknown type, e.g. analog units
        /// MAV_DISTANCE_SENSOR_UNKNOWN
        /// </summary>
        MavDistanceSensorUnknown = 4,
    }

    /// <summary>
    /// Enumeration of sensor orientation, according to its rotations
    ///  MAV_SENSOR_ORIENTATION
    /// </summary>
    public enum MavSensorOrientation
    {
        /// <summary>
        /// Roll: 0, Pitch: 0, Yaw: 0
        /// MAV_SENSOR_ROTATION_NONE
        /// </summary>
        MavSensorRotationNone = 0,
        /// <summary>
        /// Roll: 0, Pitch: 0, Yaw: 45
        /// MAV_SENSOR_ROTATION_YAW_45
        /// </summary>
        MavSensorRotationYaw45 = 1,
        /// <summary>
        /// Roll: 0, Pitch: 0, Yaw: 90
        /// MAV_SENSOR_ROTATION_YAW_90
        /// </summary>
        MavSensorRotationYaw90 = 2,
        /// <summary>
        /// Roll: 0, Pitch: 0, Yaw: 135
        /// MAV_SENSOR_ROTATION_YAW_135
        /// </summary>
        MavSensorRotationYaw135 = 3,
        /// <summary>
        /// Roll: 0, Pitch: 0, Yaw: 180
        /// MAV_SENSOR_ROTATION_YAW_180
        /// </summary>
        MavSensorRotationYaw180 = 4,
        /// <summary>
        /// Roll: 0, Pitch: 0, Yaw: 225
        /// MAV_SENSOR_ROTATION_YAW_225
        /// </summary>
        MavSensorRotationYaw225 = 5,
        /// <summary>
        /// Roll: 0, Pitch: 0, Yaw: 270
        /// MAV_SENSOR_ROTATION_YAW_270
        /// </summary>
        MavSensorRotationYaw270 = 6,
        /// <summary>
        /// Roll: 0, Pitch: 0, Yaw: 315
        /// MAV_SENSOR_ROTATION_YAW_315
        /// </summary>
        MavSensorRotationYaw315 = 7,
        /// <summary>
        /// Roll: 180, Pitch: 0, Yaw: 0
        /// MAV_SENSOR_ROTATION_ROLL_180
        /// </summary>
        MavSensorRotationRoll180 = 8,
        /// <summary>
        /// Roll: 180, Pitch: 0, Yaw: 45
        /// MAV_SENSOR_ROTATION_ROLL_180_YAW_45
        /// </summary>
        MavSensorRotationRoll180Yaw45 = 9,
        /// <summary>
        /// Roll: 180, Pitch: 0, Yaw: 90
        /// MAV_SENSOR_ROTATION_ROLL_180_YAW_90
        /// </summary>
        MavSensorRotationRoll180Yaw90 = 10,
        /// <summary>
        /// Roll: 180, Pitch: 0, Yaw: 135
        /// MAV_SENSOR_ROTATION_ROLL_180_YAW_135
        /// </summary>
        MavSensorRotationRoll180Yaw135 = 11,
        /// <summary>
        /// Roll: 0, Pitch: 180, Yaw: 0
        /// MAV_SENSOR_ROTATION_PITCH_180
        /// </summary>
        MavSensorRotationPitch180 = 12,
        /// <summary>
        /// Roll: 180, Pitch: 0, Yaw: 225
        /// MAV_SENSOR_ROTATION_ROLL_180_YAW_225
        /// </summary>
        MavSensorRotationRoll180Yaw225 = 13,
        /// <summary>
        /// Roll: 180, Pitch: 0, Yaw: 270
        /// MAV_SENSOR_ROTATION_ROLL_180_YAW_270
        /// </summary>
        MavSensorRotationRoll180Yaw270 = 14,
        /// <summary>
        /// Roll: 180, Pitch: 0, Yaw: 315
        /// MAV_SENSOR_ROTATION_ROLL_180_YAW_315
        /// </summary>
        MavSensorRotationRoll180Yaw315 = 15,
        /// <summary>
        /// Roll: 90, Pitch: 0, Yaw: 0
        /// MAV_SENSOR_ROTATION_ROLL_90
        /// </summary>
        MavSensorRotationRoll90 = 16,
        /// <summary>
        /// Roll: 90, Pitch: 0, Yaw: 45
        /// MAV_SENSOR_ROTATION_ROLL_90_YAW_45
        /// </summary>
        MavSensorRotationRoll90Yaw45 = 17,
        /// <summary>
        /// Roll: 90, Pitch: 0, Yaw: 90
        /// MAV_SENSOR_ROTATION_ROLL_90_YAW_90
        /// </summary>
        MavSensorRotationRoll90Yaw90 = 18,
        /// <summary>
        /// Roll: 90, Pitch: 0, Yaw: 135
        /// MAV_SENSOR_ROTATION_ROLL_90_YAW_135
        /// </summary>
        MavSensorRotationRoll90Yaw135 = 19,
        /// <summary>
        /// Roll: 270, Pitch: 0, Yaw: 0
        /// MAV_SENSOR_ROTATION_ROLL_270
        /// </summary>
        MavSensorRotationRoll270 = 20,
        /// <summary>
        /// Roll: 270, Pitch: 0, Yaw: 45
        /// MAV_SENSOR_ROTATION_ROLL_270_YAW_45
        /// </summary>
        MavSensorRotationRoll270Yaw45 = 21,
        /// <summary>
        /// Roll: 270, Pitch: 0, Yaw: 90
        /// MAV_SENSOR_ROTATION_ROLL_270_YAW_90
        /// </summary>
        MavSensorRotationRoll270Yaw90 = 22,
        /// <summary>
        /// Roll: 270, Pitch: 0, Yaw: 135
        /// MAV_SENSOR_ROTATION_ROLL_270_YAW_135
        /// </summary>
        MavSensorRotationRoll270Yaw135 = 23,
        /// <summary>
        /// Roll: 0, Pitch: 90, Yaw: 0
        /// MAV_SENSOR_ROTATION_PITCH_90
        /// </summary>
        MavSensorRotationPitch90 = 24,
        /// <summary>
        /// Roll: 0, Pitch: 270, Yaw: 0
        /// MAV_SENSOR_ROTATION_PITCH_270
        /// </summary>
        MavSensorRotationPitch270 = 25,
        /// <summary>
        /// Roll: 0, Pitch: 180, Yaw: 90
        /// MAV_SENSOR_ROTATION_PITCH_180_YAW_90
        /// </summary>
        MavSensorRotationPitch180Yaw90 = 26,
        /// <summary>
        /// Roll: 0, Pitch: 180, Yaw: 270
        /// MAV_SENSOR_ROTATION_PITCH_180_YAW_270
        /// </summary>
        MavSensorRotationPitch180Yaw270 = 27,
        /// <summary>
        /// Roll: 90, Pitch: 90, Yaw: 0
        /// MAV_SENSOR_ROTATION_ROLL_90_PITCH_90
        /// </summary>
        MavSensorRotationRoll90Pitch90 = 28,
        /// <summary>
        /// Roll: 180, Pitch: 90, Yaw: 0
        /// MAV_SENSOR_ROTATION_ROLL_180_PITCH_90
        /// </summary>
        MavSensorRotationRoll180Pitch90 = 29,
        /// <summary>
        /// Roll: 270, Pitch: 90, Yaw: 0
        /// MAV_SENSOR_ROTATION_ROLL_270_PITCH_90
        /// </summary>
        MavSensorRotationRoll270Pitch90 = 30,
        /// <summary>
        /// Roll: 90, Pitch: 180, Yaw: 0
        /// MAV_SENSOR_ROTATION_ROLL_90_PITCH_180
        /// </summary>
        MavSensorRotationRoll90Pitch180 = 31,
        /// <summary>
        /// Roll: 270, Pitch: 180, Yaw: 0
        /// MAV_SENSOR_ROTATION_ROLL_270_PITCH_180
        /// </summary>
        MavSensorRotationRoll270Pitch180 = 32,
        /// <summary>
        /// Roll: 90, Pitch: 270, Yaw: 0
        /// MAV_SENSOR_ROTATION_ROLL_90_PITCH_270
        /// </summary>
        MavSensorRotationRoll90Pitch270 = 33,
        /// <summary>
        /// Roll: 180, Pitch: 270, Yaw: 0
        /// MAV_SENSOR_ROTATION_ROLL_180_PITCH_270
        /// </summary>
        MavSensorRotationRoll180Pitch270 = 34,
        /// <summary>
        /// Roll: 270, Pitch: 270, Yaw: 0
        /// MAV_SENSOR_ROTATION_ROLL_270_PITCH_270
        /// </summary>
        MavSensorRotationRoll270Pitch270 = 35,
        /// <summary>
        /// Roll: 90, Pitch: 180, Yaw: 90
        /// MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90
        /// </summary>
        MavSensorRotationRoll90Pitch180Yaw90 = 36,
        /// <summary>
        /// Roll: 90, Pitch: 0, Yaw: 270
        /// MAV_SENSOR_ROTATION_ROLL_90_YAW_270
        /// </summary>
        MavSensorRotationRoll90Yaw270 = 37,
        /// <summary>
        /// Roll: 315, Pitch: 315, Yaw: 315
        /// MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315
        /// </summary>
        MavSensorRotationRoll315Pitch315Yaw315 = 38,
    }

    /// <summary>
    /// Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability.
    ///  MAV_PROTOCOL_CAPABILITY
    /// </summary>
    public enum MavProtocolCapability
    {
        /// <summary>
        /// Autopilot supports MISSION float message type.
        /// MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT
        /// </summary>
        MavProtocolCapabilityMissionFloat = 1,
        /// <summary>
        /// Autopilot supports the new param float message type.
        /// MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT
        /// </summary>
        MavProtocolCapabilityParamFloat = 2,
        /// <summary>
        /// Autopilot supports MISSION_INT scaled integer message type.
        /// MAV_PROTOCOL_CAPABILITY_MISSION_INT
        /// </summary>
        MavProtocolCapabilityMissionInt = 4,
        /// <summary>
        /// Autopilot supports COMMAND_INT scaled integer message type.
        /// MAV_PROTOCOL_CAPABILITY_COMMAND_INT
        /// </summary>
        MavProtocolCapabilityCommandInt = 8,
        /// <summary>
        /// Autopilot supports the new param union message type.
        /// MAV_PROTOCOL_CAPABILITY_PARAM_UNION
        /// </summary>
        MavProtocolCapabilityParamUnion = 16,
        /// <summary>
        /// Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.
        /// MAV_PROTOCOL_CAPABILITY_FTP
        /// </summary>
        MavProtocolCapabilityFtp = 32,
        /// <summary>
        /// Autopilot supports commanding attitude offboard.
        /// MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
        /// </summary>
        MavProtocolCapabilitySetAttitudeTarget = 64,
        /// <summary>
        /// Autopilot supports commanding position and velocity targets in local NED frame.
        /// MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED
        /// </summary>
        MavProtocolCapabilitySetPositionTargetLocalNed = 128,
        /// <summary>
        /// Autopilot supports commanding position and velocity targets in global scaled integers.
        /// MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT
        /// </summary>
        MavProtocolCapabilitySetPositionTargetGlobalInt = 256,
        /// <summary>
        /// Autopilot supports terrain protocol / data handling.
        /// MAV_PROTOCOL_CAPABILITY_TERRAIN
        /// </summary>
        MavProtocolCapabilityTerrain = 512,
        /// <summary>
        /// Autopilot supports direct actuator control.
        /// MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET
        /// </summary>
        MavProtocolCapabilitySetActuatorTarget = 1024,
        /// <summary>
        /// Autopilot supports the flight termination command.
        /// MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION
        /// </summary>
        MavProtocolCapabilityFlightTermination = 2048,
        /// <summary>
        /// Autopilot supports onboard compass calibration.
        /// MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION
        /// </summary>
        MavProtocolCapabilityCompassCalibration = 4096,
        /// <summary>
        /// Autopilot supports mavlink version 2.
        /// MAV_PROTOCOL_CAPABILITY_MAVLINK2
        /// </summary>
        MavProtocolCapabilityMavlink2 = 8192,
        /// <summary>
        /// Autopilot supports mission fence protocol.
        /// MAV_PROTOCOL_CAPABILITY_MISSION_FENCE
        /// </summary>
        MavProtocolCapabilityMissionFence = 16384,
        /// <summary>
        /// Autopilot supports mission rally point protocol.
        /// MAV_PROTOCOL_CAPABILITY_MISSION_RALLY
        /// </summary>
        MavProtocolCapabilityMissionRally = 32768,
        /// <summary>
        /// Autopilot supports the flight information protocol.
        /// MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION
        /// </summary>
        MavProtocolCapabilityFlightInformation = 65536,
    }

    /// <summary>
    /// Type of mission items being requested/sent in mission protocol.
    ///  MAV_MISSION_TYPE
    /// </summary>
    public enum MavMissionType
    {
        /// <summary>
        /// Items are mission commands for main mission.
        /// MAV_MISSION_TYPE_MISSION
        /// </summary>
        MavMissionTypeMission = 0,
        /// <summary>
        /// Specifies GeoFence area(s). Items are MAV_CMD_FENCE_ GeoFence items.
        /// MAV_MISSION_TYPE_FENCE
        /// </summary>
        MavMissionTypeFence = 1,
        /// <summary>
        /// Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_RALLY_POINT rally point items.
        /// MAV_MISSION_TYPE_RALLY
        /// </summary>
        MavMissionTypeRally = 2,
        /// <summary>
        /// Only used in MISSION_CLEAR_ALL to clear all mission types.
        /// MAV_MISSION_TYPE_ALL
        /// </summary>
        MavMissionTypeAll = 255,
    }

    /// <summary>
    /// Enumeration of estimator types
    ///  MAV_ESTIMATOR_TYPE
    /// </summary>
    public enum MavEstimatorType
    {
        /// <summary>
        /// This is a naive estimator without any real covariance feedback.
        /// MAV_ESTIMATOR_TYPE_NAIVE
        /// </summary>
        MavEstimatorTypeNaive = 1,
        /// <summary>
        /// Computer vision based estimate. Might be up to scale.
        /// MAV_ESTIMATOR_TYPE_VISION
        /// </summary>
        MavEstimatorTypeVision = 2,
        /// <summary>
        /// Visual-inertial estimate.
        /// MAV_ESTIMATOR_TYPE_VIO
        /// </summary>
        MavEstimatorTypeVio = 3,
        /// <summary>
        /// Plain GPS estimate.
        /// MAV_ESTIMATOR_TYPE_GPS
        /// </summary>
        MavEstimatorTypeGps = 4,
        /// <summary>
        /// Estimator integrating GPS and inertial sensing.
        /// MAV_ESTIMATOR_TYPE_GPS_INS
        /// </summary>
        MavEstimatorTypeGpsIns = 5,
    }

    /// <summary>
    /// Enumeration of battery types
    ///  MAV_BATTERY_TYPE
    /// </summary>
    public enum MavBatteryType
    {
        /// <summary>
        /// Not specified.
        /// MAV_BATTERY_TYPE_UNKNOWN
        /// </summary>
        MavBatteryTypeUnknown = 0,
        /// <summary>
        /// Lithium polymer battery
        /// MAV_BATTERY_TYPE_LIPO
        /// </summary>
        MavBatteryTypeLipo = 1,
        /// <summary>
        /// Lithium-iron-phosphate battery
        /// MAV_BATTERY_TYPE_LIFE
        /// </summary>
        MavBatteryTypeLife = 2,
        /// <summary>
        /// Lithium-ION battery
        /// MAV_BATTERY_TYPE_LION
        /// </summary>
        MavBatteryTypeLion = 3,
        /// <summary>
        /// Nickel metal hydride battery
        /// MAV_BATTERY_TYPE_NIMH
        /// </summary>
        MavBatteryTypeNimh = 4,
    }

    /// <summary>
    /// Enumeration of battery functions
    ///  MAV_BATTERY_FUNCTION
    /// </summary>
    public enum MavBatteryFunction
    {
        /// <summary>
        /// Battery function is unknown
        /// MAV_BATTERY_FUNCTION_UNKNOWN
        /// </summary>
        MavBatteryFunctionUnknown = 0,
        /// <summary>
        /// Battery supports all flight systems
        /// MAV_BATTERY_FUNCTION_ALL
        /// </summary>
        MavBatteryFunctionAll = 1,
        /// <summary>
        /// Battery for the propulsion system
        /// MAV_BATTERY_FUNCTION_PROPULSION
        /// </summary>
        MavBatteryFunctionPropulsion = 2,
        /// <summary>
        /// Avionics battery
        /// MAV_BATTERY_FUNCTION_AVIONICS
        /// </summary>
        MavBatteryFunctionAvionics = 3,
        /// <summary>
        /// Payload battery
        /// MAV_BATTERY_TYPE_PAYLOAD
        /// </summary>
        MavBatteryTypePayload = 4,
    }

    /// <summary>
    /// Enumeration for low battery states.
    ///  MAV_BATTERY_CHARGE_STATE
    /// </summary>
    public enum MavBatteryChargeState
    {
        /// <summary>
        /// Low battery state is not provided
        /// MAV_BATTERY_CHARGE_STATE_UNDEFINED
        /// </summary>
        MavBatteryChargeStateUndefined = 0,
        /// <summary>
        /// Battery is not in low state. Normal operation.
        /// MAV_BATTERY_CHARGE_STATE_OK
        /// </summary>
        MavBatteryChargeStateOk = 1,
        /// <summary>
        /// Battery state is low, warn and monitor close.
        /// MAV_BATTERY_CHARGE_STATE_LOW
        /// </summary>
        MavBatteryChargeStateLow = 2,
        /// <summary>
        /// Battery state is critical, return or abort immediately.
        /// MAV_BATTERY_CHARGE_STATE_CRITICAL
        /// </summary>
        MavBatteryChargeStateCritical = 3,
        /// <summary>
        /// Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to prevent damage.
        /// MAV_BATTERY_CHARGE_STATE_EMERGENCY
        /// </summary>
        MavBatteryChargeStateEmergency = 4,
        /// <summary>
        /// Battery failed, damage unavoidable.
        /// MAV_BATTERY_CHARGE_STATE_FAILED
        /// </summary>
        MavBatteryChargeStateFailed = 5,
        /// <summary>
        /// Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited.
        /// MAV_BATTERY_CHARGE_STATE_UNHEALTHY
        /// </summary>
        MavBatteryChargeStateUnhealthy = 6,
    }

    /// <summary>
    /// Enumeration of VTOL states
    ///  MAV_VTOL_STATE
    /// </summary>
    public enum MavVtolState
    {
        /// <summary>
        /// MAV is not configured as VTOL
        /// MAV_VTOL_STATE_UNDEFINED
        /// </summary>
        MavVtolStateUndefined = 0,
        /// <summary>
        /// VTOL is in transition from multicopter to fixed-wing
        /// MAV_VTOL_STATE_TRANSITION_TO_FW
        /// </summary>
        MavVtolStateTransitionToFw = 1,
        /// <summary>
        /// VTOL is in transition from fixed-wing to multicopter
        /// MAV_VTOL_STATE_TRANSITION_TO_MC
        /// </summary>
        MavVtolStateTransitionToMc = 2,
        /// <summary>
        /// VTOL is in multicopter state
        /// MAV_VTOL_STATE_MC
        /// </summary>
        MavVtolStateMc = 3,
        /// <summary>
        /// VTOL is in fixed-wing state
        /// MAV_VTOL_STATE_FW
        /// </summary>
        MavVtolStateFw = 4,
    }

    /// <summary>
    /// Enumeration of landed detector states
    ///  MAV_LANDED_STATE
    /// </summary>
    public enum MavLandedState
    {
        /// <summary>
        /// MAV landed state is unknown
        /// MAV_LANDED_STATE_UNDEFINED
        /// </summary>
        MavLandedStateUndefined = 0,
        /// <summary>
        /// MAV is landed (on ground)
        /// MAV_LANDED_STATE_ON_GROUND
        /// </summary>
        MavLandedStateOnGround = 1,
        /// <summary>
        /// MAV is in air
        /// MAV_LANDED_STATE_IN_AIR
        /// </summary>
        MavLandedStateInAir = 2,
        /// <summary>
        /// MAV currently taking off
        /// MAV_LANDED_STATE_TAKEOFF
        /// </summary>
        MavLandedStateTakeoff = 3,
        /// <summary>
        /// MAV currently landing
        /// MAV_LANDED_STATE_LANDING
        /// </summary>
        MavLandedStateLanding = 4,
    }

    /// <summary>
    /// Enumeration of the ADSB altimeter types
    ///  ADSB_ALTITUDE_TYPE
    /// </summary>
    public enum AdsbAltitudeType
    {
        /// <summary>
        /// Altitude reported from a Baro source using QNH reference
        /// ADSB_ALTITUDE_TYPE_PRESSURE_QNH
        /// </summary>
        AdsbAltitudeTypePressureQnh = 0,
        /// <summary>
        /// Altitude reported from a GNSS source
        /// ADSB_ALTITUDE_TYPE_GEOMETRIC
        /// </summary>
        AdsbAltitudeTypeGeometric = 1,
    }

    /// <summary>
    /// ADSB classification for the type of vehicle emitting the transponder signal
    ///  ADSB_EMITTER_TYPE
    /// </summary>
    public enum AdsbEmitterType
    {
        /// <summary>
        /// ADSB_EMITTER_TYPE_NO_INFO
        /// </summary>
        AdsbEmitterTypeNoInfo = 0,
        /// <summary>
        /// ADSB_EMITTER_TYPE_LIGHT
        /// </summary>
        AdsbEmitterTypeLight = 1,
        /// <summary>
        /// ADSB_EMITTER_TYPE_SMALL
        /// </summary>
        AdsbEmitterTypeSmall = 2,
        /// <summary>
        /// ADSB_EMITTER_TYPE_LARGE
        /// </summary>
        AdsbEmitterTypeLarge = 3,
        /// <summary>
        /// ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE
        /// </summary>
        AdsbEmitterTypeHighVortexLarge = 4,
        /// <summary>
        /// ADSB_EMITTER_TYPE_HEAVY
        /// </summary>
        AdsbEmitterTypeHeavy = 5,
        /// <summary>
        /// ADSB_EMITTER_TYPE_HIGHLY_MANUV
        /// </summary>
        AdsbEmitterTypeHighlyManuv = 6,
        /// <summary>
        /// ADSB_EMITTER_TYPE_ROTOCRAFT
        /// </summary>
        AdsbEmitterTypeRotocraft = 7,
        /// <summary>
        /// ADSB_EMITTER_TYPE_UNASSIGNED
        /// </summary>
        AdsbEmitterTypeUnassigned = 8,
        /// <summary>
        /// ADSB_EMITTER_TYPE_GLIDER
        /// </summary>
        AdsbEmitterTypeGlider = 9,
        /// <summary>
        /// ADSB_EMITTER_TYPE_LIGHTER_AIR
        /// </summary>
        AdsbEmitterTypeLighterAir = 10,
        /// <summary>
        /// ADSB_EMITTER_TYPE_PARACHUTE
        /// </summary>
        AdsbEmitterTypeParachute = 11,
        /// <summary>
        /// ADSB_EMITTER_TYPE_ULTRA_LIGHT
        /// </summary>
        AdsbEmitterTypeUltraLight = 12,
        /// <summary>
        /// ADSB_EMITTER_TYPE_UNASSIGNED2
        /// </summary>
        AdsbEmitterTypeUnassigned2 = 13,
        /// <summary>
        /// ADSB_EMITTER_TYPE_UAV
        /// </summary>
        AdsbEmitterTypeUav = 14,
        /// <summary>
        /// ADSB_EMITTER_TYPE_SPACE
        /// </summary>
        AdsbEmitterTypeSpace = 15,
        /// <summary>
        /// ADSB_EMITTER_TYPE_UNASSGINED3
        /// </summary>
        AdsbEmitterTypeUnassgined3 = 16,
        /// <summary>
        /// ADSB_EMITTER_TYPE_EMERGENCY_SURFACE
        /// </summary>
        AdsbEmitterTypeEmergencySurface = 17,
        /// <summary>
        /// ADSB_EMITTER_TYPE_SERVICE_SURFACE
        /// </summary>
        AdsbEmitterTypeServiceSurface = 18,
        /// <summary>
        /// ADSB_EMITTER_TYPE_POINT_OBSTACLE
        /// </summary>
        AdsbEmitterTypePointObstacle = 19,
    }

    /// <summary>
    /// These flags indicate status such as data validity of each data source. Set = data valid
    ///  ADSB_FLAGS
    /// </summary>
    public enum AdsbFlags
    {
        /// <summary>
        /// ADSB_FLAGS_VALID_COORDS
        /// </summary>
        AdsbFlagsValidCoords = 1,
        /// <summary>
        /// ADSB_FLAGS_VALID_ALTITUDE
        /// </summary>
        AdsbFlagsValidAltitude = 2,
        /// <summary>
        /// ADSB_FLAGS_VALID_HEADING
        /// </summary>
        AdsbFlagsValidHeading = 4,
        /// <summary>
        /// ADSB_FLAGS_VALID_VELOCITY
        /// </summary>
        AdsbFlagsValidVelocity = 8,
        /// <summary>
        /// ADSB_FLAGS_VALID_CALLSIGN
        /// </summary>
        AdsbFlagsValidCallsign = 16,
        /// <summary>
        /// ADSB_FLAGS_VALID_SQUAWK
        /// </summary>
        AdsbFlagsValidSquawk = 32,
        /// <summary>
        /// ADSB_FLAGS_SIMULATED
        /// </summary>
        AdsbFlagsSimulated = 64,
    }

    /// <summary>
    /// Bitmask of options for the MAV_CMD_DO_REPOSITION
    ///  MAV_DO_REPOSITION_FLAGS
    /// </summary>
    public enum MavDoRepositionFlags
    {
        /// <summary>
        /// The aircraft should immediately transition into guided. This should not be set for follow me applications
        /// MAV_DO_REPOSITION_FLAGS_CHANGE_MODE
        /// </summary>
        MavDoRepositionFlagsChangeMode = 1,
    }

    /// <summary>
    /// Flags in EKF_STATUS message
    ///  ESTIMATOR_STATUS_FLAGS
    /// </summary>
    public enum EstimatorStatusFlags
    {
        /// <summary>
        /// True if the attitude estimate is good
        /// ESTIMATOR_ATTITUDE
        /// </summary>
        EstimatorAttitude = 1,
        /// <summary>
        /// True if the horizontal velocity estimate is good
        /// ESTIMATOR_VELOCITY_HORIZ
        /// </summary>
        EstimatorVelocityHoriz = 2,
        /// <summary>
        /// True if the  vertical velocity estimate is good
        /// ESTIMATOR_VELOCITY_VERT
        /// </summary>
        EstimatorVelocityVert = 4,
        /// <summary>
        /// True if the horizontal position (relative) estimate is good
        /// ESTIMATOR_POS_HORIZ_REL
        /// </summary>
        EstimatorPosHorizRel = 8,
        /// <summary>
        /// True if the horizontal position (absolute) estimate is good
        /// ESTIMATOR_POS_HORIZ_ABS
        /// </summary>
        EstimatorPosHorizAbs = 16,
        /// <summary>
        /// True if the vertical position (absolute) estimate is good
        /// ESTIMATOR_POS_VERT_ABS
        /// </summary>
        EstimatorPosVertAbs = 32,
        /// <summary>
        /// True if the vertical position (above ground) estimate is good
        /// ESTIMATOR_POS_VERT_AGL
        /// </summary>
        EstimatorPosVertAgl = 64,
        /// <summary>
        /// True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
        /// ESTIMATOR_CONST_POS_MODE
        /// </summary>
        EstimatorConstPosMode = 128,
        /// <summary>
        /// True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
        /// ESTIMATOR_PRED_POS_HORIZ_REL
        /// </summary>
        EstimatorPredPosHorizRel = 256,
        /// <summary>
        /// True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
        /// ESTIMATOR_PRED_POS_HORIZ_ABS
        /// </summary>
        EstimatorPredPosHorizAbs = 512,
        /// <summary>
        /// True if the EKF has detected a GPS glitch
        /// ESTIMATOR_GPS_GLITCH
        /// </summary>
        EstimatorGpsGlitch = 1024,
        /// <summary>
        /// True if the EKF has detected bad accelerometer data
        /// ESTIMATOR_ACCEL_ERROR
        /// </summary>
        EstimatorAccelError = 2048,
    }

    /// <summary>
    ///  MOTOR_TEST_ORDER
    /// </summary>
    public enum MotorTestOrder
    {
        /// <summary>
        /// default autopilot motor test method
        /// MOTOR_TEST_ORDER_DEFAULT
        /// </summary>
        MotorTestOrderDefault = 0,
        /// <summary>
        /// motor numbers are specified as their index in a predefined vehicle-specific sequence
        /// MOTOR_TEST_ORDER_SEQUENCE
        /// </summary>
        MotorTestOrderSequence = 1,
        /// <summary>
        /// motor numbers are specified as the output as labeled on the board
        /// MOTOR_TEST_ORDER_BOARD
        /// </summary>
        MotorTestOrderBoard = 2,
    }

    /// <summary>
    ///  MOTOR_TEST_THROTTLE_TYPE
    /// </summary>
    public enum MotorTestThrottleType
    {
        /// <summary>
        /// throttle as a percentage from 0 ~ 100
        /// MOTOR_TEST_THROTTLE_PERCENT
        /// </summary>
        MotorTestThrottlePercent = 0,
        /// <summary>
        /// throttle as an absolute PWM value (normally in range of 1000~2000)
        /// MOTOR_TEST_THROTTLE_PWM
        /// </summary>
        MotorTestThrottlePwm = 1,
        /// <summary>
        /// throttle pass-through from pilot's transmitter
        /// MOTOR_TEST_THROTTLE_PILOT
        /// </summary>
        MotorTestThrottlePilot = 2,
        /// <summary>
        /// per-motor compass calibration test
        /// MOTOR_TEST_COMPASS_CAL
        /// </summary>
        MotorTestCompassCal = 3,
    }

    /// <summary>
    ///  GPS_INPUT_IGNORE_FLAGS
    /// </summary>
    public enum GpsInputIgnoreFlags
    {
        /// <summary>
        /// ignore altitude field
        /// GPS_INPUT_IGNORE_FLAG_ALT
        /// </summary>
        GpsInputIgnoreFlagAlt = 1,
        /// <summary>
        /// ignore hdop field
        /// GPS_INPUT_IGNORE_FLAG_HDOP
        /// </summary>
        GpsInputIgnoreFlagHdop = 2,
        /// <summary>
        /// ignore vdop field
        /// GPS_INPUT_IGNORE_FLAG_VDOP
        /// </summary>
        GpsInputIgnoreFlagVdop = 4,
        /// <summary>
        /// ignore horizontal velocity field (vn and ve)
        /// GPS_INPUT_IGNORE_FLAG_VEL_HORIZ
        /// </summary>
        GpsInputIgnoreFlagVelHoriz = 8,
        /// <summary>
        /// ignore vertical velocity field (vd)
        /// GPS_INPUT_IGNORE_FLAG_VEL_VERT
        /// </summary>
        GpsInputIgnoreFlagVelVert = 16,
        /// <summary>
        /// ignore speed accuracy field
        /// GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY
        /// </summary>
        GpsInputIgnoreFlagSpeedAccuracy = 32,
        /// <summary>
        /// ignore horizontal accuracy field
        /// GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY
        /// </summary>
        GpsInputIgnoreFlagHorizontalAccuracy = 64,
        /// <summary>
        /// ignore vertical accuracy field
        /// GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY
        /// </summary>
        GpsInputIgnoreFlagVerticalAccuracy = 128,
    }

    /// <summary>
    /// Possible actions an aircraft can take to avoid a collision.
    ///  MAV_COLLISION_ACTION
    /// </summary>
    public enum MavCollisionAction
    {
        /// <summary>
        /// Ignore any potential collisions
        /// MAV_COLLISION_ACTION_NONE
        /// </summary>
        MavCollisionActionNone = 0,
        /// <summary>
        /// Report potential collision
        /// MAV_COLLISION_ACTION_REPORT
        /// </summary>
        MavCollisionActionReport = 1,
        /// <summary>
        /// Ascend or Descend to avoid threat
        /// MAV_COLLISION_ACTION_ASCEND_OR_DESCEND
        /// </summary>
        MavCollisionActionAscendOrDescend = 2,
        /// <summary>
        /// Move horizontally to avoid threat
        /// MAV_COLLISION_ACTION_MOVE_HORIZONTALLY
        /// </summary>
        MavCollisionActionMoveHorizontally = 3,
        /// <summary>
        /// Aircraft to move perpendicular to the collision's velocity vector
        /// MAV_COLLISION_ACTION_MOVE_PERPENDICULAR
        /// </summary>
        MavCollisionActionMovePerpendicular = 4,
        /// <summary>
        /// Aircraft to fly directly back to its launch point
        /// MAV_COLLISION_ACTION_RTL
        /// </summary>
        MavCollisionActionRtl = 5,
        /// <summary>
        /// Aircraft to stop in place
        /// MAV_COLLISION_ACTION_HOVER
        /// </summary>
        MavCollisionActionHover = 6,
    }

    /// <summary>
    /// Aircraft-rated danger from this threat.
    ///  MAV_COLLISION_THREAT_LEVEL
    /// </summary>
    public enum MavCollisionThreatLevel
    {
        /// <summary>
        /// Not a threat
        /// MAV_COLLISION_THREAT_LEVEL_NONE
        /// </summary>
        MavCollisionThreatLevelNone = 0,
        /// <summary>
        /// Craft is mildly concerned about this threat
        /// MAV_COLLISION_THREAT_LEVEL_LOW
        /// </summary>
        MavCollisionThreatLevelLow = 1,
        /// <summary>
        /// Craft is panicing, and may take actions to avoid threat
        /// MAV_COLLISION_THREAT_LEVEL_HIGH
        /// </summary>
        MavCollisionThreatLevelHigh = 2,
    }

    /// <summary>
    /// Source of information about this collision.
    ///  MAV_COLLISION_SRC
    /// </summary>
    public enum MavCollisionSrc
    {
        /// <summary>
        /// ID field references ADSB_VEHICLE packets
        /// MAV_COLLISION_SRC_ADSB
        /// </summary>
        MavCollisionSrcAdsb = 0,
        /// <summary>
        /// ID field references MAVLink SRC ID
        /// MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT
        /// </summary>
        MavCollisionSrcMavlinkGpsGlobalInt = 1,
    }

    /// <summary>
    /// Type of GPS fix
    ///  GPS_FIX_TYPE
    /// </summary>
    public enum GpsFixType
    {
        /// <summary>
        /// No GPS connected
        /// GPS_FIX_TYPE_NO_GPS
        /// </summary>
        GpsFixTypeNoGps = 0,
        /// <summary>
        /// No position information, GPS is connected
        /// GPS_FIX_TYPE_NO_FIX
        /// </summary>
        GpsFixTypeNoFix = 1,
        /// <summary>
        /// 2D position
        /// GPS_FIX_TYPE_2D_FIX
        /// </summary>
        GpsFixType2dFix = 2,
        /// <summary>
        /// 3D position
        /// GPS_FIX_TYPE_3D_FIX
        /// </summary>
        GpsFixType3dFix = 3,
        /// <summary>
        /// DGPS/SBAS aided 3D position
        /// GPS_FIX_TYPE_DGPS
        /// </summary>
        GpsFixTypeDgps = 4,
        /// <summary>
        /// RTK float, 3D position
        /// GPS_FIX_TYPE_RTK_FLOAT
        /// </summary>
        GpsFixTypeRtkFloat = 5,
        /// <summary>
        /// RTK Fixed, 3D position
        /// GPS_FIX_TYPE_RTK_FIXED
        /// </summary>
        GpsFixTypeRtkFixed = 6,
        /// <summary>
        /// Static fixed, typically used for base stations
        /// GPS_FIX_TYPE_STATIC
        /// </summary>
        GpsFixTypeStatic = 7,
        /// <summary>
        /// PPP, 3D position.
        /// GPS_FIX_TYPE_PPP
        /// </summary>
        GpsFixTypePpp = 8,
    }

    /// <summary>
    /// Type of landing target
    ///  LANDING_TARGET_TYPE
    /// </summary>
    public enum LandingTargetType
    {
        /// <summary>
        /// Landing target signaled by light beacon (ex: IR-LOCK)
        /// LANDING_TARGET_TYPE_LIGHT_BEACON
        /// </summary>
        LandingTargetTypeLightBeacon = 0,
        /// <summary>
        /// Landing target signaled by radio beacon (ex: ILS, NDB)
        /// LANDING_TARGET_TYPE_RADIO_BEACON
        /// </summary>
        LandingTargetTypeRadioBeacon = 1,
        /// <summary>
        /// Landing target represented by a fiducial marker (ex: ARTag)
        /// LANDING_TARGET_TYPE_VISION_FIDUCIAL
        /// </summary>
        LandingTargetTypeVisionFiducial = 2,
        /// <summary>
        /// Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
        /// LANDING_TARGET_TYPE_VISION_OTHER
        /// </summary>
        LandingTargetTypeVisionOther = 3,
    }

    /// <summary>
    /// Direction of VTOL transition
    ///  VTOL_TRANSITION_HEADING
    /// </summary>
    public enum VtolTransitionHeading
    {
        /// <summary>
        /// Respect the heading configuration of the vehicle.
        /// VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT
        /// </summary>
        VtolTransitionHeadingVehicleDefault = 0,
        /// <summary>
        /// Use the heading pointing towards the next waypoint.
        /// VTOL_TRANSITION_HEADING_NEXT_WAYPOINT
        /// </summary>
        VtolTransitionHeadingNextWaypoint = 1,
        /// <summary>
        /// Use the heading on takeoff (while sitting on the ground).
        /// VTOL_TRANSITION_HEADING_TAKEOFF
        /// </summary>
        VtolTransitionHeadingTakeoff = 2,
        /// <summary>
        /// Use the specified heading in parameter 4.
        /// VTOL_TRANSITION_HEADING_SPECIFIED
        /// </summary>
        VtolTransitionHeadingSpecified = 3,
        /// <summary>
        /// Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning is active).
        /// VTOL_TRANSITION_HEADING_ANY
        /// </summary>
        VtolTransitionHeadingAny = 4,
    }

    /// <summary>
    /// Camera capability flags (Bitmap).
    ///  CAMERA_CAP_FLAGS
    /// </summary>
    public enum CameraCapFlags
    {
        /// <summary>
        /// Camera is able to record video.
        /// CAMERA_CAP_FLAGS_CAPTURE_VIDEO
        /// </summary>
        CameraCapFlagsCaptureVideo = 1,
        /// <summary>
        /// Camera is able to capture images.
        /// CAMERA_CAP_FLAGS_CAPTURE_IMAGE
        /// </summary>
        CameraCapFlagsCaptureImage = 2,
        /// <summary>
        /// Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
        /// CAMERA_CAP_FLAGS_HAS_MODES
        /// </summary>
        CameraCapFlagsHasModes = 4,
        /// <summary>
        /// Camera can capture images while in video mode
        /// CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE
        /// </summary>
        CameraCapFlagsCanCaptureImageInVideoMode = 8,
        /// <summary>
        /// Camera can capture videos while in Photo/Image mode
        /// CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE
        /// </summary>
        CameraCapFlagsCanCaptureVideoInImageMode = 16,
        /// <summary>
        /// Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)
        /// CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE
        /// </summary>
        CameraCapFlagsHasImageSurveyMode = 32,
    }

    /// <summary>
    /// Result from a PARAM_EXT_SET message.
    ///  PARAM_ACK
    /// </summary>
    public enum ParamAck
    {
        /// <summary>
        /// Parameter value ACCEPTED and SET
        /// PARAM_ACK_ACCEPTED
        /// </summary>
        ParamAckAccepted = 0,
        /// <summary>
        /// Parameter value UNKNOWN/UNSUPPORTED
        /// PARAM_ACK_VALUE_UNSUPPORTED
        /// </summary>
        ParamAckValueUnsupported = 1,
        /// <summary>
        /// Parameter failed to set
        /// PARAM_ACK_FAILED
        /// </summary>
        ParamAckFailed = 2,
        /// <summary>
        /// Parameter value received but not yet validated or set. A subsequent PARAM_EXT_ACK will follow once operation is completed with the actual result. These are for parameters that may take longer to set. Instead of waiting for an ACK and potentially timing out, you will immediately receive this response to let you know it was received.
        /// PARAM_ACK_IN_PROGRESS
        /// </summary>
        ParamAckInProgress = 3,
    }

    /// <summary>
    /// Camera Modes.
    ///  CAMERA_MODE
    /// </summary>
    public enum CameraMode
    {
        /// <summary>
        /// Camera is in image/photo capture mode.
        /// CAMERA_MODE_IMAGE
        /// </summary>
        CameraModeImage = 0,
        /// <summary>
        /// Camera is in video capture mode.
        /// CAMERA_MODE_VIDEO
        /// </summary>
        CameraModeVideo = 1,
        /// <summary>
        /// Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys.
        /// CAMERA_MODE_IMAGE_SURVEY
        /// </summary>
        CameraModeImageSurvey = 2,
    }

    /// <summary>
    ///  MAV_ARM_AUTH_DENIED_REASON
    /// </summary>
    public enum MavArmAuthDeniedReason
    {
        /// <summary>
        /// Not a specific reason
        /// MAV_ARM_AUTH_DENIED_REASON_GENERIC
        /// </summary>
        MavArmAuthDeniedReasonGeneric = 0,
        /// <summary>
        /// Authorizer will send the error as string to GCS
        /// MAV_ARM_AUTH_DENIED_REASON_NONE
        /// </summary>
        MavArmAuthDeniedReasonNone = 1,
        /// <summary>
        /// At least one waypoint have a invalid value
        /// MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT
        /// </summary>
        MavArmAuthDeniedReasonInvalidWaypoint = 2,
        /// <summary>
        /// Timeout in the authorizer process(in case it depends on network)
        /// MAV_ARM_AUTH_DENIED_REASON_TIMEOUT
        /// </summary>
        MavArmAuthDeniedReasonTimeout = 3,
        /// <summary>
        /// Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that caused it to be denied.
        /// MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE
        /// </summary>
        MavArmAuthDeniedReasonAirspaceInUse = 4,
        /// <summary>
        /// Weather is not good to fly
        /// MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER
        /// </summary>
        MavArmAuthDeniedReasonBadWeather = 5,
    }

    /// <summary>
    /// RTK GPS baseline coordinate system, used for RTK corrections
    ///  RTK_BASELINE_COORDINATE_SYSTEM
    /// </summary>
    public enum RtkBaselineCoordinateSystem
    {
        /// <summary>
        /// Earth-centered, Earth-fixed
        /// RTK_BASELINE_COORDINATE_SYSTEM_ECEF
        /// </summary>
        RtkBaselineCoordinateSystemEcef = 0,
        /// <summary>
        /// North, East, Down
        /// RTK_BASELINE_COORDINATE_SYSTEM_NED
        /// </summary>
        RtkBaselineCoordinateSystemNed = 1,
    }

    /// <summary>
    /// RC type
    ///  RC_TYPE
    /// </summary>
    public enum RcType
    {
        /// <summary>
        /// Spektrum DSM2
        /// RC_TYPE_SPEKTRUM_DSM2
        /// </summary>
        RcTypeSpektrumDsm2 = 0,
        /// <summary>
        /// Spektrum DSMX
        /// RC_TYPE_SPEKTRUM_DSMX
        /// </summary>
        RcTypeSpektrumDsmx = 1,
    }

    /// <summary>
    /// WORK IN PROGRESS! DO NOT DEPLOY! Enumeration of possible waypoint/trajectory representation
    ///  MAV_TRAJECTORY_REPRESENTATION
    /// </summary>
    public enum MavTrajectoryRepresentation
    {
        /// <summary>
        /// Array of waypoints with the following order
        /// Param 1 - X-coordinate of waypoint [m], set to NaN if not being used
        /// Param 2 - Y-coordinate of waypoint [m], set to NaN if not being used
        /// Param 3 - Z-coordinate of waypoint [m], set to NaN if not being used
        /// Param 4 - X-velocity of waypoint [m/s], set to NaN if not being used
        /// Param 5 - Y-velocity of waypoint [m/s], set to NaN if not being used
        /// Param 6 - Z-velocity of waypoint [m/s], set to NaN if not being used
        /// Param 7 - X-acceleration of waypoint [m/s/s], set to NaN if not being used
        /// Param 8 - Y-acceleration of waypoint [m/s/s], set to NaN if not being used
        /// Param 9 - Z-acceleration of waypoint [m/s/s], set to NaN if not being used
        /// Param 10 - Yaw [rad], set to NaN for unchanged
        /// Param 11 - Yaw-rate [rad/s], set to NaN for unchanged
        /// MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS
        /// </summary>
        MavTrajectoryRepresentationWaypoints = 0,
        /// <summary>
        /// WORK IN PROGRESS! DO NOT DEPLOY! Array of bezier points with the following order
        /// Param 1 - X-coordinate of starting bezier point [m], set to NaN if not being used
        /// Param 2 - Y-coordinate of starting bezier point [m], set to NaN if not being used
        /// Param 3 - Z-coordinate of starting bezier point [m], set to NaN if not being used
        /// Param 4 - Bezier time horizon [s], set to NaN if velocity/acceleration should not be incorporated
        /// Param 5 - Yaw [rad], set to NaN for unchanged
        /// MAV_TRAJECTORY_REPRESENTATION_BEZIER
        /// </summary>
        MavTrajectoryRepresentationBezier = 1,
    }


#endregion

#region Messages

    /// <summary>
    /// The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot).
    ///  HEARTBEAT
    /// </summary>
    public class HeartbeatPacket: PacketV2<HeartbeatPayload>
    {
	public const int PacketMessageId = 0;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 50;

        public override HeartbeatPayload Payload { get; } = new HeartbeatPayload();

        public override string Name => "HEARTBEAT";
    }

    /// <summary>
    ///  HEARTBEAT
    /// </summary>
    public class HeartbeatPayload : IPayload
    {
        public byte GetMaxByteSize() => 9;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            CustomMode = BitConverter.ToUInt32(buffer,index);index+=4;
            Type = (MavType)buffer[index++];
            Autopilot = (MavAutopilot)buffer[index++];
            BaseMode = (MavModeFlag)buffer[index++];
            SystemStatus = (MavState)buffer[index++];
            MavlinkVersion = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(CustomMode).CopyTo(buffer, index);index+=4;
            buffer[index] = (byte)Type;index+=1;
            buffer[index] = (byte)Autopilot;index+=1;
            buffer[index] = (byte)BaseMode;index+=1;
            buffer[index] = (byte)SystemStatus;index+=1;
            BitConverter.GetBytes(MavlinkVersion).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/9;
        }

        /// <summary>
        /// A bitfield for use for autopilot-specific flags
        /// OriginName: custom_mode, Units: , IsExtended: false
        /// </summary>
        public uint CustomMode { get; set; }
        /// <summary>
        /// Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        /// OriginName: type, Units: , IsExtended: false
        /// </summary>
        public MavType Type { get; set; }
        /// <summary>
        /// Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        /// OriginName: autopilot, Units: , IsExtended: false
        /// </summary>
        public MavAutopilot Autopilot { get; set; }
        /// <summary>
        /// System mode bitfield, as defined by MAV_MODE_FLAG enum
        /// OriginName: base_mode, Units: , IsExtended: false
        /// </summary>
        public MavModeFlag BaseMode { get; set; }
        /// <summary>
        /// System status flag, as defined by MAV_STATE enum
        /// OriginName: system_status, Units: , IsExtended: false
        /// </summary>
        public MavState SystemStatus { get; set; }
        /// <summary>
        /// MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
        /// OriginName: mavlink_version, Units: , IsExtended: false
        /// </summary>
        public byte MavlinkVersion { get; set; }
    }
    /// <summary>
    /// The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows whether the system is currently active or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occured it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
    ///  SYS_STATUS
    /// </summary>
    public class SysStatusPacket: PacketV2<SysStatusPayload>
    {
	public const int PacketMessageId = 1;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 124;

        public override SysStatusPayload Payload { get; } = new SysStatusPayload();

        public override string Name => "SYS_STATUS";
    }

    /// <summary>
    ///  SYS_STATUS
    /// </summary>
    public class SysStatusPayload : IPayload
    {
        public byte GetMaxByteSize() => 31;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            OnboardControlSensorsPresent = (MavSysStatusSensor)BitConverter.ToUInt32(buffer,index);index+=4;
            OnboardControlSensorsEnabled = (MavSysStatusSensor)BitConverter.ToUInt32(buffer,index);index+=4;
            OnboardControlSensorsHealth = (MavSysStatusSensor)BitConverter.ToUInt32(buffer,index);index+=4;
            Load = BitConverter.ToUInt16(buffer,index);index+=2;
            VoltageBattery = BitConverter.ToUInt16(buffer,index);index+=2;
            CurrentBattery = BitConverter.ToInt16(buffer,index);index+=2;
            DropRateComm = BitConverter.ToUInt16(buffer,index);index+=2;
            ErrorsComm = BitConverter.ToUInt16(buffer,index);index+=2;
            ErrorsCount1 = BitConverter.ToUInt16(buffer,index);index+=2;
            ErrorsCount2 = BitConverter.ToUInt16(buffer,index);index+=2;
            ErrorsCount3 = BitConverter.ToUInt16(buffer,index);index+=2;
            ErrorsCount4 = BitConverter.ToUInt16(buffer,index);index+=2;
            BatteryRemaining = (sbyte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes((uint)OnboardControlSensorsPresent).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes((uint)OnboardControlSensorsEnabled).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes((uint)OnboardControlSensorsHealth).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Load).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(VoltageBattery).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(CurrentBattery).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(DropRateComm).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(ErrorsComm).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(ErrorsCount1).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(ErrorsCount2).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(ErrorsCount3).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(ErrorsCount4).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(BatteryRemaining).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/31;
        }

        /// <summary>
        /// Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
        /// OriginName: onboard_control_sensors_present, Units: , IsExtended: false
        /// </summary>
        public MavSysStatusSensor OnboardControlSensorsPresent { get; set; }
        /// <summary>
        /// Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
        /// OriginName: onboard_control_sensors_enabled, Units: , IsExtended: false
        /// </summary>
        public MavSysStatusSensor OnboardControlSensorsEnabled { get; set; }
        /// <summary>
        /// Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
        /// OriginName: onboard_control_sensors_health, Units: , IsExtended: false
        /// </summary>
        public MavSysStatusSensor OnboardControlSensorsHealth { get; set; }
        /// <summary>
        /// Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
        /// OriginName: load, Units: d%, IsExtended: false
        /// </summary>
        public ushort Load { get; set; }
        /// <summary>
        /// Battery voltage, in millivolts (1 = 1 millivolt)
        /// OriginName: voltage_battery, Units: mV, IsExtended: false
        /// </summary>
        public ushort VoltageBattery { get; set; }
        /// <summary>
        /// Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
        /// OriginName: current_battery, Units: cA, IsExtended: false
        /// </summary>
        public short CurrentBattery { get; set; }
        /// <summary>
        /// Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        /// OriginName: drop_rate_comm, Units: c%, IsExtended: false
        /// </summary>
        public ushort DropRateComm { get; set; }
        /// <summary>
        /// Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        /// OriginName: errors_comm, Units: , IsExtended: false
        /// </summary>
        public ushort ErrorsComm { get; set; }
        /// <summary>
        /// Autopilot-specific errors
        /// OriginName: errors_count1, Units: , IsExtended: false
        /// </summary>
        public ushort ErrorsCount1 { get; set; }
        /// <summary>
        /// Autopilot-specific errors
        /// OriginName: errors_count2, Units: , IsExtended: false
        /// </summary>
        public ushort ErrorsCount2 { get; set; }
        /// <summary>
        /// Autopilot-specific errors
        /// OriginName: errors_count3, Units: , IsExtended: false
        /// </summary>
        public ushort ErrorsCount3 { get; set; }
        /// <summary>
        /// Autopilot-specific errors
        /// OriginName: errors_count4, Units: , IsExtended: false
        /// </summary>
        public ushort ErrorsCount4 { get; set; }
        /// <summary>
        /// Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
        /// OriginName: battery_remaining, Units: %, IsExtended: false
        /// </summary>
        public sbyte BatteryRemaining { get; set; }
    }
    /// <summary>
    /// The system time is the time of the master clock, typically the computer clock of the main onboard computer.
    ///  SYSTEM_TIME
    /// </summary>
    public class SystemTimePacket: PacketV2<SystemTimePayload>
    {
	public const int PacketMessageId = 2;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 137;

        public override SystemTimePayload Payload { get; } = new SystemTimePayload();

        public override string Name => "SYSTEM_TIME";
    }

    /// <summary>
    ///  SYSTEM_TIME
    /// </summary>
    public class SystemTimePayload : IPayload
    {
        public byte GetMaxByteSize() => 12;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUnixUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUnixUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/12;
        }

        /// <summary>
        /// Timestamp of the master clock in microseconds since UNIX epoch.
        /// OriginName: time_unix_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUnixUsec { get; set; }
        /// <summary>
        /// Timestamp of the component clock since boot time in milliseconds.
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
    }
    /// <summary>
    /// A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections.
    ///  PING
    /// </summary>
    public class PingPacket: PacketV2<PingPayload>
    {
	public const int PacketMessageId = 4;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 237;

        public override PingPayload Payload { get; } = new PingPayload();

        public override string Name => "PING";
    }

    /// <summary>
    ///  PING
    /// </summary>
    public class PingPayload : IPayload
    {
        public byte GetMaxByteSize() => 14;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Seq = BitConverter.ToUInt32(buffer,index);index+=4;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/14;
        }

        /// <summary>
        /// Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// PING sequence
        /// OriginName: seq, Units: , IsExtended: false
        /// </summary>
        public uint Seq { get; set; }
        /// <summary>
        /// 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Request to control this MAV
    ///  CHANGE_OPERATOR_CONTROL
    /// </summary>
    public class ChangeOperatorControlPacket: PacketV2<ChangeOperatorControlPayload>
    {
	public const int PacketMessageId = 5;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 217;

        public override ChangeOperatorControlPayload Payload { get; } = new ChangeOperatorControlPayload();

        public override string Name => "CHANGE_OPERATOR_CONTROL";
    }

    /// <summary>
    ///  CHANGE_OPERATOR_CONTROL
    /// </summary>
    public class ChangeOperatorControlPayload : IPayload
    {
        public byte GetMaxByteSize() => 28;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TargetSystem = (byte)buffer[index++];
            ControlRequest = (byte)buffer[index++];
            Version = (byte)buffer[index++];
            arraySize = /*ArrayLength*/25 - Math.Max(0,((/*PayloadByteSize*/28 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<25;i++)
            {
                Passkey[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,Passkey,0);
                index+=25;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(ControlRequest).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Version).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(Passkey,0,25,buffer,index);index+=25;
            return /*PayloadByteSize*/28;
        }

        /// <summary>
        /// System the GCS requests control for
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// 0: request control of this MAV, 1: Release control of this MAV
        /// OriginName: control_request, Units: , IsExtended: false
        /// </summary>
        public byte ControlRequest { get; set; }
        /// <summary>
        /// 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
        /// OriginName: version, Units: rad, IsExtended: false
        /// </summary>
        public byte Version { get; set; }
        /// <summary>
        /// Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
        /// OriginName: passkey, Units: , IsExtended: false
        /// </summary>
        public char[] Passkey { get; } = new char[25];
    }
    /// <summary>
    /// Accept / deny control of this MAV
    ///  CHANGE_OPERATOR_CONTROL_ACK
    /// </summary>
    public class ChangeOperatorControlAckPacket: PacketV2<ChangeOperatorControlAckPayload>
    {
	public const int PacketMessageId = 6;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 104;

        public override ChangeOperatorControlAckPayload Payload { get; } = new ChangeOperatorControlAckPayload();

        public override string Name => "CHANGE_OPERATOR_CONTROL_ACK";
    }

    /// <summary>
    ///  CHANGE_OPERATOR_CONTROL_ACK
    /// </summary>
    public class ChangeOperatorControlAckPayload : IPayload
    {
        public byte GetMaxByteSize() => 3;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            GcsSystemId = (byte)buffer[index++];
            ControlRequest = (byte)buffer[index++];
            Ack = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(GcsSystemId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(ControlRequest).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Ack).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/3;
        }

        /// <summary>
        /// ID of the GCS this message 
        /// OriginName: gcs_system_id, Units: , IsExtended: false
        /// </summary>
        public byte GcsSystemId { get; set; }
        /// <summary>
        /// 0: request control of this MAV, 1: Release control of this MAV
        /// OriginName: control_request, Units: , IsExtended: false
        /// </summary>
        public byte ControlRequest { get; set; }
        /// <summary>
        /// 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
        /// OriginName: ack, Units: , IsExtended: false
        /// </summary>
        public byte Ack { get; set; }
    }
    /// <summary>
    /// Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.
    ///  AUTH_KEY
    /// </summary>
    public class AuthKeyPacket: PacketV2<AuthKeyPayload>
    {
	public const int PacketMessageId = 7;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 119;

        public override AuthKeyPayload Payload { get; } = new AuthKeyPayload();

        public override string Name => "AUTH_KEY";
    }

    /// <summary>
    ///  AUTH_KEY
    /// </summary>
    public class AuthKeyPayload : IPayload
    {
        public byte GetMaxByteSize() => 32;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            arraySize = /*ArrayLength*/32 - Math.Max(0,((/*PayloadByteSize*/32 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<32;i++)
            {
                Key[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,Key,0);
                index+=32;
        }

        public int Serialize(byte[] buffer, int index)
        {
            Encoding.ASCII.GetBytes(Key,0,32,buffer,index);index+=32;
            return /*PayloadByteSize*/32;
        }

        /// <summary>
        /// key
        /// OriginName: key, Units: , IsExtended: false
        /// </summary>
        public char[] Key { get; } = new char[32];
    }
    /// <summary>
    /// THIS INTERFACE IS DEPRECATED. USE COMMAND_LONG with MAV_CMD_DO_SET_MODE INSTEAD. Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.
    ///  SET_MODE
    /// </summary>
    public class SetModePacket: PacketV2<SetModePayload>
    {
	public const int PacketMessageId = 11;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 89;

        public override SetModePayload Payload { get; } = new SetModePayload();

        public override string Name => "SET_MODE";
    }

    /// <summary>
    ///  SET_MODE
    /// </summary>
    public class SetModePayload : IPayload
    {
        public byte GetMaxByteSize() => 6;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            CustomMode = BitConverter.ToUInt32(buffer,index);index+=4;
            TargetSystem = (byte)buffer[index++];
            BaseMode = (MavMode)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(CustomMode).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)BaseMode;index+=1;
            return /*PayloadByteSize*/6;
        }

        /// <summary>
        /// The new autopilot-specific mode. This field can be ignored by an autopilot.
        /// OriginName: custom_mode, Units: , IsExtended: false
        /// </summary>
        public uint CustomMode { get; set; }
        /// <summary>
        /// The system setting the mode
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// The new base mode
        /// OriginName: base_mode, Units: , IsExtended: false
        /// </summary>
        public MavMode BaseMode { get; set; }
    }
    /// <summary>
    /// Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also https://mavlink.io/en/protocol/parameter.html for a full documentation of QGroundControl and IMU code.
    ///  PARAM_REQUEST_READ
    /// </summary>
    public class ParamRequestReadPacket: PacketV2<ParamRequestReadPayload>
    {
	public const int PacketMessageId = 20;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 214;

        public override ParamRequestReadPayload Payload { get; } = new ParamRequestReadPayload();

        public override string Name => "PARAM_REQUEST_READ";
    }

    /// <summary>
    ///  PARAM_REQUEST_READ
    /// </summary>
    public class ParamRequestReadPayload : IPayload
    {
        public byte GetMaxByteSize() => 20;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            ParamIndex = BitConverter.ToInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            arraySize = /*ArrayLength*/16 - Math.Max(0,((/*PayloadByteSize*/20 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<16;i++)
            {
                ParamId[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,ParamId,0);
                index+=16;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ParamIndex).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);index+=16;
            return /*PayloadByteSize*/20;
        }

        /// <summary>
        /// Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
        /// OriginName: param_index, Units: , IsExtended: false
        /// </summary>
        public short ParamIndex { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// OriginName: param_id, Units: , IsExtended: false
        /// </summary>
        public char[] ParamId { get; } = new char[16];
    }
    /// <summary>
    /// Request all parameters of this component. After this request, all parameters are emitted.
    ///  PARAM_REQUEST_LIST
    /// </summary>
    public class ParamRequestListPacket: PacketV2<ParamRequestListPayload>
    {
	public const int PacketMessageId = 21;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 159;

        public override ParamRequestListPayload Payload { get; } = new ParamRequestListPayload();

        public override string Name => "PARAM_REQUEST_LIST";
    }

    /// <summary>
    ///  PARAM_REQUEST_LIST
    /// </summary>
    public class ParamRequestListPayload : IPayload
    {
        public byte GetMaxByteSize() => 2;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/2;
        }

        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout.
    ///  PARAM_VALUE
    /// </summary>
    public class ParamValuePacket: PacketV2<ParamValuePayload>
    {
	public const int PacketMessageId = 22;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 220;

        public override ParamValuePayload Payload { get; } = new ParamValuePayload();

        public override string Name => "PARAM_VALUE";
    }

    /// <summary>
    ///  PARAM_VALUE
    /// </summary>
    public class ParamValuePayload : IPayload
    {
        public byte GetMaxByteSize() => 25;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            ParamValue = BitConverter.ToSingle(buffer, index);index+=4;
            ParamCount = BitConverter.ToUInt16(buffer,index);index+=2;
            ParamIndex = BitConverter.ToUInt16(buffer,index);index+=2;
            arraySize = /*ArrayLength*/16 - Math.Max(0,((/*PayloadByteSize*/25 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<16;i++)
            {
                ParamId[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,ParamId,0);
                index+=16;
            ParamType = (MavParamType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ParamValue).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ParamCount).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(ParamIndex).CopyTo(buffer, index);index+=2;
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);index+=16;
            buffer[index] = (byte)ParamType;index+=1;
            return /*PayloadByteSize*/25;
        }

        /// <summary>
        /// Onboard parameter value
        /// OriginName: param_value, Units: , IsExtended: false
        /// </summary>
        public float ParamValue { get; set; }
        /// <summary>
        /// Total number of onboard parameters
        /// OriginName: param_count, Units: , IsExtended: false
        /// </summary>
        public ushort ParamCount { get; set; }
        /// <summary>
        /// Index of this onboard parameter
        /// OriginName: param_index, Units: , IsExtended: false
        /// </summary>
        public ushort ParamIndex { get; set; }
        /// <summary>
        /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// OriginName: param_id, Units: , IsExtended: false
        /// </summary>
        public char[] ParamId { get; } = new char[16];
        /// <summary>
        /// Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
        /// OriginName: param_type, Units: , IsExtended: false
        /// </summary>
        public MavParamType ParamType { get; set; }
    }
    /// <summary>
    /// Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component should acknowledge the new parameter value by sending a param_value message to all communication partners. This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message.
    ///  PARAM_SET
    /// </summary>
    public class ParamSetPacket: PacketV2<ParamSetPayload>
    {
	public const int PacketMessageId = 23;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 168;

        public override ParamSetPayload Payload { get; } = new ParamSetPayload();

        public override string Name => "PARAM_SET";
    }

    /// <summary>
    ///  PARAM_SET
    /// </summary>
    public class ParamSetPayload : IPayload
    {
        public byte GetMaxByteSize() => 23;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            ParamValue = BitConverter.ToSingle(buffer, index);index+=4;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            arraySize = /*ArrayLength*/16 - Math.Max(0,((/*PayloadByteSize*/23 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<16;i++)
            {
                ParamId[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,ParamId,0);
                index+=16;
            ParamType = (MavParamType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ParamValue).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);index+=16;
            buffer[index] = (byte)ParamType;index+=1;
            return /*PayloadByteSize*/23;
        }

        /// <summary>
        /// Onboard parameter value
        /// OriginName: param_value, Units: , IsExtended: false
        /// </summary>
        public float ParamValue { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// OriginName: param_id, Units: , IsExtended: false
        /// </summary>
        public char[] ParamId { get; } = new char[16];
        /// <summary>
        /// Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
        /// OriginName: param_type, Units: , IsExtended: false
        /// </summary>
        public MavParamType ParamType { get; set; }
    }
    /// <summary>
    /// The global position, as returned by the Global Positioning System (GPS). This is
    ///                 NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate.
    ///  GPS_RAW_INT
    /// </summary>
    public class GpsRawIntPacket: PacketV2<GpsRawIntPayload>
    {
	public const int PacketMessageId = 24;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 24;

        public override GpsRawIntPayload Payload { get; } = new GpsRawIntPayload();

        public override string Name => "GPS_RAW_INT";
    }

    /// <summary>
    ///  GPS_RAW_INT
    /// </summary>
    public class GpsRawIntPayload : IPayload
    {
        public byte GetMaxByteSize() => 50;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToInt32(buffer,index);index+=4;
            Eph = BitConverter.ToUInt16(buffer,index);index+=2;
            Epv = BitConverter.ToUInt16(buffer,index);index+=2;
            Vel = BitConverter.ToUInt16(buffer,index);index+=2;
            Cog = BitConverter.ToUInt16(buffer,index);index+=2;
            FixType = (GpsFixType)buffer[index++];
            SatellitesVisible = (byte)buffer[index++];
            // extended field 'AltEllipsoid' can be empty
            if (index >= endIndex) return;
            AltEllipsoid = BitConverter.ToInt32(buffer,index);index+=4;
            // extended field 'HAcc' can be empty
            if (index >= endIndex) return;
            HAcc = BitConverter.ToUInt32(buffer,index);index+=4;
            // extended field 'VAcc' can be empty
            if (index >= endIndex) return;
            VAcc = BitConverter.ToUInt32(buffer,index);index+=4;
            // extended field 'VelAcc' can be empty
            if (index >= endIndex) return;
            VelAcc = BitConverter.ToUInt32(buffer,index);index+=4;
            // extended field 'HdgAcc' can be empty
            if (index >= endIndex) return;
            HdgAcc = BitConverter.ToUInt32(buffer,index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Eph).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Epv).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Vel).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Cog).CopyTo(buffer, index);index+=2;
            buffer[index] = (byte)FixType;index+=1;
            BitConverter.GetBytes(SatellitesVisible).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(AltEllipsoid).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(HAcc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(VAcc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(VelAcc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(HdgAcc).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/50;
        }

        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
        /// OriginName: alt, Units: mm, IsExtended: false
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
        /// OriginName: eph, Units: , IsExtended: false
        /// </summary>
        public ushort Eph { get; set; }
        /// <summary>
        /// GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
        /// OriginName: epv, Units: , IsExtended: false
        /// </summary>
        public ushort Epv { get; set; }
        /// <summary>
        /// GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
        /// OriginName: vel, Units: cm/s, IsExtended: false
        /// </summary>
        public ushort Vel { get; set; }
        /// <summary>
        /// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        /// OriginName: cog, Units: cdeg, IsExtended: false
        /// </summary>
        public ushort Cog { get; set; }
        /// <summary>
        /// See the GPS_FIX_TYPE enum.
        /// OriginName: fix_type, Units: , IsExtended: false
        /// </summary>
        public GpsFixType FixType { get; set; }
        /// <summary>
        /// Number of satellites visible. If unknown, set to 255
        /// OriginName: satellites_visible, Units: , IsExtended: false
        /// </summary>
        public byte SatellitesVisible { get; set; }
        /// <summary>
        /// Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up).
        /// OriginName: alt_ellipsoid, Units: mm, IsExtended: true
        /// </summary>
        public int AltEllipsoid { get; set; }
        /// <summary>
        /// Position uncertainty in meters * 1000 (positive for up).
        /// OriginName: h_acc, Units: mm, IsExtended: true
        /// </summary>
        public uint HAcc { get; set; }
        /// <summary>
        /// Altitude uncertainty in meters * 1000 (positive for up).
        /// OriginName: v_acc, Units: mm, IsExtended: true
        /// </summary>
        public uint VAcc { get; set; }
        /// <summary>
        /// Speed uncertainty in meters * 1000 (positive for up).
        /// OriginName: vel_acc, Units: mm, IsExtended: true
        /// </summary>
        public uint VelAcc { get; set; }
        /// <summary>
        /// Heading / track uncertainty in degrees * 1e5.
        /// OriginName: hdg_acc, Units: degE5, IsExtended: true
        /// </summary>
        public uint HdgAcc { get; set; }
    }
    /// <summary>
    /// The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
    ///  GPS_STATUS
    /// </summary>
    public class GpsStatusPacket: PacketV2<GpsStatusPayload>
    {
	public const int PacketMessageId = 25;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 23;

        public override GpsStatusPayload Payload { get; } = new GpsStatusPayload();

        public override string Name => "GPS_STATUS";
    }

    /// <summary>
    ///  GPS_STATUS
    /// </summary>
    public class GpsStatusPayload : IPayload
    {
        public byte GetMaxByteSize() => 101;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            SatellitesVisible = (byte)buffer[index++];
            arraySize = /*ArrayLength*/20 - Math.Max(0,((/*PayloadByteSize*/101 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<20;i++)
            {
                SatellitePrn[i] = default(byte);
            }
            for(var i=0;i<arraySize;i++)
            {
                SatellitePrn[i] = (byte)buffer[index++];
            }
            arraySize = 20;
            for(var i=0;i<arraySize;i++)
            {
                SatelliteUsed[i] = (byte)buffer[index++];
            }
            arraySize = 20;
            for(var i=0;i<arraySize;i++)
            {
                SatelliteElevation[i] = (byte)buffer[index++];
            }
            arraySize = 20;
            for(var i=0;i<arraySize;i++)
            {
                SatelliteAzimuth[i] = (byte)buffer[index++];
            }
            arraySize = 20;
            for(var i=0;i<arraySize;i++)
            {
                SatelliteSnr[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(SatellitesVisible).CopyTo(buffer, index);index+=1;
            for(var i=0;i<20;i++)
            {
                buffer[index] = (byte)SatellitePrn[i];index+=1;
            }
            for(var i=0;i<20;i++)
            {
                buffer[index] = (byte)SatelliteUsed[i];index+=1;
            }
            for(var i=0;i<20;i++)
            {
                buffer[index] = (byte)SatelliteElevation[i];index+=1;
            }
            for(var i=0;i<20;i++)
            {
                buffer[index] = (byte)SatelliteAzimuth[i];index+=1;
            }
            for(var i=0;i<20;i++)
            {
                buffer[index] = (byte)SatelliteSnr[i];index+=1;
            }
            return /*PayloadByteSize*/101;
        }

        /// <summary>
        /// Number of satellites visible
        /// OriginName: satellites_visible, Units: , IsExtended: false
        /// </summary>
        public byte SatellitesVisible { get; set; }
        /// <summary>
        /// Global satellite ID
        /// OriginName: satellite_prn, Units: , IsExtended: false
        /// </summary>
        public byte[] SatellitePrn { get; } = new byte[20];
        /// <summary>
        /// 0: Satellite not used, 1: used for localization
        /// OriginName: satellite_used, Units: , IsExtended: false
        /// </summary>
        public byte[] SatelliteUsed { get; } = new byte[20];
        /// <summary>
        /// Elevation (0: right on top of receiver, 90: on the horizon) of satellite
        /// OriginName: satellite_elevation, Units: deg, IsExtended: false
        /// </summary>
        public byte[] SatelliteElevation { get; } = new byte[20];
        /// <summary>
        /// Direction of satellite, 0: 0 deg, 255: 360 deg.
        /// OriginName: satellite_azimuth, Units: deg, IsExtended: false
        /// </summary>
        public byte[] SatelliteAzimuth { get; } = new byte[20];
        /// <summary>
        /// Signal to noise ratio of satellite
        /// OriginName: satellite_snr, Units: dB, IsExtended: false
        /// </summary>
        public byte[] SatelliteSnr { get; } = new byte[20];
    }
    /// <summary>
    /// The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
    ///  SCALED_IMU
    /// </summary>
    public class ScaledImuPacket: PacketV2<ScaledImuPayload>
    {
	public const int PacketMessageId = 26;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 170;

        public override ScaledImuPayload Payload { get; } = new ScaledImuPayload();

        public override string Name => "SCALED_IMU";
    }

    /// <summary>
    ///  SCALED_IMU
    /// </summary>
    public class ScaledImuPayload : IPayload
    {
        public byte GetMaxByteSize() => 22;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Xacc = BitConverter.ToInt16(buffer,index);index+=2;
            Yacc = BitConverter.ToInt16(buffer,index);index+=2;
            Zacc = BitConverter.ToInt16(buffer,index);index+=2;
            Xgyro = BitConverter.ToInt16(buffer,index);index+=2;
            Ygyro = BitConverter.ToInt16(buffer,index);index+=2;
            Zgyro = BitConverter.ToInt16(buffer,index);index+=2;
            Xmag = BitConverter.ToInt16(buffer,index);index+=2;
            Ymag = BitConverter.ToInt16(buffer,index);index+=2;
            Zmag = BitConverter.ToInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Xacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Yacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Zacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Xgyro).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Ygyro).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Zgyro).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Xmag).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Ymag).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Zmag).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/22;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X acceleration (mg)
        /// OriginName: xacc, Units: mG, IsExtended: false
        /// </summary>
        public short Xacc { get; set; }
        /// <summary>
        /// Y acceleration (mg)
        /// OriginName: yacc, Units: mG, IsExtended: false
        /// </summary>
        public short Yacc { get; set; }
        /// <summary>
        /// Z acceleration (mg)
        /// OriginName: zacc, Units: mG, IsExtended: false
        /// </summary>
        public short Zacc { get; set; }
        /// <summary>
        /// Angular speed around X axis (millirad /sec)
        /// OriginName: xgyro, Units: mrad/s, IsExtended: false
        /// </summary>
        public short Xgyro { get; set; }
        /// <summary>
        /// Angular speed around Y axis (millirad /sec)
        /// OriginName: ygyro, Units: mrad/s, IsExtended: false
        /// </summary>
        public short Ygyro { get; set; }
        /// <summary>
        /// Angular speed around Z axis (millirad /sec)
        /// OriginName: zgyro, Units: mrad/s, IsExtended: false
        /// </summary>
        public short Zgyro { get; set; }
        /// <summary>
        /// X Magnetic field (milli tesla)
        /// OriginName: xmag, Units: mT, IsExtended: false
        /// </summary>
        public short Xmag { get; set; }
        /// <summary>
        /// Y Magnetic field (milli tesla)
        /// OriginName: ymag, Units: mT, IsExtended: false
        /// </summary>
        public short Ymag { get; set; }
        /// <summary>
        /// Z Magnetic field (milli tesla)
        /// OriginName: zmag, Units: mT, IsExtended: false
        /// </summary>
        public short Zmag { get; set; }
    }
    /// <summary>
    /// The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw values without any scaling to allow data capture and system debugging.
    ///  RAW_IMU
    /// </summary>
    public class RawImuPacket: PacketV2<RawImuPayload>
    {
	public const int PacketMessageId = 27;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 144;

        public override RawImuPayload Payload { get; } = new RawImuPayload();

        public override string Name => "RAW_IMU";
    }

    /// <summary>
    ///  RAW_IMU
    /// </summary>
    public class RawImuPayload : IPayload
    {
        public byte GetMaxByteSize() => 26;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Xacc = BitConverter.ToInt16(buffer,index);index+=2;
            Yacc = BitConverter.ToInt16(buffer,index);index+=2;
            Zacc = BitConverter.ToInt16(buffer,index);index+=2;
            Xgyro = BitConverter.ToInt16(buffer,index);index+=2;
            Ygyro = BitConverter.ToInt16(buffer,index);index+=2;
            Zgyro = BitConverter.ToInt16(buffer,index);index+=2;
            Xmag = BitConverter.ToInt16(buffer,index);index+=2;
            Ymag = BitConverter.ToInt16(buffer,index);index+=2;
            Zmag = BitConverter.ToInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Xacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Yacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Zacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Xgyro).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Ygyro).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Zgyro).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Xmag).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Ymag).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Zmag).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/26;
        }

        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X acceleration (raw)
        /// OriginName: xacc, Units: , IsExtended: false
        /// </summary>
        public short Xacc { get; set; }
        /// <summary>
        /// Y acceleration (raw)
        /// OriginName: yacc, Units: , IsExtended: false
        /// </summary>
        public short Yacc { get; set; }
        /// <summary>
        /// Z acceleration (raw)
        /// OriginName: zacc, Units: , IsExtended: false
        /// </summary>
        public short Zacc { get; set; }
        /// <summary>
        /// Angular speed around X axis (raw)
        /// OriginName: xgyro, Units: , IsExtended: false
        /// </summary>
        public short Xgyro { get; set; }
        /// <summary>
        /// Angular speed around Y axis (raw)
        /// OriginName: ygyro, Units: , IsExtended: false
        /// </summary>
        public short Ygyro { get; set; }
        /// <summary>
        /// Angular speed around Z axis (raw)
        /// OriginName: zgyro, Units: , IsExtended: false
        /// </summary>
        public short Zgyro { get; set; }
        /// <summary>
        /// X Magnetic field (raw)
        /// OriginName: xmag, Units: , IsExtended: false
        /// </summary>
        public short Xmag { get; set; }
        /// <summary>
        /// Y Magnetic field (raw)
        /// OriginName: ymag, Units: , IsExtended: false
        /// </summary>
        public short Ymag { get; set; }
        /// <summary>
        /// Z Magnetic field (raw)
        /// OriginName: zmag, Units: , IsExtended: false
        /// </summary>
        public short Zmag { get; set; }
    }
    /// <summary>
    /// The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
    ///  RAW_PRESSURE
    /// </summary>
    public class RawPressurePacket: PacketV2<RawPressurePayload>
    {
	public const int PacketMessageId = 28;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 67;

        public override RawPressurePayload Payload { get; } = new RawPressurePayload();

        public override string Name => "RAW_PRESSURE";
    }

    /// <summary>
    ///  RAW_PRESSURE
    /// </summary>
    public class RawPressurePayload : IPayload
    {
        public byte GetMaxByteSize() => 16;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            PressAbs = BitConverter.ToInt16(buffer,index);index+=2;
            PressDiff1 = BitConverter.ToInt16(buffer,index);index+=2;
            PressDiff2 = BitConverter.ToInt16(buffer,index);index+=2;
            Temperature = BitConverter.ToInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(PressAbs).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(PressDiff1).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(PressDiff2).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/16;
        }

        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Absolute pressure (raw)
        /// OriginName: press_abs, Units: , IsExtended: false
        /// </summary>
        public short PressAbs { get; set; }
        /// <summary>
        /// Differential pressure 1 (raw, 0 if nonexistant)
        /// OriginName: press_diff1, Units: , IsExtended: false
        /// </summary>
        public short PressDiff1 { get; set; }
        /// <summary>
        /// Differential pressure 2 (raw, 0 if nonexistant)
        /// OriginName: press_diff2, Units: , IsExtended: false
        /// </summary>
        public short PressDiff2 { get; set; }
        /// <summary>
        /// Raw Temperature measurement (raw)
        /// OriginName: temperature, Units: , IsExtended: false
        /// </summary>
        public short Temperature { get; set; }
    }
    /// <summary>
    /// The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
    ///  SCALED_PRESSURE
    /// </summary>
    public class ScaledPressurePacket: PacketV2<ScaledPressurePayload>
    {
	public const int PacketMessageId = 29;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 115;

        public override ScaledPressurePayload Payload { get; } = new ScaledPressurePayload();

        public override string Name => "SCALED_PRESSURE";
    }

    /// <summary>
    ///  SCALED_PRESSURE
    /// </summary>
    public class ScaledPressurePayload : IPayload
    {
        public byte GetMaxByteSize() => 14;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            PressAbs = BitConverter.ToSingle(buffer, index);index+=4;
            PressDiff = BitConverter.ToSingle(buffer, index);index+=4;
            Temperature = BitConverter.ToInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PressAbs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PressDiff).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/14;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Absolute pressure (hectopascal)
        /// OriginName: press_abs, Units: hPa, IsExtended: false
        /// </summary>
        public float PressAbs { get; set; }
        /// <summary>
        /// Differential pressure 1 (hectopascal)
        /// OriginName: press_diff, Units: hPa, IsExtended: false
        /// </summary>
        public float PressDiff { get; set; }
        /// <summary>
        /// Temperature measurement (0.01 degrees celsius)
        /// OriginName: temperature, Units: cdegC, IsExtended: false
        /// </summary>
        public short Temperature { get; set; }
    }
    /// <summary>
    /// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
    ///  ATTITUDE
    /// </summary>
    public class AttitudePacket: PacketV2<AttitudePayload>
    {
	public const int PacketMessageId = 30;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 39;

        public override AttitudePayload Payload { get; } = new AttitudePayload();

        public override string Name => "ATTITUDE";
    }

    /// <summary>
    ///  ATTITUDE
    /// </summary>
    public class AttitudePayload : IPayload
    {
        public byte GetMaxByteSize() => 28;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            Rollspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Pitchspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Yawspeed = BitConverter.ToSingle(buffer, index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Roll).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitch).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yaw).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Rollspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitchspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yawspeed).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/28;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Roll angle (rad, -pi..+pi)
        /// OriginName: roll, Units: rad, IsExtended: false
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Pitch angle (rad, -pi..+pi)
        /// OriginName: pitch, Units: rad, IsExtended: false
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Yaw angle (rad, -pi..+pi)
        /// OriginName: yaw, Units: rad, IsExtended: false
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// Roll angular speed (rad/s)
        /// OriginName: rollspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Rollspeed { get; set; }
        /// <summary>
        /// Pitch angular speed (rad/s)
        /// OriginName: pitchspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Pitchspeed { get; set; }
        /// <summary>
        /// Yaw angular speed (rad/s)
        /// OriginName: yawspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Yawspeed { get; set; }
    }
    /// <summary>
    /// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
    ///  ATTITUDE_QUATERNION
    /// </summary>
    public class AttitudeQuaternionPacket: PacketV2<AttitudeQuaternionPayload>
    {
	public const int PacketMessageId = 31;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 246;

        public override AttitudeQuaternionPayload Payload { get; } = new AttitudeQuaternionPayload();

        public override string Name => "ATTITUDE_QUATERNION";
    }

    /// <summary>
    ///  ATTITUDE_QUATERNION
    /// </summary>
    public class AttitudeQuaternionPayload : IPayload
    {
        public byte GetMaxByteSize() => 32;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Q1 = BitConverter.ToSingle(buffer, index);index+=4;
            Q2 = BitConverter.ToSingle(buffer, index);index+=4;
            Q3 = BitConverter.ToSingle(buffer, index);index+=4;
            Q4 = BitConverter.ToSingle(buffer, index);index+=4;
            Rollspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Pitchspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Yawspeed = BitConverter.ToSingle(buffer, index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Q1).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Q2).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Q3).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Q4).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Rollspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitchspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yawspeed).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/32;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Quaternion component 1, w (1 in null-rotation)
        /// OriginName: q1, Units: , IsExtended: false
        /// </summary>
        public float Q1 { get; set; }
        /// <summary>
        /// Quaternion component 2, x (0 in null-rotation)
        /// OriginName: q2, Units: , IsExtended: false
        /// </summary>
        public float Q2 { get; set; }
        /// <summary>
        /// Quaternion component 3, y (0 in null-rotation)
        /// OriginName: q3, Units: , IsExtended: false
        /// </summary>
        public float Q3 { get; set; }
        /// <summary>
        /// Quaternion component 4, z (0 in null-rotation)
        /// OriginName: q4, Units: , IsExtended: false
        /// </summary>
        public float Q4 { get; set; }
        /// <summary>
        /// Roll angular speed (rad/s)
        /// OriginName: rollspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Rollspeed { get; set; }
        /// <summary>
        /// Pitch angular speed (rad/s)
        /// OriginName: pitchspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Pitchspeed { get; set; }
        /// <summary>
        /// Yaw angular speed (rad/s)
        /// OriginName: yawspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Yawspeed { get; set; }
    }
    /// <summary>
    /// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
    ///  LOCAL_POSITION_NED
    /// </summary>
    public class LocalPositionNedPacket: PacketV2<LocalPositionNedPayload>
    {
	public const int PacketMessageId = 32;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 185;

        public override LocalPositionNedPayload Payload { get; } = new LocalPositionNedPayload();

        public override string Name => "LOCAL_POSITION_NED";
    }

    /// <summary>
    ///  LOCAL_POSITION_NED
    /// </summary>
    public class LocalPositionNedPayload : IPayload
    {
        public byte GetMaxByteSize() => 28;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Vx = BitConverter.ToSingle(buffer, index);index+=4;
            Vy = BitConverter.ToSingle(buffer, index);index+=4;
            Vz = BitConverter.ToSingle(buffer, index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vx).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vz).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/28;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X Position
        /// OriginName: x, Units: m, IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y Position
        /// OriginName: y, Units: m, IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z Position
        /// OriginName: z, Units: m, IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// X Speed
        /// OriginName: vx, Units: m/s, IsExtended: false
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Y Speed
        /// OriginName: vy, Units: m/s, IsExtended: false
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Z Speed
        /// OriginName: vz, Units: m/s, IsExtended: false
        /// </summary>
        public float Vz { get; set; }
    }
    /// <summary>
    /// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
    ///                is designed as scaled integer message since the resolution of float is not sufficient.
    ///  GLOBAL_POSITION_INT
    /// </summary>
    public class GlobalPositionIntPacket: PacketV2<GlobalPositionIntPayload>
    {
	public const int PacketMessageId = 33;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 104;

        public override GlobalPositionIntPayload Payload { get; } = new GlobalPositionIntPayload();

        public override string Name => "GLOBAL_POSITION_INT";
    }

    /// <summary>
    ///  GLOBAL_POSITION_INT
    /// </summary>
    public class GlobalPositionIntPayload : IPayload
    {
        public byte GetMaxByteSize() => 28;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToInt32(buffer,index);index+=4;
            RelativeAlt = BitConverter.ToInt32(buffer,index);index+=4;
            Vx = BitConverter.ToInt16(buffer,index);index+=2;
            Vy = BitConverter.ToInt16(buffer,index);index+=2;
            Vz = BitConverter.ToInt16(buffer,index);index+=2;
            Hdg = BitConverter.ToUInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(RelativeAlt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vx).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Vy).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Vz).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Hdg).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/28;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Latitude, expressed as degrees * 1E7
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude, expressed as degrees * 1E7
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
        /// OriginName: alt, Units: mm, IsExtended: false
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// Altitude above ground in meters, expressed as * 1000 (millimeters)
        /// OriginName: relative_alt, Units: mm, IsExtended: false
        /// </summary>
        public int RelativeAlt { get; set; }
        /// <summary>
        /// Ground X Speed (Latitude, positive north), expressed as m/s * 100
        /// OriginName: vx, Units: cm/s, IsExtended: false
        /// </summary>
        public short Vx { get; set; }
        /// <summary>
        /// Ground Y Speed (Longitude, positive east), expressed as m/s * 100
        /// OriginName: vy, Units: cm/s, IsExtended: false
        /// </summary>
        public short Vy { get; set; }
        /// <summary>
        /// Ground Z Speed (Altitude, positive down), expressed as m/s * 100
        /// OriginName: vz, Units: cm/s, IsExtended: false
        /// </summary>
        public short Vz { get; set; }
        /// <summary>
        /// Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        /// OriginName: hdg, Units: cdeg, IsExtended: false
        /// </summary>
        public ushort Hdg { get; set; }
    }
    /// <summary>
    /// The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX.
    ///  RC_CHANNELS_SCALED
    /// </summary>
    public class RcChannelsScaledPacket: PacketV2<RcChannelsScaledPayload>
    {
	public const int PacketMessageId = 34;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 237;

        public override RcChannelsScaledPayload Payload { get; } = new RcChannelsScaledPayload();

        public override string Name => "RC_CHANNELS_SCALED";
    }

    /// <summary>
    ///  RC_CHANNELS_SCALED
    /// </summary>
    public class RcChannelsScaledPayload : IPayload
    {
        public byte GetMaxByteSize() => 22;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Chan1Scaled = BitConverter.ToInt16(buffer,index);index+=2;
            Chan2Scaled = BitConverter.ToInt16(buffer,index);index+=2;
            Chan3Scaled = BitConverter.ToInt16(buffer,index);index+=2;
            Chan4Scaled = BitConverter.ToInt16(buffer,index);index+=2;
            Chan5Scaled = BitConverter.ToInt16(buffer,index);index+=2;
            Chan6Scaled = BitConverter.ToInt16(buffer,index);index+=2;
            Chan7Scaled = BitConverter.ToInt16(buffer,index);index+=2;
            Chan8Scaled = BitConverter.ToInt16(buffer,index);index+=2;
            Port = (byte)buffer[index++];
            Rssi = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Chan1Scaled).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan2Scaled).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan3Scaled).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan4Scaled).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan5Scaled).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan6Scaled).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan7Scaled).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan8Scaled).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Port).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Rssi).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/22;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// OriginName: chan1_scaled, Units: , IsExtended: false
        /// </summary>
        public short Chan1Scaled { get; set; }
        /// <summary>
        /// RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// OriginName: chan2_scaled, Units: , IsExtended: false
        /// </summary>
        public short Chan2Scaled { get; set; }
        /// <summary>
        /// RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// OriginName: chan3_scaled, Units: , IsExtended: false
        /// </summary>
        public short Chan3Scaled { get; set; }
        /// <summary>
        /// RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// OriginName: chan4_scaled, Units: , IsExtended: false
        /// </summary>
        public short Chan4Scaled { get; set; }
        /// <summary>
        /// RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// OriginName: chan5_scaled, Units: , IsExtended: false
        /// </summary>
        public short Chan5Scaled { get; set; }
        /// <summary>
        /// RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// OriginName: chan6_scaled, Units: , IsExtended: false
        /// </summary>
        public short Chan6Scaled { get; set; }
        /// <summary>
        /// RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// OriginName: chan7_scaled, Units: , IsExtended: false
        /// </summary>
        public short Chan7Scaled { get; set; }
        /// <summary>
        /// RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// OriginName: chan8_scaled, Units: , IsExtended: false
        /// </summary>
        public short Chan8Scaled { get; set; }
        /// <summary>
        /// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
        /// OriginName: port, Units: , IsExtended: false
        /// </summary>
        public byte Port { get; set; }
        /// <summary>
        /// Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
        /// OriginName: rssi, Units: %, IsExtended: false
        /// </summary>
        public byte Rssi { get; set; }
    }
    /// <summary>
    /// The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
    ///  RC_CHANNELS_RAW
    /// </summary>
    public class RcChannelsRawPacket: PacketV2<RcChannelsRawPayload>
    {
	public const int PacketMessageId = 35;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 244;

        public override RcChannelsRawPayload Payload { get; } = new RcChannelsRawPayload();

        public override string Name => "RC_CHANNELS_RAW";
    }

    /// <summary>
    ///  RC_CHANNELS_RAW
    /// </summary>
    public class RcChannelsRawPayload : IPayload
    {
        public byte GetMaxByteSize() => 22;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Chan1Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan2Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan3Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan4Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan5Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan6Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan7Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan8Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Port = (byte)buffer[index++];
            Rssi = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Chan1Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan2Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan3Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan4Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan5Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan6Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan7Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan8Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Port).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Rssi).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/22;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan1_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan1Raw { get; set; }
        /// <summary>
        /// RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan2_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan2Raw { get; set; }
        /// <summary>
        /// RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan3_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan3Raw { get; set; }
        /// <summary>
        /// RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan4_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan4Raw { get; set; }
        /// <summary>
        /// RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan5_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan5Raw { get; set; }
        /// <summary>
        /// RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan6_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan6Raw { get; set; }
        /// <summary>
        /// RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan7_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan7Raw { get; set; }
        /// <summary>
        /// RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan8_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan8Raw { get; set; }
        /// <summary>
        /// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
        /// OriginName: port, Units: , IsExtended: false
        /// </summary>
        public byte Port { get; set; }
        /// <summary>
        /// Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
        /// OriginName: rssi, Units: %, IsExtended: false
        /// </summary>
        public byte Rssi { get; set; }
    }
    /// <summary>
    /// The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
    ///  SERVO_OUTPUT_RAW
    /// </summary>
    public class ServoOutputRawPacket: PacketV2<ServoOutputRawPayload>
    {
	public const int PacketMessageId = 36;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 222;

        public override ServoOutputRawPayload Payload { get; } = new ServoOutputRawPayload();

        public override string Name => "SERVO_OUTPUT_RAW";
    }

    /// <summary>
    ///  SERVO_OUTPUT_RAW
    /// </summary>
    public class ServoOutputRawPayload : IPayload
    {
        public byte GetMaxByteSize() => 37;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt32(buffer,index);index+=4;
            Servo1Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Servo2Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Servo3Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Servo4Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Servo5Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Servo6Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Servo7Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Servo8Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Port = (byte)buffer[index++];
            // extended field 'Servo9Raw' can be empty
            if (index >= endIndex) return;
            Servo9Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Servo10Raw' can be empty
            if (index >= endIndex) return;
            Servo10Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Servo11Raw' can be empty
            if (index >= endIndex) return;
            Servo11Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Servo12Raw' can be empty
            if (index >= endIndex) return;
            Servo12Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Servo13Raw' can be empty
            if (index >= endIndex) return;
            Servo13Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Servo14Raw' can be empty
            if (index >= endIndex) return;
            Servo14Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Servo15Raw' can be empty
            if (index >= endIndex) return;
            Servo15Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Servo16Raw' can be empty
            if (index >= endIndex) return;
            Servo16Raw = BitConverter.ToUInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Servo1Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Servo2Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Servo3Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Servo4Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Servo5Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Servo6Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Servo7Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Servo8Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Port).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Servo9Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Servo10Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Servo11Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Servo12Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Servo13Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Servo14Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Servo15Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Servo16Raw).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/37;
        }

        /// <summary>
        /// Timestamp (microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public uint TimeUsec { get; set; }
        /// <summary>
        /// Servo output 1 value, in microseconds
        /// OriginName: servo1_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Servo1Raw { get; set; }
        /// <summary>
        /// Servo output 2 value, in microseconds
        /// OriginName: servo2_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Servo2Raw { get; set; }
        /// <summary>
        /// Servo output 3 value, in microseconds
        /// OriginName: servo3_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Servo3Raw { get; set; }
        /// <summary>
        /// Servo output 4 value, in microseconds
        /// OriginName: servo4_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Servo4Raw { get; set; }
        /// <summary>
        /// Servo output 5 value, in microseconds
        /// OriginName: servo5_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Servo5Raw { get; set; }
        /// <summary>
        /// Servo output 6 value, in microseconds
        /// OriginName: servo6_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Servo6Raw { get; set; }
        /// <summary>
        /// Servo output 7 value, in microseconds
        /// OriginName: servo7_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Servo7Raw { get; set; }
        /// <summary>
        /// Servo output 8 value, in microseconds
        /// OriginName: servo8_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Servo8Raw { get; set; }
        /// <summary>
        /// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
        /// OriginName: port, Units: , IsExtended: false
        /// </summary>
        public byte Port { get; set; }
        /// <summary>
        /// Servo output 9 value, in microseconds
        /// OriginName: servo9_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Servo9Raw { get; set; }
        /// <summary>
        /// Servo output 10 value, in microseconds
        /// OriginName: servo10_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Servo10Raw { get; set; }
        /// <summary>
        /// Servo output 11 value, in microseconds
        /// OriginName: servo11_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Servo11Raw { get; set; }
        /// <summary>
        /// Servo output 12 value, in microseconds
        /// OriginName: servo12_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Servo12Raw { get; set; }
        /// <summary>
        /// Servo output 13 value, in microseconds
        /// OriginName: servo13_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Servo13Raw { get; set; }
        /// <summary>
        /// Servo output 14 value, in microseconds
        /// OriginName: servo14_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Servo14Raw { get; set; }
        /// <summary>
        /// Servo output 15 value, in microseconds
        /// OriginName: servo15_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Servo15Raw { get; set; }
        /// <summary>
        /// Servo output 16 value, in microseconds
        /// OriginName: servo16_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Servo16Raw { get; set; }
    }
    /// <summary>
    /// Request a partial list of mission items from the system/component. https://mavlink.io/en/protocol/mission.html. If start and end index are the same, just send one waypoint.
    ///  MISSION_REQUEST_PARTIAL_LIST
    /// </summary>
    public class MissionRequestPartialListPacket: PacketV2<MissionRequestPartialListPayload>
    {
	public const int PacketMessageId = 37;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 212;

        public override MissionRequestPartialListPayload Payload { get; } = new MissionRequestPartialListPayload();

        public override string Name => "MISSION_REQUEST_PARTIAL_LIST";
    }

    /// <summary>
    ///  MISSION_REQUEST_PARTIAL_LIST
    /// </summary>
    public class MissionRequestPartialListPayload : IPayload
    {
        public byte GetMaxByteSize() => 7;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            StartIndex = BitConverter.ToInt16(buffer,index);index+=2;
            EndIndex = BitConverter.ToInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            // extended field 'MissionType' can be empty
            if (index >= endIndex) return;
            MissionType = (MavMissionType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(StartIndex).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(EndIndex).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)MissionType;index+=1;
            return /*PayloadByteSize*/7;
        }

        /// <summary>
        /// Start index, 0 by default
        /// OriginName: start_index, Units: , IsExtended: false
        /// </summary>
        public short StartIndex { get; set; }
        /// <summary>
        /// End index, -1 by default (-1: send list to end). Else a valid index of the list
        /// OriginName: end_index, Units: , IsExtended: false
        /// </summary>
        public short EndIndex { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission type, see MAV_MISSION_TYPE
        /// OriginName: mission_type, Units: , IsExtended: true
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
    ///  MISSION_WRITE_PARTIAL_LIST
    /// </summary>
    public class MissionWritePartialListPacket: PacketV2<MissionWritePartialListPayload>
    {
	public const int PacketMessageId = 38;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 9;

        public override MissionWritePartialListPayload Payload { get; } = new MissionWritePartialListPayload();

        public override string Name => "MISSION_WRITE_PARTIAL_LIST";
    }

    /// <summary>
    ///  MISSION_WRITE_PARTIAL_LIST
    /// </summary>
    public class MissionWritePartialListPayload : IPayload
    {
        public byte GetMaxByteSize() => 7;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            StartIndex = BitConverter.ToInt16(buffer,index);index+=2;
            EndIndex = BitConverter.ToInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            // extended field 'MissionType' can be empty
            if (index >= endIndex) return;
            MissionType = (MavMissionType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(StartIndex).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(EndIndex).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)MissionType;index+=1;
            return /*PayloadByteSize*/7;
        }

        /// <summary>
        /// Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
        /// OriginName: start_index, Units: , IsExtended: false
        /// </summary>
        public short StartIndex { get; set; }
        /// <summary>
        /// End index, equal or greater than start index.
        /// OriginName: end_index, Units: , IsExtended: false
        /// </summary>
        public short EndIndex { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission type, see MAV_MISSION_TYPE
        /// OriginName: mission_type, Units: , IsExtended: true
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// Message encoding a mission item. This message is emitted to announce
    ///                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also https://mavlink.io/en/protocol/mission.html.
    ///  MISSION_ITEM
    /// </summary>
    public class MissionItemPacket: PacketV2<MissionItemPayload>
    {
	public const int PacketMessageId = 39;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 254;

        public override MissionItemPayload Payload { get; } = new MissionItemPayload();

        public override string Name => "MISSION_ITEM";
    }

    /// <summary>
    ///  MISSION_ITEM
    /// </summary>
    public class MissionItemPayload : IPayload
    {
        public byte GetMaxByteSize() => 38;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Param1 = BitConverter.ToSingle(buffer, index);index+=4;
            Param2 = BitConverter.ToSingle(buffer, index);index+=4;
            Param3 = BitConverter.ToSingle(buffer, index);index+=4;
            Param4 = BitConverter.ToSingle(buffer, index);index+=4;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Seq = BitConverter.ToUInt16(buffer,index);index+=2;
            Command = (MavCmd)BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            Frame = (MavFrame)buffer[index++];
            Current = (byte)buffer[index++];
            Autocontinue = (byte)buffer[index++];
            // extended field 'MissionType' can be empty
            if (index >= endIndex) return;
            MissionType = (MavMissionType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Param1).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param2).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param3).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param4).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes((ushort)Command).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)Frame;index+=1;
            BitConverter.GetBytes(Current).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Autocontinue).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)MissionType;index+=1;
            return /*PayloadByteSize*/38;
        }

        /// <summary>
        /// PARAM1, see MAV_CMD enum
        /// OriginName: param1, Units: , IsExtended: false
        /// </summary>
        public float Param1 { get; set; }
        /// <summary>
        /// PARAM2, see MAV_CMD enum
        /// OriginName: param2, Units: , IsExtended: false
        /// </summary>
        public float Param2 { get; set; }
        /// <summary>
        /// PARAM3, see MAV_CMD enum
        /// OriginName: param3, Units: , IsExtended: false
        /// </summary>
        public float Param3 { get; set; }
        /// <summary>
        /// PARAM4, see MAV_CMD enum
        /// OriginName: param4, Units: , IsExtended: false
        /// </summary>
        public float Param4 { get; set; }
        /// <summary>
        /// PARAM5 / local: X coordinate, global: latitude
        /// OriginName: x, Units: , IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// PARAM6 / local: Y coordinate, global: longitude
        /// OriginName: y, Units: , IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
        /// OriginName: z, Units: , IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Sequence
        /// OriginName: seq, Units: , IsExtended: false
        /// </summary>
        public ushort Seq { get; set; }
        /// <summary>
        /// The scheduled action for the waypoint, as defined by MAV_CMD enum
        /// OriginName: command, Units: , IsExtended: false
        /// </summary>
        public MavCmd Command { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// The coordinate system of the waypoint, as defined by MAV_FRAME enum
        /// OriginName: frame, Units: , IsExtended: false
        /// </summary>
        public MavFrame Frame { get; set; }
        /// <summary>
        /// false:0, true:1
        /// OriginName: current, Units: , IsExtended: false
        /// </summary>
        public byte Current { get; set; }
        /// <summary>
        /// autocontinue to next wp
        /// OriginName: autocontinue, Units: , IsExtended: false
        /// </summary>
        public byte Autocontinue { get; set; }
        /// <summary>
        /// Mission type, see MAV_MISSION_TYPE
        /// OriginName: mission_type, Units: , IsExtended: true
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. https://mavlink.io/en/protocol/mission.html
    ///  MISSION_REQUEST
    /// </summary>
    public class MissionRequestPacket: PacketV2<MissionRequestPayload>
    {
	public const int PacketMessageId = 40;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 230;

        public override MissionRequestPayload Payload { get; } = new MissionRequestPayload();

        public override string Name => "MISSION_REQUEST";
    }

    /// <summary>
    ///  MISSION_REQUEST
    /// </summary>
    public class MissionRequestPayload : IPayload
    {
        public byte GetMaxByteSize() => 5;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Seq = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            // extended field 'MissionType' can be empty
            if (index >= endIndex) return;
            MissionType = (MavMissionType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)MissionType;index+=1;
            return /*PayloadByteSize*/5;
        }

        /// <summary>
        /// Sequence
        /// OriginName: seq, Units: , IsExtended: false
        /// </summary>
        public ushort Seq { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission type, see MAV_MISSION_TYPE
        /// OriginName: mission_type, Units: , IsExtended: true
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
    ///  MISSION_SET_CURRENT
    /// </summary>
    public class MissionSetCurrentPacket: PacketV2<MissionSetCurrentPayload>
    {
	public const int PacketMessageId = 41;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 28;

        public override MissionSetCurrentPayload Payload { get; } = new MissionSetCurrentPayload();

        public override string Name => "MISSION_SET_CURRENT";
    }

    /// <summary>
    ///  MISSION_SET_CURRENT
    /// </summary>
    public class MissionSetCurrentPayload : IPayload
    {
        public byte GetMaxByteSize() => 4;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Seq = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/4;
        }

        /// <summary>
        /// Sequence
        /// OriginName: seq, Units: , IsExtended: false
        /// </summary>
        public ushort Seq { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
    ///  MISSION_CURRENT
    /// </summary>
    public class MissionCurrentPacket: PacketV2<MissionCurrentPayload>
    {
	public const int PacketMessageId = 42;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 28;

        public override MissionCurrentPayload Payload { get; } = new MissionCurrentPayload();

        public override string Name => "MISSION_CURRENT";
    }

    /// <summary>
    ///  MISSION_CURRENT
    /// </summary>
    public class MissionCurrentPayload : IPayload
    {
        public byte GetMaxByteSize() => 2;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Seq = BitConverter.ToUInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/2;
        }

        /// <summary>
        /// Sequence
        /// OriginName: seq, Units: , IsExtended: false
        /// </summary>
        public ushort Seq { get; set; }
    }
    /// <summary>
    /// Request the overall list of mission items from the system/component.
    ///  MISSION_REQUEST_LIST
    /// </summary>
    public class MissionRequestListPacket: PacketV2<MissionRequestListPayload>
    {
	public const int PacketMessageId = 43;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 132;

        public override MissionRequestListPayload Payload { get; } = new MissionRequestListPayload();

        public override string Name => "MISSION_REQUEST_LIST";
    }

    /// <summary>
    ///  MISSION_REQUEST_LIST
    /// </summary>
    public class MissionRequestListPayload : IPayload
    {
        public byte GetMaxByteSize() => 3;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            // extended field 'MissionType' can be empty
            if (index >= endIndex) return;
            MissionType = (MavMissionType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)MissionType;index+=1;
            return /*PayloadByteSize*/3;
        }

        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission type, see MAV_MISSION_TYPE
        /// OriginName: mission_type, Units: , IsExtended: true
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of waypoints.
    ///  MISSION_COUNT
    /// </summary>
    public class MissionCountPacket: PacketV2<MissionCountPayload>
    {
	public const int PacketMessageId = 44;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 221;

        public override MissionCountPayload Payload { get; } = new MissionCountPayload();

        public override string Name => "MISSION_COUNT";
    }

    /// <summary>
    ///  MISSION_COUNT
    /// </summary>
    public class MissionCountPayload : IPayload
    {
        public byte GetMaxByteSize() => 5;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Count = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            // extended field 'MissionType' can be empty
            if (index >= endIndex) return;
            MissionType = (MavMissionType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Count).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)MissionType;index+=1;
            return /*PayloadByteSize*/5;
        }

        /// <summary>
        /// Number of mission items in the sequence
        /// OriginName: count, Units: , IsExtended: false
        /// </summary>
        public ushort Count { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission type, see MAV_MISSION_TYPE
        /// OriginName: mission_type, Units: , IsExtended: true
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// Delete all mission items at once.
    ///  MISSION_CLEAR_ALL
    /// </summary>
    public class MissionClearAllPacket: PacketV2<MissionClearAllPayload>
    {
	public const int PacketMessageId = 45;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 232;

        public override MissionClearAllPayload Payload { get; } = new MissionClearAllPayload();

        public override string Name => "MISSION_CLEAR_ALL";
    }

    /// <summary>
    ///  MISSION_CLEAR_ALL
    /// </summary>
    public class MissionClearAllPayload : IPayload
    {
        public byte GetMaxByteSize() => 3;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            // extended field 'MissionType' can be empty
            if (index >= endIndex) return;
            MissionType = (MavMissionType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)MissionType;index+=1;
            return /*PayloadByteSize*/3;
        }

        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission type, see MAV_MISSION_TYPE
        /// OriginName: mission_type, Units: , IsExtended: true
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next waypoint.
    ///  MISSION_ITEM_REACHED
    /// </summary>
    public class MissionItemReachedPacket: PacketV2<MissionItemReachedPayload>
    {
	public const int PacketMessageId = 46;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 11;

        public override MissionItemReachedPayload Payload { get; } = new MissionItemReachedPayload();

        public override string Name => "MISSION_ITEM_REACHED";
    }

    /// <summary>
    ///  MISSION_ITEM_REACHED
    /// </summary>
    public class MissionItemReachedPayload : IPayload
    {
        public byte GetMaxByteSize() => 2;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Seq = BitConverter.ToUInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/2;
        }

        /// <summary>
        /// Sequence
        /// OriginName: seq, Units: , IsExtended: false
        /// </summary>
        public ushort Seq { get; set; }
    }
    /// <summary>
    /// Ack message during waypoint handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
    ///  MISSION_ACK
    /// </summary>
    public class MissionAckPacket: PacketV2<MissionAckPayload>
    {
	public const int PacketMessageId = 47;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 153;

        public override MissionAckPayload Payload { get; } = new MissionAckPayload();

        public override string Name => "MISSION_ACK";
    }

    /// <summary>
    ///  MISSION_ACK
    /// </summary>
    public class MissionAckPayload : IPayload
    {
        public byte GetMaxByteSize() => 4;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            Type = (MavMissionResult)buffer[index++];
            // extended field 'MissionType' can be empty
            if (index >= endIndex) return;
            MissionType = (MavMissionType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)Type;index+=1;
            buffer[index] = (byte)MissionType;index+=1;
            return /*PayloadByteSize*/4;
        }

        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// See MAV_MISSION_RESULT enum
        /// OriginName: type, Units: , IsExtended: false
        /// </summary>
        public MavMissionResult Type { get; set; }
        /// <summary>
        /// Mission type, see MAV_MISSION_TYPE
        /// OriginName: mission_type, Units: , IsExtended: true
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// As local waypoints exist, the global waypoint reference allows to transform between the local coordinate frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings are connected and the MAV should move from in- to outdoor.
    ///  SET_GPS_GLOBAL_ORIGIN
    /// </summary>
    public class SetGpsGlobalOriginPacket: PacketV2<SetGpsGlobalOriginPayload>
    {
	public const int PacketMessageId = 48;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 41;

        public override SetGpsGlobalOriginPayload Payload { get; } = new SetGpsGlobalOriginPayload();

        public override string Name => "SET_GPS_GLOBAL_ORIGIN";
    }

    /// <summary>
    ///  SET_GPS_GLOBAL_ORIGIN
    /// </summary>
    public class SetGpsGlobalOriginPayload : IPayload
    {
        public byte GetMaxByteSize() => 21;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Latitude = BitConverter.ToInt32(buffer,index);index+=4;
            Longitude = BitConverter.ToInt32(buffer,index);index+=4;
            Altitude = BitConverter.ToInt32(buffer,index);index+=4;
            TargetSystem = (byte)buffer[index++];
            // extended field 'TimeUsec' can be empty
            if (index >= endIndex) return;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Latitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Longitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Altitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            return /*PayloadByteSize*/21;
        }

        /// <summary>
        /// Latitude (WGS84), in degrees * 1E7
        /// OriginName: latitude, Units: degE7, IsExtended: false
        /// </summary>
        public int Latitude { get; set; }
        /// <summary>
        /// Longitude (WGS84), in degrees * 1E7
        /// OriginName: longitude, Units: degE7, IsExtended: false
        /// </summary>
        public int Longitude { get; set; }
        /// <summary>
        /// Altitude (AMSL), in meters * 1000 (positive for up)
        /// OriginName: altitude, Units: mm, IsExtended: false
        /// </summary>
        public int Altitude { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: true
        /// </summary>
        public ulong TimeUsec { get; set; }
    }
    /// <summary>
    /// Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position
    ///  GPS_GLOBAL_ORIGIN
    /// </summary>
    public class GpsGlobalOriginPacket: PacketV2<GpsGlobalOriginPayload>
    {
	public const int PacketMessageId = 49;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 39;

        public override GpsGlobalOriginPayload Payload { get; } = new GpsGlobalOriginPayload();

        public override string Name => "GPS_GLOBAL_ORIGIN";
    }

    /// <summary>
    ///  GPS_GLOBAL_ORIGIN
    /// </summary>
    public class GpsGlobalOriginPayload : IPayload
    {
        public byte GetMaxByteSize() => 20;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Latitude = BitConverter.ToInt32(buffer,index);index+=4;
            Longitude = BitConverter.ToInt32(buffer,index);index+=4;
            Altitude = BitConverter.ToInt32(buffer,index);index+=4;
            // extended field 'TimeUsec' can be empty
            if (index >= endIndex) return;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Latitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Longitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Altitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            return /*PayloadByteSize*/20;
        }

        /// <summary>
        /// Latitude (WGS84), in degrees * 1E7
        /// OriginName: latitude, Units: degE7, IsExtended: false
        /// </summary>
        public int Latitude { get; set; }
        /// <summary>
        /// Longitude (WGS84), in degrees * 1E7
        /// OriginName: longitude, Units: degE7, IsExtended: false
        /// </summary>
        public int Longitude { get; set; }
        /// <summary>
        /// Altitude (AMSL), in meters * 1000 (positive for up)
        /// OriginName: altitude, Units: mm, IsExtended: false
        /// </summary>
        public int Altitude { get; set; }
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: true
        /// </summary>
        public ulong TimeUsec { get; set; }
    }
    /// <summary>
    /// Bind a RC channel to a parameter. The parameter should change accoding to the RC channel value.
    ///  PARAM_MAP_RC
    /// </summary>
    public class ParamMapRcPacket: PacketV2<ParamMapRcPayload>
    {
	public const int PacketMessageId = 50;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 78;

        public override ParamMapRcPayload Payload { get; } = new ParamMapRcPayload();

        public override string Name => "PARAM_MAP_RC";
    }

    /// <summary>
    ///  PARAM_MAP_RC
    /// </summary>
    public class ParamMapRcPayload : IPayload
    {
        public byte GetMaxByteSize() => 37;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            ParamValue0 = BitConverter.ToSingle(buffer, index);index+=4;
            Scale = BitConverter.ToSingle(buffer, index);index+=4;
            ParamValueMin = BitConverter.ToSingle(buffer, index);index+=4;
            ParamValueMax = BitConverter.ToSingle(buffer, index);index+=4;
            ParamIndex = BitConverter.ToInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            arraySize = /*ArrayLength*/16 - Math.Max(0,((/*PayloadByteSize*/37 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<16;i++)
            {
                ParamId[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,ParamId,0);
                index+=16;
            ParameterRcChannelIndex = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ParamValue0).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Scale).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ParamValueMin).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ParamValueMax).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ParamIndex).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);index+=16;
            BitConverter.GetBytes(ParameterRcChannelIndex).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/37;
        }

        /// <summary>
        /// Initial parameter value
        /// OriginName: param_value0, Units: , IsExtended: false
        /// </summary>
        public float ParamValue0 { get; set; }
        /// <summary>
        /// Scale, maps the RC range [-1, 1] to a parameter value
        /// OriginName: scale, Units: , IsExtended: false
        /// </summary>
        public float Scale { get; set; }
        /// <summary>
        /// Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends on implementation)
        /// OriginName: param_value_min, Units: , IsExtended: false
        /// </summary>
        public float ParamValueMin { get; set; }
        /// <summary>
        /// Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends on implementation)
        /// OriginName: param_value_max, Units: , IsExtended: false
        /// </summary>
        public float ParamValueMax { get; set; }
        /// <summary>
        /// Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored), send -2 to disable any existing map for this rc_channel_index.
        /// OriginName: param_index, Units: , IsExtended: false
        /// </summary>
        public short ParamIndex { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// OriginName: param_id, Units: , IsExtended: false
        /// </summary>
        public char[] ParamId { get; } = new char[16];
        /// <summary>
        /// Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob on the RC.
        /// OriginName: parameter_rc_channel_index, Units: , IsExtended: false
        /// </summary>
        public byte ParameterRcChannelIndex { get; set; }
    }
    /// <summary>
    /// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM_INT message. https://mavlink.io/en/protocol/mission.html
    ///  MISSION_REQUEST_INT
    /// </summary>
    public class MissionRequestIntPacket: PacketV2<MissionRequestIntPayload>
    {
	public const int PacketMessageId = 51;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 196;

        public override MissionRequestIntPayload Payload { get; } = new MissionRequestIntPayload();

        public override string Name => "MISSION_REQUEST_INT";
    }

    /// <summary>
    ///  MISSION_REQUEST_INT
    /// </summary>
    public class MissionRequestIntPayload : IPayload
    {
        public byte GetMaxByteSize() => 5;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Seq = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            // extended field 'MissionType' can be empty
            if (index >= endIndex) return;
            MissionType = (MavMissionType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)MissionType;index+=1;
            return /*PayloadByteSize*/5;
        }

        /// <summary>
        /// Sequence
        /// OriginName: seq, Units: , IsExtended: false
        /// </summary>
        public ushort Seq { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission type, see MAV_MISSION_TYPE
        /// OriginName: mission_type, Units: , IsExtended: true
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national or competition regulations.
    ///  SAFETY_SET_ALLOWED_AREA
    /// </summary>
    public class SafetySetAllowedAreaPacket: PacketV2<SafetySetAllowedAreaPayload>
    {
	public const int PacketMessageId = 54;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 15;

        public override SafetySetAllowedAreaPayload Payload { get; } = new SafetySetAllowedAreaPayload();

        public override string Name => "SAFETY_SET_ALLOWED_AREA";
    }

    /// <summary>
    ///  SAFETY_SET_ALLOWED_AREA
    /// </summary>
    public class SafetySetAllowedAreaPayload : IPayload
    {
        public byte GetMaxByteSize() => 27;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            P1x = BitConverter.ToSingle(buffer, index);index+=4;
            P1y = BitConverter.ToSingle(buffer, index);index+=4;
            P1z = BitConverter.ToSingle(buffer, index);index+=4;
            P2x = BitConverter.ToSingle(buffer, index);index+=4;
            P2y = BitConverter.ToSingle(buffer, index);index+=4;
            P2z = BitConverter.ToSingle(buffer, index);index+=4;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            Frame = (MavFrame)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(P1x).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P1y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P1z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P2x).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P2y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P2z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)Frame;index+=1;
            return /*PayloadByteSize*/27;
        }

        /// <summary>
        /// x position 1 / Latitude 1
        /// OriginName: p1x, Units: m, IsExtended: false
        /// </summary>
        public float P1x { get; set; }
        /// <summary>
        /// y position 1 / Longitude 1
        /// OriginName: p1y, Units: m, IsExtended: false
        /// </summary>
        public float P1y { get; set; }
        /// <summary>
        /// z position 1 / Altitude 1
        /// OriginName: p1z, Units: m, IsExtended: false
        /// </summary>
        public float P1z { get; set; }
        /// <summary>
        /// x position 2 / Latitude 2
        /// OriginName: p2x, Units: m, IsExtended: false
        /// </summary>
        public float P2x { get; set; }
        /// <summary>
        /// y position 2 / Longitude 2
        /// OriginName: p2y, Units: m, IsExtended: false
        /// </summary>
        public float P2y { get; set; }
        /// <summary>
        /// z position 2 / Altitude 2
        /// OriginName: p2z, Units: m, IsExtended: false
        /// </summary>
        public float P2z { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Coordinate frame, as defined by MAV_FRAME enum. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
        /// OriginName: frame, Units: , IsExtended: false
        /// </summary>
        public MavFrame Frame { get; set; }
    }
    /// <summary>
    /// Read out the safety zone the MAV currently assumes.
    ///  SAFETY_ALLOWED_AREA
    /// </summary>
    public class SafetyAllowedAreaPacket: PacketV2<SafetyAllowedAreaPayload>
    {
	public const int PacketMessageId = 55;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 3;

        public override SafetyAllowedAreaPayload Payload { get; } = new SafetyAllowedAreaPayload();

        public override string Name => "SAFETY_ALLOWED_AREA";
    }

    /// <summary>
    ///  SAFETY_ALLOWED_AREA
    /// </summary>
    public class SafetyAllowedAreaPayload : IPayload
    {
        public byte GetMaxByteSize() => 25;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            P1x = BitConverter.ToSingle(buffer, index);index+=4;
            P1y = BitConverter.ToSingle(buffer, index);index+=4;
            P1z = BitConverter.ToSingle(buffer, index);index+=4;
            P2x = BitConverter.ToSingle(buffer, index);index+=4;
            P2y = BitConverter.ToSingle(buffer, index);index+=4;
            P2z = BitConverter.ToSingle(buffer, index);index+=4;
            Frame = (MavFrame)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(P1x).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P1y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P1z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P2x).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P2y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P2z).CopyTo(buffer, index);index+=4;
            buffer[index] = (byte)Frame;index+=1;
            return /*PayloadByteSize*/25;
        }

        /// <summary>
        /// x position 1 / Latitude 1
        /// OriginName: p1x, Units: m, IsExtended: false
        /// </summary>
        public float P1x { get; set; }
        /// <summary>
        /// y position 1 / Longitude 1
        /// OriginName: p1y, Units: m, IsExtended: false
        /// </summary>
        public float P1y { get; set; }
        /// <summary>
        /// z position 1 / Altitude 1
        /// OriginName: p1z, Units: m, IsExtended: false
        /// </summary>
        public float P1z { get; set; }
        /// <summary>
        /// x position 2 / Latitude 2
        /// OriginName: p2x, Units: m, IsExtended: false
        /// </summary>
        public float P2x { get; set; }
        /// <summary>
        /// y position 2 / Longitude 2
        /// OriginName: p2y, Units: m, IsExtended: false
        /// </summary>
        public float P2y { get; set; }
        /// <summary>
        /// z position 2 / Altitude 2
        /// OriginName: p2z, Units: m, IsExtended: false
        /// </summary>
        public float P2z { get; set; }
        /// <summary>
        /// Coordinate frame, as defined by MAV_FRAME enum. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
        /// OriginName: frame, Units: , IsExtended: false
        /// </summary>
        public MavFrame Frame { get; set; }
    }
    /// <summary>
    /// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
    ///  ATTITUDE_QUATERNION_COV
    /// </summary>
    public class AttitudeQuaternionCovPacket: PacketV2<AttitudeQuaternionCovPayload>
    {
	public const int PacketMessageId = 61;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 167;

        public override AttitudeQuaternionCovPayload Payload { get; } = new AttitudeQuaternionCovPayload();

        public override string Name => "ATTITUDE_QUATERNION_COV";
    }

    /// <summary>
    ///  ATTITUDE_QUATERNION_COV
    /// </summary>
    public class AttitudeQuaternionCovPayload : IPayload
    {
        public byte GetMaxByteSize() => 72;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            arraySize = 4;
            for(var i=0;i<arraySize;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            Rollspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Pitchspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Yawspeed = BitConverter.ToSingle(buffer, index);index+=4;
            arraySize = /*ArrayLength*/9 - Math.Max(0,((/*PayloadByteSize*/72 - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<9;i++)
            {
                Covariance[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(Q[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(Rollspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitchspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yawspeed).CopyTo(buffer, index);index+=4;
            for(var i=0;i<9;i++)
            {
                BitConverter.GetBytes(Covariance[i]).CopyTo(buffer, index);index+=4;
            }
            return /*PayloadByteSize*/72;
        }

        /// <summary>
        /// Timestamp (microseconds since system boot or since UNIX epoch)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
        /// OriginName: q, Units: , IsExtended: false
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Roll angular speed (rad/s)
        /// OriginName: rollspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Rollspeed { get; set; }
        /// <summary>
        /// Pitch angular speed (rad/s)
        /// OriginName: pitchspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Pitchspeed { get; set; }
        /// <summary>
        /// Yaw angular speed (rad/s)
        /// OriginName: yawspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Yawspeed { get; set; }
        /// <summary>
        /// Attitude covariance
        /// OriginName: covariance, Units: , IsExtended: false
        /// </summary>
        public float[] Covariance { get; } = new float[9];
    }
    /// <summary>
    /// The state of the fixed wing navigation and position controller.
    ///  NAV_CONTROLLER_OUTPUT
    /// </summary>
    public class NavControllerOutputPacket: PacketV2<NavControllerOutputPayload>
    {
	public const int PacketMessageId = 62;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 183;

        public override NavControllerOutputPayload Payload { get; } = new NavControllerOutputPayload();

        public override string Name => "NAV_CONTROLLER_OUTPUT";
    }

    /// <summary>
    ///  NAV_CONTROLLER_OUTPUT
    /// </summary>
    public class NavControllerOutputPayload : IPayload
    {
        public byte GetMaxByteSize() => 26;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            NavRoll = BitConverter.ToSingle(buffer, index);index+=4;
            NavPitch = BitConverter.ToSingle(buffer, index);index+=4;
            AltError = BitConverter.ToSingle(buffer, index);index+=4;
            AspdError = BitConverter.ToSingle(buffer, index);index+=4;
            XtrackError = BitConverter.ToSingle(buffer, index);index+=4;
            NavBearing = BitConverter.ToInt16(buffer,index);index+=2;
            TargetBearing = BitConverter.ToInt16(buffer,index);index+=2;
            WpDist = BitConverter.ToUInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(NavRoll).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(NavPitch).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(AltError).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(AspdError).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(XtrackError).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(NavBearing).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetBearing).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(WpDist).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/26;
        }

        /// <summary>
        /// Current desired roll in degrees
        /// OriginName: nav_roll, Units: deg, IsExtended: false
        /// </summary>
        public float NavRoll { get; set; }
        /// <summary>
        /// Current desired pitch in degrees
        /// OriginName: nav_pitch, Units: deg, IsExtended: false
        /// </summary>
        public float NavPitch { get; set; }
        /// <summary>
        /// Current altitude error in meters
        /// OriginName: alt_error, Units: m, IsExtended: false
        /// </summary>
        public float AltError { get; set; }
        /// <summary>
        /// Current airspeed error in meters/second
        /// OriginName: aspd_error, Units: m/s, IsExtended: false
        /// </summary>
        public float AspdError { get; set; }
        /// <summary>
        /// Current crosstrack error on x-y plane in meters
        /// OriginName: xtrack_error, Units: m, IsExtended: false
        /// </summary>
        public float XtrackError { get; set; }
        /// <summary>
        /// Current desired heading in degrees
        /// OriginName: nav_bearing, Units: deg, IsExtended: false
        /// </summary>
        public short NavBearing { get; set; }
        /// <summary>
        /// Bearing to current waypoint/target in degrees
        /// OriginName: target_bearing, Units: deg, IsExtended: false
        /// </summary>
        public short TargetBearing { get; set; }
        /// <summary>
        /// Distance to active waypoint in meters
        /// OriginName: wp_dist, Units: m, IsExtended: false
        /// </summary>
        public ushort WpDist { get; set; }
    }
    /// <summary>
    /// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.
    ///  GLOBAL_POSITION_INT_COV
    /// </summary>
    public class GlobalPositionIntCovPacket: PacketV2<GlobalPositionIntCovPayload>
    {
	public const int PacketMessageId = 63;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 119;

        public override GlobalPositionIntCovPayload Payload { get; } = new GlobalPositionIntCovPayload();

        public override string Name => "GLOBAL_POSITION_INT_COV";
    }

    /// <summary>
    ///  GLOBAL_POSITION_INT_COV
    /// </summary>
    public class GlobalPositionIntCovPayload : IPayload
    {
        public byte GetMaxByteSize() => 181;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToInt32(buffer,index);index+=4;
            RelativeAlt = BitConverter.ToInt32(buffer,index);index+=4;
            Vx = BitConverter.ToSingle(buffer, index);index+=4;
            Vy = BitConverter.ToSingle(buffer, index);index+=4;
            Vz = BitConverter.ToSingle(buffer, index);index+=4;
            arraySize = /*ArrayLength*/36 - Math.Max(0,((/*PayloadByteSize*/181 - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<36;i++)
            {
                Covariance[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            EstimatorType = (MavEstimatorType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(RelativeAlt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vx).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vz).CopyTo(buffer, index);index+=4;
            for(var i=0;i<36;i++)
            {
                BitConverter.GetBytes(Covariance[i]).CopyTo(buffer, index);index+=4;
            }
            buffer[index] = (byte)EstimatorType;index+=1;
            return /*PayloadByteSize*/181;
        }

        /// <summary>
        /// Timestamp (microseconds since system boot or since UNIX epoch)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Latitude, expressed as degrees * 1E7
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude, expressed as degrees * 1E7
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude in meters, expressed as * 1000 (millimeters), above MSL
        /// OriginName: alt, Units: mm, IsExtended: false
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// Altitude above ground in meters, expressed as * 1000 (millimeters)
        /// OriginName: relative_alt, Units: mm, IsExtended: false
        /// </summary>
        public int RelativeAlt { get; set; }
        /// <summary>
        /// Ground X Speed (Latitude), expressed as m/s
        /// OriginName: vx, Units: m/s, IsExtended: false
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Ground Y Speed (Longitude), expressed as m/s
        /// OriginName: vy, Units: m/s, IsExtended: false
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Ground Z Speed (Altitude), expressed as m/s
        /// OriginName: vz, Units: m/s, IsExtended: false
        /// </summary>
        public float Vz { get; set; }
        /// <summary>
        /// Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
        /// OriginName: covariance, Units: , IsExtended: false
        /// </summary>
        public float[] Covariance { get; } = new float[36];
        /// <summary>
        /// Class id of the estimator this estimate originated from.
        /// OriginName: estimator_type, Units: , IsExtended: false
        /// </summary>
        public MavEstimatorType EstimatorType { get; set; }
    }
    /// <summary>
    /// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
    ///  LOCAL_POSITION_NED_COV
    /// </summary>
    public class LocalPositionNedCovPacket: PacketV2<LocalPositionNedCovPayload>
    {
	public const int PacketMessageId = 64;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 191;

        public override LocalPositionNedCovPayload Payload { get; } = new LocalPositionNedCovPayload();

        public override string Name => "LOCAL_POSITION_NED_COV";
    }

    /// <summary>
    ///  LOCAL_POSITION_NED_COV
    /// </summary>
    public class LocalPositionNedCovPayload : IPayload
    {
        public byte GetMaxByteSize() => 225;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Vx = BitConverter.ToSingle(buffer, index);index+=4;
            Vy = BitConverter.ToSingle(buffer, index);index+=4;
            Vz = BitConverter.ToSingle(buffer, index);index+=4;
            Ax = BitConverter.ToSingle(buffer, index);index+=4;
            Ay = BitConverter.ToSingle(buffer, index);index+=4;
            Az = BitConverter.ToSingle(buffer, index);index+=4;
            arraySize = /*ArrayLength*/45 - Math.Max(0,((/*PayloadByteSize*/225 - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<45;i++)
            {
                Covariance[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            EstimatorType = (MavEstimatorType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vx).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vz).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Ax).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Ay).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Az).CopyTo(buffer, index);index+=4;
            for(var i=0;i<45;i++)
            {
                BitConverter.GetBytes(Covariance[i]).CopyTo(buffer, index);index+=4;
            }
            buffer[index] = (byte)EstimatorType;index+=1;
            return /*PayloadByteSize*/225;
        }

        /// <summary>
        /// Timestamp (microseconds since system boot or since UNIX epoch)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X Position
        /// OriginName: x, Units: m, IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y Position
        /// OriginName: y, Units: m, IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z Position
        /// OriginName: z, Units: m, IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// X Speed (m/s)
        /// OriginName: vx, Units: m/s, IsExtended: false
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Y Speed (m/s)
        /// OriginName: vy, Units: m/s, IsExtended: false
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Z Speed (m/s)
        /// OriginName: vz, Units: m/s, IsExtended: false
        /// </summary>
        public float Vz { get; set; }
        /// <summary>
        /// X Acceleration (m/s^2)
        /// OriginName: ax, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Ax { get; set; }
        /// <summary>
        /// Y Acceleration (m/s^2)
        /// OriginName: ay, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Ay { get; set; }
        /// <summary>
        /// Z Acceleration (m/s^2)
        /// OriginName: az, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Az { get; set; }
        /// <summary>
        /// Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are the second row, etc.)
        /// OriginName: covariance, Units: , IsExtended: false
        /// </summary>
        public float[] Covariance { get; } = new float[45];
        /// <summary>
        /// Class id of the estimator this estimate originated from.
        /// OriginName: estimator_type, Units: , IsExtended: false
        /// </summary>
        public MavEstimatorType EstimatorType { get; set; }
    }
    /// <summary>
    /// The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
    ///  RC_CHANNELS
    /// </summary>
    public class RcChannelsPacket: PacketV2<RcChannelsPayload>
    {
	public const int PacketMessageId = 65;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 118;

        public override RcChannelsPayload Payload { get; } = new RcChannelsPayload();

        public override string Name => "RC_CHANNELS";
    }

    /// <summary>
    ///  RC_CHANNELS
    /// </summary>
    public class RcChannelsPayload : IPayload
    {
        public byte GetMaxByteSize() => 42;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Chan1Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan2Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan3Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan4Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan5Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan6Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan7Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan8Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan9Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan10Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan11Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan12Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan13Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan14Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan15Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan16Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan17Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan18Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chancount = (byte)buffer[index++];
            Rssi = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Chan1Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan2Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan3Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan4Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan5Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan6Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan7Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan8Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan9Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan10Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan11Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan12Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan13Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan14Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan15Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan16Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan17Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan18Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chancount).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Rssi).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/42;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan1_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan1Raw { get; set; }
        /// <summary>
        /// RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan2_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan2Raw { get; set; }
        /// <summary>
        /// RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan3_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan3Raw { get; set; }
        /// <summary>
        /// RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan4_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan4Raw { get; set; }
        /// <summary>
        /// RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan5_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan5Raw { get; set; }
        /// <summary>
        /// RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan6_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan6Raw { get; set; }
        /// <summary>
        /// RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan7_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan7Raw { get; set; }
        /// <summary>
        /// RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan8_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan8Raw { get; set; }
        /// <summary>
        /// RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan9_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan9Raw { get; set; }
        /// <summary>
        /// RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan10_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan10Raw { get; set; }
        /// <summary>
        /// RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan11_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan11Raw { get; set; }
        /// <summary>
        /// RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan12_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan12Raw { get; set; }
        /// <summary>
        /// RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan13_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan13Raw { get; set; }
        /// <summary>
        /// RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan14_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan14Raw { get; set; }
        /// <summary>
        /// RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan15_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan15Raw { get; set; }
        /// <summary>
        /// RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan16_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan16Raw { get; set; }
        /// <summary>
        /// RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan17_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan17Raw { get; set; }
        /// <summary>
        /// RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// OriginName: chan18_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan18Raw { get; set; }
        /// <summary>
        /// Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
        /// OriginName: chancount, Units: , IsExtended: false
        /// </summary>
        public byte Chancount { get; set; }
        /// <summary>
        /// Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
        /// OriginName: rssi, Units: %, IsExtended: false
        /// </summary>
        public byte Rssi { get; set; }
    }
    /// <summary>
    /// THIS INTERFACE IS DEPRECATED. USE SET_MESSAGE_INTERVAL INSTEAD.
    ///  REQUEST_DATA_STREAM
    /// </summary>
    public class RequestDataStreamPacket: PacketV2<RequestDataStreamPayload>
    {
	public const int PacketMessageId = 66;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 148;

        public override RequestDataStreamPayload Payload { get; } = new RequestDataStreamPayload();

        public override string Name => "REQUEST_DATA_STREAM";
    }

    /// <summary>
    ///  REQUEST_DATA_STREAM
    /// </summary>
    public class RequestDataStreamPayload : IPayload
    {
        public byte GetMaxByteSize() => 6;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            ReqMessageRate = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            ReqStreamId = (byte)buffer[index++];
            StartStop = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ReqMessageRate).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(ReqStreamId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(StartStop).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/6;
        }

        /// <summary>
        /// The requested message rate
        /// OriginName: req_message_rate, Units: Hz, IsExtended: false
        /// </summary>
        public ushort ReqMessageRate { get; set; }
        /// <summary>
        /// The target requested to send the message stream.
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// The target requested to send the message stream.
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// The ID of the requested data stream
        /// OriginName: req_stream_id, Units: , IsExtended: false
        /// </summary>
        public byte ReqStreamId { get; set; }
        /// <summary>
        /// 1 to start sending, 0 to stop sending.
        /// OriginName: start_stop, Units: , IsExtended: false
        /// </summary>
        public byte StartStop { get; set; }
    }
    /// <summary>
    /// THIS INTERFACE IS DEPRECATED. USE MESSAGE_INTERVAL INSTEAD.
    ///  DATA_STREAM
    /// </summary>
    public class DataStreamPacket: PacketV2<DataStreamPayload>
    {
	public const int PacketMessageId = 67;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 21;

        public override DataStreamPayload Payload { get; } = new DataStreamPayload();

        public override string Name => "DATA_STREAM";
    }

    /// <summary>
    ///  DATA_STREAM
    /// </summary>
    public class DataStreamPayload : IPayload
    {
        public byte GetMaxByteSize() => 4;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            MessageRate = BitConverter.ToUInt16(buffer,index);index+=2;
            StreamId = (byte)buffer[index++];
            OnOff = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(MessageRate).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(StreamId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(OnOff).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/4;
        }

        /// <summary>
        /// The message rate
        /// OriginName: message_rate, Units: Hz, IsExtended: false
        /// </summary>
        public ushort MessageRate { get; set; }
        /// <summary>
        /// The ID of the requested data stream
        /// OriginName: stream_id, Units: , IsExtended: false
        /// </summary>
        public byte StreamId { get; set; }
        /// <summary>
        /// 1 stream is enabled, 0 stream is stopped.
        /// OriginName: on_off, Units: , IsExtended: false
        /// </summary>
        public byte OnOff { get; set; }
    }
    /// <summary>
    /// This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their 
    ///  MANUAL_CONTROL
    /// </summary>
    public class ManualControlPacket: PacketV2<ManualControlPayload>
    {
	public const int PacketMessageId = 69;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 243;

        public override ManualControlPayload Payload { get; } = new ManualControlPayload();

        public override string Name => "MANUAL_CONTROL";
    }

    /// <summary>
    ///  MANUAL_CONTROL
    /// </summary>
    public class ManualControlPayload : IPayload
    {
        public byte GetMaxByteSize() => 11;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            X = BitConverter.ToInt16(buffer,index);index+=2;
            Y = BitConverter.ToInt16(buffer,index);index+=2;
            Z = BitConverter.ToInt16(buffer,index);index+=2;
            R = BitConverter.ToInt16(buffer,index);index+=2;
            Buttons = BitConverter.ToUInt16(buffer,index);index+=2;
            Target = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(R).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Buttons).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Target).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/11;
        }

        /// <summary>
        /// X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
        /// OriginName: x, Units: , IsExtended: false
        /// </summary>
        public short X { get; set; }
        /// <summary>
        /// Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
        /// OriginName: y, Units: , IsExtended: false
        /// </summary>
        public short Y { get; set; }
        /// <summary>
        /// Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.
        /// OriginName: z, Units: , IsExtended: false
        /// </summary>
        public short Z { get; set; }
        /// <summary>
        /// R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
        /// OriginName: r, Units: , IsExtended: false
        /// </summary>
        public short R { get; set; }
        /// <summary>
        /// A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
        /// OriginName: buttons, Units: , IsExtended: false
        /// </summary>
        public ushort Buttons { get; set; }
        /// <summary>
        /// The system to be controlled.
        /// OriginName: target, Units: , IsExtended: false
        /// </summary>
        public byte Target { get; set; }
    }
    /// <summary>
    /// The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
    ///  RC_CHANNELS_OVERRIDE
    /// </summary>
    public class RcChannelsOverridePacket: PacketV2<RcChannelsOverridePayload>
    {
	public const int PacketMessageId = 70;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 124;

        public override RcChannelsOverridePayload Payload { get; } = new RcChannelsOverridePayload();

        public override string Name => "RC_CHANNELS_OVERRIDE";
    }

    /// <summary>
    ///  RC_CHANNELS_OVERRIDE
    /// </summary>
    public class RcChannelsOverridePayload : IPayload
    {
        public byte GetMaxByteSize() => 38;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Chan1Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan2Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan3Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan4Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan5Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan6Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan7Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan8Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            // extended field 'Chan9Raw' can be empty
            if (index >= endIndex) return;
            Chan9Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Chan10Raw' can be empty
            if (index >= endIndex) return;
            Chan10Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Chan11Raw' can be empty
            if (index >= endIndex) return;
            Chan11Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Chan12Raw' can be empty
            if (index >= endIndex) return;
            Chan12Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Chan13Raw' can be empty
            if (index >= endIndex) return;
            Chan13Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Chan14Raw' can be empty
            if (index >= endIndex) return;
            Chan14Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Chan15Raw' can be empty
            if (index >= endIndex) return;
            Chan15Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Chan16Raw' can be empty
            if (index >= endIndex) return;
            Chan16Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Chan17Raw' can be empty
            if (index >= endIndex) return;
            Chan17Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            // extended field 'Chan18Raw' can be empty
            if (index >= endIndex) return;
            Chan18Raw = BitConverter.ToUInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Chan1Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan2Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan3Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan4Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan5Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan6Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan7Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan8Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Chan9Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan10Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan11Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan12Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan13Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan14Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan15Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan16Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan17Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan18Raw).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/38;
        }

        /// <summary>
        /// RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// OriginName: chan1_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan1Raw { get; set; }
        /// <summary>
        /// RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// OriginName: chan2_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan2Raw { get; set; }
        /// <summary>
        /// RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// OriginName: chan3_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan3Raw { get; set; }
        /// <summary>
        /// RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// OriginName: chan4_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan4Raw { get; set; }
        /// <summary>
        /// RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// OriginName: chan5_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan5Raw { get; set; }
        /// <summary>
        /// RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// OriginName: chan6_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan6Raw { get; set; }
        /// <summary>
        /// RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// OriginName: chan7_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan7Raw { get; set; }
        /// <summary>
        /// RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// OriginName: chan8_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan8Raw { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// RC channel 9 value, in microseconds. A value of 0 means to ignore this field.
        /// OriginName: chan9_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Chan9Raw { get; set; }
        /// <summary>
        /// RC channel 10 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field.
        /// OriginName: chan10_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Chan10Raw { get; set; }
        /// <summary>
        /// RC channel 11 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field.
        /// OriginName: chan11_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Chan11Raw { get; set; }
        /// <summary>
        /// RC channel 12 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field.
        /// OriginName: chan12_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Chan12Raw { get; set; }
        /// <summary>
        /// RC channel 13 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field.
        /// OriginName: chan13_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Chan13Raw { get; set; }
        /// <summary>
        /// RC channel 14 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field.
        /// OriginName: chan14_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Chan14Raw { get; set; }
        /// <summary>
        /// RC channel 15 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field.
        /// OriginName: chan15_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Chan15Raw { get; set; }
        /// <summary>
        /// RC channel 16 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field.
        /// OriginName: chan16_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Chan16Raw { get; set; }
        /// <summary>
        /// RC channel 17 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field.
        /// OriginName: chan17_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Chan17Raw { get; set; }
        /// <summary>
        /// RC channel 18 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field.
        /// OriginName: chan18_raw, Units: us, IsExtended: true
        /// </summary>
        public ushort Chan18Raw { get; set; }
    }
    /// <summary>
    /// Message encoding a mission item. This message is emitted to announce
    ///                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also https://mavlink.io/en/protocol/mission.html.
    ///  MISSION_ITEM_INT
    /// </summary>
    public class MissionItemIntPacket: PacketV2<MissionItemIntPayload>
    {
	public const int PacketMessageId = 73;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 38;

        public override MissionItemIntPayload Payload { get; } = new MissionItemIntPayload();

        public override string Name => "MISSION_ITEM_INT";
    }

    /// <summary>
    ///  MISSION_ITEM_INT
    /// </summary>
    public class MissionItemIntPayload : IPayload
    {
        public byte GetMaxByteSize() => 38;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Param1 = BitConverter.ToSingle(buffer, index);index+=4;
            Param2 = BitConverter.ToSingle(buffer, index);index+=4;
            Param3 = BitConverter.ToSingle(buffer, index);index+=4;
            Param4 = BitConverter.ToSingle(buffer, index);index+=4;
            X = BitConverter.ToInt32(buffer,index);index+=4;
            Y = BitConverter.ToInt32(buffer,index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Seq = BitConverter.ToUInt16(buffer,index);index+=2;
            Command = (MavCmd)BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            Frame = (MavFrame)buffer[index++];
            Current = (byte)buffer[index++];
            Autocontinue = (byte)buffer[index++];
            // extended field 'MissionType' can be empty
            if (index >= endIndex) return;
            MissionType = (MavMissionType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Param1).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param2).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param3).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param4).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes((ushort)Command).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)Frame;index+=1;
            BitConverter.GetBytes(Current).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Autocontinue).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)MissionType;index+=1;
            return /*PayloadByteSize*/38;
        }

        /// <summary>
        /// PARAM1, see MAV_CMD enum
        /// OriginName: param1, Units: , IsExtended: false
        /// </summary>
        public float Param1 { get; set; }
        /// <summary>
        /// PARAM2, see MAV_CMD enum
        /// OriginName: param2, Units: , IsExtended: false
        /// </summary>
        public float Param2 { get; set; }
        /// <summary>
        /// PARAM3, see MAV_CMD enum
        /// OriginName: param3, Units: , IsExtended: false
        /// </summary>
        public float Param3 { get; set; }
        /// <summary>
        /// PARAM4, see MAV_CMD enum
        /// OriginName: param4, Units: , IsExtended: false
        /// </summary>
        public float Param4 { get; set; }
        /// <summary>
        /// PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
        /// OriginName: x, Units: , IsExtended: false
        /// </summary>
        public int X { get; set; }
        /// <summary>
        /// PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
        /// OriginName: y, Units: , IsExtended: false
        /// </summary>
        public int Y { get; set; }
        /// <summary>
        /// PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
        /// OriginName: z, Units: , IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
        /// OriginName: seq, Units: , IsExtended: false
        /// </summary>
        public ushort Seq { get; set; }
        /// <summary>
        /// The scheduled action for the waypoint, as defined by MAV_CMD enum
        /// OriginName: command, Units: , IsExtended: false
        /// </summary>
        public MavCmd Command { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// The coordinate system of the waypoint, as defined by MAV_FRAME enum
        /// OriginName: frame, Units: , IsExtended: false
        /// </summary>
        public MavFrame Frame { get; set; }
        /// <summary>
        /// false:0, true:1
        /// OriginName: current, Units: , IsExtended: false
        /// </summary>
        public byte Current { get; set; }
        /// <summary>
        /// autocontinue to next wp
        /// OriginName: autocontinue, Units: , IsExtended: false
        /// </summary>
        public byte Autocontinue { get; set; }
        /// <summary>
        /// Mission type, see MAV_MISSION_TYPE
        /// OriginName: mission_type, Units: , IsExtended: true
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// Metrics typically displayed on a HUD for fixed wing aircraft
    ///  VFR_HUD
    /// </summary>
    public class VfrHudPacket: PacketV2<VfrHudPayload>
    {
	public const int PacketMessageId = 74;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 20;

        public override VfrHudPayload Payload { get; } = new VfrHudPayload();

        public override string Name => "VFR_HUD";
    }

    /// <summary>
    ///  VFR_HUD
    /// </summary>
    public class VfrHudPayload : IPayload
    {
        public byte GetMaxByteSize() => 20;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Airspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Groundspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Alt = BitConverter.ToSingle(buffer, index);index+=4;
            Climb = BitConverter.ToSingle(buffer, index);index+=4;
            Heading = BitConverter.ToInt16(buffer,index);index+=2;
            Throttle = BitConverter.ToUInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Airspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Groundspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Climb).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Heading).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Throttle).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/20;
        }

        /// <summary>
        /// Current airspeed in m/s
        /// OriginName: airspeed, Units: m/s, IsExtended: false
        /// </summary>
        public float Airspeed { get; set; }
        /// <summary>
        /// Current ground speed in m/s
        /// OriginName: groundspeed, Units: m/s, IsExtended: false
        /// </summary>
        public float Groundspeed { get; set; }
        /// <summary>
        /// Current altitude (MSL), in meters
        /// OriginName: alt, Units: m, IsExtended: false
        /// </summary>
        public float Alt { get; set; }
        /// <summary>
        /// Current climb rate in meters/second
        /// OriginName: climb, Units: m/s, IsExtended: false
        /// </summary>
        public float Climb { get; set; }
        /// <summary>
        /// Current heading in degrees, in compass units (0..360, 0=north)
        /// OriginName: heading, Units: deg, IsExtended: false
        /// </summary>
        public short Heading { get; set; }
        /// <summary>
        /// Current throttle setting in integer percent, 0 to 100
        /// OriginName: throttle, Units: %, IsExtended: false
        /// </summary>
        public ushort Throttle { get; set; }
    }
    /// <summary>
    /// Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value.
    ///  COMMAND_INT
    /// </summary>
    public class CommandIntPacket: PacketV2<CommandIntPayload>
    {
	public const int PacketMessageId = 75;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 158;

        public override CommandIntPayload Payload { get; } = new CommandIntPayload();

        public override string Name => "COMMAND_INT";
    }

    /// <summary>
    ///  COMMAND_INT
    /// </summary>
    public class CommandIntPayload : IPayload
    {
        public byte GetMaxByteSize() => 35;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Param1 = BitConverter.ToSingle(buffer, index);index+=4;
            Param2 = BitConverter.ToSingle(buffer, index);index+=4;
            Param3 = BitConverter.ToSingle(buffer, index);index+=4;
            Param4 = BitConverter.ToSingle(buffer, index);index+=4;
            X = BitConverter.ToInt32(buffer,index);index+=4;
            Y = BitConverter.ToInt32(buffer,index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Command = (MavCmd)BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            Frame = (MavFrame)buffer[index++];
            Current = (byte)buffer[index++];
            Autocontinue = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Param1).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param2).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param3).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param4).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes((ushort)Command).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)Frame;index+=1;
            BitConverter.GetBytes(Current).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Autocontinue).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/35;
        }

        /// <summary>
        /// PARAM1, see MAV_CMD enum
        /// OriginName: param1, Units: , IsExtended: false
        /// </summary>
        public float Param1 { get; set; }
        /// <summary>
        /// PARAM2, see MAV_CMD enum
        /// OriginName: param2, Units: , IsExtended: false
        /// </summary>
        public float Param2 { get; set; }
        /// <summary>
        /// PARAM3, see MAV_CMD enum
        /// OriginName: param3, Units: , IsExtended: false
        /// </summary>
        public float Param3 { get; set; }
        /// <summary>
        /// PARAM4, see MAV_CMD enum
        /// OriginName: param4, Units: , IsExtended: false
        /// </summary>
        public float Param4 { get; set; }
        /// <summary>
        /// PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
        /// OriginName: x, Units: , IsExtended: false
        /// </summary>
        public int X { get; set; }
        /// <summary>
        /// PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
        /// OriginName: y, Units: , IsExtended: false
        /// </summary>
        public int Y { get; set; }
        /// <summary>
        /// PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
        /// OriginName: z, Units: , IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// The scheduled action for the mission item, as defined by MAV_CMD enum
        /// OriginName: command, Units: , IsExtended: false
        /// </summary>
        public MavCmd Command { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// The coordinate system of the COMMAND, as defined by MAV_FRAME enum
        /// OriginName: frame, Units: , IsExtended: false
        /// </summary>
        public MavFrame Frame { get; set; }
        /// <summary>
        /// false:0, true:1
        /// OriginName: current, Units: , IsExtended: false
        /// </summary>
        public byte Current { get; set; }
        /// <summary>
        /// autocontinue to next wp
        /// OriginName: autocontinue, Units: , IsExtended: false
        /// </summary>
        public byte Autocontinue { get; set; }
    }
    /// <summary>
    /// Send a command with up to seven parameters to the MAV
    ///  COMMAND_LONG
    /// </summary>
    public class CommandLongPacket: PacketV2<CommandLongPayload>
    {
	public const int PacketMessageId = 76;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 152;

        public override CommandLongPayload Payload { get; } = new CommandLongPayload();

        public override string Name => "COMMAND_LONG";
    }

    /// <summary>
    ///  COMMAND_LONG
    /// </summary>
    public class CommandLongPayload : IPayload
    {
        public byte GetMaxByteSize() => 33;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Param1 = BitConverter.ToSingle(buffer, index);index+=4;
            Param2 = BitConverter.ToSingle(buffer, index);index+=4;
            Param3 = BitConverter.ToSingle(buffer, index);index+=4;
            Param4 = BitConverter.ToSingle(buffer, index);index+=4;
            Param5 = BitConverter.ToSingle(buffer, index);index+=4;
            Param6 = BitConverter.ToSingle(buffer, index);index+=4;
            Param7 = BitConverter.ToSingle(buffer, index);index+=4;
            Command = (MavCmd)BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            Confirmation = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Param1).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param2).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param3).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param4).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param5).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param6).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Param7).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes((ushort)Command).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Confirmation).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/33;
        }

        /// <summary>
        /// Parameter 1, as defined by MAV_CMD enum.
        /// OriginName: param1, Units: , IsExtended: false
        /// </summary>
        public float Param1 { get; set; }
        /// <summary>
        /// Parameter 2, as defined by MAV_CMD enum.
        /// OriginName: param2, Units: , IsExtended: false
        /// </summary>
        public float Param2 { get; set; }
        /// <summary>
        /// Parameter 3, as defined by MAV_CMD enum.
        /// OriginName: param3, Units: , IsExtended: false
        /// </summary>
        public float Param3 { get; set; }
        /// <summary>
        /// Parameter 4, as defined by MAV_CMD enum.
        /// OriginName: param4, Units: , IsExtended: false
        /// </summary>
        public float Param4 { get; set; }
        /// <summary>
        /// Parameter 5, as defined by MAV_CMD enum.
        /// OriginName: param5, Units: , IsExtended: false
        /// </summary>
        public float Param5 { get; set; }
        /// <summary>
        /// Parameter 6, as defined by MAV_CMD enum.
        /// OriginName: param6, Units: , IsExtended: false
        /// </summary>
        public float Param6 { get; set; }
        /// <summary>
        /// Parameter 7, as defined by MAV_CMD enum.
        /// OriginName: param7, Units: , IsExtended: false
        /// </summary>
        public float Param7 { get; set; }
        /// <summary>
        /// Command ID, as defined by MAV_CMD enum.
        /// OriginName: command, Units: , IsExtended: false
        /// </summary>
        public MavCmd Command { get; set; }
        /// <summary>
        /// System which should execute the command
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component which should execute the command, 0 for all components
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
        /// OriginName: confirmation, Units: , IsExtended: false
        /// </summary>
        public byte Confirmation { get; set; }
    }
    /// <summary>
    /// Report status of a command. Includes feedback whether the command was executed.
    ///  COMMAND_ACK
    /// </summary>
    public class CommandAckPacket: PacketV2<CommandAckPayload>
    {
	public const int PacketMessageId = 77;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 143;

        public override CommandAckPayload Payload { get; } = new CommandAckPayload();

        public override string Name => "COMMAND_ACK";
    }

    /// <summary>
    ///  COMMAND_ACK
    /// </summary>
    public class CommandAckPayload : IPayload
    {
        public byte GetMaxByteSize() => 10;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Command = (MavCmd)BitConverter.ToUInt16(buffer,index);index+=2;
            Result = (MavResult)buffer[index++];
            // extended field 'Progress' can be empty
            if (index >= endIndex) return;
            Progress = (byte)buffer[index++];
            // extended field 'ResultParam2' can be empty
            if (index >= endIndex) return;
            ResultParam2 = BitConverter.ToInt32(buffer,index);index+=4;
            // extended field 'TargetSystem' can be empty
            if (index >= endIndex) return;
            TargetSystem = (byte)buffer[index++];
            // extended field 'TargetComponent' can be empty
            if (index >= endIndex) return;
            TargetComponent = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes((ushort)Command).CopyTo(buffer, index);index+=2;
            buffer[index] = (byte)Result;index+=1;
            BitConverter.GetBytes(Progress).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(ResultParam2).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/10;
        }

        /// <summary>
        /// Command ID, as defined by MAV_CMD enum.
        /// OriginName: command, Units: , IsExtended: false
        /// </summary>
        public MavCmd Command { get; set; }
        /// <summary>
        /// See MAV_RESULT enum
        /// OriginName: result, Units: , IsExtended: false
        /// </summary>
        public MavResult Result { get; set; }
        /// <summary>
        /// WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS.
        /// OriginName: progress, Units: , IsExtended: true
        /// </summary>
        public byte Progress { get; set; }
        /// <summary>
        /// WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied.
        /// OriginName: result_param2, Units: , IsExtended: true
        /// </summary>
        public int ResultParam2 { get; set; }
        /// <summary>
        /// WIP: System which requested the command to be executed
        /// OriginName: target_system, Units: , IsExtended: true
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// WIP: Component which requested the command to be executed
        /// OriginName: target_component, Units: , IsExtended: true
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Setpoint in roll, pitch, yaw and thrust from the operator
    ///  MANUAL_SETPOINT
    /// </summary>
    public class ManualSetpointPacket: PacketV2<ManualSetpointPayload>
    {
	public const int PacketMessageId = 81;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 106;

        public override ManualSetpointPayload Payload { get; } = new ManualSetpointPayload();

        public override string Name => "MANUAL_SETPOINT";
    }

    /// <summary>
    ///  MANUAL_SETPOINT
    /// </summary>
    public class ManualSetpointPayload : IPayload
    {
        public byte GetMaxByteSize() => 22;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            Thrust = BitConverter.ToSingle(buffer, index);index+=4;
            ModeSwitch = (byte)buffer[index++];
            ManualOverrideSwitch = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Roll).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitch).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yaw).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Thrust).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ModeSwitch).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(ManualOverrideSwitch).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/22;
        }

        /// <summary>
        /// Timestamp in milliseconds since system boot
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Desired roll rate in radians per second
        /// OriginName: roll, Units: rad/s, IsExtended: false
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Desired pitch rate in radians per second
        /// OriginName: pitch, Units: rad/s, IsExtended: false
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Desired yaw rate in radians per second
        /// OriginName: yaw, Units: rad/s, IsExtended: false
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// Collective thrust, normalized to 0 .. 1
        /// OriginName: thrust, Units: , IsExtended: false
        /// </summary>
        public float Thrust { get; set; }
        /// <summary>
        /// Flight mode switch position, 0.. 255
        /// OriginName: mode_switch, Units: , IsExtended: false
        /// </summary>
        public byte ModeSwitch { get; set; }
        /// <summary>
        /// Override mode switch position, 0.. 255
        /// OriginName: manual_override_switch, Units: , IsExtended: false
        /// </summary>
        public byte ManualOverrideSwitch { get; set; }
    }
    /// <summary>
    /// Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system).
    ///  SET_ATTITUDE_TARGET
    /// </summary>
    public class SetAttitudeTargetPacket: PacketV2<SetAttitudeTargetPayload>
    {
	public const int PacketMessageId = 82;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 49;

        public override SetAttitudeTargetPayload Payload { get; } = new SetAttitudeTargetPayload();

        public override string Name => "SET_ATTITUDE_TARGET";
    }

    /// <summary>
    ///  SET_ATTITUDE_TARGET
    /// </summary>
    public class SetAttitudeTargetPayload : IPayload
    {
        public byte GetMaxByteSize() => 39;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            arraySize = /*ArrayLength*/4 - Math.Max(0,((/*PayloadByteSize*/39 - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<4;i++)
            {
                Q[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            BodyRollRate = BitConverter.ToSingle(buffer, index);index+=4;
            BodyPitchRate = BitConverter.ToSingle(buffer, index);index+=4;
            BodyYawRate = BitConverter.ToSingle(buffer, index);index+=4;
            Thrust = BitConverter.ToSingle(buffer, index);index+=4;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            TypeMask = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(Q[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(BodyRollRate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(BodyPitchRate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(BodyYawRate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Thrust).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TypeMask).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/39;
        }

        /// <summary>
        /// Timestamp in milliseconds since system boot
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        /// OriginName: q, Units: , IsExtended: false
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Body roll rate in radians per second
        /// OriginName: body_roll_rate, Units: rad/s, IsExtended: false
        /// </summary>
        public float BodyRollRate { get; set; }
        /// <summary>
        /// Body pitch rate in radians per second
        /// OriginName: body_pitch_rate, Units: rad/s, IsExtended: false
        /// </summary>
        public float BodyPitchRate { get; set; }
        /// <summary>
        /// Body yaw rate in radians per second
        /// OriginName: body_yaw_rate, Units: rad/s, IsExtended: false
        /// </summary>
        public float BodyYawRate { get; set; }
        /// <summary>
        /// Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
        /// OriginName: thrust, Units: , IsExtended: false
        /// </summary>
        public float Thrust { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
        /// OriginName: type_mask, Units: , IsExtended: false
        /// </summary>
        public byte TypeMask { get; set; }
    }
    /// <summary>
    /// Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.
    ///  ATTITUDE_TARGET
    /// </summary>
    public class AttitudeTargetPacket: PacketV2<AttitudeTargetPayload>
    {
	public const int PacketMessageId = 83;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 22;

        public override AttitudeTargetPayload Payload { get; } = new AttitudeTargetPayload();

        public override string Name => "ATTITUDE_TARGET";
    }

    /// <summary>
    ///  ATTITUDE_TARGET
    /// </summary>
    public class AttitudeTargetPayload : IPayload
    {
        public byte GetMaxByteSize() => 37;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            arraySize = /*ArrayLength*/4 - Math.Max(0,((/*PayloadByteSize*/37 - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<4;i++)
            {
                Q[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            BodyRollRate = BitConverter.ToSingle(buffer, index);index+=4;
            BodyPitchRate = BitConverter.ToSingle(buffer, index);index+=4;
            BodyYawRate = BitConverter.ToSingle(buffer, index);index+=4;
            Thrust = BitConverter.ToSingle(buffer, index);index+=4;
            TypeMask = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(Q[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(BodyRollRate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(BodyPitchRate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(BodyYawRate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Thrust).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TypeMask).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/37;
        }

        /// <summary>
        /// Timestamp in milliseconds since system boot
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        /// OriginName: q, Units: , IsExtended: false
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Body roll rate in radians per second
        /// OriginName: body_roll_rate, Units: rad/s, IsExtended: false
        /// </summary>
        public float BodyRollRate { get; set; }
        /// <summary>
        /// Body pitch rate in radians per second
        /// OriginName: body_pitch_rate, Units: rad/s, IsExtended: false
        /// </summary>
        public float BodyPitchRate { get; set; }
        /// <summary>
        /// Body yaw rate in radians per second
        /// OriginName: body_yaw_rate, Units: rad/s, IsExtended: false
        /// </summary>
        public float BodyYawRate { get; set; }
        /// <summary>
        /// Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
        /// OriginName: thrust, Units: , IsExtended: false
        /// </summary>
        public float Thrust { get; set; }
        /// <summary>
        /// Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude
        /// OriginName: type_mask, Units: , IsExtended: false
        /// </summary>
        public byte TypeMask { get; set; }
    }
    /// <summary>
    /// Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system).
    ///  SET_POSITION_TARGET_LOCAL_NED
    /// </summary>
    public class SetPositionTargetLocalNedPacket: PacketV2<SetPositionTargetLocalNedPayload>
    {
	public const int PacketMessageId = 84;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 143;

        public override SetPositionTargetLocalNedPayload Payload { get; } = new SetPositionTargetLocalNedPayload();

        public override string Name => "SET_POSITION_TARGET_LOCAL_NED";
    }

    /// <summary>
    ///  SET_POSITION_TARGET_LOCAL_NED
    /// </summary>
    public class SetPositionTargetLocalNedPayload : IPayload
    {
        public byte GetMaxByteSize() => 53;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Vx = BitConverter.ToSingle(buffer, index);index+=4;
            Vy = BitConverter.ToSingle(buffer, index);index+=4;
            Vz = BitConverter.ToSingle(buffer, index);index+=4;
            Afx = BitConverter.ToSingle(buffer, index);index+=4;
            Afy = BitConverter.ToSingle(buffer, index);index+=4;
            Afz = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            YawRate = BitConverter.ToSingle(buffer, index);index+=4;
            TypeMask = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            CoordinateFrame = (MavFrame)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vx).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vz).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Afx).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Afy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Afz).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yaw).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(YawRate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TypeMask).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)CoordinateFrame;index+=1;
            return /*PayloadByteSize*/53;
        }

        /// <summary>
        /// Timestamp in milliseconds since system boot
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X Position in NED frame in meters
        /// OriginName: x, Units: m, IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y Position in NED frame in meters
        /// OriginName: y, Units: m, IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z Position in NED frame in meters (note, altitude is negative in NED)
        /// OriginName: z, Units: m, IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// X velocity in NED frame in meter / s
        /// OriginName: vx, Units: m/s, IsExtended: false
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Y velocity in NED frame in meter / s
        /// OriginName: vy, Units: m/s, IsExtended: false
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Z velocity in NED frame in meter / s
        /// OriginName: vz, Units: m/s, IsExtended: false
        /// </summary>
        public float Vz { get; set; }
        /// <summary>
        /// X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// OriginName: afx, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Afx { get; set; }
        /// <summary>
        /// Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// OriginName: afy, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Afy { get; set; }
        /// <summary>
        /// Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// OriginName: afz, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Afz { get; set; }
        /// <summary>
        /// yaw setpoint in rad
        /// OriginName: yaw, Units: rad, IsExtended: false
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// yaw rate setpoint in rad/s
        /// OriginName: yaw_rate, Units: rad/s, IsExtended: false
        /// </summary>
        public float YawRate { get; set; }
        /// <summary>
        /// Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
        /// OriginName: type_mask, Units: , IsExtended: false
        /// </summary>
        public ushort TypeMask { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
        /// OriginName: coordinate_frame, Units: , IsExtended: false
        /// </summary>
        public MavFrame CoordinateFrame { get; set; }
    }
    /// <summary>
    /// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled this way.
    ///  POSITION_TARGET_LOCAL_NED
    /// </summary>
    public class PositionTargetLocalNedPacket: PacketV2<PositionTargetLocalNedPayload>
    {
	public const int PacketMessageId = 85;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 140;

        public override PositionTargetLocalNedPayload Payload { get; } = new PositionTargetLocalNedPayload();

        public override string Name => "POSITION_TARGET_LOCAL_NED";
    }

    /// <summary>
    ///  POSITION_TARGET_LOCAL_NED
    /// </summary>
    public class PositionTargetLocalNedPayload : IPayload
    {
        public byte GetMaxByteSize() => 51;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Vx = BitConverter.ToSingle(buffer, index);index+=4;
            Vy = BitConverter.ToSingle(buffer, index);index+=4;
            Vz = BitConverter.ToSingle(buffer, index);index+=4;
            Afx = BitConverter.ToSingle(buffer, index);index+=4;
            Afy = BitConverter.ToSingle(buffer, index);index+=4;
            Afz = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            YawRate = BitConverter.ToSingle(buffer, index);index+=4;
            TypeMask = BitConverter.ToUInt16(buffer,index);index+=2;
            CoordinateFrame = (MavFrame)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vx).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vz).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Afx).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Afy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Afz).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yaw).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(YawRate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TypeMask).CopyTo(buffer, index);index+=2;
            buffer[index] = (byte)CoordinateFrame;index+=1;
            return /*PayloadByteSize*/51;
        }

        /// <summary>
        /// Timestamp in milliseconds since system boot
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X Position in NED frame in meters
        /// OriginName: x, Units: m, IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y Position in NED frame in meters
        /// OriginName: y, Units: m, IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z Position in NED frame in meters (note, altitude is negative in NED)
        /// OriginName: z, Units: m, IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// X velocity in NED frame in meter / s
        /// OriginName: vx, Units: m/s, IsExtended: false
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Y velocity in NED frame in meter / s
        /// OriginName: vy, Units: m/s, IsExtended: false
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Z velocity in NED frame in meter / s
        /// OriginName: vz, Units: m/s, IsExtended: false
        /// </summary>
        public float Vz { get; set; }
        /// <summary>
        /// X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// OriginName: afx, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Afx { get; set; }
        /// <summary>
        /// Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// OriginName: afy, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Afy { get; set; }
        /// <summary>
        /// Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// OriginName: afz, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Afz { get; set; }
        /// <summary>
        /// yaw setpoint in rad
        /// OriginName: yaw, Units: rad, IsExtended: false
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// yaw rate setpoint in rad/s
        /// OriginName: yaw_rate, Units: rad/s, IsExtended: false
        /// </summary>
        public float YawRate { get; set; }
        /// <summary>
        /// Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
        /// OriginName: type_mask, Units: , IsExtended: false
        /// </summary>
        public ushort TypeMask { get; set; }
        /// <summary>
        /// Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
        /// OriginName: coordinate_frame, Units: , IsExtended: false
        /// </summary>
        public MavFrame CoordinateFrame { get; set; }
    }
    /// <summary>
    /// Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84). Used by an external controller to command the vehicle (manual controller or other system).
    ///  SET_POSITION_TARGET_GLOBAL_INT
    /// </summary>
    public class SetPositionTargetGlobalIntPacket: PacketV2<SetPositionTargetGlobalIntPayload>
    {
	public const int PacketMessageId = 86;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 5;

        public override SetPositionTargetGlobalIntPayload Payload { get; } = new SetPositionTargetGlobalIntPayload();

        public override string Name => "SET_POSITION_TARGET_GLOBAL_INT";
    }

    /// <summary>
    ///  SET_POSITION_TARGET_GLOBAL_INT
    /// </summary>
    public class SetPositionTargetGlobalIntPayload : IPayload
    {
        public byte GetMaxByteSize() => 53;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            LatInt = BitConverter.ToInt32(buffer,index);index+=4;
            LonInt = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToSingle(buffer, index);index+=4;
            Vx = BitConverter.ToSingle(buffer, index);index+=4;
            Vy = BitConverter.ToSingle(buffer, index);index+=4;
            Vz = BitConverter.ToSingle(buffer, index);index+=4;
            Afx = BitConverter.ToSingle(buffer, index);index+=4;
            Afy = BitConverter.ToSingle(buffer, index);index+=4;
            Afz = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            YawRate = BitConverter.ToSingle(buffer, index);index+=4;
            TypeMask = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            CoordinateFrame = (MavFrame)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(LatInt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(LonInt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vx).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vz).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Afx).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Afy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Afz).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yaw).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(YawRate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TypeMask).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)CoordinateFrame;index+=1;
            return /*PayloadByteSize*/53;
        }

        /// <summary>
        /// Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X Position in WGS84 frame in 1e7 * degrees
        /// OriginName: lat_int, Units: degE7, IsExtended: false
        /// </summary>
        public int LatInt { get; set; }
        /// <summary>
        /// Y Position in WGS84 frame in 1e7 * degrees
        /// OriginName: lon_int, Units: degE7, IsExtended: false
        /// </summary>
        public int LonInt { get; set; }
        /// <summary>
        /// Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        /// OriginName: alt, Units: m, IsExtended: false
        /// </summary>
        public float Alt { get; set; }
        /// <summary>
        /// X velocity in NED frame in meter / s
        /// OriginName: vx, Units: m/s, IsExtended: false
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Y velocity in NED frame in meter / s
        /// OriginName: vy, Units: m/s, IsExtended: false
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Z velocity in NED frame in meter / s
        /// OriginName: vz, Units: m/s, IsExtended: false
        /// </summary>
        public float Vz { get; set; }
        /// <summary>
        /// X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// OriginName: afx, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Afx { get; set; }
        /// <summary>
        /// Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// OriginName: afy, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Afy { get; set; }
        /// <summary>
        /// Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// OriginName: afz, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Afz { get; set; }
        /// <summary>
        /// yaw setpoint in rad
        /// OriginName: yaw, Units: rad, IsExtended: false
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// yaw rate setpoint in rad/s
        /// OriginName: yaw_rate, Units: rad/s, IsExtended: false
        /// </summary>
        public float YawRate { get; set; }
        /// <summary>
        /// Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
        /// OriginName: type_mask, Units: , IsExtended: false
        /// </summary>
        public ushort TypeMask { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
        /// OriginName: coordinate_frame, Units: , IsExtended: false
        /// </summary>
        public MavFrame CoordinateFrame { get; set; }
    }
    /// <summary>
    /// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way.
    ///  POSITION_TARGET_GLOBAL_INT
    /// </summary>
    public class PositionTargetGlobalIntPacket: PacketV2<PositionTargetGlobalIntPayload>
    {
	public const int PacketMessageId = 87;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 150;

        public override PositionTargetGlobalIntPayload Payload { get; } = new PositionTargetGlobalIntPayload();

        public override string Name => "POSITION_TARGET_GLOBAL_INT";
    }

    /// <summary>
    ///  POSITION_TARGET_GLOBAL_INT
    /// </summary>
    public class PositionTargetGlobalIntPayload : IPayload
    {
        public byte GetMaxByteSize() => 51;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            LatInt = BitConverter.ToInt32(buffer,index);index+=4;
            LonInt = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToSingle(buffer, index);index+=4;
            Vx = BitConverter.ToSingle(buffer, index);index+=4;
            Vy = BitConverter.ToSingle(buffer, index);index+=4;
            Vz = BitConverter.ToSingle(buffer, index);index+=4;
            Afx = BitConverter.ToSingle(buffer, index);index+=4;
            Afy = BitConverter.ToSingle(buffer, index);index+=4;
            Afz = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            YawRate = BitConverter.ToSingle(buffer, index);index+=4;
            TypeMask = BitConverter.ToUInt16(buffer,index);index+=2;
            CoordinateFrame = (MavFrame)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(LatInt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(LonInt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vx).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vz).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Afx).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Afy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Afz).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yaw).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(YawRate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TypeMask).CopyTo(buffer, index);index+=2;
            buffer[index] = (byte)CoordinateFrame;index+=1;
            return /*PayloadByteSize*/51;
        }

        /// <summary>
        /// Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X Position in WGS84 frame in 1e7 * degrees
        /// OriginName: lat_int, Units: degE7, IsExtended: false
        /// </summary>
        public int LatInt { get; set; }
        /// <summary>
        /// Y Position in WGS84 frame in 1e7 * degrees
        /// OriginName: lon_int, Units: degE7, IsExtended: false
        /// </summary>
        public int LonInt { get; set; }
        /// <summary>
        /// Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        /// OriginName: alt, Units: m, IsExtended: false
        /// </summary>
        public float Alt { get; set; }
        /// <summary>
        /// X velocity in NED frame in meter / s
        /// OriginName: vx, Units: m/s, IsExtended: false
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Y velocity in NED frame in meter / s
        /// OriginName: vy, Units: m/s, IsExtended: false
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Z velocity in NED frame in meter / s
        /// OriginName: vz, Units: m/s, IsExtended: false
        /// </summary>
        public float Vz { get; set; }
        /// <summary>
        /// X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// OriginName: afx, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Afx { get; set; }
        /// <summary>
        /// Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// OriginName: afy, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Afy { get; set; }
        /// <summary>
        /// Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// OriginName: afz, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Afz { get; set; }
        /// <summary>
        /// yaw setpoint in rad
        /// OriginName: yaw, Units: rad, IsExtended: false
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// yaw rate setpoint in rad/s
        /// OriginName: yaw_rate, Units: rad/s, IsExtended: false
        /// </summary>
        public float YawRate { get; set; }
        /// <summary>
        /// Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
        /// OriginName: type_mask, Units: , IsExtended: false
        /// </summary>
        public ushort TypeMask { get; set; }
        /// <summary>
        /// Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
        /// OriginName: coordinate_frame, Units: , IsExtended: false
        /// </summary>
        public MavFrame CoordinateFrame { get; set; }
    }
    /// <summary>
    /// The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
    ///  LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
    /// </summary>
    public class LocalPositionNedSystemGlobalOffsetPacket: PacketV2<LocalPositionNedSystemGlobalOffsetPayload>
    {
	public const int PacketMessageId = 89;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 231;

        public override LocalPositionNedSystemGlobalOffsetPayload Payload { get; } = new LocalPositionNedSystemGlobalOffsetPayload();

        public override string Name => "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET";
    }

    /// <summary>
    ///  LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
    /// </summary>
    public class LocalPositionNedSystemGlobalOffsetPayload : IPayload
    {
        public byte GetMaxByteSize() => 28;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Roll).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitch).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yaw).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/28;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X Position
        /// OriginName: x, Units: m, IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y Position
        /// OriginName: y, Units: m, IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z Position
        /// OriginName: z, Units: m, IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Roll
        /// OriginName: roll, Units: rad, IsExtended: false
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Pitch
        /// OriginName: pitch, Units: rad, IsExtended: false
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Yaw
        /// OriginName: yaw, Units: rad, IsExtended: false
        /// </summary>
        public float Yaw { get; set; }
    }
    /// <summary>
    /// DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
    ///  HIL_STATE
    /// </summary>
    public class HilStatePacket: PacketV2<HilStatePayload>
    {
	public const int PacketMessageId = 90;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 183;

        public override HilStatePayload Payload { get; } = new HilStatePayload();

        public override string Name => "HIL_STATE";
    }

    /// <summary>
    ///  HIL_STATE
    /// </summary>
    public class HilStatePayload : IPayload
    {
        public byte GetMaxByteSize() => 56;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            Rollspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Pitchspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Yawspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToInt32(buffer,index);index+=4;
            Vx = BitConverter.ToInt16(buffer,index);index+=2;
            Vy = BitConverter.ToInt16(buffer,index);index+=2;
            Vz = BitConverter.ToInt16(buffer,index);index+=2;
            Xacc = BitConverter.ToInt16(buffer,index);index+=2;
            Yacc = BitConverter.ToInt16(buffer,index);index+=2;
            Zacc = BitConverter.ToInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Roll).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitch).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yaw).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Rollspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitchspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yawspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vx).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Vy).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Vz).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Xacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Yacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Zacc).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/56;
        }

        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Roll angle (rad)
        /// OriginName: roll, Units: rad, IsExtended: false
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Pitch angle (rad)
        /// OriginName: pitch, Units: rad, IsExtended: false
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Yaw angle (rad)
        /// OriginName: yaw, Units: rad, IsExtended: false
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// Body frame roll / phi angular speed (rad/s)
        /// OriginName: rollspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Rollspeed { get; set; }
        /// <summary>
        /// Body frame pitch / theta angular speed (rad/s)
        /// OriginName: pitchspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Pitchspeed { get; set; }
        /// <summary>
        /// Body frame yaw / psi angular speed (rad/s)
        /// OriginName: yawspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Yawspeed { get; set; }
        /// <summary>
        /// Latitude, expressed as degrees * 1E7
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude, expressed as degrees * 1E7
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude in meters, expressed as * 1000 (millimeters)
        /// OriginName: alt, Units: mm, IsExtended: false
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// Ground X Speed (Latitude), expressed as m/s * 100
        /// OriginName: vx, Units: cm/s, IsExtended: false
        /// </summary>
        public short Vx { get; set; }
        /// <summary>
        /// Ground Y Speed (Longitude), expressed as m/s * 100
        /// OriginName: vy, Units: cm/s, IsExtended: false
        /// </summary>
        public short Vy { get; set; }
        /// <summary>
        /// Ground Z Speed (Altitude), expressed as m/s * 100
        /// OriginName: vz, Units: cm/s, IsExtended: false
        /// </summary>
        public short Vz { get; set; }
        /// <summary>
        /// X acceleration (mg)
        /// OriginName: xacc, Units: mG, IsExtended: false
        /// </summary>
        public short Xacc { get; set; }
        /// <summary>
        /// Y acceleration (mg)
        /// OriginName: yacc, Units: mG, IsExtended: false
        /// </summary>
        public short Yacc { get; set; }
        /// <summary>
        /// Z acceleration (mg)
        /// OriginName: zacc, Units: mG, IsExtended: false
        /// </summary>
        public short Zacc { get; set; }
    }
    /// <summary>
    /// Sent from autopilot to simulation. Hardware in the loop control outputs
    ///  HIL_CONTROLS
    /// </summary>
    public class HilControlsPacket: PacketV2<HilControlsPayload>
    {
	public const int PacketMessageId = 91;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 63;

        public override HilControlsPayload Payload { get; } = new HilControlsPayload();

        public override string Name => "HIL_CONTROLS";
    }

    /// <summary>
    ///  HIL_CONTROLS
    /// </summary>
    public class HilControlsPayload : IPayload
    {
        public byte GetMaxByteSize() => 42;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            RollAilerons = BitConverter.ToSingle(buffer, index);index+=4;
            PitchElevator = BitConverter.ToSingle(buffer, index);index+=4;
            YawRudder = BitConverter.ToSingle(buffer, index);index+=4;
            Throttle = BitConverter.ToSingle(buffer, index);index+=4;
            Aux1 = BitConverter.ToSingle(buffer, index);index+=4;
            Aux2 = BitConverter.ToSingle(buffer, index);index+=4;
            Aux3 = BitConverter.ToSingle(buffer, index);index+=4;
            Aux4 = BitConverter.ToSingle(buffer, index);index+=4;
            Mode = (MavMode)buffer[index++];
            NavMode = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(RollAilerons).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PitchElevator).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(YawRudder).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Throttle).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Aux1).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Aux2).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Aux3).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Aux4).CopyTo(buffer, index);index+=4;
            buffer[index] = (byte)Mode;index+=1;
            BitConverter.GetBytes(NavMode).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/42;
        }

        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Control output -1 .. 1
        /// OriginName: roll_ailerons, Units: , IsExtended: false
        /// </summary>
        public float RollAilerons { get; set; }
        /// <summary>
        /// Control output -1 .. 1
        /// OriginName: pitch_elevator, Units: , IsExtended: false
        /// </summary>
        public float PitchElevator { get; set; }
        /// <summary>
        /// Control output -1 .. 1
        /// OriginName: yaw_rudder, Units: , IsExtended: false
        /// </summary>
        public float YawRudder { get; set; }
        /// <summary>
        /// Throttle 0 .. 1
        /// OriginName: throttle, Units: , IsExtended: false
        /// </summary>
        public float Throttle { get; set; }
        /// <summary>
        /// Aux 1, -1 .. 1
        /// OriginName: aux1, Units: , IsExtended: false
        /// </summary>
        public float Aux1 { get; set; }
        /// <summary>
        /// Aux 2, -1 .. 1
        /// OriginName: aux2, Units: , IsExtended: false
        /// </summary>
        public float Aux2 { get; set; }
        /// <summary>
        /// Aux 3, -1 .. 1
        /// OriginName: aux3, Units: , IsExtended: false
        /// </summary>
        public float Aux3 { get; set; }
        /// <summary>
        /// Aux 4, -1 .. 1
        /// OriginName: aux4, Units: , IsExtended: false
        /// </summary>
        public float Aux4 { get; set; }
        /// <summary>
        /// System mode (MAV_MODE)
        /// OriginName: mode, Units: , IsExtended: false
        /// </summary>
        public MavMode Mode { get; set; }
        /// <summary>
        /// Navigation mode (MAV_NAV_MODE)
        /// OriginName: nav_mode, Units: , IsExtended: false
        /// </summary>
        public byte NavMode { get; set; }
    }
    /// <summary>
    /// Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
    ///  HIL_RC_INPUTS_RAW
    /// </summary>
    public class HilRcInputsRawPacket: PacketV2<HilRcInputsRawPayload>
    {
	public const int PacketMessageId = 92;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 54;

        public override HilRcInputsRawPayload Payload { get; } = new HilRcInputsRawPayload();

        public override string Name => "HIL_RC_INPUTS_RAW";
    }

    /// <summary>
    ///  HIL_RC_INPUTS_RAW
    /// </summary>
    public class HilRcInputsRawPayload : IPayload
    {
        public byte GetMaxByteSize() => 33;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Chan1Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan2Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan3Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan4Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan5Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan6Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan7Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan8Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan9Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan10Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan11Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Chan12Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Rssi = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Chan1Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan2Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan3Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan4Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan5Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan6Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan7Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan8Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan9Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan10Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan11Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Chan12Raw).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Rssi).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/33;
        }

        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// RC channel 1 value, in microseconds
        /// OriginName: chan1_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan1Raw { get; set; }
        /// <summary>
        /// RC channel 2 value, in microseconds
        /// OriginName: chan2_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan2Raw { get; set; }
        /// <summary>
        /// RC channel 3 value, in microseconds
        /// OriginName: chan3_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan3Raw { get; set; }
        /// <summary>
        /// RC channel 4 value, in microseconds
        /// OriginName: chan4_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan4Raw { get; set; }
        /// <summary>
        /// RC channel 5 value, in microseconds
        /// OriginName: chan5_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan5Raw { get; set; }
        /// <summary>
        /// RC channel 6 value, in microseconds
        /// OriginName: chan6_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan6Raw { get; set; }
        /// <summary>
        /// RC channel 7 value, in microseconds
        /// OriginName: chan7_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan7Raw { get; set; }
        /// <summary>
        /// RC channel 8 value, in microseconds
        /// OriginName: chan8_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan8Raw { get; set; }
        /// <summary>
        /// RC channel 9 value, in microseconds
        /// OriginName: chan9_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan9Raw { get; set; }
        /// <summary>
        /// RC channel 10 value, in microseconds
        /// OriginName: chan10_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan10Raw { get; set; }
        /// <summary>
        /// RC channel 11 value, in microseconds
        /// OriginName: chan11_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan11Raw { get; set; }
        /// <summary>
        /// RC channel 12 value, in microseconds
        /// OriginName: chan12_raw, Units: us, IsExtended: false
        /// </summary>
        public ushort Chan12Raw { get; set; }
        /// <summary>
        /// Receive signal strength indicator, 0: 0%, 255: 100%
        /// OriginName: rssi, Units: , IsExtended: false
        /// </summary>
        public byte Rssi { get; set; }
    }
    /// <summary>
    /// Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS)
    ///  HIL_ACTUATOR_CONTROLS
    /// </summary>
    public class HilActuatorControlsPacket: PacketV2<HilActuatorControlsPayload>
    {
	public const int PacketMessageId = 93;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 47;

        public override HilActuatorControlsPayload Payload { get; } = new HilActuatorControlsPayload();

        public override string Name => "HIL_ACTUATOR_CONTROLS";
    }

    /// <summary>
    ///  HIL_ACTUATOR_CONTROLS
    /// </summary>
    public class HilActuatorControlsPayload : IPayload
    {
        public byte GetMaxByteSize() => 81;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Flags = BitConverter.ToUInt64(buffer,index);index+=8;
            arraySize = /*ArrayLength*/16 - Math.Max(0,((/*PayloadByteSize*/81 - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<16;i++)
            {
                Controls[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                Controls[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            Mode = (MavMode)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Flags).CopyTo(buffer, index);index+=8;
            for(var i=0;i<16;i++)
            {
                BitConverter.GetBytes(Controls[i]).CopyTo(buffer, index);index+=4;
            }
            buffer[index] = (byte)Mode;index+=1;
            return /*PayloadByteSize*/81;
        }

        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Flags as bitfield, reserved for future use.
        /// OriginName: flags, Units: , IsExtended: false
        /// </summary>
        public ulong Flags { get; set; }
        /// <summary>
        /// Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
        /// OriginName: controls, Units: , IsExtended: false
        /// </summary>
        public float[] Controls { get; } = new float[16];
        /// <summary>
        /// System mode (MAV_MODE), includes arming state.
        /// OriginName: mode, Units: , IsExtended: false
        /// </summary>
        public MavMode Mode { get; set; }
    }
    /// <summary>
    /// Optical flow from a flow sensor (e.g. optical mouse sensor)
    ///  OPTICAL_FLOW
    /// </summary>
    public class OpticalFlowPacket: PacketV2<OpticalFlowPayload>
    {
	public const int PacketMessageId = 100;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 175;

        public override OpticalFlowPayload Payload { get; } = new OpticalFlowPayload();

        public override string Name => "OPTICAL_FLOW";
    }

    /// <summary>
    ///  OPTICAL_FLOW
    /// </summary>
    public class OpticalFlowPayload : IPayload
    {
        public byte GetMaxByteSize() => 34;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            FlowCompMX = BitConverter.ToSingle(buffer, index);index+=4;
            FlowCompMY = BitConverter.ToSingle(buffer, index);index+=4;
            GroundDistance = BitConverter.ToSingle(buffer, index);index+=4;
            FlowX = BitConverter.ToInt16(buffer,index);index+=2;
            FlowY = BitConverter.ToInt16(buffer,index);index+=2;
            SensorId = (byte)buffer[index++];
            Quality = (byte)buffer[index++];
            // extended field 'FlowRateX' can be empty
            if (index >= endIndex) return;
            FlowRateX = BitConverter.ToSingle(buffer, index);index+=4;
            // extended field 'FlowRateY' can be empty
            if (index >= endIndex) return;
            FlowRateY = BitConverter.ToSingle(buffer, index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(FlowCompMX).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(FlowCompMY).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(GroundDistance).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(FlowX).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(FlowY).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(SensorId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Quality).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(FlowRateX).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(FlowRateY).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/34;
        }

        /// <summary>
        /// Timestamp (UNIX)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Flow in meters in x-sensor direction, angular-speed compensated
        /// OriginName: flow_comp_m_x, Units: m, IsExtended: false
        /// </summary>
        public float FlowCompMX { get; set; }
        /// <summary>
        /// Flow in meters in y-sensor direction, angular-speed compensated
        /// OriginName: flow_comp_m_y, Units: m, IsExtended: false
        /// </summary>
        public float FlowCompMY { get; set; }
        /// <summary>
        /// Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
        /// OriginName: ground_distance, Units: m, IsExtended: false
        /// </summary>
        public float GroundDistance { get; set; }
        /// <summary>
        /// Flow in pixels * 10 in x-sensor direction (dezi-pixels)
        /// OriginName: flow_x, Units: dpixels, IsExtended: false
        /// </summary>
        public short FlowX { get; set; }
        /// <summary>
        /// Flow in pixels * 10 in y-sensor direction (dezi-pixels)
        /// OriginName: flow_y, Units: dpixels, IsExtended: false
        /// </summary>
        public short FlowY { get; set; }
        /// <summary>
        /// Sensor ID
        /// OriginName: sensor_id, Units: , IsExtended: false
        /// </summary>
        public byte SensorId { get; set; }
        /// <summary>
        /// Optical flow quality / confidence. 0: bad, 255: maximum quality
        /// OriginName: quality, Units: , IsExtended: false
        /// </summary>
        public byte Quality { get; set; }
        /// <summary>
        /// Flow rate in radians/second about X axis
        /// OriginName: flow_rate_x, Units: rad/s, IsExtended: true
        /// </summary>
        public float FlowRateX { get; set; }
        /// <summary>
        /// Flow rate in radians/second about Y axis
        /// OriginName: flow_rate_y, Units: rad/s, IsExtended: true
        /// </summary>
        public float FlowRateY { get; set; }
    }
    /// <summary>
    ///  GLOBAL_VISION_POSITION_ESTIMATE
    /// </summary>
    public class GlobalVisionPositionEstimatePacket: PacketV2<GlobalVisionPositionEstimatePayload>
    {
	public const int PacketMessageId = 101;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 102;

        public override GlobalVisionPositionEstimatePayload Payload { get; } = new GlobalVisionPositionEstimatePayload();

        public override string Name => "GLOBAL_VISION_POSITION_ESTIMATE";
    }

    /// <summary>
    ///  GLOBAL_VISION_POSITION_ESTIMATE
    /// </summary>
    public class GlobalVisionPositionEstimatePayload : IPayload
    {
        public byte GetMaxByteSize() => 116;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Usec = BitConverter.ToUInt64(buffer,index);index+=8;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            // extended field 'Covariance' can be empty
            if (index >= endIndex) return;
            arraySize = 21;
            for(var i=0;i<arraySize;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Usec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Roll).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitch).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yaw).CopyTo(buffer, index);index+=4;
            for(var i=0;i<21;i++)
            {
                BitConverter.GetBytes(Covariance[i]).CopyTo(buffer, index);index+=4;
            }
            return /*PayloadByteSize*/116;
        }

        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// OriginName: usec, Units: us, IsExtended: false
        /// </summary>
        public ulong Usec { get; set; }
        /// <summary>
        /// Global X position
        /// OriginName: x, Units: m, IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Global Y position
        /// OriginName: y, Units: m, IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Global Z position
        /// OriginName: z, Units: m, IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Roll angle in rad
        /// OriginName: roll, Units: rad, IsExtended: false
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Pitch angle in rad
        /// OriginName: pitch, Units: rad, IsExtended: false
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Yaw angle in rad
        /// OriginName: yaw, Units: rad, IsExtended: false
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// Pose covariance matrix upper right triangular (first six entries are the first ROW, next five entries are the second ROW, etc.)
        /// OriginName: covariance, Units: , IsExtended: true
        /// </summary>
        public float[] Covariance { get; } = new float[21];
    }
    /// <summary>
    ///  VISION_POSITION_ESTIMATE
    /// </summary>
    public class VisionPositionEstimatePacket: PacketV2<VisionPositionEstimatePayload>
    {
	public const int PacketMessageId = 102;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 158;

        public override VisionPositionEstimatePayload Payload { get; } = new VisionPositionEstimatePayload();

        public override string Name => "VISION_POSITION_ESTIMATE";
    }

    /// <summary>
    ///  VISION_POSITION_ESTIMATE
    /// </summary>
    public class VisionPositionEstimatePayload : IPayload
    {
        public byte GetMaxByteSize() => 116;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Usec = BitConverter.ToUInt64(buffer,index);index+=8;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            // extended field 'Covariance' can be empty
            if (index >= endIndex) return;
            arraySize = 21;
            for(var i=0;i<arraySize;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Usec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Roll).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitch).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yaw).CopyTo(buffer, index);index+=4;
            for(var i=0;i<21;i++)
            {
                BitConverter.GetBytes(Covariance[i]).CopyTo(buffer, index);index+=4;
            }
            return /*PayloadByteSize*/116;
        }

        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// OriginName: usec, Units: us, IsExtended: false
        /// </summary>
        public ulong Usec { get; set; }
        /// <summary>
        /// Global X position
        /// OriginName: x, Units: m, IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Global Y position
        /// OriginName: y, Units: m, IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Global Z position
        /// OriginName: z, Units: m, IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Roll angle in rad
        /// OriginName: roll, Units: rad, IsExtended: false
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Pitch angle in rad
        /// OriginName: pitch, Units: rad, IsExtended: false
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Yaw angle in rad
        /// OriginName: yaw, Units: rad, IsExtended: false
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// Pose covariance matrix upper right triangular (first six entries are the first ROW, next five entries are the second ROW, etc.)
        /// OriginName: covariance, Units: , IsExtended: true
        /// </summary>
        public float[] Covariance { get; } = new float[21];
    }
    /// <summary>
    ///  VISION_SPEED_ESTIMATE
    /// </summary>
    public class VisionSpeedEstimatePacket: PacketV2<VisionSpeedEstimatePayload>
    {
	public const int PacketMessageId = 103;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 208;

        public override VisionSpeedEstimatePayload Payload { get; } = new VisionSpeedEstimatePayload();

        public override string Name => "VISION_SPEED_ESTIMATE";
    }

    /// <summary>
    ///  VISION_SPEED_ESTIMATE
    /// </summary>
    public class VisionSpeedEstimatePayload : IPayload
    {
        public byte GetMaxByteSize() => 56;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Usec = BitConverter.ToUInt64(buffer,index);index+=8;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            // extended field 'Covariance' can be empty
            if (index >= endIndex) return;
            arraySize = 9;
            for(var i=0;i<arraySize;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Usec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            for(var i=0;i<9;i++)
            {
                BitConverter.GetBytes(Covariance[i]).CopyTo(buffer, index);index+=4;
            }
            return /*PayloadByteSize*/56;
        }

        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// OriginName: usec, Units: us, IsExtended: false
        /// </summary>
        public ulong Usec { get; set; }
        /// <summary>
        /// Global X speed
        /// OriginName: x, Units: m/s, IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Global Y speed
        /// OriginName: y, Units: m/s, IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Global Z speed
        /// OriginName: z, Units: m/s, IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Linear velocity covariance matrix (1st three entries - 1st row, etc.)
        /// OriginName: covariance, Units: , IsExtended: true
        /// </summary>
        public float[] Covariance { get; } = new float[9];
    }
    /// <summary>
    ///  VICON_POSITION_ESTIMATE
    /// </summary>
    public class ViconPositionEstimatePacket: PacketV2<ViconPositionEstimatePayload>
    {
	public const int PacketMessageId = 104;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 56;

        public override ViconPositionEstimatePayload Payload { get; } = new ViconPositionEstimatePayload();

        public override string Name => "VICON_POSITION_ESTIMATE";
    }

    /// <summary>
    ///  VICON_POSITION_ESTIMATE
    /// </summary>
    public class ViconPositionEstimatePayload : IPayload
    {
        public byte GetMaxByteSize() => 116;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Usec = BitConverter.ToUInt64(buffer,index);index+=8;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            // extended field 'Covariance' can be empty
            if (index >= endIndex) return;
            arraySize = 21;
            for(var i=0;i<arraySize;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Usec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Roll).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitch).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yaw).CopyTo(buffer, index);index+=4;
            for(var i=0;i<21;i++)
            {
                BitConverter.GetBytes(Covariance[i]).CopyTo(buffer, index);index+=4;
            }
            return /*PayloadByteSize*/116;
        }

        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// OriginName: usec, Units: us, IsExtended: false
        /// </summary>
        public ulong Usec { get; set; }
        /// <summary>
        /// Global X position
        /// OriginName: x, Units: m, IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Global Y position
        /// OriginName: y, Units: m, IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Global Z position
        /// OriginName: z, Units: m, IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Roll angle in rad
        /// OriginName: roll, Units: rad, IsExtended: false
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Pitch angle in rad
        /// OriginName: pitch, Units: rad, IsExtended: false
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Yaw angle in rad
        /// OriginName: yaw, Units: rad, IsExtended: false
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// Pose covariance matrix upper right triangular (first six entries are the first ROW, next five entries are the second ROW, etc.)
        /// OriginName: covariance, Units: , IsExtended: true
        /// </summary>
        public float[] Covariance { get; } = new float[21];
    }
    /// <summary>
    /// The IMU readings in SI units in NED body frame
    ///  HIGHRES_IMU
    /// </summary>
    public class HighresImuPacket: PacketV2<HighresImuPayload>
    {
	public const int PacketMessageId = 105;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 93;

        public override HighresImuPayload Payload { get; } = new HighresImuPayload();

        public override string Name => "HIGHRES_IMU";
    }

    /// <summary>
    ///  HIGHRES_IMU
    /// </summary>
    public class HighresImuPayload : IPayload
    {
        public byte GetMaxByteSize() => 62;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Xacc = BitConverter.ToSingle(buffer, index);index+=4;
            Yacc = BitConverter.ToSingle(buffer, index);index+=4;
            Zacc = BitConverter.ToSingle(buffer, index);index+=4;
            Xgyro = BitConverter.ToSingle(buffer, index);index+=4;
            Ygyro = BitConverter.ToSingle(buffer, index);index+=4;
            Zgyro = BitConverter.ToSingle(buffer, index);index+=4;
            Xmag = BitConverter.ToSingle(buffer, index);index+=4;
            Ymag = BitConverter.ToSingle(buffer, index);index+=4;
            Zmag = BitConverter.ToSingle(buffer, index);index+=4;
            AbsPressure = BitConverter.ToSingle(buffer, index);index+=4;
            DiffPressure = BitConverter.ToSingle(buffer, index);index+=4;
            PressureAlt = BitConverter.ToSingle(buffer, index);index+=4;
            Temperature = BitConverter.ToSingle(buffer, index);index+=4;
            FieldsUpdated = BitConverter.ToUInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Xacc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yacc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Zacc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Xgyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Ygyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Zgyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Xmag).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Ymag).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Zmag).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(AbsPressure).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(DiffPressure).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PressureAlt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(FieldsUpdated).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/62;
        }

        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X acceleration (m/s^2)
        /// OriginName: xacc, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Xacc { get; set; }
        /// <summary>
        /// Y acceleration (m/s^2)
        /// OriginName: yacc, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Yacc { get; set; }
        /// <summary>
        /// Z acceleration (m/s^2)
        /// OriginName: zacc, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Zacc { get; set; }
        /// <summary>
        /// Angular speed around X axis (rad / sec)
        /// OriginName: xgyro, Units: rad/s, IsExtended: false
        /// </summary>
        public float Xgyro { get; set; }
        /// <summary>
        /// Angular speed around Y axis (rad / sec)
        /// OriginName: ygyro, Units: rad/s, IsExtended: false
        /// </summary>
        public float Ygyro { get; set; }
        /// <summary>
        /// Angular speed around Z axis (rad / sec)
        /// OriginName: zgyro, Units: rad/s, IsExtended: false
        /// </summary>
        public float Zgyro { get; set; }
        /// <summary>
        /// X Magnetic field (Gauss)
        /// OriginName: xmag, Units: gauss, IsExtended: false
        /// </summary>
        public float Xmag { get; set; }
        /// <summary>
        /// Y Magnetic field (Gauss)
        /// OriginName: ymag, Units: gauss, IsExtended: false
        /// </summary>
        public float Ymag { get; set; }
        /// <summary>
        /// Z Magnetic field (Gauss)
        /// OriginName: zmag, Units: gauss, IsExtended: false
        /// </summary>
        public float Zmag { get; set; }
        /// <summary>
        /// Absolute pressure in millibar
        /// OriginName: abs_pressure, Units: mbar, IsExtended: false
        /// </summary>
        public float AbsPressure { get; set; }
        /// <summary>
        /// Differential pressure in millibar
        /// OriginName: diff_pressure, Units: mbar, IsExtended: false
        /// </summary>
        public float DiffPressure { get; set; }
        /// <summary>
        /// Altitude calculated from pressure
        /// OriginName: pressure_alt, Units: , IsExtended: false
        /// </summary>
        public float PressureAlt { get; set; }
        /// <summary>
        /// Temperature in degrees celsius
        /// OriginName: temperature, Units: degC, IsExtended: false
        /// </summary>
        public float Temperature { get; set; }
        /// <summary>
        /// Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
        /// OriginName: fields_updated, Units: , IsExtended: false
        /// </summary>
        public ushort FieldsUpdated { get; set; }
    }
    /// <summary>
    /// Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
    ///  OPTICAL_FLOW_RAD
    /// </summary>
    public class OpticalFlowRadPacket: PacketV2<OpticalFlowRadPayload>
    {
	public const int PacketMessageId = 106;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 138;

        public override OpticalFlowRadPayload Payload { get; } = new OpticalFlowRadPayload();

        public override string Name => "OPTICAL_FLOW_RAD";
    }

    /// <summary>
    ///  OPTICAL_FLOW_RAD
    /// </summary>
    public class OpticalFlowRadPayload : IPayload
    {
        public byte GetMaxByteSize() => 44;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            IntegrationTimeUs = BitConverter.ToUInt32(buffer,index);index+=4;
            IntegratedX = BitConverter.ToSingle(buffer, index);index+=4;
            IntegratedY = BitConverter.ToSingle(buffer, index);index+=4;
            IntegratedXgyro = BitConverter.ToSingle(buffer, index);index+=4;
            IntegratedYgyro = BitConverter.ToSingle(buffer, index);index+=4;
            IntegratedZgyro = BitConverter.ToSingle(buffer, index);index+=4;
            TimeDeltaDistanceUs = BitConverter.ToUInt32(buffer,index);index+=4;
            Distance = BitConverter.ToSingle(buffer, index);index+=4;
            Temperature = BitConverter.ToInt16(buffer,index);index+=2;
            SensorId = (byte)buffer[index++];
            Quality = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(IntegrationTimeUs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(IntegratedX).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(IntegratedY).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(IntegratedXgyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(IntegratedYgyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(IntegratedZgyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TimeDeltaDistanceUs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Distance).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(SensorId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Quality).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/44;
        }

        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
        /// OriginName: integration_time_us, Units: us, IsExtended: false
        /// </summary>
        public uint IntegrationTimeUs { get; set; }
        /// <summary>
        /// Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
        /// OriginName: integrated_x, Units: rad, IsExtended: false
        /// </summary>
        public float IntegratedX { get; set; }
        /// <summary>
        /// Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
        /// OriginName: integrated_y, Units: rad, IsExtended: false
        /// </summary>
        public float IntegratedY { get; set; }
        /// <summary>
        /// RH rotation around X axis (rad)
        /// OriginName: integrated_xgyro, Units: rad, IsExtended: false
        /// </summary>
        public float IntegratedXgyro { get; set; }
        /// <summary>
        /// RH rotation around Y axis (rad)
        /// OriginName: integrated_ygyro, Units: rad, IsExtended: false
        /// </summary>
        public float IntegratedYgyro { get; set; }
        /// <summary>
        /// RH rotation around Z axis (rad)
        /// OriginName: integrated_zgyro, Units: rad, IsExtended: false
        /// </summary>
        public float IntegratedZgyro { get; set; }
        /// <summary>
        /// Time in microseconds since the distance was sampled.
        /// OriginName: time_delta_distance_us, Units: us, IsExtended: false
        /// </summary>
        public uint TimeDeltaDistanceUs { get; set; }
        /// <summary>
        /// Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
        /// OriginName: distance, Units: m, IsExtended: false
        /// </summary>
        public float Distance { get; set; }
        /// <summary>
        /// Temperature * 100 in centi-degrees Celsius
        /// OriginName: temperature, Units: cdegC, IsExtended: false
        /// </summary>
        public short Temperature { get; set; }
        /// <summary>
        /// Sensor ID
        /// OriginName: sensor_id, Units: , IsExtended: false
        /// </summary>
        public byte SensorId { get; set; }
        /// <summary>
        /// Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
        /// OriginName: quality, Units: , IsExtended: false
        /// </summary>
        public byte Quality { get; set; }
    }
    /// <summary>
    /// The IMU readings in SI units in NED body frame
    ///  HIL_SENSOR
    /// </summary>
    public class HilSensorPacket: PacketV2<HilSensorPayload>
    {
	public const int PacketMessageId = 107;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 108;

        public override HilSensorPayload Payload { get; } = new HilSensorPayload();

        public override string Name => "HIL_SENSOR";
    }

    /// <summary>
    ///  HIL_SENSOR
    /// </summary>
    public class HilSensorPayload : IPayload
    {
        public byte GetMaxByteSize() => 64;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Xacc = BitConverter.ToSingle(buffer, index);index+=4;
            Yacc = BitConverter.ToSingle(buffer, index);index+=4;
            Zacc = BitConverter.ToSingle(buffer, index);index+=4;
            Xgyro = BitConverter.ToSingle(buffer, index);index+=4;
            Ygyro = BitConverter.ToSingle(buffer, index);index+=4;
            Zgyro = BitConverter.ToSingle(buffer, index);index+=4;
            Xmag = BitConverter.ToSingle(buffer, index);index+=4;
            Ymag = BitConverter.ToSingle(buffer, index);index+=4;
            Zmag = BitConverter.ToSingle(buffer, index);index+=4;
            AbsPressure = BitConverter.ToSingle(buffer, index);index+=4;
            DiffPressure = BitConverter.ToSingle(buffer, index);index+=4;
            PressureAlt = BitConverter.ToSingle(buffer, index);index+=4;
            Temperature = BitConverter.ToSingle(buffer, index);index+=4;
            FieldsUpdated = BitConverter.ToUInt32(buffer,index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Xacc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yacc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Zacc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Xgyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Ygyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Zgyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Xmag).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Ymag).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Zmag).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(AbsPressure).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(DiffPressure).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PressureAlt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(FieldsUpdated).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/64;
        }

        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X acceleration (m/s^2)
        /// OriginName: xacc, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Xacc { get; set; }
        /// <summary>
        /// Y acceleration (m/s^2)
        /// OriginName: yacc, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Yacc { get; set; }
        /// <summary>
        /// Z acceleration (m/s^2)
        /// OriginName: zacc, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Zacc { get; set; }
        /// <summary>
        /// Angular speed around X axis in body frame (rad / sec)
        /// OriginName: xgyro, Units: rad/s, IsExtended: false
        /// </summary>
        public float Xgyro { get; set; }
        /// <summary>
        /// Angular speed around Y axis in body frame (rad / sec)
        /// OriginName: ygyro, Units: rad/s, IsExtended: false
        /// </summary>
        public float Ygyro { get; set; }
        /// <summary>
        /// Angular speed around Z axis in body frame (rad / sec)
        /// OriginName: zgyro, Units: rad/s, IsExtended: false
        /// </summary>
        public float Zgyro { get; set; }
        /// <summary>
        /// X Magnetic field (Gauss)
        /// OriginName: xmag, Units: gauss, IsExtended: false
        /// </summary>
        public float Xmag { get; set; }
        /// <summary>
        /// Y Magnetic field (Gauss)
        /// OriginName: ymag, Units: gauss, IsExtended: false
        /// </summary>
        public float Ymag { get; set; }
        /// <summary>
        /// Z Magnetic field (Gauss)
        /// OriginName: zmag, Units: gauss, IsExtended: false
        /// </summary>
        public float Zmag { get; set; }
        /// <summary>
        /// Absolute pressure in millibar
        /// OriginName: abs_pressure, Units: mbar, IsExtended: false
        /// </summary>
        public float AbsPressure { get; set; }
        /// <summary>
        /// Differential pressure (airspeed) in millibar
        /// OriginName: diff_pressure, Units: mbar, IsExtended: false
        /// </summary>
        public float DiffPressure { get; set; }
        /// <summary>
        /// Altitude calculated from pressure
        /// OriginName: pressure_alt, Units: , IsExtended: false
        /// </summary>
        public float PressureAlt { get; set; }
        /// <summary>
        /// Temperature in degrees celsius
        /// OriginName: temperature, Units: degC, IsExtended: false
        /// </summary>
        public float Temperature { get; set; }
        /// <summary>
        /// Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
        /// OriginName: fields_updated, Units: , IsExtended: false
        /// </summary>
        public uint FieldsUpdated { get; set; }
    }
    /// <summary>
    /// Status of simulation environment, if used
    ///  SIM_STATE
    /// </summary>
    public class SimStatePacket: PacketV2<SimStatePayload>
    {
	public const int PacketMessageId = 108;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 32;

        public override SimStatePayload Payload { get; } = new SimStatePayload();

        public override string Name => "SIM_STATE";
    }

    /// <summary>
    ///  SIM_STATE
    /// </summary>
    public class SimStatePayload : IPayload
    {
        public byte GetMaxByteSize() => 84;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Q1 = BitConverter.ToSingle(buffer, index);index+=4;
            Q2 = BitConverter.ToSingle(buffer, index);index+=4;
            Q3 = BitConverter.ToSingle(buffer, index);index+=4;
            Q4 = BitConverter.ToSingle(buffer, index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            Xacc = BitConverter.ToSingle(buffer, index);index+=4;
            Yacc = BitConverter.ToSingle(buffer, index);index+=4;
            Zacc = BitConverter.ToSingle(buffer, index);index+=4;
            Xgyro = BitConverter.ToSingle(buffer, index);index+=4;
            Ygyro = BitConverter.ToSingle(buffer, index);index+=4;
            Zgyro = BitConverter.ToSingle(buffer, index);index+=4;
            Lat = BitConverter.ToSingle(buffer, index);index+=4;
            Lon = BitConverter.ToSingle(buffer, index);index+=4;
            Alt = BitConverter.ToSingle(buffer, index);index+=4;
            StdDevHorz = BitConverter.ToSingle(buffer, index);index+=4;
            StdDevVert = BitConverter.ToSingle(buffer, index);index+=4;
            Vn = BitConverter.ToSingle(buffer, index);index+=4;
            Ve = BitConverter.ToSingle(buffer, index);index+=4;
            Vd = BitConverter.ToSingle(buffer, index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Q1).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Q2).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Q3).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Q4).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Roll).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitch).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yaw).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Xacc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yacc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Zacc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Xgyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Ygyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Zgyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(StdDevHorz).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(StdDevVert).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vn).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Ve).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vd).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/84;
        }

        /// <summary>
        /// True attitude quaternion component 1, w (1 in null-rotation)
        /// OriginName: q1, Units: , IsExtended: false
        /// </summary>
        public float Q1 { get; set; }
        /// <summary>
        /// True attitude quaternion component 2, x (0 in null-rotation)
        /// OriginName: q2, Units: , IsExtended: false
        /// </summary>
        public float Q2 { get; set; }
        /// <summary>
        /// True attitude quaternion component 3, y (0 in null-rotation)
        /// OriginName: q3, Units: , IsExtended: false
        /// </summary>
        public float Q3 { get; set; }
        /// <summary>
        /// True attitude quaternion component 4, z (0 in null-rotation)
        /// OriginName: q4, Units: , IsExtended: false
        /// </summary>
        public float Q4 { get; set; }
        /// <summary>
        /// Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
        /// OriginName: roll, Units: , IsExtended: false
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
        /// OriginName: pitch, Units: , IsExtended: false
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
        /// OriginName: yaw, Units: , IsExtended: false
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// X acceleration m/s/s
        /// OriginName: xacc, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Xacc { get; set; }
        /// <summary>
        /// Y acceleration m/s/s
        /// OriginName: yacc, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Yacc { get; set; }
        /// <summary>
        /// Z acceleration m/s/s
        /// OriginName: zacc, Units: m/s/s, IsExtended: false
        /// </summary>
        public float Zacc { get; set; }
        /// <summary>
        /// Angular speed around X axis rad/s
        /// OriginName: xgyro, Units: rad/s, IsExtended: false
        /// </summary>
        public float Xgyro { get; set; }
        /// <summary>
        /// Angular speed around Y axis rad/s
        /// OriginName: ygyro, Units: rad/s, IsExtended: false
        /// </summary>
        public float Ygyro { get; set; }
        /// <summary>
        /// Angular speed around Z axis rad/s
        /// OriginName: zgyro, Units: rad/s, IsExtended: false
        /// </summary>
        public float Zgyro { get; set; }
        /// <summary>
        /// Latitude in degrees
        /// OriginName: lat, Units: deg, IsExtended: false
        /// </summary>
        public float Lat { get; set; }
        /// <summary>
        /// Longitude in degrees
        /// OriginName: lon, Units: deg, IsExtended: false
        /// </summary>
        public float Lon { get; set; }
        /// <summary>
        /// Altitude in meters
        /// OriginName: alt, Units: m, IsExtended: false
        /// </summary>
        public float Alt { get; set; }
        /// <summary>
        /// Horizontal position standard deviation
        /// OriginName: std_dev_horz, Units: , IsExtended: false
        /// </summary>
        public float StdDevHorz { get; set; }
        /// <summary>
        /// Vertical position standard deviation
        /// OriginName: std_dev_vert, Units: , IsExtended: false
        /// </summary>
        public float StdDevVert { get; set; }
        /// <summary>
        /// True velocity in m/s in NORTH direction in earth-fixed NED frame
        /// OriginName: vn, Units: m/s, IsExtended: false
        /// </summary>
        public float Vn { get; set; }
        /// <summary>
        /// True velocity in m/s in EAST direction in earth-fixed NED frame
        /// OriginName: ve, Units: m/s, IsExtended: false
        /// </summary>
        public float Ve { get; set; }
        /// <summary>
        /// True velocity in m/s in DOWN direction in earth-fixed NED frame
        /// OriginName: vd, Units: m/s, IsExtended: false
        /// </summary>
        public float Vd { get; set; }
    }
    /// <summary>
    /// Status generated by radio and injected into MAVLink stream.
    ///  RADIO_STATUS
    /// </summary>
    public class RadioStatusPacket: PacketV2<RadioStatusPayload>
    {
	public const int PacketMessageId = 109;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 185;

        public override RadioStatusPayload Payload { get; } = new RadioStatusPayload();

        public override string Name => "RADIO_STATUS";
    }

    /// <summary>
    ///  RADIO_STATUS
    /// </summary>
    public class RadioStatusPayload : IPayload
    {
        public byte GetMaxByteSize() => 9;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Rxerrors = BitConverter.ToUInt16(buffer,index);index+=2;
            Fixed = BitConverter.ToUInt16(buffer,index);index+=2;
            Rssi = (byte)buffer[index++];
            Remrssi = (byte)buffer[index++];
            Txbuf = (byte)buffer[index++];
            Noise = (byte)buffer[index++];
            Remnoise = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Rxerrors).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Fixed).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Rssi).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Remrssi).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Txbuf).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Noise).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Remnoise).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/9;
        }

        /// <summary>
        /// Receive errors
        /// OriginName: rxerrors, Units: , IsExtended: false
        /// </summary>
        public ushort Rxerrors { get; set; }
        /// <summary>
        /// Count of error corrected packets
        /// OriginName: fixed, Units: , IsExtended: false
        /// </summary>
        public ushort Fixed { get; set; }
        /// <summary>
        /// Local signal strength
        /// OriginName: rssi, Units: , IsExtended: false
        /// </summary>
        public byte Rssi { get; set; }
        /// <summary>
        /// Remote signal strength
        /// OriginName: remrssi, Units: , IsExtended: false
        /// </summary>
        public byte Remrssi { get; set; }
        /// <summary>
        /// Remaining free buffer space in percent.
        /// OriginName: txbuf, Units: %, IsExtended: false
        /// </summary>
        public byte Txbuf { get; set; }
        /// <summary>
        /// Background noise level
        /// OriginName: noise, Units: , IsExtended: false
        /// </summary>
        public byte Noise { get; set; }
        /// <summary>
        /// Remote background noise level
        /// OriginName: remnoise, Units: , IsExtended: false
        /// </summary>
        public byte Remnoise { get; set; }
    }
    /// <summary>
    /// File transfer message
    ///  FILE_TRANSFER_PROTOCOL
    /// </summary>
    public class FileTransferProtocolPacket: PacketV2<FileTransferProtocolPayload>
    {
	public const int PacketMessageId = 110;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 84;

        public override FileTransferProtocolPayload Payload { get; } = new FileTransferProtocolPayload();

        public override string Name => "FILE_TRANSFER_PROTOCOL";
    }

    /// <summary>
    ///  FILE_TRANSFER_PROTOCOL
    /// </summary>
    public class FileTransferProtocolPayload : IPayload
    {
        public byte GetMaxByteSize() => 254;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TargetNetwork = (byte)buffer[index++];
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            arraySize = /*ArrayLength*/251 - Math.Max(0,((/*PayloadByteSize*/254 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<251;i++)
            {
                Payload[i] = default(byte);
            }
            for(var i=0;i<arraySize;i++)
            {
                Payload[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetNetwork).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            for(var i=0;i<251;i++)
            {
                buffer[index] = (byte)Payload[i];index+=1;
            }
            return /*PayloadByteSize*/254;
        }

        /// <summary>
        /// Network ID (0 for broadcast)
        /// OriginName: target_network, Units: , IsExtended: false
        /// </summary>
        public byte TargetNetwork { get; set; }
        /// <summary>
        /// System ID (0 for broadcast)
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID (0 for broadcast)
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
        /// OriginName: payload, Units: , IsExtended: false
        /// </summary>
        public byte[] Payload { get; } = new byte[251];
    }
    /// <summary>
    /// Time synchronization message.
    ///  TIMESYNC
    /// </summary>
    public class TimesyncPacket: PacketV2<TimesyncPayload>
    {
	public const int PacketMessageId = 111;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 34;

        public override TimesyncPayload Payload { get; } = new TimesyncPayload();

        public override string Name => "TIMESYNC";
    }

    /// <summary>
    ///  TIMESYNC
    /// </summary>
    public class TimesyncPayload : IPayload
    {
        public byte GetMaxByteSize() => 16;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Tc1 = BitConverter.ToInt64(buffer,index);index+=8;
            Ts1 = BitConverter.ToInt64(buffer,index);index+=8;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Tc1).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Ts1).CopyTo(buffer, index);index+=8;
            return /*PayloadByteSize*/16;
        }

        /// <summary>
        /// Time sync timestamp 1
        /// OriginName: tc1, Units: , IsExtended: false
        /// </summary>
        public long Tc1 { get; set; }
        /// <summary>
        /// Time sync timestamp 2
        /// OriginName: ts1, Units: , IsExtended: false
        /// </summary>
        public long Ts1 { get; set; }
    }
    /// <summary>
    /// Camera-IMU triggering and synchronisation message.
    ///  CAMERA_TRIGGER
    /// </summary>
    public class CameraTriggerPacket: PacketV2<CameraTriggerPayload>
    {
	public const int PacketMessageId = 112;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 174;

        public override CameraTriggerPayload Payload { get; } = new CameraTriggerPayload();

        public override string Name => "CAMERA_TRIGGER";
    }

    /// <summary>
    ///  CAMERA_TRIGGER
    /// </summary>
    public class CameraTriggerPayload : IPayload
    {
        public byte GetMaxByteSize() => 12;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Seq = BitConverter.ToUInt32(buffer,index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/12;
        }

        /// <summary>
        /// Timestamp for the image frame in microseconds
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Image frame sequence
        /// OriginName: seq, Units: , IsExtended: false
        /// </summary>
        public uint Seq { get; set; }
    }
    /// <summary>
    /// The global position, as returned by the Global Positioning System (GPS). This is
    ///                  NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate.
    ///  HIL_GPS
    /// </summary>
    public class HilGpsPacket: PacketV2<HilGpsPayload>
    {
	public const int PacketMessageId = 113;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 124;

        public override HilGpsPayload Payload { get; } = new HilGpsPayload();

        public override string Name => "HIL_GPS";
    }

    /// <summary>
    ///  HIL_GPS
    /// </summary>
    public class HilGpsPayload : IPayload
    {
        public byte GetMaxByteSize() => 36;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToInt32(buffer,index);index+=4;
            Eph = BitConverter.ToUInt16(buffer,index);index+=2;
            Epv = BitConverter.ToUInt16(buffer,index);index+=2;
            Vel = BitConverter.ToUInt16(buffer,index);index+=2;
            Vn = BitConverter.ToInt16(buffer,index);index+=2;
            Ve = BitConverter.ToInt16(buffer,index);index+=2;
            Vd = BitConverter.ToInt16(buffer,index);index+=2;
            Cog = BitConverter.ToUInt16(buffer,index);index+=2;
            FixType = (byte)buffer[index++];
            SatellitesVisible = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Eph).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Epv).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Vel).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Vn).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Ve).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Vd).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Cog).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(FixType).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(SatellitesVisible).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/36;
        }

        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Latitude (WGS84), in degrees * 1E7
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude (WGS84), in degrees * 1E7
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
        /// OriginName: alt, Units: mm, IsExtended: false
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
        /// OriginName: eph, Units: , IsExtended: false
        /// </summary>
        public ushort Eph { get; set; }
        /// <summary>
        /// GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
        /// OriginName: epv, Units: , IsExtended: false
        /// </summary>
        public ushort Epv { get; set; }
        /// <summary>
        /// GPS ground speed in cm/s. If unknown, set to: 65535
        /// OriginName: vel, Units: cm/s, IsExtended: false
        /// </summary>
        public ushort Vel { get; set; }
        /// <summary>
        /// GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
        /// OriginName: vn, Units: cm/s, IsExtended: false
        /// </summary>
        public short Vn { get; set; }
        /// <summary>
        /// GPS velocity in cm/s in EAST direction in earth-fixed NED frame
        /// OriginName: ve, Units: cm/s, IsExtended: false
        /// </summary>
        public short Ve { get; set; }
        /// <summary>
        /// GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
        /// OriginName: vd, Units: cm/s, IsExtended: false
        /// </summary>
        public short Vd { get; set; }
        /// <summary>
        /// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
        /// OriginName: cog, Units: cdeg, IsExtended: false
        /// </summary>
        public ushort Cog { get; set; }
        /// <summary>
        /// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
        /// OriginName: fix_type, Units: , IsExtended: false
        /// </summary>
        public byte FixType { get; set; }
        /// <summary>
        /// Number of satellites visible. If unknown, set to 255
        /// OriginName: satellites_visible, Units: , IsExtended: false
        /// </summary>
        public byte SatellitesVisible { get; set; }
    }
    /// <summary>
    /// Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
    ///  HIL_OPTICAL_FLOW
    /// </summary>
    public class HilOpticalFlowPacket: PacketV2<HilOpticalFlowPayload>
    {
	public const int PacketMessageId = 114;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 237;

        public override HilOpticalFlowPayload Payload { get; } = new HilOpticalFlowPayload();

        public override string Name => "HIL_OPTICAL_FLOW";
    }

    /// <summary>
    ///  HIL_OPTICAL_FLOW
    /// </summary>
    public class HilOpticalFlowPayload : IPayload
    {
        public byte GetMaxByteSize() => 44;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            IntegrationTimeUs = BitConverter.ToUInt32(buffer,index);index+=4;
            IntegratedX = BitConverter.ToSingle(buffer, index);index+=4;
            IntegratedY = BitConverter.ToSingle(buffer, index);index+=4;
            IntegratedXgyro = BitConverter.ToSingle(buffer, index);index+=4;
            IntegratedYgyro = BitConverter.ToSingle(buffer, index);index+=4;
            IntegratedZgyro = BitConverter.ToSingle(buffer, index);index+=4;
            TimeDeltaDistanceUs = BitConverter.ToUInt32(buffer,index);index+=4;
            Distance = BitConverter.ToSingle(buffer, index);index+=4;
            Temperature = BitConverter.ToInt16(buffer,index);index+=2;
            SensorId = (byte)buffer[index++];
            Quality = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(IntegrationTimeUs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(IntegratedX).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(IntegratedY).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(IntegratedXgyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(IntegratedYgyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(IntegratedZgyro).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TimeDeltaDistanceUs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Distance).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(SensorId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Quality).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/44;
        }

        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
        /// OriginName: integration_time_us, Units: us, IsExtended: false
        /// </summary>
        public uint IntegrationTimeUs { get; set; }
        /// <summary>
        /// Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
        /// OriginName: integrated_x, Units: rad, IsExtended: false
        /// </summary>
        public float IntegratedX { get; set; }
        /// <summary>
        /// Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
        /// OriginName: integrated_y, Units: rad, IsExtended: false
        /// </summary>
        public float IntegratedY { get; set; }
        /// <summary>
        /// RH rotation around X axis (rad)
        /// OriginName: integrated_xgyro, Units: rad, IsExtended: false
        /// </summary>
        public float IntegratedXgyro { get; set; }
        /// <summary>
        /// RH rotation around Y axis (rad)
        /// OriginName: integrated_ygyro, Units: rad, IsExtended: false
        /// </summary>
        public float IntegratedYgyro { get; set; }
        /// <summary>
        /// RH rotation around Z axis (rad)
        /// OriginName: integrated_zgyro, Units: rad, IsExtended: false
        /// </summary>
        public float IntegratedZgyro { get; set; }
        /// <summary>
        /// Time in microseconds since the distance was sampled.
        /// OriginName: time_delta_distance_us, Units: us, IsExtended: false
        /// </summary>
        public uint TimeDeltaDistanceUs { get; set; }
        /// <summary>
        /// Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
        /// OriginName: distance, Units: m, IsExtended: false
        /// </summary>
        public float Distance { get; set; }
        /// <summary>
        /// Temperature * 100 in centi-degrees Celsius
        /// OriginName: temperature, Units: cdegC, IsExtended: false
        /// </summary>
        public short Temperature { get; set; }
        /// <summary>
        /// Sensor ID
        /// OriginName: sensor_id, Units: , IsExtended: false
        /// </summary>
        public byte SensorId { get; set; }
        /// <summary>
        /// Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
        /// OriginName: quality, Units: , IsExtended: false
        /// </summary>
        public byte Quality { get; set; }
    }
    /// <summary>
    /// Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.
    ///  HIL_STATE_QUATERNION
    /// </summary>
    public class HilStateQuaternionPacket: PacketV2<HilStateQuaternionPayload>
    {
	public const int PacketMessageId = 115;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 4;

        public override HilStateQuaternionPayload Payload { get; } = new HilStateQuaternionPayload();

        public override string Name => "HIL_STATE_QUATERNION";
    }

    /// <summary>
    ///  HIL_STATE_QUATERNION
    /// </summary>
    public class HilStateQuaternionPayload : IPayload
    {
        public byte GetMaxByteSize() => 64;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            arraySize = /*ArrayLength*/4 - Math.Max(0,((/*PayloadByteSize*/64 - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<4;i++)
            {
                AttitudeQuaternion[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                AttitudeQuaternion[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            Rollspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Pitchspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Yawspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToInt32(buffer,index);index+=4;
            Vx = BitConverter.ToInt16(buffer,index);index+=2;
            Vy = BitConverter.ToInt16(buffer,index);index+=2;
            Vz = BitConverter.ToInt16(buffer,index);index+=2;
            IndAirspeed = BitConverter.ToUInt16(buffer,index);index+=2;
            TrueAirspeed = BitConverter.ToUInt16(buffer,index);index+=2;
            Xacc = BitConverter.ToInt16(buffer,index);index+=2;
            Yacc = BitConverter.ToInt16(buffer,index);index+=2;
            Zacc = BitConverter.ToInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(AttitudeQuaternion[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(Rollspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitchspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yawspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vx).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Vy).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Vz).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(IndAirspeed).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TrueAirspeed).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Xacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Yacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Zacc).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/64;
        }

        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
        /// OriginName: attitude_quaternion, Units: , IsExtended: false
        /// </summary>
        public float[] AttitudeQuaternion { get; } = new float[4];
        /// <summary>
        /// Body frame roll / phi angular speed (rad/s)
        /// OriginName: rollspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Rollspeed { get; set; }
        /// <summary>
        /// Body frame pitch / theta angular speed (rad/s)
        /// OriginName: pitchspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Pitchspeed { get; set; }
        /// <summary>
        /// Body frame yaw / psi angular speed (rad/s)
        /// OriginName: yawspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Yawspeed { get; set; }
        /// <summary>
        /// Latitude, expressed as degrees * 1E7
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude, expressed as degrees * 1E7
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude in meters, expressed as * 1000 (millimeters)
        /// OriginName: alt, Units: mm, IsExtended: false
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// Ground X Speed (Latitude), expressed as cm/s
        /// OriginName: vx, Units: cm/s, IsExtended: false
        /// </summary>
        public short Vx { get; set; }
        /// <summary>
        /// Ground Y Speed (Longitude), expressed as cm/s
        /// OriginName: vy, Units: cm/s, IsExtended: false
        /// </summary>
        public short Vy { get; set; }
        /// <summary>
        /// Ground Z Speed (Altitude), expressed as cm/s
        /// OriginName: vz, Units: cm/s, IsExtended: false
        /// </summary>
        public short Vz { get; set; }
        /// <summary>
        /// Indicated airspeed, expressed as cm/s
        /// OriginName: ind_airspeed, Units: cm/s, IsExtended: false
        /// </summary>
        public ushort IndAirspeed { get; set; }
        /// <summary>
        /// True airspeed, expressed as cm/s
        /// OriginName: true_airspeed, Units: cm/s, IsExtended: false
        /// </summary>
        public ushort TrueAirspeed { get; set; }
        /// <summary>
        /// X acceleration (mg)
        /// OriginName: xacc, Units: mG, IsExtended: false
        /// </summary>
        public short Xacc { get; set; }
        /// <summary>
        /// Y acceleration (mg)
        /// OriginName: yacc, Units: mG, IsExtended: false
        /// </summary>
        public short Yacc { get; set; }
        /// <summary>
        /// Z acceleration (mg)
        /// OriginName: zacc, Units: mG, IsExtended: false
        /// </summary>
        public short Zacc { get; set; }
    }
    /// <summary>
    /// The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units
    ///  SCALED_IMU2
    /// </summary>
    public class ScaledImu2Packet: PacketV2<ScaledImu2Payload>
    {
	public const int PacketMessageId = 116;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 76;

        public override ScaledImu2Payload Payload { get; } = new ScaledImu2Payload();

        public override string Name => "SCALED_IMU2";
    }

    /// <summary>
    ///  SCALED_IMU2
    /// </summary>
    public class ScaledImu2Payload : IPayload
    {
        public byte GetMaxByteSize() => 22;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Xacc = BitConverter.ToInt16(buffer,index);index+=2;
            Yacc = BitConverter.ToInt16(buffer,index);index+=2;
            Zacc = BitConverter.ToInt16(buffer,index);index+=2;
            Xgyro = BitConverter.ToInt16(buffer,index);index+=2;
            Ygyro = BitConverter.ToInt16(buffer,index);index+=2;
            Zgyro = BitConverter.ToInt16(buffer,index);index+=2;
            Xmag = BitConverter.ToInt16(buffer,index);index+=2;
            Ymag = BitConverter.ToInt16(buffer,index);index+=2;
            Zmag = BitConverter.ToInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Xacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Yacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Zacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Xgyro).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Ygyro).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Zgyro).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Xmag).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Ymag).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Zmag).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/22;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X acceleration (mg)
        /// OriginName: xacc, Units: mG, IsExtended: false
        /// </summary>
        public short Xacc { get; set; }
        /// <summary>
        /// Y acceleration (mg)
        /// OriginName: yacc, Units: mG, IsExtended: false
        /// </summary>
        public short Yacc { get; set; }
        /// <summary>
        /// Z acceleration (mg)
        /// OriginName: zacc, Units: mG, IsExtended: false
        /// </summary>
        public short Zacc { get; set; }
        /// <summary>
        /// Angular speed around X axis (millirad /sec)
        /// OriginName: xgyro, Units: mrad/s, IsExtended: false
        /// </summary>
        public short Xgyro { get; set; }
        /// <summary>
        /// Angular speed around Y axis (millirad /sec)
        /// OriginName: ygyro, Units: mrad/s, IsExtended: false
        /// </summary>
        public short Ygyro { get; set; }
        /// <summary>
        /// Angular speed around Z axis (millirad /sec)
        /// OriginName: zgyro, Units: mrad/s, IsExtended: false
        /// </summary>
        public short Zgyro { get; set; }
        /// <summary>
        /// X Magnetic field (milli tesla)
        /// OriginName: xmag, Units: mT, IsExtended: false
        /// </summary>
        public short Xmag { get; set; }
        /// <summary>
        /// Y Magnetic field (milli tesla)
        /// OriginName: ymag, Units: mT, IsExtended: false
        /// </summary>
        public short Ymag { get; set; }
        /// <summary>
        /// Z Magnetic field (milli tesla)
        /// OriginName: zmag, Units: mT, IsExtended: false
        /// </summary>
        public short Zmag { get; set; }
    }
    /// <summary>
    /// Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END is called.
    ///  LOG_REQUEST_LIST
    /// </summary>
    public class LogRequestListPacket: PacketV2<LogRequestListPayload>
    {
	public const int PacketMessageId = 117;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 128;

        public override LogRequestListPayload Payload { get; } = new LogRequestListPayload();

        public override string Name => "LOG_REQUEST_LIST";
    }

    /// <summary>
    ///  LOG_REQUEST_LIST
    /// </summary>
    public class LogRequestListPayload : IPayload
    {
        public byte GetMaxByteSize() => 6;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Start = BitConverter.ToUInt16(buffer,index);index+=2;
            End = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Start).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(End).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/6;
        }

        /// <summary>
        /// First log id (0 for first available)
        /// OriginName: start, Units: , IsExtended: false
        /// </summary>
        public ushort Start { get; set; }
        /// <summary>
        /// Last log id (0xffff for last available)
        /// OriginName: end, Units: , IsExtended: false
        /// </summary>
        public ushort End { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Reply to LOG_REQUEST_LIST
    ///  LOG_ENTRY
    /// </summary>
    public class LogEntryPacket: PacketV2<LogEntryPayload>
    {
	public const int PacketMessageId = 118;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 56;

        public override LogEntryPayload Payload { get; } = new LogEntryPayload();

        public override string Name => "LOG_ENTRY";
    }

    /// <summary>
    ///  LOG_ENTRY
    /// </summary>
    public class LogEntryPayload : IPayload
    {
        public byte GetMaxByteSize() => 14;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUtc = BitConverter.ToUInt32(buffer,index);index+=4;
            Size = BitConverter.ToUInt32(buffer,index);index+=4;
            Id = BitConverter.ToUInt16(buffer,index);index+=2;
            NumLogs = BitConverter.ToUInt16(buffer,index);index+=2;
            LastLogNum = BitConverter.ToUInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUtc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Size).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Id).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(NumLogs).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(LastLogNum).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/14;
        }

        /// <summary>
        /// UTC timestamp of log in seconds since 1970, or 0 if not available
        /// OriginName: time_utc, Units: s, IsExtended: false
        /// </summary>
        public uint TimeUtc { get; set; }
        /// <summary>
        /// Size of the log (may be approximate) in bytes
        /// OriginName: size, Units: bytes, IsExtended: false
        /// </summary>
        public uint Size { get; set; }
        /// <summary>
        /// Log id
        /// OriginName: id, Units: , IsExtended: false
        /// </summary>
        public ushort Id { get; set; }
        /// <summary>
        /// Total number of logs
        /// OriginName: num_logs, Units: , IsExtended: false
        /// </summary>
        public ushort NumLogs { get; set; }
        /// <summary>
        /// High log number
        /// OriginName: last_log_num, Units: , IsExtended: false
        /// </summary>
        public ushort LastLogNum { get; set; }
    }
    /// <summary>
    /// Request a chunk of a log
    ///  LOG_REQUEST_DATA
    /// </summary>
    public class LogRequestDataPacket: PacketV2<LogRequestDataPayload>
    {
	public const int PacketMessageId = 119;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 116;

        public override LogRequestDataPayload Payload { get; } = new LogRequestDataPayload();

        public override string Name => "LOG_REQUEST_DATA";
    }

    /// <summary>
    ///  LOG_REQUEST_DATA
    /// </summary>
    public class LogRequestDataPayload : IPayload
    {
        public byte GetMaxByteSize() => 12;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Ofs = BitConverter.ToUInt32(buffer,index);index+=4;
            Count = BitConverter.ToUInt32(buffer,index);index+=4;
            Id = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Ofs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Count).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Id).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/12;
        }

        /// <summary>
        /// Offset into the log
        /// OriginName: ofs, Units: , IsExtended: false
        /// </summary>
        public uint Ofs { get; set; }
        /// <summary>
        /// Number of bytes
        /// OriginName: count, Units: bytes, IsExtended: false
        /// </summary>
        public uint Count { get; set; }
        /// <summary>
        /// Log id (from LOG_ENTRY reply)
        /// OriginName: id, Units: , IsExtended: false
        /// </summary>
        public ushort Id { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Reply to LOG_REQUEST_DATA
    ///  LOG_DATA
    /// </summary>
    public class LogDataPacket: PacketV2<LogDataPayload>
    {
	public const int PacketMessageId = 120;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 134;

        public override LogDataPayload Payload { get; } = new LogDataPayload();

        public override string Name => "LOG_DATA";
    }

    /// <summary>
    ///  LOG_DATA
    /// </summary>
    public class LogDataPayload : IPayload
    {
        public byte GetMaxByteSize() => 97;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Ofs = BitConverter.ToUInt32(buffer,index);index+=4;
            Id = BitConverter.ToUInt16(buffer,index);index+=2;
            Count = (byte)buffer[index++];
            arraySize = /*ArrayLength*/90 - Math.Max(0,((/*PayloadByteSize*/97 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<90;i++)
            {
                Data[i] = default(byte);
            }
            for(var i=0;i<arraySize;i++)
            {
                Data[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Ofs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Id).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Count).CopyTo(buffer, index);index+=1;
            for(var i=0;i<90;i++)
            {
                buffer[index] = (byte)Data[i];index+=1;
            }
            return /*PayloadByteSize*/97;
        }

        /// <summary>
        /// Offset into the log
        /// OriginName: ofs, Units: , IsExtended: false
        /// </summary>
        public uint Ofs { get; set; }
        /// <summary>
        /// Log id (from LOG_ENTRY reply)
        /// OriginName: id, Units: , IsExtended: false
        /// </summary>
        public ushort Id { get; set; }
        /// <summary>
        /// Number of bytes (zero for end of log)
        /// OriginName: count, Units: bytes, IsExtended: false
        /// </summary>
        public byte Count { get; set; }
        /// <summary>
        /// log data
        /// OriginName: data, Units: , IsExtended: false
        /// </summary>
        public byte[] Data { get; } = new byte[90];
    }
    /// <summary>
    /// Erase all logs
    ///  LOG_ERASE
    /// </summary>
    public class LogErasePacket: PacketV2<LogErasePayload>
    {
	public const int PacketMessageId = 121;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 237;

        public override LogErasePayload Payload { get; } = new LogErasePayload();

        public override string Name => "LOG_ERASE";
    }

    /// <summary>
    ///  LOG_ERASE
    /// </summary>
    public class LogErasePayload : IPayload
    {
        public byte GetMaxByteSize() => 2;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/2;
        }

        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Stop log transfer and resume normal logging
    ///  LOG_REQUEST_END
    /// </summary>
    public class LogRequestEndPacket: PacketV2<LogRequestEndPayload>
    {
	public const int PacketMessageId = 122;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 203;

        public override LogRequestEndPayload Payload { get; } = new LogRequestEndPayload();

        public override string Name => "LOG_REQUEST_END";
    }

    /// <summary>
    ///  LOG_REQUEST_END
    /// </summary>
    public class LogRequestEndPayload : IPayload
    {
        public byte GetMaxByteSize() => 2;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/2;
        }

        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// data for injecting into the onboard GPS (used for DGPS)
    ///  GPS_INJECT_DATA
    /// </summary>
    public class GpsInjectDataPacket: PacketV2<GpsInjectDataPayload>
    {
	public const int PacketMessageId = 123;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 250;

        public override GpsInjectDataPayload Payload { get; } = new GpsInjectDataPayload();

        public override string Name => "GPS_INJECT_DATA";
    }

    /// <summary>
    ///  GPS_INJECT_DATA
    /// </summary>
    public class GpsInjectDataPayload : IPayload
    {
        public byte GetMaxByteSize() => 113;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            Len = (byte)buffer[index++];
            arraySize = /*ArrayLength*/110 - Math.Max(0,((/*PayloadByteSize*/113 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<110;i++)
            {
                Data[i] = default(byte);
            }
            for(var i=0;i<arraySize;i++)
            {
                Data[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Len).CopyTo(buffer, index);index+=1;
            for(var i=0;i<110;i++)
            {
                buffer[index] = (byte)Data[i];index+=1;
            }
            return /*PayloadByteSize*/113;
        }

        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// data length
        /// OriginName: len, Units: bytes, IsExtended: false
        /// </summary>
        public byte Len { get; set; }
        /// <summary>
        /// raw data (110 is enough for 12 satellites of RTCMv2)
        /// OriginName: data, Units: , IsExtended: false
        /// </summary>
        public byte[] Data { get; } = new byte[110];
    }
    /// <summary>
    /// Second GPS data.
    ///  GPS2_RAW
    /// </summary>
    public class Gps2RawPacket: PacketV2<Gps2RawPayload>
    {
	public const int PacketMessageId = 124;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 87;

        public override Gps2RawPayload Payload { get; } = new Gps2RawPayload();

        public override string Name => "GPS2_RAW";
    }

    /// <summary>
    ///  GPS2_RAW
    /// </summary>
    public class Gps2RawPayload : IPayload
    {
        public byte GetMaxByteSize() => 35;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToInt32(buffer,index);index+=4;
            DgpsAge = BitConverter.ToUInt32(buffer,index);index+=4;
            Eph = BitConverter.ToUInt16(buffer,index);index+=2;
            Epv = BitConverter.ToUInt16(buffer,index);index+=2;
            Vel = BitConverter.ToUInt16(buffer,index);index+=2;
            Cog = BitConverter.ToUInt16(buffer,index);index+=2;
            FixType = (GpsFixType)buffer[index++];
            SatellitesVisible = (byte)buffer[index++];
            DgpsNumch = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(DgpsAge).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Eph).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Epv).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Vel).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Cog).CopyTo(buffer, index);index+=2;
            buffer[index] = (byte)FixType;index+=1;
            BitConverter.GetBytes(SatellitesVisible).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(DgpsNumch).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/35;
        }

        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Latitude (WGS84), in degrees * 1E7
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude (WGS84), in degrees * 1E7
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
        /// OriginName: alt, Units: mm, IsExtended: false
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// Age of DGPS info
        /// OriginName: dgps_age, Units: ms, IsExtended: false
        /// </summary>
        public uint DgpsAge { get; set; }
        /// <summary>
        /// GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
        /// OriginName: eph, Units: cm, IsExtended: false
        /// </summary>
        public ushort Eph { get; set; }
        /// <summary>
        /// GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
        /// OriginName: epv, Units: cm, IsExtended: false
        /// </summary>
        public ushort Epv { get; set; }
        /// <summary>
        /// GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
        /// OriginName: vel, Units: cm/s, IsExtended: false
        /// </summary>
        public ushort Vel { get; set; }
        /// <summary>
        /// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        /// OriginName: cog, Units: cdeg, IsExtended: false
        /// </summary>
        public ushort Cog { get; set; }
        /// <summary>
        /// See the GPS_FIX_TYPE enum.
        /// OriginName: fix_type, Units: , IsExtended: false
        /// </summary>
        public GpsFixType FixType { get; set; }
        /// <summary>
        /// Number of satellites visible. If unknown, set to 255
        /// OriginName: satellites_visible, Units: , IsExtended: false
        /// </summary>
        public byte SatellitesVisible { get; set; }
        /// <summary>
        /// Number of DGPS satellites
        /// OriginName: dgps_numch, Units: , IsExtended: false
        /// </summary>
        public byte DgpsNumch { get; set; }
    }
    /// <summary>
    /// Power supply status
    ///  POWER_STATUS
    /// </summary>
    public class PowerStatusPacket: PacketV2<PowerStatusPayload>
    {
	public const int PacketMessageId = 125;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 203;

        public override PowerStatusPayload Payload { get; } = new PowerStatusPayload();

        public override string Name => "POWER_STATUS";
    }

    /// <summary>
    ///  POWER_STATUS
    /// </summary>
    public class PowerStatusPayload : IPayload
    {
        public byte GetMaxByteSize() => 6;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Vcc = BitConverter.ToUInt16(buffer,index);index+=2;
            Vservo = BitConverter.ToUInt16(buffer,index);index+=2;
            Flags = (MavPowerStatus)BitConverter.ToUInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Vcc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Vservo).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes((ushort)Flags).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/6;
        }

        /// <summary>
        /// 5V rail voltage in millivolts
        /// OriginName: Vcc, Units: mV, IsExtended: false
        /// </summary>
        public ushort Vcc { get; set; }
        /// <summary>
        /// servo rail voltage in millivolts
        /// OriginName: Vservo, Units: mV, IsExtended: false
        /// </summary>
        public ushort Vservo { get; set; }
        /// <summary>
        /// power supply status flags (see MAV_POWER_STATUS enum)
        /// OriginName: flags, Units: , IsExtended: false
        /// </summary>
        public MavPowerStatus Flags { get; set; }
    }
    /// <summary>
    /// Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate.
    ///  SERIAL_CONTROL
    /// </summary>
    public class SerialControlPacket: PacketV2<SerialControlPayload>
    {
	public const int PacketMessageId = 126;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 220;

        public override SerialControlPayload Payload { get; } = new SerialControlPayload();

        public override string Name => "SERIAL_CONTROL";
    }

    /// <summary>
    ///  SERIAL_CONTROL
    /// </summary>
    public class SerialControlPayload : IPayload
    {
        public byte GetMaxByteSize() => 79;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Baudrate = BitConverter.ToUInt32(buffer,index);index+=4;
            Timeout = BitConverter.ToUInt16(buffer,index);index+=2;
            Device = (SerialControlDev)buffer[index++];
            Flags = (SerialControlFlag)buffer[index++];
            Count = (byte)buffer[index++];
            arraySize = /*ArrayLength*/70 - Math.Max(0,((/*PayloadByteSize*/79 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<70;i++)
            {
                Data[i] = default(byte);
            }
            for(var i=0;i<arraySize;i++)
            {
                Data[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Baudrate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Timeout).CopyTo(buffer, index);index+=2;
            buffer[index] = (byte)Device;index+=1;
            buffer[index] = (byte)Flags;index+=1;
            BitConverter.GetBytes(Count).CopyTo(buffer, index);index+=1;
            for(var i=0;i<70;i++)
            {
                buffer[index] = (byte)Data[i];index+=1;
            }
            return /*PayloadByteSize*/79;
        }

        /// <summary>
        /// Baudrate of transfer. Zero means no change.
        /// OriginName: baudrate, Units: bits/s, IsExtended: false
        /// </summary>
        public uint Baudrate { get; set; }
        /// <summary>
        /// Timeout for reply data in milliseconds
        /// OriginName: timeout, Units: ms, IsExtended: false
        /// </summary>
        public ushort Timeout { get; set; }
        /// <summary>
        /// See SERIAL_CONTROL_DEV enum
        /// OriginName: device, Units: , IsExtended: false
        /// </summary>
        public SerialControlDev Device { get; set; }
        /// <summary>
        /// See SERIAL_CONTROL_FLAG enum
        /// OriginName: flags, Units: , IsExtended: false
        /// </summary>
        public SerialControlFlag Flags { get; set; }
        /// <summary>
        /// how many bytes in this transfer
        /// OriginName: count, Units: bytes, IsExtended: false
        /// </summary>
        public byte Count { get; set; }
        /// <summary>
        /// serial data
        /// OriginName: data, Units: , IsExtended: false
        /// </summary>
        public byte[] Data { get; } = new byte[70];
    }
    /// <summary>
    /// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
    ///  GPS_RTK
    /// </summary>
    public class GpsRtkPacket: PacketV2<GpsRtkPayload>
    {
	public const int PacketMessageId = 127;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 25;

        public override GpsRtkPayload Payload { get; } = new GpsRtkPayload();

        public override string Name => "GPS_RTK";
    }

    /// <summary>
    ///  GPS_RTK
    /// </summary>
    public class GpsRtkPayload : IPayload
    {
        public byte GetMaxByteSize() => 35;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeLastBaselineMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Tow = BitConverter.ToUInt32(buffer,index);index+=4;
            BaselineAMm = BitConverter.ToInt32(buffer,index);index+=4;
            BaselineBMm = BitConverter.ToInt32(buffer,index);index+=4;
            BaselineCMm = BitConverter.ToInt32(buffer,index);index+=4;
            Accuracy = BitConverter.ToUInt32(buffer,index);index+=4;
            IarNumHypotheses = BitConverter.ToInt32(buffer,index);index+=4;
            Wn = BitConverter.ToUInt16(buffer,index);index+=2;
            RtkReceiverId = (byte)buffer[index++];
            RtkHealth = (byte)buffer[index++];
            RtkRate = (byte)buffer[index++];
            Nsats = (byte)buffer[index++];
            BaselineCoordsType = (RtkBaselineCoordinateSystem)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeLastBaselineMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Tow).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(BaselineAMm).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(BaselineBMm).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(BaselineCMm).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Accuracy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(IarNumHypotheses).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Wn).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(RtkReceiverId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(RtkHealth).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(RtkRate).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Nsats).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)BaselineCoordsType;index+=1;
            return /*PayloadByteSize*/35;
        }

        /// <summary>
        /// Time since boot of last baseline message received in ms.
        /// OriginName: time_last_baseline_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeLastBaselineMs { get; set; }
        /// <summary>
        /// GPS Time of Week of last baseline
        /// OriginName: tow, Units: ms, IsExtended: false
        /// </summary>
        public uint Tow { get; set; }
        /// <summary>
        /// Current baseline in ECEF x or NED north component in mm.
        /// OriginName: baseline_a_mm, Units: mm, IsExtended: false
        /// </summary>
        public int BaselineAMm { get; set; }
        /// <summary>
        /// Current baseline in ECEF y or NED east component in mm.
        /// OriginName: baseline_b_mm, Units: mm, IsExtended: false
        /// </summary>
        public int BaselineBMm { get; set; }
        /// <summary>
        /// Current baseline in ECEF z or NED down component in mm.
        /// OriginName: baseline_c_mm, Units: mm, IsExtended: false
        /// </summary>
        public int BaselineCMm { get; set; }
        /// <summary>
        /// Current estimate of baseline accuracy.
        /// OriginName: accuracy, Units: , IsExtended: false
        /// </summary>
        public uint Accuracy { get; set; }
        /// <summary>
        /// Current number of integer ambiguity hypotheses.
        /// OriginName: iar_num_hypotheses, Units: , IsExtended: false
        /// </summary>
        public int IarNumHypotheses { get; set; }
        /// <summary>
        /// GPS Week Number of last baseline
        /// OriginName: wn, Units: , IsExtended: false
        /// </summary>
        public ushort Wn { get; set; }
        /// <summary>
        /// Identification of connected RTK receiver.
        /// OriginName: rtk_receiver_id, Units: , IsExtended: false
        /// </summary>
        public byte RtkReceiverId { get; set; }
        /// <summary>
        /// GPS-specific health report for RTK data.
        /// OriginName: rtk_health, Units: , IsExtended: false
        /// </summary>
        public byte RtkHealth { get; set; }
        /// <summary>
        /// Rate of baseline messages being received by GPS, in HZ
        /// OriginName: rtk_rate, Units: Hz, IsExtended: false
        /// </summary>
        public byte RtkRate { get; set; }
        /// <summary>
        /// Current number of sats used for RTK calculation.
        /// OriginName: nsats, Units: , IsExtended: false
        /// </summary>
        public byte Nsats { get; set; }
        /// <summary>
        /// Coordinate system of baseline
        /// OriginName: baseline_coords_type, Units: , IsExtended: false
        /// </summary>
        public RtkBaselineCoordinateSystem BaselineCoordsType { get; set; }
    }
    /// <summary>
    /// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
    ///  GPS2_RTK
    /// </summary>
    public class Gps2RtkPacket: PacketV2<Gps2RtkPayload>
    {
	public const int PacketMessageId = 128;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 226;

        public override Gps2RtkPayload Payload { get; } = new Gps2RtkPayload();

        public override string Name => "GPS2_RTK";
    }

    /// <summary>
    ///  GPS2_RTK
    /// </summary>
    public class Gps2RtkPayload : IPayload
    {
        public byte GetMaxByteSize() => 35;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeLastBaselineMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Tow = BitConverter.ToUInt32(buffer,index);index+=4;
            BaselineAMm = BitConverter.ToInt32(buffer,index);index+=4;
            BaselineBMm = BitConverter.ToInt32(buffer,index);index+=4;
            BaselineCMm = BitConverter.ToInt32(buffer,index);index+=4;
            Accuracy = BitConverter.ToUInt32(buffer,index);index+=4;
            IarNumHypotheses = BitConverter.ToInt32(buffer,index);index+=4;
            Wn = BitConverter.ToUInt16(buffer,index);index+=2;
            RtkReceiverId = (byte)buffer[index++];
            RtkHealth = (byte)buffer[index++];
            RtkRate = (byte)buffer[index++];
            Nsats = (byte)buffer[index++];
            BaselineCoordsType = (RtkBaselineCoordinateSystem)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeLastBaselineMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Tow).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(BaselineAMm).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(BaselineBMm).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(BaselineCMm).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Accuracy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(IarNumHypotheses).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Wn).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(RtkReceiverId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(RtkHealth).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(RtkRate).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Nsats).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)BaselineCoordsType;index+=1;
            return /*PayloadByteSize*/35;
        }

        /// <summary>
        /// Time since boot of last baseline message received in ms.
        /// OriginName: time_last_baseline_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeLastBaselineMs { get; set; }
        /// <summary>
        /// GPS Time of Week of last baseline
        /// OriginName: tow, Units: ms, IsExtended: false
        /// </summary>
        public uint Tow { get; set; }
        /// <summary>
        /// Current baseline in ECEF x or NED north component in mm.
        /// OriginName: baseline_a_mm, Units: mm, IsExtended: false
        /// </summary>
        public int BaselineAMm { get; set; }
        /// <summary>
        /// Current baseline in ECEF y or NED east component in mm.
        /// OriginName: baseline_b_mm, Units: mm, IsExtended: false
        /// </summary>
        public int BaselineBMm { get; set; }
        /// <summary>
        /// Current baseline in ECEF z or NED down component in mm.
        /// OriginName: baseline_c_mm, Units: mm, IsExtended: false
        /// </summary>
        public int BaselineCMm { get; set; }
        /// <summary>
        /// Current estimate of baseline accuracy.
        /// OriginName: accuracy, Units: , IsExtended: false
        /// </summary>
        public uint Accuracy { get; set; }
        /// <summary>
        /// Current number of integer ambiguity hypotheses.
        /// OriginName: iar_num_hypotheses, Units: , IsExtended: false
        /// </summary>
        public int IarNumHypotheses { get; set; }
        /// <summary>
        /// GPS Week Number of last baseline
        /// OriginName: wn, Units: , IsExtended: false
        /// </summary>
        public ushort Wn { get; set; }
        /// <summary>
        /// Identification of connected RTK receiver.
        /// OriginName: rtk_receiver_id, Units: , IsExtended: false
        /// </summary>
        public byte RtkReceiverId { get; set; }
        /// <summary>
        /// GPS-specific health report for RTK data.
        /// OriginName: rtk_health, Units: , IsExtended: false
        /// </summary>
        public byte RtkHealth { get; set; }
        /// <summary>
        /// Rate of baseline messages being received by GPS, in HZ
        /// OriginName: rtk_rate, Units: Hz, IsExtended: false
        /// </summary>
        public byte RtkRate { get; set; }
        /// <summary>
        /// Current number of sats used for RTK calculation.
        /// OriginName: nsats, Units: , IsExtended: false
        /// </summary>
        public byte Nsats { get; set; }
        /// <summary>
        /// Coordinate system of baseline
        /// OriginName: baseline_coords_type, Units: , IsExtended: false
        /// </summary>
        public RtkBaselineCoordinateSystem BaselineCoordsType { get; set; }
    }
    /// <summary>
    /// The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described units
    ///  SCALED_IMU3
    /// </summary>
    public class ScaledImu3Packet: PacketV2<ScaledImu3Payload>
    {
	public const int PacketMessageId = 129;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 46;

        public override ScaledImu3Payload Payload { get; } = new ScaledImu3Payload();

        public override string Name => "SCALED_IMU3";
    }

    /// <summary>
    ///  SCALED_IMU3
    /// </summary>
    public class ScaledImu3Payload : IPayload
    {
        public byte GetMaxByteSize() => 22;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Xacc = BitConverter.ToInt16(buffer,index);index+=2;
            Yacc = BitConverter.ToInt16(buffer,index);index+=2;
            Zacc = BitConverter.ToInt16(buffer,index);index+=2;
            Xgyro = BitConverter.ToInt16(buffer,index);index+=2;
            Ygyro = BitConverter.ToInt16(buffer,index);index+=2;
            Zgyro = BitConverter.ToInt16(buffer,index);index+=2;
            Xmag = BitConverter.ToInt16(buffer,index);index+=2;
            Ymag = BitConverter.ToInt16(buffer,index);index+=2;
            Zmag = BitConverter.ToInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Xacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Yacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Zacc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Xgyro).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Ygyro).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Zgyro).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Xmag).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Ymag).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Zmag).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/22;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X acceleration (mg)
        /// OriginName: xacc, Units: mG, IsExtended: false
        /// </summary>
        public short Xacc { get; set; }
        /// <summary>
        /// Y acceleration (mg)
        /// OriginName: yacc, Units: mG, IsExtended: false
        /// </summary>
        public short Yacc { get; set; }
        /// <summary>
        /// Z acceleration (mg)
        /// OriginName: zacc, Units: mG, IsExtended: false
        /// </summary>
        public short Zacc { get; set; }
        /// <summary>
        /// Angular speed around X axis (millirad /sec)
        /// OriginName: xgyro, Units: mrad/s, IsExtended: false
        /// </summary>
        public short Xgyro { get; set; }
        /// <summary>
        /// Angular speed around Y axis (millirad /sec)
        /// OriginName: ygyro, Units: mrad/s, IsExtended: false
        /// </summary>
        public short Ygyro { get; set; }
        /// <summary>
        /// Angular speed around Z axis (millirad /sec)
        /// OriginName: zgyro, Units: mrad/s, IsExtended: false
        /// </summary>
        public short Zgyro { get; set; }
        /// <summary>
        /// X Magnetic field (milli tesla)
        /// OriginName: xmag, Units: mT, IsExtended: false
        /// </summary>
        public short Xmag { get; set; }
        /// <summary>
        /// Y Magnetic field (milli tesla)
        /// OriginName: ymag, Units: mT, IsExtended: false
        /// </summary>
        public short Ymag { get; set; }
        /// <summary>
        /// Z Magnetic field (milli tesla)
        /// OriginName: zmag, Units: mT, IsExtended: false
        /// </summary>
        public short Zmag { get; set; }
    }
    /// <summary>
    ///  DATA_TRANSMISSION_HANDSHAKE
    /// </summary>
    public class DataTransmissionHandshakePacket: PacketV2<DataTransmissionHandshakePayload>
    {
	public const int PacketMessageId = 130;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 29;

        public override DataTransmissionHandshakePayload Payload { get; } = new DataTransmissionHandshakePayload();

        public override string Name => "DATA_TRANSMISSION_HANDSHAKE";
    }

    /// <summary>
    ///  DATA_TRANSMISSION_HANDSHAKE
    /// </summary>
    public class DataTransmissionHandshakePayload : IPayload
    {
        public byte GetMaxByteSize() => 13;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Size = BitConverter.ToUInt32(buffer,index);index+=4;
            Width = BitConverter.ToUInt16(buffer,index);index+=2;
            Height = BitConverter.ToUInt16(buffer,index);index+=2;
            Packets = BitConverter.ToUInt16(buffer,index);index+=2;
            Type = (byte)buffer[index++];
            Payload = (byte)buffer[index++];
            JpgQuality = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Size).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Width).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Height).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Packets).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Type).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Payload).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(JpgQuality).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/13;
        }

        /// <summary>
        /// total data size in bytes (set on ACK only)
        /// OriginName: size, Units: bytes, IsExtended: false
        /// </summary>
        public uint Size { get; set; }
        /// <summary>
        /// Width of a matrix or image
        /// OriginName: width, Units: , IsExtended: false
        /// </summary>
        public ushort Width { get; set; }
        /// <summary>
        /// Height of a matrix or image
        /// OriginName: height, Units: , IsExtended: false
        /// </summary>
        public ushort Height { get; set; }
        /// <summary>
        /// number of packets beeing sent (set on ACK only)
        /// OriginName: packets, Units: , IsExtended: false
        /// </summary>
        public ushort Packets { get; set; }
        /// <summary>
        /// type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
        /// OriginName: type, Units: , IsExtended: false
        /// </summary>
        public byte Type { get; set; }
        /// <summary>
        /// payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
        /// OriginName: payload, Units: bytes, IsExtended: false
        /// </summary>
        public byte Payload { get; set; }
        /// <summary>
        /// JPEG quality out of [1,100]
        /// OriginName: jpg_quality, Units: %, IsExtended: false
        /// </summary>
        public byte JpgQuality { get; set; }
    }
    /// <summary>
    ///  ENCAPSULATED_DATA
    /// </summary>
    public class EncapsulatedDataPacket: PacketV2<EncapsulatedDataPayload>
    {
	public const int PacketMessageId = 131;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 223;

        public override EncapsulatedDataPayload Payload { get; } = new EncapsulatedDataPayload();

        public override string Name => "ENCAPSULATED_DATA";
    }

    /// <summary>
    ///  ENCAPSULATED_DATA
    /// </summary>
    public class EncapsulatedDataPayload : IPayload
    {
        public byte GetMaxByteSize() => 255;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Seqnr = BitConverter.ToUInt16(buffer,index);index+=2;
            arraySize = /*ArrayLength*/253 - Math.Max(0,((/*PayloadByteSize*/255 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<253;i++)
            {
                Data[i] = default(byte);
            }
            for(var i=0;i<arraySize;i++)
            {
                Data[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Seqnr).CopyTo(buffer, index);index+=2;
            for(var i=0;i<253;i++)
            {
                buffer[index] = (byte)Data[i];index+=1;
            }
            return /*PayloadByteSize*/255;
        }

        /// <summary>
        /// sequence number (starting with 0 on every transmission)
        /// OriginName: seqnr, Units: , IsExtended: false
        /// </summary>
        public ushort Seqnr { get; set; }
        /// <summary>
        /// image data bytes
        /// OriginName: data, Units: , IsExtended: false
        /// </summary>
        public byte[] Data { get; } = new byte[253];
    }
    /// <summary>
    ///  DISTANCE_SENSOR
    /// </summary>
    public class DistanceSensorPacket: PacketV2<DistanceSensorPayload>
    {
	public const int PacketMessageId = 132;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 85;

        public override DistanceSensorPayload Payload { get; } = new DistanceSensorPayload();

        public override string Name => "DISTANCE_SENSOR";
    }

    /// <summary>
    ///  DISTANCE_SENSOR
    /// </summary>
    public class DistanceSensorPayload : IPayload
    {
        public byte GetMaxByteSize() => 14;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            MinDistance = BitConverter.ToUInt16(buffer,index);index+=2;
            MaxDistance = BitConverter.ToUInt16(buffer,index);index+=2;
            CurrentDistance = BitConverter.ToUInt16(buffer,index);index+=2;
            Type = (MavDistanceSensor)buffer[index++];
            Id = (byte)buffer[index++];
            Orientation = (MavSensorOrientation)buffer[index++];
            Covariance = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(MinDistance).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(MaxDistance).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(CurrentDistance).CopyTo(buffer, index);index+=2;
            buffer[index] = (byte)Type;index+=1;
            BitConverter.GetBytes(Id).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)Orientation;index+=1;
            BitConverter.GetBytes(Covariance).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/14;
        }

        /// <summary>
        /// Time since system boot
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Minimum distance the sensor can measure in centimeters
        /// OriginName: min_distance, Units: cm, IsExtended: false
        /// </summary>
        public ushort MinDistance { get; set; }
        /// <summary>
        /// Maximum distance the sensor can measure in centimeters
        /// OriginName: max_distance, Units: cm, IsExtended: false
        /// </summary>
        public ushort MaxDistance { get; set; }
        /// <summary>
        /// Current distance reading
        /// OriginName: current_distance, Units: cm, IsExtended: false
        /// </summary>
        public ushort CurrentDistance { get; set; }
        /// <summary>
        /// Type from MAV_DISTANCE_SENSOR enum.
        /// OriginName: type, Units: , IsExtended: false
        /// </summary>
        public MavDistanceSensor Type { get; set; }
        /// <summary>
        /// Onboard ID of the sensor
        /// OriginName: id, Units: , IsExtended: false
        /// </summary>
        public byte Id { get; set; }
        /// <summary>
        /// Direction the sensor faces from MAV_SENSOR_ORIENTATION enum. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270
        /// OriginName: orientation, Units: , IsExtended: false
        /// </summary>
        public MavSensorOrientation Orientation { get; set; }
        /// <summary>
        /// Measurement covariance in centimeters, 0 for unknown / invalid readings
        /// OriginName: covariance, Units: cm, IsExtended: false
        /// </summary>
        public byte Covariance { get; set; }
    }
    /// <summary>
    /// Request for terrain data and terrain status
    ///  TERRAIN_REQUEST
    /// </summary>
    public class TerrainRequestPacket: PacketV2<TerrainRequestPayload>
    {
	public const int PacketMessageId = 133;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 6;

        public override TerrainRequestPayload Payload { get; } = new TerrainRequestPayload();

        public override string Name => "TERRAIN_REQUEST";
    }

    /// <summary>
    ///  TERRAIN_REQUEST
    /// </summary>
    public class TerrainRequestPayload : IPayload
    {
        public byte GetMaxByteSize() => 18;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Mask = BitConverter.ToUInt64(buffer,index);index+=8;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            GridSpacing = BitConverter.ToUInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Mask).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(GridSpacing).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/18;
        }

        /// <summary>
        /// Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
        /// OriginName: mask, Units: , IsExtended: false
        /// </summary>
        public ulong Mask { get; set; }
        /// <summary>
        /// Latitude of SW corner of first grid (degrees *10^7)
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude of SW corner of first grid (in degrees *10^7)
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Grid spacing in meters
        /// OriginName: grid_spacing, Units: m, IsExtended: false
        /// </summary>
        public ushort GridSpacing { get; set; }
    }
    /// <summary>
    /// Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST
    ///  TERRAIN_DATA
    /// </summary>
    public class TerrainDataPacket: PacketV2<TerrainDataPayload>
    {
	public const int PacketMessageId = 134;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 229;

        public override TerrainDataPayload Payload { get; } = new TerrainDataPayload();

        public override string Name => "TERRAIN_DATA";
    }

    /// <summary>
    ///  TERRAIN_DATA
    /// </summary>
    public class TerrainDataPayload : IPayload
    {
        public byte GetMaxByteSize() => 43;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            GridSpacing = BitConverter.ToUInt16(buffer,index);index+=2;
            arraySize = /*ArrayLength*/16 - Math.Max(0,((/*PayloadByteSize*/43 - payloadSize - /*ExtendedFieldsLength*/0)/2 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<16;i++)
            {
                Data[i] = default(short);
            }
            for(var i=0;i<arraySize;i++)
            {
                Data[i] = BitConverter.ToInt16(buffer,index);index+=2;
            }
            Gridbit = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(GridSpacing).CopyTo(buffer, index);index+=2;
            for(var i=0;i<16;i++)
            {
                BitConverter.GetBytes(Data[i]).CopyTo(buffer, index);index+=2;
            }
            BitConverter.GetBytes(Gridbit).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/43;
        }

        /// <summary>
        /// Latitude of SW corner of first grid (degrees *10^7)
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude of SW corner of first grid (in degrees *10^7)
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Grid spacing in meters
        /// OriginName: grid_spacing, Units: m, IsExtended: false
        /// </summary>
        public ushort GridSpacing { get; set; }
        /// <summary>
        /// Terrain data in meters AMSL
        /// OriginName: data, Units: m, IsExtended: false
        /// </summary>
        public short[] Data { get; } = new short[16];
        /// <summary>
        /// bit within the terrain request mask
        /// OriginName: gridbit, Units: , IsExtended: false
        /// </summary>
        public byte Gridbit { get; set; }
    }
    /// <summary>
    /// Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle has all terrain data needed for a mission.
    ///  TERRAIN_CHECK
    /// </summary>
    public class TerrainCheckPacket: PacketV2<TerrainCheckPayload>
    {
	public const int PacketMessageId = 135;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 203;

        public override TerrainCheckPayload Payload { get; } = new TerrainCheckPayload();

        public override string Name => "TERRAIN_CHECK";
    }

    /// <summary>
    ///  TERRAIN_CHECK
    /// </summary>
    public class TerrainCheckPayload : IPayload
    {
        public byte GetMaxByteSize() => 8;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/8;
        }

        /// <summary>
        /// Latitude (degrees *10^7)
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude (degrees *10^7)
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
    }
    /// <summary>
    /// Response from a TERRAIN_CHECK request
    ///  TERRAIN_REPORT
    /// </summary>
    public class TerrainReportPacket: PacketV2<TerrainReportPayload>
    {
	public const int PacketMessageId = 136;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 1;

        public override TerrainReportPayload Payload { get; } = new TerrainReportPayload();

        public override string Name => "TERRAIN_REPORT";
    }

    /// <summary>
    ///  TERRAIN_REPORT
    /// </summary>
    public class TerrainReportPayload : IPayload
    {
        public byte GetMaxByteSize() => 22;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            TerrainHeight = BitConverter.ToSingle(buffer, index);index+=4;
            CurrentHeight = BitConverter.ToSingle(buffer, index);index+=4;
            Spacing = BitConverter.ToUInt16(buffer,index);index+=2;
            Pending = BitConverter.ToUInt16(buffer,index);index+=2;
            Loaded = BitConverter.ToUInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TerrainHeight).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(CurrentHeight).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Spacing).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Pending).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Loaded).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/22;
        }

        /// <summary>
        /// Latitude (degrees *10^7)
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude (degrees *10^7)
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Terrain height in meters AMSL
        /// OriginName: terrain_height, Units: m, IsExtended: false
        /// </summary>
        public float TerrainHeight { get; set; }
        /// <summary>
        /// Current vehicle height above lat/lon terrain height (meters)
        /// OriginName: current_height, Units: m, IsExtended: false
        /// </summary>
        public float CurrentHeight { get; set; }
        /// <summary>
        /// grid spacing (zero if terrain at this location unavailable)
        /// OriginName: spacing, Units: , IsExtended: false
        /// </summary>
        public ushort Spacing { get; set; }
        /// <summary>
        /// Number of 4x4 terrain blocks waiting to be received or read from disk
        /// OriginName: pending, Units: , IsExtended: false
        /// </summary>
        public ushort Pending { get; set; }
        /// <summary>
        /// Number of 4x4 terrain blocks in memory
        /// OriginName: loaded, Units: , IsExtended: false
        /// </summary>
        public ushort Loaded { get; set; }
    }
    /// <summary>
    /// Barometer readings for 2nd barometer
    ///  SCALED_PRESSURE2
    /// </summary>
    public class ScaledPressure2Packet: PacketV2<ScaledPressure2Payload>
    {
	public const int PacketMessageId = 137;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 195;

        public override ScaledPressure2Payload Payload { get; } = new ScaledPressure2Payload();

        public override string Name => "SCALED_PRESSURE2";
    }

    /// <summary>
    ///  SCALED_PRESSURE2
    /// </summary>
    public class ScaledPressure2Payload : IPayload
    {
        public byte GetMaxByteSize() => 14;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            PressAbs = BitConverter.ToSingle(buffer, index);index+=4;
            PressDiff = BitConverter.ToSingle(buffer, index);index+=4;
            Temperature = BitConverter.ToInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PressAbs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PressDiff).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/14;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Absolute pressure (hectopascal)
        /// OriginName: press_abs, Units: hPa, IsExtended: false
        /// </summary>
        public float PressAbs { get; set; }
        /// <summary>
        /// Differential pressure 1 (hectopascal)
        /// OriginName: press_diff, Units: hPa, IsExtended: false
        /// </summary>
        public float PressDiff { get; set; }
        /// <summary>
        /// Temperature measurement (0.01 degrees celsius)
        /// OriginName: temperature, Units: cdegC, IsExtended: false
        /// </summary>
        public short Temperature { get; set; }
    }
    /// <summary>
    /// Motion capture attitude and position
    ///  ATT_POS_MOCAP
    /// </summary>
    public class AttPosMocapPacket: PacketV2<AttPosMocapPayload>
    {
	public const int PacketMessageId = 138;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 109;

        public override AttPosMocapPayload Payload { get; } = new AttPosMocapPayload();

        public override string Name => "ATT_POS_MOCAP";
    }

    /// <summary>
    ///  ATT_POS_MOCAP
    /// </summary>
    public class AttPosMocapPayload : IPayload
    {
        public byte GetMaxByteSize() => 120;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            arraySize = /*ArrayLength*/4 - Math.Max(0,((/*PayloadByteSize*/120 - payloadSize - /*ExtendedFieldsLength*/84)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<4;i++)
            {
                Q[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            // extended field 'Covariance' can be empty
            if (index >= endIndex) return;
            arraySize = 21;
            for(var i=0;i<arraySize;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(Q[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            for(var i=0;i<21;i++)
            {
                BitConverter.GetBytes(Covariance[i]).CopyTo(buffer, index);index+=4;
            }
            return /*PayloadByteSize*/120;
        }

        /// <summary>
        /// Timestamp (micros since boot or Unix epoch)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        /// OriginName: q, Units: , IsExtended: false
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// X position in meters (NED)
        /// OriginName: x, Units: m, IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y position in meters (NED)
        /// OriginName: y, Units: m, IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z position in meters (NED)
        /// OriginName: z, Units: m, IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Pose covariance matrix upper right triangular (first six entries are the first ROW, next five entries are the second ROW, etc.)
        /// OriginName: covariance, Units: , IsExtended: true
        /// </summary>
        public float[] Covariance { get; } = new float[21];
    }
    /// <summary>
    /// Set the vehicle attitude and body angular rates.
    ///  SET_ACTUATOR_CONTROL_TARGET
    /// </summary>
    public class SetActuatorControlTargetPacket: PacketV2<SetActuatorControlTargetPayload>
    {
	public const int PacketMessageId = 139;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 168;

        public override SetActuatorControlTargetPayload Payload { get; } = new SetActuatorControlTargetPayload();

        public override string Name => "SET_ACTUATOR_CONTROL_TARGET";
    }

    /// <summary>
    ///  SET_ACTUATOR_CONTROL_TARGET
    /// </summary>
    public class SetActuatorControlTargetPayload : IPayload
    {
        public byte GetMaxByteSize() => 43;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            arraySize = /*ArrayLength*/8 - Math.Max(0,((/*PayloadByteSize*/43 - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<8;i++)
            {
                Controls[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                Controls[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            GroupMlx = (byte)buffer[index++];
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            for(var i=0;i<8;i++)
            {
                BitConverter.GetBytes(Controls[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(GroupMlx).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/43;
        }

        /// <summary>
        /// Timestamp (micros since boot or Unix epoch)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
        /// OriginName: controls, Units: , IsExtended: false
        /// </summary>
        public float[] Controls { get; } = new float[8];
        /// <summary>
        /// Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
        /// OriginName: group_mlx, Units: , IsExtended: false
        /// </summary>
        public byte GroupMlx { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Set the vehicle attitude and body angular rates.
    ///  ACTUATOR_CONTROL_TARGET
    /// </summary>
    public class ActuatorControlTargetPacket: PacketV2<ActuatorControlTargetPayload>
    {
	public const int PacketMessageId = 140;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 181;

        public override ActuatorControlTargetPayload Payload { get; } = new ActuatorControlTargetPayload();

        public override string Name => "ACTUATOR_CONTROL_TARGET";
    }

    /// <summary>
    ///  ACTUATOR_CONTROL_TARGET
    /// </summary>
    public class ActuatorControlTargetPayload : IPayload
    {
        public byte GetMaxByteSize() => 41;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            arraySize = /*ArrayLength*/8 - Math.Max(0,((/*PayloadByteSize*/41 - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<8;i++)
            {
                Controls[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                Controls[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            GroupMlx = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            for(var i=0;i<8;i++)
            {
                BitConverter.GetBytes(Controls[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(GroupMlx).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/41;
        }

        /// <summary>
        /// Timestamp (micros since boot or Unix epoch)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
        /// OriginName: controls, Units: , IsExtended: false
        /// </summary>
        public float[] Controls { get; } = new float[8];
        /// <summary>
        /// Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
        /// OriginName: group_mlx, Units: , IsExtended: false
        /// </summary>
        public byte GroupMlx { get; set; }
    }
    /// <summary>
    /// The current system altitude.
    ///  ALTITUDE
    /// </summary>
    public class AltitudePacket: PacketV2<AltitudePayload>
    {
	public const int PacketMessageId = 141;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 47;

        public override AltitudePayload Payload { get; } = new AltitudePayload();

        public override string Name => "ALTITUDE";
    }

    /// <summary>
    ///  ALTITUDE
    /// </summary>
    public class AltitudePayload : IPayload
    {
        public byte GetMaxByteSize() => 32;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            AltitudeMonotonic = BitConverter.ToSingle(buffer, index);index+=4;
            AltitudeAmsl = BitConverter.ToSingle(buffer, index);index+=4;
            AltitudeLocal = BitConverter.ToSingle(buffer, index);index+=4;
            AltitudeRelative = BitConverter.ToSingle(buffer, index);index+=4;
            AltitudeTerrain = BitConverter.ToSingle(buffer, index);index+=4;
            BottomClearance = BitConverter.ToSingle(buffer, index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(AltitudeMonotonic).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(AltitudeAmsl).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(AltitudeLocal).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(AltitudeRelative).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(AltitudeTerrain).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(BottomClearance).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/32;
        }

        /// <summary>
        /// Timestamp (micros since boot or Unix epoch)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). The only guarantee on this field is that it will never be reset and is consistent within a flight. The recommended value for this field is the uncorrected barometric altitude at boot time. This altitude will also drift and vary between flights.
        /// OriginName: altitude_monotonic, Units: m, IsExtended: false
        /// </summary>
        public float AltitudeMonotonic { get; set; }
        /// <summary>
        /// This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL by default and not the WGS84 altitude.
        /// OriginName: altitude_amsl, Units: m, IsExtended: false
        /// </summary>
        public float AltitudeAmsl { get; set; }
        /// <summary>
        /// This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive.
        /// OriginName: altitude_local, Units: m, IsExtended: false
        /// </summary>
        public float AltitudeLocal { get; set; }
        /// <summary>
        /// This is the altitude above the home position. It resets on each change of the current home position.
        /// OriginName: altitude_relative, Units: m, IsExtended: false
        /// </summary>
        public float AltitudeRelative { get; set; }
        /// <summary>
        /// This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown.
        /// OriginName: altitude_terrain, Units: m, IsExtended: false
        /// </summary>
        public float AltitudeTerrain { get; set; }
        /// <summary>
        /// This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available.
        /// OriginName: bottom_clearance, Units: m, IsExtended: false
        /// </summary>
        public float BottomClearance { get; set; }
    }
    /// <summary>
    /// The autopilot is requesting a resource (file, binary, other type of data)
    ///  RESOURCE_REQUEST
    /// </summary>
    public class ResourceRequestPacket: PacketV2<ResourceRequestPayload>
    {
	public const int PacketMessageId = 142;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 72;

        public override ResourceRequestPayload Payload { get; } = new ResourceRequestPayload();

        public override string Name => "RESOURCE_REQUEST";
    }

    /// <summary>
    ///  RESOURCE_REQUEST
    /// </summary>
    public class ResourceRequestPayload : IPayload
    {
        public byte GetMaxByteSize() => 243;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            RequestId = (byte)buffer[index++];
            UriType = (byte)buffer[index++];
            arraySize = /*ArrayLength*/120 - Math.Max(0,((/*PayloadByteSize*/243 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<120;i++)
            {
                Uri[i] = default(byte);
            }
            for(var i=0;i<arraySize;i++)
            {
                Uri[i] = (byte)buffer[index++];
            }
            TransferType = (byte)buffer[index++];
            arraySize = 120;
            for(var i=0;i<arraySize;i++)
            {
                Storage[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(RequestId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(UriType).CopyTo(buffer, index);index+=1;
            for(var i=0;i<120;i++)
            {
                buffer[index] = (byte)Uri[i];index+=1;
            }
            BitConverter.GetBytes(TransferType).CopyTo(buffer, index);index+=1;
            for(var i=0;i<120;i++)
            {
                buffer[index] = (byte)Storage[i];index+=1;
            }
            return /*PayloadByteSize*/243;
        }

        /// <summary>
        /// Request ID. This ID should be re-used when sending back URI contents
        /// OriginName: request_id, Units: , IsExtended: false
        /// </summary>
        public byte RequestId { get; set; }
        /// <summary>
        /// The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
        /// OriginName: uri_type, Units: , IsExtended: false
        /// </summary>
        public byte UriType { get; set; }
        /// <summary>
        /// The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)
        /// OriginName: uri, Units: , IsExtended: false
        /// </summary>
        public byte[] Uri { get; } = new byte[120];
        /// <summary>
        /// The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
        /// OriginName: transfer_type, Units: , IsExtended: false
        /// </summary>
        public byte TransferType { get; set; }
        /// <summary>
        /// The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP).
        /// OriginName: storage, Units: , IsExtended: false
        /// </summary>
        public byte[] Storage { get; } = new byte[120];
    }
    /// <summary>
    /// Barometer readings for 3rd barometer
    ///  SCALED_PRESSURE3
    /// </summary>
    public class ScaledPressure3Packet: PacketV2<ScaledPressure3Payload>
    {
	public const int PacketMessageId = 143;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 131;

        public override ScaledPressure3Payload Payload { get; } = new ScaledPressure3Payload();

        public override string Name => "SCALED_PRESSURE3";
    }

    /// <summary>
    ///  SCALED_PRESSURE3
    /// </summary>
    public class ScaledPressure3Payload : IPayload
    {
        public byte GetMaxByteSize() => 14;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            PressAbs = BitConverter.ToSingle(buffer, index);index+=4;
            PressDiff = BitConverter.ToSingle(buffer, index);index+=4;
            Temperature = BitConverter.ToInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PressAbs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PressDiff).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/14;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Absolute pressure (hectopascal)
        /// OriginName: press_abs, Units: hPa, IsExtended: false
        /// </summary>
        public float PressAbs { get; set; }
        /// <summary>
        /// Differential pressure 1 (hectopascal)
        /// OriginName: press_diff, Units: hPa, IsExtended: false
        /// </summary>
        public float PressDiff { get; set; }
        /// <summary>
        /// Temperature measurement (0.01 degrees celsius)
        /// OriginName: temperature, Units: cdegC, IsExtended: false
        /// </summary>
        public short Temperature { get; set; }
    }
    /// <summary>
    /// current motion information from a designated system
    ///  FOLLOW_TARGET
    /// </summary>
    public class FollowTargetPacket: PacketV2<FollowTargetPayload>
    {
	public const int PacketMessageId = 144;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 127;

        public override FollowTargetPayload Payload { get; } = new FollowTargetPayload();

        public override string Name => "FOLLOW_TARGET";
    }

    /// <summary>
    ///  FOLLOW_TARGET
    /// </summary>
    public class FollowTargetPayload : IPayload
    {
        public byte GetMaxByteSize() => 93;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Timestamp = BitConverter.ToUInt64(buffer,index);index+=8;
            CustomState = BitConverter.ToUInt64(buffer,index);index+=8;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToSingle(buffer, index);index+=4;
            arraySize = 3;
            for(var i=0;i<arraySize;i++)
            {
                Vel[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            arraySize = 3;
            for(var i=0;i<arraySize;i++)
            {
                Acc[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            arraySize = /*ArrayLength*/4 - Math.Max(0,((/*PayloadByteSize*/93 - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<4;i++)
            {
                AttitudeQ[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                AttitudeQ[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            arraySize = 3;
            for(var i=0;i<arraySize;i++)
            {
                Rates[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            arraySize = 3;
            for(var i=0;i<arraySize;i++)
            {
                PositionCov[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            EstCapabilities = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Timestamp).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(CustomState).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            for(var i=0;i<3;i++)
            {
                BitConverter.GetBytes(Vel[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<3;i++)
            {
                BitConverter.GetBytes(Acc[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(AttitudeQ[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<3;i++)
            {
                BitConverter.GetBytes(Rates[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<3;i++)
            {
                BitConverter.GetBytes(PositionCov[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(EstCapabilities).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/93;
        }

        /// <summary>
        /// Timestamp in milliseconds since system boot
        /// OriginName: timestamp, Units: ms, IsExtended: false
        /// </summary>
        public ulong Timestamp { get; set; }
        /// <summary>
        /// button states or switches of a tracker device
        /// OriginName: custom_state, Units: , IsExtended: false
        /// </summary>
        public ulong CustomState { get; set; }
        /// <summary>
        /// Latitude (WGS84), in degrees * 1E7
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude (WGS84), in degrees * 1E7
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// AMSL, in meters
        /// OriginName: alt, Units: m, IsExtended: false
        /// </summary>
        public float Alt { get; set; }
        /// <summary>
        /// target velocity (0,0,0) for unknown
        /// OriginName: vel, Units: m/s, IsExtended: false
        /// </summary>
        public float[] Vel { get; } = new float[3];
        /// <summary>
        /// linear target acceleration (0,0,0) for unknown
        /// OriginName: acc, Units: m/s/s, IsExtended: false
        /// </summary>
        public float[] Acc { get; } = new float[3];
        /// <summary>
        /// (1 0 0 0 for unknown)
        /// OriginName: attitude_q, Units: , IsExtended: false
        /// </summary>
        public float[] AttitudeQ { get; } = new float[4];
        /// <summary>
        /// (0 0 0 for unknown)
        /// OriginName: rates, Units: , IsExtended: false
        /// </summary>
        public float[] Rates { get; } = new float[3];
        /// <summary>
        /// eph epv
        /// OriginName: position_cov, Units: , IsExtended: false
        /// </summary>
        public float[] PositionCov { get; } = new float[3];
        /// <summary>
        /// bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
        /// OriginName: est_capabilities, Units: , IsExtended: false
        /// </summary>
        public byte EstCapabilities { get; set; }
    }
    /// <summary>
    /// The smoothed, monotonic system state used to feed the control loops of the system.
    ///  CONTROL_SYSTEM_STATE
    /// </summary>
    public class ControlSystemStatePacket: PacketV2<ControlSystemStatePayload>
    {
	public const int PacketMessageId = 146;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 103;

        public override ControlSystemStatePayload Payload { get; } = new ControlSystemStatePayload();

        public override string Name => "CONTROL_SYSTEM_STATE";
    }

    /// <summary>
    ///  CONTROL_SYSTEM_STATE
    /// </summary>
    public class ControlSystemStatePayload : IPayload
    {
        public byte GetMaxByteSize() => 100;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            XAcc = BitConverter.ToSingle(buffer, index);index+=4;
            YAcc = BitConverter.ToSingle(buffer, index);index+=4;
            ZAcc = BitConverter.ToSingle(buffer, index);index+=4;
            XVel = BitConverter.ToSingle(buffer, index);index+=4;
            YVel = BitConverter.ToSingle(buffer, index);index+=4;
            ZVel = BitConverter.ToSingle(buffer, index);index+=4;
            XPos = BitConverter.ToSingle(buffer, index);index+=4;
            YPos = BitConverter.ToSingle(buffer, index);index+=4;
            ZPos = BitConverter.ToSingle(buffer, index);index+=4;
            Airspeed = BitConverter.ToSingle(buffer, index);index+=4;
            arraySize = 3;
            for(var i=0;i<arraySize;i++)
            {
                VelVariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            arraySize = 3;
            for(var i=0;i<arraySize;i++)
            {
                PosVariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            arraySize = /*ArrayLength*/4 - Math.Max(0,((/*PayloadByteSize*/100 - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<4;i++)
            {
                Q[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            RollRate = BitConverter.ToSingle(buffer, index);index+=4;
            PitchRate = BitConverter.ToSingle(buffer, index);index+=4;
            YawRate = BitConverter.ToSingle(buffer, index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(XAcc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(YAcc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ZAcc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(XVel).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(YVel).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ZVel).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(XPos).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(YPos).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ZPos).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Airspeed).CopyTo(buffer, index);index+=4;
            for(var i=0;i<3;i++)
            {
                BitConverter.GetBytes(VelVariance[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<3;i++)
            {
                BitConverter.GetBytes(PosVariance[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(Q[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(RollRate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PitchRate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(YawRate).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/100;
        }

        /// <summary>
        /// Timestamp (micros since boot or Unix epoch)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X acceleration in body frame
        /// OriginName: x_acc, Units: m/s/s, IsExtended: false
        /// </summary>
        public float XAcc { get; set; }
        /// <summary>
        /// Y acceleration in body frame
        /// OriginName: y_acc, Units: m/s/s, IsExtended: false
        /// </summary>
        public float YAcc { get; set; }
        /// <summary>
        /// Z acceleration in body frame
        /// OriginName: z_acc, Units: m/s/s, IsExtended: false
        /// </summary>
        public float ZAcc { get; set; }
        /// <summary>
        /// X velocity in body frame
        /// OriginName: x_vel, Units: m/s, IsExtended: false
        /// </summary>
        public float XVel { get; set; }
        /// <summary>
        /// Y velocity in body frame
        /// OriginName: y_vel, Units: m/s, IsExtended: false
        /// </summary>
        public float YVel { get; set; }
        /// <summary>
        /// Z velocity in body frame
        /// OriginName: z_vel, Units: m/s, IsExtended: false
        /// </summary>
        public float ZVel { get; set; }
        /// <summary>
        /// X position in local frame
        /// OriginName: x_pos, Units: m, IsExtended: false
        /// </summary>
        public float XPos { get; set; }
        /// <summary>
        /// Y position in local frame
        /// OriginName: y_pos, Units: m, IsExtended: false
        /// </summary>
        public float YPos { get; set; }
        /// <summary>
        /// Z position in local frame
        /// OriginName: z_pos, Units: m, IsExtended: false
        /// </summary>
        public float ZPos { get; set; }
        /// <summary>
        /// Airspeed, set to -1 if unknown
        /// OriginName: airspeed, Units: m/s, IsExtended: false
        /// </summary>
        public float Airspeed { get; set; }
        /// <summary>
        /// Variance of body velocity estimate
        /// OriginName: vel_variance, Units: , IsExtended: false
        /// </summary>
        public float[] VelVariance { get; } = new float[3];
        /// <summary>
        /// Variance in local position
        /// OriginName: pos_variance, Units: , IsExtended: false
        /// </summary>
        public float[] PosVariance { get; } = new float[3];
        /// <summary>
        /// The attitude, represented as Quaternion
        /// OriginName: q, Units: , IsExtended: false
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Angular rate in roll axis
        /// OriginName: roll_rate, Units: rad/s, IsExtended: false
        /// </summary>
        public float RollRate { get; set; }
        /// <summary>
        /// Angular rate in pitch axis
        /// OriginName: pitch_rate, Units: rad/s, IsExtended: false
        /// </summary>
        public float PitchRate { get; set; }
        /// <summary>
        /// Angular rate in yaw axis
        /// OriginName: yaw_rate, Units: rad/s, IsExtended: false
        /// </summary>
        public float YawRate { get; set; }
    }
    /// <summary>
    /// Battery information
    ///  BATTERY_STATUS
    /// </summary>
    public class BatteryStatusPacket: PacketV2<BatteryStatusPayload>
    {
	public const int PacketMessageId = 147;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 154;

        public override BatteryStatusPayload Payload { get; } = new BatteryStatusPayload();

        public override string Name => "BATTERY_STATUS";
    }

    /// <summary>
    ///  BATTERY_STATUS
    /// </summary>
    public class BatteryStatusPayload : IPayload
    {
        public byte GetMaxByteSize() => 41;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            CurrentConsumed = BitConverter.ToInt32(buffer,index);index+=4;
            EnergyConsumed = BitConverter.ToInt32(buffer,index);index+=4;
            Temperature = BitConverter.ToInt16(buffer,index);index+=2;
            arraySize = /*ArrayLength*/10 - Math.Max(0,((/*PayloadByteSize*/41 - payloadSize - /*ExtendedFieldsLength*/5)/2 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<10;i++)
            {
                Voltages[i] = default(ushort);
            }
            for(var i=0;i<arraySize;i++)
            {
                Voltages[i] = BitConverter.ToUInt16(buffer,index);index+=2;
            }
            CurrentBattery = BitConverter.ToInt16(buffer,index);index+=2;
            Id = (byte)buffer[index++];
            BatteryFunction = (MavBatteryFunction)buffer[index++];
            Type = (MavBatteryType)buffer[index++];
            BatteryRemaining = (sbyte)buffer[index++];
            // extended field 'TimeRemaining' can be empty
            if (index >= endIndex) return;
            TimeRemaining = BitConverter.ToInt32(buffer,index);index+=4;
            // extended field 'ChargeState' can be empty
            if (index >= endIndex) return;
            ChargeState = (MavBatteryChargeState)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(CurrentConsumed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(EnergyConsumed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=2;
            for(var i=0;i<10;i++)
            {
                BitConverter.GetBytes(Voltages[i]).CopyTo(buffer, index);index+=2;
            }
            BitConverter.GetBytes(CurrentBattery).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Id).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)BatteryFunction;index+=1;
            buffer[index] = (byte)Type;index+=1;
            BitConverter.GetBytes(BatteryRemaining).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TimeRemaining).CopyTo(buffer, index);index+=4;
            buffer[index] = (byte)ChargeState;index+=1;
            return /*PayloadByteSize*/41;
        }

        /// <summary>
        /// Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
        /// OriginName: current_consumed, Units: mAh, IsExtended: false
        /// </summary>
        public int CurrentConsumed { get; set; }
        /// <summary>
        /// Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
        /// OriginName: energy_consumed, Units: hJ, IsExtended: false
        /// </summary>
        public int EnergyConsumed { get; set; }
        /// <summary>
        /// Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
        /// OriginName: temperature, Units: cdegC, IsExtended: false
        /// </summary>
        public short Temperature { get; set; }
        /// <summary>
        /// Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value.
        /// OriginName: voltages, Units: mV, IsExtended: false
        /// </summary>
        public ushort[] Voltages { get; } = new ushort[10];
        /// <summary>
        /// Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
        /// OriginName: current_battery, Units: cA, IsExtended: false
        /// </summary>
        public short CurrentBattery { get; set; }
        /// <summary>
        /// Battery ID
        /// OriginName: id, Units: , IsExtended: false
        /// </summary>
        public byte Id { get; set; }
        /// <summary>
        /// Function of the battery
        /// OriginName: battery_function, Units: , IsExtended: false
        /// </summary>
        public MavBatteryFunction BatteryFunction { get; set; }
        /// <summary>
        /// Type (chemistry) of the battery
        /// OriginName: type, Units: , IsExtended: false
        /// </summary>
        public MavBatteryType Type { get; set; }
        /// <summary>
        /// Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
        /// OriginName: battery_remaining, Units: %, IsExtended: false
        /// </summary>
        public sbyte BatteryRemaining { get; set; }
        /// <summary>
        /// Remaining battery time, in seconds (1 = 1s = 0% energy left), 0: autopilot does not provide remaining battery time estimate
        /// OriginName: time_remaining, Units: s, IsExtended: true
        /// </summary>
        public int TimeRemaining { get; set; }
        /// <summary>
        /// State for extent of discharge, provided by autopilot for warning or external reactions
        /// OriginName: charge_state, Units: , IsExtended: true
        /// </summary>
        public MavBatteryChargeState ChargeState { get; set; }
    }
    /// <summary>
    /// Version and capability of autopilot software
    ///  AUTOPILOT_VERSION
    /// </summary>
    public class AutopilotVersionPacket: PacketV2<AutopilotVersionPayload>
    {
	public const int PacketMessageId = 148;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 178;

        public override AutopilotVersionPayload Payload { get; } = new AutopilotVersionPayload();

        public override string Name => "AUTOPILOT_VERSION";
    }

    /// <summary>
    ///  AUTOPILOT_VERSION
    /// </summary>
    public class AutopilotVersionPayload : IPayload
    {
        public byte GetMaxByteSize() => 78;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Capabilities = (MavProtocolCapability)BitConverter.ToUInt64(buffer,index);index+=8;
            Uid = BitConverter.ToUInt64(buffer,index);index+=8;
            FlightSwVersion = BitConverter.ToUInt32(buffer,index);index+=4;
            MiddlewareSwVersion = BitConverter.ToUInt32(buffer,index);index+=4;
            OsSwVersion = BitConverter.ToUInt32(buffer,index);index+=4;
            BoardVersion = BitConverter.ToUInt32(buffer,index);index+=4;
            VendorId = BitConverter.ToUInt16(buffer,index);index+=2;
            ProductId = BitConverter.ToUInt16(buffer,index);index+=2;
            arraySize = /*ArrayLength*/8 - Math.Max(0,((/*PayloadByteSize*/78 - payloadSize - /*ExtendedFieldsLength*/18)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<8;i++)
            {
                FlightCustomVersion[i] = default(byte);
            }
            for(var i=0;i<arraySize;i++)
            {
                FlightCustomVersion[i] = (byte)buffer[index++];
            }
            arraySize = 8;
            for(var i=0;i<arraySize;i++)
            {
                MiddlewareCustomVersion[i] = (byte)buffer[index++];
            }
            arraySize = 8;
            for(var i=0;i<arraySize;i++)
            {
                OsCustomVersion[i] = (byte)buffer[index++];
            }
            // extended field 'Uid2' can be empty
            if (index >= endIndex) return;
            arraySize = 18;
            for(var i=0;i<arraySize;i++)
            {
                Uid2[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes((ulong)Capabilities).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Uid).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(FlightSwVersion).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(MiddlewareSwVersion).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(OsSwVersion).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(BoardVersion).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(VendorId).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(ProductId).CopyTo(buffer, index);index+=2;
            for(var i=0;i<8;i++)
            {
                buffer[index] = (byte)FlightCustomVersion[i];index+=1;
            }
            for(var i=0;i<8;i++)
            {
                buffer[index] = (byte)MiddlewareCustomVersion[i];index+=1;
            }
            for(var i=0;i<8;i++)
            {
                buffer[index] = (byte)OsCustomVersion[i];index+=1;
            }
            for(var i=0;i<18;i++)
            {
                buffer[index] = (byte)Uid2[i];index+=1;
            }
            return /*PayloadByteSize*/78;
        }

        /// <summary>
        /// bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
        /// OriginName: capabilities, Units: , IsExtended: false
        /// </summary>
        public MavProtocolCapability Capabilities { get; set; }
        /// <summary>
        /// UID if provided by hardware (see uid2)
        /// OriginName: uid, Units: , IsExtended: false
        /// </summary>
        public ulong Uid { get; set; }
        /// <summary>
        /// Firmware version number
        /// OriginName: flight_sw_version, Units: , IsExtended: false
        /// </summary>
        public uint FlightSwVersion { get; set; }
        /// <summary>
        /// Middleware version number
        /// OriginName: middleware_sw_version, Units: , IsExtended: false
        /// </summary>
        public uint MiddlewareSwVersion { get; set; }
        /// <summary>
        /// Operating system version number
        /// OriginName: os_sw_version, Units: , IsExtended: false
        /// </summary>
        public uint OsSwVersion { get; set; }
        /// <summary>
        /// HW / board version (last 8 bytes should be silicon ID, if any)
        /// OriginName: board_version, Units: , IsExtended: false
        /// </summary>
        public uint BoardVersion { get; set; }
        /// <summary>
        /// ID of the board vendor
        /// OriginName: vendor_id, Units: , IsExtended: false
        /// </summary>
        public ushort VendorId { get; set; }
        /// <summary>
        /// ID of the product
        /// OriginName: product_id, Units: , IsExtended: false
        /// </summary>
        public ushort ProductId { get; set; }
        /// <summary>
        /// Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
        /// OriginName: flight_custom_version, Units: , IsExtended: false
        /// </summary>
        public byte[] FlightCustomVersion { get; } = new byte[8];
        /// <summary>
        /// Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
        /// OriginName: middleware_custom_version, Units: , IsExtended: false
        /// </summary>
        public byte[] MiddlewareCustomVersion { get; } = new byte[8];
        /// <summary>
        /// Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
        /// OriginName: os_custom_version, Units: , IsExtended: false
        /// </summary>
        public byte[] OsCustomVersion { get; } = new byte[8];
        /// <summary>
        /// UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise use uid)
        /// OriginName: uid2, Units: , IsExtended: true
        /// </summary>
        public byte[] Uid2 { get; } = new byte[18];
    }
    /// <summary>
    /// The location of a landing area captured from a downward facing camera
    ///  LANDING_TARGET
    /// </summary>
    public class LandingTargetPacket: PacketV2<LandingTargetPayload>
    {
	public const int PacketMessageId = 149;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 200;

        public override LandingTargetPayload Payload { get; } = new LandingTargetPayload();

        public override string Name => "LANDING_TARGET";
    }

    /// <summary>
    ///  LANDING_TARGET
    /// </summary>
    public class LandingTargetPayload : IPayload
    {
        public byte GetMaxByteSize() => 60;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            AngleX = BitConverter.ToSingle(buffer, index);index+=4;
            AngleY = BitConverter.ToSingle(buffer, index);index+=4;
            Distance = BitConverter.ToSingle(buffer, index);index+=4;
            SizeX = BitConverter.ToSingle(buffer, index);index+=4;
            SizeY = BitConverter.ToSingle(buffer, index);index+=4;
            TargetNum = (byte)buffer[index++];
            Frame = (MavFrame)buffer[index++];
            // extended field 'X' can be empty
            if (index >= endIndex) return;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            // extended field 'Y' can be empty
            if (index >= endIndex) return;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            // extended field 'Z' can be empty
            if (index >= endIndex) return;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            // extended field 'Q' can be empty
            if (index >= endIndex) return;
            arraySize = 4;
            for(var i=0;i<arraySize;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            // extended field 'Type' can be empty
            if (index >= endIndex) return;
            Type = (LandingTargetType)buffer[index++];
            // extended field 'PositionValid' can be empty
            if (index >= endIndex) return;
            PositionValid = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(AngleX).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(AngleY).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Distance).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(SizeX).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(SizeY).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TargetNum).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)Frame;index+=1;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(Q[i]).CopyTo(buffer, index);index+=4;
            }
            buffer[index] = (byte)Type;index+=1;
            BitConverter.GetBytes(PositionValid).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/60;
        }

        /// <summary>
        /// Timestamp (micros since boot or Unix epoch)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X-axis angular offset (in radians) of the target from the center of the image
        /// OriginName: angle_x, Units: rad, IsExtended: false
        /// </summary>
        public float AngleX { get; set; }
        /// <summary>
        /// Y-axis angular offset (in radians) of the target from the center of the image
        /// OriginName: angle_y, Units: rad, IsExtended: false
        /// </summary>
        public float AngleY { get; set; }
        /// <summary>
        /// Distance to the target from the vehicle in meters
        /// OriginName: distance, Units: m, IsExtended: false
        /// </summary>
        public float Distance { get; set; }
        /// <summary>
        /// Size in radians of target along x-axis
        /// OriginName: size_x, Units: rad, IsExtended: false
        /// </summary>
        public float SizeX { get; set; }
        /// <summary>
        /// Size in radians of target along y-axis
        /// OriginName: size_y, Units: rad, IsExtended: false
        /// </summary>
        public float SizeY { get; set; }
        /// <summary>
        /// The ID of the target if multiple targets are present
        /// OriginName: target_num, Units: , IsExtended: false
        /// </summary>
        public byte TargetNum { get; set; }
        /// <summary>
        /// MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
        /// OriginName: frame, Units: , IsExtended: false
        /// </summary>
        public MavFrame Frame { get; set; }
        /// <summary>
        /// X Position of the landing target on MAV_FRAME
        /// OriginName: x, Units: m, IsExtended: true
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y Position of the landing target on MAV_FRAME
        /// OriginName: y, Units: m, IsExtended: true
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z Position of the landing target on MAV_FRAME
        /// OriginName: z, Units: m, IsExtended: true
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        /// OriginName: q, Units: , IsExtended: true
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// LANDING_TARGET_TYPE enum specifying the type of landing target
        /// OriginName: type, Units: , IsExtended: true
        /// </summary>
        public LandingTargetType Type { get; set; }
        /// <summary>
        /// Boolean indicating known position (1) or default unkown position (0), for validation of positioning of the landing target
        /// OriginName: position_valid, Units: , IsExtended: true
        /// </summary>
        public byte PositionValid { get; set; }
    }
    /// <summary>
    /// Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS enum definition for further information. The innovaton test ratios show the magnitude of the sensor innovation divided by the innovation check threshold. Under normal operation the innovaton test ratios should be below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should be optional and controllable by the user.
    ///  ESTIMATOR_STATUS
    /// </summary>
    public class EstimatorStatusPacket: PacketV2<EstimatorStatusPayload>
    {
	public const int PacketMessageId = 230;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 163;

        public override EstimatorStatusPayload Payload { get; } = new EstimatorStatusPayload();

        public override string Name => "ESTIMATOR_STATUS";
    }

    /// <summary>
    ///  ESTIMATOR_STATUS
    /// </summary>
    public class EstimatorStatusPayload : IPayload
    {
        public byte GetMaxByteSize() => 42;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            VelRatio = BitConverter.ToSingle(buffer, index);index+=4;
            PosHorizRatio = BitConverter.ToSingle(buffer, index);index+=4;
            PosVertRatio = BitConverter.ToSingle(buffer, index);index+=4;
            MagRatio = BitConverter.ToSingle(buffer, index);index+=4;
            HaglRatio = BitConverter.ToSingle(buffer, index);index+=4;
            TasRatio = BitConverter.ToSingle(buffer, index);index+=4;
            PosHorizAccuracy = BitConverter.ToSingle(buffer, index);index+=4;
            PosVertAccuracy = BitConverter.ToSingle(buffer, index);index+=4;
            Flags = (EstimatorStatusFlags)BitConverter.ToUInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(VelRatio).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PosHorizRatio).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PosVertRatio).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(MagRatio).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(HaglRatio).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TasRatio).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PosHorizAccuracy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PosVertAccuracy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes((ushort)Flags).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/42;
        }

        /// <summary>
        /// Timestamp (micros since boot or Unix epoch)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Velocity innovation test ratio
        /// OriginName: vel_ratio, Units: , IsExtended: false
        /// </summary>
        public float VelRatio { get; set; }
        /// <summary>
        /// Horizontal position innovation test ratio
        /// OriginName: pos_horiz_ratio, Units: , IsExtended: false
        /// </summary>
        public float PosHorizRatio { get; set; }
        /// <summary>
        /// Vertical position innovation test ratio
        /// OriginName: pos_vert_ratio, Units: , IsExtended: false
        /// </summary>
        public float PosVertRatio { get; set; }
        /// <summary>
        /// Magnetometer innovation test ratio
        /// OriginName: mag_ratio, Units: , IsExtended: false
        /// </summary>
        public float MagRatio { get; set; }
        /// <summary>
        /// Height above terrain innovation test ratio
        /// OriginName: hagl_ratio, Units: , IsExtended: false
        /// </summary>
        public float HaglRatio { get; set; }
        /// <summary>
        /// True airspeed innovation test ratio
        /// OriginName: tas_ratio, Units: , IsExtended: false
        /// </summary>
        public float TasRatio { get; set; }
        /// <summary>
        /// Horizontal position 1-STD accuracy relative to the EKF local origin (m)
        /// OriginName: pos_horiz_accuracy, Units: m, IsExtended: false
        /// </summary>
        public float PosHorizAccuracy { get; set; }
        /// <summary>
        /// Vertical position 1-STD accuracy relative to the EKF local origin (m)
        /// OriginName: pos_vert_accuracy, Units: m, IsExtended: false
        /// </summary>
        public float PosVertAccuracy { get; set; }
        /// <summary>
        /// Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS.
        /// OriginName: flags, Units: , IsExtended: false
        /// </summary>
        public EstimatorStatusFlags Flags { get; set; }
    }
    /// <summary>
    ///  WIND_COV
    /// </summary>
    public class WindCovPacket: PacketV2<WindCovPayload>
    {
	public const int PacketMessageId = 231;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 105;

        public override WindCovPayload Payload { get; } = new WindCovPayload();

        public override string Name => "WIND_COV";
    }

    /// <summary>
    ///  WIND_COV
    /// </summary>
    public class WindCovPayload : IPayload
    {
        public byte GetMaxByteSize() => 40;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            WindX = BitConverter.ToSingle(buffer, index);index+=4;
            WindY = BitConverter.ToSingle(buffer, index);index+=4;
            WindZ = BitConverter.ToSingle(buffer, index);index+=4;
            VarHoriz = BitConverter.ToSingle(buffer, index);index+=4;
            VarVert = BitConverter.ToSingle(buffer, index);index+=4;
            WindAlt = BitConverter.ToSingle(buffer, index);index+=4;
            HorizAccuracy = BitConverter.ToSingle(buffer, index);index+=4;
            VertAccuracy = BitConverter.ToSingle(buffer, index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(WindX).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(WindY).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(WindZ).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(VarHoriz).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(VarVert).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(WindAlt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(HorizAccuracy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(VertAccuracy).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/40;
        }

        /// <summary>
        /// Timestamp (micros since boot or Unix epoch)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Wind in X (NED) direction in m/s
        /// OriginName: wind_x, Units: m/s, IsExtended: false
        /// </summary>
        public float WindX { get; set; }
        /// <summary>
        /// Wind in Y (NED) direction in m/s
        /// OriginName: wind_y, Units: m/s, IsExtended: false
        /// </summary>
        public float WindY { get; set; }
        /// <summary>
        /// Wind in Z (NED) direction in m/s
        /// OriginName: wind_z, Units: m/s, IsExtended: false
        /// </summary>
        public float WindZ { get; set; }
        /// <summary>
        /// Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
        /// OriginName: var_horiz, Units: m/s, IsExtended: false
        /// </summary>
        public float VarHoriz { get; set; }
        /// <summary>
        /// Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
        /// OriginName: var_vert, Units: m/s, IsExtended: false
        /// </summary>
        public float VarVert { get; set; }
        /// <summary>
        /// AMSL altitude (m) this measurement was taken at
        /// OriginName: wind_alt, Units: m, IsExtended: false
        /// </summary>
        public float WindAlt { get; set; }
        /// <summary>
        /// Horizontal speed 1-STD accuracy
        /// OriginName: horiz_accuracy, Units: m, IsExtended: false
        /// </summary>
        public float HorizAccuracy { get; set; }
        /// <summary>
        /// Vertical speed 1-STD accuracy
        /// OriginName: vert_accuracy, Units: m, IsExtended: false
        /// </summary>
        public float VertAccuracy { get; set; }
    }
    /// <summary>
    /// GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position estimate of the sytem.
    ///  GPS_INPUT
    /// </summary>
    public class GpsInputPacket: PacketV2<GpsInputPayload>
    {
	public const int PacketMessageId = 232;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 151;

        public override GpsInputPayload Payload { get; } = new GpsInputPayload();

        public override string Name => "GPS_INPUT";
    }

    /// <summary>
    ///  GPS_INPUT
    /// </summary>
    public class GpsInputPayload : IPayload
    {
        public byte GetMaxByteSize() => 63;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            TimeWeekMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToSingle(buffer, index);index+=4;
            Hdop = BitConverter.ToSingle(buffer, index);index+=4;
            Vdop = BitConverter.ToSingle(buffer, index);index+=4;
            Vn = BitConverter.ToSingle(buffer, index);index+=4;
            Ve = BitConverter.ToSingle(buffer, index);index+=4;
            Vd = BitConverter.ToSingle(buffer, index);index+=4;
            SpeedAccuracy = BitConverter.ToSingle(buffer, index);index+=4;
            HorizAccuracy = BitConverter.ToSingle(buffer, index);index+=4;
            VertAccuracy = BitConverter.ToSingle(buffer, index);index+=4;
            IgnoreFlags = (GpsInputIgnoreFlags)BitConverter.ToUInt16(buffer,index);index+=2;
            TimeWeek = BitConverter.ToUInt16(buffer,index);index+=2;
            GpsId = (byte)buffer[index++];
            FixType = (byte)buffer[index++];
            SatellitesVisible = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(TimeWeekMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Hdop).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vdop).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vn).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Ve).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vd).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(SpeedAccuracy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(HorizAccuracy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(VertAccuracy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes((ushort)IgnoreFlags).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TimeWeek).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(GpsId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(FixType).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(SatellitesVisible).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/63;
        }

        /// <summary>
        /// Timestamp (micros since boot or Unix epoch)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// GPS time (milliseconds from start of GPS week)
        /// OriginName: time_week_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeWeekMs { get; set; }
        /// <summary>
        /// Latitude (WGS84), in degrees * 1E7
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude (WGS84), in degrees * 1E7
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude (AMSL, not WGS84), in m (positive for up)
        /// OriginName: alt, Units: m, IsExtended: false
        /// </summary>
        public float Alt { get; set; }
        /// <summary>
        /// GPS HDOP horizontal dilution of position in m
        /// OriginName: hdop, Units: m, IsExtended: false
        /// </summary>
        public float Hdop { get; set; }
        /// <summary>
        /// GPS VDOP vertical dilution of position in m
        /// OriginName: vdop, Units: m, IsExtended: false
        /// </summary>
        public float Vdop { get; set; }
        /// <summary>
        /// GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        /// OriginName: vn, Units: m/s, IsExtended: false
        /// </summary>
        public float Vn { get; set; }
        /// <summary>
        /// GPS velocity in m/s in EAST direction in earth-fixed NED frame
        /// OriginName: ve, Units: m/s, IsExtended: false
        /// </summary>
        public float Ve { get; set; }
        /// <summary>
        /// GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        /// OriginName: vd, Units: m/s, IsExtended: false
        /// </summary>
        public float Vd { get; set; }
        /// <summary>
        /// GPS speed accuracy in m/s
        /// OriginName: speed_accuracy, Units: m/s, IsExtended: false
        /// </summary>
        public float SpeedAccuracy { get; set; }
        /// <summary>
        /// GPS horizontal accuracy in m
        /// OriginName: horiz_accuracy, Units: m, IsExtended: false
        /// </summary>
        public float HorizAccuracy { get; set; }
        /// <summary>
        /// GPS vertical accuracy in m
        /// OriginName: vert_accuracy, Units: m, IsExtended: false
        /// </summary>
        public float VertAccuracy { get; set; }
        /// <summary>
        /// Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided.
        /// OriginName: ignore_flags, Units: , IsExtended: false
        /// </summary>
        public GpsInputIgnoreFlags IgnoreFlags { get; set; }
        /// <summary>
        /// GPS week number
        /// OriginName: time_week, Units: , IsExtended: false
        /// </summary>
        public ushort TimeWeek { get; set; }
        /// <summary>
        /// ID of the GPS for multiple GPS inputs
        /// OriginName: gps_id, Units: , IsExtended: false
        /// </summary>
        public byte GpsId { get; set; }
        /// <summary>
        /// 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        /// OriginName: fix_type, Units: , IsExtended: false
        /// </summary>
        public byte FixType { get; set; }
        /// <summary>
        /// Number of satellites visible.
        /// OriginName: satellites_visible, Units: , IsExtended: false
        /// </summary>
        public byte SatellitesVisible { get; set; }
    }
    /// <summary>
    /// RTCM message for injecting into the onboard GPS (used for DGPS)
    ///  GPS_RTCM_DATA
    /// </summary>
    public class GpsRtcmDataPacket: PacketV2<GpsRtcmDataPayload>
    {
	public const int PacketMessageId = 233;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 35;

        public override GpsRtcmDataPayload Payload { get; } = new GpsRtcmDataPayload();

        public override string Name => "GPS_RTCM_DATA";
    }

    /// <summary>
    ///  GPS_RTCM_DATA
    /// </summary>
    public class GpsRtcmDataPayload : IPayload
    {
        public byte GetMaxByteSize() => 182;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Flags = (byte)buffer[index++];
            Len = (byte)buffer[index++];
            arraySize = /*ArrayLength*/180 - Math.Max(0,((/*PayloadByteSize*/182 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<180;i++)
            {
                Data[i] = default(byte);
            }
            for(var i=0;i<arraySize;i++)
            {
                Data[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Flags).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Len).CopyTo(buffer, index);index+=1;
            for(var i=0;i<180;i++)
            {
                buffer[index] = (byte)Data[i];index+=1;
            }
            return /*PayloadByteSize*/182;
        }

        /// <summary>
        /// LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer, while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment with a non full payload is received. This management is used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable transport delivery order.
        /// OriginName: flags, Units: , IsExtended: false
        /// </summary>
        public byte Flags { get; set; }
        /// <summary>
        /// data length
        /// OriginName: len, Units: bytes, IsExtended: false
        /// </summary>
        public byte Len { get; set; }
        /// <summary>
        /// RTCM message (may be fragmented)
        /// OriginName: data, Units: , IsExtended: false
        /// </summary>
        public byte[] Data { get; } = new byte[180];
    }
    /// <summary>
    /// Message appropriate for high latency connections like Iridium
    ///  HIGH_LATENCY
    /// </summary>
    public class HighLatencyPacket: PacketV2<HighLatencyPayload>
    {
	public const int PacketMessageId = 234;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 150;

        public override HighLatencyPayload Payload { get; } = new HighLatencyPayload();

        public override string Name => "HIGH_LATENCY";
    }

    /// <summary>
    ///  HIGH_LATENCY
    /// </summary>
    public class HighLatencyPayload : IPayload
    {
        public byte GetMaxByteSize() => 40;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            CustomMode = BitConverter.ToUInt32(buffer,index);index+=4;
            Latitude = BitConverter.ToInt32(buffer,index);index+=4;
            Longitude = BitConverter.ToInt32(buffer,index);index+=4;
            Roll = BitConverter.ToInt16(buffer,index);index+=2;
            Pitch = BitConverter.ToInt16(buffer,index);index+=2;
            Heading = BitConverter.ToUInt16(buffer,index);index+=2;
            HeadingSp = BitConverter.ToInt16(buffer,index);index+=2;
            AltitudeAmsl = BitConverter.ToInt16(buffer,index);index+=2;
            AltitudeSp = BitConverter.ToInt16(buffer,index);index+=2;
            WpDistance = BitConverter.ToUInt16(buffer,index);index+=2;
            BaseMode = (MavModeFlag)buffer[index++];
            LandedState = (MavLandedState)buffer[index++];
            Throttle = (sbyte)buffer[index++];
            Airspeed = (byte)buffer[index++];
            AirspeedSp = (byte)buffer[index++];
            Groundspeed = (byte)buffer[index++];
            ClimbRate = (sbyte)buffer[index++];
            GpsNsat = (byte)buffer[index++];
            GpsFixType = (GpsFixType)buffer[index++];
            BatteryRemaining = (byte)buffer[index++];
            Temperature = (sbyte)buffer[index++];
            TemperatureAir = (sbyte)buffer[index++];
            Failsafe = (byte)buffer[index++];
            WpNum = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(CustomMode).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Latitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Longitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Roll).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Pitch).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Heading).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(HeadingSp).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(AltitudeAmsl).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(AltitudeSp).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(WpDistance).CopyTo(buffer, index);index+=2;
            buffer[index] = (byte)BaseMode;index+=1;
            buffer[index] = (byte)LandedState;index+=1;
            BitConverter.GetBytes(Throttle).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Airspeed).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(AirspeedSp).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Groundspeed).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(ClimbRate).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(GpsNsat).CopyTo(buffer, index);index+=1;
            buffer[index] = (byte)GpsFixType;index+=1;
            BitConverter.GetBytes(BatteryRemaining).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TemperatureAir).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Failsafe).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(WpNum).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/40;
        }

        /// <summary>
        /// A bitfield for use for autopilot-specific flags.
        /// OriginName: custom_mode, Units: , IsExtended: false
        /// </summary>
        public uint CustomMode { get; set; }
        /// <summary>
        /// Latitude, expressed as degrees * 1E7
        /// OriginName: latitude, Units: degE7, IsExtended: false
        /// </summary>
        public int Latitude { get; set; }
        /// <summary>
        /// Longitude, expressed as degrees * 1E7
        /// OriginName: longitude, Units: degE7, IsExtended: false
        /// </summary>
        public int Longitude { get; set; }
        /// <summary>
        /// roll (centidegrees)
        /// OriginName: roll, Units: cdeg, IsExtended: false
        /// </summary>
        public short Roll { get; set; }
        /// <summary>
        /// pitch (centidegrees)
        /// OriginName: pitch, Units: cdeg, IsExtended: false
        /// </summary>
        public short Pitch { get; set; }
        /// <summary>
        /// heading (centidegrees)
        /// OriginName: heading, Units: cdeg, IsExtended: false
        /// </summary>
        public ushort Heading { get; set; }
        /// <summary>
        /// heading setpoint (centidegrees)
        /// OriginName: heading_sp, Units: cdeg, IsExtended: false
        /// </summary>
        public short HeadingSp { get; set; }
        /// <summary>
        /// Altitude above mean sea level (meters)
        /// OriginName: altitude_amsl, Units: m, IsExtended: false
        /// </summary>
        public short AltitudeAmsl { get; set; }
        /// <summary>
        /// Altitude setpoint relative to the home position (meters)
        /// OriginName: altitude_sp, Units: m, IsExtended: false
        /// </summary>
        public short AltitudeSp { get; set; }
        /// <summary>
        /// distance to target (meters)
        /// OriginName: wp_distance, Units: m, IsExtended: false
        /// </summary>
        public ushort WpDistance { get; set; }
        /// <summary>
        /// System mode bitfield, as defined by MAV_MODE_FLAG enum.
        /// OriginName: base_mode, Units: , IsExtended: false
        /// </summary>
        public MavModeFlag BaseMode { get; set; }
        /// <summary>
        /// The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
        /// OriginName: landed_state, Units: , IsExtended: false
        /// </summary>
        public MavLandedState LandedState { get; set; }
        /// <summary>
        /// throttle (percentage)
        /// OriginName: throttle, Units: %, IsExtended: false
        /// </summary>
        public sbyte Throttle { get; set; }
        /// <summary>
        /// airspeed (m/s)
        /// OriginName: airspeed, Units: m/s, IsExtended: false
        /// </summary>
        public byte Airspeed { get; set; }
        /// <summary>
        /// airspeed setpoint (m/s)
        /// OriginName: airspeed_sp, Units: m/s, IsExtended: false
        /// </summary>
        public byte AirspeedSp { get; set; }
        /// <summary>
        /// groundspeed (m/s)
        /// OriginName: groundspeed, Units: m/s, IsExtended: false
        /// </summary>
        public byte Groundspeed { get; set; }
        /// <summary>
        /// climb rate (m/s)
        /// OriginName: climb_rate, Units: m/s, IsExtended: false
        /// </summary>
        public sbyte ClimbRate { get; set; }
        /// <summary>
        /// Number of satellites visible. If unknown, set to 255
        /// OriginName: gps_nsat, Units: , IsExtended: false
        /// </summary>
        public byte GpsNsat { get; set; }
        /// <summary>
        /// See the GPS_FIX_TYPE enum.
        /// OriginName: gps_fix_type, Units: , IsExtended: false
        /// </summary>
        public GpsFixType GpsFixType { get; set; }
        /// <summary>
        /// Remaining battery (percentage)
        /// OriginName: battery_remaining, Units: %, IsExtended: false
        /// </summary>
        public byte BatteryRemaining { get; set; }
        /// <summary>
        /// Autopilot temperature (degrees C)
        /// OriginName: temperature, Units: degC, IsExtended: false
        /// </summary>
        public sbyte Temperature { get; set; }
        /// <summary>
        /// Air temperature (degrees C) from airspeed sensor
        /// OriginName: temperature_air, Units: degC, IsExtended: false
        /// </summary>
        public sbyte TemperatureAir { get; set; }
        /// <summary>
        /// failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
        /// OriginName: failsafe, Units: , IsExtended: false
        /// </summary>
        public byte Failsafe { get; set; }
        /// <summary>
        /// current waypoint number
        /// OriginName: wp_num, Units: , IsExtended: false
        /// </summary>
        public byte WpNum { get; set; }
    }
    /// <summary>
    /// WIP: Message appropriate for high latency connections like Iridium (version 2)
    ///  HIGH_LATENCY2
    /// </summary>
    public class HighLatency2Packet: PacketV2<HighLatency2Payload>
    {
	public const int PacketMessageId = 235;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 179;

        public override HighLatency2Payload Payload { get; } = new HighLatency2Payload();

        public override string Name => "HIGH_LATENCY2";
    }

    /// <summary>
    ///  HIGH_LATENCY2
    /// </summary>
    public class HighLatency2Payload : IPayload
    {
        public byte GetMaxByteSize() => 42;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Timestamp = BitConverter.ToUInt32(buffer,index);index+=4;
            Latitude = BitConverter.ToInt32(buffer,index);index+=4;
            Longitude = BitConverter.ToInt32(buffer,index);index+=4;
            CustomMode = BitConverter.ToUInt16(buffer,index);index+=2;
            Altitude = BitConverter.ToInt16(buffer,index);index+=2;
            TargetAltitude = BitConverter.ToInt16(buffer,index);index+=2;
            TargetDistance = BitConverter.ToUInt16(buffer,index);index+=2;
            WpNum = BitConverter.ToUInt16(buffer,index);index+=2;
            FailureFlags = (HlFailureFlag)BitConverter.ToUInt16(buffer,index);index+=2;
            Type = (MavType)buffer[index++];
            Autopilot = (MavAutopilot)buffer[index++];
            Heading = (byte)buffer[index++];
            TargetHeading = (byte)buffer[index++];
            Throttle = (byte)buffer[index++];
            Airspeed = (byte)buffer[index++];
            AirspeedSp = (byte)buffer[index++];
            Groundspeed = (byte)buffer[index++];
            Windspeed = (byte)buffer[index++];
            WindHeading = (byte)buffer[index++];
            Eph = (byte)buffer[index++];
            Epv = (byte)buffer[index++];
            TemperatureAir = (sbyte)buffer[index++];
            ClimbRate = (sbyte)buffer[index++];
            Battery = (sbyte)buffer[index++];
            Custom0 = (sbyte)buffer[index++];
            Custom1 = (sbyte)buffer[index++];
            Custom2 = (sbyte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Timestamp).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Latitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Longitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(CustomMode).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Altitude).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetAltitude).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetDistance).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(WpNum).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes((ushort)FailureFlags).CopyTo(buffer, index);index+=2;
            buffer[index] = (byte)Type;index+=1;
            buffer[index] = (byte)Autopilot;index+=1;
            BitConverter.GetBytes(Heading).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetHeading).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Throttle).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Airspeed).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(AirspeedSp).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Groundspeed).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Windspeed).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(WindHeading).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Eph).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Epv).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TemperatureAir).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(ClimbRate).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Battery).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Custom0).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Custom1).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Custom2).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/42;
        }

        /// <summary>
        /// Timestamp (milliseconds since boot or Unix epoch)
        /// OriginName: timestamp, Units: ms, IsExtended: false
        /// </summary>
        public uint Timestamp { get; set; }
        /// <summary>
        /// Latitude, expressed as degrees * 1E7
        /// OriginName: latitude, Units: degE7, IsExtended: false
        /// </summary>
        public int Latitude { get; set; }
        /// <summary>
        /// Longitude, expressed as degrees * 1E7
        /// OriginName: longitude, Units: degE7, IsExtended: false
        /// </summary>
        public int Longitude { get; set; }
        /// <summary>
        /// A bitfield for use for autopilot-specific flags (2 byte version).
        /// OriginName: custom_mode, Units: , IsExtended: false
        /// </summary>
        public ushort CustomMode { get; set; }
        /// <summary>
        /// Altitude above mean sea level
        /// OriginName: altitude, Units: m, IsExtended: false
        /// </summary>
        public short Altitude { get; set; }
        /// <summary>
        /// Altitude setpoint
        /// OriginName: target_altitude, Units: m, IsExtended: false
        /// </summary>
        public short TargetAltitude { get; set; }
        /// <summary>
        /// Distance to target waypoint or position (meters / 10)
        /// OriginName: target_distance, Units: dam, IsExtended: false
        /// </summary>
        public ushort TargetDistance { get; set; }
        /// <summary>
        /// Current waypoint number
        /// OriginName: wp_num, Units: , IsExtended: false
        /// </summary>
        public ushort WpNum { get; set; }
        /// <summary>
        /// Indicates failures as defined in HL_FAILURE_FLAG ENUM. 
        /// OriginName: failure_flags, Units: , IsExtended: false
        /// </summary>
        public HlFailureFlag FailureFlags { get; set; }
        /// <summary>
        /// Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        /// OriginName: type, Units: , IsExtended: false
        /// </summary>
        public MavType Type { get; set; }
        /// <summary>
        /// Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        /// OriginName: autopilot, Units: , IsExtended: false
        /// </summary>
        public MavAutopilot Autopilot { get; set; }
        /// <summary>
        /// Heading (degrees / 2)
        /// OriginName: heading, Units: deg/2, IsExtended: false
        /// </summary>
        public byte Heading { get; set; }
        /// <summary>
        /// Heading setpoint (degrees / 2)
        /// OriginName: target_heading, Units: deg/2, IsExtended: false
        /// </summary>
        public byte TargetHeading { get; set; }
        /// <summary>
        /// Throttle (percentage)
        /// OriginName: throttle, Units: %, IsExtended: false
        /// </summary>
        public byte Throttle { get; set; }
        /// <summary>
        /// Airspeed (m/s * 5)
        /// OriginName: airspeed, Units: m/s*5, IsExtended: false
        /// </summary>
        public byte Airspeed { get; set; }
        /// <summary>
        /// Airspeed setpoint (m/s * 5)
        /// OriginName: airspeed_sp, Units: m/s*5, IsExtended: false
        /// </summary>
        public byte AirspeedSp { get; set; }
        /// <summary>
        /// Groundspeed (m/s * 5)
        /// OriginName: groundspeed, Units: m/s*5, IsExtended: false
        /// </summary>
        public byte Groundspeed { get; set; }
        /// <summary>
        /// Windspeed (m/s * 5)
        /// OriginName: windspeed, Units: m/s*5, IsExtended: false
        /// </summary>
        public byte Windspeed { get; set; }
        /// <summary>
        /// Wind heading (deg / 2)
        /// OriginName: wind_heading, Units: deg/2, IsExtended: false
        /// </summary>
        public byte WindHeading { get; set; }
        /// <summary>
        /// Maximum error horizontal position since last message (m * 10)
        /// OriginName: eph, Units: dm, IsExtended: false
        /// </summary>
        public byte Eph { get; set; }
        /// <summary>
        /// Maximum error vertical position since last message (m * 10)
        /// OriginName: epv, Units: dm, IsExtended: false
        /// </summary>
        public byte Epv { get; set; }
        /// <summary>
        /// Air temperature (degrees C) from airspeed sensor
        /// OriginName: temperature_air, Units: degC, IsExtended: false
        /// </summary>
        public sbyte TemperatureAir { get; set; }
        /// <summary>
        /// Maximum climb rate magnitude since last message (m/s * 10)
        /// OriginName: climb_rate, Units: dm/s, IsExtended: false
        /// </summary>
        public sbyte ClimbRate { get; set; }
        /// <summary>
        /// Battery (percentage, -1 for DNU)
        /// OriginName: battery, Units: %, IsExtended: false
        /// </summary>
        public sbyte Battery { get; set; }
        /// <summary>
        /// Field for custom payload.
        /// OriginName: custom0, Units: , IsExtended: false
        /// </summary>
        public sbyte Custom0 { get; set; }
        /// <summary>
        /// Field for custom payload.
        /// OriginName: custom1, Units: , IsExtended: false
        /// </summary>
        public sbyte Custom1 { get; set; }
        /// <summary>
        /// Field for custom payload.
        /// OriginName: custom2, Units: , IsExtended: false
        /// </summary>
        public sbyte Custom2 { get; set; }
    }
    /// <summary>
    /// Vibration levels and accelerometer clipping
    ///  VIBRATION
    /// </summary>
    public class VibrationPacket: PacketV2<VibrationPayload>
    {
	public const int PacketMessageId = 241;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 90;

        public override VibrationPayload Payload { get; } = new VibrationPayload();

        public override string Name => "VIBRATION";
    }

    /// <summary>
    ///  VIBRATION
    /// </summary>
    public class VibrationPayload : IPayload
    {
        public byte GetMaxByteSize() => 32;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            VibrationX = BitConverter.ToSingle(buffer, index);index+=4;
            VibrationY = BitConverter.ToSingle(buffer, index);index+=4;
            VibrationZ = BitConverter.ToSingle(buffer, index);index+=4;
            Clipping0 = BitConverter.ToUInt32(buffer,index);index+=4;
            Clipping1 = BitConverter.ToUInt32(buffer,index);index+=4;
            Clipping2 = BitConverter.ToUInt32(buffer,index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(VibrationX).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(VibrationY).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(VibrationZ).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Clipping0).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Clipping1).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Clipping2).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/32;
        }

        /// <summary>
        /// Timestamp (micros since boot or Unix epoch)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Vibration levels on X-axis
        /// OriginName: vibration_x, Units: , IsExtended: false
        /// </summary>
        public float VibrationX { get; set; }
        /// <summary>
        /// Vibration levels on Y-axis
        /// OriginName: vibration_y, Units: , IsExtended: false
        /// </summary>
        public float VibrationY { get; set; }
        /// <summary>
        /// Vibration levels on Z-axis
        /// OriginName: vibration_z, Units: , IsExtended: false
        /// </summary>
        public float VibrationZ { get; set; }
        /// <summary>
        /// first accelerometer clipping count
        /// OriginName: clipping_0, Units: , IsExtended: false
        /// </summary>
        public uint Clipping0 { get; set; }
        /// <summary>
        /// second accelerometer clipping count
        /// OriginName: clipping_1, Units: , IsExtended: false
        /// </summary>
        public uint Clipping1 { get; set; }
        /// <summary>
        /// third accelerometer clipping count
        /// OriginName: clipping_2, Units: , IsExtended: false
        /// </summary>
        public uint Clipping2 { get; set; }
    }
    /// <summary>
    /// This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitely set by the operator before or after. The position the system will return to and land on. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
    ///  HOME_POSITION
    /// </summary>
    public class HomePositionPacket: PacketV2<HomePositionPayload>
    {
	public const int PacketMessageId = 242;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 104;

        public override HomePositionPayload Payload { get; } = new HomePositionPayload();

        public override string Name => "HOME_POSITION";
    }

    /// <summary>
    ///  HOME_POSITION
    /// </summary>
    public class HomePositionPayload : IPayload
    {
        public byte GetMaxByteSize() => 60;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Latitude = BitConverter.ToInt32(buffer,index);index+=4;
            Longitude = BitConverter.ToInt32(buffer,index);index+=4;
            Altitude = BitConverter.ToInt32(buffer,index);index+=4;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            arraySize = /*ArrayLength*/4 - Math.Max(0,((/*PayloadByteSize*/60 - payloadSize - /*ExtendedFieldsLength*/8)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<4;i++)
            {
                Q[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            ApproachX = BitConverter.ToSingle(buffer, index);index+=4;
            ApproachY = BitConverter.ToSingle(buffer, index);index+=4;
            ApproachZ = BitConverter.ToSingle(buffer, index);index+=4;
            // extended field 'TimeUsec' can be empty
            if (index >= endIndex) return;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Latitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Longitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Altitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(Q[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(ApproachX).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ApproachY).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ApproachZ).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            return /*PayloadByteSize*/60;
        }

        /// <summary>
        /// Latitude (WGS84), in degrees * 1E7
        /// OriginName: latitude, Units: degE7, IsExtended: false
        /// </summary>
        public int Latitude { get; set; }
        /// <summary>
        /// Longitude (WGS84, in degrees * 1E7
        /// OriginName: longitude, Units: degE7, IsExtended: false
        /// </summary>
        public int Longitude { get; set; }
        /// <summary>
        /// Altitude (AMSL), in meters * 1000 (positive for up)
        /// OriginName: altitude, Units: mm, IsExtended: false
        /// </summary>
        public int Altitude { get; set; }
        /// <summary>
        /// Local X position of this position in the local coordinate frame
        /// OriginName: x, Units: m, IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Local Y position of this position in the local coordinate frame
        /// OriginName: y, Units: m, IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Local Z position of this position in the local coordinate frame
        /// OriginName: z, Units: m, IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
        /// OriginName: q, Units: , IsExtended: false
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
        /// OriginName: approach_x, Units: m, IsExtended: false
        /// </summary>
        public float ApproachX { get; set; }
        /// <summary>
        /// Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
        /// OriginName: approach_y, Units: m, IsExtended: false
        /// </summary>
        public float ApproachY { get; set; }
        /// <summary>
        /// Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
        /// OriginName: approach_z, Units: m, IsExtended: false
        /// </summary>
        public float ApproachZ { get; set; }
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: true
        /// </summary>
        public ulong TimeUsec { get; set; }
    }
    /// <summary>
    /// The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitely set by the operator before or after. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
    ///  SET_HOME_POSITION
    /// </summary>
    public class SetHomePositionPacket: PacketV2<SetHomePositionPayload>
    {
	public const int PacketMessageId = 243;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 85;

        public override SetHomePositionPayload Payload { get; } = new SetHomePositionPayload();

        public override string Name => "SET_HOME_POSITION";
    }

    /// <summary>
    ///  SET_HOME_POSITION
    /// </summary>
    public class SetHomePositionPayload : IPayload
    {
        public byte GetMaxByteSize() => 61;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Latitude = BitConverter.ToInt32(buffer,index);index+=4;
            Longitude = BitConverter.ToInt32(buffer,index);index+=4;
            Altitude = BitConverter.ToInt32(buffer,index);index+=4;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            arraySize = /*ArrayLength*/4 - Math.Max(0,((/*PayloadByteSize*/61 - payloadSize - /*ExtendedFieldsLength*/8)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<4;i++)
            {
                Q[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            ApproachX = BitConverter.ToSingle(buffer, index);index+=4;
            ApproachY = BitConverter.ToSingle(buffer, index);index+=4;
            ApproachZ = BitConverter.ToSingle(buffer, index);index+=4;
            TargetSystem = (byte)buffer[index++];
            // extended field 'TimeUsec' can be empty
            if (index >= endIndex) return;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Latitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Longitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Altitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(Q[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(ApproachX).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ApproachY).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ApproachZ).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            return /*PayloadByteSize*/61;
        }

        /// <summary>
        /// Latitude (WGS84), in degrees * 1E7
        /// OriginName: latitude, Units: degE7, IsExtended: false
        /// </summary>
        public int Latitude { get; set; }
        /// <summary>
        /// Longitude (WGS84, in degrees * 1E7
        /// OriginName: longitude, Units: degE7, IsExtended: false
        /// </summary>
        public int Longitude { get; set; }
        /// <summary>
        /// Altitude (AMSL), in meters * 1000 (positive for up)
        /// OriginName: altitude, Units: mm, IsExtended: false
        /// </summary>
        public int Altitude { get; set; }
        /// <summary>
        /// Local X position of this position in the local coordinate frame
        /// OriginName: x, Units: m, IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Local Y position of this position in the local coordinate frame
        /// OriginName: y, Units: m, IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Local Z position of this position in the local coordinate frame
        /// OriginName: z, Units: m, IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
        /// OriginName: q, Units: , IsExtended: false
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
        /// OriginName: approach_x, Units: m, IsExtended: false
        /// </summary>
        public float ApproachX { get; set; }
        /// <summary>
        /// Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
        /// OriginName: approach_y, Units: m, IsExtended: false
        /// </summary>
        public float ApproachY { get; set; }
        /// <summary>
        /// Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
        /// OriginName: approach_z, Units: m, IsExtended: false
        /// </summary>
        public float ApproachZ { get; set; }
        /// <summary>
        /// System ID.
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: true
        /// </summary>
        public ulong TimeUsec { get; set; }
    }
    /// <summary>
    /// The interval between messages for a particular MAVLink message ID. This interface replaces DATA_STREAM
    ///  MESSAGE_INTERVAL
    /// </summary>
    public class MessageIntervalPacket: PacketV2<MessageIntervalPayload>
    {
	public const int PacketMessageId = 244;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 95;

        public override MessageIntervalPayload Payload { get; } = new MessageIntervalPayload();

        public override string Name => "MESSAGE_INTERVAL";
    }

    /// <summary>
    ///  MESSAGE_INTERVAL
    /// </summary>
    public class MessageIntervalPayload : IPayload
    {
        public byte GetMaxByteSize() => 6;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            IntervalUs = BitConverter.ToInt32(buffer,index);index+=4;
            MessageId = BitConverter.ToUInt16(buffer,index);index+=2;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(IntervalUs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(MessageId).CopyTo(buffer, index);index+=2;
            return /*PayloadByteSize*/6;
        }

        /// <summary>
        /// The interval between two messages, in microseconds. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.
        /// OriginName: interval_us, Units: us, IsExtended: false
        /// </summary>
        public int IntervalUs { get; set; }
        /// <summary>
        /// The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
        /// OriginName: message_id, Units: , IsExtended: false
        /// </summary>
        public ushort MessageId { get; set; }
    }
    /// <summary>
    /// Provides state for additional features
    ///  EXTENDED_SYS_STATE
    /// </summary>
    public class ExtendedSysStatePacket: PacketV2<ExtendedSysStatePayload>
    {
	public const int PacketMessageId = 245;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 130;

        public override ExtendedSysStatePayload Payload { get; } = new ExtendedSysStatePayload();

        public override string Name => "EXTENDED_SYS_STATE";
    }

    /// <summary>
    ///  EXTENDED_SYS_STATE
    /// </summary>
    public class ExtendedSysStatePayload : IPayload
    {
        public byte GetMaxByteSize() => 2;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            VtolState = (MavVtolState)buffer[index++];
            LandedState = (MavLandedState)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            buffer[index] = (byte)VtolState;index+=1;
            buffer[index] = (byte)LandedState;index+=1;
            return /*PayloadByteSize*/2;
        }

        /// <summary>
        /// The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
        /// OriginName: vtol_state, Units: , IsExtended: false
        /// </summary>
        public MavVtolState VtolState { get; set; }
        /// <summary>
        /// The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
        /// OriginName: landed_state, Units: , IsExtended: false
        /// </summary>
        public MavLandedState LandedState { get; set; }
    }
    /// <summary>
    /// The location and information of an ADSB vehicle
    ///  ADSB_VEHICLE
    /// </summary>
    public class AdsbVehiclePacket: PacketV2<AdsbVehiclePayload>
    {
	public const int PacketMessageId = 246;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 184;

        public override AdsbVehiclePayload Payload { get; } = new AdsbVehiclePayload();

        public override string Name => "ADSB_VEHICLE";
    }

    /// <summary>
    ///  ADSB_VEHICLE
    /// </summary>
    public class AdsbVehiclePayload : IPayload
    {
        public byte GetMaxByteSize() => 38;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            IcaoAddress = BitConverter.ToUInt32(buffer,index);index+=4;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Altitude = BitConverter.ToInt32(buffer,index);index+=4;
            Heading = BitConverter.ToUInt16(buffer,index);index+=2;
            HorVelocity = BitConverter.ToUInt16(buffer,index);index+=2;
            VerVelocity = BitConverter.ToInt16(buffer,index);index+=2;
            Flags = (AdsbFlags)BitConverter.ToUInt16(buffer,index);index+=2;
            Squawk = BitConverter.ToUInt16(buffer,index);index+=2;
            AltitudeType = (AdsbAltitudeType)buffer[index++];
            arraySize = /*ArrayLength*/9 - Math.Max(0,((/*PayloadByteSize*/38 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<9;i++)
            {
                Callsign[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,Callsign,0);
                index+=9;
            EmitterType = (AdsbEmitterType)buffer[index++];
            Tslc = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(IcaoAddress).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Altitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Heading).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(HorVelocity).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(VerVelocity).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes((ushort)Flags).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Squawk).CopyTo(buffer, index);index+=2;
            buffer[index] = (byte)AltitudeType;index+=1;
            Encoding.ASCII.GetBytes(Callsign,0,9,buffer,index);index+=9;
            buffer[index] = (byte)EmitterType;index+=1;
            BitConverter.GetBytes(Tslc).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/38;
        }

        /// <summary>
        /// ICAO address
        /// OriginName: ICAO_address, Units: , IsExtended: false
        /// </summary>
        public uint IcaoAddress { get; set; }
        /// <summary>
        /// Latitude, expressed as degrees * 1E7
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude, expressed as degrees * 1E7
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude(ASL) in millimeters
        /// OriginName: altitude, Units: mm, IsExtended: false
        /// </summary>
        public int Altitude { get; set; }
        /// <summary>
        /// Course over ground in centidegrees
        /// OriginName: heading, Units: cdeg, IsExtended: false
        /// </summary>
        public ushort Heading { get; set; }
        /// <summary>
        /// The horizontal velocity in centimeters/second
        /// OriginName: hor_velocity, Units: cm/s, IsExtended: false
        /// </summary>
        public ushort HorVelocity { get; set; }
        /// <summary>
        /// The vertical velocity in centimeters/second, positive is up
        /// OriginName: ver_velocity, Units: cm/s, IsExtended: false
        /// </summary>
        public short VerVelocity { get; set; }
        /// <summary>
        /// Flags to indicate various statuses including valid data fields
        /// OriginName: flags, Units: , IsExtended: false
        /// </summary>
        public AdsbFlags Flags { get; set; }
        /// <summary>
        /// Squawk code
        /// OriginName: squawk, Units: , IsExtended: false
        /// </summary>
        public ushort Squawk { get; set; }
        /// <summary>
        /// Type from ADSB_ALTITUDE_TYPE enum
        /// OriginName: altitude_type, Units: , IsExtended: false
        /// </summary>
        public AdsbAltitudeType AltitudeType { get; set; }
        /// <summary>
        /// The callsign, 8+null
        /// OriginName: callsign, Units: , IsExtended: false
        /// </summary>
        public char[] Callsign { get; } = new char[9];
        /// <summary>
        /// Type from ADSB_EMITTER_TYPE enum
        /// OriginName: emitter_type, Units: , IsExtended: false
        /// </summary>
        public AdsbEmitterType EmitterType { get; set; }
        /// <summary>
        /// Time since last communication in seconds
        /// OriginName: tslc, Units: s, IsExtended: false
        /// </summary>
        public byte Tslc { get; set; }
    }
    /// <summary>
    /// Information about a potential collision
    ///  COLLISION
    /// </summary>
    public class CollisionPacket: PacketV2<CollisionPayload>
    {
	public const int PacketMessageId = 247;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 81;

        public override CollisionPayload Payload { get; } = new CollisionPayload();

        public override string Name => "COLLISION";
    }

    /// <summary>
    ///  COLLISION
    /// </summary>
    public class CollisionPayload : IPayload
    {
        public byte GetMaxByteSize() => 19;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Id = BitConverter.ToUInt32(buffer,index);index+=4;
            TimeToMinimumDelta = BitConverter.ToSingle(buffer, index);index+=4;
            AltitudeMinimumDelta = BitConverter.ToSingle(buffer, index);index+=4;
            HorizontalMinimumDelta = BitConverter.ToSingle(buffer, index);index+=4;
            Src = (MavCollisionSrc)buffer[index++];
            Action = (MavCollisionAction)buffer[index++];
            ThreatLevel = (MavCollisionThreatLevel)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Id).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TimeToMinimumDelta).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(AltitudeMinimumDelta).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(HorizontalMinimumDelta).CopyTo(buffer, index);index+=4;
            buffer[index] = (byte)Src;index+=1;
            buffer[index] = (byte)Action;index+=1;
            buffer[index] = (byte)ThreatLevel;index+=1;
            return /*PayloadByteSize*/19;
        }

        /// <summary>
        /// Unique identifier, domain based on src field
        /// OriginName: id, Units: , IsExtended: false
        /// </summary>
        public uint Id { get; set; }
        /// <summary>
        /// Estimated time until collision occurs (seconds)
        /// OriginName: time_to_minimum_delta, Units: s, IsExtended: false
        /// </summary>
        public float TimeToMinimumDelta { get; set; }
        /// <summary>
        /// Closest vertical distance in meters between vehicle and object
        /// OriginName: altitude_minimum_delta, Units: m, IsExtended: false
        /// </summary>
        public float AltitudeMinimumDelta { get; set; }
        /// <summary>
        /// Closest horizontal distance in meteres between vehicle and object
        /// OriginName: horizontal_minimum_delta, Units: m, IsExtended: false
        /// </summary>
        public float HorizontalMinimumDelta { get; set; }
        /// <summary>
        /// Collision data source
        /// OriginName: src, Units: , IsExtended: false
        /// </summary>
        public MavCollisionSrc Src { get; set; }
        /// <summary>
        /// Action that is being taken to avoid this collision
        /// OriginName: action, Units: , IsExtended: false
        /// </summary>
        public MavCollisionAction Action { get; set; }
        /// <summary>
        /// How concerned the aircraft is about this collision
        /// OriginName: threat_level, Units: , IsExtended: false
        /// </summary>
        public MavCollisionThreatLevel ThreatLevel { get; set; }
    }
    /// <summary>
    /// Message implementing parts of the V2 payload specs in V1 frames for transitional support.
    ///  V2_EXTENSION
    /// </summary>
    public class V2ExtensionPacket: PacketV2<V2ExtensionPayload>
    {
	public const int PacketMessageId = 248;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 8;

        public override V2ExtensionPayload Payload { get; } = new V2ExtensionPayload();

        public override string Name => "V2_EXTENSION";
    }

    /// <summary>
    ///  V2_EXTENSION
    /// </summary>
    public class V2ExtensionPayload : IPayload
    {
        public byte GetMaxByteSize() => 254;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            MessageType = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetNetwork = (byte)buffer[index++];
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            arraySize = /*ArrayLength*/249 - Math.Max(0,((/*PayloadByteSize*/254 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<249;i++)
            {
                Payload[i] = default(byte);
            }
            for(var i=0;i<arraySize;i++)
            {
                Payload[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(MessageType).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetNetwork).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            for(var i=0;i<249;i++)
            {
                buffer[index] = (byte)Payload[i];index+=1;
            }
            return /*PayloadByteSize*/254;
        }

        /// <summary>
        /// A code that identifies the software component that understands this message (analogous to usb device classes or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
        /// OriginName: message_type, Units: , IsExtended: false
        /// </summary>
        public ushort MessageType { get; set; }
        /// <summary>
        /// Network ID (0 for broadcast)
        /// OriginName: target_network, Units: , IsExtended: false
        /// </summary>
        public byte TargetNetwork { get; set; }
        /// <summary>
        /// System ID (0 for broadcast)
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID (0 for broadcast)
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
        /// OriginName: payload, Units: , IsExtended: false
        /// </summary>
        public byte[] Payload { get; } = new byte[249];
    }
    /// <summary>
    /// Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    ///  MEMORY_VECT
    /// </summary>
    public class MemoryVectPacket: PacketV2<MemoryVectPayload>
    {
	public const int PacketMessageId = 249;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 204;

        public override MemoryVectPayload Payload { get; } = new MemoryVectPayload();

        public override string Name => "MEMORY_VECT";
    }

    /// <summary>
    ///  MEMORY_VECT
    /// </summary>
    public class MemoryVectPayload : IPayload
    {
        public byte GetMaxByteSize() => 36;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Address = BitConverter.ToUInt16(buffer,index);index+=2;
            Ver = (byte)buffer[index++];
            Type = (byte)buffer[index++];
            arraySize = /*ArrayLength*/32 - Math.Max(0,((/*PayloadByteSize*/36 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<32;i++)
            {
                Value[i] = default(sbyte);
            }
            for(var i=0;i<arraySize;i++)
            {
                Value[i] = (sbyte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Address).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Ver).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Type).CopyTo(buffer, index);index+=1;
            for(var i=0;i<32;i++)
            {
                buffer[index] = (byte)Value[i];index+=1;
            }
            return /*PayloadByteSize*/36;
        }

        /// <summary>
        /// Starting address of the debug variables
        /// OriginName: address, Units: , IsExtended: false
        /// </summary>
        public ushort Address { get; set; }
        /// <summary>
        /// Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
        /// OriginName: ver, Units: , IsExtended: false
        /// </summary>
        public byte Ver { get; set; }
        /// <summary>
        /// Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
        /// OriginName: type, Units: , IsExtended: false
        /// </summary>
        public byte Type { get; set; }
        /// <summary>
        /// Memory contents at specified address
        /// OriginName: value, Units: , IsExtended: false
        /// </summary>
        public sbyte[] Value { get; } = new sbyte[32];
    }
    /// <summary>
    ///  DEBUG_VECT
    /// </summary>
    public class DebugVectPacket: PacketV2<DebugVectPayload>
    {
	public const int PacketMessageId = 250;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 49;

        public override DebugVectPayload Payload { get; } = new DebugVectPayload();

        public override string Name => "DEBUG_VECT";
    }

    /// <summary>
    ///  DEBUG_VECT
    /// </summary>
    public class DebugVectPayload : IPayload
    {
        public byte GetMaxByteSize() => 30;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            arraySize = /*ArrayLength*/10 - Math.Max(0,((/*PayloadByteSize*/30 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<10;i++)
            {
                Name[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,Name,0);
                index+=10;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            Encoding.ASCII.GetBytes(Name,0,10,buffer,index);index+=10;
            return /*PayloadByteSize*/30;
        }

        /// <summary>
        /// Timestamp
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// x
        /// OriginName: x, Units: , IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// y
        /// OriginName: y, Units: , IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// z
        /// OriginName: z, Units: , IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Name
        /// OriginName: name, Units: , IsExtended: false
        /// </summary>
        public char[] Name { get; } = new char[10];
    }
    /// <summary>
    /// Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    ///  NAMED_VALUE_FLOAT
    /// </summary>
    public class NamedValueFloatPacket: PacketV2<NamedValueFloatPayload>
    {
	public const int PacketMessageId = 251;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 170;

        public override NamedValueFloatPayload Payload { get; } = new NamedValueFloatPayload();

        public override string Name => "NAMED_VALUE_FLOAT";
    }

    /// <summary>
    ///  NAMED_VALUE_FLOAT
    /// </summary>
    public class NamedValueFloatPayload : IPayload
    {
        public byte GetMaxByteSize() => 18;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Value = BitConverter.ToSingle(buffer, index);index+=4;
            arraySize = /*ArrayLength*/10 - Math.Max(0,((/*PayloadByteSize*/18 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<10;i++)
            {
                Name[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,Name,0);
                index+=10;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Value).CopyTo(buffer, index);index+=4;
            Encoding.ASCII.GetBytes(Name,0,10,buffer,index);index+=10;
            return /*PayloadByteSize*/18;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Floating point value
        /// OriginName: value, Units: , IsExtended: false
        /// </summary>
        public float Value { get; set; }
        /// <summary>
        /// Name of the debug variable
        /// OriginName: name, Units: , IsExtended: false
        /// </summary>
        public char[] Name { get; } = new char[10];
    }
    /// <summary>
    /// Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    ///  NAMED_VALUE_INT
    /// </summary>
    public class NamedValueIntPacket: PacketV2<NamedValueIntPayload>
    {
	public const int PacketMessageId = 252;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 44;

        public override NamedValueIntPayload Payload { get; } = new NamedValueIntPayload();

        public override string Name => "NAMED_VALUE_INT";
    }

    /// <summary>
    ///  NAMED_VALUE_INT
    /// </summary>
    public class NamedValueIntPayload : IPayload
    {
        public byte GetMaxByteSize() => 18;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Value = BitConverter.ToInt32(buffer,index);index+=4;
            arraySize = /*ArrayLength*/10 - Math.Max(0,((/*PayloadByteSize*/18 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<10;i++)
            {
                Name[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,Name,0);
                index+=10;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Value).CopyTo(buffer, index);index+=4;
            Encoding.ASCII.GetBytes(Name,0,10,buffer,index);index+=10;
            return /*PayloadByteSize*/18;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Signed integer value
        /// OriginName: value, Units: , IsExtended: false
        /// </summary>
        public int Value { get; set; }
        /// <summary>
        /// Name of the debug variable
        /// OriginName: name, Units: , IsExtended: false
        /// </summary>
        public char[] Name { get; } = new char[10];
    }
    /// <summary>
    /// Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
    ///  STATUSTEXT
    /// </summary>
    public class StatustextPacket: PacketV2<StatustextPayload>
    {
	public const int PacketMessageId = 253;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 83;

        public override StatustextPayload Payload { get; } = new StatustextPayload();

        public override string Name => "STATUSTEXT";
    }

    /// <summary>
    ///  STATUSTEXT
    /// </summary>
    public class StatustextPayload : IPayload
    {
        public byte GetMaxByteSize() => 51;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Severity = (MavSeverity)buffer[index++];
            arraySize = /*ArrayLength*/50 - Math.Max(0,((/*PayloadByteSize*/51 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<50;i++)
            {
                Text[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,Text,0);
                index+=50;
        }

        public int Serialize(byte[] buffer, int index)
        {
            buffer[index] = (byte)Severity;index+=1;
            Encoding.ASCII.GetBytes(Text,0,50,buffer,index);index+=50;
            return /*PayloadByteSize*/51;
        }

        /// <summary>
        /// Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
        /// OriginName: severity, Units: , IsExtended: false
        /// </summary>
        public MavSeverity Severity { get; set; }
        /// <summary>
        /// Status text message, without null termination character
        /// OriginName: text, Units: , IsExtended: false
        /// </summary>
        public char[] Text { get; } = new char[50];
    }
    /// <summary>
    /// Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
    ///  DEBUG
    /// </summary>
    public class DebugPacket: PacketV2<DebugPayload>
    {
	public const int PacketMessageId = 254;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 46;

        public override DebugPayload Payload { get; } = new DebugPayload();

        public override string Name => "DEBUG";
    }

    /// <summary>
    ///  DEBUG
    /// </summary>
    public class DebugPayload : IPayload
    {
        public byte GetMaxByteSize() => 9;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Value = BitConverter.ToSingle(buffer, index);index+=4;
            Ind = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Value).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Ind).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/9;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// DEBUG value
        /// OriginName: value, Units: , IsExtended: false
        /// </summary>
        public float Value { get; set; }
        /// <summary>
        /// index of debug variable
        /// OriginName: ind, Units: , IsExtended: false
        /// </summary>
        public byte Ind { get; set; }
    }
    /// <summary>
    /// Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable signing
    ///  SETUP_SIGNING
    /// </summary>
    public class SetupSigningPacket: PacketV2<SetupSigningPayload>
    {
	public const int PacketMessageId = 256;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 71;

        public override SetupSigningPayload Payload { get; } = new SetupSigningPayload();

        public override string Name => "SETUP_SIGNING";
    }

    /// <summary>
    ///  SETUP_SIGNING
    /// </summary>
    public class SetupSigningPayload : IPayload
    {
        public byte GetMaxByteSize() => 42;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            InitialTimestamp = BitConverter.ToUInt64(buffer,index);index+=8;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            arraySize = /*ArrayLength*/32 - Math.Max(0,((/*PayloadByteSize*/42 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<32;i++)
            {
                SecretKey[i] = default(byte);
            }
            for(var i=0;i<arraySize;i++)
            {
                SecretKey[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(InitialTimestamp).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            for(var i=0;i<32;i++)
            {
                buffer[index] = (byte)SecretKey[i];index+=1;
            }
            return /*PayloadByteSize*/42;
        }

        /// <summary>
        /// initial timestamp
        /// OriginName: initial_timestamp, Units: , IsExtended: false
        /// </summary>
        public ulong InitialTimestamp { get; set; }
        /// <summary>
        /// system id of the target
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// component ID of the target
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// signing key
        /// OriginName: secret_key, Units: , IsExtended: false
        /// </summary>
        public byte[] SecretKey { get; } = new byte[32];
    }
    /// <summary>
    /// Report button state change
    ///  BUTTON_CHANGE
    /// </summary>
    public class ButtonChangePacket: PacketV2<ButtonChangePayload>
    {
	public const int PacketMessageId = 257;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 131;

        public override ButtonChangePayload Payload { get; } = new ButtonChangePayload();

        public override string Name => "BUTTON_CHANGE";
    }

    /// <summary>
    ///  BUTTON_CHANGE
    /// </summary>
    public class ButtonChangePayload : IPayload
    {
        public byte GetMaxByteSize() => 9;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            LastChangeMs = BitConverter.ToUInt32(buffer,index);index+=4;
            State = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(LastChangeMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(State).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/9;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Time of last change of button state
        /// OriginName: last_change_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint LastChangeMs { get; set; }
        /// <summary>
        /// Bitmap state of buttons
        /// OriginName: state, Units: , IsExtended: false
        /// </summary>
        public byte State { get; set; }
    }
    /// <summary>
    /// Control vehicle tone generation (buzzer)
    ///  PLAY_TUNE
    /// </summary>
    public class PlayTunePacket: PacketV2<PlayTunePayload>
    {
	public const int PacketMessageId = 258;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 187;

        public override PlayTunePayload Payload { get; } = new PlayTunePayload();

        public override string Name => "PLAY_TUNE";
    }

    /// <summary>
    ///  PLAY_TUNE
    /// </summary>
    public class PlayTunePayload : IPayload
    {
        public byte GetMaxByteSize() => 32;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            arraySize = /*ArrayLength*/30 - Math.Max(0,((/*PayloadByteSize*/32 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<30;i++)
            {
                Tune[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,Tune,0);
                index+=30;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(Tune,0,30,buffer,index);index+=30;
            return /*PayloadByteSize*/32;
        }

        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// tune in board specific format
        /// OriginName: tune, Units: , IsExtended: false
        /// </summary>
        public char[] Tune { get; } = new char[30];
    }
    /// <summary>
    /// Information about a camera
    ///  CAMERA_INFORMATION
    /// </summary>
    public class CameraInformationPacket: PacketV2<CameraInformationPayload>
    {
	public const int PacketMessageId = 259;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 92;

        public override CameraInformationPayload Payload { get; } = new CameraInformationPayload();

        public override string Name => "CAMERA_INFORMATION";
    }

    /// <summary>
    ///  CAMERA_INFORMATION
    /// </summary>
    public class CameraInformationPayload : IPayload
    {
        public byte GetMaxByteSize() => 235;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            FirmwareVersion = BitConverter.ToUInt32(buffer,index);index+=4;
            FocalLength = BitConverter.ToSingle(buffer, index);index+=4;
            SensorSizeH = BitConverter.ToSingle(buffer, index);index+=4;
            SensorSizeV = BitConverter.ToSingle(buffer, index);index+=4;
            Flags = (CameraCapFlags)BitConverter.ToUInt32(buffer,index);index+=4;
            ResolutionH = BitConverter.ToUInt16(buffer,index);index+=2;
            ResolutionV = BitConverter.ToUInt16(buffer,index);index+=2;
            CamDefinitionVersion = BitConverter.ToUInt16(buffer,index);index+=2;
            arraySize = 32;
            for(var i=0;i<arraySize;i++)
            {
                VendorName[i] = (byte)buffer[index++];
            }
            arraySize = 32;
            for(var i=0;i<arraySize;i++)
            {
                ModelName[i] = (byte)buffer[index++];
            }
            LensId = (byte)buffer[index++];
            arraySize = /*ArrayLength*/140 - Math.Max(0,((/*PayloadByteSize*/235 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<140;i++)
            {
                CamDefinitionUri[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,CamDefinitionUri,0);
                index+=140;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(FirmwareVersion).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(FocalLength).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(SensorSizeH).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(SensorSizeV).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes((uint)Flags).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ResolutionH).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(ResolutionV).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(CamDefinitionVersion).CopyTo(buffer, index);index+=2;
            for(var i=0;i<32;i++)
            {
                buffer[index] = (byte)VendorName[i];index+=1;
            }
            for(var i=0;i<32;i++)
            {
                buffer[index] = (byte)ModelName[i];index+=1;
            }
            BitConverter.GetBytes(LensId).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(CamDefinitionUri,0,140,buffer,index);index+=140;
            return /*PayloadByteSize*/235;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Version of the camera firmware (v << 24 & 0xff = Dev, v << 16 & 0xff = Patch, v << 8 & 0xff = Minor, v & 0xff = Major)
        /// OriginName: firmware_version, Units: , IsExtended: false
        /// </summary>
        public uint FirmwareVersion { get; set; }
        /// <summary>
        /// Focal length in mm
        /// OriginName: focal_length, Units: mm, IsExtended: false
        /// </summary>
        public float FocalLength { get; set; }
        /// <summary>
        /// Image sensor size horizontal in mm
        /// OriginName: sensor_size_h, Units: mm, IsExtended: false
        /// </summary>
        public float SensorSizeH { get; set; }
        /// <summary>
        /// Image sensor size vertical in mm
        /// OriginName: sensor_size_v, Units: mm, IsExtended: false
        /// </summary>
        public float SensorSizeV { get; set; }
        /// <summary>
        /// CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities.
        /// OriginName: flags, Units: , IsExtended: false
        /// </summary>
        public CameraCapFlags Flags { get; set; }
        /// <summary>
        /// Image resolution in pixels horizontal
        /// OriginName: resolution_h, Units: pix, IsExtended: false
        /// </summary>
        public ushort ResolutionH { get; set; }
        /// <summary>
        /// Image resolution in pixels vertical
        /// OriginName: resolution_v, Units: pix, IsExtended: false
        /// </summary>
        public ushort ResolutionV { get; set; }
        /// <summary>
        /// Camera definition version (iteration)
        /// OriginName: cam_definition_version, Units: , IsExtended: false
        /// </summary>
        public ushort CamDefinitionVersion { get; set; }
        /// <summary>
        /// Name of the camera vendor
        /// OriginName: vendor_name, Units: , IsExtended: false
        /// </summary>
        public byte[] VendorName { get; } = new byte[32];
        /// <summary>
        /// Name of the camera model
        /// OriginName: model_name, Units: , IsExtended: false
        /// </summary>
        public byte[] ModelName { get; } = new byte[32];
        /// <summary>
        /// Reserved for a lens ID
        /// OriginName: lens_id, Units: , IsExtended: false
        /// </summary>
        public byte LensId { get; set; }
        /// <summary>
        /// Camera definition URI (if any, otherwise only basic functions will be available).
        /// OriginName: cam_definition_uri, Units: , IsExtended: false
        /// </summary>
        public char[] CamDefinitionUri { get; } = new char[140];
    }
    /// <summary>
    /// Settings of a camera, can be requested using MAV_CMD_REQUEST_CAMERA_SETTINGS.
    ///  CAMERA_SETTINGS
    /// </summary>
    public class CameraSettingsPacket: PacketV2<CameraSettingsPayload>
    {
	public const int PacketMessageId = 260;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 146;

        public override CameraSettingsPayload Payload { get; } = new CameraSettingsPayload();

        public override string Name => "CAMERA_SETTINGS";
    }

    /// <summary>
    ///  CAMERA_SETTINGS
    /// </summary>
    public class CameraSettingsPayload : IPayload
    {
        public byte GetMaxByteSize() => 5;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            ModeId = (CameraMode)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            buffer[index] = (byte)ModeId;index+=1;
            return /*PayloadByteSize*/5;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Camera mode (CAMERA_MODE)
        /// OriginName: mode_id, Units: , IsExtended: false
        /// </summary>
        public CameraMode ModeId { get; set; }
    }
    /// <summary>
    /// WIP: Information about a storage medium.
    ///  STORAGE_INFORMATION
    /// </summary>
    public class StorageInformationPacket: PacketV2<StorageInformationPayload>
    {
	public const int PacketMessageId = 261;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 179;

        public override StorageInformationPayload Payload { get; } = new StorageInformationPayload();

        public override string Name => "STORAGE_INFORMATION";
    }

    /// <summary>
    ///  STORAGE_INFORMATION
    /// </summary>
    public class StorageInformationPayload : IPayload
    {
        public byte GetMaxByteSize() => 27;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            TotalCapacity = BitConverter.ToSingle(buffer, index);index+=4;
            UsedCapacity = BitConverter.ToSingle(buffer, index);index+=4;
            AvailableCapacity = BitConverter.ToSingle(buffer, index);index+=4;
            ReadSpeed = BitConverter.ToSingle(buffer, index);index+=4;
            WriteSpeed = BitConverter.ToSingle(buffer, index);index+=4;
            StorageId = (byte)buffer[index++];
            StorageCount = (byte)buffer[index++];
            Status = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TotalCapacity).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(UsedCapacity).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(AvailableCapacity).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ReadSpeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(WriteSpeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(StorageId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(StorageCount).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Status).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/27;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Total capacity in MiB
        /// OriginName: total_capacity, Units: Mibytes, IsExtended: false
        /// </summary>
        public float TotalCapacity { get; set; }
        /// <summary>
        /// Used capacity in MiB
        /// OriginName: used_capacity, Units: Mibytes, IsExtended: false
        /// </summary>
        public float UsedCapacity { get; set; }
        /// <summary>
        /// Available capacity in MiB
        /// OriginName: available_capacity, Units: Mibytes, IsExtended: false
        /// </summary>
        public float AvailableCapacity { get; set; }
        /// <summary>
        /// Read speed in MiB/s
        /// OriginName: read_speed, Units: Mibytes/s, IsExtended: false
        /// </summary>
        public float ReadSpeed { get; set; }
        /// <summary>
        /// Write speed in MiB/s
        /// OriginName: write_speed, Units: Mibytes/s, IsExtended: false
        /// </summary>
        public float WriteSpeed { get; set; }
        /// <summary>
        /// Storage ID (1 for first, 2 for second, etc.)
        /// OriginName: storage_id, Units: , IsExtended: false
        /// </summary>
        public byte StorageId { get; set; }
        /// <summary>
        /// Number of storage devices
        /// OriginName: storage_count, Units: , IsExtended: false
        /// </summary>
        public byte StorageCount { get; set; }
        /// <summary>
        /// Status of storage (0 not available, 1 unformatted, 2 formatted)
        /// OriginName: status, Units: , IsExtended: false
        /// </summary>
        public byte Status { get; set; }
    }
    /// <summary>
    /// Information about the status of a capture
    ///  CAMERA_CAPTURE_STATUS
    /// </summary>
    public class CameraCaptureStatusPacket: PacketV2<CameraCaptureStatusPayload>
    {
	public const int PacketMessageId = 262;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 12;

        public override CameraCaptureStatusPayload Payload { get; } = new CameraCaptureStatusPayload();

        public override string Name => "CAMERA_CAPTURE_STATUS";
    }

    /// <summary>
    ///  CAMERA_CAPTURE_STATUS
    /// </summary>
    public class CameraCaptureStatusPayload : IPayload
    {
        public byte GetMaxByteSize() => 18;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            ImageInterval = BitConverter.ToSingle(buffer, index);index+=4;
            RecordingTimeMs = BitConverter.ToUInt32(buffer,index);index+=4;
            AvailableCapacity = BitConverter.ToSingle(buffer, index);index+=4;
            ImageStatus = (byte)buffer[index++];
            VideoStatus = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ImageInterval).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(RecordingTimeMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(AvailableCapacity).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ImageStatus).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(VideoStatus).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/18;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Image capture interval in seconds
        /// OriginName: image_interval, Units: s, IsExtended: false
        /// </summary>
        public float ImageInterval { get; set; }
        /// <summary>
        /// Time in milliseconds since recording started
        /// OriginName: recording_time_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint RecordingTimeMs { get; set; }
        /// <summary>
        /// Available storage capacity in MiB
        /// OriginName: available_capacity, Units: Mibytes, IsExtended: false
        /// </summary>
        public float AvailableCapacity { get; set; }
        /// <summary>
        /// Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)
        /// OriginName: image_status, Units: , IsExtended: false
        /// </summary>
        public byte ImageStatus { get; set; }
        /// <summary>
        /// Current status of video capturing (0: idle, 1: capture in progress)
        /// OriginName: video_status, Units: , IsExtended: false
        /// </summary>
        public byte VideoStatus { get; set; }
    }
    /// <summary>
    /// Information about a captured image
    ///  CAMERA_IMAGE_CAPTURED
    /// </summary>
    public class CameraImageCapturedPacket: PacketV2<CameraImageCapturedPayload>
    {
	public const int PacketMessageId = 263;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 133;

        public override CameraImageCapturedPayload Payload { get; } = new CameraImageCapturedPayload();

        public override string Name => "CAMERA_IMAGE_CAPTURED";
    }

    /// <summary>
    ///  CAMERA_IMAGE_CAPTURED
    /// </summary>
    public class CameraImageCapturedPayload : IPayload
    {
        public byte GetMaxByteSize() => 255;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUtc = BitConverter.ToUInt64(buffer,index);index+=8;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToInt32(buffer,index);index+=4;
            RelativeAlt = BitConverter.ToInt32(buffer,index);index+=4;
            arraySize = 4;
            for(var i=0;i<arraySize;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            ImageIndex = BitConverter.ToInt32(buffer,index);index+=4;
            CameraId = (byte)buffer[index++];
            CaptureResult = (sbyte)buffer[index++];
            arraySize = /*ArrayLength*/205 - Math.Max(0,((/*PayloadByteSize*/255 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<205;i++)
            {
                FileUrl[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,FileUrl,0);
                index+=205;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUtc).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(RelativeAlt).CopyTo(buffer, index);index+=4;
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(Q[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(ImageIndex).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(CameraId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(CaptureResult).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(FileUrl,0,205,buffer,index);index+=205;
            return /*PayloadByteSize*/255;
        }

        /// <summary>
        /// Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
        /// OriginName: time_utc, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUtc { get; set; }
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Latitude, expressed as degrees * 1E7 where image was taken
        /// OriginName: lat, Units: degE7, IsExtended: false
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude, expressed as degrees * 1E7 where capture was taken
        /// OriginName: lon, Units: degE7, IsExtended: false
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
        /// OriginName: alt, Units: m, IsExtended: false
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// Altitude above ground in meters, expressed as * 1E3 where image was taken
        /// OriginName: relative_alt, Units: m, IsExtended: false
        /// </summary>
        public int RelativeAlt { get; set; }
        /// <summary>
        /// Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
        /// OriginName: q, Units: , IsExtended: false
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Zero based index of this image (image count since armed -1)
        /// OriginName: image_index, Units: , IsExtended: false
        /// </summary>
        public int ImageIndex { get; set; }
        /// <summary>
        /// Camera ID (1 for first, 2 for second, etc.)
        /// OriginName: camera_id, Units: , IsExtended: false
        /// </summary>
        public byte CameraId { get; set; }
        /// <summary>
        /// Boolean indicating success (1) or failure (0) while capturing this image.
        /// OriginName: capture_result, Units: , IsExtended: false
        /// </summary>
        public sbyte CaptureResult { get; set; }
        /// <summary>
        /// URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
        /// OriginName: file_url, Units: , IsExtended: false
        /// </summary>
        public char[] FileUrl { get; } = new char[205];
    }
    /// <summary>
    /// WIP: Information about flight since last arming
    ///  FLIGHT_INFORMATION
    /// </summary>
    public class FlightInformationPacket: PacketV2<FlightInformationPayload>
    {
	public const int PacketMessageId = 264;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 49;

        public override FlightInformationPayload Payload { get; } = new FlightInformationPayload();

        public override string Name => "FLIGHT_INFORMATION";
    }

    /// <summary>
    ///  FLIGHT_INFORMATION
    /// </summary>
    public class FlightInformationPayload : IPayload
    {
        public byte GetMaxByteSize() => 28;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            ArmingTimeUtc = BitConverter.ToUInt64(buffer,index);index+=8;
            TakeoffTimeUtc = BitConverter.ToUInt64(buffer,index);index+=8;
            FlightUuid = BitConverter.ToUInt64(buffer,index);index+=8;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ArmingTimeUtc).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(TakeoffTimeUtc).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(FlightUuid).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/28;
        }

        /// <summary>
        /// Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
        /// OriginName: arming_time_utc, Units: us, IsExtended: false
        /// </summary>
        public ulong ArmingTimeUtc { get; set; }
        /// <summary>
        /// Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
        /// OriginName: takeoff_time_utc, Units: us, IsExtended: false
        /// </summary>
        public ulong TakeoffTimeUtc { get; set; }
        /// <summary>
        /// Universally unique identifier (UUID) of flight, should correspond to name of logfiles
        /// OriginName: flight_uuid, Units: , IsExtended: false
        /// </summary>
        public ulong FlightUuid { get; set; }
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
    }
    /// <summary>
    /// Orientation of a mount
    ///  MOUNT_ORIENTATION
    /// </summary>
    public class MountOrientationPacket: PacketV2<MountOrientationPayload>
    {
	public const int PacketMessageId = 265;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 26;

        public override MountOrientationPayload Payload { get; } = new MountOrientationPayload();

        public override string Name => "MOUNT_ORIENTATION";
    }

    /// <summary>
    ///  MOUNT_ORIENTATION
    /// </summary>
    public class MountOrientationPayload : IPayload
    {
        public byte GetMaxByteSize() => 20;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            // extended field 'YawAbsolute' can be empty
            if (index >= endIndex) return;
            YawAbsolute = BitConverter.ToSingle(buffer, index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Roll).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitch).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yaw).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(YawAbsolute).CopyTo(buffer, index);index+=4;
            return /*PayloadByteSize*/20;
        }

        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// OriginName: time_boot_ms, Units: ms, IsExtended: false
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Roll in global frame in degrees (set to NaN for invalid).
        /// OriginName: roll, Units: deg, IsExtended: false
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Pitch in global frame in degrees (set to NaN for invalid).
        /// OriginName: pitch, Units: deg, IsExtended: false
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Yaw relative to vehicle in degrees (set to NaN for invalid).
        /// OriginName: yaw, Units: deg, IsExtended: false
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// Yaw in absolute frame in degrees, North is 0 (set to NaN for invalid).
        /// OriginName: yaw_absolute, Units: deg, IsExtended: true
        /// </summary>
        public float YawAbsolute { get; set; }
    }
    /// <summary>
    /// A message containing logged data (see also MAV_CMD_LOGGING_START)
    ///  LOGGING_DATA
    /// </summary>
    public class LoggingDataPacket: PacketV2<LoggingDataPayload>
    {
	public const int PacketMessageId = 266;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 193;

        public override LoggingDataPayload Payload { get; } = new LoggingDataPayload();

        public override string Name => "LOGGING_DATA";
    }

    /// <summary>
    ///  LOGGING_DATA
    /// </summary>
    public class LoggingDataPayload : IPayload
    {
        public byte GetMaxByteSize() => 255;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Sequence = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            Length = (byte)buffer[index++];
            FirstMessageOffset = (byte)buffer[index++];
            arraySize = /*ArrayLength*/249 - Math.Max(0,((/*PayloadByteSize*/255 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<249;i++)
            {
                Data[i] = default(byte);
            }
            for(var i=0;i<arraySize;i++)
            {
                Data[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Sequence).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Length).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(FirstMessageOffset).CopyTo(buffer, index);index+=1;
            for(var i=0;i<249;i++)
            {
                buffer[index] = (byte)Data[i];index+=1;
            }
            return /*PayloadByteSize*/255;
        }

        /// <summary>
        /// sequence number (can wrap)
        /// OriginName: sequence, Units: , IsExtended: false
        /// </summary>
        public ushort Sequence { get; set; }
        /// <summary>
        /// system ID of the target
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// component ID of the target
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// data length
        /// OriginName: length, Units: bytes, IsExtended: false
        /// </summary>
        public byte Length { get; set; }
        /// <summary>
        /// offset into data where first message starts. This can be used for recovery, when a previous message got lost (set to 255 if no start exists).
        /// OriginName: first_message_offset, Units: bytes, IsExtended: false
        /// </summary>
        public byte FirstMessageOffset { get; set; }
        /// <summary>
        /// logged data
        /// OriginName: data, Units: , IsExtended: false
        /// </summary>
        public byte[] Data { get; } = new byte[249];
    }
    /// <summary>
    /// A message containing logged data which requires a LOGGING_ACK to be sent back
    ///  LOGGING_DATA_ACKED
    /// </summary>
    public class LoggingDataAckedPacket: PacketV2<LoggingDataAckedPayload>
    {
	public const int PacketMessageId = 267;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 35;

        public override LoggingDataAckedPayload Payload { get; } = new LoggingDataAckedPayload();

        public override string Name => "LOGGING_DATA_ACKED";
    }

    /// <summary>
    ///  LOGGING_DATA_ACKED
    /// </summary>
    public class LoggingDataAckedPayload : IPayload
    {
        public byte GetMaxByteSize() => 255;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Sequence = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            Length = (byte)buffer[index++];
            FirstMessageOffset = (byte)buffer[index++];
            arraySize = /*ArrayLength*/249 - Math.Max(0,((/*PayloadByteSize*/255 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<249;i++)
            {
                Data[i] = default(byte);
            }
            for(var i=0;i<arraySize;i++)
            {
                Data[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Sequence).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Length).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(FirstMessageOffset).CopyTo(buffer, index);index+=1;
            for(var i=0;i<249;i++)
            {
                buffer[index] = (byte)Data[i];index+=1;
            }
            return /*PayloadByteSize*/255;
        }

        /// <summary>
        /// sequence number (can wrap)
        /// OriginName: sequence, Units: , IsExtended: false
        /// </summary>
        public ushort Sequence { get; set; }
        /// <summary>
        /// system ID of the target
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// component ID of the target
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// data length
        /// OriginName: length, Units: bytes, IsExtended: false
        /// </summary>
        public byte Length { get; set; }
        /// <summary>
        /// offset into data where first message starts. This can be used for recovery, when a previous message got lost (set to 255 if no start exists).
        /// OriginName: first_message_offset, Units: bytes, IsExtended: false
        /// </summary>
        public byte FirstMessageOffset { get; set; }
        /// <summary>
        /// logged data
        /// OriginName: data, Units: , IsExtended: false
        /// </summary>
        public byte[] Data { get; } = new byte[249];
    }
    /// <summary>
    /// An ack for a LOGGING_DATA_ACKED message
    ///  LOGGING_ACK
    /// </summary>
    public class LoggingAckPacket: PacketV2<LoggingAckPayload>
    {
	public const int PacketMessageId = 268;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 14;

        public override LoggingAckPayload Payload { get; } = new LoggingAckPayload();

        public override string Name => "LOGGING_ACK";
    }

    /// <summary>
    ///  LOGGING_ACK
    /// </summary>
    public class LoggingAckPayload : IPayload
    {
        public byte GetMaxByteSize() => 4;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Sequence = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Sequence).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/4;
        }

        /// <summary>
        /// sequence number (must match the one in LOGGING_DATA_ACKED)
        /// OriginName: sequence, Units: , IsExtended: false
        /// </summary>
        public ushort Sequence { get; set; }
        /// <summary>
        /// system ID of the target
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// component ID of the target
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// WIP: Information about video stream
    ///  VIDEO_STREAM_INFORMATION
    /// </summary>
    public class VideoStreamInformationPacket: PacketV2<VideoStreamInformationPayload>
    {
	public const int PacketMessageId = 269;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 58;

        public override VideoStreamInformationPayload Payload { get; } = new VideoStreamInformationPayload();

        public override string Name => "VIDEO_STREAM_INFORMATION";
    }

    /// <summary>
    ///  VIDEO_STREAM_INFORMATION
    /// </summary>
    public class VideoStreamInformationPayload : IPayload
    {
        public byte GetMaxByteSize() => 246;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Framerate = BitConverter.ToSingle(buffer, index);index+=4;
            Bitrate = BitConverter.ToUInt32(buffer,index);index+=4;
            ResolutionH = BitConverter.ToUInt16(buffer,index);index+=2;
            ResolutionV = BitConverter.ToUInt16(buffer,index);index+=2;
            Rotation = BitConverter.ToUInt16(buffer,index);index+=2;
            CameraId = (byte)buffer[index++];
            Status = (byte)buffer[index++];
            arraySize = /*ArrayLength*/230 - Math.Max(0,((/*PayloadByteSize*/246 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<230;i++)
            {
                Uri[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,Uri,0);
                index+=230;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Framerate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Bitrate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ResolutionH).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(ResolutionV).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Rotation).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(CameraId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Status).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(Uri,0,230,buffer,index);index+=230;
            return /*PayloadByteSize*/246;
        }

        /// <summary>
        /// Frames per second
        /// OriginName: framerate, Units: Hz, IsExtended: false
        /// </summary>
        public float Framerate { get; set; }
        /// <summary>
        /// Bit rate in bits per second
        /// OriginName: bitrate, Units: bits/s, IsExtended: false
        /// </summary>
        public uint Bitrate { get; set; }
        /// <summary>
        /// Resolution horizontal in pixels
        /// OriginName: resolution_h, Units: pix, IsExtended: false
        /// </summary>
        public ushort ResolutionH { get; set; }
        /// <summary>
        /// Resolution vertical in pixels
        /// OriginName: resolution_v, Units: pix, IsExtended: false
        /// </summary>
        public ushort ResolutionV { get; set; }
        /// <summary>
        /// Video image rotation clockwise
        /// OriginName: rotation, Units: deg, IsExtended: false
        /// </summary>
        public ushort Rotation { get; set; }
        /// <summary>
        /// Camera ID (1 for first, 2 for second, etc.)
        /// OriginName: camera_id, Units: , IsExtended: false
        /// </summary>
        public byte CameraId { get; set; }
        /// <summary>
        /// Current status of video streaming (0: not running, 1: in progress)
        /// OriginName: status, Units: , IsExtended: false
        /// </summary>
        public byte Status { get; set; }
        /// <summary>
        /// Video stream URI
        /// OriginName: uri, Units: , IsExtended: false
        /// </summary>
        public char[] Uri { get; } = new char[230];
    }
    /// <summary>
    /// WIP: Message that sets video stream settings
    ///  SET_VIDEO_STREAM_SETTINGS
    /// </summary>
    public class SetVideoStreamSettingsPacket: PacketV2<SetVideoStreamSettingsPayload>
    {
	public const int PacketMessageId = 270;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 232;

        public override SetVideoStreamSettingsPayload Payload { get; } = new SetVideoStreamSettingsPayload();

        public override string Name => "SET_VIDEO_STREAM_SETTINGS";
    }

    /// <summary>
    ///  SET_VIDEO_STREAM_SETTINGS
    /// </summary>
    public class SetVideoStreamSettingsPayload : IPayload
    {
        public byte GetMaxByteSize() => 247;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Framerate = BitConverter.ToSingle(buffer, index);index+=4;
            Bitrate = BitConverter.ToUInt32(buffer,index);index+=4;
            ResolutionH = BitConverter.ToUInt16(buffer,index);index+=2;
            ResolutionV = BitConverter.ToUInt16(buffer,index);index+=2;
            Rotation = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            CameraId = (byte)buffer[index++];
            arraySize = /*ArrayLength*/230 - Math.Max(0,((/*PayloadByteSize*/247 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<230;i++)
            {
                Uri[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,Uri,0);
                index+=230;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Framerate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Bitrate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ResolutionH).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(ResolutionV).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Rotation).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(CameraId).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(Uri,0,230,buffer,index);index+=230;
            return /*PayloadByteSize*/247;
        }

        /// <summary>
        /// Frames per second (set to -1 for highest framerate possible)
        /// OriginName: framerate, Units: Hz, IsExtended: false
        /// </summary>
        public float Framerate { get; set; }
        /// <summary>
        /// Bit rate in bits per second (set to -1 for auto)
        /// OriginName: bitrate, Units: bits/s, IsExtended: false
        /// </summary>
        public uint Bitrate { get; set; }
        /// <summary>
        /// Resolution horizontal in pixels (set to -1 for highest resolution possible)
        /// OriginName: resolution_h, Units: pix, IsExtended: false
        /// </summary>
        public ushort ResolutionH { get; set; }
        /// <summary>
        /// Resolution vertical in pixels (set to -1 for highest resolution possible)
        /// OriginName: resolution_v, Units: pix, IsExtended: false
        /// </summary>
        public ushort ResolutionV { get; set; }
        /// <summary>
        /// Video image rotation clockwise (0-359 degrees)
        /// OriginName: rotation, Units: deg, IsExtended: false
        /// </summary>
        public ushort Rotation { get; set; }
        /// <summary>
        /// system ID of the target
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// component ID of the target
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Camera ID (1 for first, 2 for second, etc.)
        /// OriginName: camera_id, Units: , IsExtended: false
        /// </summary>
        public byte CameraId { get; set; }
        /// <summary>
        /// Video stream URI
        /// OriginName: uri, Units: , IsExtended: false
        /// </summary>
        public char[] Uri { get; } = new char[230];
    }
    /// <summary>
    /// Configure AP SSID and Password.
    ///  WIFI_CONFIG_AP
    /// </summary>
    public class WifiConfigApPacket: PacketV2<WifiConfigApPayload>
    {
	public const int PacketMessageId = 299;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 19;

        public override WifiConfigApPayload Payload { get; } = new WifiConfigApPayload();

        public override string Name => "WIFI_CONFIG_AP";
    }

    /// <summary>
    ///  WIFI_CONFIG_AP
    /// </summary>
    public class WifiConfigApPayload : IPayload
    {
        public byte GetMaxByteSize() => 96;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            arraySize = 32;
                Encoding.ASCII.GetChars(buffer, index,arraySize,Ssid,0);
                index+=32;
            arraySize = /*ArrayLength*/64 - Math.Max(0,((/*PayloadByteSize*/96 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<64;i++)
            {
                Password[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,Password,0);
                index+=64;
        }

        public int Serialize(byte[] buffer, int index)
        {
            Encoding.ASCII.GetBytes(Ssid,0,32,buffer,index);index+=32;
            Encoding.ASCII.GetBytes(Password,0,64,buffer,index);index+=64;
            return /*PayloadByteSize*/96;
        }

        /// <summary>
        /// Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
        /// OriginName: ssid, Units: , IsExtended: false
        /// </summary>
        public char[] Ssid { get; } = new char[32];
        /// <summary>
        /// Password. Leave it blank for an open AP.
        /// OriginName: password, Units: , IsExtended: false
        /// </summary>
        public char[] Password { get; } = new char[64];
    }
    /// <summary>
    /// WIP: Version and capability of protocol version. This message is the response to REQUEST_PROTOCOL_VERSION and is used as part of the handshaking to establish which MAVLink version should be used on the network. Every node should respond to REQUEST_PROTOCOL_VERSION to enable the handshaking. Library implementers should consider adding this into the default decoding state machine to allow the protocol core to respond directly.
    ///  PROTOCOL_VERSION
    /// </summary>
    public class ProtocolVersionPacket: PacketV2<ProtocolVersionPayload>
    {
	public const int PacketMessageId = 300;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 217;

        public override ProtocolVersionPayload Payload { get; } = new ProtocolVersionPayload();

        public override string Name => "PROTOCOL_VERSION";
    }

    /// <summary>
    ///  PROTOCOL_VERSION
    /// </summary>
    public class ProtocolVersionPayload : IPayload
    {
        public byte GetMaxByteSize() => 22;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Version = BitConverter.ToUInt16(buffer,index);index+=2;
            MinVersion = BitConverter.ToUInt16(buffer,index);index+=2;
            MaxVersion = BitConverter.ToUInt16(buffer,index);index+=2;
            arraySize = /*ArrayLength*/8 - Math.Max(0,((/*PayloadByteSize*/22 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<8;i++)
            {
                SpecVersionHash[i] = default(byte);
            }
            for(var i=0;i<arraySize;i++)
            {
                SpecVersionHash[i] = (byte)buffer[index++];
            }
            arraySize = 8;
            for(var i=0;i<arraySize;i++)
            {
                LibraryVersionHash[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Version).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(MinVersion).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(MaxVersion).CopyTo(buffer, index);index+=2;
            for(var i=0;i<8;i++)
            {
                buffer[index] = (byte)SpecVersionHash[i];index+=1;
            }
            for(var i=0;i<8;i++)
            {
                buffer[index] = (byte)LibraryVersionHash[i];index+=1;
            }
            return /*PayloadByteSize*/22;
        }

        /// <summary>
        /// Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
        /// OriginName: version, Units: , IsExtended: false
        /// </summary>
        public ushort Version { get; set; }
        /// <summary>
        /// Minimum MAVLink version supported
        /// OriginName: min_version, Units: , IsExtended: false
        /// </summary>
        public ushort MinVersion { get; set; }
        /// <summary>
        /// Maximum MAVLink version supported (set to the same value as version by default)
        /// OriginName: max_version, Units: , IsExtended: false
        /// </summary>
        public ushort MaxVersion { get; set; }
        /// <summary>
        /// The first 8 bytes (not characters printed in hex!) of the git hash.
        /// OriginName: spec_version_hash, Units: , IsExtended: false
        /// </summary>
        public byte[] SpecVersionHash { get; } = new byte[8];
        /// <summary>
        /// The first 8 bytes (not characters printed in hex!) of the git hash.
        /// OriginName: library_version_hash, Units: , IsExtended: false
        /// </summary>
        public byte[] LibraryVersionHash { get; } = new byte[8];
    }
    /// <summary>
    /// General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message "uavcan.protocol.NodeStatus" for the background information. The UAVCAN specification is available at http://uavcan.org.
    ///  UAVCAN_NODE_STATUS
    /// </summary>
    public class UavcanNodeStatusPacket: PacketV2<UavcanNodeStatusPayload>
    {
	public const int PacketMessageId = 310;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 28;

        public override UavcanNodeStatusPayload Payload { get; } = new UavcanNodeStatusPayload();

        public override string Name => "UAVCAN_NODE_STATUS";
    }

    /// <summary>
    ///  UAVCAN_NODE_STATUS
    /// </summary>
    public class UavcanNodeStatusPayload : IPayload
    {
        public byte GetMaxByteSize() => 17;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            UptimeSec = BitConverter.ToUInt32(buffer,index);index+=4;
            VendorSpecificStatusCode = BitConverter.ToUInt16(buffer,index);index+=2;
            Health = (UavcanNodeHealth)buffer[index++];
            Mode = (UavcanNodeMode)buffer[index++];
            SubMode = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(UptimeSec).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(VendorSpecificStatusCode).CopyTo(buffer, index);index+=2;
            buffer[index] = (byte)Health;index+=1;
            buffer[index] = (byte)Mode;index+=1;
            BitConverter.GetBytes(SubMode).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/17;
        }

        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// The number of seconds since the start-up of the node.
        /// OriginName: uptime_sec, Units: s, IsExtended: false
        /// </summary>
        public uint UptimeSec { get; set; }
        /// <summary>
        /// Vendor-specific status information.
        /// OriginName: vendor_specific_status_code, Units: , IsExtended: false
        /// </summary>
        public ushort VendorSpecificStatusCode { get; set; }
        /// <summary>
        /// Generalized node health status.
        /// OriginName: health, Units: , IsExtended: false
        /// </summary>
        public UavcanNodeHealth Health { get; set; }
        /// <summary>
        /// Generalized operating mode.
        /// OriginName: mode, Units: , IsExtended: false
        /// </summary>
        public UavcanNodeMode Mode { get; set; }
        /// <summary>
        /// Not used currently.
        /// OriginName: sub_mode, Units: , IsExtended: false
        /// </summary>
        public byte SubMode { get; set; }
    }
    /// <summary>
    /// General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN service "uavcan.protocol.GetNodeInfo" for the background information. This message should be emitted by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification is available at http://uavcan.org.
    ///  UAVCAN_NODE_INFO
    /// </summary>
    public class UavcanNodeInfoPacket: PacketV2<UavcanNodeInfoPayload>
    {
	public const int PacketMessageId = 311;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 95;

        public override UavcanNodeInfoPayload Payload { get; } = new UavcanNodeInfoPayload();

        public override string Name => "UAVCAN_NODE_INFO";
    }

    /// <summary>
    ///  UAVCAN_NODE_INFO
    /// </summary>
    public class UavcanNodeInfoPayload : IPayload
    {
        public byte GetMaxByteSize() => 116;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            UptimeSec = BitConverter.ToUInt32(buffer,index);index+=4;
            SwVcsCommit = BitConverter.ToUInt32(buffer,index);index+=4;
            arraySize = /*ArrayLength*/80 - Math.Max(0,((/*PayloadByteSize*/116 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<80;i++)
            {
                Name[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,Name,0);
                index+=80;
            HwVersionMajor = (byte)buffer[index++];
            HwVersionMinor = (byte)buffer[index++];
            arraySize = 16;
            for(var i=0;i<arraySize;i++)
            {
                HwUniqueId[i] = (byte)buffer[index++];
            }
            SwVersionMajor = (byte)buffer[index++];
            SwVersionMinor = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(UptimeSec).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(SwVcsCommit).CopyTo(buffer, index);index+=4;
            Encoding.ASCII.GetBytes(Name,0,80,buffer,index);index+=80;
            BitConverter.GetBytes(HwVersionMajor).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(HwVersionMinor).CopyTo(buffer, index);index+=1;
            for(var i=0;i<16;i++)
            {
                buffer[index] = (byte)HwUniqueId[i];index+=1;
            }
            BitConverter.GetBytes(SwVersionMajor).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(SwVersionMinor).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/116;
        }

        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// The number of seconds since the start-up of the node.
        /// OriginName: uptime_sec, Units: s, IsExtended: false
        /// </summary>
        public uint UptimeSec { get; set; }
        /// <summary>
        /// Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
        /// OriginName: sw_vcs_commit, Units: , IsExtended: false
        /// </summary>
        public uint SwVcsCommit { get; set; }
        /// <summary>
        /// Node name string. For example, "sapog.px4.io".
        /// OriginName: name, Units: , IsExtended: false
        /// </summary>
        public char[] Name { get; } = new char[80];
        /// <summary>
        /// Hardware major version number.
        /// OriginName: hw_version_major, Units: , IsExtended: false
        /// </summary>
        public byte HwVersionMajor { get; set; }
        /// <summary>
        /// Hardware minor version number.
        /// OriginName: hw_version_minor, Units: , IsExtended: false
        /// </summary>
        public byte HwVersionMinor { get; set; }
        /// <summary>
        /// Hardware unique 128-bit ID.
        /// OriginName: hw_unique_id, Units: , IsExtended: false
        /// </summary>
        public byte[] HwUniqueId { get; } = new byte[16];
        /// <summary>
        /// Software major version number.
        /// OriginName: sw_version_major, Units: , IsExtended: false
        /// </summary>
        public byte SwVersionMajor { get; set; }
        /// <summary>
        /// Software minor version number.
        /// OriginName: sw_version_minor, Units: , IsExtended: false
        /// </summary>
        public byte SwVersionMinor { get; set; }
    }
    /// <summary>
    /// Request to read the value of a parameter with the either the param_id string id or param_index.
    ///  PARAM_EXT_REQUEST_READ
    /// </summary>
    public class ParamExtRequestReadPacket: PacketV2<ParamExtRequestReadPayload>
    {
	public const int PacketMessageId = 320;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 243;

        public override ParamExtRequestReadPayload Payload { get; } = new ParamExtRequestReadPayload();

        public override string Name => "PARAM_EXT_REQUEST_READ";
    }

    /// <summary>
    ///  PARAM_EXT_REQUEST_READ
    /// </summary>
    public class ParamExtRequestReadPayload : IPayload
    {
        public byte GetMaxByteSize() => 20;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            ParamIndex = BitConverter.ToInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            arraySize = /*ArrayLength*/16 - Math.Max(0,((/*PayloadByteSize*/20 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<16;i++)
            {
                ParamId[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,ParamId,0);
                index+=16;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ParamIndex).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);index+=16;
            return /*PayloadByteSize*/20;
        }

        /// <summary>
        /// Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored)
        /// OriginName: param_index, Units: , IsExtended: false
        /// </summary>
        public short ParamIndex { get; set; }
        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// OriginName: param_id, Units: , IsExtended: false
        /// </summary>
        public char[] ParamId { get; } = new char[16];
    }
    /// <summary>
    /// Request all parameters of this component. After this request, all parameters are emitted.
    ///  PARAM_EXT_REQUEST_LIST
    /// </summary>
    public class ParamExtRequestListPacket: PacketV2<ParamExtRequestListPayload>
    {
	public const int PacketMessageId = 321;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 88;

        public override ParamExtRequestListPayload Payload { get; } = new ParamExtRequestListPayload();

        public override string Name => "PARAM_EXT_REQUEST_LIST";
    }

    /// <summary>
    ///  PARAM_EXT_REQUEST_LIST
    /// </summary>
    public class ParamExtRequestListPayload : IPayload
    {
        public byte GetMaxByteSize() => 2;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/2;
        }

        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows them to re-request missing parameters after a loss or timeout.
    ///  PARAM_EXT_VALUE
    /// </summary>
    public class ParamExtValuePacket: PacketV2<ParamExtValuePayload>
    {
	public const int PacketMessageId = 322;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 243;

        public override ParamExtValuePayload Payload { get; } = new ParamExtValuePayload();

        public override string Name => "PARAM_EXT_VALUE";
    }

    /// <summary>
    ///  PARAM_EXT_VALUE
    /// </summary>
    public class ParamExtValuePayload : IPayload
    {
        public byte GetMaxByteSize() => 149;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            ParamCount = BitConverter.ToUInt16(buffer,index);index+=2;
            ParamIndex = BitConverter.ToUInt16(buffer,index);index+=2;
            arraySize = 16;
                Encoding.ASCII.GetChars(buffer, index,arraySize,ParamId,0);
                index+=16;
            arraySize = /*ArrayLength*/128 - Math.Max(0,((/*PayloadByteSize*/149 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<128;i++)
            {
                ParamValue[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,ParamValue,0);
                index+=128;
            ParamType = (MavParamExtType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ParamCount).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(ParamIndex).CopyTo(buffer, index);index+=2;
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);index+=16;
            Encoding.ASCII.GetBytes(ParamValue,0,128,buffer,index);index+=128;
            buffer[index] = (byte)ParamType;index+=1;
            return /*PayloadByteSize*/149;
        }

        /// <summary>
        /// Total number of parameters
        /// OriginName: param_count, Units: , IsExtended: false
        /// </summary>
        public ushort ParamCount { get; set; }
        /// <summary>
        /// Index of this parameter
        /// OriginName: param_index, Units: , IsExtended: false
        /// </summary>
        public ushort ParamIndex { get; set; }
        /// <summary>
        /// Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// OriginName: param_id, Units: , IsExtended: false
        /// </summary>
        public char[] ParamId { get; } = new char[16];
        /// <summary>
        /// Parameter value
        /// OriginName: param_value, Units: , IsExtended: false
        /// </summary>
        public char[] ParamValue { get; } = new char[128];
        /// <summary>
        /// Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
        /// OriginName: param_type, Units: , IsExtended: false
        /// </summary>
        public MavParamExtType ParamType { get; set; }
    }
    /// <summary>
    /// Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when setting a parameter value and the new value is the same as the current value, you will immediately get a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive a PARAM_ACK_IN_PROGRESS in response.
    ///  PARAM_EXT_SET
    /// </summary>
    public class ParamExtSetPacket: PacketV2<ParamExtSetPayload>
    {
	public const int PacketMessageId = 323;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 78;

        public override ParamExtSetPayload Payload { get; } = new ParamExtSetPayload();

        public override string Name => "PARAM_EXT_SET";
    }

    /// <summary>
    ///  PARAM_EXT_SET
    /// </summary>
    public class ParamExtSetPayload : IPayload
    {
        public byte GetMaxByteSize() => 147;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            arraySize = 16;
                Encoding.ASCII.GetChars(buffer, index,arraySize,ParamId,0);
                index+=16;
            arraySize = /*ArrayLength*/128 - Math.Max(0,((/*PayloadByteSize*/147 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<128;i++)
            {
                ParamValue[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,ParamValue,0);
                index+=128;
            ParamType = (MavParamExtType)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);index+=16;
            Encoding.ASCII.GetBytes(ParamValue,0,128,buffer,index);index+=128;
            buffer[index] = (byte)ParamType;index+=1;
            return /*PayloadByteSize*/147;
        }

        /// <summary>
        /// System ID
        /// OriginName: target_system, Units: , IsExtended: false
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// OriginName: target_component, Units: , IsExtended: false
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// OriginName: param_id, Units: , IsExtended: false
        /// </summary>
        public char[] ParamId { get; } = new char[16];
        /// <summary>
        /// Parameter value
        /// OriginName: param_value, Units: , IsExtended: false
        /// </summary>
        public char[] ParamValue { get; } = new char[128];
        /// <summary>
        /// Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
        /// OriginName: param_type, Units: , IsExtended: false
        /// </summary>
        public MavParamExtType ParamType { get; set; }
    }
    /// <summary>
    /// Response from a PARAM_EXT_SET message.
    ///  PARAM_EXT_ACK
    /// </summary>
    public class ParamExtAckPacket: PacketV2<ParamExtAckPayload>
    {
	public const int PacketMessageId = 324;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 132;

        public override ParamExtAckPayload Payload { get; } = new ParamExtAckPayload();

        public override string Name => "PARAM_EXT_ACK";
    }

    /// <summary>
    ///  PARAM_EXT_ACK
    /// </summary>
    public class ParamExtAckPayload : IPayload
    {
        public byte GetMaxByteSize() => 146;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            arraySize = 16;
                Encoding.ASCII.GetChars(buffer, index,arraySize,ParamId,0);
                index+=16;
            arraySize = /*ArrayLength*/128 - Math.Max(0,((/*PayloadByteSize*/146 - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<128;i++)
            {
                ParamValue[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,ParamValue,0);
                index+=128;
            ParamType = (MavParamExtType)buffer[index++];
            ParamResult = (ParamAck)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);index+=16;
            Encoding.ASCII.GetBytes(ParamValue,0,128,buffer,index);index+=128;
            buffer[index] = (byte)ParamType;index+=1;
            buffer[index] = (byte)ParamResult;index+=1;
            return /*PayloadByteSize*/146;
        }

        /// <summary>
        /// Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// OriginName: param_id, Units: , IsExtended: false
        /// </summary>
        public char[] ParamId { get; } = new char[16];
        /// <summary>
        /// Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
        /// OriginName: param_value, Units: , IsExtended: false
        /// </summary>
        public char[] ParamValue { get; } = new char[128];
        /// <summary>
        /// Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
        /// OriginName: param_type, Units: , IsExtended: false
        /// </summary>
        public MavParamExtType ParamType { get; set; }
        /// <summary>
        /// Result code: see the PARAM_ACK enum for possible codes.
        /// OriginName: param_result, Units: , IsExtended: false
        /// </summary>
        public ParamAck ParamResult { get; set; }
    }
    /// <summary>
    /// Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
    ///  OBSTACLE_DISTANCE
    /// </summary>
    public class ObstacleDistancePacket: PacketV2<ObstacleDistancePayload>
    {
	public const int PacketMessageId = 330;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 23;

        public override ObstacleDistancePayload Payload { get; } = new ObstacleDistancePayload();

        public override string Name => "OBSTACLE_DISTANCE";
    }

    /// <summary>
    ///  OBSTACLE_DISTANCE
    /// </summary>
    public class ObstacleDistancePayload : IPayload
    {
        public byte GetMaxByteSize() => 158;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            arraySize = /*ArrayLength*/72 - Math.Max(0,((/*PayloadByteSize*/158 - payloadSize - /*ExtendedFieldsLength*/0)/2 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<72;i++)
            {
                Distances[i] = default(ushort);
            }
            for(var i=0;i<arraySize;i++)
            {
                Distances[i] = BitConverter.ToUInt16(buffer,index);index+=2;
            }
            MinDistance = BitConverter.ToUInt16(buffer,index);index+=2;
            MaxDistance = BitConverter.ToUInt16(buffer,index);index+=2;
            SensorType = (MavDistanceSensor)buffer[index++];
            Increment = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            for(var i=0;i<72;i++)
            {
                BitConverter.GetBytes(Distances[i]).CopyTo(buffer, index);index+=2;
            }
            BitConverter.GetBytes(MinDistance).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(MaxDistance).CopyTo(buffer, index);index+=2;
            buffer[index] = (byte)SensorType;index+=1;
            BitConverter.GetBytes(Increment).CopyTo(buffer, index);index+=1;
            return /*PayloadByteSize*/158;
        }

        /// <summary>
        /// Timestamp (microseconds since system boot or since UNIX epoch).
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Distance of obstacles around the UAV with index 0 corresponding to local North. A value of 0 means that the obstacle is right in front of the sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX for unknown/not used. In a array element, one unit corresponds to 1cm.
        /// OriginName: distances, Units: cm, IsExtended: false
        /// </summary>
        public ushort[] Distances { get; } = new ushort[72];
        /// <summary>
        /// Minimum distance the sensor can measure in centimeters.
        /// OriginName: min_distance, Units: cm, IsExtended: false
        /// </summary>
        public ushort MinDistance { get; set; }
        /// <summary>
        /// Maximum distance the sensor can measure in centimeters.
        /// OriginName: max_distance, Units: cm, IsExtended: false
        /// </summary>
        public ushort MaxDistance { get; set; }
        /// <summary>
        /// Class id of the distance sensor type.
        /// OriginName: sensor_type, Units: , IsExtended: false
        /// </summary>
        public MavDistanceSensor SensorType { get; set; }
        /// <summary>
        /// Angular width in degrees of each array element.
        /// OriginName: increment, Units: deg, IsExtended: false
        /// </summary>
        public byte Increment { get; set; }
    }
    /// <summary>
    /// Odometry message to communicate odometry information with an external interface. Fits ROS REP 147 standard for aerial vehicles (http://www.ros.org/reps/rep-0147.html).
    ///  ODOMETRY
    /// </summary>
    public class OdometryPacket: PacketV2<OdometryPayload>
    {
	public const int PacketMessageId = 331;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 58;

        public override OdometryPayload Payload { get; } = new OdometryPayload();

        public override string Name => "ODOMETRY";
    }

    /// <summary>
    ///  ODOMETRY
    /// </summary>
    public class OdometryPayload : IPayload
    {
        public byte GetMaxByteSize() => 230;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            arraySize = 4;
            for(var i=0;i<arraySize;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            Vx = BitConverter.ToSingle(buffer, index);index+=4;
            Vy = BitConverter.ToSingle(buffer, index);index+=4;
            Vz = BitConverter.ToSingle(buffer, index);index+=4;
            Rollspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Pitchspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Yawspeed = BitConverter.ToSingle(buffer, index);index+=4;
            arraySize = /*ArrayLength*/21 - Math.Max(0,((/*PayloadByteSize*/230 - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<21;i++)
            {
                PoseCovariance[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                PoseCovariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            arraySize = 21;
            for(var i=0;i<arraySize;i++)
            {
                TwistCovariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            FrameId = (MavFrame)buffer[index++];
            ChildFrameId = (MavFrame)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(Q[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(Vx).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vy).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Vz).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Rollspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitchspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yawspeed).CopyTo(buffer, index);index+=4;
            for(var i=0;i<21;i++)
            {
                BitConverter.GetBytes(PoseCovariance[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<21;i++)
            {
                BitConverter.GetBytes(TwistCovariance[i]).CopyTo(buffer, index);index+=4;
            }
            buffer[index] = (byte)FrameId;index+=1;
            buffer[index] = (byte)ChildFrameId;index+=1;
            return /*PayloadByteSize*/230;
        }

        /// <summary>
        /// Timestamp (microseconds since system boot or since UNIX epoch).
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X Position
        /// OriginName: x, Units: m, IsExtended: false
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y Position
        /// OriginName: y, Units: m, IsExtended: false
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z Position
        /// OriginName: z, Units: m, IsExtended: false
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
        /// OriginName: q, Units: , IsExtended: false
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// X linear speed
        /// OriginName: vx, Units: m/s, IsExtended: false
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Y linear speed
        /// OriginName: vy, Units: m/s, IsExtended: false
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Z linear speed
        /// OriginName: vz, Units: m/s, IsExtended: false
        /// </summary>
        public float Vz { get; set; }
        /// <summary>
        /// Roll angular speed
        /// OriginName: rollspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Rollspeed { get; set; }
        /// <summary>
        /// Pitch angular speed
        /// OriginName: pitchspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Pitchspeed { get; set; }
        /// <summary>
        /// Yaw angular speed
        /// OriginName: yawspeed, Units: rad/s, IsExtended: false
        /// </summary>
        public float Yawspeed { get; set; }
        /// <summary>
        /// Pose (states: x, y, z, roll, pitch, yaw) covariance matrix upper right triangle (first six entries are the first ROW, next five entries are the second ROW, etc.)
        /// OriginName: pose_covariance, Units: , IsExtended: false
        /// </summary>
        public float[] PoseCovariance { get; } = new float[21];
        /// <summary>
        /// Twist (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed) covariance matrix upper right triangle (first six entries are the first ROW, next five entries are the second ROW, etc.)
        /// OriginName: twist_covariance, Units: , IsExtended: false
        /// </summary>
        public float[] TwistCovariance { get; } = new float[21];
        /// <summary>
        /// Coordinate frame of reference for the pose data, as defined by MAV_FRAME enum.
        /// OriginName: frame_id, Units: , IsExtended: false
        /// </summary>
        public MavFrame FrameId { get; set; }
        /// <summary>
        /// Coordinate frame of reference for the velocity in free space (twist) data, as defined by MAV_FRAME enum.
        /// OriginName: child_frame_id, Units: , IsExtended: false
        /// </summary>
        public MavFrame ChildFrameId { get; set; }
    }
    /// <summary>
    /// WORK IN PROGRESS! DO NOT DEPLOY! Message to describe a trajectory in the local frame. Supported trajectory types are enumerated in MAV_TRAJECTORY_REPRESENTATION
    ///  TRAJECTORY
    /// </summary>
    public class TrajectoryPacket: PacketV2<TrajectoryPayload>
    {
	public const int PacketMessageId = 332;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 131;

        public override TrajectoryPayload Payload { get; } = new TrajectoryPayload();

        public override string Name => "TRAJECTORY";
    }

    /// <summary>
    ///  TRAJECTORY
    /// </summary>
    public class TrajectoryPayload : IPayload
    {
        public byte GetMaxByteSize() => 234;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            arraySize = /*ArrayLength*/11 - Math.Max(0,((/*PayloadByteSize*/234 - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<11;i++)
            {
                Point1[i] = default(float);
            }
            for(var i=0;i<arraySize;i++)
            {
                Point1[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            arraySize = 11;
            for(var i=0;i<arraySize;i++)
            {
                Point2[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            arraySize = 11;
            for(var i=0;i<arraySize;i++)
            {
                Point3[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            arraySize = 11;
            for(var i=0;i<arraySize;i++)
            {
                Point4[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            arraySize = 11;
            for(var i=0;i<arraySize;i++)
            {
                Point5[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            Type = (MavTrajectoryRepresentation)buffer[index++];
            arraySize = 5;
            for(var i=0;i<arraySize;i++)
            {
                PointValid[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            for(var i=0;i<11;i++)
            {
                BitConverter.GetBytes(Point1[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<11;i++)
            {
                BitConverter.GetBytes(Point2[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<11;i++)
            {
                BitConverter.GetBytes(Point3[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<11;i++)
            {
                BitConverter.GetBytes(Point4[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<11;i++)
            {
                BitConverter.GetBytes(Point5[i]).CopyTo(buffer, index);index+=4;
            }
            buffer[index] = (byte)Type;index+=1;
            for(var i=0;i<5;i++)
            {
                buffer[index] = (byte)PointValid[i];index+=1;
            }
            return /*PayloadByteSize*/234;
        }

        /// <summary>
        /// Timestamp (microseconds since system boot or since UNIX epoch).
        /// OriginName: time_usec, Units: us, IsExtended: false
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Depending on the type (see MAV_TRAJECTORY_REPRESENTATION)
        /// OriginName: point_1, Units: , IsExtended: false
        /// </summary>
        public float[] Point1 { get; } = new float[11];
        /// <summary>
        /// Depending on the type (see MAV_TRAJECTORY_REPRESENTATION)
        /// OriginName: point_2, Units: , IsExtended: false
        /// </summary>
        public float[] Point2 { get; } = new float[11];
        /// <summary>
        /// Depending on the type (see MAV_TRAJECTORY_REPRESENTATION)
        /// OriginName: point_3, Units: , IsExtended: false
        /// </summary>
        public float[] Point3 { get; } = new float[11];
        /// <summary>
        /// Depending on the type (see MAV_TRAJECTORY_REPRESENTATION)
        /// OriginName: point_4, Units: , IsExtended: false
        /// </summary>
        public float[] Point4 { get; } = new float[11];
        /// <summary>
        /// Depending on the type (see MAV_TRAJECTORY_REPRESENTATION)
        /// OriginName: point_5, Units: , IsExtended: false
        /// </summary>
        public float[] Point5 { get; } = new float[11];
        /// <summary>
        /// Waypoints, Bezier etc. see MAV_TRAJECTORY_REPRESENTATION
        /// OriginName: type, Units: , IsExtended: false
        /// </summary>
        public MavTrajectoryRepresentation Type { get; set; }
        /// <summary>
        /// States if respective point is valid (boolean)
        /// OriginName: point_valid, Units: , IsExtended: false
        /// </summary>
        public byte[] PointValid { get; } = new byte[5];
    }


#endregion


}
