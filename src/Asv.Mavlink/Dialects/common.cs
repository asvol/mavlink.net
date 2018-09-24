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

// This code was generate by tool Asv.Mavlink.Gen version v0.1.0-0-g8733181

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
            src.Register(()=>new TrajectoryRepresentationWaypointsPacket());
            src.Register(()=>new TrajectoryRepresentationBezierPacket());
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
        /// <summary>
        /// 0x8000000 Satellite Communication 
        /// MAV_SYS_STATUS_SENSOR_SATCOM
        /// </summary>
        MavSysStatusSensorSatcom = 134217728,
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
        /// Start orbiting on the circumference of a circle defined by the parameters. Setting any value NaN results in using defaults.
        /// Param 1 - Radius of the circle in meters. positive: Orbit clockwise. negative: Orbit counter-clockwise. 
        /// Param 2 - Velocity tangential in m/s. NaN: Vehicle configuration default.
        /// Param 3 - Yaw behavior of the vehicle. 0: vehicle front points to the center (default). 1: Hold last heading. 2: Leave yaw uncontrolled.
        /// Param 4 - Reserved (e.g. for dynamic center beacon options)
        /// Param 5 - Center point latitude (if no MAV_FRAME specified) / X coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.
        /// Param 6 - Center point longitude (if no MAV_FRAME specified) / Y coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.
        /// Param 7 - Center point altitude (AMSL) (if no MAV_FRAME specified) / Z coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.
        /// MAV_CMD_DO_ORBIT
        /// </summary>
        MavCmdDoOrbit = 34,
        /// <summary>
        /// Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
        /// Param 1 - Region of interest mode. (see MAV_ROI enum)
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
        /// Cycle a relay on and off for a desired number of cycles with a desired period.
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
        /// Mission command to safely abort an autonomous landing.
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
        /// Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
        /// Param 1 - Region of interest mode. (see MAV_ROI enum)
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
        /// Param 5 - roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
        /// Param 6 - pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
        /// Param 7 - yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
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
        /// Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera.
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
        /// Param 2 - Absolute altitude (AMSL) min, in meters - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit
        /// Param 3 - Absolute altitude (AMSL) max, in meters - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit
        /// Param 4 - Horizontal move limit (AMSL), in meters - if vehicle moves more than this distance from its location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit
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
        /// Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
        /// Param 1 - Mission sequence value to set
        /// Param 2 - Empty
        /// Param 3 - Empty
        /// Param 4 - Empty
        /// Param 5 - Empty
        /// Param 5 - Empty
        /// Param 6 - Empty
        /// Param 7 - Empty
        /// MAV_CMD_DO_SET_MISSION_CURRENT
        /// </summary>
        MavCmdDoSetMissionCurrent = 224,
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
        /// Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage.
        /// Param 1 - Storage ID (0 for all, 1 for first, 2 for second, etc.)
        /// Param 2 - 0: No Action 1: Request storage information
        /// Param 3 - Reserved (all remaining params)
        /// MAV_CMD_REQUEST_STORAGE_INFORMATION
        /// </summary>
        MavCmdRequestStorageInformation = 525,
        /// <summary>
        /// Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage.
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
        /// Request flight information (FLIGHT_INFORMATION)
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
        /// Re-request a CAMERA_IMAGE_CAPTURE packet. Use NAN for reserved values.
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
        /// Start video streaming
        /// Param 1 - Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
        /// Param 2 - Reserved
        /// MAV_CMD_VIDEO_START_STREAMING
        /// </summary>
        MavCmdVideoStartStreaming = 2502,
        /// <summary>
        /// Stop the current video streaming
        /// Param 1 - Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
        /// Param 2 - Reserved
        /// MAV_CMD_VIDEO_STOP_STREAMING
        /// </summary>
        MavCmdVideoStopStreaming = 2503,
        /// <summary>
        /// Request video stream information (VIDEO_STREAM_INFORMATION)
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
        /// Param 1 - Control transmission over high latency telemetry (0: stop, 1: start)
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
        /// Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
        ///         
        /// Param 1 - Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle
        /// MAV_CMD_ARM_AUTHORIZATION_REQUEST
        /// </summary>
        MavCmdArmAuthorizationRequest = 3001,
        /// <summary>
        /// This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.
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
        /// Delay mission state machine until gate has been reached.
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
        /// Param 3 - Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.
        /// Param 4 - Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.
        /// Param 5 - Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
        /// Param 6 - Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
        /// Param 7 - Altitude (AMSL), in meters
        /// MAV_CMD_PAYLOAD_PREPARE_DEPLOY
        /// </summary>
        MavCmdPayloadPrepareDeploy = 30001,
        /// <summary>
        /// Control the payload deployment.
        /// Param 1 - Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deployment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.
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
        /// Param 7 - Altitude (AMSL), in meters
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
        /// Param 7 - Altitude (AMSL), in meters
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
        /// Param 7 - Altitude (AMSL), in meters
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
        /// Param 7 - Altitude (AMSL), in meters
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
        /// Param 7 - Altitude (AMSL), in meters
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
        /// Param 7 - Altitude (AMSL), in meters
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
        /// Param 7 - Altitude (AMSL), in meters
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
        /// Param 7 - Altitude (AMSL), in meters
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
        /// Param 7 - Altitude (AMSL), in meters
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
        /// Param 7 - Altitude (AMSL), in meters
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
    /// A data stream is not a fixed set of messages, but rather a
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
    /// The ROI (region of interest) for the vehicle. This can be
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
    /// result in a MAVLink mission ack
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
        /// An unusual event has occurred, though not an error condition. This should be investigated for the root cause.
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
        /// Roll: 90, Pitch: 68, Yaw: 293
        /// MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293
        /// </summary>
        MavSensorRotationRoll90Pitch68Yaw293 = 38,
        /// <summary>
        /// Pitch: 315
        /// MAV_SENSOR_ROTATION_PITCH_315
        /// </summary>
        MavSensorRotationPitch315 = 39,
        /// <summary>
        /// Roll: 90, Pitch: 315
        /// MAV_SENSOR_ROTATION_ROLL_90_PITCH_315
        /// </summary>
        MavSensorRotationRoll90Pitch315 = 40,
        /// <summary>
        /// Custom orientation
        /// MAV_SENSOR_ROTATION_CUSTOM
        /// </summary>
        MavSensorRotationCustom = 100,
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
        /// Autopilot supports MAVLink version 2.
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
    /// Bitmap of options for the MAV_CMD_DO_REPOSITION
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


#endregion

#region Messages

    /// <summary>
    /// The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot).
    ///  HEARTBEAT
    /// </summary>
    public class HeartbeatPacket: PacketV2<HeartbeatPayload>
    {
        public override byte CrcEtra => 50;
        public override int MessageId => 0;
        public override HeartbeatPayload Payload { get; } = new HeartbeatPayload();

        public override string Name => "HEARTBEAT";
        public override string ToString()
        {
            var name = "HEARTBEAT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  HEARTBEAT
    /// </summary>
    public class HeartbeatPayload : IPayload
    {
        public byte ByteSize => 9;

        public int Deserialize(byte[] buffer, int index)
        {
            CustomMode = BitConverter.ToUInt32(buffer,index);index+=4;
            Type = (MavType)buffer[index++];
            Autopilot = (MavAutopilot)buffer[index++];
            BaseMode = (MavModeFlag)buffer[index++];
            SystemStatus = (MavState)buffer[index++];
            MavlinkVersion = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(CustomMode).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes((byte)Type).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)Autopilot).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)BaseMode).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)SystemStatus).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(MavlinkVersion).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// A bitfield for use for autopilot-specific flags
        /// 
        /// custom_mode
        /// </summary>
        public uint CustomMode { get; set; }
        /// <summary>
        /// Type of the MAV (quadrotor, helicopter, etc.)
        /// 
        /// type
        /// </summary>
        public MavType Type { get; set; }
        /// <summary>
        /// Autopilot type / class.
        /// 
        /// autopilot
        /// </summary>
        public MavAutopilot Autopilot { get; set; }
        /// <summary>
        /// System mode bitmap.
        /// 
        /// base_mode
        /// </summary>
        public MavModeFlag BaseMode { get; set; }
        /// <summary>
        /// System status flag.
        /// 
        /// system_status
        /// </summary>
        public MavState SystemStatus { get; set; }
        /// <summary>
        /// MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
        /// 
        /// mavlink_version
        /// </summary>
        public byte MavlinkVersion { get; set; }
    }
    /// <summary>
    /// The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows whether the system is currently active or not and if an emergency occurred. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occurred it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
    ///  SYS_STATUS
    /// </summary>
    public class SysStatusPacket: PacketV2<SysStatusPayload>
    {
        public override byte CrcEtra => 124;
        public override int MessageId => 1;
        public override SysStatusPayload Payload { get; } = new SysStatusPayload();

        public override string Name => "SYS_STATUS";
        public override string ToString()
        {
            var name = "SYS_STATUS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SYS_STATUS
    /// </summary>
    public class SysStatusPayload : IPayload
    {
        public byte ByteSize => 31;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
        /// 
        /// onboard_control_sensors_present
        /// </summary>
        public MavSysStatusSensor OnboardControlSensorsPresent { get; set; }
        /// <summary>
        /// Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
        /// 
        /// onboard_control_sensors_enabled
        /// </summary>
        public MavSysStatusSensor OnboardControlSensorsEnabled { get; set; }
        /// <summary>
        /// Bitmap showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled.
        /// 
        /// onboard_control_sensors_health
        /// </summary>
        public MavSysStatusSensor OnboardControlSensorsHealth { get; set; }
        /// <summary>
        /// Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
        /// d%
        /// load
        /// </summary>
        public ushort Load { get; set; }
        /// <summary>
        /// Battery voltage
        /// mV
        /// voltage_battery
        /// </summary>
        public ushort VoltageBattery { get; set; }
        /// <summary>
        /// Battery current, -1: autopilot does not measure the current
        /// cA
        /// current_battery
        /// </summary>
        public short CurrentBattery { get; set; }
        /// <summary>
        /// Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        /// c%
        /// drop_rate_comm
        /// </summary>
        public ushort DropRateComm { get; set; }
        /// <summary>
        /// Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        /// 
        /// errors_comm
        /// </summary>
        public ushort ErrorsComm { get; set; }
        /// <summary>
        /// Autopilot-specific errors
        /// 
        /// errors_count1
        /// </summary>
        public ushort ErrorsCount1 { get; set; }
        /// <summary>
        /// Autopilot-specific errors
        /// 
        /// errors_count2
        /// </summary>
        public ushort ErrorsCount2 { get; set; }
        /// <summary>
        /// Autopilot-specific errors
        /// 
        /// errors_count3
        /// </summary>
        public ushort ErrorsCount3 { get; set; }
        /// <summary>
        /// Autopilot-specific errors
        /// 
        /// errors_count4
        /// </summary>
        public ushort ErrorsCount4 { get; set; }
        /// <summary>
        /// Remaining battery energy, -1: autopilot estimate the remaining battery
        /// %
        /// battery_remaining
        /// </summary>
        public sbyte BatteryRemaining { get; set; }
    }
    /// <summary>
    /// The system time is the time of the master clock, typically the computer clock of the main onboard computer.
    ///  SYSTEM_TIME
    /// </summary>
    public class SystemTimePacket: PacketV2<SystemTimePayload>
    {
        public override byte CrcEtra => 137;
        public override int MessageId => 2;
        public override SystemTimePayload Payload { get; } = new SystemTimePayload();

        public override string Name => "SYSTEM_TIME";
        public override string ToString()
        {
            var name = "SYSTEM_TIME".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SYSTEM_TIME
    /// </summary>
    public class SystemTimePayload : IPayload
    {
        public byte ByteSize => 12;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUnixUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUnixUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX epoch time).
        /// us
        /// time_unix_usec
        /// </summary>
        public ulong TimeUnixUsec { get; set; }
        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
    }
    /// <summary>
    /// A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections.
    ///  PING
    /// </summary>
    public class PingPacket: PacketV2<PingPayload>
    {
        public override byte CrcEtra => 237;
        public override int MessageId => 4;
        public override PingPayload Payload { get; } = new PingPayload();

        public override string Name => "PING";
        public override string ToString()
        {
            var name = "PING".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  PING
    /// </summary>
    public class PingPayload : IPayload
    {
        public byte ByteSize => 14;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Seq = BitConverter.ToUInt32(buffer,index);index+=4;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// PING sequence
        /// 
        /// seq
        /// </summary>
        public uint Seq { get; set; }
        /// <summary>
        /// 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Request to control this MAV
    ///  CHANGE_OPERATOR_CONTROL
    /// </summary>
    public class ChangeOperatorControlPacket: PacketV2<ChangeOperatorControlPayload>
    {
        public override byte CrcEtra => 217;
        public override int MessageId => 5;
        public override ChangeOperatorControlPayload Payload { get; } = new ChangeOperatorControlPayload();

        public override string Name => "CHANGE_OPERATOR_CONTROL";
        public override string ToString()
        {
            var name = "CHANGE_OPERATOR_CONTROL".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  CHANGE_OPERATOR_CONTROL
    /// </summary>
    public class ChangeOperatorControlPayload : IPayload
    {
        public byte ByteSize => 28;

        public int Deserialize(byte[] buffer, int index)
        {
            TargetSystem = (byte)buffer[index++];
            ControlRequest = (byte)buffer[index++];
            Version = (byte)buffer[index++];
                Encoding.ASCII.GetChars(buffer, index,25,Passkey,0);
                index+=25;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(ControlRequest).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Version).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(Passkey,0,25,buffer,index);
            index+=25;
            return ByteSize;
        }

        /// <summary>
        /// System the GCS requests control for
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// 0: request control of this MAV, 1: Release control of this MAV
        /// 
        /// control_request
        /// </summary>
        public byte ControlRequest { get; set; }
        /// <summary>
        /// 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
        /// rad
        /// version
        /// </summary>
        public byte Version { get; set; }
        /// <summary>
        /// Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
        /// 
        /// passkey
        /// </summary>
        public char[] Passkey { get; } = new char[25];
    }
    /// <summary>
    /// Accept / deny control of this MAV
    ///  CHANGE_OPERATOR_CONTROL_ACK
    /// </summary>
    public class ChangeOperatorControlAckPacket: PacketV2<ChangeOperatorControlAckPayload>
    {
        public override byte CrcEtra => 104;
        public override int MessageId => 6;
        public override ChangeOperatorControlAckPayload Payload { get; } = new ChangeOperatorControlAckPayload();

        public override string Name => "CHANGE_OPERATOR_CONTROL_ACK";
        public override string ToString()
        {
            var name = "CHANGE_OPERATOR_CONTROL_ACK".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  CHANGE_OPERATOR_CONTROL_ACK
    /// </summary>
    public class ChangeOperatorControlAckPayload : IPayload
    {
        public byte ByteSize => 3;

        public int Deserialize(byte[] buffer, int index)
        {
            GcsSystemId = (byte)buffer[index++];
            ControlRequest = (byte)buffer[index++];
            Ack = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(GcsSystemId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(ControlRequest).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Ack).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// ID of the GCS this message 
        /// 
        /// gcs_system_id
        /// </summary>
        public byte GcsSystemId { get; set; }
        /// <summary>
        /// 0: request control of this MAV, 1: Release control of this MAV
        /// 
        /// control_request
        /// </summary>
        public byte ControlRequest { get; set; }
        /// <summary>
        /// 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
        /// 
        /// ack
        /// </summary>
        public byte Ack { get; set; }
    }
    /// <summary>
    /// Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.
    ///  AUTH_KEY
    /// </summary>
    public class AuthKeyPacket: PacketV2<AuthKeyPayload>
    {
        public override byte CrcEtra => 119;
        public override int MessageId => 7;
        public override AuthKeyPayload Payload { get; } = new AuthKeyPayload();

        public override string Name => "AUTH_KEY";
        public override string ToString()
        {
            var name = "AUTH_KEY".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  AUTH_KEY
    /// </summary>
    public class AuthKeyPayload : IPayload
    {
        public byte ByteSize => 32;

        public int Deserialize(byte[] buffer, int index)
        {
                Encoding.ASCII.GetChars(buffer, index,32,Key,0);
                index+=32;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            Encoding.ASCII.GetBytes(Key,0,32,buffer,index);
            index+=32;
            return ByteSize;
        }

        /// <summary>
        /// key
        /// 
        /// key
        /// </summary>
        public char[] Key { get; } = new char[32];
    }
    /// <summary>
    /// Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.
    ///  SET_MODE
    /// </summary>
    public class SetModePacket: PacketV2<SetModePayload>
    {
        public override byte CrcEtra => 89;
        public override int MessageId => 11;
        public override SetModePayload Payload { get; } = new SetModePayload();

        public override string Name => "SET_MODE";
        public override string ToString()
        {
            var name = "SET_MODE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SET_MODE
    /// </summary>
    public class SetModePayload : IPayload
    {
        public byte ByteSize => 6;

        public int Deserialize(byte[] buffer, int index)
        {
            CustomMode = BitConverter.ToUInt32(buffer,index);index+=4;
            TargetSystem = (byte)buffer[index++];
            BaseMode = (MavMode)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(CustomMode).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)BaseMode).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// The new autopilot-specific mode. This field can be ignored by an autopilot.
        /// 
        /// custom_mode
        /// </summary>
        public uint CustomMode { get; set; }
        /// <summary>
        /// The system setting the mode
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// The new base mode.
        /// 
        /// base_mode
        /// </summary>
        public MavMode BaseMode { get; set; }
    }
    /// <summary>
    /// Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also https://mavlink.io/en/protocol/parameter.html for a full documentation of QGroundControl and IMU code.
    ///  PARAM_REQUEST_READ
    /// </summary>
    public class ParamRequestReadPacket: PacketV2<ParamRequestReadPayload>
    {
        public override byte CrcEtra => 214;
        public override int MessageId => 20;
        public override ParamRequestReadPayload Payload { get; } = new ParamRequestReadPayload();

        public override string Name => "PARAM_REQUEST_READ";
        public override string ToString()
        {
            var name = "PARAM_REQUEST_READ".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  PARAM_REQUEST_READ
    /// </summary>
    public class ParamRequestReadPayload : IPayload
    {
        public byte ByteSize => 20;

        public int Deserialize(byte[] buffer, int index)
        {
            ParamIndex = BitConverter.ToInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
                Encoding.ASCII.GetChars(buffer, index,16,ParamId,0);
                index+=16;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ParamIndex).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);
            index+=16;
            return ByteSize;
        }

        /// <summary>
        /// Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
        /// 
        /// param_index
        /// </summary>
        public short ParamIndex { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// 
        /// param_id
        /// </summary>
        public char[] ParamId { get; } = new char[16];
    }
    /// <summary>
    /// Request all parameters of this component. After this request, all parameters are emitted.
    ///  PARAM_REQUEST_LIST
    /// </summary>
    public class ParamRequestListPacket: PacketV2<ParamRequestListPayload>
    {
        public override byte CrcEtra => 159;
        public override int MessageId => 21;
        public override ParamRequestListPayload Payload { get; } = new ParamRequestListPayload();

        public override string Name => "PARAM_REQUEST_LIST";
        public override string ToString()
        {
            var name = "PARAM_REQUEST_LIST".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  PARAM_REQUEST_LIST
    /// </summary>
    public class ParamRequestListPayload : IPayload
    {
        public byte ByteSize => 2;

        public int Deserialize(byte[] buffer, int index)
        {
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout.
    ///  PARAM_VALUE
    /// </summary>
    public class ParamValuePacket: PacketV2<ParamValuePayload>
    {
        public override byte CrcEtra => 220;
        public override int MessageId => 22;
        public override ParamValuePayload Payload { get; } = new ParamValuePayload();

        public override string Name => "PARAM_VALUE";
        public override string ToString()
        {
            var name = "PARAM_VALUE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  PARAM_VALUE
    /// </summary>
    public class ParamValuePayload : IPayload
    {
        public byte ByteSize => 25;

        public int Deserialize(byte[] buffer, int index)
        {
            ParamValue = BitConverter.ToSingle(buffer, index);index+=4;
            ParamCount = BitConverter.ToUInt16(buffer,index);index+=2;
            ParamIndex = BitConverter.ToUInt16(buffer,index);index+=2;
                Encoding.ASCII.GetChars(buffer, index,16,ParamId,0);
                index+=16;
            ParamType = (MavParamType)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ParamValue).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ParamCount).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(ParamIndex).CopyTo(buffer, index);index+=2;
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);
            index+=16;
            BitConverter.GetBytes((byte)ParamType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Onboard parameter value
        /// 
        /// param_value
        /// </summary>
        public float ParamValue { get; set; }
        /// <summary>
        /// Total number of onboard parameters
        /// 
        /// param_count
        /// </summary>
        public ushort ParamCount { get; set; }
        /// <summary>
        /// Index of this onboard parameter
        /// 
        /// param_index
        /// </summary>
        public ushort ParamIndex { get; set; }
        /// <summary>
        /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// 
        /// param_id
        /// </summary>
        public char[] ParamId { get; } = new char[16];
        /// <summary>
        /// Onboard parameter type.
        /// 
        /// param_type
        /// </summary>
        public MavParamType ParamType { get; set; }
    }
    /// <summary>
    /// Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component should acknowledge the new parameter value by sending a param_value message to all communication partners. This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message.
    ///  PARAM_SET
    /// </summary>
    public class ParamSetPacket: PacketV2<ParamSetPayload>
    {
        public override byte CrcEtra => 168;
        public override int MessageId => 23;
        public override ParamSetPayload Payload { get; } = new ParamSetPayload();

        public override string Name => "PARAM_SET";
        public override string ToString()
        {
            var name = "PARAM_SET".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  PARAM_SET
    /// </summary>
    public class ParamSetPayload : IPayload
    {
        public byte ByteSize => 23;

        public int Deserialize(byte[] buffer, int index)
        {
            ParamValue = BitConverter.ToSingle(buffer, index);index+=4;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
                Encoding.ASCII.GetChars(buffer, index,16,ParamId,0);
                index+=16;
            ParamType = (MavParamType)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ParamValue).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);
            index+=16;
            BitConverter.GetBytes((byte)ParamType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Onboard parameter value
        /// 
        /// param_value
        /// </summary>
        public float ParamValue { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// 
        /// param_id
        /// </summary>
        public char[] ParamId { get; } = new char[16];
        /// <summary>
        /// Onboard parameter type.
        /// 
        /// param_type
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
        public override byte CrcEtra => 24;
        public override int MessageId => 24;
        public override GpsRawIntPayload Payload { get; } = new GpsRawIntPayload();

        public override string Name => "GPS_RAW_INT";
        public override string ToString()
        {
            var name = "GPS_RAW_INT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  GPS_RAW_INT
    /// </summary>
    public class GpsRawIntPayload : IPayload
    {
        public byte ByteSize => 30;

        public int Deserialize(byte[] buffer, int index)
        {
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
            AltEllipsoid = BitConverter.ToInt32(buffer,index);index+=4;
            HAcc = BitConverter.ToUInt32(buffer,index);index+=4;
            VAcc = BitConverter.ToUInt32(buffer,index);index+=4;
            VelAcc = BitConverter.ToUInt32(buffer,index);index+=4;
            HdgAcc = BitConverter.ToUInt32(buffer,index);index+=4;
            return ByteSize;
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
            BitConverter.GetBytes((byte)FixType).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(SatellitesVisible).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(AltEllipsoid).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(HAcc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(VAcc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(VelAcc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(HdgAcc).CopyTo(buffer, index);index+=4;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Latitude (WGS84, EGM96 ellipsoid)
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude (WGS84, EGM96 ellipsoid)
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude (AMSL). Positive for up. Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
        /// mm
        /// alt
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
        /// 
        /// eph
        /// </summary>
        public ushort Eph { get; set; }
        /// <summary>
        /// GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
        /// 
        /// epv
        /// </summary>
        public ushort Epv { get; set; }
        /// <summary>
        /// GPS ground speed. If unknown, set to: UINT16_MAX
        /// cm/s
        /// vel
        /// </summary>
        public ushort Vel { get; set; }
        /// <summary>
        /// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        /// cdeg
        /// cog
        /// </summary>
        public ushort Cog { get; set; }
        /// <summary>
        /// GPS fix type.
        /// 
        /// fix_type
        /// </summary>
        public GpsFixType FixType { get; set; }
        /// <summary>
        /// Number of satellites visible. If unknown, set to 255
        /// 
        /// satellites_visible
        /// </summary>
        public byte SatellitesVisible { get; set; }
        /// <summary>
        /// Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
        /// mm
        /// alt_ellipsoid
        /// </summary>
        public int AltEllipsoid { get; set; }
        /// <summary>
        /// Position uncertainty. Positive for up.
        /// mm
        /// h_acc
        /// </summary>
        public uint HAcc { get; set; }
        /// <summary>
        /// Altitude uncertainty. Positive for up.
        /// mm
        /// v_acc
        /// </summary>
        public uint VAcc { get; set; }
        /// <summary>
        /// Speed uncertainty. Positive for up.
        /// mm
        /// vel_acc
        /// </summary>
        public uint VelAcc { get; set; }
        /// <summary>
        /// Heading / track uncertainty
        /// degE5
        /// hdg_acc
        /// </summary>
        public uint HdgAcc { get; set; }
    }
    /// <summary>
    /// The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
    ///  GPS_STATUS
    /// </summary>
    public class GpsStatusPacket: PacketV2<GpsStatusPayload>
    {
        public override byte CrcEtra => 23;
        public override int MessageId => 25;
        public override GpsStatusPayload Payload { get; } = new GpsStatusPayload();

        public override string Name => "GPS_STATUS";
        public override string ToString()
        {
            var name = "GPS_STATUS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  GPS_STATUS
    /// </summary>
    public class GpsStatusPayload : IPayload
    {
        public byte ByteSize => 101;

        public int Deserialize(byte[] buffer, int index)
        {
            SatellitesVisible = (byte)buffer[index++];
            for(var i=0;i<20;i++)
            {
                SatellitePrn[i] = (byte)buffer[index++];
            }
            for(var i=0;i<20;i++)
            {
                SatelliteUsed[i] = (byte)buffer[index++];
            }
            for(var i=0;i<20;i++)
            {
                SatelliteElevation[i] = (byte)buffer[index++];
            }
            for(var i=0;i<20;i++)
            {
                SatelliteAzimuth[i] = (byte)buffer[index++];
            }
            for(var i=0;i<20;i++)
            {
                SatelliteSnr[i] = (byte)buffer[index++];
            }
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(SatellitesVisible).CopyTo(buffer, index);index+=1;
            for(var i=0;i<20;i++)
            {
                BitConverter.GetBytes(SatellitePrn[i]).CopyTo(buffer, index);index+=1;
            }
            for(var i=0;i<20;i++)
            {
                BitConverter.GetBytes(SatelliteUsed[i]).CopyTo(buffer, index);index+=1;
            }
            for(var i=0;i<20;i++)
            {
                BitConverter.GetBytes(SatelliteElevation[i]).CopyTo(buffer, index);index+=1;
            }
            for(var i=0;i<20;i++)
            {
                BitConverter.GetBytes(SatelliteAzimuth[i]).CopyTo(buffer, index);index+=1;
            }
            for(var i=0;i<20;i++)
            {
                BitConverter.GetBytes(SatelliteSnr[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// Number of satellites visible
        /// 
        /// satellites_visible
        /// </summary>
        public byte SatellitesVisible { get; set; }
        /// <summary>
        /// Global satellite ID
        /// 
        /// satellite_prn
        /// </summary>
        public byte[] SatellitePrn { get; } = new byte[20];
        /// <summary>
        /// 0: Satellite not used, 1: used for localization
        /// 
        /// satellite_used
        /// </summary>
        public byte[] SatelliteUsed { get; } = new byte[20];
        /// <summary>
        /// Elevation (0: right on top of receiver, 90: on the horizon) of satellite
        /// deg
        /// satellite_elevation
        /// </summary>
        public byte[] SatelliteElevation { get; } = new byte[20];
        /// <summary>
        /// Direction of satellite, 0: 0 deg, 255: 360 deg.
        /// deg
        /// satellite_azimuth
        /// </summary>
        public byte[] SatelliteAzimuth { get; } = new byte[20];
        /// <summary>
        /// Signal to noise ratio of satellite
        /// dB
        /// satellite_snr
        /// </summary>
        public byte[] SatelliteSnr { get; } = new byte[20];
    }
    /// <summary>
    /// The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
    ///  SCALED_IMU
    /// </summary>
    public class ScaledImuPacket: PacketV2<ScaledImuPayload>
    {
        public override byte CrcEtra => 170;
        public override int MessageId => 26;
        public override ScaledImuPayload Payload { get; } = new ScaledImuPayload();

        public override string Name => "SCALED_IMU";
        public override string ToString()
        {
            var name = "SCALED_IMU".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SCALED_IMU
    /// </summary>
    public class ScaledImuPayload : IPayload
    {
        public byte ByteSize => 22;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X acceleration
        /// mG
        /// xacc
        /// </summary>
        public short Xacc { get; set; }
        /// <summary>
        /// Y acceleration
        /// mG
        /// yacc
        /// </summary>
        public short Yacc { get; set; }
        /// <summary>
        /// Z acceleration
        /// mG
        /// zacc
        /// </summary>
        public short Zacc { get; set; }
        /// <summary>
        /// Angular speed around X axis
        /// mrad/s
        /// xgyro
        /// </summary>
        public short Xgyro { get; set; }
        /// <summary>
        /// Angular speed around Y axis
        /// mrad/s
        /// ygyro
        /// </summary>
        public short Ygyro { get; set; }
        /// <summary>
        /// Angular speed around Z axis
        /// mrad/s
        /// zgyro
        /// </summary>
        public short Zgyro { get; set; }
        /// <summary>
        /// X Magnetic field
        /// mT
        /// xmag
        /// </summary>
        public short Xmag { get; set; }
        /// <summary>
        /// Y Magnetic field
        /// mT
        /// ymag
        /// </summary>
        public short Ymag { get; set; }
        /// <summary>
        /// Z Magnetic field
        /// mT
        /// zmag
        /// </summary>
        public short Zmag { get; set; }
    }
    /// <summary>
    /// The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw values without any scaling to allow data capture and system debugging.
    ///  RAW_IMU
    /// </summary>
    public class RawImuPacket: PacketV2<RawImuPayload>
    {
        public override byte CrcEtra => 144;
        public override int MessageId => 27;
        public override RawImuPayload Payload { get; } = new RawImuPayload();

        public override string Name => "RAW_IMU";
        public override string ToString()
        {
            var name = "RAW_IMU".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  RAW_IMU
    /// </summary>
    public class RawImuPayload : IPayload
    {
        public byte ByteSize => 26;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X acceleration (raw)
        /// 
        /// xacc
        /// </summary>
        public short Xacc { get; set; }
        /// <summary>
        /// Y acceleration (raw)
        /// 
        /// yacc
        /// </summary>
        public short Yacc { get; set; }
        /// <summary>
        /// Z acceleration (raw)
        /// 
        /// zacc
        /// </summary>
        public short Zacc { get; set; }
        /// <summary>
        /// Angular speed around X axis (raw)
        /// 
        /// xgyro
        /// </summary>
        public short Xgyro { get; set; }
        /// <summary>
        /// Angular speed around Y axis (raw)
        /// 
        /// ygyro
        /// </summary>
        public short Ygyro { get; set; }
        /// <summary>
        /// Angular speed around Z axis (raw)
        /// 
        /// zgyro
        /// </summary>
        public short Zgyro { get; set; }
        /// <summary>
        /// X Magnetic field (raw)
        /// 
        /// xmag
        /// </summary>
        public short Xmag { get; set; }
        /// <summary>
        /// Y Magnetic field (raw)
        /// 
        /// ymag
        /// </summary>
        public short Ymag { get; set; }
        /// <summary>
        /// Z Magnetic field (raw)
        /// 
        /// zmag
        /// </summary>
        public short Zmag { get; set; }
    }
    /// <summary>
    /// The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
    ///  RAW_PRESSURE
    /// </summary>
    public class RawPressurePacket: PacketV2<RawPressurePayload>
    {
        public override byte CrcEtra => 67;
        public override int MessageId => 28;
        public override RawPressurePayload Payload { get; } = new RawPressurePayload();

        public override string Name => "RAW_PRESSURE";
        public override string ToString()
        {
            var name = "RAW_PRESSURE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  RAW_PRESSURE
    /// </summary>
    public class RawPressurePayload : IPayload
    {
        public byte ByteSize => 16;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            PressAbs = BitConverter.ToInt16(buffer,index);index+=2;
            PressDiff1 = BitConverter.ToInt16(buffer,index);index+=2;
            PressDiff2 = BitConverter.ToInt16(buffer,index);index+=2;
            Temperature = BitConverter.ToInt16(buffer,index);index+=2;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(PressAbs).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(PressDiff1).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(PressDiff2).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=2;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Absolute pressure (raw)
        /// 
        /// press_abs
        /// </summary>
        public short PressAbs { get; set; }
        /// <summary>
        /// Differential pressure 1 (raw, 0 if nonexistent)
        /// 
        /// press_diff1
        /// </summary>
        public short PressDiff1 { get; set; }
        /// <summary>
        /// Differential pressure 2 (raw, 0 if nonexistent)
        /// 
        /// press_diff2
        /// </summary>
        public short PressDiff2 { get; set; }
        /// <summary>
        /// Raw Temperature measurement (raw)
        /// 
        /// temperature
        /// </summary>
        public short Temperature { get; set; }
    }
    /// <summary>
    /// The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
    ///  SCALED_PRESSURE
    /// </summary>
    public class ScaledPressurePacket: PacketV2<ScaledPressurePayload>
    {
        public override byte CrcEtra => 115;
        public override int MessageId => 29;
        public override ScaledPressurePayload Payload { get; } = new ScaledPressurePayload();

        public override string Name => "SCALED_PRESSURE";
        public override string ToString()
        {
            var name = "SCALED_PRESSURE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SCALED_PRESSURE
    /// </summary>
    public class ScaledPressurePayload : IPayload
    {
        public byte ByteSize => 14;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            PressAbs = BitConverter.ToSingle(buffer, index);index+=4;
            PressDiff = BitConverter.ToSingle(buffer, index);index+=4;
            Temperature = BitConverter.ToInt16(buffer,index);index+=2;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PressAbs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PressDiff).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=2;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Absolute pressure
        /// hPa
        /// press_abs
        /// </summary>
        public float PressAbs { get; set; }
        /// <summary>
        /// Differential pressure 1
        /// hPa
        /// press_diff
        /// </summary>
        public float PressDiff { get; set; }
        /// <summary>
        /// Temperature
        /// cdegC
        /// temperature
        /// </summary>
        public short Temperature { get; set; }
    }
    /// <summary>
    /// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
    ///  ATTITUDE
    /// </summary>
    public class AttitudePacket: PacketV2<AttitudePayload>
    {
        public override byte CrcEtra => 39;
        public override int MessageId => 30;
        public override AttitudePayload Payload { get; } = new AttitudePayload();

        public override string Name => "ATTITUDE";
        public override string ToString()
        {
            var name = "ATTITUDE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  ATTITUDE
    /// </summary>
    public class AttitudePayload : IPayload
    {
        public byte ByteSize => 28;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            Rollspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Pitchspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Yawspeed = BitConverter.ToSingle(buffer, index);index+=4;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Roll angle (-pi..+pi)
        /// rad
        /// roll
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Pitch angle (-pi..+pi)
        /// rad
        /// pitch
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Yaw angle (-pi..+pi)
        /// rad
        /// yaw
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// Roll angular speed
        /// rad/s
        /// rollspeed
        /// </summary>
        public float Rollspeed { get; set; }
        /// <summary>
        /// Pitch angular speed
        /// rad/s
        /// pitchspeed
        /// </summary>
        public float Pitchspeed { get; set; }
        /// <summary>
        /// Yaw angular speed
        /// rad/s
        /// yawspeed
        /// </summary>
        public float Yawspeed { get; set; }
    }
    /// <summary>
    /// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
    ///  ATTITUDE_QUATERNION
    /// </summary>
    public class AttitudeQuaternionPacket: PacketV2<AttitudeQuaternionPayload>
    {
        public override byte CrcEtra => 246;
        public override int MessageId => 31;
        public override AttitudeQuaternionPayload Payload { get; } = new AttitudeQuaternionPayload();

        public override string Name => "ATTITUDE_QUATERNION";
        public override string ToString()
        {
            var name = "ATTITUDE_QUATERNION".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  ATTITUDE_QUATERNION
    /// </summary>
    public class AttitudeQuaternionPayload : IPayload
    {
        public byte ByteSize => 32;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Q1 = BitConverter.ToSingle(buffer, index);index+=4;
            Q2 = BitConverter.ToSingle(buffer, index);index+=4;
            Q3 = BitConverter.ToSingle(buffer, index);index+=4;
            Q4 = BitConverter.ToSingle(buffer, index);index+=4;
            Rollspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Pitchspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Yawspeed = BitConverter.ToSingle(buffer, index);index+=4;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Quaternion component 1, w (1 in null-rotation)
        /// 
        /// q1
        /// </summary>
        public float Q1 { get; set; }
        /// <summary>
        /// Quaternion component 2, x (0 in null-rotation)
        /// 
        /// q2
        /// </summary>
        public float Q2 { get; set; }
        /// <summary>
        /// Quaternion component 3, y (0 in null-rotation)
        /// 
        /// q3
        /// </summary>
        public float Q3 { get; set; }
        /// <summary>
        /// Quaternion component 4, z (0 in null-rotation)
        /// 
        /// q4
        /// </summary>
        public float Q4 { get; set; }
        /// <summary>
        /// Roll angular speed
        /// rad/s
        /// rollspeed
        /// </summary>
        public float Rollspeed { get; set; }
        /// <summary>
        /// Pitch angular speed
        /// rad/s
        /// pitchspeed
        /// </summary>
        public float Pitchspeed { get; set; }
        /// <summary>
        /// Yaw angular speed
        /// rad/s
        /// yawspeed
        /// </summary>
        public float Yawspeed { get; set; }
    }
    /// <summary>
    /// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
    ///  LOCAL_POSITION_NED
    /// </summary>
    public class LocalPositionNedPacket: PacketV2<LocalPositionNedPayload>
    {
        public override byte CrcEtra => 185;
        public override int MessageId => 32;
        public override LocalPositionNedPayload Payload { get; } = new LocalPositionNedPayload();

        public override string Name => "LOCAL_POSITION_NED";
        public override string ToString()
        {
            var name = "LOCAL_POSITION_NED".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  LOCAL_POSITION_NED
    /// </summary>
    public class LocalPositionNedPayload : IPayload
    {
        public byte ByteSize => 28;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Vx = BitConverter.ToSingle(buffer, index);index+=4;
            Vy = BitConverter.ToSingle(buffer, index);index+=4;
            Vz = BitConverter.ToSingle(buffer, index);index+=4;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X Position
        /// m
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y Position
        /// m
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z Position
        /// m
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// X Speed
        /// m/s
        /// vx
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Y Speed
        /// m/s
        /// vy
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Z Speed
        /// m/s
        /// vz
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
        public override byte CrcEtra => 104;
        public override int MessageId => 33;
        public override GlobalPositionIntPayload Payload { get; } = new GlobalPositionIntPayload();

        public override string Name => "GLOBAL_POSITION_INT";
        public override string ToString()
        {
            var name = "GLOBAL_POSITION_INT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  GLOBAL_POSITION_INT
    /// </summary>
    public class GlobalPositionIntPayload : IPayload
    {
        public byte ByteSize => 28;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToInt32(buffer,index);index+=4;
            RelativeAlt = BitConverter.ToInt32(buffer,index);index+=4;
            Vx = BitConverter.ToInt16(buffer,index);index+=2;
            Vy = BitConverter.ToInt16(buffer,index);index+=2;
            Vz = BitConverter.ToInt16(buffer,index);index+=2;
            Hdg = BitConverter.ToUInt16(buffer,index);index+=2;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Latitude, expressed
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude, expressed
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude (AMSL). Note that virtually all GPS modules provide both WGS84 and AMSL.
        /// mm
        /// alt
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// Altitude above ground
        /// mm
        /// relative_alt
        /// </summary>
        public int RelativeAlt { get; set; }
        /// <summary>
        /// Ground X Speed (Latitude, positive north)
        /// cm/s
        /// vx
        /// </summary>
        public short Vx { get; set; }
        /// <summary>
        /// Ground Y Speed (Longitude, positive east)
        /// cm/s
        /// vy
        /// </summary>
        public short Vy { get; set; }
        /// <summary>
        /// Ground Z Speed (Altitude, positive down)
        /// cm/s
        /// vz
        /// </summary>
        public short Vz { get; set; }
        /// <summary>
        /// Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        /// cdeg
        /// hdg
        /// </summary>
        public ushort Hdg { get; set; }
    }
    /// <summary>
    /// The scaled values of the RC channels received: (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX.
    ///  RC_CHANNELS_SCALED
    /// </summary>
    public class RcChannelsScaledPacket: PacketV2<RcChannelsScaledPayload>
    {
        public override byte CrcEtra => 237;
        public override int MessageId => 34;
        public override RcChannelsScaledPayload Payload { get; } = new RcChannelsScaledPayload();

        public override string Name => "RC_CHANNELS_SCALED";
        public override string ToString()
        {
            var name = "RC_CHANNELS_SCALED".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  RC_CHANNELS_SCALED
    /// </summary>
    public class RcChannelsScaledPayload : IPayload
    {
        public byte ByteSize => 22;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// RC channel 1 value scaled.
        /// 
        /// chan1_scaled
        /// </summary>
        public short Chan1Scaled { get; set; }
        /// <summary>
        /// RC channel 2 value scaled.
        /// 
        /// chan2_scaled
        /// </summary>
        public short Chan2Scaled { get; set; }
        /// <summary>
        /// RC channel 3 value scaled.
        /// 
        /// chan3_scaled
        /// </summary>
        public short Chan3Scaled { get; set; }
        /// <summary>
        /// RC channel 4 value scaled.
        /// 
        /// chan4_scaled
        /// </summary>
        public short Chan4Scaled { get; set; }
        /// <summary>
        /// RC channel 5 value scaled.
        /// 
        /// chan5_scaled
        /// </summary>
        public short Chan5Scaled { get; set; }
        /// <summary>
        /// RC channel 6 value scaled.
        /// 
        /// chan6_scaled
        /// </summary>
        public short Chan6Scaled { get; set; }
        /// <summary>
        /// RC channel 7 value scaled.
        /// 
        /// chan7_scaled
        /// </summary>
        public short Chan7Scaled { get; set; }
        /// <summary>
        /// RC channel 8 value scaled.
        /// 
        /// chan8_scaled
        /// </summary>
        public short Chan8Scaled { get; set; }
        /// <summary>
        /// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
        /// 
        /// port
        /// </summary>
        public byte Port { get; set; }
        /// <summary>
        /// Receive signal strength indicator. Values: [0-100], 255: invalid/unknown.
        /// %
        /// rssi
        /// </summary>
        public byte Rssi { get; set; }
    }
    /// <summary>
    /// The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters might violate this specification.
    ///  RC_CHANNELS_RAW
    /// </summary>
    public class RcChannelsRawPacket: PacketV2<RcChannelsRawPayload>
    {
        public override byte CrcEtra => 244;
        public override int MessageId => 35;
        public override RcChannelsRawPayload Payload { get; } = new RcChannelsRawPayload();

        public override string Name => "RC_CHANNELS_RAW";
        public override string ToString()
        {
            var name = "RC_CHANNELS_RAW".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  RC_CHANNELS_RAW
    /// </summary>
    public class RcChannelsRawPayload : IPayload
    {
        public byte ByteSize => 22;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// RC channel 1 value.
        /// us
        /// chan1_raw
        /// </summary>
        public ushort Chan1Raw { get; set; }
        /// <summary>
        /// RC channel 2 value.
        /// us
        /// chan2_raw
        /// </summary>
        public ushort Chan2Raw { get; set; }
        /// <summary>
        /// RC channel 3 value.
        /// us
        /// chan3_raw
        /// </summary>
        public ushort Chan3Raw { get; set; }
        /// <summary>
        /// RC channel 4 value.
        /// us
        /// chan4_raw
        /// </summary>
        public ushort Chan4Raw { get; set; }
        /// <summary>
        /// RC channel 5 value.
        /// us
        /// chan5_raw
        /// </summary>
        public ushort Chan5Raw { get; set; }
        /// <summary>
        /// RC channel 6 value.
        /// us
        /// chan6_raw
        /// </summary>
        public ushort Chan6Raw { get; set; }
        /// <summary>
        /// RC channel 7 value.
        /// us
        /// chan7_raw
        /// </summary>
        public ushort Chan7Raw { get; set; }
        /// <summary>
        /// RC channel 8 value.
        /// us
        /// chan8_raw
        /// </summary>
        public ushort Chan8Raw { get; set; }
        /// <summary>
        /// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
        /// 
        /// port
        /// </summary>
        public byte Port { get; set; }
        /// <summary>
        /// Receive signal strength indicator. Values: [0-100], 255: invalid/unknown.
        /// %
        /// rssi
        /// </summary>
        public byte Rssi { get; set; }
    }
    /// <summary>
    /// The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
    ///  SERVO_OUTPUT_RAW
    /// </summary>
    public class ServoOutputRawPacket: PacketV2<ServoOutputRawPayload>
    {
        public override byte CrcEtra => 222;
        public override int MessageId => 36;
        public override ServoOutputRawPayload Payload { get; } = new ServoOutputRawPayload();

        public override string Name => "SERVO_OUTPUT_RAW";
        public override string ToString()
        {
            var name = "SERVO_OUTPUT_RAW".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SERVO_OUTPUT_RAW
    /// </summary>
    public class ServoOutputRawPayload : IPayload
    {
        public byte ByteSize => 21;

        public int Deserialize(byte[] buffer, int index)
        {
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
            Servo9Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Servo10Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Servo11Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Servo12Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Servo13Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Servo14Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Servo15Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            Servo16Raw = BitConverter.ToUInt16(buffer,index);index+=2;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public uint TimeUsec { get; set; }
        /// <summary>
        /// Servo output 1 value
        /// us
        /// servo1_raw
        /// </summary>
        public ushort Servo1Raw { get; set; }
        /// <summary>
        /// Servo output 2 value
        /// us
        /// servo2_raw
        /// </summary>
        public ushort Servo2Raw { get; set; }
        /// <summary>
        /// Servo output 3 value
        /// us
        /// servo3_raw
        /// </summary>
        public ushort Servo3Raw { get; set; }
        /// <summary>
        /// Servo output 4 value
        /// us
        /// servo4_raw
        /// </summary>
        public ushort Servo4Raw { get; set; }
        /// <summary>
        /// Servo output 5 value
        /// us
        /// servo5_raw
        /// </summary>
        public ushort Servo5Raw { get; set; }
        /// <summary>
        /// Servo output 6 value
        /// us
        /// servo6_raw
        /// </summary>
        public ushort Servo6Raw { get; set; }
        /// <summary>
        /// Servo output 7 value
        /// us
        /// servo7_raw
        /// </summary>
        public ushort Servo7Raw { get; set; }
        /// <summary>
        /// Servo output 8 value
        /// us
        /// servo8_raw
        /// </summary>
        public ushort Servo8Raw { get; set; }
        /// <summary>
        /// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
        /// 
        /// port
        /// </summary>
        public byte Port { get; set; }
        /// <summary>
        /// Servo output 9 value
        /// us
        /// servo9_raw
        /// </summary>
        public ushort Servo9Raw { get; set; }
        /// <summary>
        /// Servo output 10 value
        /// us
        /// servo10_raw
        /// </summary>
        public ushort Servo10Raw { get; set; }
        /// <summary>
        /// Servo output 11 value
        /// us
        /// servo11_raw
        /// </summary>
        public ushort Servo11Raw { get; set; }
        /// <summary>
        /// Servo output 12 value
        /// us
        /// servo12_raw
        /// </summary>
        public ushort Servo12Raw { get; set; }
        /// <summary>
        /// Servo output 13 value
        /// us
        /// servo13_raw
        /// </summary>
        public ushort Servo13Raw { get; set; }
        /// <summary>
        /// Servo output 14 value
        /// us
        /// servo14_raw
        /// </summary>
        public ushort Servo14Raw { get; set; }
        /// <summary>
        /// Servo output 15 value
        /// us
        /// servo15_raw
        /// </summary>
        public ushort Servo15Raw { get; set; }
        /// <summary>
        /// Servo output 16 value
        /// us
        /// servo16_raw
        /// </summary>
        public ushort Servo16Raw { get; set; }
    }
    /// <summary>
    /// Request a partial list of mission items from the system/component. https://mavlink.io/en/protocol/mission.html. If start and end index are the same, just send one waypoint.
    ///  MISSION_REQUEST_PARTIAL_LIST
    /// </summary>
    public class MissionRequestPartialListPacket: PacketV2<MissionRequestPartialListPayload>
    {
        public override byte CrcEtra => 212;
        public override int MessageId => 37;
        public override MissionRequestPartialListPayload Payload { get; } = new MissionRequestPartialListPayload();

        public override string Name => "MISSION_REQUEST_PARTIAL_LIST";
        public override string ToString()
        {
            var name = "MISSION_REQUEST_PARTIAL_LIST".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MISSION_REQUEST_PARTIAL_LIST
    /// </summary>
    public class MissionRequestPartialListPayload : IPayload
    {
        public byte ByteSize => 6;

        public int Deserialize(byte[] buffer, int index)
        {
            StartIndex = BitConverter.ToInt16(buffer,index);index+=2;
            EndIndex = BitConverter.ToInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            MissionType = (MavMissionType)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(StartIndex).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(EndIndex).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)MissionType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Start index, 0 by default
        /// 
        /// start_index
        /// </summary>
        public short StartIndex { get; set; }
        /// <summary>
        /// End index, -1 by default (-1: send list to end). Else a valid index of the list
        /// 
        /// end_index
        /// </summary>
        public short EndIndex { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission type.
        /// 
        /// mission_type
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
    ///  MISSION_WRITE_PARTIAL_LIST
    /// </summary>
    public class MissionWritePartialListPacket: PacketV2<MissionWritePartialListPayload>
    {
        public override byte CrcEtra => 9;
        public override int MessageId => 38;
        public override MissionWritePartialListPayload Payload { get; } = new MissionWritePartialListPayload();

        public override string Name => "MISSION_WRITE_PARTIAL_LIST";
        public override string ToString()
        {
            var name = "MISSION_WRITE_PARTIAL_LIST".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MISSION_WRITE_PARTIAL_LIST
    /// </summary>
    public class MissionWritePartialListPayload : IPayload
    {
        public byte ByteSize => 6;

        public int Deserialize(byte[] buffer, int index)
        {
            StartIndex = BitConverter.ToInt16(buffer,index);index+=2;
            EndIndex = BitConverter.ToInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            MissionType = (MavMissionType)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(StartIndex).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(EndIndex).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)MissionType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
        /// 
        /// start_index
        /// </summary>
        public short StartIndex { get; set; }
        /// <summary>
        /// End index, equal or greater than start index.
        /// 
        /// end_index
        /// </summary>
        public short EndIndex { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission type.
        /// 
        /// mission_type
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
        public override byte CrcEtra => 254;
        public override int MessageId => 39;
        public override MissionItemPayload Payload { get; } = new MissionItemPayload();

        public override string Name => "MISSION_ITEM";
        public override string ToString()
        {
            var name = "MISSION_ITEM".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MISSION_ITEM
    /// </summary>
    public class MissionItemPayload : IPayload
    {
        public byte ByteSize => 37;

        public int Deserialize(byte[] buffer, int index)
        {
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
            MissionType = (MavMissionType)buffer[index++];
            return ByteSize;
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
            BitConverter.GetBytes((byte)Frame).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Current).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Autocontinue).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)MissionType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// PARAM1, see MAV_CMD enum
        /// 
        /// param1
        /// </summary>
        public float Param1 { get; set; }
        /// <summary>
        /// PARAM2, see MAV_CMD enum
        /// 
        /// param2
        /// </summary>
        public float Param2 { get; set; }
        /// <summary>
        /// PARAM3, see MAV_CMD enum
        /// 
        /// param3
        /// </summary>
        public float Param3 { get; set; }
        /// <summary>
        /// PARAM4, see MAV_CMD enum
        /// 
        /// param4
        /// </summary>
        public float Param4 { get; set; }
        /// <summary>
        /// PARAM5 / local: X coordinate, global: latitude
        /// 
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// PARAM6 / local: Y coordinate, global: longitude
        /// 
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
        /// 
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Sequence
        /// 
        /// seq
        /// </summary>
        public ushort Seq { get; set; }
        /// <summary>
        /// The scheduled action for the waypoint.
        /// 
        /// command
        /// </summary>
        public MavCmd Command { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// The coordinate system of the waypoint.
        /// 
        /// frame
        /// </summary>
        public MavFrame Frame { get; set; }
        /// <summary>
        /// false:0, true:1
        /// 
        /// current
        /// </summary>
        public byte Current { get; set; }
        /// <summary>
        /// Autocontinue to next waypoint
        /// 
        /// autocontinue
        /// </summary>
        public byte Autocontinue { get; set; }
        /// <summary>
        /// Mission type.
        /// 
        /// mission_type
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. https://mavlink.io/en/protocol/mission.html
    ///  MISSION_REQUEST
    /// </summary>
    public class MissionRequestPacket: PacketV2<MissionRequestPayload>
    {
        public override byte CrcEtra => 230;
        public override int MessageId => 40;
        public override MissionRequestPayload Payload { get; } = new MissionRequestPayload();

        public override string Name => "MISSION_REQUEST";
        public override string ToString()
        {
            var name = "MISSION_REQUEST".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MISSION_REQUEST
    /// </summary>
    public class MissionRequestPayload : IPayload
    {
        public byte ByteSize => 4;

        public int Deserialize(byte[] buffer, int index)
        {
            Seq = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            MissionType = (MavMissionType)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)MissionType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Sequence
        /// 
        /// seq
        /// </summary>
        public ushort Seq { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission type.
        /// 
        /// mission_type
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
    ///  MISSION_SET_CURRENT
    /// </summary>
    public class MissionSetCurrentPacket: PacketV2<MissionSetCurrentPayload>
    {
        public override byte CrcEtra => 28;
        public override int MessageId => 41;
        public override MissionSetCurrentPayload Payload { get; } = new MissionSetCurrentPayload();

        public override string Name => "MISSION_SET_CURRENT";
        public override string ToString()
        {
            var name = "MISSION_SET_CURRENT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MISSION_SET_CURRENT
    /// </summary>
    public class MissionSetCurrentPayload : IPayload
    {
        public byte ByteSize => 4;

        public int Deserialize(byte[] buffer, int index)
        {
            Seq = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Sequence
        /// 
        /// seq
        /// </summary>
        public ushort Seq { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
    ///  MISSION_CURRENT
    /// </summary>
    public class MissionCurrentPacket: PacketV2<MissionCurrentPayload>
    {
        public override byte CrcEtra => 28;
        public override int MessageId => 42;
        public override MissionCurrentPayload Payload { get; } = new MissionCurrentPayload();

        public override string Name => "MISSION_CURRENT";
        public override string ToString()
        {
            var name = "MISSION_CURRENT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MISSION_CURRENT
    /// </summary>
    public class MissionCurrentPayload : IPayload
    {
        public byte ByteSize => 2;

        public int Deserialize(byte[] buffer, int index)
        {
            Seq = BitConverter.ToUInt16(buffer,index);index+=2;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=2;
            return ByteSize;
        }

        /// <summary>
        /// Sequence
        /// 
        /// seq
        /// </summary>
        public ushort Seq { get; set; }
    }
    /// <summary>
    /// Request the overall list of mission items from the system/component.
    ///  MISSION_REQUEST_LIST
    /// </summary>
    public class MissionRequestListPacket: PacketV2<MissionRequestListPayload>
    {
        public override byte CrcEtra => 132;
        public override int MessageId => 43;
        public override MissionRequestListPayload Payload { get; } = new MissionRequestListPayload();

        public override string Name => "MISSION_REQUEST_LIST";
        public override string ToString()
        {
            var name = "MISSION_REQUEST_LIST".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MISSION_REQUEST_LIST
    /// </summary>
    public class MissionRequestListPayload : IPayload
    {
        public byte ByteSize => 2;

        public int Deserialize(byte[] buffer, int index)
        {
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            MissionType = (MavMissionType)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)MissionType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission type.
        /// 
        /// mission_type
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of waypoints.
    ///  MISSION_COUNT
    /// </summary>
    public class MissionCountPacket: PacketV2<MissionCountPayload>
    {
        public override byte CrcEtra => 221;
        public override int MessageId => 44;
        public override MissionCountPayload Payload { get; } = new MissionCountPayload();

        public override string Name => "MISSION_COUNT";
        public override string ToString()
        {
            var name = "MISSION_COUNT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MISSION_COUNT
    /// </summary>
    public class MissionCountPayload : IPayload
    {
        public byte ByteSize => 4;

        public int Deserialize(byte[] buffer, int index)
        {
            Count = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            MissionType = (MavMissionType)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Count).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)MissionType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Number of mission items in the sequence
        /// 
        /// count
        /// </summary>
        public ushort Count { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission type.
        /// 
        /// mission_type
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// Delete all mission items at once.
    ///  MISSION_CLEAR_ALL
    /// </summary>
    public class MissionClearAllPacket: PacketV2<MissionClearAllPayload>
    {
        public override byte CrcEtra => 232;
        public override int MessageId => 45;
        public override MissionClearAllPayload Payload { get; } = new MissionClearAllPayload();

        public override string Name => "MISSION_CLEAR_ALL";
        public override string ToString()
        {
            var name = "MISSION_CLEAR_ALL".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MISSION_CLEAR_ALL
    /// </summary>
    public class MissionClearAllPayload : IPayload
    {
        public byte ByteSize => 2;

        public int Deserialize(byte[] buffer, int index)
        {
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            MissionType = (MavMissionType)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)MissionType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission type.
        /// 
        /// mission_type
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next waypoint.
    ///  MISSION_ITEM_REACHED
    /// </summary>
    public class MissionItemReachedPacket: PacketV2<MissionItemReachedPayload>
    {
        public override byte CrcEtra => 11;
        public override int MessageId => 46;
        public override MissionItemReachedPayload Payload { get; } = new MissionItemReachedPayload();

        public override string Name => "MISSION_ITEM_REACHED";
        public override string ToString()
        {
            var name = "MISSION_ITEM_REACHED".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MISSION_ITEM_REACHED
    /// </summary>
    public class MissionItemReachedPayload : IPayload
    {
        public byte ByteSize => 2;

        public int Deserialize(byte[] buffer, int index)
        {
            Seq = BitConverter.ToUInt16(buffer,index);index+=2;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=2;
            return ByteSize;
        }

        /// <summary>
        /// Sequence
        /// 
        /// seq
        /// </summary>
        public ushort Seq { get; set; }
    }
    /// <summary>
    /// Acknowledgment message during waypoint handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
    ///  MISSION_ACK
    /// </summary>
    public class MissionAckPacket: PacketV2<MissionAckPayload>
    {
        public override byte CrcEtra => 153;
        public override int MessageId => 47;
        public override MissionAckPayload Payload { get; } = new MissionAckPayload();

        public override string Name => "MISSION_ACK";
        public override string ToString()
        {
            var name = "MISSION_ACK".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MISSION_ACK
    /// </summary>
    public class MissionAckPayload : IPayload
    {
        public byte ByteSize => 3;

        public int Deserialize(byte[] buffer, int index)
        {
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            Type = (MavMissionResult)buffer[index++];
            MissionType = (MavMissionType)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)Type).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)MissionType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission result.
        /// 
        /// type
        /// </summary>
        public MavMissionResult Type { get; set; }
        /// <summary>
        /// Mission type.
        /// 
        /// mission_type
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// As local waypoints exist, the global waypoint reference allows to transform between the local coordinate frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings are connected and the MAV should move from in- to outdoor.
    ///  SET_GPS_GLOBAL_ORIGIN
    /// </summary>
    public class SetGpsGlobalOriginPacket: PacketV2<SetGpsGlobalOriginPayload>
    {
        public override byte CrcEtra => 41;
        public override int MessageId => 48;
        public override SetGpsGlobalOriginPayload Payload { get; } = new SetGpsGlobalOriginPayload();

        public override string Name => "SET_GPS_GLOBAL_ORIGIN";
        public override string ToString()
        {
            var name = "SET_GPS_GLOBAL_ORIGIN".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SET_GPS_GLOBAL_ORIGIN
    /// </summary>
    public class SetGpsGlobalOriginPayload : IPayload
    {
        public byte ByteSize => 13;

        public int Deserialize(byte[] buffer, int index)
        {
            Latitude = BitConverter.ToInt32(buffer,index);index+=4;
            Longitude = BitConverter.ToInt32(buffer,index);index+=4;
            Altitude = BitConverter.ToInt32(buffer,index);index+=4;
            TargetSystem = (byte)buffer[index++];
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Latitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Longitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Altitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            return ByteSize;
        }

        /// <summary>
        /// Latitude (WGS84)
        /// degE7
        /// latitude
        /// </summary>
        public int Latitude { get; set; }
        /// <summary>
        /// Longitude (WGS84)
        /// degE7
        /// longitude
        /// </summary>
        public int Longitude { get; set; }
        /// <summary>
        /// Altitude (AMSL). Positive for up.
        /// mm
        /// altitude
        /// </summary>
        public int Altitude { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
    }
    /// <summary>
    /// Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position
    ///  GPS_GLOBAL_ORIGIN
    /// </summary>
    public class GpsGlobalOriginPacket: PacketV2<GpsGlobalOriginPayload>
    {
        public override byte CrcEtra => 39;
        public override int MessageId => 49;
        public override GpsGlobalOriginPayload Payload { get; } = new GpsGlobalOriginPayload();

        public override string Name => "GPS_GLOBAL_ORIGIN";
        public override string ToString()
        {
            var name = "GPS_GLOBAL_ORIGIN".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  GPS_GLOBAL_ORIGIN
    /// </summary>
    public class GpsGlobalOriginPayload : IPayload
    {
        public byte ByteSize => 12;

        public int Deserialize(byte[] buffer, int index)
        {
            Latitude = BitConverter.ToInt32(buffer,index);index+=4;
            Longitude = BitConverter.ToInt32(buffer,index);index+=4;
            Altitude = BitConverter.ToInt32(buffer,index);index+=4;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Latitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Longitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Altitude).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            return ByteSize;
        }

        /// <summary>
        /// Latitude (WGS84)
        /// degE7
        /// latitude
        /// </summary>
        public int Latitude { get; set; }
        /// <summary>
        /// Longitude (WGS84)
        /// degE7
        /// longitude
        /// </summary>
        public int Longitude { get; set; }
        /// <summary>
        /// Altitude (AMSL). Positive for up.
        /// mm
        /// altitude
        /// </summary>
        public int Altitude { get; set; }
        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
    }
    /// <summary>
    /// Bind a RC channel to a parameter. The parameter should change according to the RC channel value.
    ///  PARAM_MAP_RC
    /// </summary>
    public class ParamMapRcPacket: PacketV2<ParamMapRcPayload>
    {
        public override byte CrcEtra => 78;
        public override int MessageId => 50;
        public override ParamMapRcPayload Payload { get; } = new ParamMapRcPayload();

        public override string Name => "PARAM_MAP_RC";
        public override string ToString()
        {
            var name = "PARAM_MAP_RC".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  PARAM_MAP_RC
    /// </summary>
    public class ParamMapRcPayload : IPayload
    {
        public byte ByteSize => 37;

        public int Deserialize(byte[] buffer, int index)
        {
            ParamValue0 = BitConverter.ToSingle(buffer, index);index+=4;
            Scale = BitConverter.ToSingle(buffer, index);index+=4;
            ParamValueMin = BitConverter.ToSingle(buffer, index);index+=4;
            ParamValueMax = BitConverter.ToSingle(buffer, index);index+=4;
            ParamIndex = BitConverter.ToInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
                Encoding.ASCII.GetChars(buffer, index,16,ParamId,0);
                index+=16;
            ParameterRcChannelIndex = (byte)buffer[index++];
            return ByteSize;
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
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);
            index+=16;
            BitConverter.GetBytes(ParameterRcChannelIndex).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Initial parameter value
        /// 
        /// param_value0
        /// </summary>
        public float ParamValue0 { get; set; }
        /// <summary>
        /// Scale, maps the RC range [-1, 1] to a parameter value
        /// 
        /// scale
        /// </summary>
        public float Scale { get; set; }
        /// <summary>
        /// Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends on implementation)
        /// 
        /// param_value_min
        /// </summary>
        public float ParamValueMin { get; set; }
        /// <summary>
        /// Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends on implementation)
        /// 
        /// param_value_max
        /// </summary>
        public float ParamValueMax { get; set; }
        /// <summary>
        /// Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored), send -2 to disable any existing map for this rc_channel_index.
        /// 
        /// param_index
        /// </summary>
        public short ParamIndex { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// 
        /// param_id
        /// </summary>
        public char[] ParamId { get; } = new char[16];
        /// <summary>
        /// Index of parameter RC channel. Not equal to the RC channel id. Typically corresponds to a potentiometer-knob on the RC.
        /// 
        /// parameter_rc_channel_index
        /// </summary>
        public byte ParameterRcChannelIndex { get; set; }
    }
    /// <summary>
    /// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM_INT message. https://mavlink.io/en/protocol/mission.html
    ///  MISSION_REQUEST_INT
    /// </summary>
    public class MissionRequestIntPacket: PacketV2<MissionRequestIntPayload>
    {
        public override byte CrcEtra => 196;
        public override int MessageId => 51;
        public override MissionRequestIntPayload Payload { get; } = new MissionRequestIntPayload();

        public override string Name => "MISSION_REQUEST_INT";
        public override string ToString()
        {
            var name = "MISSION_REQUEST_INT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MISSION_REQUEST_INT
    /// </summary>
    public class MissionRequestIntPayload : IPayload
    {
        public byte ByteSize => 4;

        public int Deserialize(byte[] buffer, int index)
        {
            Seq = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            MissionType = (MavMissionType)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)MissionType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Sequence
        /// 
        /// seq
        /// </summary>
        public ushort Seq { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mission type.
        /// 
        /// mission_type
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national or competition regulations.
    ///  SAFETY_SET_ALLOWED_AREA
    /// </summary>
    public class SafetySetAllowedAreaPacket: PacketV2<SafetySetAllowedAreaPayload>
    {
        public override byte CrcEtra => 15;
        public override int MessageId => 54;
        public override SafetySetAllowedAreaPayload Payload { get; } = new SafetySetAllowedAreaPayload();

        public override string Name => "SAFETY_SET_ALLOWED_AREA";
        public override string ToString()
        {
            var name = "SAFETY_SET_ALLOWED_AREA".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SAFETY_SET_ALLOWED_AREA
    /// </summary>
    public class SafetySetAllowedAreaPayload : IPayload
    {
        public byte ByteSize => 27;

        public int Deserialize(byte[] buffer, int index)
        {
            P1x = BitConverter.ToSingle(buffer, index);index+=4;
            P1y = BitConverter.ToSingle(buffer, index);index+=4;
            P1z = BitConverter.ToSingle(buffer, index);index+=4;
            P2x = BitConverter.ToSingle(buffer, index);index+=4;
            P2y = BitConverter.ToSingle(buffer, index);index+=4;
            P2z = BitConverter.ToSingle(buffer, index);index+=4;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            Frame = (MavFrame)buffer[index++];
            return ByteSize;
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
            BitConverter.GetBytes((byte)Frame).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// x position 1 / Latitude 1
        /// m
        /// p1x
        /// </summary>
        public float P1x { get; set; }
        /// <summary>
        /// y position 1 / Longitude 1
        /// m
        /// p1y
        /// </summary>
        public float P1y { get; set; }
        /// <summary>
        /// z position 1 / Altitude 1
        /// m
        /// p1z
        /// </summary>
        public float P1z { get; set; }
        /// <summary>
        /// x position 2 / Latitude 2
        /// m
        /// p2x
        /// </summary>
        public float P2x { get; set; }
        /// <summary>
        /// y position 2 / Longitude 2
        /// m
        /// p2y
        /// </summary>
        public float P2y { get; set; }
        /// <summary>
        /// z position 2 / Altitude 2
        /// m
        /// p2z
        /// </summary>
        public float P2z { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
        /// 
        /// frame
        /// </summary>
        public MavFrame Frame { get; set; }
    }
    /// <summary>
    /// Read out the safety zone the MAV currently assumes.
    ///  SAFETY_ALLOWED_AREA
    /// </summary>
    public class SafetyAllowedAreaPacket: PacketV2<SafetyAllowedAreaPayload>
    {
        public override byte CrcEtra => 3;
        public override int MessageId => 55;
        public override SafetyAllowedAreaPayload Payload { get; } = new SafetyAllowedAreaPayload();

        public override string Name => "SAFETY_ALLOWED_AREA";
        public override string ToString()
        {
            var name = "SAFETY_ALLOWED_AREA".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SAFETY_ALLOWED_AREA
    /// </summary>
    public class SafetyAllowedAreaPayload : IPayload
    {
        public byte ByteSize => 25;

        public int Deserialize(byte[] buffer, int index)
        {
            P1x = BitConverter.ToSingle(buffer, index);index+=4;
            P1y = BitConverter.ToSingle(buffer, index);index+=4;
            P1z = BitConverter.ToSingle(buffer, index);index+=4;
            P2x = BitConverter.ToSingle(buffer, index);index+=4;
            P2y = BitConverter.ToSingle(buffer, index);index+=4;
            P2z = BitConverter.ToSingle(buffer, index);index+=4;
            Frame = (MavFrame)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(P1x).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P1y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P1z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P2x).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P2y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(P2z).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes((byte)Frame).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// x position 1 / Latitude 1
        /// m
        /// p1x
        /// </summary>
        public float P1x { get; set; }
        /// <summary>
        /// y position 1 / Longitude 1
        /// m
        /// p1y
        /// </summary>
        public float P1y { get; set; }
        /// <summary>
        /// z position 1 / Altitude 1
        /// m
        /// p1z
        /// </summary>
        public float P1z { get; set; }
        /// <summary>
        /// x position 2 / Latitude 2
        /// m
        /// p2x
        /// </summary>
        public float P2x { get; set; }
        /// <summary>
        /// y position 2 / Longitude 2
        /// m
        /// p2y
        /// </summary>
        public float P2y { get; set; }
        /// <summary>
        /// z position 2 / Altitude 2
        /// m
        /// p2z
        /// </summary>
        public float P2z { get; set; }
        /// <summary>
        /// Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
        /// 
        /// frame
        /// </summary>
        public MavFrame Frame { get; set; }
    }
    /// <summary>
    /// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
    ///  ATTITUDE_QUATERNION_COV
    /// </summary>
    public class AttitudeQuaternionCovPacket: PacketV2<AttitudeQuaternionCovPayload>
    {
        public override byte CrcEtra => 167;
        public override int MessageId => 61;
        public override AttitudeQuaternionCovPayload Payload { get; } = new AttitudeQuaternionCovPayload();

        public override string Name => "ATTITUDE_QUATERNION_COV";
        public override string ToString()
        {
            var name = "ATTITUDE_QUATERNION_COV".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  ATTITUDE_QUATERNION_COV
    /// </summary>
    public class AttitudeQuaternionCovPayload : IPayload
    {
        public byte ByteSize => 72;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            for(var i=0;i<4;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            Rollspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Pitchspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Yawspeed = BitConverter.ToSingle(buffer, index);index+=4;
            for(var i=0;i<9;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
        /// 
        /// q
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Roll angular speed
        /// rad/s
        /// rollspeed
        /// </summary>
        public float Rollspeed { get; set; }
        /// <summary>
        /// Pitch angular speed
        /// rad/s
        /// pitchspeed
        /// </summary>
        public float Pitchspeed { get; set; }
        /// <summary>
        /// Yaw angular speed
        /// rad/s
        /// yawspeed
        /// </summary>
        public float Yawspeed { get; set; }
        /// <summary>
        /// Attitude covariance
        /// 
        /// covariance
        /// </summary>
        public float[] Covariance { get; } = new float[9];
    }
    /// <summary>
    /// The state of the fixed wing navigation and position controller.
    ///  NAV_CONTROLLER_OUTPUT
    /// </summary>
    public class NavControllerOutputPacket: PacketV2<NavControllerOutputPayload>
    {
        public override byte CrcEtra => 183;
        public override int MessageId => 62;
        public override NavControllerOutputPayload Payload { get; } = new NavControllerOutputPayload();

        public override string Name => "NAV_CONTROLLER_OUTPUT";
        public override string ToString()
        {
            var name = "NAV_CONTROLLER_OUTPUT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  NAV_CONTROLLER_OUTPUT
    /// </summary>
    public class NavControllerOutputPayload : IPayload
    {
        public byte ByteSize => 26;

        public int Deserialize(byte[] buffer, int index)
        {
            NavRoll = BitConverter.ToSingle(buffer, index);index+=4;
            NavPitch = BitConverter.ToSingle(buffer, index);index+=4;
            AltError = BitConverter.ToSingle(buffer, index);index+=4;
            AspdError = BitConverter.ToSingle(buffer, index);index+=4;
            XtrackError = BitConverter.ToSingle(buffer, index);index+=4;
            NavBearing = BitConverter.ToInt16(buffer,index);index+=2;
            TargetBearing = BitConverter.ToInt16(buffer,index);index+=2;
            WpDist = BitConverter.ToUInt16(buffer,index);index+=2;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Current desired roll
        /// deg
        /// nav_roll
        /// </summary>
        public float NavRoll { get; set; }
        /// <summary>
        /// Current desired pitch
        /// deg
        /// nav_pitch
        /// </summary>
        public float NavPitch { get; set; }
        /// <summary>
        /// Current altitude error
        /// m
        /// alt_error
        /// </summary>
        public float AltError { get; set; }
        /// <summary>
        /// Current airspeed error
        /// m/s
        /// aspd_error
        /// </summary>
        public float AspdError { get; set; }
        /// <summary>
        /// Current crosstrack error on x-y plane
        /// m
        /// xtrack_error
        /// </summary>
        public float XtrackError { get; set; }
        /// <summary>
        /// Current desired heading
        /// deg
        /// nav_bearing
        /// </summary>
        public short NavBearing { get; set; }
        /// <summary>
        /// Bearing to current waypoint/target
        /// deg
        /// target_bearing
        /// </summary>
        public short TargetBearing { get; set; }
        /// <summary>
        /// Distance to active waypoint
        /// m
        /// wp_dist
        /// </summary>
        public ushort WpDist { get; set; }
    }
    /// <summary>
    /// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.
    ///  GLOBAL_POSITION_INT_COV
    /// </summary>
    public class GlobalPositionIntCovPacket: PacketV2<GlobalPositionIntCovPayload>
    {
        public override byte CrcEtra => 119;
        public override int MessageId => 63;
        public override GlobalPositionIntCovPayload Payload { get; } = new GlobalPositionIntCovPayload();

        public override string Name => "GLOBAL_POSITION_INT_COV";
        public override string ToString()
        {
            var name = "GLOBAL_POSITION_INT_COV".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  GLOBAL_POSITION_INT_COV
    /// </summary>
    public class GlobalPositionIntCovPayload : IPayload
    {
        public byte ByteSize => 181;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToInt32(buffer,index);index+=4;
            RelativeAlt = BitConverter.ToInt32(buffer,index);index+=4;
            Vx = BitConverter.ToSingle(buffer, index);index+=4;
            Vy = BitConverter.ToSingle(buffer, index);index+=4;
            Vz = BitConverter.ToSingle(buffer, index);index+=4;
            for(var i=0;i<36;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            EstimatorType = (MavEstimatorType)buffer[index++];
            return ByteSize;
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
            BitConverter.GetBytes((byte)EstimatorType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Latitude
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude in meters above MSL
        /// mm
        /// alt
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// Altitude above ground
        /// mm
        /// relative_alt
        /// </summary>
        public int RelativeAlt { get; set; }
        /// <summary>
        /// Ground X Speed (Latitude)
        /// m/s
        /// vx
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Ground Y Speed (Longitude)
        /// m/s
        /// vy
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Ground Z Speed (Altitude)
        /// m/s
        /// vz
        /// </summary>
        public float Vz { get; set; }
        /// <summary>
        /// Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
        /// 
        /// covariance
        /// </summary>
        public float[] Covariance { get; } = new float[36];
        /// <summary>
        /// Class id of the estimator this estimate originated from.
        /// 
        /// estimator_type
        /// </summary>
        public MavEstimatorType EstimatorType { get; set; }
    }
    /// <summary>
    /// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
    ///  LOCAL_POSITION_NED_COV
    /// </summary>
    public class LocalPositionNedCovPacket: PacketV2<LocalPositionNedCovPayload>
    {
        public override byte CrcEtra => 191;
        public override int MessageId => 64;
        public override LocalPositionNedCovPayload Payload { get; } = new LocalPositionNedCovPayload();

        public override string Name => "LOCAL_POSITION_NED_COV";
        public override string ToString()
        {
            var name = "LOCAL_POSITION_NED_COV".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  LOCAL_POSITION_NED_COV
    /// </summary>
    public class LocalPositionNedCovPayload : IPayload
    {
        public byte ByteSize => 225;

        public int Deserialize(byte[] buffer, int index)
        {
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
            for(var i=0;i<45;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            EstimatorType = (MavEstimatorType)buffer[index++];
            return ByteSize;
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
            BitConverter.GetBytes((byte)EstimatorType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X Position
        /// m
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y Position
        /// m
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z Position
        /// m
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// X Speed
        /// m/s
        /// vx
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Y Speed
        /// m/s
        /// vy
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Z Speed
        /// m/s
        /// vz
        /// </summary>
        public float Vz { get; set; }
        /// <summary>
        /// X Acceleration
        /// m/s/s
        /// ax
        /// </summary>
        public float Ax { get; set; }
        /// <summary>
        /// Y Acceleration
        /// m/s/s
        /// ay
        /// </summary>
        public float Ay { get; set; }
        /// <summary>
        /// Z Acceleration
        /// m/s/s
        /// az
        /// </summary>
        public float Az { get; set; }
        /// <summary>
        /// Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are the second row, etc.)
        /// 
        /// covariance
        /// </summary>
        public float[] Covariance { get; } = new float[45];
        /// <summary>
        /// Class id of the estimator this estimate originated from.
        /// 
        /// estimator_type
        /// </summary>
        public MavEstimatorType EstimatorType { get; set; }
    }
    /// <summary>
    /// The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.  A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters might violate this specification.
    ///  RC_CHANNELS
    /// </summary>
    public class RcChannelsPacket: PacketV2<RcChannelsPayload>
    {
        public override byte CrcEtra => 118;
        public override int MessageId => 65;
        public override RcChannelsPayload Payload { get; } = new RcChannelsPayload();

        public override string Name => "RC_CHANNELS";
        public override string ToString()
        {
            var name = "RC_CHANNELS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  RC_CHANNELS
    /// </summary>
    public class RcChannelsPayload : IPayload
    {
        public byte ByteSize => 42;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// RC channel 1 value.
        /// us
        /// chan1_raw
        /// </summary>
        public ushort Chan1Raw { get; set; }
        /// <summary>
        /// RC channel 2 value.
        /// us
        /// chan2_raw
        /// </summary>
        public ushort Chan2Raw { get; set; }
        /// <summary>
        /// RC channel 3 value.
        /// us
        /// chan3_raw
        /// </summary>
        public ushort Chan3Raw { get; set; }
        /// <summary>
        /// RC channel 4 value.
        /// us
        /// chan4_raw
        /// </summary>
        public ushort Chan4Raw { get; set; }
        /// <summary>
        /// RC channel 5 value.
        /// us
        /// chan5_raw
        /// </summary>
        public ushort Chan5Raw { get; set; }
        /// <summary>
        /// RC channel 6 value.
        /// us
        /// chan6_raw
        /// </summary>
        public ushort Chan6Raw { get; set; }
        /// <summary>
        /// RC channel 7 value.
        /// us
        /// chan7_raw
        /// </summary>
        public ushort Chan7Raw { get; set; }
        /// <summary>
        /// RC channel 8 value.
        /// us
        /// chan8_raw
        /// </summary>
        public ushort Chan8Raw { get; set; }
        /// <summary>
        /// RC channel 9 value.
        /// us
        /// chan9_raw
        /// </summary>
        public ushort Chan9Raw { get; set; }
        /// <summary>
        /// RC channel 10 value.
        /// us
        /// chan10_raw
        /// </summary>
        public ushort Chan10Raw { get; set; }
        /// <summary>
        /// RC channel 11 value.
        /// us
        /// chan11_raw
        /// </summary>
        public ushort Chan11Raw { get; set; }
        /// <summary>
        /// RC channel 12 value.
        /// us
        /// chan12_raw
        /// </summary>
        public ushort Chan12Raw { get; set; }
        /// <summary>
        /// RC channel 13 value.
        /// us
        /// chan13_raw
        /// </summary>
        public ushort Chan13Raw { get; set; }
        /// <summary>
        /// RC channel 14 value.
        /// us
        /// chan14_raw
        /// </summary>
        public ushort Chan14Raw { get; set; }
        /// <summary>
        /// RC channel 15 value.
        /// us
        /// chan15_raw
        /// </summary>
        public ushort Chan15Raw { get; set; }
        /// <summary>
        /// RC channel 16 value.
        /// us
        /// chan16_raw
        /// </summary>
        public ushort Chan16Raw { get; set; }
        /// <summary>
        /// RC channel 17 value.
        /// us
        /// chan17_raw
        /// </summary>
        public ushort Chan17Raw { get; set; }
        /// <summary>
        /// RC channel 18 value.
        /// us
        /// chan18_raw
        /// </summary>
        public ushort Chan18Raw { get; set; }
        /// <summary>
        /// Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
        /// 
        /// chancount
        /// </summary>
        public byte Chancount { get; set; }
        /// <summary>
        /// Receive signal strength indicator. Values: [0-100], 255: invalid/unknown.
        /// %
        /// rssi
        /// </summary>
        public byte Rssi { get; set; }
    }
    /// <summary>
    /// Request a data stream.
    ///  REQUEST_DATA_STREAM
    /// </summary>
    public class RequestDataStreamPacket: PacketV2<RequestDataStreamPayload>
    {
        public override byte CrcEtra => 148;
        public override int MessageId => 66;
        public override RequestDataStreamPayload Payload { get; } = new RequestDataStreamPayload();

        public override string Name => "REQUEST_DATA_STREAM";
        public override string ToString()
        {
            var name = "REQUEST_DATA_STREAM".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  REQUEST_DATA_STREAM
    /// </summary>
    public class RequestDataStreamPayload : IPayload
    {
        public byte ByteSize => 6;

        public int Deserialize(byte[] buffer, int index)
        {
            ReqMessageRate = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            ReqStreamId = (byte)buffer[index++];
            StartStop = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ReqMessageRate).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(ReqStreamId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(StartStop).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// The requested message rate
        /// Hz
        /// req_message_rate
        /// </summary>
        public ushort ReqMessageRate { get; set; }
        /// <summary>
        /// The target requested to send the message stream.
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// The target requested to send the message stream.
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// The ID of the requested data stream
        /// 
        /// req_stream_id
        /// </summary>
        public byte ReqStreamId { get; set; }
        /// <summary>
        /// 1 to start sending, 0 to stop sending.
        /// 
        /// start_stop
        /// </summary>
        public byte StartStop { get; set; }
    }
    /// <summary>
    /// Data stream status information.
    ///  DATA_STREAM
    /// </summary>
    public class DataStreamPacket: PacketV2<DataStreamPayload>
    {
        public override byte CrcEtra => 21;
        public override int MessageId => 67;
        public override DataStreamPayload Payload { get; } = new DataStreamPayload();

        public override string Name => "DATA_STREAM";
        public override string ToString()
        {
            var name = "DATA_STREAM".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  DATA_STREAM
    /// </summary>
    public class DataStreamPayload : IPayload
    {
        public byte ByteSize => 4;

        public int Deserialize(byte[] buffer, int index)
        {
            MessageRate = BitConverter.ToUInt16(buffer,index);index+=2;
            StreamId = (byte)buffer[index++];
            OnOff = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(MessageRate).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(StreamId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(OnOff).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// The message rate
        /// Hz
        /// message_rate
        /// </summary>
        public ushort MessageRate { get; set; }
        /// <summary>
        /// The ID of the requested data stream
        /// 
        /// stream_id
        /// </summary>
        public byte StreamId { get; set; }
        /// <summary>
        /// 1 stream is enabled, 0 stream is stopped.
        /// 
        /// on_off
        /// </summary>
        public byte OnOff { get; set; }
    }
    /// <summary>
    /// This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their 
    ///  MANUAL_CONTROL
    /// </summary>
    public class ManualControlPacket: PacketV2<ManualControlPayload>
    {
        public override byte CrcEtra => 243;
        public override int MessageId => 69;
        public override ManualControlPayload Payload { get; } = new ManualControlPayload();

        public override string Name => "MANUAL_CONTROL";
        public override string ToString()
        {
            var name = "MANUAL_CONTROL".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MANUAL_CONTROL
    /// </summary>
    public class ManualControlPayload : IPayload
    {
        public byte ByteSize => 11;

        public int Deserialize(byte[] buffer, int index)
        {
            X = BitConverter.ToInt16(buffer,index);index+=2;
            Y = BitConverter.ToInt16(buffer,index);index+=2;
            Z = BitConverter.ToInt16(buffer,index);index+=2;
            R = BitConverter.ToInt16(buffer,index);index+=2;
            Buttons = BitConverter.ToUInt16(buffer,index);index+=2;
            Target = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(R).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Buttons).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Target).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
        /// 
        /// x
        /// </summary>
        public short X { get; set; }
        /// <summary>
        /// Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
        /// 
        /// y
        /// </summary>
        public short Y { get; set; }
        /// <summary>
        /// Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.
        /// 
        /// z
        /// </summary>
        public short Z { get; set; }
        /// <summary>
        /// R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
        /// 
        /// r
        /// </summary>
        public short R { get; set; }
        /// <summary>
        /// A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
        /// 
        /// buttons
        /// </summary>
        public ushort Buttons { get; set; }
        /// <summary>
        /// The system to be controlled.
        /// 
        /// target
        /// </summary>
        public byte Target { get; set; }
    }
    /// <summary>
    /// The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
    ///  RC_CHANNELS_OVERRIDE
    /// </summary>
    public class RcChannelsOverridePacket: PacketV2<RcChannelsOverridePayload>
    {
        public override byte CrcEtra => 124;
        public override int MessageId => 70;
        public override RcChannelsOverridePayload Payload { get; } = new RcChannelsOverridePayload();

        public override string Name => "RC_CHANNELS_OVERRIDE";
        public override string ToString()
        {
            var name = "RC_CHANNELS_OVERRIDE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  RC_CHANNELS_OVERRIDE
    /// </summary>
    public class RcChannelsOverridePayload : IPayload
    {
        public byte ByteSize => 18;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// RC channel 1 value. A value of UINT16_MAX means to ignore this field.
        /// us
        /// chan1_raw
        /// </summary>
        public ushort Chan1Raw { get; set; }
        /// <summary>
        /// RC channel 2 value. A value of UINT16_MAX means to ignore this field.
        /// us
        /// chan2_raw
        /// </summary>
        public ushort Chan2Raw { get; set; }
        /// <summary>
        /// RC channel 3 value. A value of UINT16_MAX means to ignore this field.
        /// us
        /// chan3_raw
        /// </summary>
        public ushort Chan3Raw { get; set; }
        /// <summary>
        /// RC channel 4 value. A value of UINT16_MAX means to ignore this field.
        /// us
        /// chan4_raw
        /// </summary>
        public ushort Chan4Raw { get; set; }
        /// <summary>
        /// RC channel 5 value. A value of UINT16_MAX means to ignore this field.
        /// us
        /// chan5_raw
        /// </summary>
        public ushort Chan5Raw { get; set; }
        /// <summary>
        /// RC channel 6 value. A value of UINT16_MAX means to ignore this field.
        /// us
        /// chan6_raw
        /// </summary>
        public ushort Chan6Raw { get; set; }
        /// <summary>
        /// RC channel 7 value. A value of UINT16_MAX means to ignore this field.
        /// us
        /// chan7_raw
        /// </summary>
        public ushort Chan7Raw { get; set; }
        /// <summary>
        /// RC channel 8 value. A value of UINT16_MAX means to ignore this field.
        /// us
        /// chan8_raw
        /// </summary>
        public ushort Chan8Raw { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// RC channel 9 value. A value of 0 or UINT16_MAX means to ignore this field.
        /// us
        /// chan9_raw
        /// </summary>
        public ushort Chan9Raw { get; set; }
        /// <summary>
        /// RC channel 10 value. A value of 0 or UINT16_MAX means to ignore this field.
        /// us
        /// chan10_raw
        /// </summary>
        public ushort Chan10Raw { get; set; }
        /// <summary>
        /// RC channel 11 value. A value of 0 or UINT16_MAX means to ignore this field.
        /// us
        /// chan11_raw
        /// </summary>
        public ushort Chan11Raw { get; set; }
        /// <summary>
        /// RC channel 12 value. A value of 0 or UINT16_MAX means to ignore this field.
        /// us
        /// chan12_raw
        /// </summary>
        public ushort Chan12Raw { get; set; }
        /// <summary>
        /// RC channel 13 value. A value of 0 or UINT16_MAX means to ignore this field.
        /// us
        /// chan13_raw
        /// </summary>
        public ushort Chan13Raw { get; set; }
        /// <summary>
        /// RC channel 14 value. A value of 0 or UINT16_MAX means to ignore this field.
        /// us
        /// chan14_raw
        /// </summary>
        public ushort Chan14Raw { get; set; }
        /// <summary>
        /// RC channel 15 value. A value of 0 or UINT16_MAX means to ignore this field.
        /// us
        /// chan15_raw
        /// </summary>
        public ushort Chan15Raw { get; set; }
        /// <summary>
        /// RC channel 16 value. A value of 0 or UINT16_MAX means to ignore this field.
        /// us
        /// chan16_raw
        /// </summary>
        public ushort Chan16Raw { get; set; }
        /// <summary>
        /// RC channel 17 value. A value of 0 or UINT16_MAX means to ignore this field.
        /// us
        /// chan17_raw
        /// </summary>
        public ushort Chan17Raw { get; set; }
        /// <summary>
        /// RC channel 18 value. A value of 0 or UINT16_MAX means to ignore this field.
        /// us
        /// chan18_raw
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
        public override byte CrcEtra => 38;
        public override int MessageId => 73;
        public override MissionItemIntPayload Payload { get; } = new MissionItemIntPayload();

        public override string Name => "MISSION_ITEM_INT";
        public override string ToString()
        {
            var name = "MISSION_ITEM_INT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MISSION_ITEM_INT
    /// </summary>
    public class MissionItemIntPayload : IPayload
    {
        public byte ByteSize => 37;

        public int Deserialize(byte[] buffer, int index)
        {
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
            MissionType = (MavMissionType)buffer[index++];
            return ByteSize;
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
            BitConverter.GetBytes((byte)Frame).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Current).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Autocontinue).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)MissionType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// PARAM1, see MAV_CMD enum
        /// 
        /// param1
        /// </summary>
        public float Param1 { get; set; }
        /// <summary>
        /// PARAM2, see MAV_CMD enum
        /// 
        /// param2
        /// </summary>
        public float Param2 { get; set; }
        /// <summary>
        /// PARAM3, see MAV_CMD enum
        /// 
        /// param3
        /// </summary>
        public float Param3 { get; set; }
        /// <summary>
        /// PARAM4, see MAV_CMD enum
        /// 
        /// param4
        /// </summary>
        public float Param4 { get; set; }
        /// <summary>
        /// PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
        /// 
        /// x
        /// </summary>
        public int X { get; set; }
        /// <summary>
        /// PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
        /// 
        /// y
        /// </summary>
        public int Y { get; set; }
        /// <summary>
        /// PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
        /// 
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
        /// 
        /// seq
        /// </summary>
        public ushort Seq { get; set; }
        /// <summary>
        /// The scheduled action for the waypoint.
        /// 
        /// command
        /// </summary>
        public MavCmd Command { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// The coordinate system of the waypoint.
        /// 
        /// frame
        /// </summary>
        public MavFrame Frame { get; set; }
        /// <summary>
        /// false:0, true:1
        /// 
        /// current
        /// </summary>
        public byte Current { get; set; }
        /// <summary>
        /// Autocontinue to next waypoint
        /// 
        /// autocontinue
        /// </summary>
        public byte Autocontinue { get; set; }
        /// <summary>
        /// Mission type.
        /// 
        /// mission_type
        /// </summary>
        public MavMissionType MissionType { get; set; }
    }
    /// <summary>
    /// Metrics typically displayed on a HUD for fixed wing aircraft
    ///  VFR_HUD
    /// </summary>
    public class VfrHudPacket: PacketV2<VfrHudPayload>
    {
        public override byte CrcEtra => 20;
        public override int MessageId => 74;
        public override VfrHudPayload Payload { get; } = new VfrHudPayload();

        public override string Name => "VFR_HUD";
        public override string ToString()
        {
            var name = "VFR_HUD".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  VFR_HUD
    /// </summary>
    public class VfrHudPayload : IPayload
    {
        public byte ByteSize => 20;

        public int Deserialize(byte[] buffer, int index)
        {
            Airspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Groundspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Alt = BitConverter.ToSingle(buffer, index);index+=4;
            Climb = BitConverter.ToSingle(buffer, index);index+=4;
            Heading = BitConverter.ToInt16(buffer,index);index+=2;
            Throttle = BitConverter.ToUInt16(buffer,index);index+=2;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Airspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Groundspeed).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Alt).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Climb).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Heading).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Throttle).CopyTo(buffer, index);index+=2;
            return ByteSize;
        }

        /// <summary>
        /// Current airspeed
        /// m/s
        /// airspeed
        /// </summary>
        public float Airspeed { get; set; }
        /// <summary>
        /// Current ground speed
        /// m/s
        /// groundspeed
        /// </summary>
        public float Groundspeed { get; set; }
        /// <summary>
        /// Current altitude (MSL)
        /// m
        /// alt
        /// </summary>
        public float Alt { get; set; }
        /// <summary>
        /// Current climb rate
        /// m/s
        /// climb
        /// </summary>
        public float Climb { get; set; }
        /// <summary>
        /// Current heading in degrees, in compass units (0..360, 0=north)
        /// deg
        /// heading
        /// </summary>
        public short Heading { get; set; }
        /// <summary>
        /// Current throttle setting in integer percent, 0 to 100
        /// %
        /// throttle
        /// </summary>
        public ushort Throttle { get; set; }
    }
    /// <summary>
    /// Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value.
    ///  COMMAND_INT
    /// </summary>
    public class CommandIntPacket: PacketV2<CommandIntPayload>
    {
        public override byte CrcEtra => 158;
        public override int MessageId => 75;
        public override CommandIntPayload Payload { get; } = new CommandIntPayload();

        public override string Name => "COMMAND_INT";
        public override string ToString()
        {
            var name = "COMMAND_INT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  COMMAND_INT
    /// </summary>
    public class CommandIntPayload : IPayload
    {
        public byte ByteSize => 35;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            BitConverter.GetBytes((byte)Frame).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Current).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Autocontinue).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// PARAM1, see MAV_CMD enum
        /// 
        /// param1
        /// </summary>
        public float Param1 { get; set; }
        /// <summary>
        /// PARAM2, see MAV_CMD enum
        /// 
        /// param2
        /// </summary>
        public float Param2 { get; set; }
        /// <summary>
        /// PARAM3, see MAV_CMD enum
        /// 
        /// param3
        /// </summary>
        public float Param3 { get; set; }
        /// <summary>
        /// PARAM4, see MAV_CMD enum
        /// 
        /// param4
        /// </summary>
        public float Param4 { get; set; }
        /// <summary>
        /// PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
        /// 
        /// x
        /// </summary>
        public int X { get; set; }
        /// <summary>
        /// PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
        /// 
        /// y
        /// </summary>
        public int Y { get; set; }
        /// <summary>
        /// PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame).
        /// 
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// The scheduled action for the mission item.
        /// 
        /// command
        /// </summary>
        public MavCmd Command { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// The coordinate system of the COMMAND.
        /// 
        /// frame
        /// </summary>
        public MavFrame Frame { get; set; }
        /// <summary>
        /// false:0, true:1
        /// 
        /// current
        /// </summary>
        public byte Current { get; set; }
        /// <summary>
        /// autocontinue to next wp
        /// 
        /// autocontinue
        /// </summary>
        public byte Autocontinue { get; set; }
    }
    /// <summary>
    /// Send a command with up to seven parameters to the MAV
    ///  COMMAND_LONG
    /// </summary>
    public class CommandLongPacket: PacketV2<CommandLongPayload>
    {
        public override byte CrcEtra => 152;
        public override int MessageId => 76;
        public override CommandLongPayload Payload { get; } = new CommandLongPayload();

        public override string Name => "COMMAND_LONG";
        public override string ToString()
        {
            var name = "COMMAND_LONG".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  COMMAND_LONG
    /// </summary>
    public class CommandLongPayload : IPayload
    {
        public byte ByteSize => 33;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Parameter 1 (for the specific command).
        /// 
        /// param1
        /// </summary>
        public float Param1 { get; set; }
        /// <summary>
        /// Parameter 2 (for the specific command).
        /// 
        /// param2
        /// </summary>
        public float Param2 { get; set; }
        /// <summary>
        /// Parameter 3 (for the specific command).
        /// 
        /// param3
        /// </summary>
        public float Param3 { get; set; }
        /// <summary>
        /// Parameter 4 (for the specific command).
        /// 
        /// param4
        /// </summary>
        public float Param4 { get; set; }
        /// <summary>
        /// Parameter 5 (for the specific command).
        /// 
        /// param5
        /// </summary>
        public float Param5 { get; set; }
        /// <summary>
        /// Parameter 6 (for the specific command).
        /// 
        /// param6
        /// </summary>
        public float Param6 { get; set; }
        /// <summary>
        /// Parameter 7 (for the specific command).
        /// 
        /// param7
        /// </summary>
        public float Param7 { get; set; }
        /// <summary>
        /// Command ID (of command to send).
        /// 
        /// command
        /// </summary>
        public MavCmd Command { get; set; }
        /// <summary>
        /// System which should execute the command
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component which should execute the command, 0 for all components
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
        /// 
        /// confirmation
        /// </summary>
        public byte Confirmation { get; set; }
    }
    /// <summary>
    /// Report status of a command. Includes feedback whether the command was executed.
    ///  COMMAND_ACK
    /// </summary>
    public class CommandAckPacket: PacketV2<CommandAckPayload>
    {
        public override byte CrcEtra => 143;
        public override int MessageId => 77;
        public override CommandAckPayload Payload { get; } = new CommandAckPayload();

        public override string Name => "COMMAND_ACK";
        public override string ToString()
        {
            var name = "COMMAND_ACK".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  COMMAND_ACK
    /// </summary>
    public class CommandAckPayload : IPayload
    {
        public byte ByteSize => 3;

        public int Deserialize(byte[] buffer, int index)
        {
            Command = (MavCmd)BitConverter.ToUInt16(buffer,index);index+=2;
            Result = (MavResult)buffer[index++];
            Progress = (byte)buffer[index++];
            ResultParam2 = BitConverter.ToInt32(buffer,index);index+=4;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes((ushort)Command).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes((byte)Result).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Progress).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(ResultParam2).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Command ID (of acknowledged command).
        /// 
        /// command
        /// </summary>
        public MavCmd Command { get; set; }
        /// <summary>
        /// Result of command.
        /// 
        /// result
        /// </summary>
        public MavResult Result { get; set; }
        /// <summary>
        /// WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS.
        /// 
        /// progress
        /// </summary>
        public byte Progress { get; set; }
        /// <summary>
        /// WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied.
        /// 
        /// result_param2
        /// </summary>
        public int ResultParam2 { get; set; }
        /// <summary>
        /// WIP: System which requested the command to be executed
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// WIP: Component which requested the command to be executed
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Setpoint in roll, pitch, yaw and thrust from the operator
    ///  MANUAL_SETPOINT
    /// </summary>
    public class ManualSetpointPacket: PacketV2<ManualSetpointPayload>
    {
        public override byte CrcEtra => 106;
        public override int MessageId => 81;
        public override ManualSetpointPayload Payload { get; } = new ManualSetpointPayload();

        public override string Name => "MANUAL_SETPOINT";
        public override string ToString()
        {
            var name = "MANUAL_SETPOINT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MANUAL_SETPOINT
    /// </summary>
    public class ManualSetpointPayload : IPayload
    {
        public byte ByteSize => 22;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            Thrust = BitConverter.ToSingle(buffer, index);index+=4;
            ModeSwitch = (byte)buffer[index++];
            ManualOverrideSwitch = (byte)buffer[index++];
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Desired roll rate
        /// rad/s
        /// roll
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Desired pitch rate
        /// rad/s
        /// pitch
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Desired yaw rate
        /// rad/s
        /// yaw
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// Collective thrust, normalized to 0 .. 1
        /// 
        /// thrust
        /// </summary>
        public float Thrust { get; set; }
        /// <summary>
        /// Flight mode switch position, 0.. 255
        /// 
        /// mode_switch
        /// </summary>
        public byte ModeSwitch { get; set; }
        /// <summary>
        /// Override mode switch position, 0.. 255
        /// 
        /// manual_override_switch
        /// </summary>
        public byte ManualOverrideSwitch { get; set; }
    }
    /// <summary>
    /// Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system).
    ///  SET_ATTITUDE_TARGET
    /// </summary>
    public class SetAttitudeTargetPacket: PacketV2<SetAttitudeTargetPayload>
    {
        public override byte CrcEtra => 49;
        public override int MessageId => 82;
        public override SetAttitudeTargetPayload Payload { get; } = new SetAttitudeTargetPayload();

        public override string Name => "SET_ATTITUDE_TARGET";
        public override string ToString()
        {
            var name = "SET_ATTITUDE_TARGET".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SET_ATTITUDE_TARGET
    /// </summary>
    public class SetAttitudeTargetPayload : IPayload
    {
        public byte ByteSize => 39;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            for(var i=0;i<4;i++)
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        /// 
        /// q
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Body roll rate
        /// rad/s
        /// body_roll_rate
        /// </summary>
        public float BodyRollRate { get; set; }
        /// <summary>
        /// Body pitch rate
        /// rad/s
        /// body_pitch_rate
        /// </summary>
        public float BodyPitchRate { get; set; }
        /// <summary>
        /// Body yaw rate
        /// rad/s
        /// body_yaw_rate
        /// </summary>
        public float BodyYawRate { get; set; }
        /// <summary>
        /// Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
        /// 
        /// thrust
        /// </summary>
        public float Thrust { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
        /// 
        /// type_mask
        /// </summary>
        public byte TypeMask { get; set; }
    }
    /// <summary>
    /// Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.
    ///  ATTITUDE_TARGET
    /// </summary>
    public class AttitudeTargetPacket: PacketV2<AttitudeTargetPayload>
    {
        public override byte CrcEtra => 22;
        public override int MessageId => 83;
        public override AttitudeTargetPayload Payload { get; } = new AttitudeTargetPayload();

        public override string Name => "ATTITUDE_TARGET";
        public override string ToString()
        {
            var name = "ATTITUDE_TARGET".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  ATTITUDE_TARGET
    /// </summary>
    public class AttitudeTargetPayload : IPayload
    {
        public byte ByteSize => 37;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            for(var i=0;i<4;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            BodyRollRate = BitConverter.ToSingle(buffer, index);index+=4;
            BodyPitchRate = BitConverter.ToSingle(buffer, index);index+=4;
            BodyYawRate = BitConverter.ToSingle(buffer, index);index+=4;
            Thrust = BitConverter.ToSingle(buffer, index);index+=4;
            TypeMask = (byte)buffer[index++];
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        /// 
        /// q
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Body roll rate
        /// rad/s
        /// body_roll_rate
        /// </summary>
        public float BodyRollRate { get; set; }
        /// <summary>
        /// Body pitch rate
        /// rad/s
        /// body_pitch_rate
        /// </summary>
        public float BodyPitchRate { get; set; }
        /// <summary>
        /// Body yaw rate
        /// rad/s
        /// body_yaw_rate
        /// </summary>
        public float BodyYawRate { get; set; }
        /// <summary>
        /// Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
        /// 
        /// thrust
        /// </summary>
        public float Thrust { get; set; }
        /// <summary>
        /// Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude
        /// 
        /// type_mask
        /// </summary>
        public byte TypeMask { get; set; }
    }
    /// <summary>
    /// Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system).
    ///  SET_POSITION_TARGET_LOCAL_NED
    /// </summary>
    public class SetPositionTargetLocalNedPacket: PacketV2<SetPositionTargetLocalNedPayload>
    {
        public override byte CrcEtra => 143;
        public override int MessageId => 84;
        public override SetPositionTargetLocalNedPayload Payload { get; } = new SetPositionTargetLocalNedPayload();

        public override string Name => "SET_POSITION_TARGET_LOCAL_NED";
        public override string ToString()
        {
            var name = "SET_POSITION_TARGET_LOCAL_NED".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SET_POSITION_TARGET_LOCAL_NED
    /// </summary>
    public class SetPositionTargetLocalNedPayload : IPayload
    {
        public byte ByteSize => 53;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            BitConverter.GetBytes((byte)CoordinateFrame).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X Position in NED frame
        /// m
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y Position in NED frame
        /// m
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z Position in NED frame (note, altitude is negative in NED)
        /// m
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// X velocity in NED frame
        /// m/s
        /// vx
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Y velocity in NED frame
        /// m/s
        /// vy
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Z velocity in NED frame
        /// m/s
        /// vz
        /// </summary>
        public float Vz { get; set; }
        /// <summary>
        /// X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// m/s/s
        /// afx
        /// </summary>
        public float Afx { get; set; }
        /// <summary>
        /// Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// m/s/s
        /// afy
        /// </summary>
        public float Afy { get; set; }
        /// <summary>
        /// Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// m/s/s
        /// afz
        /// </summary>
        public float Afz { get; set; }
        /// <summary>
        /// yaw setpoint
        /// rad
        /// yaw
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// yaw rate setpoint
        /// rad/s
        /// yaw_rate
        /// </summary>
        public float YawRate { get; set; }
        /// <summary>
        /// Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
        /// 
        /// type_mask
        /// </summary>
        public ushort TypeMask { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
        /// 
        /// coordinate_frame
        /// </summary>
        public MavFrame CoordinateFrame { get; set; }
    }
    /// <summary>
    /// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled this way.
    ///  POSITION_TARGET_LOCAL_NED
    /// </summary>
    public class PositionTargetLocalNedPacket: PacketV2<PositionTargetLocalNedPayload>
    {
        public override byte CrcEtra => 140;
        public override int MessageId => 85;
        public override PositionTargetLocalNedPayload Payload { get; } = new PositionTargetLocalNedPayload();

        public override string Name => "POSITION_TARGET_LOCAL_NED";
        public override string ToString()
        {
            var name = "POSITION_TARGET_LOCAL_NED".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  POSITION_TARGET_LOCAL_NED
    /// </summary>
    public class PositionTargetLocalNedPayload : IPayload
    {
        public byte ByteSize => 51;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            BitConverter.GetBytes((byte)CoordinateFrame).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X Position in NED frame
        /// m
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y Position in NED frame
        /// m
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z Position in NED frame (note, altitude is negative in NED)
        /// m
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// X velocity in NED frame
        /// m/s
        /// vx
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Y velocity in NED frame
        /// m/s
        /// vy
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Z velocity in NED frame
        /// m/s
        /// vz
        /// </summary>
        public float Vz { get; set; }
        /// <summary>
        /// X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// m/s/s
        /// afx
        /// </summary>
        public float Afx { get; set; }
        /// <summary>
        /// Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// m/s/s
        /// afy
        /// </summary>
        public float Afy { get; set; }
        /// <summary>
        /// Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// m/s/s
        /// afz
        /// </summary>
        public float Afz { get; set; }
        /// <summary>
        /// yaw setpoint
        /// rad
        /// yaw
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// yaw rate setpoint
        /// rad/s
        /// yaw_rate
        /// </summary>
        public float YawRate { get; set; }
        /// <summary>
        /// Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
        /// 
        /// type_mask
        /// </summary>
        public ushort TypeMask { get; set; }
        /// <summary>
        /// Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
        /// 
        /// coordinate_frame
        /// </summary>
        public MavFrame CoordinateFrame { get; set; }
    }
    /// <summary>
    /// Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84). Used by an external controller to command the vehicle (manual controller or other system).
    ///  SET_POSITION_TARGET_GLOBAL_INT
    /// </summary>
    public class SetPositionTargetGlobalIntPacket: PacketV2<SetPositionTargetGlobalIntPayload>
    {
        public override byte CrcEtra => 5;
        public override int MessageId => 86;
        public override SetPositionTargetGlobalIntPayload Payload { get; } = new SetPositionTargetGlobalIntPayload();

        public override string Name => "SET_POSITION_TARGET_GLOBAL_INT";
        public override string ToString()
        {
            var name = "SET_POSITION_TARGET_GLOBAL_INT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SET_POSITION_TARGET_GLOBAL_INT
    /// </summary>
    public class SetPositionTargetGlobalIntPayload : IPayload
    {
        public byte ByteSize => 53;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            BitConverter.GetBytes((byte)CoordinateFrame).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X Position in WGS84 frame
        /// degE7
        /// lat_int
        /// </summary>
        public int LatInt { get; set; }
        /// <summary>
        /// Y Position in WGS84 frame
        /// degE7
        /// lon_int
        /// </summary>
        public int LonInt { get; set; }
        /// <summary>
        /// Altitude (AMSL) if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        /// m
        /// alt
        /// </summary>
        public float Alt { get; set; }
        /// <summary>
        /// X velocity in NED frame
        /// m/s
        /// vx
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Y velocity in NED frame
        /// m/s
        /// vy
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Z velocity in NED frame
        /// m/s
        /// vz
        /// </summary>
        public float Vz { get; set; }
        /// <summary>
        /// X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// m/s/s
        /// afx
        /// </summary>
        public float Afx { get; set; }
        /// <summary>
        /// Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// m/s/s
        /// afy
        /// </summary>
        public float Afy { get; set; }
        /// <summary>
        /// Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// m/s/s
        /// afz
        /// </summary>
        public float Afz { get; set; }
        /// <summary>
        /// yaw setpoint
        /// rad
        /// yaw
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// yaw rate setpoint
        /// rad/s
        /// yaw_rate
        /// </summary>
        public float YawRate { get; set; }
        /// <summary>
        /// Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
        /// 
        /// type_mask
        /// </summary>
        public ushort TypeMask { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
        /// 
        /// coordinate_frame
        /// </summary>
        public MavFrame CoordinateFrame { get; set; }
    }
    /// <summary>
    /// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way.
    ///  POSITION_TARGET_GLOBAL_INT
    /// </summary>
    public class PositionTargetGlobalIntPacket: PacketV2<PositionTargetGlobalIntPayload>
    {
        public override byte CrcEtra => 150;
        public override int MessageId => 87;
        public override PositionTargetGlobalIntPayload Payload { get; } = new PositionTargetGlobalIntPayload();

        public override string Name => "POSITION_TARGET_GLOBAL_INT";
        public override string ToString()
        {
            var name = "POSITION_TARGET_GLOBAL_INT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  POSITION_TARGET_GLOBAL_INT
    /// </summary>
    public class PositionTargetGlobalIntPayload : IPayload
    {
        public byte ByteSize => 51;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            BitConverter.GetBytes((byte)CoordinateFrame).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X Position in WGS84 frame
        /// degE7
        /// lat_int
        /// </summary>
        public int LatInt { get; set; }
        /// <summary>
        /// Y Position in WGS84 frame
        /// degE7
        /// lon_int
        /// </summary>
        public int LonInt { get; set; }
        /// <summary>
        /// Altitude (AMSL) if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        /// m
        /// alt
        /// </summary>
        public float Alt { get; set; }
        /// <summary>
        /// X velocity in NED frame
        /// m/s
        /// vx
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Y velocity in NED frame
        /// m/s
        /// vy
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Z velocity in NED frame
        /// m/s
        /// vz
        /// </summary>
        public float Vz { get; set; }
        /// <summary>
        /// X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// m/s/s
        /// afx
        /// </summary>
        public float Afx { get; set; }
        /// <summary>
        /// Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// m/s/s
        /// afy
        /// </summary>
        public float Afy { get; set; }
        /// <summary>
        /// Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        /// m/s/s
        /// afz
        /// </summary>
        public float Afz { get; set; }
        /// <summary>
        /// yaw setpoint
        /// rad
        /// yaw
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// yaw rate setpoint
        /// rad/s
        /// yaw_rate
        /// </summary>
        public float YawRate { get; set; }
        /// <summary>
        /// Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
        /// 
        /// type_mask
        /// </summary>
        public ushort TypeMask { get; set; }
        /// <summary>
        /// Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
        /// 
        /// coordinate_frame
        /// </summary>
        public MavFrame CoordinateFrame { get; set; }
    }
    /// <summary>
    /// The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
    ///  LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
    /// </summary>
    public class LocalPositionNedSystemGlobalOffsetPacket: PacketV2<LocalPositionNedSystemGlobalOffsetPayload>
    {
        public override byte CrcEtra => 231;
        public override int MessageId => 89;
        public override LocalPositionNedSystemGlobalOffsetPayload Payload { get; } = new LocalPositionNedSystemGlobalOffsetPayload();

        public override string Name => "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET";
        public override string ToString()
        {
            var name = "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
    /// </summary>
    public class LocalPositionNedSystemGlobalOffsetPayload : IPayload
    {
        public byte ByteSize => 28;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X Position
        /// m
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y Position
        /// m
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z Position
        /// m
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Roll
        /// rad
        /// roll
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Pitch
        /// rad
        /// pitch
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Yaw
        /// rad
        /// yaw
        /// </summary>
        public float Yaw { get; set; }
    }
    /// <summary>
    /// Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
    ///  HIL_STATE
    /// </summary>
    public class HilStatePacket: PacketV2<HilStatePayload>
    {
        public override byte CrcEtra => 183;
        public override int MessageId => 90;
        public override HilStatePayload Payload { get; } = new HilStatePayload();

        public override string Name => "HIL_STATE";
        public override string ToString()
        {
            var name = "HIL_STATE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  HIL_STATE
    /// </summary>
    public class HilStatePayload : IPayload
    {
        public byte ByteSize => 56;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Roll angle
        /// rad
        /// roll
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Pitch angle
        /// rad
        /// pitch
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Yaw angle
        /// rad
        /// yaw
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// Body frame roll / phi angular speed
        /// rad/s
        /// rollspeed
        /// </summary>
        public float Rollspeed { get; set; }
        /// <summary>
        /// Body frame pitch / theta angular speed
        /// rad/s
        /// pitchspeed
        /// </summary>
        public float Pitchspeed { get; set; }
        /// <summary>
        /// Body frame yaw / psi angular speed
        /// rad/s
        /// yawspeed
        /// </summary>
        public float Yawspeed { get; set; }
        /// <summary>
        /// Latitude
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude
        /// mm
        /// alt
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// Ground X Speed (Latitude)
        /// cm/s
        /// vx
        /// </summary>
        public short Vx { get; set; }
        /// <summary>
        /// Ground Y Speed (Longitude)
        /// cm/s
        /// vy
        /// </summary>
        public short Vy { get; set; }
        /// <summary>
        /// Ground Z Speed (Altitude)
        /// cm/s
        /// vz
        /// </summary>
        public short Vz { get; set; }
        /// <summary>
        /// X acceleration
        /// mG
        /// xacc
        /// </summary>
        public short Xacc { get; set; }
        /// <summary>
        /// Y acceleration
        /// mG
        /// yacc
        /// </summary>
        public short Yacc { get; set; }
        /// <summary>
        /// Z acceleration
        /// mG
        /// zacc
        /// </summary>
        public short Zacc { get; set; }
    }
    /// <summary>
    /// Sent from autopilot to simulation. Hardware in the loop control outputs
    ///  HIL_CONTROLS
    /// </summary>
    public class HilControlsPacket: PacketV2<HilControlsPayload>
    {
        public override byte CrcEtra => 63;
        public override int MessageId => 91;
        public override HilControlsPayload Payload { get; } = new HilControlsPayload();

        public override string Name => "HIL_CONTROLS";
        public override string ToString()
        {
            var name = "HIL_CONTROLS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  HIL_CONTROLS
    /// </summary>
    public class HilControlsPayload : IPayload
    {
        public byte ByteSize => 42;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            BitConverter.GetBytes((byte)Mode).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(NavMode).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Control output -1 .. 1
        /// 
        /// roll_ailerons
        /// </summary>
        public float RollAilerons { get; set; }
        /// <summary>
        /// Control output -1 .. 1
        /// 
        /// pitch_elevator
        /// </summary>
        public float PitchElevator { get; set; }
        /// <summary>
        /// Control output -1 .. 1
        /// 
        /// yaw_rudder
        /// </summary>
        public float YawRudder { get; set; }
        /// <summary>
        /// Throttle 0 .. 1
        /// 
        /// throttle
        /// </summary>
        public float Throttle { get; set; }
        /// <summary>
        /// Aux 1, -1 .. 1
        /// 
        /// aux1
        /// </summary>
        public float Aux1 { get; set; }
        /// <summary>
        /// Aux 2, -1 .. 1
        /// 
        /// aux2
        /// </summary>
        public float Aux2 { get; set; }
        /// <summary>
        /// Aux 3, -1 .. 1
        /// 
        /// aux3
        /// </summary>
        public float Aux3 { get; set; }
        /// <summary>
        /// Aux 4, -1 .. 1
        /// 
        /// aux4
        /// </summary>
        public float Aux4 { get; set; }
        /// <summary>
        /// System mode.
        /// 
        /// mode
        /// </summary>
        public MavMode Mode { get; set; }
        /// <summary>
        /// Navigation mode (MAV_NAV_MODE)
        /// 
        /// nav_mode
        /// </summary>
        public byte NavMode { get; set; }
    }
    /// <summary>
    /// Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
    ///  HIL_RC_INPUTS_RAW
    /// </summary>
    public class HilRcInputsRawPacket: PacketV2<HilRcInputsRawPayload>
    {
        public override byte CrcEtra => 54;
        public override int MessageId => 92;
        public override HilRcInputsRawPayload Payload { get; } = new HilRcInputsRawPayload();

        public override string Name => "HIL_RC_INPUTS_RAW";
        public override string ToString()
        {
            var name = "HIL_RC_INPUTS_RAW".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  HIL_RC_INPUTS_RAW
    /// </summary>
    public class HilRcInputsRawPayload : IPayload
    {
        public byte ByteSize => 33;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// RC channel 1 value
        /// us
        /// chan1_raw
        /// </summary>
        public ushort Chan1Raw { get; set; }
        /// <summary>
        /// RC channel 2 value
        /// us
        /// chan2_raw
        /// </summary>
        public ushort Chan2Raw { get; set; }
        /// <summary>
        /// RC channel 3 value
        /// us
        /// chan3_raw
        /// </summary>
        public ushort Chan3Raw { get; set; }
        /// <summary>
        /// RC channel 4 value
        /// us
        /// chan4_raw
        /// </summary>
        public ushort Chan4Raw { get; set; }
        /// <summary>
        /// RC channel 5 value
        /// us
        /// chan5_raw
        /// </summary>
        public ushort Chan5Raw { get; set; }
        /// <summary>
        /// RC channel 6 value
        /// us
        /// chan6_raw
        /// </summary>
        public ushort Chan6Raw { get; set; }
        /// <summary>
        /// RC channel 7 value
        /// us
        /// chan7_raw
        /// </summary>
        public ushort Chan7Raw { get; set; }
        /// <summary>
        /// RC channel 8 value
        /// us
        /// chan8_raw
        /// </summary>
        public ushort Chan8Raw { get; set; }
        /// <summary>
        /// RC channel 9 value
        /// us
        /// chan9_raw
        /// </summary>
        public ushort Chan9Raw { get; set; }
        /// <summary>
        /// RC channel 10 value
        /// us
        /// chan10_raw
        /// </summary>
        public ushort Chan10Raw { get; set; }
        /// <summary>
        /// RC channel 11 value
        /// us
        /// chan11_raw
        /// </summary>
        public ushort Chan11Raw { get; set; }
        /// <summary>
        /// RC channel 12 value
        /// us
        /// chan12_raw
        /// </summary>
        public ushort Chan12Raw { get; set; }
        /// <summary>
        /// Receive signal strength indicator. Values: [0-100], 255: invalid/unknown.
        /// 
        /// rssi
        /// </summary>
        public byte Rssi { get; set; }
    }
    /// <summary>
    /// Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS)
    ///  HIL_ACTUATOR_CONTROLS
    /// </summary>
    public class HilActuatorControlsPacket: PacketV2<HilActuatorControlsPayload>
    {
        public override byte CrcEtra => 47;
        public override int MessageId => 93;
        public override HilActuatorControlsPayload Payload { get; } = new HilActuatorControlsPayload();

        public override string Name => "HIL_ACTUATOR_CONTROLS";
        public override string ToString()
        {
            var name = "HIL_ACTUATOR_CONTROLS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  HIL_ACTUATOR_CONTROLS
    /// </summary>
    public class HilActuatorControlsPayload : IPayload
    {
        public byte ByteSize => 81;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Flags = BitConverter.ToUInt64(buffer,index);index+=8;
            for(var i=0;i<16;i++)
            {
                Controls[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            Mode = (MavMode)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Flags).CopyTo(buffer, index);index+=8;
            for(var i=0;i<16;i++)
            {
                BitConverter.GetBytes(Controls[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes((byte)Mode).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Flags as bitfield, reserved for future use.
        /// 
        /// flags
        /// </summary>
        public ulong Flags { get; set; }
        /// <summary>
        /// Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
        /// 
        /// controls
        /// </summary>
        public float[] Controls { get; } = new float[16];
        /// <summary>
        /// System mode. Includes arming state.
        /// 
        /// mode
        /// </summary>
        public MavMode Mode { get; set; }
    }
    /// <summary>
    /// Optical flow from a flow sensor (e.g. optical mouse sensor)
    ///  OPTICAL_FLOW
    /// </summary>
    public class OpticalFlowPacket: PacketV2<OpticalFlowPayload>
    {
        public override byte CrcEtra => 175;
        public override int MessageId => 100;
        public override OpticalFlowPayload Payload { get; } = new OpticalFlowPayload();

        public override string Name => "OPTICAL_FLOW";
        public override string ToString()
        {
            var name = "OPTICAL_FLOW".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  OPTICAL_FLOW
    /// </summary>
    public class OpticalFlowPayload : IPayload
    {
        public byte ByteSize => 26;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            FlowCompMX = BitConverter.ToSingle(buffer, index);index+=4;
            FlowCompMY = BitConverter.ToSingle(buffer, index);index+=4;
            GroundDistance = BitConverter.ToSingle(buffer, index);index+=4;
            FlowX = BitConverter.ToInt16(buffer,index);index+=2;
            FlowY = BitConverter.ToInt16(buffer,index);index+=2;
            SensorId = (byte)buffer[index++];
            Quality = (byte)buffer[index++];
            FlowRateX = BitConverter.ToSingle(buffer, index);index+=4;
            FlowRateY = BitConverter.ToSingle(buffer, index);index+=4;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Flow in x-sensor direction, angular-speed compensated
        /// m
        /// flow_comp_m_x
        /// </summary>
        public float FlowCompMX { get; set; }
        /// <summary>
        /// Flow in y-sensor direction, angular-speed compensated
        /// m
        /// flow_comp_m_y
        /// </summary>
        public float FlowCompMY { get; set; }
        /// <summary>
        /// Ground distance. Positive value: distance known. Negative value: Unknown distance
        /// m
        /// ground_distance
        /// </summary>
        public float GroundDistance { get; set; }
        /// <summary>
        /// Flow in x-sensor direction
        /// dpix
        /// flow_x
        /// </summary>
        public short FlowX { get; set; }
        /// <summary>
        /// Flow in y-sensor direction
        /// dpix
        /// flow_y
        /// </summary>
        public short FlowY { get; set; }
        /// <summary>
        /// Sensor ID
        /// 
        /// sensor_id
        /// </summary>
        public byte SensorId { get; set; }
        /// <summary>
        /// Optical flow quality / confidence. 0: bad, 255: maximum quality
        /// 
        /// quality
        /// </summary>
        public byte Quality { get; set; }
        /// <summary>
        /// Flow rate about X axis
        /// rad/s
        /// flow_rate_x
        /// </summary>
        public float FlowRateX { get; set; }
        /// <summary>
        /// Flow rate about Y axis
        /// rad/s
        /// flow_rate_y
        /// </summary>
        public float FlowRateY { get; set; }
    }
    /// <summary>
    ///  GLOBAL_VISION_POSITION_ESTIMATE
    /// </summary>
    public class GlobalVisionPositionEstimatePacket: PacketV2<GlobalVisionPositionEstimatePayload>
    {
        public override byte CrcEtra => 102;
        public override int MessageId => 101;
        public override GlobalVisionPositionEstimatePayload Payload { get; } = new GlobalVisionPositionEstimatePayload();

        public override string Name => "GLOBAL_VISION_POSITION_ESTIMATE";
        public override string ToString()
        {
            var name = "GLOBAL_VISION_POSITION_ESTIMATE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  GLOBAL_VISION_POSITION_ESTIMATE
    /// </summary>
    public class GlobalVisionPositionEstimatePayload : IPayload
    {
        public byte ByteSize => 32;

        public int Deserialize(byte[] buffer, int index)
        {
            Usec = BitConverter.ToUInt64(buffer,index);index+=8;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            for(var i=0;i<21;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX time or since system boot)
        /// us
        /// usec
        /// </summary>
        public ulong Usec { get; set; }
        /// <summary>
        /// Global X position
        /// m
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Global Y position
        /// m
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Global Z position
        /// m
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Roll angle
        /// rad
        /// roll
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Pitch angle
        /// rad
        /// pitch
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Yaw angle
        /// rad
        /// yaw
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// Pose covariance matrix upper right triangular (first six entries are the first ROW, next five entries are the second ROW, etc.)
        /// 
        /// covariance
        /// </summary>
        public float[] Covariance { get; } = new float[21];
    }
    /// <summary>
    ///  VISION_POSITION_ESTIMATE
    /// </summary>
    public class VisionPositionEstimatePacket: PacketV2<VisionPositionEstimatePayload>
    {
        public override byte CrcEtra => 158;
        public override int MessageId => 102;
        public override VisionPositionEstimatePayload Payload { get; } = new VisionPositionEstimatePayload();

        public override string Name => "VISION_POSITION_ESTIMATE";
        public override string ToString()
        {
            var name = "VISION_POSITION_ESTIMATE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  VISION_POSITION_ESTIMATE
    /// </summary>
    public class VisionPositionEstimatePayload : IPayload
    {
        public byte ByteSize => 32;

        public int Deserialize(byte[] buffer, int index)
        {
            Usec = BitConverter.ToUInt64(buffer,index);index+=8;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            for(var i=0;i<21;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX time or time since system boot)
        /// us
        /// usec
        /// </summary>
        public ulong Usec { get; set; }
        /// <summary>
        /// Global X position
        /// m
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Global Y position
        /// m
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Global Z position
        /// m
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Roll angle
        /// rad
        /// roll
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Pitch angle
        /// rad
        /// pitch
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Yaw angle
        /// rad
        /// yaw
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// Pose covariance matrix upper right triangular (first six entries are the first ROW, next five entries are the second ROW, etc.)
        /// 
        /// covariance
        /// </summary>
        public float[] Covariance { get; } = new float[21];
    }
    /// <summary>
    ///  VISION_SPEED_ESTIMATE
    /// </summary>
    public class VisionSpeedEstimatePacket: PacketV2<VisionSpeedEstimatePayload>
    {
        public override byte CrcEtra => 208;
        public override int MessageId => 103;
        public override VisionSpeedEstimatePayload Payload { get; } = new VisionSpeedEstimatePayload();

        public override string Name => "VISION_SPEED_ESTIMATE";
        public override string ToString()
        {
            var name = "VISION_SPEED_ESTIMATE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  VISION_SPEED_ESTIMATE
    /// </summary>
    public class VisionSpeedEstimatePayload : IPayload
    {
        public byte ByteSize => 20;

        public int Deserialize(byte[] buffer, int index)
        {
            Usec = BitConverter.ToUInt64(buffer,index);index+=8;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            for(var i=0;i<9;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX time or time since system boot)
        /// us
        /// usec
        /// </summary>
        public ulong Usec { get; set; }
        /// <summary>
        /// Global X speed
        /// m/s
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Global Y speed
        /// m/s
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Global Z speed
        /// m/s
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Linear velocity covariance matrix (1st three entries - 1st row, etc.)
        /// 
        /// covariance
        /// </summary>
        public float[] Covariance { get; } = new float[9];
    }
    /// <summary>
    ///  VICON_POSITION_ESTIMATE
    /// </summary>
    public class ViconPositionEstimatePacket: PacketV2<ViconPositionEstimatePayload>
    {
        public override byte CrcEtra => 56;
        public override int MessageId => 104;
        public override ViconPositionEstimatePayload Payload { get; } = new ViconPositionEstimatePayload();

        public override string Name => "VICON_POSITION_ESTIMATE";
        public override string ToString()
        {
            var name = "VICON_POSITION_ESTIMATE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  VICON_POSITION_ESTIMATE
    /// </summary>
    public class ViconPositionEstimatePayload : IPayload
    {
        public byte ByteSize => 32;

        public int Deserialize(byte[] buffer, int index)
        {
            Usec = BitConverter.ToUInt64(buffer,index);index+=8;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            for(var i=0;i<21;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX time or time since system boot)
        /// us
        /// usec
        /// </summary>
        public ulong Usec { get; set; }
        /// <summary>
        /// Global X position
        /// m
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Global Y position
        /// m
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Global Z position
        /// m
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Roll angle
        /// rad
        /// roll
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Pitch angle
        /// rad
        /// pitch
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Yaw angle
        /// rad
        /// yaw
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// Pose covariance matrix upper right triangular (first six entries are the first ROW, next five entries are the second ROW, etc.)
        /// 
        /// covariance
        /// </summary>
        public float[] Covariance { get; } = new float[21];
    }
    /// <summary>
    /// The IMU readings in SI units in NED body frame
    ///  HIGHRES_IMU
    /// </summary>
    public class HighresImuPacket: PacketV2<HighresImuPayload>
    {
        public override byte CrcEtra => 93;
        public override int MessageId => 105;
        public override HighresImuPayload Payload { get; } = new HighresImuPayload();

        public override string Name => "HIGHRES_IMU";
        public override string ToString()
        {
            var name = "HIGHRES_IMU".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  HIGHRES_IMU
    /// </summary>
    public class HighresImuPayload : IPayload
    {
        public byte ByteSize => 62;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X acceleration
        /// m/s/s
        /// xacc
        /// </summary>
        public float Xacc { get; set; }
        /// <summary>
        /// Y acceleration
        /// m/s/s
        /// yacc
        /// </summary>
        public float Yacc { get; set; }
        /// <summary>
        /// Z acceleration
        /// m/s/s
        /// zacc
        /// </summary>
        public float Zacc { get; set; }
        /// <summary>
        /// Angular speed around X axis
        /// rad/s
        /// xgyro
        /// </summary>
        public float Xgyro { get; set; }
        /// <summary>
        /// Angular speed around Y axis
        /// rad/s
        /// ygyro
        /// </summary>
        public float Ygyro { get; set; }
        /// <summary>
        /// Angular speed around Z axis
        /// rad/s
        /// zgyro
        /// </summary>
        public float Zgyro { get; set; }
        /// <summary>
        /// X Magnetic field
        /// gauss
        /// xmag
        /// </summary>
        public float Xmag { get; set; }
        /// <summary>
        /// Y Magnetic field
        /// gauss
        /// ymag
        /// </summary>
        public float Ymag { get; set; }
        /// <summary>
        /// Z Magnetic field
        /// gauss
        /// zmag
        /// </summary>
        public float Zmag { get; set; }
        /// <summary>
        /// Absolute pressure
        /// mbar
        /// abs_pressure
        /// </summary>
        public float AbsPressure { get; set; }
        /// <summary>
        /// Differential pressure
        /// mbar
        /// diff_pressure
        /// </summary>
        public float DiffPressure { get; set; }
        /// <summary>
        /// Altitude calculated from pressure
        /// 
        /// pressure_alt
        /// </summary>
        public float PressureAlt { get; set; }
        /// <summary>
        /// Temperature
        /// degC
        /// temperature
        /// </summary>
        public float Temperature { get; set; }
        /// <summary>
        /// Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
        /// 
        /// fields_updated
        /// </summary>
        public ushort FieldsUpdated { get; set; }
    }
    /// <summary>
    /// Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
    ///  OPTICAL_FLOW_RAD
    /// </summary>
    public class OpticalFlowRadPacket: PacketV2<OpticalFlowRadPayload>
    {
        public override byte CrcEtra => 138;
        public override int MessageId => 106;
        public override OpticalFlowRadPayload Payload { get; } = new OpticalFlowRadPayload();

        public override string Name => "OPTICAL_FLOW_RAD";
        public override string ToString()
        {
            var name = "OPTICAL_FLOW_RAD".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  OPTICAL_FLOW_RAD
    /// </summary>
    public class OpticalFlowRadPayload : IPayload
    {
        public byte ByteSize => 44;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
        /// us
        /// integration_time_us
        /// </summary>
        public uint IntegrationTimeUs { get; set; }
        /// <summary>
        /// Flow around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
        /// rad
        /// integrated_x
        /// </summary>
        public float IntegratedX { get; set; }
        /// <summary>
        /// Flow around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
        /// rad
        /// integrated_y
        /// </summary>
        public float IntegratedY { get; set; }
        /// <summary>
        /// RH rotation around X axis
        /// rad
        /// integrated_xgyro
        /// </summary>
        public float IntegratedXgyro { get; set; }
        /// <summary>
        /// RH rotation around Y axis
        /// rad
        /// integrated_ygyro
        /// </summary>
        public float IntegratedYgyro { get; set; }
        /// <summary>
        /// RH rotation around Z axis
        /// rad
        /// integrated_zgyro
        /// </summary>
        public float IntegratedZgyro { get; set; }
        /// <summary>
        /// Time since the distance was sampled.
        /// us
        /// time_delta_distance_us
        /// </summary>
        public uint TimeDeltaDistanceUs { get; set; }
        /// <summary>
        /// Distance to the center of the flow field. Positive value (including zero): distance known. Negative value: Unknown distance.
        /// m
        /// distance
        /// </summary>
        public float Distance { get; set; }
        /// <summary>
        /// Temperature
        /// cdegC
        /// temperature
        /// </summary>
        public short Temperature { get; set; }
        /// <summary>
        /// Sensor ID
        /// 
        /// sensor_id
        /// </summary>
        public byte SensorId { get; set; }
        /// <summary>
        /// Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
        /// 
        /// quality
        /// </summary>
        public byte Quality { get; set; }
    }
    /// <summary>
    /// The IMU readings in SI units in NED body frame
    ///  HIL_SENSOR
    /// </summary>
    public class HilSensorPacket: PacketV2<HilSensorPayload>
    {
        public override byte CrcEtra => 108;
        public override int MessageId => 107;
        public override HilSensorPayload Payload { get; } = new HilSensorPayload();

        public override string Name => "HIL_SENSOR";
        public override string ToString()
        {
            var name = "HIL_SENSOR".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  HIL_SENSOR
    /// </summary>
    public class HilSensorPayload : IPayload
    {
        public byte ByteSize => 64;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X acceleration
        /// m/s/s
        /// xacc
        /// </summary>
        public float Xacc { get; set; }
        /// <summary>
        /// Y acceleration
        /// m/s/s
        /// yacc
        /// </summary>
        public float Yacc { get; set; }
        /// <summary>
        /// Z acceleration
        /// m/s/s
        /// zacc
        /// </summary>
        public float Zacc { get; set; }
        /// <summary>
        /// Angular speed around X axis in body frame
        /// rad/s
        /// xgyro
        /// </summary>
        public float Xgyro { get; set; }
        /// <summary>
        /// Angular speed around Y axis in body frame
        /// rad/s
        /// ygyro
        /// </summary>
        public float Ygyro { get; set; }
        /// <summary>
        /// Angular speed around Z axis in body frame
        /// rad/s
        /// zgyro
        /// </summary>
        public float Zgyro { get; set; }
        /// <summary>
        /// X Magnetic field
        /// gauss
        /// xmag
        /// </summary>
        public float Xmag { get; set; }
        /// <summary>
        /// Y Magnetic field
        /// gauss
        /// ymag
        /// </summary>
        public float Ymag { get; set; }
        /// <summary>
        /// Z Magnetic field
        /// gauss
        /// zmag
        /// </summary>
        public float Zmag { get; set; }
        /// <summary>
        /// Absolute pressure
        /// mbar
        /// abs_pressure
        /// </summary>
        public float AbsPressure { get; set; }
        /// <summary>
        /// Differential pressure (airspeed)
        /// mbar
        /// diff_pressure
        /// </summary>
        public float DiffPressure { get; set; }
        /// <summary>
        /// Altitude calculated from pressure
        /// 
        /// pressure_alt
        /// </summary>
        public float PressureAlt { get; set; }
        /// <summary>
        /// Temperature
        /// degC
        /// temperature
        /// </summary>
        public float Temperature { get; set; }
        /// <summary>
        /// Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
        /// 
        /// fields_updated
        /// </summary>
        public uint FieldsUpdated { get; set; }
    }
    /// <summary>
    /// Status of simulation environment, if used
    ///  SIM_STATE
    /// </summary>
    public class SimStatePacket: PacketV2<SimStatePayload>
    {
        public override byte CrcEtra => 32;
        public override int MessageId => 108;
        public override SimStatePayload Payload { get; } = new SimStatePayload();

        public override string Name => "SIM_STATE";
        public override string ToString()
        {
            var name = "SIM_STATE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SIM_STATE
    /// </summary>
    public class SimStatePayload : IPayload
    {
        public byte ByteSize => 84;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// True attitude quaternion component 1, w (1 in null-rotation)
        /// 
        /// q1
        /// </summary>
        public float Q1 { get; set; }
        /// <summary>
        /// True attitude quaternion component 2, x (0 in null-rotation)
        /// 
        /// q2
        /// </summary>
        public float Q2 { get; set; }
        /// <summary>
        /// True attitude quaternion component 3, y (0 in null-rotation)
        /// 
        /// q3
        /// </summary>
        public float Q3 { get; set; }
        /// <summary>
        /// True attitude quaternion component 4, z (0 in null-rotation)
        /// 
        /// q4
        /// </summary>
        public float Q4 { get; set; }
        /// <summary>
        /// Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
        /// 
        /// roll
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
        /// 
        /// pitch
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
        /// 
        /// yaw
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// X acceleration
        /// m/s/s
        /// xacc
        /// </summary>
        public float Xacc { get; set; }
        /// <summary>
        /// Y acceleration
        /// m/s/s
        /// yacc
        /// </summary>
        public float Yacc { get; set; }
        /// <summary>
        /// Z acceleration
        /// m/s/s
        /// zacc
        /// </summary>
        public float Zacc { get; set; }
        /// <summary>
        /// Angular speed around X axis
        /// rad/s
        /// xgyro
        /// </summary>
        public float Xgyro { get; set; }
        /// <summary>
        /// Angular speed around Y axis
        /// rad/s
        /// ygyro
        /// </summary>
        public float Ygyro { get; set; }
        /// <summary>
        /// Angular speed around Z axis
        /// rad/s
        /// zgyro
        /// </summary>
        public float Zgyro { get; set; }
        /// <summary>
        /// Latitude
        /// deg
        /// lat
        /// </summary>
        public float Lat { get; set; }
        /// <summary>
        /// Longitude
        /// deg
        /// lon
        /// </summary>
        public float Lon { get; set; }
        /// <summary>
        /// Altitude
        /// m
        /// alt
        /// </summary>
        public float Alt { get; set; }
        /// <summary>
        /// Horizontal position standard deviation
        /// 
        /// std_dev_horz
        /// </summary>
        public float StdDevHorz { get; set; }
        /// <summary>
        /// Vertical position standard deviation
        /// 
        /// std_dev_vert
        /// </summary>
        public float StdDevVert { get; set; }
        /// <summary>
        /// True velocity in NORTH direction in earth-fixed NED frame
        /// m/s
        /// vn
        /// </summary>
        public float Vn { get; set; }
        /// <summary>
        /// True velocity in EAST direction in earth-fixed NED frame
        /// m/s
        /// ve
        /// </summary>
        public float Ve { get; set; }
        /// <summary>
        /// True velocity in DOWN direction in earth-fixed NED frame
        /// m/s
        /// vd
        /// </summary>
        public float Vd { get; set; }
    }
    /// <summary>
    /// Status generated by radio and injected into MAVLink stream.
    ///  RADIO_STATUS
    /// </summary>
    public class RadioStatusPacket: PacketV2<RadioStatusPayload>
    {
        public override byte CrcEtra => 185;
        public override int MessageId => 109;
        public override RadioStatusPayload Payload { get; } = new RadioStatusPayload();

        public override string Name => "RADIO_STATUS";
        public override string ToString()
        {
            var name = "RADIO_STATUS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  RADIO_STATUS
    /// </summary>
    public class RadioStatusPayload : IPayload
    {
        public byte ByteSize => 9;

        public int Deserialize(byte[] buffer, int index)
        {
            Rxerrors = BitConverter.ToUInt16(buffer,index);index+=2;
            Fixed = BitConverter.ToUInt16(buffer,index);index+=2;
            Rssi = (byte)buffer[index++];
            Remrssi = (byte)buffer[index++];
            Txbuf = (byte)buffer[index++];
            Noise = (byte)buffer[index++];
            Remnoise = (byte)buffer[index++];
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Receive errors
        /// 
        /// rxerrors
        /// </summary>
        public ushort Rxerrors { get; set; }
        /// <summary>
        /// Count of error corrected packets
        /// 
        /// fixed
        /// </summary>
        public ushort Fixed { get; set; }
        /// <summary>
        /// Local signal strength
        /// 
        /// rssi
        /// </summary>
        public byte Rssi { get; set; }
        /// <summary>
        /// Remote signal strength
        /// 
        /// remrssi
        /// </summary>
        public byte Remrssi { get; set; }
        /// <summary>
        /// Remaining free buffer space.
        /// %
        /// txbuf
        /// </summary>
        public byte Txbuf { get; set; }
        /// <summary>
        /// Background noise level
        /// 
        /// noise
        /// </summary>
        public byte Noise { get; set; }
        /// <summary>
        /// Remote background noise level
        /// 
        /// remnoise
        /// </summary>
        public byte Remnoise { get; set; }
    }
    /// <summary>
    /// File transfer message
    ///  FILE_TRANSFER_PROTOCOL
    /// </summary>
    public class FileTransferProtocolPacket: PacketV2<FileTransferProtocolPayload>
    {
        public override byte CrcEtra => 84;
        public override int MessageId => 110;
        public override FileTransferProtocolPayload Payload { get; } = new FileTransferProtocolPayload();

        public override string Name => "FILE_TRANSFER_PROTOCOL";
        public override string ToString()
        {
            var name = "FILE_TRANSFER_PROTOCOL".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  FILE_TRANSFER_PROTOCOL
    /// </summary>
    public class FileTransferProtocolPayload : IPayload
    {
        public byte ByteSize => 254;

        public int Deserialize(byte[] buffer, int index)
        {
            TargetNetwork = (byte)buffer[index++];
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            for(var i=0;i<251;i++)
            {
                Payload[i] = (byte)buffer[index++];
            }
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetNetwork).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            for(var i=0;i<251;i++)
            {
                BitConverter.GetBytes(Payload[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// Network ID (0 for broadcast)
        /// 
        /// target_network
        /// </summary>
        public byte TargetNetwork { get; set; }
        /// <summary>
        /// System ID (0 for broadcast)
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID (0 for broadcast)
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
        /// 
        /// payload
        /// </summary>
        public byte[] Payload { get; } = new byte[251];
    }
    /// <summary>
    /// Time synchronization message.
    ///  TIMESYNC
    /// </summary>
    public class TimesyncPacket: PacketV2<TimesyncPayload>
    {
        public override byte CrcEtra => 34;
        public override int MessageId => 111;
        public override TimesyncPayload Payload { get; } = new TimesyncPayload();

        public override string Name => "TIMESYNC";
        public override string ToString()
        {
            var name = "TIMESYNC".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  TIMESYNC
    /// </summary>
    public class TimesyncPayload : IPayload
    {
        public byte ByteSize => 16;

        public int Deserialize(byte[] buffer, int index)
        {
            Tc1 = BitConverter.ToInt64(buffer,index);index+=8;
            Ts1 = BitConverter.ToInt64(buffer,index);index+=8;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Tc1).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Ts1).CopyTo(buffer, index);index+=8;
            return ByteSize;
        }

        /// <summary>
        /// Time sync timestamp 1
        /// 
        /// tc1
        /// </summary>
        public long Tc1 { get; set; }
        /// <summary>
        /// Time sync timestamp 2
        /// 
        /// ts1
        /// </summary>
        public long Ts1 { get; set; }
    }
    /// <summary>
    /// Camera-IMU triggering and synchronisation message.
    ///  CAMERA_TRIGGER
    /// </summary>
    public class CameraTriggerPacket: PacketV2<CameraTriggerPayload>
    {
        public override byte CrcEtra => 174;
        public override int MessageId => 112;
        public override CameraTriggerPayload Payload { get; } = new CameraTriggerPayload();

        public override string Name => "CAMERA_TRIGGER";
        public override string ToString()
        {
            var name = "CAMERA_TRIGGER".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  CAMERA_TRIGGER
    /// </summary>
    public class CameraTriggerPayload : IPayload
    {
        public byte ByteSize => 12;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            Seq = BitConverter.ToUInt32(buffer,index);index+=4;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Seq).CopyTo(buffer, index);index+=4;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp for image frame (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Image frame sequence
        /// 
        /// seq
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
        public override byte CrcEtra => 124;
        public override int MessageId => 113;
        public override HilGpsPayload Payload { get; } = new HilGpsPayload();

        public override string Name => "HIL_GPS";
        public override string ToString()
        {
            var name = "HIL_GPS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  HIL_GPS
    /// </summary>
    public class HilGpsPayload : IPayload
    {
        public byte ByteSize => 36;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Latitude (WGS84)
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude (WGS84)
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude (AMSL). Positive for up.
        /// mm
        /// alt
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// GPS HDOP horizontal dilution of position. If unknown, set to: 65535
        /// cm
        /// eph
        /// </summary>
        public ushort Eph { get; set; }
        /// <summary>
        /// GPS VDOP vertical dilution of position. If unknown, set to: 65535
        /// cm
        /// epv
        /// </summary>
        public ushort Epv { get; set; }
        /// <summary>
        /// GPS ground speed. If unknown, set to: 65535
        /// cm/s
        /// vel
        /// </summary>
        public ushort Vel { get; set; }
        /// <summary>
        /// GPS velocity in NORTH direction in earth-fixed NED frame
        /// cm/s
        /// vn
        /// </summary>
        public short Vn { get; set; }
        /// <summary>
        /// GPS velocity in EAST direction in earth-fixed NED frame
        /// cm/s
        /// ve
        /// </summary>
        public short Ve { get; set; }
        /// <summary>
        /// GPS velocity in DOWN direction in earth-fixed NED frame
        /// cm/s
        /// vd
        /// </summary>
        public short Vd { get; set; }
        /// <summary>
        /// Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: 65535
        /// cdeg
        /// cog
        /// </summary>
        public ushort Cog { get; set; }
        /// <summary>
        /// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
        /// 
        /// fix_type
        /// </summary>
        public byte FixType { get; set; }
        /// <summary>
        /// Number of satellites visible. If unknown, set to 255
        /// 
        /// satellites_visible
        /// </summary>
        public byte SatellitesVisible { get; set; }
    }
    /// <summary>
    /// Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
    ///  HIL_OPTICAL_FLOW
    /// </summary>
    public class HilOpticalFlowPacket: PacketV2<HilOpticalFlowPayload>
    {
        public override byte CrcEtra => 237;
        public override int MessageId => 114;
        public override HilOpticalFlowPayload Payload { get; } = new HilOpticalFlowPayload();

        public override string Name => "HIL_OPTICAL_FLOW";
        public override string ToString()
        {
            var name = "HIL_OPTICAL_FLOW".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  HIL_OPTICAL_FLOW
    /// </summary>
    public class HilOpticalFlowPayload : IPayload
    {
        public byte ByteSize => 44;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
        /// us
        /// integration_time_us
        /// </summary>
        public uint IntegrationTimeUs { get; set; }
        /// <summary>
        /// Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
        /// rad
        /// integrated_x
        /// </summary>
        public float IntegratedX { get; set; }
        /// <summary>
        /// Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
        /// rad
        /// integrated_y
        /// </summary>
        public float IntegratedY { get; set; }
        /// <summary>
        /// RH rotation around X axis
        /// rad
        /// integrated_xgyro
        /// </summary>
        public float IntegratedXgyro { get; set; }
        /// <summary>
        /// RH rotation around Y axis
        /// rad
        /// integrated_ygyro
        /// </summary>
        public float IntegratedYgyro { get; set; }
        /// <summary>
        /// RH rotation around Z axis
        /// rad
        /// integrated_zgyro
        /// </summary>
        public float IntegratedZgyro { get; set; }
        /// <summary>
        /// Time since the distance was sampled.
        /// us
        /// time_delta_distance_us
        /// </summary>
        public uint TimeDeltaDistanceUs { get; set; }
        /// <summary>
        /// Distance to the center of the flow field. Positive value (including zero): distance known. Negative value: Unknown distance.
        /// m
        /// distance
        /// </summary>
        public float Distance { get; set; }
        /// <summary>
        /// Temperature
        /// cdegC
        /// temperature
        /// </summary>
        public short Temperature { get; set; }
        /// <summary>
        /// Sensor ID
        /// 
        /// sensor_id
        /// </summary>
        public byte SensorId { get; set; }
        /// <summary>
        /// Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
        /// 
        /// quality
        /// </summary>
        public byte Quality { get; set; }
    }
    /// <summary>
    /// Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.
    ///  HIL_STATE_QUATERNION
    /// </summary>
    public class HilStateQuaternionPacket: PacketV2<HilStateQuaternionPayload>
    {
        public override byte CrcEtra => 4;
        public override int MessageId => 115;
        public override HilStateQuaternionPayload Payload { get; } = new HilStateQuaternionPayload();

        public override string Name => "HIL_STATE_QUATERNION";
        public override string ToString()
        {
            var name = "HIL_STATE_QUATERNION".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  HIL_STATE_QUATERNION
    /// </summary>
    public class HilStateQuaternionPayload : IPayload
    {
        public byte ByteSize => 64;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            for(var i=0;i<4;i++)
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
        /// 
        /// attitude_quaternion
        /// </summary>
        public float[] AttitudeQuaternion { get; } = new float[4];
        /// <summary>
        /// Body frame roll / phi angular speed
        /// rad/s
        /// rollspeed
        /// </summary>
        public float Rollspeed { get; set; }
        /// <summary>
        /// Body frame pitch / theta angular speed
        /// rad/s
        /// pitchspeed
        /// </summary>
        public float Pitchspeed { get; set; }
        /// <summary>
        /// Body frame yaw / psi angular speed
        /// rad/s
        /// yawspeed
        /// </summary>
        public float Yawspeed { get; set; }
        /// <summary>
        /// Latitude
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude
        /// mm
        /// alt
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// Ground X Speed (Latitude)
        /// cm/s
        /// vx
        /// </summary>
        public short Vx { get; set; }
        /// <summary>
        /// Ground Y Speed (Longitude)
        /// cm/s
        /// vy
        /// </summary>
        public short Vy { get; set; }
        /// <summary>
        /// Ground Z Speed (Altitude)
        /// cm/s
        /// vz
        /// </summary>
        public short Vz { get; set; }
        /// <summary>
        /// Indicated airspeed
        /// cm/s
        /// ind_airspeed
        /// </summary>
        public ushort IndAirspeed { get; set; }
        /// <summary>
        /// True airspeed
        /// cm/s
        /// true_airspeed
        /// </summary>
        public ushort TrueAirspeed { get; set; }
        /// <summary>
        /// X acceleration
        /// mG
        /// xacc
        /// </summary>
        public short Xacc { get; set; }
        /// <summary>
        /// Y acceleration
        /// mG
        /// yacc
        /// </summary>
        public short Yacc { get; set; }
        /// <summary>
        /// Z acceleration
        /// mG
        /// zacc
        /// </summary>
        public short Zacc { get; set; }
    }
    /// <summary>
    /// The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units
    ///  SCALED_IMU2
    /// </summary>
    public class ScaledImu2Packet: PacketV2<ScaledImu2Payload>
    {
        public override byte CrcEtra => 76;
        public override int MessageId => 116;
        public override ScaledImu2Payload Payload { get; } = new ScaledImu2Payload();

        public override string Name => "SCALED_IMU2";
        public override string ToString()
        {
            var name = "SCALED_IMU2".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SCALED_IMU2
    /// </summary>
    public class ScaledImu2Payload : IPayload
    {
        public byte ByteSize => 22;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X acceleration
        /// mG
        /// xacc
        /// </summary>
        public short Xacc { get; set; }
        /// <summary>
        /// Y acceleration
        /// mG
        /// yacc
        /// </summary>
        public short Yacc { get; set; }
        /// <summary>
        /// Z acceleration
        /// mG
        /// zacc
        /// </summary>
        public short Zacc { get; set; }
        /// <summary>
        /// Angular speed around X axis
        /// mrad/s
        /// xgyro
        /// </summary>
        public short Xgyro { get; set; }
        /// <summary>
        /// Angular speed around Y axis
        /// mrad/s
        /// ygyro
        /// </summary>
        public short Ygyro { get; set; }
        /// <summary>
        /// Angular speed around Z axis
        /// mrad/s
        /// zgyro
        /// </summary>
        public short Zgyro { get; set; }
        /// <summary>
        /// X Magnetic field
        /// mT
        /// xmag
        /// </summary>
        public short Xmag { get; set; }
        /// <summary>
        /// Y Magnetic field
        /// mT
        /// ymag
        /// </summary>
        public short Ymag { get; set; }
        /// <summary>
        /// Z Magnetic field
        /// mT
        /// zmag
        /// </summary>
        public short Zmag { get; set; }
    }
    /// <summary>
    /// Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END is called.
    ///  LOG_REQUEST_LIST
    /// </summary>
    public class LogRequestListPacket: PacketV2<LogRequestListPayload>
    {
        public override byte CrcEtra => 128;
        public override int MessageId => 117;
        public override LogRequestListPayload Payload { get; } = new LogRequestListPayload();

        public override string Name => "LOG_REQUEST_LIST";
        public override string ToString()
        {
            var name = "LOG_REQUEST_LIST".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  LOG_REQUEST_LIST
    /// </summary>
    public class LogRequestListPayload : IPayload
    {
        public byte ByteSize => 6;

        public int Deserialize(byte[] buffer, int index)
        {
            Start = BitConverter.ToUInt16(buffer,index);index+=2;
            End = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Start).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(End).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// First log id (0 for first available)
        /// 
        /// start
        /// </summary>
        public ushort Start { get; set; }
        /// <summary>
        /// Last log id (0xffff for last available)
        /// 
        /// end
        /// </summary>
        public ushort End { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Reply to LOG_REQUEST_LIST
    ///  LOG_ENTRY
    /// </summary>
    public class LogEntryPacket: PacketV2<LogEntryPayload>
    {
        public override byte CrcEtra => 56;
        public override int MessageId => 118;
        public override LogEntryPayload Payload { get; } = new LogEntryPayload();

        public override string Name => "LOG_ENTRY";
        public override string ToString()
        {
            var name = "LOG_ENTRY".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  LOG_ENTRY
    /// </summary>
    public class LogEntryPayload : IPayload
    {
        public byte ByteSize => 14;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUtc = BitConverter.ToUInt32(buffer,index);index+=4;
            Size = BitConverter.ToUInt32(buffer,index);index+=4;
            Id = BitConverter.ToUInt16(buffer,index);index+=2;
            NumLogs = BitConverter.ToUInt16(buffer,index);index+=2;
            LastLogNum = BitConverter.ToUInt16(buffer,index);index+=2;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUtc).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Size).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Id).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(NumLogs).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(LastLogNum).CopyTo(buffer, index);index+=2;
            return ByteSize;
        }

        /// <summary>
        /// UTC timestamp of log since 1970, or 0 if not available
        /// s
        /// time_utc
        /// </summary>
        public uint TimeUtc { get; set; }
        /// <summary>
        /// Size of the log (may be approximate)
        /// bytes
        /// size
        /// </summary>
        public uint Size { get; set; }
        /// <summary>
        /// Log id
        /// 
        /// id
        /// </summary>
        public ushort Id { get; set; }
        /// <summary>
        /// Total number of logs
        /// 
        /// num_logs
        /// </summary>
        public ushort NumLogs { get; set; }
        /// <summary>
        /// High log number
        /// 
        /// last_log_num
        /// </summary>
        public ushort LastLogNum { get; set; }
    }
    /// <summary>
    /// Request a chunk of a log
    ///  LOG_REQUEST_DATA
    /// </summary>
    public class LogRequestDataPacket: PacketV2<LogRequestDataPayload>
    {
        public override byte CrcEtra => 116;
        public override int MessageId => 119;
        public override LogRequestDataPayload Payload { get; } = new LogRequestDataPayload();

        public override string Name => "LOG_REQUEST_DATA";
        public override string ToString()
        {
            var name = "LOG_REQUEST_DATA".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  LOG_REQUEST_DATA
    /// </summary>
    public class LogRequestDataPayload : IPayload
    {
        public byte ByteSize => 12;

        public int Deserialize(byte[] buffer, int index)
        {
            Ofs = BitConverter.ToUInt32(buffer,index);index+=4;
            Count = BitConverter.ToUInt32(buffer,index);index+=4;
            Id = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Ofs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Count).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Id).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Offset into the log
        /// 
        /// ofs
        /// </summary>
        public uint Ofs { get; set; }
        /// <summary>
        /// Number of bytes
        /// bytes
        /// count
        /// </summary>
        public uint Count { get; set; }
        /// <summary>
        /// Log id (from LOG_ENTRY reply)
        /// 
        /// id
        /// </summary>
        public ushort Id { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Reply to LOG_REQUEST_DATA
    ///  LOG_DATA
    /// </summary>
    public class LogDataPacket: PacketV2<LogDataPayload>
    {
        public override byte CrcEtra => 134;
        public override int MessageId => 120;
        public override LogDataPayload Payload { get; } = new LogDataPayload();

        public override string Name => "LOG_DATA";
        public override string ToString()
        {
            var name = "LOG_DATA".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  LOG_DATA
    /// </summary>
    public class LogDataPayload : IPayload
    {
        public byte ByteSize => 97;

        public int Deserialize(byte[] buffer, int index)
        {
            Ofs = BitConverter.ToUInt32(buffer,index);index+=4;
            Id = BitConverter.ToUInt16(buffer,index);index+=2;
            Count = (byte)buffer[index++];
            for(var i=0;i<90;i++)
            {
                Data[i] = (byte)buffer[index++];
            }
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Ofs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Id).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Count).CopyTo(buffer, index);index+=1;
            for(var i=0;i<90;i++)
            {
                BitConverter.GetBytes(Data[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// Offset into the log
        /// 
        /// ofs
        /// </summary>
        public uint Ofs { get; set; }
        /// <summary>
        /// Log id (from LOG_ENTRY reply)
        /// 
        /// id
        /// </summary>
        public ushort Id { get; set; }
        /// <summary>
        /// Number of bytes (zero for end of log)
        /// bytes
        /// count
        /// </summary>
        public byte Count { get; set; }
        /// <summary>
        /// log data
        /// 
        /// data
        /// </summary>
        public byte[] Data { get; } = new byte[90];
    }
    /// <summary>
    /// Erase all logs
    ///  LOG_ERASE
    /// </summary>
    public class LogErasePacket: PacketV2<LogErasePayload>
    {
        public override byte CrcEtra => 237;
        public override int MessageId => 121;
        public override LogErasePayload Payload { get; } = new LogErasePayload();

        public override string Name => "LOG_ERASE";
        public override string ToString()
        {
            var name = "LOG_ERASE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  LOG_ERASE
    /// </summary>
    public class LogErasePayload : IPayload
    {
        public byte ByteSize => 2;

        public int Deserialize(byte[] buffer, int index)
        {
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Stop log transfer and resume normal logging
    ///  LOG_REQUEST_END
    /// </summary>
    public class LogRequestEndPacket: PacketV2<LogRequestEndPayload>
    {
        public override byte CrcEtra => 203;
        public override int MessageId => 122;
        public override LogRequestEndPayload Payload { get; } = new LogRequestEndPayload();

        public override string Name => "LOG_REQUEST_END";
        public override string ToString()
        {
            var name = "LOG_REQUEST_END".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  LOG_REQUEST_END
    /// </summary>
    public class LogRequestEndPayload : IPayload
    {
        public byte ByteSize => 2;

        public int Deserialize(byte[] buffer, int index)
        {
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// data for injecting into the onboard GPS (used for DGPS)
    ///  GPS_INJECT_DATA
    /// </summary>
    public class GpsInjectDataPacket: PacketV2<GpsInjectDataPayload>
    {
        public override byte CrcEtra => 250;
        public override int MessageId => 123;
        public override GpsInjectDataPayload Payload { get; } = new GpsInjectDataPayload();

        public override string Name => "GPS_INJECT_DATA";
        public override string ToString()
        {
            var name = "GPS_INJECT_DATA".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  GPS_INJECT_DATA
    /// </summary>
    public class GpsInjectDataPayload : IPayload
    {
        public byte ByteSize => 113;

        public int Deserialize(byte[] buffer, int index)
        {
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            Len = (byte)buffer[index++];
            for(var i=0;i<110;i++)
            {
                Data[i] = (byte)buffer[index++];
            }
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Len).CopyTo(buffer, index);index+=1;
            for(var i=0;i<110;i++)
            {
                BitConverter.GetBytes(Data[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// data length
        /// bytes
        /// len
        /// </summary>
        public byte Len { get; set; }
        /// <summary>
        /// raw data (110 is enough for 12 satellites of RTCMv2)
        /// 
        /// data
        /// </summary>
        public byte[] Data { get; } = new byte[110];
    }
    /// <summary>
    /// Second GPS data.
    ///  GPS2_RAW
    /// </summary>
    public class Gps2RawPacket: PacketV2<Gps2RawPayload>
    {
        public override byte CrcEtra => 87;
        public override int MessageId => 124;
        public override Gps2RawPayload Payload { get; } = new Gps2RawPayload();

        public override string Name => "GPS2_RAW";
        public override string ToString()
        {
            var name = "GPS2_RAW".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  GPS2_RAW
    /// </summary>
    public class Gps2RawPayload : IPayload
    {
        public byte ByteSize => 35;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            BitConverter.GetBytes((byte)FixType).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(SatellitesVisible).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(DgpsNumch).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Latitude (WGS84)
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude (WGS84)
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude (AMSL). Positive for up.
        /// mm
        /// alt
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// Age of DGPS info
        /// ms
        /// dgps_age
        /// </summary>
        public uint DgpsAge { get; set; }
        /// <summary>
        /// GPS HDOP horizontal dilution of position. If unknown, set to: UINT16_MAX
        /// cm
        /// eph
        /// </summary>
        public ushort Eph { get; set; }
        /// <summary>
        /// GPS VDOP vertical dilution of position. If unknown, set to: UINT16_MAX
        /// cm
        /// epv
        /// </summary>
        public ushort Epv { get; set; }
        /// <summary>
        /// GPS ground speed. If unknown, set to: UINT16_MAX
        /// cm/s
        /// vel
        /// </summary>
        public ushort Vel { get; set; }
        /// <summary>
        /// Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        /// cdeg
        /// cog
        /// </summary>
        public ushort Cog { get; set; }
        /// <summary>
        /// GPS fix type.
        /// 
        /// fix_type
        /// </summary>
        public GpsFixType FixType { get; set; }
        /// <summary>
        /// Number of satellites visible. If unknown, set to 255
        /// 
        /// satellites_visible
        /// </summary>
        public byte SatellitesVisible { get; set; }
        /// <summary>
        /// Number of DGPS satellites
        /// 
        /// dgps_numch
        /// </summary>
        public byte DgpsNumch { get; set; }
    }
    /// <summary>
    /// Power supply status
    ///  POWER_STATUS
    /// </summary>
    public class PowerStatusPacket: PacketV2<PowerStatusPayload>
    {
        public override byte CrcEtra => 203;
        public override int MessageId => 125;
        public override PowerStatusPayload Payload { get; } = new PowerStatusPayload();

        public override string Name => "POWER_STATUS";
        public override string ToString()
        {
            var name = "POWER_STATUS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  POWER_STATUS
    /// </summary>
    public class PowerStatusPayload : IPayload
    {
        public byte ByteSize => 6;

        public int Deserialize(byte[] buffer, int index)
        {
            Vcc = BitConverter.ToUInt16(buffer,index);index+=2;
            Vservo = BitConverter.ToUInt16(buffer,index);index+=2;
            Flags = (MavPowerStatus)BitConverter.ToUInt16(buffer,index);index+=2;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Vcc).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Vservo).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes((ushort)Flags).CopyTo(buffer, index);index+=2;
            return ByteSize;
        }

        /// <summary>
        /// 5V rail voltage.
        /// mV
        /// Vcc
        /// </summary>
        public ushort Vcc { get; set; }
        /// <summary>
        /// Servo rail voltage.
        /// mV
        /// Vservo
        /// </summary>
        public ushort Vservo { get; set; }
        /// <summary>
        /// Bitmap of power supply status flags.
        /// 
        /// flags
        /// </summary>
        public MavPowerStatus Flags { get; set; }
    }
    /// <summary>
    /// Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate.
    ///  SERIAL_CONTROL
    /// </summary>
    public class SerialControlPacket: PacketV2<SerialControlPayload>
    {
        public override byte CrcEtra => 220;
        public override int MessageId => 126;
        public override SerialControlPayload Payload { get; } = new SerialControlPayload();

        public override string Name => "SERIAL_CONTROL";
        public override string ToString()
        {
            var name = "SERIAL_CONTROL".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SERIAL_CONTROL
    /// </summary>
    public class SerialControlPayload : IPayload
    {
        public byte ByteSize => 79;

        public int Deserialize(byte[] buffer, int index)
        {
            Baudrate = BitConverter.ToUInt32(buffer,index);index+=4;
            Timeout = BitConverter.ToUInt16(buffer,index);index+=2;
            Device = (SerialControlDev)buffer[index++];
            Flags = (SerialControlFlag)buffer[index++];
            Count = (byte)buffer[index++];
            for(var i=0;i<70;i++)
            {
                Data[i] = (byte)buffer[index++];
            }
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Baudrate).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Timeout).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes((byte)Device).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)Flags).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Count).CopyTo(buffer, index);index+=1;
            for(var i=0;i<70;i++)
            {
                BitConverter.GetBytes(Data[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// Baudrate of transfer. Zero means no change.
        /// bits/s
        /// baudrate
        /// </summary>
        public uint Baudrate { get; set; }
        /// <summary>
        /// Timeout for reply data
        /// ms
        /// timeout
        /// </summary>
        public ushort Timeout { get; set; }
        /// <summary>
        /// Serial control device type.
        /// 
        /// device
        /// </summary>
        public SerialControlDev Device { get; set; }
        /// <summary>
        /// Bitmap of serial control flags.
        /// 
        /// flags
        /// </summary>
        public SerialControlFlag Flags { get; set; }
        /// <summary>
        /// how many bytes in this transfer
        /// bytes
        /// count
        /// </summary>
        public byte Count { get; set; }
        /// <summary>
        /// serial data
        /// 
        /// data
        /// </summary>
        public byte[] Data { get; } = new byte[70];
    }
    /// <summary>
    /// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
    ///  GPS_RTK
    /// </summary>
    public class GpsRtkPacket: PacketV2<GpsRtkPayload>
    {
        public override byte CrcEtra => 25;
        public override int MessageId => 127;
        public override GpsRtkPayload Payload { get; } = new GpsRtkPayload();

        public override string Name => "GPS_RTK";
        public override string ToString()
        {
            var name = "GPS_RTK".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  GPS_RTK
    /// </summary>
    public class GpsRtkPayload : IPayload
    {
        public byte ByteSize => 35;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            BitConverter.GetBytes((byte)BaselineCoordsType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Time since boot of last baseline message received.
        /// ms
        /// time_last_baseline_ms
        /// </summary>
        public uint TimeLastBaselineMs { get; set; }
        /// <summary>
        /// GPS Time of Week of last baseline
        /// ms
        /// tow
        /// </summary>
        public uint Tow { get; set; }
        /// <summary>
        /// Current baseline in ECEF x or NED north component.
        /// mm
        /// baseline_a_mm
        /// </summary>
        public int BaselineAMm { get; set; }
        /// <summary>
        /// Current baseline in ECEF y or NED east component.
        /// mm
        /// baseline_b_mm
        /// </summary>
        public int BaselineBMm { get; set; }
        /// <summary>
        /// Current baseline in ECEF z or NED down component.
        /// mm
        /// baseline_c_mm
        /// </summary>
        public int BaselineCMm { get; set; }
        /// <summary>
        /// Current estimate of baseline accuracy.
        /// 
        /// accuracy
        /// </summary>
        public uint Accuracy { get; set; }
        /// <summary>
        /// Current number of integer ambiguity hypotheses.
        /// 
        /// iar_num_hypotheses
        /// </summary>
        public int IarNumHypotheses { get; set; }
        /// <summary>
        /// GPS Week Number of last baseline
        /// 
        /// wn
        /// </summary>
        public ushort Wn { get; set; }
        /// <summary>
        /// Identification of connected RTK receiver.
        /// 
        /// rtk_receiver_id
        /// </summary>
        public byte RtkReceiverId { get; set; }
        /// <summary>
        /// GPS-specific health report for RTK data.
        /// 
        /// rtk_health
        /// </summary>
        public byte RtkHealth { get; set; }
        /// <summary>
        /// Rate of baseline messages being received by GPS
        /// Hz
        /// rtk_rate
        /// </summary>
        public byte RtkRate { get; set; }
        /// <summary>
        /// Current number of sats used for RTK calculation.
        /// 
        /// nsats
        /// </summary>
        public byte Nsats { get; set; }
        /// <summary>
        /// Coordinate system of baseline
        /// 
        /// baseline_coords_type
        /// </summary>
        public RtkBaselineCoordinateSystem BaselineCoordsType { get; set; }
    }
    /// <summary>
    /// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
    ///  GPS2_RTK
    /// </summary>
    public class Gps2RtkPacket: PacketV2<Gps2RtkPayload>
    {
        public override byte CrcEtra => 226;
        public override int MessageId => 128;
        public override Gps2RtkPayload Payload { get; } = new Gps2RtkPayload();

        public override string Name => "GPS2_RTK";
        public override string ToString()
        {
            var name = "GPS2_RTK".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  GPS2_RTK
    /// </summary>
    public class Gps2RtkPayload : IPayload
    {
        public byte ByteSize => 35;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            BitConverter.GetBytes((byte)BaselineCoordsType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Time since boot of last baseline message received.
        /// ms
        /// time_last_baseline_ms
        /// </summary>
        public uint TimeLastBaselineMs { get; set; }
        /// <summary>
        /// GPS Time of Week of last baseline
        /// ms
        /// tow
        /// </summary>
        public uint Tow { get; set; }
        /// <summary>
        /// Current baseline in ECEF x or NED north component.
        /// mm
        /// baseline_a_mm
        /// </summary>
        public int BaselineAMm { get; set; }
        /// <summary>
        /// Current baseline in ECEF y or NED east component.
        /// mm
        /// baseline_b_mm
        /// </summary>
        public int BaselineBMm { get; set; }
        /// <summary>
        /// Current baseline in ECEF z or NED down component.
        /// mm
        /// baseline_c_mm
        /// </summary>
        public int BaselineCMm { get; set; }
        /// <summary>
        /// Current estimate of baseline accuracy.
        /// 
        /// accuracy
        /// </summary>
        public uint Accuracy { get; set; }
        /// <summary>
        /// Current number of integer ambiguity hypotheses.
        /// 
        /// iar_num_hypotheses
        /// </summary>
        public int IarNumHypotheses { get; set; }
        /// <summary>
        /// GPS Week Number of last baseline
        /// 
        /// wn
        /// </summary>
        public ushort Wn { get; set; }
        /// <summary>
        /// Identification of connected RTK receiver.
        /// 
        /// rtk_receiver_id
        /// </summary>
        public byte RtkReceiverId { get; set; }
        /// <summary>
        /// GPS-specific health report for RTK data.
        /// 
        /// rtk_health
        /// </summary>
        public byte RtkHealth { get; set; }
        /// <summary>
        /// Rate of baseline messages being received by GPS
        /// Hz
        /// rtk_rate
        /// </summary>
        public byte RtkRate { get; set; }
        /// <summary>
        /// Current number of sats used for RTK calculation.
        /// 
        /// nsats
        /// </summary>
        public byte Nsats { get; set; }
        /// <summary>
        /// Coordinate system of baseline
        /// 
        /// baseline_coords_type
        /// </summary>
        public RtkBaselineCoordinateSystem BaselineCoordsType { get; set; }
    }
    /// <summary>
    /// The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described units
    ///  SCALED_IMU3
    /// </summary>
    public class ScaledImu3Packet: PacketV2<ScaledImu3Payload>
    {
        public override byte CrcEtra => 46;
        public override int MessageId => 129;
        public override ScaledImu3Payload Payload { get; } = new ScaledImu3Payload();

        public override string Name => "SCALED_IMU3";
        public override string ToString()
        {
            var name = "SCALED_IMU3".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SCALED_IMU3
    /// </summary>
    public class ScaledImu3Payload : IPayload
    {
        public byte ByteSize => 22;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// X acceleration
        /// mG
        /// xacc
        /// </summary>
        public short Xacc { get; set; }
        /// <summary>
        /// Y acceleration
        /// mG
        /// yacc
        /// </summary>
        public short Yacc { get; set; }
        /// <summary>
        /// Z acceleration
        /// mG
        /// zacc
        /// </summary>
        public short Zacc { get; set; }
        /// <summary>
        /// Angular speed around X axis
        /// mrad/s
        /// xgyro
        /// </summary>
        public short Xgyro { get; set; }
        /// <summary>
        /// Angular speed around Y axis
        /// mrad/s
        /// ygyro
        /// </summary>
        public short Ygyro { get; set; }
        /// <summary>
        /// Angular speed around Z axis
        /// mrad/s
        /// zgyro
        /// </summary>
        public short Zgyro { get; set; }
        /// <summary>
        /// X Magnetic field
        /// mT
        /// xmag
        /// </summary>
        public short Xmag { get; set; }
        /// <summary>
        /// Y Magnetic field
        /// mT
        /// ymag
        /// </summary>
        public short Ymag { get; set; }
        /// <summary>
        /// Z Magnetic field
        /// mT
        /// zmag
        /// </summary>
        public short Zmag { get; set; }
    }
    /// <summary>
    ///  DATA_TRANSMISSION_HANDSHAKE
    /// </summary>
    public class DataTransmissionHandshakePacket: PacketV2<DataTransmissionHandshakePayload>
    {
        public override byte CrcEtra => 29;
        public override int MessageId => 130;
        public override DataTransmissionHandshakePayload Payload { get; } = new DataTransmissionHandshakePayload();

        public override string Name => "DATA_TRANSMISSION_HANDSHAKE";
        public override string ToString()
        {
            var name = "DATA_TRANSMISSION_HANDSHAKE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  DATA_TRANSMISSION_HANDSHAKE
    /// </summary>
    public class DataTransmissionHandshakePayload : IPayload
    {
        public byte ByteSize => 13;

        public int Deserialize(byte[] buffer, int index)
        {
            Size = BitConverter.ToUInt32(buffer,index);index+=4;
            Width = BitConverter.ToUInt16(buffer,index);index+=2;
            Height = BitConverter.ToUInt16(buffer,index);index+=2;
            Packets = BitConverter.ToUInt16(buffer,index);index+=2;
            Type = (byte)buffer[index++];
            Payload = (byte)buffer[index++];
            JpgQuality = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Size).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Width).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Height).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Packets).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes((byte)Type).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Payload).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(JpgQuality).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// total data size (set on ACK only).
        /// bytes
        /// size
        /// </summary>
        public uint Size { get; set; }
        /// <summary>
        /// Width of a matrix or image.
        /// 
        /// width
        /// </summary>
        public ushort Width { get; set; }
        /// <summary>
        /// Height of a matrix or image.
        /// 
        /// height
        /// </summary>
        public ushort Height { get; set; }
        /// <summary>
        /// Number of packets being sent (set on ACK only).
        /// 
        /// packets
        /// </summary>
        public ushort Packets { get; set; }
        /// <summary>
        /// Type of requested/acknowledged data.
        /// 
        /// type
        /// </summary>
        public byte Type { get; set; }
        /// <summary>
        /// Payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only).
        /// bytes
        /// payload
        /// </summary>
        public byte Payload { get; set; }
        /// <summary>
        /// JPEG quality. Values: [1-100].
        /// %
        /// jpg_quality
        /// </summary>
        public byte JpgQuality { get; set; }
    }
    /// <summary>
    ///  ENCAPSULATED_DATA
    /// </summary>
    public class EncapsulatedDataPacket: PacketV2<EncapsulatedDataPayload>
    {
        public override byte CrcEtra => 223;
        public override int MessageId => 131;
        public override EncapsulatedDataPayload Payload { get; } = new EncapsulatedDataPayload();

        public override string Name => "ENCAPSULATED_DATA";
        public override string ToString()
        {
            var name = "ENCAPSULATED_DATA".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  ENCAPSULATED_DATA
    /// </summary>
    public class EncapsulatedDataPayload : IPayload
    {
        public byte ByteSize => 255;

        public int Deserialize(byte[] buffer, int index)
        {
            Seqnr = BitConverter.ToUInt16(buffer,index);index+=2;
            for(var i=0;i<253;i++)
            {
                Data[i] = (byte)buffer[index++];
            }
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Seqnr).CopyTo(buffer, index);index+=2;
            for(var i=0;i<253;i++)
            {
                BitConverter.GetBytes(Data[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// sequence number (starting with 0 on every transmission)
        /// 
        /// seqnr
        /// </summary>
        public ushort Seqnr { get; set; }
        /// <summary>
        /// image data bytes
        /// 
        /// data
        /// </summary>
        public byte[] Data { get; } = new byte[253];
    }
    /// <summary>
    ///  DISTANCE_SENSOR
    /// </summary>
    public class DistanceSensorPacket: PacketV2<DistanceSensorPayload>
    {
        public override byte CrcEtra => 85;
        public override int MessageId => 132;
        public override DistanceSensorPayload Payload { get; } = new DistanceSensorPayload();

        public override string Name => "DISTANCE_SENSOR";
        public override string ToString()
        {
            var name = "DISTANCE_SENSOR".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  DISTANCE_SENSOR
    /// </summary>
    public class DistanceSensorPayload : IPayload
    {
        public byte ByteSize => 14;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            MinDistance = BitConverter.ToUInt16(buffer,index);index+=2;
            MaxDistance = BitConverter.ToUInt16(buffer,index);index+=2;
            CurrentDistance = BitConverter.ToUInt16(buffer,index);index+=2;
            Type = (MavDistanceSensor)buffer[index++];
            Id = (byte)buffer[index++];
            Orientation = (MavSensorOrientation)buffer[index++];
            Covariance = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(MinDistance).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(MaxDistance).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(CurrentDistance).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes((byte)Type).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Id).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)Orientation).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Covariance).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Minimum distance the sensor can measure
        /// cm
        /// min_distance
        /// </summary>
        public ushort MinDistance { get; set; }
        /// <summary>
        /// Maximum distance the sensor can measure
        /// cm
        /// max_distance
        /// </summary>
        public ushort MaxDistance { get; set; }
        /// <summary>
        /// Current distance reading
        /// cm
        /// current_distance
        /// </summary>
        public ushort CurrentDistance { get; set; }
        /// <summary>
        /// Type of distance sensor.
        /// 
        /// type
        /// </summary>
        public MavDistanceSensor Type { get; set; }
        /// <summary>
        /// Onboard ID of the sensor
        /// 
        /// id
        /// </summary>
        public byte Id { get; set; }
        /// <summary>
        /// Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270
        /// 
        /// orientation
        /// </summary>
        public MavSensorOrientation Orientation { get; set; }
        /// <summary>
        /// Measurement covariance, 0 for unknown / invalid readings
        /// cm
        /// covariance
        /// </summary>
        public byte Covariance { get; set; }
    }
    /// <summary>
    /// Request for terrain data and terrain status
    ///  TERRAIN_REQUEST
    /// </summary>
    public class TerrainRequestPacket: PacketV2<TerrainRequestPayload>
    {
        public override byte CrcEtra => 6;
        public override int MessageId => 133;
        public override TerrainRequestPayload Payload { get; } = new TerrainRequestPayload();

        public override string Name => "TERRAIN_REQUEST";
        public override string ToString()
        {
            var name = "TERRAIN_REQUEST".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  TERRAIN_REQUEST
    /// </summary>
    public class TerrainRequestPayload : IPayload
    {
        public byte ByteSize => 18;

        public int Deserialize(byte[] buffer, int index)
        {
            Mask = BitConverter.ToUInt64(buffer,index);index+=8;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            GridSpacing = BitConverter.ToUInt16(buffer,index);index+=2;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Mask).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(GridSpacing).CopyTo(buffer, index);index+=2;
            return ByteSize;
        }

        /// <summary>
        /// Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
        /// 
        /// mask
        /// </summary>
        public ulong Mask { get; set; }
        /// <summary>
        /// Latitude of SW corner of first grid
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude of SW corner of first grid
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Grid spacing
        /// m
        /// grid_spacing
        /// </summary>
        public ushort GridSpacing { get; set; }
    }
    /// <summary>
    /// Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST
    ///  TERRAIN_DATA
    /// </summary>
    public class TerrainDataPacket: PacketV2<TerrainDataPayload>
    {
        public override byte CrcEtra => 229;
        public override int MessageId => 134;
        public override TerrainDataPayload Payload { get; } = new TerrainDataPayload();

        public override string Name => "TERRAIN_DATA";
        public override string ToString()
        {
            var name = "TERRAIN_DATA".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  TERRAIN_DATA
    /// </summary>
    public class TerrainDataPayload : IPayload
    {
        public byte ByteSize => 43;

        public int Deserialize(byte[] buffer, int index)
        {
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            GridSpacing = BitConverter.ToUInt16(buffer,index);index+=2;
            for(var i=0;i<16;i++)
            {
                Data[i] = BitConverter.ToInt16(buffer,index);index+=2;
            }
            Gridbit = (byte)buffer[index++];
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Latitude of SW corner of first grid
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude of SW corner of first grid
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Grid spacing
        /// m
        /// grid_spacing
        /// </summary>
        public ushort GridSpacing { get; set; }
        /// <summary>
        /// Terrain data AMSL
        /// m
        /// data
        /// </summary>
        public short[] Data { get; } = new short[16];
        /// <summary>
        /// bit within the terrain request mask
        /// 
        /// gridbit
        /// </summary>
        public byte Gridbit { get; set; }
    }
    /// <summary>
    /// Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle has all terrain data needed for a mission.
    ///  TERRAIN_CHECK
    /// </summary>
    public class TerrainCheckPacket: PacketV2<TerrainCheckPayload>
    {
        public override byte CrcEtra => 203;
        public override int MessageId => 135;
        public override TerrainCheckPayload Payload { get; } = new TerrainCheckPayload();

        public override string Name => "TERRAIN_CHECK";
        public override string ToString()
        {
            var name = "TERRAIN_CHECK".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  TERRAIN_CHECK
    /// </summary>
    public class TerrainCheckPayload : IPayload
    {
        public byte ByteSize => 8;

        public int Deserialize(byte[] buffer, int index)
        {
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Lat).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Lon).CopyTo(buffer, index);index+=4;
            return ByteSize;
        }

        /// <summary>
        /// Latitude
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
    }
    /// <summary>
    /// Response from a TERRAIN_CHECK request
    ///  TERRAIN_REPORT
    /// </summary>
    public class TerrainReportPacket: PacketV2<TerrainReportPayload>
    {
        public override byte CrcEtra => 1;
        public override int MessageId => 136;
        public override TerrainReportPayload Payload { get; } = new TerrainReportPayload();

        public override string Name => "TERRAIN_REPORT";
        public override string ToString()
        {
            var name = "TERRAIN_REPORT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  TERRAIN_REPORT
    /// </summary>
    public class TerrainReportPayload : IPayload
    {
        public byte ByteSize => 22;

        public int Deserialize(byte[] buffer, int index)
        {
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            TerrainHeight = BitConverter.ToSingle(buffer, index);index+=4;
            CurrentHeight = BitConverter.ToSingle(buffer, index);index+=4;
            Spacing = BitConverter.ToUInt16(buffer,index);index+=2;
            Pending = BitConverter.ToUInt16(buffer,index);index+=2;
            Loaded = BitConverter.ToUInt16(buffer,index);index+=2;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Latitude
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Terrain height AMSL
        /// m
        /// terrain_height
        /// </summary>
        public float TerrainHeight { get; set; }
        /// <summary>
        /// Current vehicle height above lat/lon terrain height
        /// m
        /// current_height
        /// </summary>
        public float CurrentHeight { get; set; }
        /// <summary>
        /// grid spacing (zero if terrain at this location unavailable)
        /// 
        /// spacing
        /// </summary>
        public ushort Spacing { get; set; }
        /// <summary>
        /// Number of 4x4 terrain blocks waiting to be received or read from disk
        /// 
        /// pending
        /// </summary>
        public ushort Pending { get; set; }
        /// <summary>
        /// Number of 4x4 terrain blocks in memory
        /// 
        /// loaded
        /// </summary>
        public ushort Loaded { get; set; }
    }
    /// <summary>
    /// Barometer readings for 2nd barometer
    ///  SCALED_PRESSURE2
    /// </summary>
    public class ScaledPressure2Packet: PacketV2<ScaledPressure2Payload>
    {
        public override byte CrcEtra => 195;
        public override int MessageId => 137;
        public override ScaledPressure2Payload Payload { get; } = new ScaledPressure2Payload();

        public override string Name => "SCALED_PRESSURE2";
        public override string ToString()
        {
            var name = "SCALED_PRESSURE2".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SCALED_PRESSURE2
    /// </summary>
    public class ScaledPressure2Payload : IPayload
    {
        public byte ByteSize => 14;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            PressAbs = BitConverter.ToSingle(buffer, index);index+=4;
            PressDiff = BitConverter.ToSingle(buffer, index);index+=4;
            Temperature = BitConverter.ToInt16(buffer,index);index+=2;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PressAbs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PressDiff).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=2;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Absolute pressure
        /// hPa
        /// press_abs
        /// </summary>
        public float PressAbs { get; set; }
        /// <summary>
        /// Differential pressure
        /// hPa
        /// press_diff
        /// </summary>
        public float PressDiff { get; set; }
        /// <summary>
        /// Temperature measurement
        /// cdegC
        /// temperature
        /// </summary>
        public short Temperature { get; set; }
    }
    /// <summary>
    /// Motion capture attitude and position
    ///  ATT_POS_MOCAP
    /// </summary>
    public class AttPosMocapPacket: PacketV2<AttPosMocapPayload>
    {
        public override byte CrcEtra => 109;
        public override int MessageId => 138;
        public override AttPosMocapPayload Payload { get; } = new AttPosMocapPayload();

        public override string Name => "ATT_POS_MOCAP";
        public override string ToString()
        {
            var name = "ATT_POS_MOCAP".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  ATT_POS_MOCAP
    /// </summary>
    public class AttPosMocapPayload : IPayload
    {
        public byte ByteSize => 36;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            for(var i=0;i<4;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            for(var i=0;i<21;i++)
            {
                Covariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        /// 
        /// q
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// X position (NED)
        /// m
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y position (NED)
        /// m
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z position (NED)
        /// m
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Pose covariance matrix upper right triangular (first six entries are the first ROW, next five entries are the second ROW, etc.)
        /// 
        /// covariance
        /// </summary>
        public float[] Covariance { get; } = new float[21];
    }
    /// <summary>
    /// Set the vehicle attitude and body angular rates.
    ///  SET_ACTUATOR_CONTROL_TARGET
    /// </summary>
    public class SetActuatorControlTargetPacket: PacketV2<SetActuatorControlTargetPayload>
    {
        public override byte CrcEtra => 168;
        public override int MessageId => 139;
        public override SetActuatorControlTargetPayload Payload { get; } = new SetActuatorControlTargetPayload();

        public override string Name => "SET_ACTUATOR_CONTROL_TARGET";
        public override string ToString()
        {
            var name = "SET_ACTUATOR_CONTROL_TARGET".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SET_ACTUATOR_CONTROL_TARGET
    /// </summary>
    public class SetActuatorControlTargetPayload : IPayload
    {
        public byte ByteSize => 43;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            for(var i=0;i<8;i++)
            {
                Controls[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            GroupMlx = (byte)buffer[index++];
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
        /// 
        /// controls
        /// </summary>
        public float[] Controls { get; } = new float[8];
        /// <summary>
        /// Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
        /// 
        /// group_mlx
        /// </summary>
        public byte GroupMlx { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Set the vehicle attitude and body angular rates.
    ///  ACTUATOR_CONTROL_TARGET
    /// </summary>
    public class ActuatorControlTargetPacket: PacketV2<ActuatorControlTargetPayload>
    {
        public override byte CrcEtra => 181;
        public override int MessageId => 140;
        public override ActuatorControlTargetPayload Payload { get; } = new ActuatorControlTargetPayload();

        public override string Name => "ACTUATOR_CONTROL_TARGET";
        public override string ToString()
        {
            var name = "ACTUATOR_CONTROL_TARGET".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  ACTUATOR_CONTROL_TARGET
    /// </summary>
    public class ActuatorControlTargetPayload : IPayload
    {
        public byte ByteSize => 41;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            for(var i=0;i<8;i++)
            {
                Controls[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            GroupMlx = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            for(var i=0;i<8;i++)
            {
                BitConverter.GetBytes(Controls[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(GroupMlx).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
        /// 
        /// controls
        /// </summary>
        public float[] Controls { get; } = new float[8];
        /// <summary>
        /// Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
        /// 
        /// group_mlx
        /// </summary>
        public byte GroupMlx { get; set; }
    }
    /// <summary>
    /// The current system altitude.
    ///  ALTITUDE
    /// </summary>
    public class AltitudePacket: PacketV2<AltitudePayload>
    {
        public override byte CrcEtra => 47;
        public override int MessageId => 141;
        public override AltitudePayload Payload { get; } = new AltitudePayload();

        public override string Name => "ALTITUDE";
        public override string ToString()
        {
            var name = "ALTITUDE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  ALTITUDE
    /// </summary>
    public class AltitudePayload : IPayload
    {
        public byte ByteSize => 32;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            AltitudeMonotonic = BitConverter.ToSingle(buffer, index);index+=4;
            AltitudeAmsl = BitConverter.ToSingle(buffer, index);index+=4;
            AltitudeLocal = BitConverter.ToSingle(buffer, index);index+=4;
            AltitudeRelative = BitConverter.ToSingle(buffer, index);index+=4;
            AltitudeTerrain = BitConverter.ToSingle(buffer, index);index+=4;
            BottomClearance = BitConverter.ToSingle(buffer, index);index+=4;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). The only guarantee on this field is that it will never be reset and is consistent within a flight. The recommended value for this field is the uncorrected barometric altitude at boot time. This altitude will also drift and vary between flights.
        /// m
        /// altitude_monotonic
        /// </summary>
        public float AltitudeMonotonic { get; set; }
        /// <summary>
        /// This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL by default and not the WGS84 altitude.
        /// m
        /// altitude_amsl
        /// </summary>
        public float AltitudeAmsl { get; set; }
        /// <summary>
        /// This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive.
        /// m
        /// altitude_local
        /// </summary>
        public float AltitudeLocal { get; set; }
        /// <summary>
        /// This is the altitude above the home position. It resets on each change of the current home position.
        /// m
        /// altitude_relative
        /// </summary>
        public float AltitudeRelative { get; set; }
        /// <summary>
        /// This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown.
        /// m
        /// altitude_terrain
        /// </summary>
        public float AltitudeTerrain { get; set; }
        /// <summary>
        /// This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available.
        /// m
        /// bottom_clearance
        /// </summary>
        public float BottomClearance { get; set; }
    }
    /// <summary>
    /// The autopilot is requesting a resource (file, binary, other type of data)
    ///  RESOURCE_REQUEST
    /// </summary>
    public class ResourceRequestPacket: PacketV2<ResourceRequestPayload>
    {
        public override byte CrcEtra => 72;
        public override int MessageId => 142;
        public override ResourceRequestPayload Payload { get; } = new ResourceRequestPayload();

        public override string Name => "RESOURCE_REQUEST";
        public override string ToString()
        {
            var name = "RESOURCE_REQUEST".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  RESOURCE_REQUEST
    /// </summary>
    public class ResourceRequestPayload : IPayload
    {
        public byte ByteSize => 243;

        public int Deserialize(byte[] buffer, int index)
        {
            RequestId = (byte)buffer[index++];
            UriType = (byte)buffer[index++];
            for(var i=0;i<120;i++)
            {
                Uri[i] = (byte)buffer[index++];
            }
            TransferType = (byte)buffer[index++];
            for(var i=0;i<120;i++)
            {
                Storage[i] = (byte)buffer[index++];
            }
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(RequestId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(UriType).CopyTo(buffer, index);index+=1;
            for(var i=0;i<120;i++)
            {
                BitConverter.GetBytes(Uri[i]).CopyTo(buffer, index);index+=1;
            }
            BitConverter.GetBytes(TransferType).CopyTo(buffer, index);index+=1;
            for(var i=0;i<120;i++)
            {
                BitConverter.GetBytes(Storage[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// Request ID. This ID should be re-used when sending back URI contents
        /// 
        /// request_id
        /// </summary>
        public byte RequestId { get; set; }
        /// <summary>
        /// The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
        /// 
        /// uri_type
        /// </summary>
        public byte UriType { get; set; }
        /// <summary>
        /// The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)
        /// 
        /// uri
        /// </summary>
        public byte[] Uri { get; } = new byte[120];
        /// <summary>
        /// The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
        /// 
        /// transfer_type
        /// </summary>
        public byte TransferType { get; set; }
        /// <summary>
        /// The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP).
        /// 
        /// storage
        /// </summary>
        public byte[] Storage { get; } = new byte[120];
    }
    /// <summary>
    /// Barometer readings for 3rd barometer
    ///  SCALED_PRESSURE3
    /// </summary>
    public class ScaledPressure3Packet: PacketV2<ScaledPressure3Payload>
    {
        public override byte CrcEtra => 131;
        public override int MessageId => 143;
        public override ScaledPressure3Payload Payload { get; } = new ScaledPressure3Payload();

        public override string Name => "SCALED_PRESSURE3";
        public override string ToString()
        {
            var name = "SCALED_PRESSURE3".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SCALED_PRESSURE3
    /// </summary>
    public class ScaledPressure3Payload : IPayload
    {
        public byte ByteSize => 14;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            PressAbs = BitConverter.ToSingle(buffer, index);index+=4;
            PressDiff = BitConverter.ToSingle(buffer, index);index+=4;
            Temperature = BitConverter.ToInt16(buffer,index);index+=2;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PressAbs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(PressDiff).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=2;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Absolute pressure
        /// hPa
        /// press_abs
        /// </summary>
        public float PressAbs { get; set; }
        /// <summary>
        /// Differential pressure
        /// hPa
        /// press_diff
        /// </summary>
        public float PressDiff { get; set; }
        /// <summary>
        /// Temperature measurement
        /// cdegC
        /// temperature
        /// </summary>
        public short Temperature { get; set; }
    }
    /// <summary>
    /// current motion information from a designated system
    ///  FOLLOW_TARGET
    /// </summary>
    public class FollowTargetPacket: PacketV2<FollowTargetPayload>
    {
        public override byte CrcEtra => 127;
        public override int MessageId => 144;
        public override FollowTargetPayload Payload { get; } = new FollowTargetPayload();

        public override string Name => "FOLLOW_TARGET";
        public override string ToString()
        {
            var name = "FOLLOW_TARGET".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  FOLLOW_TARGET
    /// </summary>
    public class FollowTargetPayload : IPayload
    {
        public byte ByteSize => 93;

        public int Deserialize(byte[] buffer, int index)
        {
            Timestamp = BitConverter.ToUInt64(buffer,index);index+=8;
            CustomState = BitConverter.ToUInt64(buffer,index);index+=8;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToSingle(buffer, index);index+=4;
            for(var i=0;i<3;i++)
            {
                Vel[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<3;i++)
            {
                Acc[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<4;i++)
            {
                AttitudeQ[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<3;i++)
            {
                Rates[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<3;i++)
            {
                PositionCov[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            EstCapabilities = (byte)buffer[index++];
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// timestamp
        /// </summary>
        public ulong Timestamp { get; set; }
        /// <summary>
        /// button states or switches of a tracker device
        /// 
        /// custom_state
        /// </summary>
        public ulong CustomState { get; set; }
        /// <summary>
        /// Latitude (WGS84)
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude (WGS84)
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude (AMSL)
        /// m
        /// alt
        /// </summary>
        public float Alt { get; set; }
        /// <summary>
        /// target velocity (0,0,0) for unknown
        /// m/s
        /// vel
        /// </summary>
        public float[] Vel { get; } = new float[3];
        /// <summary>
        /// linear target acceleration (0,0,0) for unknown
        /// m/s/s
        /// acc
        /// </summary>
        public float[] Acc { get; } = new float[3];
        /// <summary>
        /// (1 0 0 0 for unknown)
        /// 
        /// attitude_q
        /// </summary>
        public float[] AttitudeQ { get; } = new float[4];
        /// <summary>
        /// (0 0 0 for unknown)
        /// 
        /// rates
        /// </summary>
        public float[] Rates { get; } = new float[3];
        /// <summary>
        /// eph epv
        /// 
        /// position_cov
        /// </summary>
        public float[] PositionCov { get; } = new float[3];
        /// <summary>
        /// bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
        /// 
        /// est_capabilities
        /// </summary>
        public byte EstCapabilities { get; set; }
    }
    /// <summary>
    /// The smoothed, monotonic system state used to feed the control loops of the system.
    ///  CONTROL_SYSTEM_STATE
    /// </summary>
    public class ControlSystemStatePacket: PacketV2<ControlSystemStatePayload>
    {
        public override byte CrcEtra => 103;
        public override int MessageId => 146;
        public override ControlSystemStatePayload Payload { get; } = new ControlSystemStatePayload();

        public override string Name => "CONTROL_SYSTEM_STATE";
        public override string ToString()
        {
            var name = "CONTROL_SYSTEM_STATE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  CONTROL_SYSTEM_STATE
    /// </summary>
    public class ControlSystemStatePayload : IPayload
    {
        public byte ByteSize => 100;

        public int Deserialize(byte[] buffer, int index)
        {
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
            for(var i=0;i<3;i++)
            {
                VelVariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<3;i++)
            {
                PosVariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<4;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            RollRate = BitConverter.ToSingle(buffer, index);index+=4;
            PitchRate = BitConverter.ToSingle(buffer, index);index+=4;
            YawRate = BitConverter.ToSingle(buffer, index);index+=4;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X acceleration in body frame
        /// m/s/s
        /// x_acc
        /// </summary>
        public float XAcc { get; set; }
        /// <summary>
        /// Y acceleration in body frame
        /// m/s/s
        /// y_acc
        /// </summary>
        public float YAcc { get; set; }
        /// <summary>
        /// Z acceleration in body frame
        /// m/s/s
        /// z_acc
        /// </summary>
        public float ZAcc { get; set; }
        /// <summary>
        /// X velocity in body frame
        /// m/s
        /// x_vel
        /// </summary>
        public float XVel { get; set; }
        /// <summary>
        /// Y velocity in body frame
        /// m/s
        /// y_vel
        /// </summary>
        public float YVel { get; set; }
        /// <summary>
        /// Z velocity in body frame
        /// m/s
        /// z_vel
        /// </summary>
        public float ZVel { get; set; }
        /// <summary>
        /// X position in local frame
        /// m
        /// x_pos
        /// </summary>
        public float XPos { get; set; }
        /// <summary>
        /// Y position in local frame
        /// m
        /// y_pos
        /// </summary>
        public float YPos { get; set; }
        /// <summary>
        /// Z position in local frame
        /// m
        /// z_pos
        /// </summary>
        public float ZPos { get; set; }
        /// <summary>
        /// Airspeed, set to -1 if unknown
        /// m/s
        /// airspeed
        /// </summary>
        public float Airspeed { get; set; }
        /// <summary>
        /// Variance of body velocity estimate
        /// 
        /// vel_variance
        /// </summary>
        public float[] VelVariance { get; } = new float[3];
        /// <summary>
        /// Variance in local position
        /// 
        /// pos_variance
        /// </summary>
        public float[] PosVariance { get; } = new float[3];
        /// <summary>
        /// The attitude, represented as Quaternion
        /// 
        /// q
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Angular rate in roll axis
        /// rad/s
        /// roll_rate
        /// </summary>
        public float RollRate { get; set; }
        /// <summary>
        /// Angular rate in pitch axis
        /// rad/s
        /// pitch_rate
        /// </summary>
        public float PitchRate { get; set; }
        /// <summary>
        /// Angular rate in yaw axis
        /// rad/s
        /// yaw_rate
        /// </summary>
        public float YawRate { get; set; }
    }
    /// <summary>
    /// Battery information
    ///  BATTERY_STATUS
    /// </summary>
    public class BatteryStatusPacket: PacketV2<BatteryStatusPayload>
    {
        public override byte CrcEtra => 154;
        public override int MessageId => 147;
        public override BatteryStatusPayload Payload { get; } = new BatteryStatusPayload();

        public override string Name => "BATTERY_STATUS";
        public override string ToString()
        {
            var name = "BATTERY_STATUS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  BATTERY_STATUS
    /// </summary>
    public class BatteryStatusPayload : IPayload
    {
        public byte ByteSize => 36;

        public int Deserialize(byte[] buffer, int index)
        {
            CurrentConsumed = BitConverter.ToInt32(buffer,index);index+=4;
            EnergyConsumed = BitConverter.ToInt32(buffer,index);index+=4;
            Temperature = BitConverter.ToInt16(buffer,index);index+=2;
            for(var i=0;i<10;i++)
            {
                Voltages[i] = BitConverter.ToUInt16(buffer,index);index+=2;
            }
            CurrentBattery = BitConverter.ToInt16(buffer,index);index+=2;
            Id = (byte)buffer[index++];
            BatteryFunction = (MavBatteryFunction)buffer[index++];
            Type = (MavBatteryType)buffer[index++];
            BatteryRemaining = (sbyte)buffer[index++];
            TimeRemaining = BitConverter.ToInt32(buffer,index);index+=4;
            ChargeState = (MavBatteryChargeState)buffer[index++];
            return ByteSize;
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
            BitConverter.GetBytes((byte)BatteryFunction).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)Type).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(BatteryRemaining).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TimeRemaining).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes((byte)ChargeState).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Consumed charge, -1: autopilot does not provide consumption estimate
        /// mAh
        /// current_consumed
        /// </summary>
        public int CurrentConsumed { get; set; }
        /// <summary>
        /// Consumed energy, -1: autopilot does not provide energy consumption estimate
        /// hJ
        /// energy_consumed
        /// </summary>
        public int EnergyConsumed { get; set; }
        /// <summary>
        /// Temperature of the battery. INT16_MAX for unknown temperature.
        /// cdegC
        /// temperature
        /// </summary>
        public short Temperature { get; set; }
        /// <summary>
        /// Battery voltage of cells. Cells above the valid cell count for this battery should have the UINT16_MAX value.
        /// mV
        /// voltages
        /// </summary>
        public ushort[] Voltages { get; } = new ushort[10];
        /// <summary>
        /// Battery current, -1: autopilot does not measure the current
        /// cA
        /// current_battery
        /// </summary>
        public short CurrentBattery { get; set; }
        /// <summary>
        /// Battery ID
        /// 
        /// id
        /// </summary>
        public byte Id { get; set; }
        /// <summary>
        /// Function of the battery
        /// 
        /// battery_function
        /// </summary>
        public MavBatteryFunction BatteryFunction { get; set; }
        /// <summary>
        /// Type (chemistry) of the battery
        /// 
        /// type
        /// </summary>
        public MavBatteryType Type { get; set; }
        /// <summary>
        /// Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.
        /// %
        /// battery_remaining
        /// </summary>
        public sbyte BatteryRemaining { get; set; }
        /// <summary>
        /// Remaining battery time, 0: autopilot does not provide remaining battery time estimate
        /// s
        /// time_remaining
        /// </summary>
        public int TimeRemaining { get; set; }
        /// <summary>
        /// State for extent of discharge, provided by autopilot for warning or external reactions
        /// 
        /// charge_state
        /// </summary>
        public MavBatteryChargeState ChargeState { get; set; }
    }
    /// <summary>
    /// Version and capability of autopilot software
    ///  AUTOPILOT_VERSION
    /// </summary>
    public class AutopilotVersionPacket: PacketV2<AutopilotVersionPayload>
    {
        public override byte CrcEtra => 178;
        public override int MessageId => 148;
        public override AutopilotVersionPayload Payload { get; } = new AutopilotVersionPayload();

        public override string Name => "AUTOPILOT_VERSION";
        public override string ToString()
        {
            var name = "AUTOPILOT_VERSION".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  AUTOPILOT_VERSION
    /// </summary>
    public class AutopilotVersionPayload : IPayload
    {
        public byte ByteSize => 60;

        public int Deserialize(byte[] buffer, int index)
        {
            Capabilities = (MavProtocolCapability)BitConverter.ToUInt64(buffer,index);index+=8;
            Uid = BitConverter.ToUInt64(buffer,index);index+=8;
            FlightSwVersion = BitConverter.ToUInt32(buffer,index);index+=4;
            MiddlewareSwVersion = BitConverter.ToUInt32(buffer,index);index+=4;
            OsSwVersion = BitConverter.ToUInt32(buffer,index);index+=4;
            BoardVersion = BitConverter.ToUInt32(buffer,index);index+=4;
            VendorId = BitConverter.ToUInt16(buffer,index);index+=2;
            ProductId = BitConverter.ToUInt16(buffer,index);index+=2;
            for(var i=0;i<8;i++)
            {
                FlightCustomVersion[i] = (byte)buffer[index++];
            }
            for(var i=0;i<8;i++)
            {
                MiddlewareCustomVersion[i] = (byte)buffer[index++];
            }
            for(var i=0;i<8;i++)
            {
                OsCustomVersion[i] = (byte)buffer[index++];
            }
            for(var i=0;i<18;i++)
            {
                Uid2[i] = (byte)buffer[index++];
            }
            return ByteSize;
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
                BitConverter.GetBytes(FlightCustomVersion[i]).CopyTo(buffer, index);index+=1;
            }
            for(var i=0;i<8;i++)
            {
                BitConverter.GetBytes(MiddlewareCustomVersion[i]).CopyTo(buffer, index);index+=1;
            }
            for(var i=0;i<8;i++)
            {
                BitConverter.GetBytes(OsCustomVersion[i]).CopyTo(buffer, index);index+=1;
            }
            for(var i=0;i<18;i++)
            {
                BitConverter.GetBytes(Uid2[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// Bitmap of capabilities
        /// 
        /// capabilities
        /// </summary>
        public MavProtocolCapability Capabilities { get; set; }
        /// <summary>
        /// UID if provided by hardware (see uid2)
        /// 
        /// uid
        /// </summary>
        public ulong Uid { get; set; }
        /// <summary>
        /// Firmware version number
        /// 
        /// flight_sw_version
        /// </summary>
        public uint FlightSwVersion { get; set; }
        /// <summary>
        /// Middleware version number
        /// 
        /// middleware_sw_version
        /// </summary>
        public uint MiddlewareSwVersion { get; set; }
        /// <summary>
        /// Operating system version number
        /// 
        /// os_sw_version
        /// </summary>
        public uint OsSwVersion { get; set; }
        /// <summary>
        /// HW / board version (last 8 bytes should be silicon ID, if any)
        /// 
        /// board_version
        /// </summary>
        public uint BoardVersion { get; set; }
        /// <summary>
        /// ID of the board vendor
        /// 
        /// vendor_id
        /// </summary>
        public ushort VendorId { get; set; }
        /// <summary>
        /// ID of the product
        /// 
        /// product_id
        /// </summary>
        public ushort ProductId { get; set; }
        /// <summary>
        /// Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
        /// 
        /// flight_custom_version
        /// </summary>
        public byte[] FlightCustomVersion { get; } = new byte[8];
        /// <summary>
        /// Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
        /// 
        /// middleware_custom_version
        /// </summary>
        public byte[] MiddlewareCustomVersion { get; } = new byte[8];
        /// <summary>
        /// Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
        /// 
        /// os_custom_version
        /// </summary>
        public byte[] OsCustomVersion { get; } = new byte[8];
        /// <summary>
        /// UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise use uid)
        /// 
        /// uid2
        /// </summary>
        public byte[] Uid2 { get; } = new byte[18];
    }
    /// <summary>
    /// The location of a landing area captured from a downward facing camera
    ///  LANDING_TARGET
    /// </summary>
    public class LandingTargetPacket: PacketV2<LandingTargetPayload>
    {
        public override byte CrcEtra => 200;
        public override int MessageId => 149;
        public override LandingTargetPayload Payload { get; } = new LandingTargetPayload();

        public override string Name => "LANDING_TARGET";
        public override string ToString()
        {
            var name = "LANDING_TARGET".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  LANDING_TARGET
    /// </summary>
    public class LandingTargetPayload : IPayload
    {
        public byte ByteSize => 30;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            AngleX = BitConverter.ToSingle(buffer, index);index+=4;
            AngleY = BitConverter.ToSingle(buffer, index);index+=4;
            Distance = BitConverter.ToSingle(buffer, index);index+=4;
            SizeX = BitConverter.ToSingle(buffer, index);index+=4;
            SizeY = BitConverter.ToSingle(buffer, index);index+=4;
            TargetNum = (byte)buffer[index++];
            Frame = (MavFrame)buffer[index++];
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            for(var i=0;i<4;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            Type = (LandingTargetType)buffer[index++];
            PositionValid = (byte)buffer[index++];
            return ByteSize;
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
            BitConverter.GetBytes((byte)Frame).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(Q[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes((byte)Type).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(PositionValid).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X-axis angular offset of the target from the center of the image
        /// rad
        /// angle_x
        /// </summary>
        public float AngleX { get; set; }
        /// <summary>
        /// Y-axis angular offset of the target from the center of the image
        /// rad
        /// angle_y
        /// </summary>
        public float AngleY { get; set; }
        /// <summary>
        /// Distance to the target from the vehicle
        /// m
        /// distance
        /// </summary>
        public float Distance { get; set; }
        /// <summary>
        /// Size of target along x-axis
        /// rad
        /// size_x
        /// </summary>
        public float SizeX { get; set; }
        /// <summary>
        /// Size of target along y-axis
        /// rad
        /// size_y
        /// </summary>
        public float SizeY { get; set; }
        /// <summary>
        /// The ID of the target if multiple targets are present
        /// 
        /// target_num
        /// </summary>
        public byte TargetNum { get; set; }
        /// <summary>
        /// Coordinate frame used for following fields.
        /// 
        /// frame
        /// </summary>
        public MavFrame Frame { get; set; }
        /// <summary>
        /// X Position of the landing target on MAV_FRAME
        /// m
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y Position of the landing target on MAV_FRAME
        /// m
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z Position of the landing target on MAV_FRAME
        /// m
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        /// 
        /// q
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Type of landing target
        /// 
        /// type
        /// </summary>
        public LandingTargetType Type { get; set; }
        /// <summary>
        /// Boolean indicating known position (1) or default unknown position (0), for validation of positioning of the landing target
        /// 
        /// position_valid
        /// </summary>
        public byte PositionValid { get; set; }
    }
    /// <summary>
    /// Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS enum definition for further information. The innovation test ratios show the magnitude of the sensor innovation divided by the innovation check threshold. Under normal operation the innovation test ratios should be below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should be optional and controllable by the user.
    ///  ESTIMATOR_STATUS
    /// </summary>
    public class EstimatorStatusPacket: PacketV2<EstimatorStatusPayload>
    {
        public override byte CrcEtra => 163;
        public override int MessageId => 230;
        public override EstimatorStatusPayload Payload { get; } = new EstimatorStatusPayload();

        public override string Name => "ESTIMATOR_STATUS";
        public override string ToString()
        {
            var name = "ESTIMATOR_STATUS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  ESTIMATOR_STATUS
    /// </summary>
    public class EstimatorStatusPayload : IPayload
    {
        public byte ByteSize => 42;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Velocity innovation test ratio
        /// 
        /// vel_ratio
        /// </summary>
        public float VelRatio { get; set; }
        /// <summary>
        /// Horizontal position innovation test ratio
        /// 
        /// pos_horiz_ratio
        /// </summary>
        public float PosHorizRatio { get; set; }
        /// <summary>
        /// Vertical position innovation test ratio
        /// 
        /// pos_vert_ratio
        /// </summary>
        public float PosVertRatio { get; set; }
        /// <summary>
        /// Magnetometer innovation test ratio
        /// 
        /// mag_ratio
        /// </summary>
        public float MagRatio { get; set; }
        /// <summary>
        /// Height above terrain innovation test ratio
        /// 
        /// hagl_ratio
        /// </summary>
        public float HaglRatio { get; set; }
        /// <summary>
        /// True airspeed innovation test ratio
        /// 
        /// tas_ratio
        /// </summary>
        public float TasRatio { get; set; }
        /// <summary>
        /// Horizontal position 1-STD accuracy relative to the EKF local origin
        /// m
        /// pos_horiz_accuracy
        /// </summary>
        public float PosHorizAccuracy { get; set; }
        /// <summary>
        /// Vertical position 1-STD accuracy relative to the EKF local origin
        /// m
        /// pos_vert_accuracy
        /// </summary>
        public float PosVertAccuracy { get; set; }
        /// <summary>
        /// Bitmap indicating which EKF outputs are valid.
        /// 
        /// flags
        /// </summary>
        public EstimatorStatusFlags Flags { get; set; }
    }
    /// <summary>
    ///  WIND_COV
    /// </summary>
    public class WindCovPacket: PacketV2<WindCovPayload>
    {
        public override byte CrcEtra => 105;
        public override int MessageId => 231;
        public override WindCovPayload Payload { get; } = new WindCovPayload();

        public override string Name => "WIND_COV";
        public override string ToString()
        {
            var name = "WIND_COV".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  WIND_COV
    /// </summary>
    public class WindCovPayload : IPayload
    {
        public byte ByteSize => 40;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            WindX = BitConverter.ToSingle(buffer, index);index+=4;
            WindY = BitConverter.ToSingle(buffer, index);index+=4;
            WindZ = BitConverter.ToSingle(buffer, index);index+=4;
            VarHoriz = BitConverter.ToSingle(buffer, index);index+=4;
            VarVert = BitConverter.ToSingle(buffer, index);index+=4;
            WindAlt = BitConverter.ToSingle(buffer, index);index+=4;
            HorizAccuracy = BitConverter.ToSingle(buffer, index);index+=4;
            VertAccuracy = BitConverter.ToSingle(buffer, index);index+=4;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Wind in X (NED) direction
        /// m/s
        /// wind_x
        /// </summary>
        public float WindX { get; set; }
        /// <summary>
        /// Wind in Y (NED) direction
        /// m/s
        /// wind_y
        /// </summary>
        public float WindY { get; set; }
        /// <summary>
        /// Wind in Z (NED) direction
        /// m/s
        /// wind_z
        /// </summary>
        public float WindZ { get; set; }
        /// <summary>
        /// Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
        /// m/s
        /// var_horiz
        /// </summary>
        public float VarHoriz { get; set; }
        /// <summary>
        /// Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
        /// m/s
        /// var_vert
        /// </summary>
        public float VarVert { get; set; }
        /// <summary>
        /// Altitude (AMSL) that this measurement was taken at
        /// m
        /// wind_alt
        /// </summary>
        public float WindAlt { get; set; }
        /// <summary>
        /// Horizontal speed 1-STD accuracy
        /// m
        /// horiz_accuracy
        /// </summary>
        public float HorizAccuracy { get; set; }
        /// <summary>
        /// Vertical speed 1-STD accuracy
        /// m
        /// vert_accuracy
        /// </summary>
        public float VertAccuracy { get; set; }
    }
    /// <summary>
    /// GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position estimate of the system.
    ///  GPS_INPUT
    /// </summary>
    public class GpsInputPacket: PacketV2<GpsInputPayload>
    {
        public override byte CrcEtra => 151;
        public override int MessageId => 232;
        public override GpsInputPayload Payload { get; } = new GpsInputPayload();

        public override string Name => "GPS_INPUT";
        public override string ToString()
        {
            var name = "GPS_INPUT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  GPS_INPUT
    /// </summary>
    public class GpsInputPayload : IPayload
    {
        public byte ByteSize => 63;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// GPS time (from start of GPS week)
        /// ms
        /// time_week_ms
        /// </summary>
        public uint TimeWeekMs { get; set; }
        /// <summary>
        /// Latitude (WGS84)
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude (WGS84)
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude (AMSL). Positive for up.
        /// m
        /// alt
        /// </summary>
        public float Alt { get; set; }
        /// <summary>
        /// GPS HDOP horizontal dilution of position
        /// m
        /// hdop
        /// </summary>
        public float Hdop { get; set; }
        /// <summary>
        /// GPS VDOP vertical dilution of position
        /// m
        /// vdop
        /// </summary>
        public float Vdop { get; set; }
        /// <summary>
        /// GPS velocity in NORTH direction in earth-fixed NED frame
        /// m/s
        /// vn
        /// </summary>
        public float Vn { get; set; }
        /// <summary>
        /// GPS velocity in EAST direction in earth-fixed NED frame
        /// m/s
        /// ve
        /// </summary>
        public float Ve { get; set; }
        /// <summary>
        /// GPS velocity in DOWN direction in earth-fixed NED frame
        /// m/s
        /// vd
        /// </summary>
        public float Vd { get; set; }
        /// <summary>
        /// GPS speed accuracy
        /// m/s
        /// speed_accuracy
        /// </summary>
        public float SpeedAccuracy { get; set; }
        /// <summary>
        /// GPS horizontal accuracy
        /// m
        /// horiz_accuracy
        /// </summary>
        public float HorizAccuracy { get; set; }
        /// <summary>
        /// GPS vertical accuracy
        /// m
        /// vert_accuracy
        /// </summary>
        public float VertAccuracy { get; set; }
        /// <summary>
        /// Bitmap indicating which GPS input flags fields to ignore.  All other fields must be provided.
        /// 
        /// ignore_flags
        /// </summary>
        public GpsInputIgnoreFlags IgnoreFlags { get; set; }
        /// <summary>
        /// GPS week number
        /// 
        /// time_week
        /// </summary>
        public ushort TimeWeek { get; set; }
        /// <summary>
        /// ID of the GPS for multiple GPS inputs
        /// 
        /// gps_id
        /// </summary>
        public byte GpsId { get; set; }
        /// <summary>
        /// 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        /// 
        /// fix_type
        /// </summary>
        public byte FixType { get; set; }
        /// <summary>
        /// Number of satellites visible.
        /// 
        /// satellites_visible
        /// </summary>
        public byte SatellitesVisible { get; set; }
    }
    /// <summary>
    /// RTCM message for injecting into the onboard GPS (used for DGPS)
    ///  GPS_RTCM_DATA
    /// </summary>
    public class GpsRtcmDataPacket: PacketV2<GpsRtcmDataPayload>
    {
        public override byte CrcEtra => 35;
        public override int MessageId => 233;
        public override GpsRtcmDataPayload Payload { get; } = new GpsRtcmDataPayload();

        public override string Name => "GPS_RTCM_DATA";
        public override string ToString()
        {
            var name = "GPS_RTCM_DATA".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  GPS_RTCM_DATA
    /// </summary>
    public class GpsRtcmDataPayload : IPayload
    {
        public byte ByteSize => 182;

        public int Deserialize(byte[] buffer, int index)
        {
            Flags = (byte)buffer[index++];
            Len = (byte)buffer[index++];
            for(var i=0;i<180;i++)
            {
                Data[i] = (byte)buffer[index++];
            }
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Flags).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Len).CopyTo(buffer, index);index+=1;
            for(var i=0;i<180;i++)
            {
                BitConverter.GetBytes(Data[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer, while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment with a non full payload is received. This management is used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable transport delivery order.
        /// 
        /// flags
        /// </summary>
        public byte Flags { get; set; }
        /// <summary>
        /// data length
        /// bytes
        /// len
        /// </summary>
        public byte Len { get; set; }
        /// <summary>
        /// RTCM message (may be fragmented)
        /// 
        /// data
        /// </summary>
        public byte[] Data { get; } = new byte[180];
    }
    /// <summary>
    /// Message appropriate for high latency connections like Iridium
    ///  HIGH_LATENCY
    /// </summary>
    public class HighLatencyPacket: PacketV2<HighLatencyPayload>
    {
        public override byte CrcEtra => 150;
        public override int MessageId => 234;
        public override HighLatencyPayload Payload { get; } = new HighLatencyPayload();

        public override string Name => "HIGH_LATENCY";
        public override string ToString()
        {
            var name = "HIGH_LATENCY".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  HIGH_LATENCY
    /// </summary>
    public class HighLatencyPayload : IPayload
    {
        public byte ByteSize => 40;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            BitConverter.GetBytes((byte)BaseMode).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)LandedState).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Throttle).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Airspeed).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(AirspeedSp).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Groundspeed).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(ClimbRate).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(GpsNsat).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)GpsFixType).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(BatteryRemaining).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Temperature).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TemperatureAir).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Failsafe).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(WpNum).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// A bitfield for use for autopilot-specific flags.
        /// 
        /// custom_mode
        /// </summary>
        public uint CustomMode { get; set; }
        /// <summary>
        /// Latitude
        /// degE7
        /// latitude
        /// </summary>
        public int Latitude { get; set; }
        /// <summary>
        /// Longitude
        /// degE7
        /// longitude
        /// </summary>
        public int Longitude { get; set; }
        /// <summary>
        /// roll
        /// cdeg
        /// roll
        /// </summary>
        public short Roll { get; set; }
        /// <summary>
        /// pitch
        /// cdeg
        /// pitch
        /// </summary>
        public short Pitch { get; set; }
        /// <summary>
        /// heading
        /// cdeg
        /// heading
        /// </summary>
        public ushort Heading { get; set; }
        /// <summary>
        /// heading setpoint
        /// cdeg
        /// heading_sp
        /// </summary>
        public short HeadingSp { get; set; }
        /// <summary>
        /// Altitude above mean sea level
        /// m
        /// altitude_amsl
        /// </summary>
        public short AltitudeAmsl { get; set; }
        /// <summary>
        /// Altitude setpoint relative to the home position
        /// m
        /// altitude_sp
        /// </summary>
        public short AltitudeSp { get; set; }
        /// <summary>
        /// distance to target
        /// m
        /// wp_distance
        /// </summary>
        public ushort WpDistance { get; set; }
        /// <summary>
        /// Bitmap of enabled system modes.
        /// 
        /// base_mode
        /// </summary>
        public MavModeFlag BaseMode { get; set; }
        /// <summary>
        /// The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
        /// 
        /// landed_state
        /// </summary>
        public MavLandedState LandedState { get; set; }
        /// <summary>
        /// throttle (percentage)
        /// %
        /// throttle
        /// </summary>
        public sbyte Throttle { get; set; }
        /// <summary>
        /// airspeed
        /// m/s
        /// airspeed
        /// </summary>
        public byte Airspeed { get; set; }
        /// <summary>
        /// airspeed setpoint
        /// m/s
        /// airspeed_sp
        /// </summary>
        public byte AirspeedSp { get; set; }
        /// <summary>
        /// groundspeed
        /// m/s
        /// groundspeed
        /// </summary>
        public byte Groundspeed { get; set; }
        /// <summary>
        /// climb rate
        /// m/s
        /// climb_rate
        /// </summary>
        public sbyte ClimbRate { get; set; }
        /// <summary>
        /// Number of satellites visible. If unknown, set to 255
        /// 
        /// gps_nsat
        /// </summary>
        public byte GpsNsat { get; set; }
        /// <summary>
        /// GPS Fix type.
        /// 
        /// gps_fix_type
        /// </summary>
        public GpsFixType GpsFixType { get; set; }
        /// <summary>
        /// Remaining battery (percentage)
        /// %
        /// battery_remaining
        /// </summary>
        public byte BatteryRemaining { get; set; }
        /// <summary>
        /// Autopilot temperature (degrees C)
        /// degC
        /// temperature
        /// </summary>
        public sbyte Temperature { get; set; }
        /// <summary>
        /// Air temperature (degrees C) from airspeed sensor
        /// degC
        /// temperature_air
        /// </summary>
        public sbyte TemperatureAir { get; set; }
        /// <summary>
        /// failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
        /// 
        /// failsafe
        /// </summary>
        public byte Failsafe { get; set; }
        /// <summary>
        /// current waypoint number
        /// 
        /// wp_num
        /// </summary>
        public byte WpNum { get; set; }
    }
    /// <summary>
    /// Message appropriate for high latency connections like Iridium (version 2)
    ///  HIGH_LATENCY2
    /// </summary>
    public class HighLatency2Packet: PacketV2<HighLatency2Payload>
    {
        public override byte CrcEtra => 179;
        public override int MessageId => 235;
        public override HighLatency2Payload Payload { get; } = new HighLatency2Payload();

        public override string Name => "HIGH_LATENCY2";
        public override string ToString()
        {
            var name = "HIGH_LATENCY2".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  HIGH_LATENCY2
    /// </summary>
    public class HighLatency2Payload : IPayload
    {
        public byte ByteSize => 42;

        public int Deserialize(byte[] buffer, int index)
        {
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
            return ByteSize;
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
            BitConverter.GetBytes((byte)Type).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)Autopilot).CopyTo(buffer, index);index+=1;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (milliseconds since boot or Unix epoch)
        /// ms
        /// timestamp
        /// </summary>
        public uint Timestamp { get; set; }
        /// <summary>
        /// Latitude
        /// degE7
        /// latitude
        /// </summary>
        public int Latitude { get; set; }
        /// <summary>
        /// Longitude
        /// degE7
        /// longitude
        /// </summary>
        public int Longitude { get; set; }
        /// <summary>
        /// A bitfield for use for autopilot-specific flags (2 byte version).
        /// 
        /// custom_mode
        /// </summary>
        public ushort CustomMode { get; set; }
        /// <summary>
        /// Altitude above mean sea level
        /// m
        /// altitude
        /// </summary>
        public short Altitude { get; set; }
        /// <summary>
        /// Altitude setpoint
        /// m
        /// target_altitude
        /// </summary>
        public short TargetAltitude { get; set; }
        /// <summary>
        /// Distance to target waypoint or position
        /// dam
        /// target_distance
        /// </summary>
        public ushort TargetDistance { get; set; }
        /// <summary>
        /// Current waypoint number
        /// 
        /// wp_num
        /// </summary>
        public ushort WpNum { get; set; }
        /// <summary>
        /// Bitmap of failure flags.
        /// 
        /// failure_flags
        /// </summary>
        public HlFailureFlag FailureFlags { get; set; }
        /// <summary>
        /// Type of the MAV (quadrotor, helicopter, etc.)
        /// 
        /// type
        /// </summary>
        public MavType Type { get; set; }
        /// <summary>
        /// Autopilot type / class.
        /// 
        /// autopilot
        /// </summary>
        public MavAutopilot Autopilot { get; set; }
        /// <summary>
        /// Heading
        /// deg/2
        /// heading
        /// </summary>
        public byte Heading { get; set; }
        /// <summary>
        /// Heading setpoint
        /// deg/2
        /// target_heading
        /// </summary>
        public byte TargetHeading { get; set; }
        /// <summary>
        /// Throttle
        /// %
        /// throttle
        /// </summary>
        public byte Throttle { get; set; }
        /// <summary>
        /// Airspeed
        /// m/s*5
        /// airspeed
        /// </summary>
        public byte Airspeed { get; set; }
        /// <summary>
        /// Airspeed setpoint
        /// m/s*5
        /// airspeed_sp
        /// </summary>
        public byte AirspeedSp { get; set; }
        /// <summary>
        /// Groundspeed
        /// m/s*5
        /// groundspeed
        /// </summary>
        public byte Groundspeed { get; set; }
        /// <summary>
        /// Windspeed
        /// m/s*5
        /// windspeed
        /// </summary>
        public byte Windspeed { get; set; }
        /// <summary>
        /// Wind heading
        /// deg/2
        /// wind_heading
        /// </summary>
        public byte WindHeading { get; set; }
        /// <summary>
        /// Maximum error horizontal position since last message
        /// dm
        /// eph
        /// </summary>
        public byte Eph { get; set; }
        /// <summary>
        /// Maximum error vertical position since last message
        /// dm
        /// epv
        /// </summary>
        public byte Epv { get; set; }
        /// <summary>
        /// Air temperature from airspeed sensor
        /// degC
        /// temperature_air
        /// </summary>
        public sbyte TemperatureAir { get; set; }
        /// <summary>
        /// Maximum climb rate magnitude since last message
        /// dm/s
        /// climb_rate
        /// </summary>
        public sbyte ClimbRate { get; set; }
        /// <summary>
        /// Battery (percentage, -1 for DNU)
        /// %
        /// battery
        /// </summary>
        public sbyte Battery { get; set; }
        /// <summary>
        /// Field for custom payload.
        /// 
        /// custom0
        /// </summary>
        public sbyte Custom0 { get; set; }
        /// <summary>
        /// Field for custom payload.
        /// 
        /// custom1
        /// </summary>
        public sbyte Custom1 { get; set; }
        /// <summary>
        /// Field for custom payload.
        /// 
        /// custom2
        /// </summary>
        public sbyte Custom2 { get; set; }
    }
    /// <summary>
    /// Vibration levels and accelerometer clipping
    ///  VIBRATION
    /// </summary>
    public class VibrationPacket: PacketV2<VibrationPayload>
    {
        public override byte CrcEtra => 90;
        public override int MessageId => 241;
        public override VibrationPayload Payload { get; } = new VibrationPayload();

        public override string Name => "VIBRATION";
        public override string ToString()
        {
            var name = "VIBRATION".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  VIBRATION
    /// </summary>
    public class VibrationPayload : IPayload
    {
        public byte ByteSize => 32;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            VibrationX = BitConverter.ToSingle(buffer, index);index+=4;
            VibrationY = BitConverter.ToSingle(buffer, index);index+=4;
            VibrationZ = BitConverter.ToSingle(buffer, index);index+=4;
            Clipping0 = BitConverter.ToUInt32(buffer,index);index+=4;
            Clipping1 = BitConverter.ToUInt32(buffer,index);index+=4;
            Clipping2 = BitConverter.ToUInt32(buffer,index);index+=4;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Vibration levels on X-axis
        /// 
        /// vibration_x
        /// </summary>
        public float VibrationX { get; set; }
        /// <summary>
        /// Vibration levels on Y-axis
        /// 
        /// vibration_y
        /// </summary>
        public float VibrationY { get; set; }
        /// <summary>
        /// Vibration levels on Z-axis
        /// 
        /// vibration_z
        /// </summary>
        public float VibrationZ { get; set; }
        /// <summary>
        /// first accelerometer clipping count
        /// 
        /// clipping_0
        /// </summary>
        public uint Clipping0 { get; set; }
        /// <summary>
        /// second accelerometer clipping count
        /// 
        /// clipping_1
        /// </summary>
        public uint Clipping1 { get; set; }
        /// <summary>
        /// third accelerometer clipping count
        /// 
        /// clipping_2
        /// </summary>
        public uint Clipping2 { get; set; }
    }
    /// <summary>
    /// This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitly set by the operator before or after. The position the system will return to and land on. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
    ///  HOME_POSITION
    /// </summary>
    public class HomePositionPacket: PacketV2<HomePositionPayload>
    {
        public override byte CrcEtra => 104;
        public override int MessageId => 242;
        public override HomePositionPayload Payload { get; } = new HomePositionPayload();

        public override string Name => "HOME_POSITION";
        public override string ToString()
        {
            var name = "HOME_POSITION".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  HOME_POSITION
    /// </summary>
    public class HomePositionPayload : IPayload
    {
        public byte ByteSize => 52;

        public int Deserialize(byte[] buffer, int index)
        {
            Latitude = BitConverter.ToInt32(buffer,index);index+=4;
            Longitude = BitConverter.ToInt32(buffer,index);index+=4;
            Altitude = BitConverter.ToInt32(buffer,index);index+=4;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            for(var i=0;i<4;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            ApproachX = BitConverter.ToSingle(buffer, index);index+=4;
            ApproachY = BitConverter.ToSingle(buffer, index);index+=4;
            ApproachZ = BitConverter.ToSingle(buffer, index);index+=4;
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Latitude (WGS84)
        /// degE7
        /// latitude
        /// </summary>
        public int Latitude { get; set; }
        /// <summary>
        /// Longitude (WGS84)
        /// degE7
        /// longitude
        /// </summary>
        public int Longitude { get; set; }
        /// <summary>
        /// Altitude (AMSL). Positive for up.
        /// mm
        /// altitude
        /// </summary>
        public int Altitude { get; set; }
        /// <summary>
        /// Local X position of this position in the local coordinate frame
        /// m
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Local Y position of this position in the local coordinate frame
        /// m
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Local Z position of this position in the local coordinate frame
        /// m
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
        /// 
        /// q
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
        /// m
        /// approach_x
        /// </summary>
        public float ApproachX { get; set; }
        /// <summary>
        /// Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
        /// m
        /// approach_y
        /// </summary>
        public float ApproachY { get; set; }
        /// <summary>
        /// Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
        /// m
        /// approach_z
        /// </summary>
        public float ApproachZ { get; set; }
        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
    }
    /// <summary>
    /// The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitly set by the operator before or after. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
    ///  SET_HOME_POSITION
    /// </summary>
    public class SetHomePositionPacket: PacketV2<SetHomePositionPayload>
    {
        public override byte CrcEtra => 85;
        public override int MessageId => 243;
        public override SetHomePositionPayload Payload { get; } = new SetHomePositionPayload();

        public override string Name => "SET_HOME_POSITION";
        public override string ToString()
        {
            var name = "SET_HOME_POSITION".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SET_HOME_POSITION
    /// </summary>
    public class SetHomePositionPayload : IPayload
    {
        public byte ByteSize => 53;

        public int Deserialize(byte[] buffer, int index)
        {
            Latitude = BitConverter.ToInt32(buffer,index);index+=4;
            Longitude = BitConverter.ToInt32(buffer,index);index+=4;
            Altitude = BitConverter.ToInt32(buffer,index);index+=4;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            for(var i=0;i<4;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            ApproachX = BitConverter.ToSingle(buffer, index);index+=4;
            ApproachY = BitConverter.ToSingle(buffer, index);index+=4;
            ApproachZ = BitConverter.ToSingle(buffer, index);index+=4;
            TargetSystem = (byte)buffer[index++];
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Latitude (WGS84)
        /// degE7
        /// latitude
        /// </summary>
        public int Latitude { get; set; }
        /// <summary>
        /// Longitude (WGS84)
        /// degE7
        /// longitude
        /// </summary>
        public int Longitude { get; set; }
        /// <summary>
        /// Altitude (AMSL). Positive for up.
        /// mm
        /// altitude
        /// </summary>
        public int Altitude { get; set; }
        /// <summary>
        /// Local X position of this position in the local coordinate frame
        /// m
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Local Y position of this position in the local coordinate frame
        /// m
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Local Z position of this position in the local coordinate frame
        /// m
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
        /// 
        /// q
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
        /// m
        /// approach_x
        /// </summary>
        public float ApproachX { get; set; }
        /// <summary>
        /// Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
        /// m
        /// approach_y
        /// </summary>
        public float ApproachY { get; set; }
        /// <summary>
        /// Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
        /// m
        /// approach_z
        /// </summary>
        public float ApproachZ { get; set; }
        /// <summary>
        /// System ID.
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
    }
    /// <summary>
    /// The interval between messages for a particular MAVLink message ID. This interface replaces DATA_STREAM
    ///  MESSAGE_INTERVAL
    /// </summary>
    public class MessageIntervalPacket: PacketV2<MessageIntervalPayload>
    {
        public override byte CrcEtra => 95;
        public override int MessageId => 244;
        public override MessageIntervalPayload Payload { get; } = new MessageIntervalPayload();

        public override string Name => "MESSAGE_INTERVAL";
        public override string ToString()
        {
            var name = "MESSAGE_INTERVAL".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MESSAGE_INTERVAL
    /// </summary>
    public class MessageIntervalPayload : IPayload
    {
        public byte ByteSize => 6;

        public int Deserialize(byte[] buffer, int index)
        {
            IntervalUs = BitConverter.ToInt32(buffer,index);index+=4;
            MessageId = BitConverter.ToUInt16(buffer,index);index+=2;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(IntervalUs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(MessageId).CopyTo(buffer, index);index+=2;
            return ByteSize;
        }

        /// <summary>
        /// The interval between two messages. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.
        /// us
        /// interval_us
        /// </summary>
        public int IntervalUs { get; set; }
        /// <summary>
        /// The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
        /// 
        /// message_id
        /// </summary>
        public ushort MessageId { get; set; }
    }
    /// <summary>
    /// Provides state for additional features
    ///  EXTENDED_SYS_STATE
    /// </summary>
    public class ExtendedSysStatePacket: PacketV2<ExtendedSysStatePayload>
    {
        public override byte CrcEtra => 130;
        public override int MessageId => 245;
        public override ExtendedSysStatePayload Payload { get; } = new ExtendedSysStatePayload();

        public override string Name => "EXTENDED_SYS_STATE";
        public override string ToString()
        {
            var name = "EXTENDED_SYS_STATE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  EXTENDED_SYS_STATE
    /// </summary>
    public class ExtendedSysStatePayload : IPayload
    {
        public byte ByteSize => 2;

        public int Deserialize(byte[] buffer, int index)
        {
            VtolState = (MavVtolState)buffer[index++];
            LandedState = (MavLandedState)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes((byte)VtolState).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)LandedState).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
        /// 
        /// vtol_state
        /// </summary>
        public MavVtolState VtolState { get; set; }
        /// <summary>
        /// The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
        /// 
        /// landed_state
        /// </summary>
        public MavLandedState LandedState { get; set; }
    }
    /// <summary>
    /// The location and information of an ADSB vehicle
    ///  ADSB_VEHICLE
    /// </summary>
    public class AdsbVehiclePacket: PacketV2<AdsbVehiclePayload>
    {
        public override byte CrcEtra => 184;
        public override int MessageId => 246;
        public override AdsbVehiclePayload Payload { get; } = new AdsbVehiclePayload();

        public override string Name => "ADSB_VEHICLE";
        public override string ToString()
        {
            var name = "ADSB_VEHICLE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  ADSB_VEHICLE
    /// </summary>
    public class AdsbVehiclePayload : IPayload
    {
        public byte ByteSize => 38;

        public int Deserialize(byte[] buffer, int index)
        {
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
                Encoding.ASCII.GetChars(buffer, index,9,Callsign,0);
                index+=9;
            EmitterType = (AdsbEmitterType)buffer[index++];
            Tslc = (byte)buffer[index++];
            return ByteSize;
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
            BitConverter.GetBytes((byte)AltitudeType).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(Callsign,0,9,buffer,index);
            index+=9;
            BitConverter.GetBytes((byte)EmitterType).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Tslc).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// ICAO address
        /// 
        /// ICAO_address
        /// </summary>
        public uint IcaoAddress { get; set; }
        /// <summary>
        /// Latitude
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude(ASL)
        /// mm
        /// altitude
        /// </summary>
        public int Altitude { get; set; }
        /// <summary>
        /// Course over ground
        /// cdeg
        /// heading
        /// </summary>
        public ushort Heading { get; set; }
        /// <summary>
        /// The horizontal velocity
        /// cm/s
        /// hor_velocity
        /// </summary>
        public ushort HorVelocity { get; set; }
        /// <summary>
        /// The vertical velocity. Positive is up
        /// cm/s
        /// ver_velocity
        /// </summary>
        public short VerVelocity { get; set; }
        /// <summary>
        /// Bitmap to indicate various statuses including valid data fields
        /// 
        /// flags
        /// </summary>
        public AdsbFlags Flags { get; set; }
        /// <summary>
        /// Squawk code
        /// 
        /// squawk
        /// </summary>
        public ushort Squawk { get; set; }
        /// <summary>
        /// ADSB altitude type.
        /// 
        /// altitude_type
        /// </summary>
        public AdsbAltitudeType AltitudeType { get; set; }
        /// <summary>
        /// The callsign, 8+null
        /// 
        /// callsign
        /// </summary>
        public char[] Callsign { get; } = new char[9];
        /// <summary>
        /// ADSB emitter type.
        /// 
        /// emitter_type
        /// </summary>
        public AdsbEmitterType EmitterType { get; set; }
        /// <summary>
        /// Time since last communication in seconds
        /// s
        /// tslc
        /// </summary>
        public byte Tslc { get; set; }
    }
    /// <summary>
    /// Information about a potential collision
    ///  COLLISION
    /// </summary>
    public class CollisionPacket: PacketV2<CollisionPayload>
    {
        public override byte CrcEtra => 81;
        public override int MessageId => 247;
        public override CollisionPayload Payload { get; } = new CollisionPayload();

        public override string Name => "COLLISION";
        public override string ToString()
        {
            var name = "COLLISION".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  COLLISION
    /// </summary>
    public class CollisionPayload : IPayload
    {
        public byte ByteSize => 19;

        public int Deserialize(byte[] buffer, int index)
        {
            Id = BitConverter.ToUInt32(buffer,index);index+=4;
            TimeToMinimumDelta = BitConverter.ToSingle(buffer, index);index+=4;
            AltitudeMinimumDelta = BitConverter.ToSingle(buffer, index);index+=4;
            HorizontalMinimumDelta = BitConverter.ToSingle(buffer, index);index+=4;
            Src = (MavCollisionSrc)buffer[index++];
            Action = (MavCollisionAction)buffer[index++];
            ThreatLevel = (MavCollisionThreatLevel)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Id).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(TimeToMinimumDelta).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(AltitudeMinimumDelta).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(HorizontalMinimumDelta).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes((byte)Src).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)Action).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)ThreatLevel).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Unique identifier, domain based on src field
        /// 
        /// id
        /// </summary>
        public uint Id { get; set; }
        /// <summary>
        /// Estimated time until collision occurs
        /// s
        /// time_to_minimum_delta
        /// </summary>
        public float TimeToMinimumDelta { get; set; }
        /// <summary>
        /// Closest vertical distance between vehicle and object
        /// m
        /// altitude_minimum_delta
        /// </summary>
        public float AltitudeMinimumDelta { get; set; }
        /// <summary>
        /// Closest horizontal distance between vehicle and object
        /// m
        /// horizontal_minimum_delta
        /// </summary>
        public float HorizontalMinimumDelta { get; set; }
        /// <summary>
        /// Collision data source
        /// 
        /// src
        /// </summary>
        public MavCollisionSrc Src { get; set; }
        /// <summary>
        /// Action that is being taken to avoid this collision
        /// 
        /// action
        /// </summary>
        public MavCollisionAction Action { get; set; }
        /// <summary>
        /// How concerned the aircraft is about this collision
        /// 
        /// threat_level
        /// </summary>
        public MavCollisionThreatLevel ThreatLevel { get; set; }
    }
    /// <summary>
    /// Message implementing parts of the V2 payload specs in V1 frames for transitional support.
    ///  V2_EXTENSION
    /// </summary>
    public class V2ExtensionPacket: PacketV2<V2ExtensionPayload>
    {
        public override byte CrcEtra => 8;
        public override int MessageId => 248;
        public override V2ExtensionPayload Payload { get; } = new V2ExtensionPayload();

        public override string Name => "V2_EXTENSION";
        public override string ToString()
        {
            var name = "V2_EXTENSION".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  V2_EXTENSION
    /// </summary>
    public class V2ExtensionPayload : IPayload
    {
        public byte ByteSize => 254;

        public int Deserialize(byte[] buffer, int index)
        {
            MessageType = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetNetwork = (byte)buffer[index++];
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            for(var i=0;i<249;i++)
            {
                Payload[i] = (byte)buffer[index++];
            }
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(MessageType).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetNetwork).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            for(var i=0;i<249;i++)
            {
                BitConverter.GetBytes(Payload[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// A code that identifies the software component that understands this message (analogous to USB device classes or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
        /// 
        /// message_type
        /// </summary>
        public ushort MessageType { get; set; }
        /// <summary>
        /// Network ID (0 for broadcast)
        /// 
        /// target_network
        /// </summary>
        public byte TargetNetwork { get; set; }
        /// <summary>
        /// System ID (0 for broadcast)
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID (0 for broadcast)
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
        /// 
        /// payload
        /// </summary>
        public byte[] Payload { get; } = new byte[249];
    }
    /// <summary>
    /// Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    ///  MEMORY_VECT
    /// </summary>
    public class MemoryVectPacket: PacketV2<MemoryVectPayload>
    {
        public override byte CrcEtra => 204;
        public override int MessageId => 249;
        public override MemoryVectPayload Payload { get; } = new MemoryVectPayload();

        public override string Name => "MEMORY_VECT";
        public override string ToString()
        {
            var name = "MEMORY_VECT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MEMORY_VECT
    /// </summary>
    public class MemoryVectPayload : IPayload
    {
        public byte ByteSize => 36;

        public int Deserialize(byte[] buffer, int index)
        {
            Address = BitConverter.ToUInt16(buffer,index);index+=2;
            Ver = (byte)buffer[index++];
            Type = (byte)buffer[index++];
            for(var i=0;i<32;i++)
            {
                Value[i] = (sbyte)buffer[index++];
            }
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Address).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(Ver).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Type).CopyTo(buffer, index);index+=1;
            for(var i=0;i<32;i++)
            {
                BitConverter.GetBytes(Value[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// Starting address of the debug variables
        /// 
        /// address
        /// </summary>
        public ushort Address { get; set; }
        /// <summary>
        /// Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
        /// 
        /// ver
        /// </summary>
        public byte Ver { get; set; }
        /// <summary>
        /// Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
        /// 
        /// type
        /// </summary>
        public byte Type { get; set; }
        /// <summary>
        /// Memory contents at specified address
        /// 
        /// value
        /// </summary>
        public sbyte[] Value { get; } = new sbyte[32];
    }
    /// <summary>
    /// To debug something using a named 3D vector.
    ///  DEBUG_VECT
    /// </summary>
    public class DebugVectPacket: PacketV2<DebugVectPayload>
    {
        public override byte CrcEtra => 49;
        public override int MessageId => 250;
        public override DebugVectPayload Payload { get; } = new DebugVectPayload();

        public override string Name => "DEBUG_VECT";
        public override string ToString()
        {
            var name = "DEBUG_VECT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  DEBUG_VECT
    /// </summary>
    public class DebugVectPayload : IPayload
    {
        public byte ByteSize => 30;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
                Encoding.ASCII.GetChars(buffer, index,10,Name,0);
                index+=10;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(X).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Y).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Z).CopyTo(buffer, index);index+=4;
            Encoding.ASCII.GetBytes(Name,0,10,buffer,index);
            index+=10;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// x
        /// 
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// y
        /// 
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// z
        /// 
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Name
        /// 
        /// name
        /// </summary>
        public char[] Name { get; } = new char[10];
    }
    /// <summary>
    /// Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    ///  NAMED_VALUE_FLOAT
    /// </summary>
    public class NamedValueFloatPacket: PacketV2<NamedValueFloatPayload>
    {
        public override byte CrcEtra => 170;
        public override int MessageId => 251;
        public override NamedValueFloatPayload Payload { get; } = new NamedValueFloatPayload();

        public override string Name => "NAMED_VALUE_FLOAT";
        public override string ToString()
        {
            var name = "NAMED_VALUE_FLOAT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  NAMED_VALUE_FLOAT
    /// </summary>
    public class NamedValueFloatPayload : IPayload
    {
        public byte ByteSize => 18;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Value = BitConverter.ToSingle(buffer, index);index+=4;
                Encoding.ASCII.GetChars(buffer, index,10,Name,0);
                index+=10;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Value).CopyTo(buffer, index);index+=4;
            Encoding.ASCII.GetBytes(Name,0,10,buffer,index);
            index+=10;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Floating point value
        /// 
        /// value
        /// </summary>
        public float Value { get; set; }
        /// <summary>
        /// Name of the debug variable
        /// 
        /// name
        /// </summary>
        public char[] Name { get; } = new char[10];
    }
    /// <summary>
    /// Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    ///  NAMED_VALUE_INT
    /// </summary>
    public class NamedValueIntPacket: PacketV2<NamedValueIntPayload>
    {
        public override byte CrcEtra => 44;
        public override int MessageId => 252;
        public override NamedValueIntPayload Payload { get; } = new NamedValueIntPayload();

        public override string Name => "NAMED_VALUE_INT";
        public override string ToString()
        {
            var name = "NAMED_VALUE_INT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  NAMED_VALUE_INT
    /// </summary>
    public class NamedValueIntPayload : IPayload
    {
        public byte ByteSize => 18;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Value = BitConverter.ToInt32(buffer,index);index+=4;
                Encoding.ASCII.GetChars(buffer, index,10,Name,0);
                index+=10;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Value).CopyTo(buffer, index);index+=4;
            Encoding.ASCII.GetBytes(Name,0,10,buffer,index);
            index+=10;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Signed integer value
        /// 
        /// value
        /// </summary>
        public int Value { get; set; }
        /// <summary>
        /// Name of the debug variable
        /// 
        /// name
        /// </summary>
        public char[] Name { get; } = new char[10];
    }
    /// <summary>
    /// Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
    ///  STATUSTEXT
    /// </summary>
    public class StatustextPacket: PacketV2<StatustextPayload>
    {
        public override byte CrcEtra => 83;
        public override int MessageId => 253;
        public override StatustextPayload Payload { get; } = new StatustextPayload();

        public override string Name => "STATUSTEXT";
        public override string ToString()
        {
            var name = "STATUSTEXT".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  STATUSTEXT
    /// </summary>
    public class StatustextPayload : IPayload
    {
        public byte ByteSize => 51;

        public int Deserialize(byte[] buffer, int index)
        {
            Severity = (MavSeverity)buffer[index++];
                Encoding.ASCII.GetChars(buffer, index,50,Text,0);
                index+=50;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes((byte)Severity).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(Text,0,50,buffer,index);
            index+=50;
            return ByteSize;
        }

        /// <summary>
        /// Severity of status. Relies on the definitions within RFC-5424.
        /// 
        /// severity
        /// </summary>
        public MavSeverity Severity { get; set; }
        /// <summary>
        /// Status text message, without null termination character
        /// 
        /// text
        /// </summary>
        public char[] Text { get; } = new char[50];
    }
    /// <summary>
    /// Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
    ///  DEBUG
    /// </summary>
    public class DebugPacket: PacketV2<DebugPayload>
    {
        public override byte CrcEtra => 46;
        public override int MessageId => 254;
        public override DebugPayload Payload { get; } = new DebugPayload();

        public override string Name => "DEBUG";
        public override string ToString()
        {
            var name = "DEBUG".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  DEBUG
    /// </summary>
    public class DebugPayload : IPayload
    {
        public byte ByteSize => 9;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Value = BitConverter.ToSingle(buffer, index);index+=4;
            Ind = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Value).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Ind).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// DEBUG value
        /// 
        /// value
        /// </summary>
        public float Value { get; set; }
        /// <summary>
        /// index of debug variable
        /// 
        /// ind
        /// </summary>
        public byte Ind { get; set; }
    }
    /// <summary>
    /// Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable signing
    ///  SETUP_SIGNING
    /// </summary>
    public class SetupSigningPacket: PacketV2<SetupSigningPayload>
    {
        public override byte CrcEtra => 71;
        public override int MessageId => 256;
        public override SetupSigningPayload Payload { get; } = new SetupSigningPayload();

        public override string Name => "SETUP_SIGNING";
        public override string ToString()
        {
            var name = "SETUP_SIGNING".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SETUP_SIGNING
    /// </summary>
    public class SetupSigningPayload : IPayload
    {
        public byte ByteSize => 42;

        public int Deserialize(byte[] buffer, int index)
        {
            InitialTimestamp = BitConverter.ToUInt64(buffer,index);index+=8;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            for(var i=0;i<32;i++)
            {
                SecretKey[i] = (byte)buffer[index++];
            }
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(InitialTimestamp).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            for(var i=0;i<32;i++)
            {
                BitConverter.GetBytes(SecretKey[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// initial timestamp
        /// 
        /// initial_timestamp
        /// </summary>
        public ulong InitialTimestamp { get; set; }
        /// <summary>
        /// system id of the target
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// component ID of the target
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// signing key
        /// 
        /// secret_key
        /// </summary>
        public byte[] SecretKey { get; } = new byte[32];
    }
    /// <summary>
    /// Report button state change.
    ///  BUTTON_CHANGE
    /// </summary>
    public class ButtonChangePacket: PacketV2<ButtonChangePayload>
    {
        public override byte CrcEtra => 131;
        public override int MessageId => 257;
        public override ButtonChangePayload Payload { get; } = new ButtonChangePayload();

        public override string Name => "BUTTON_CHANGE";
        public override string ToString()
        {
            var name = "BUTTON_CHANGE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  BUTTON_CHANGE
    /// </summary>
    public class ButtonChangePayload : IPayload
    {
        public byte ByteSize => 9;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            LastChangeMs = BitConverter.ToUInt32(buffer,index);index+=4;
            State = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(LastChangeMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(State).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Time of last change of button state.
        /// ms
        /// last_change_ms
        /// </summary>
        public uint LastChangeMs { get; set; }
        /// <summary>
        /// Bitmap for state of buttons.
        /// 
        /// state
        /// </summary>
        public byte State { get; set; }
    }
    /// <summary>
    /// Control vehicle tone generation (buzzer)
    ///  PLAY_TUNE
    /// </summary>
    public class PlayTunePacket: PacketV2<PlayTunePayload>
    {
        public override byte CrcEtra => 187;
        public override int MessageId => 258;
        public override PlayTunePayload Payload { get; } = new PlayTunePayload();

        public override string Name => "PLAY_TUNE";
        public override string ToString()
        {
            var name = "PLAY_TUNE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  PLAY_TUNE
    /// </summary>
    public class PlayTunePayload : IPayload
    {
        public byte ByteSize => 32;

        public int Deserialize(byte[] buffer, int index)
        {
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
                Encoding.ASCII.GetChars(buffer, index,30,Tune,0);
                index+=30;
                Encoding.ASCII.GetChars(buffer, index,200,Tune2,0);
                index+=200;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(Tune,0,30,buffer,index);
            index+=30;
            Encoding.ASCII.GetBytes(Tune2,0,200,buffer,index);
            index+=200;
            return ByteSize;
        }

        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// tune in board specific format
        /// 
        /// tune
        /// </summary>
        public char[] Tune { get; } = new char[30];
        /// <summary>
        /// tune extension (appended to tune)
        /// 
        /// tune2
        /// </summary>
        public char[] Tune2 { get; } = new char[200];
    }
    /// <summary>
    /// Information about a camera
    ///  CAMERA_INFORMATION
    /// </summary>
    public class CameraInformationPacket: PacketV2<CameraInformationPayload>
    {
        public override byte CrcEtra => 92;
        public override int MessageId => 259;
        public override CameraInformationPayload Payload { get; } = new CameraInformationPayload();

        public override string Name => "CAMERA_INFORMATION";
        public override string ToString()
        {
            var name = "CAMERA_INFORMATION".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  CAMERA_INFORMATION
    /// </summary>
    public class CameraInformationPayload : IPayload
    {
        public byte ByteSize => 235;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            FirmwareVersion = BitConverter.ToUInt32(buffer,index);index+=4;
            FocalLength = BitConverter.ToSingle(buffer, index);index+=4;
            SensorSizeH = BitConverter.ToSingle(buffer, index);index+=4;
            SensorSizeV = BitConverter.ToSingle(buffer, index);index+=4;
            Flags = (CameraCapFlags)BitConverter.ToUInt32(buffer,index);index+=4;
            ResolutionH = BitConverter.ToUInt16(buffer,index);index+=2;
            ResolutionV = BitConverter.ToUInt16(buffer,index);index+=2;
            CamDefinitionVersion = BitConverter.ToUInt16(buffer,index);index+=2;
            for(var i=0;i<32;i++)
            {
                VendorName[i] = (byte)buffer[index++];
            }
            for(var i=0;i<32;i++)
            {
                ModelName[i] = (byte)buffer[index++];
            }
            LensId = (byte)buffer[index++];
                Encoding.ASCII.GetChars(buffer, index,140,CamDefinitionUri,0);
                index+=140;
            return ByteSize;
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
                BitConverter.GetBytes(VendorName[i]).CopyTo(buffer, index);index+=1;
            }
            for(var i=0;i<32;i++)
            {
                BitConverter.GetBytes(ModelName[i]).CopyTo(buffer, index);index+=1;
            }
            BitConverter.GetBytes(LensId).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(CamDefinitionUri,0,140,buffer,index);
            index+=140;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Version of the camera firmware (v << 24 & 0xff = Dev, v << 16 & 0xff = Patch, v << 8 & 0xff = Minor, v & 0xff = Major)
        /// 
        /// firmware_version
        /// </summary>
        public uint FirmwareVersion { get; set; }
        /// <summary>
        /// Focal length
        /// mm
        /// focal_length
        /// </summary>
        public float FocalLength { get; set; }
        /// <summary>
        /// Image sensor size horizontal
        /// mm
        /// sensor_size_h
        /// </summary>
        public float SensorSizeH { get; set; }
        /// <summary>
        /// Image sensor size vertical
        /// mm
        /// sensor_size_v
        /// </summary>
        public float SensorSizeV { get; set; }
        /// <summary>
        /// Bitmap of camera capability flags.
        /// 
        /// flags
        /// </summary>
        public CameraCapFlags Flags { get; set; }
        /// <summary>
        /// Horizontal image resolution
        /// pix
        /// resolution_h
        /// </summary>
        public ushort ResolutionH { get; set; }
        /// <summary>
        /// Vertical image resolution
        /// pix
        /// resolution_v
        /// </summary>
        public ushort ResolutionV { get; set; }
        /// <summary>
        /// Camera definition version (iteration)
        /// 
        /// cam_definition_version
        /// </summary>
        public ushort CamDefinitionVersion { get; set; }
        /// <summary>
        /// Name of the camera vendor
        /// 
        /// vendor_name
        /// </summary>
        public byte[] VendorName { get; } = new byte[32];
        /// <summary>
        /// Name of the camera model
        /// 
        /// model_name
        /// </summary>
        public byte[] ModelName { get; } = new byte[32];
        /// <summary>
        /// Reserved for a lens ID
        /// 
        /// lens_id
        /// </summary>
        public byte LensId { get; set; }
        /// <summary>
        /// Camera definition URI (if any, otherwise only basic functions will be available).
        /// 
        /// cam_definition_uri
        /// </summary>
        public char[] CamDefinitionUri { get; } = new char[140];
    }
    /// <summary>
    /// Settings of a camera, can be requested using MAV_CMD_REQUEST_CAMERA_SETTINGS.
    ///  CAMERA_SETTINGS
    /// </summary>
    public class CameraSettingsPacket: PacketV2<CameraSettingsPayload>
    {
        public override byte CrcEtra => 146;
        public override int MessageId => 260;
        public override CameraSettingsPayload Payload { get; } = new CameraSettingsPayload();

        public override string Name => "CAMERA_SETTINGS";
        public override string ToString()
        {
            var name = "CAMERA_SETTINGS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  CAMERA_SETTINGS
    /// </summary>
    public class CameraSettingsPayload : IPayload
    {
        public byte ByteSize => 5;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            ModeId = (CameraMode)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes((byte)ModeId).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Camera mode
        /// 
        /// mode_id
        /// </summary>
        public CameraMode ModeId { get; set; }
    }
    /// <summary>
    /// Information about a storage medium.
    ///  STORAGE_INFORMATION
    /// </summary>
    public class StorageInformationPacket: PacketV2<StorageInformationPayload>
    {
        public override byte CrcEtra => 179;
        public override int MessageId => 261;
        public override StorageInformationPayload Payload { get; } = new StorageInformationPayload();

        public override string Name => "STORAGE_INFORMATION";
        public override string ToString()
        {
            var name = "STORAGE_INFORMATION".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  STORAGE_INFORMATION
    /// </summary>
    public class StorageInformationPayload : IPayload
    {
        public byte ByteSize => 27;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            TotalCapacity = BitConverter.ToSingle(buffer, index);index+=4;
            UsedCapacity = BitConverter.ToSingle(buffer, index);index+=4;
            AvailableCapacity = BitConverter.ToSingle(buffer, index);index+=4;
            ReadSpeed = BitConverter.ToSingle(buffer, index);index+=4;
            WriteSpeed = BitConverter.ToSingle(buffer, index);index+=4;
            StorageId = (byte)buffer[index++];
            StorageCount = (byte)buffer[index++];
            Status = (byte)buffer[index++];
            return ByteSize;
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
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Total capacity.
        /// MiB
        /// total_capacity
        /// </summary>
        public float TotalCapacity { get; set; }
        /// <summary>
        /// Used capacity.
        /// MiB
        /// used_capacity
        /// </summary>
        public float UsedCapacity { get; set; }
        /// <summary>
        /// Available storage capacity.
        /// MiB
        /// available_capacity
        /// </summary>
        public float AvailableCapacity { get; set; }
        /// <summary>
        /// Read speed.
        /// MiB/s
        /// read_speed
        /// </summary>
        public float ReadSpeed { get; set; }
        /// <summary>
        /// Write speed.
        /// MiB/s
        /// write_speed
        /// </summary>
        public float WriteSpeed { get; set; }
        /// <summary>
        /// Storage ID (1 for first, 2 for second, etc.)
        /// 
        /// storage_id
        /// </summary>
        public byte StorageId { get; set; }
        /// <summary>
        /// Number of storage devices
        /// 
        /// storage_count
        /// </summary>
        public byte StorageCount { get; set; }
        /// <summary>
        /// Status of storage (0 not available, 1 unformatted, 2 formatted)
        /// 
        /// status
        /// </summary>
        public byte Status { get; set; }
    }
    /// <summary>
    /// Information about the status of a capture.
    ///  CAMERA_CAPTURE_STATUS
    /// </summary>
    public class CameraCaptureStatusPacket: PacketV2<CameraCaptureStatusPayload>
    {
        public override byte CrcEtra => 12;
        public override int MessageId => 262;
        public override CameraCaptureStatusPayload Payload { get; } = new CameraCaptureStatusPayload();

        public override string Name => "CAMERA_CAPTURE_STATUS";
        public override string ToString()
        {
            var name = "CAMERA_CAPTURE_STATUS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  CAMERA_CAPTURE_STATUS
    /// </summary>
    public class CameraCaptureStatusPayload : IPayload
    {
        public byte ByteSize => 18;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            ImageInterval = BitConverter.ToSingle(buffer, index);index+=4;
            RecordingTimeMs = BitConverter.ToUInt32(buffer,index);index+=4;
            AvailableCapacity = BitConverter.ToSingle(buffer, index);index+=4;
            ImageStatus = (byte)buffer[index++];
            VideoStatus = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ImageInterval).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(RecordingTimeMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(AvailableCapacity).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(ImageStatus).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(VideoStatus).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Image capture interval
        /// s
        /// image_interval
        /// </summary>
        public float ImageInterval { get; set; }
        /// <summary>
        /// Time since recording started
        /// ms
        /// recording_time_ms
        /// </summary>
        public uint RecordingTimeMs { get; set; }
        /// <summary>
        /// Available storage capacity.
        /// MiB
        /// available_capacity
        /// </summary>
        public float AvailableCapacity { get; set; }
        /// <summary>
        /// Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)
        /// 
        /// image_status
        /// </summary>
        public byte ImageStatus { get; set; }
        /// <summary>
        /// Current status of video capturing (0: idle, 1: capture in progress)
        /// 
        /// video_status
        /// </summary>
        public byte VideoStatus { get; set; }
    }
    /// <summary>
    /// Information about a captured image
    ///  CAMERA_IMAGE_CAPTURED
    /// </summary>
    public class CameraImageCapturedPacket: PacketV2<CameraImageCapturedPayload>
    {
        public override byte CrcEtra => 133;
        public override int MessageId => 263;
        public override CameraImageCapturedPayload Payload { get; } = new CameraImageCapturedPayload();

        public override string Name => "CAMERA_IMAGE_CAPTURED";
        public override string ToString()
        {
            var name = "CAMERA_IMAGE_CAPTURED".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  CAMERA_IMAGE_CAPTURED
    /// </summary>
    public class CameraImageCapturedPayload : IPayload
    {
        public byte ByteSize => 255;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUtc = BitConverter.ToUInt64(buffer,index);index+=8;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Lat = BitConverter.ToInt32(buffer,index);index+=4;
            Lon = BitConverter.ToInt32(buffer,index);index+=4;
            Alt = BitConverter.ToInt32(buffer,index);index+=4;
            RelativeAlt = BitConverter.ToInt32(buffer,index);index+=4;
            for(var i=0;i<4;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            ImageIndex = BitConverter.ToInt32(buffer,index);index+=4;
            CameraId = (byte)buffer[index++];
            CaptureResult = (sbyte)buffer[index++];
                Encoding.ASCII.GetChars(buffer, index,205,FileUrl,0);
                index+=205;
            return ByteSize;
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
            Encoding.ASCII.GetBytes(FileUrl,0,205,buffer,index);
            index+=205;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since UNIX epoch) in UTC. 0 for unknown.
        /// us
        /// time_utc
        /// </summary>
        public ulong TimeUtc { get; set; }
        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Latitude where image was taken
        /// degE7
        /// lat
        /// </summary>
        public int Lat { get; set; }
        /// <summary>
        /// Longitude where capture was taken
        /// degE7
        /// lon
        /// </summary>
        public int Lon { get; set; }
        /// <summary>
        /// Altitude (AMSL) where image was taken
        /// mm
        /// alt
        /// </summary>
        public int Alt { get; set; }
        /// <summary>
        /// Altitude above ground
        /// mm
        /// relative_alt
        /// </summary>
        public int RelativeAlt { get; set; }
        /// <summary>
        /// Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
        /// 
        /// q
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// Zero based index of this image (image count since armed -1)
        /// 
        /// image_index
        /// </summary>
        public int ImageIndex { get; set; }
        /// <summary>
        /// Camera ID (1 for first, 2 for second, etc.)
        /// 
        /// camera_id
        /// </summary>
        public byte CameraId { get; set; }
        /// <summary>
        /// Boolean indicating success (1) or failure (0) while capturing this image.
        /// 
        /// capture_result
        /// </summary>
        public sbyte CaptureResult { get; set; }
        /// <summary>
        /// URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
        /// 
        /// file_url
        /// </summary>
        public char[] FileUrl { get; } = new char[205];
    }
    /// <summary>
    /// Information about flight since last arming.
    ///  FLIGHT_INFORMATION
    /// </summary>
    public class FlightInformationPacket: PacketV2<FlightInformationPayload>
    {
        public override byte CrcEtra => 49;
        public override int MessageId => 264;
        public override FlightInformationPayload Payload { get; } = new FlightInformationPayload();

        public override string Name => "FLIGHT_INFORMATION";
        public override string ToString()
        {
            var name = "FLIGHT_INFORMATION".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  FLIGHT_INFORMATION
    /// </summary>
    public class FlightInformationPayload : IPayload
    {
        public byte ByteSize => 28;

        public int Deserialize(byte[] buffer, int index)
        {
            ArmingTimeUtc = BitConverter.ToUInt64(buffer,index);index+=8;
            TakeoffTimeUtc = BitConverter.ToUInt64(buffer,index);index+=8;
            FlightUuid = BitConverter.ToUInt64(buffer,index);index+=8;
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ArmingTimeUtc).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(TakeoffTimeUtc).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(FlightUuid).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp at arming (time since UNIX epoch) in UTC, 0 for unknown
        /// us
        /// arming_time_utc
        /// </summary>
        public ulong ArmingTimeUtc { get; set; }
        /// <summary>
        /// Timestamp at takeoff (time since UNIX epoch) in UTC, 0 for unknown
        /// us
        /// takeoff_time_utc
        /// </summary>
        public ulong TakeoffTimeUtc { get; set; }
        /// <summary>
        /// Universally unique identifier (UUID) of flight, should correspond to name of log files
        /// 
        /// flight_uuid
        /// </summary>
        public ulong FlightUuid { get; set; }
        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
    }
    /// <summary>
    /// Orientation of a mount
    ///  MOUNT_ORIENTATION
    /// </summary>
    public class MountOrientationPacket: PacketV2<MountOrientationPayload>
    {
        public override byte CrcEtra => 26;
        public override int MessageId => 265;
        public override MountOrientationPayload Payload { get; } = new MountOrientationPayload();

        public override string Name => "MOUNT_ORIENTATION";
        public override string ToString()
        {
            var name = "MOUNT_ORIENTATION".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  MOUNT_ORIENTATION
    /// </summary>
    public class MountOrientationPayload : IPayload
    {
        public byte ByteSize => 16;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeBootMs = BitConverter.ToUInt32(buffer,index);index+=4;
            Roll = BitConverter.ToSingle(buffer, index);index+=4;
            Pitch = BitConverter.ToSingle(buffer, index);index+=4;
            Yaw = BitConverter.ToSingle(buffer, index);index+=4;
            YawAbsolute = BitConverter.ToSingle(buffer, index);index+=4;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeBootMs).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Roll).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Pitch).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Yaw).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(YawAbsolute).CopyTo(buffer, index);index+=4;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (time since system boot).
        /// ms
        /// time_boot_ms
        /// </summary>
        public uint TimeBootMs { get; set; }
        /// <summary>
        /// Roll in global frame (set to NaN for invalid).
        /// deg
        /// roll
        /// </summary>
        public float Roll { get; set; }
        /// <summary>
        /// Pitch in global frame (set to NaN for invalid).
        /// deg
        /// pitch
        /// </summary>
        public float Pitch { get; set; }
        /// <summary>
        /// Yaw relative to vehicle(set to NaN for invalid).
        /// deg
        /// yaw
        /// </summary>
        public float Yaw { get; set; }
        /// <summary>
        /// Yaw in absolute frame, North is 0 (set to NaN for invalid).
        /// deg
        /// yaw_absolute
        /// </summary>
        public float YawAbsolute { get; set; }
    }
    /// <summary>
    /// A message containing logged data (see also MAV_CMD_LOGGING_START)
    ///  LOGGING_DATA
    /// </summary>
    public class LoggingDataPacket: PacketV2<LoggingDataPayload>
    {
        public override byte CrcEtra => 193;
        public override int MessageId => 266;
        public override LoggingDataPayload Payload { get; } = new LoggingDataPayload();

        public override string Name => "LOGGING_DATA";
        public override string ToString()
        {
            var name = "LOGGING_DATA".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  LOGGING_DATA
    /// </summary>
    public class LoggingDataPayload : IPayload
    {
        public byte ByteSize => 255;

        public int Deserialize(byte[] buffer, int index)
        {
            Sequence = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            Length = (byte)buffer[index++];
            FirstMessageOffset = (byte)buffer[index++];
            for(var i=0;i<249;i++)
            {
                Data[i] = (byte)buffer[index++];
            }
            return ByteSize;
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
                BitConverter.GetBytes(Data[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// sequence number (can wrap)
        /// 
        /// sequence
        /// </summary>
        public ushort Sequence { get; set; }
        /// <summary>
        /// system ID of the target
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// component ID of the target
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// data length
        /// bytes
        /// length
        /// </summary>
        public byte Length { get; set; }
        /// <summary>
        /// offset into data where first message starts. This can be used for recovery, when a previous message got lost (set to 255 if no start exists).
        /// bytes
        /// first_message_offset
        /// </summary>
        public byte FirstMessageOffset { get; set; }
        /// <summary>
        /// logged data
        /// 
        /// data
        /// </summary>
        public byte[] Data { get; } = new byte[249];
    }
    /// <summary>
    /// A message containing logged data which requires a LOGGING_ACK to be sent back
    ///  LOGGING_DATA_ACKED
    /// </summary>
    public class LoggingDataAckedPacket: PacketV2<LoggingDataAckedPayload>
    {
        public override byte CrcEtra => 35;
        public override int MessageId => 267;
        public override LoggingDataAckedPayload Payload { get; } = new LoggingDataAckedPayload();

        public override string Name => "LOGGING_DATA_ACKED";
        public override string ToString()
        {
            var name = "LOGGING_DATA_ACKED".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  LOGGING_DATA_ACKED
    /// </summary>
    public class LoggingDataAckedPayload : IPayload
    {
        public byte ByteSize => 255;

        public int Deserialize(byte[] buffer, int index)
        {
            Sequence = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            Length = (byte)buffer[index++];
            FirstMessageOffset = (byte)buffer[index++];
            for(var i=0;i<249;i++)
            {
                Data[i] = (byte)buffer[index++];
            }
            return ByteSize;
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
                BitConverter.GetBytes(Data[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// sequence number (can wrap)
        /// 
        /// sequence
        /// </summary>
        public ushort Sequence { get; set; }
        /// <summary>
        /// system ID of the target
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// component ID of the target
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// data length
        /// bytes
        /// length
        /// </summary>
        public byte Length { get; set; }
        /// <summary>
        /// offset into data where first message starts. This can be used for recovery, when a previous message got lost (set to 255 if no start exists).
        /// bytes
        /// first_message_offset
        /// </summary>
        public byte FirstMessageOffset { get; set; }
        /// <summary>
        /// logged data
        /// 
        /// data
        /// </summary>
        public byte[] Data { get; } = new byte[249];
    }
    /// <summary>
    /// An ack for a LOGGING_DATA_ACKED message
    ///  LOGGING_ACK
    /// </summary>
    public class LoggingAckPacket: PacketV2<LoggingAckPayload>
    {
        public override byte CrcEtra => 14;
        public override int MessageId => 268;
        public override LoggingAckPayload Payload { get; } = new LoggingAckPayload();

        public override string Name => "LOGGING_ACK";
        public override string ToString()
        {
            var name = "LOGGING_ACK".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  LOGGING_ACK
    /// </summary>
    public class LoggingAckPayload : IPayload
    {
        public byte ByteSize => 4;

        public int Deserialize(byte[] buffer, int index)
        {
            Sequence = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Sequence).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// sequence number (must match the one in LOGGING_DATA_ACKED)
        /// 
        /// sequence
        /// </summary>
        public ushort Sequence { get; set; }
        /// <summary>
        /// system ID of the target
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// component ID of the target
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Information about video stream
    ///  VIDEO_STREAM_INFORMATION
    /// </summary>
    public class VideoStreamInformationPacket: PacketV2<VideoStreamInformationPayload>
    {
        public override byte CrcEtra => 58;
        public override int MessageId => 269;
        public override VideoStreamInformationPayload Payload { get; } = new VideoStreamInformationPayload();

        public override string Name => "VIDEO_STREAM_INFORMATION";
        public override string ToString()
        {
            var name = "VIDEO_STREAM_INFORMATION".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  VIDEO_STREAM_INFORMATION
    /// </summary>
    public class VideoStreamInformationPayload : IPayload
    {
        public byte ByteSize => 246;

        public int Deserialize(byte[] buffer, int index)
        {
            Framerate = BitConverter.ToSingle(buffer, index);index+=4;
            Bitrate = BitConverter.ToUInt32(buffer,index);index+=4;
            ResolutionH = BitConverter.ToUInt16(buffer,index);index+=2;
            ResolutionV = BitConverter.ToUInt16(buffer,index);index+=2;
            Rotation = BitConverter.ToUInt16(buffer,index);index+=2;
            CameraId = (byte)buffer[index++];
            Status = (byte)buffer[index++];
                Encoding.ASCII.GetChars(buffer, index,230,Uri,0);
                index+=230;
            return ByteSize;
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
            Encoding.ASCII.GetBytes(Uri,0,230,buffer,index);
            index+=230;
            return ByteSize;
        }

        /// <summary>
        /// Frame rate
        /// Hz
        /// framerate
        /// </summary>
        public float Framerate { get; set; }
        /// <summary>
        /// Bit rate in bits per second
        /// bits/s
        /// bitrate
        /// </summary>
        public uint Bitrate { get; set; }
        /// <summary>
        /// Horizontal resolution
        /// pix
        /// resolution_h
        /// </summary>
        public ushort ResolutionH { get; set; }
        /// <summary>
        /// Vertical resolution
        /// pix
        /// resolution_v
        /// </summary>
        public ushort ResolutionV { get; set; }
        /// <summary>
        /// Video image rotation clockwise
        /// deg
        /// rotation
        /// </summary>
        public ushort Rotation { get; set; }
        /// <summary>
        /// Camera ID (1 for first, 2 for second, etc.)
        /// 
        /// camera_id
        /// </summary>
        public byte CameraId { get; set; }
        /// <summary>
        /// Current status of video streaming (0: not running, 1: in progress)
        /// 
        /// status
        /// </summary>
        public byte Status { get; set; }
        /// <summary>
        /// Video stream URI
        /// 
        /// uri
        /// </summary>
        public char[] Uri { get; } = new char[230];
    }
    /// <summary>
    /// Message that sets video stream settings
    ///  SET_VIDEO_STREAM_SETTINGS
    /// </summary>
    public class SetVideoStreamSettingsPacket: PacketV2<SetVideoStreamSettingsPayload>
    {
        public override byte CrcEtra => 232;
        public override int MessageId => 270;
        public override SetVideoStreamSettingsPayload Payload { get; } = new SetVideoStreamSettingsPayload();

        public override string Name => "SET_VIDEO_STREAM_SETTINGS";
        public override string ToString()
        {
            var name = "SET_VIDEO_STREAM_SETTINGS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  SET_VIDEO_STREAM_SETTINGS
    /// </summary>
    public class SetVideoStreamSettingsPayload : IPayload
    {
        public byte ByteSize => 247;

        public int Deserialize(byte[] buffer, int index)
        {
            Framerate = BitConverter.ToSingle(buffer, index);index+=4;
            Bitrate = BitConverter.ToUInt32(buffer,index);index+=4;
            ResolutionH = BitConverter.ToUInt16(buffer,index);index+=2;
            ResolutionV = BitConverter.ToUInt16(buffer,index);index+=2;
            Rotation = BitConverter.ToUInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            CameraId = (byte)buffer[index++];
                Encoding.ASCII.GetChars(buffer, index,230,Uri,0);
                index+=230;
            return ByteSize;
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
            Encoding.ASCII.GetBytes(Uri,0,230,buffer,index);
            index+=230;
            return ByteSize;
        }

        /// <summary>
        /// Frame rate (set to -1 for highest framerate possible)
        /// Hz
        /// framerate
        /// </summary>
        public float Framerate { get; set; }
        /// <summary>
        /// Bit rate (set to -1 for auto)
        /// bits/s
        /// bitrate
        /// </summary>
        public uint Bitrate { get; set; }
        /// <summary>
        /// Horizontal resolution (set to -1 for highest resolution possible)
        /// pix
        /// resolution_h
        /// </summary>
        public ushort ResolutionH { get; set; }
        /// <summary>
        /// Vertical resolution (set to -1 for highest resolution possible)
        /// pix
        /// resolution_v
        /// </summary>
        public ushort ResolutionV { get; set; }
        /// <summary>
        /// Video image rotation clockwise (0-359 degrees)
        /// deg
        /// rotation
        /// </summary>
        public ushort Rotation { get; set; }
        /// <summary>
        /// system ID of the target
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// component ID of the target
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Camera ID (1 for first, 2 for second, etc.)
        /// 
        /// camera_id
        /// </summary>
        public byte CameraId { get; set; }
        /// <summary>
        /// Video stream URI
        /// 
        /// uri
        /// </summary>
        public char[] Uri { get; } = new char[230];
    }
    /// <summary>
    /// Configure AP SSID and Password.
    ///  WIFI_CONFIG_AP
    /// </summary>
    public class WifiConfigApPacket: PacketV2<WifiConfigApPayload>
    {
        public override byte CrcEtra => 19;
        public override int MessageId => 299;
        public override WifiConfigApPayload Payload { get; } = new WifiConfigApPayload();

        public override string Name => "WIFI_CONFIG_AP";
        public override string ToString()
        {
            var name = "WIFI_CONFIG_AP".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  WIFI_CONFIG_AP
    /// </summary>
    public class WifiConfigApPayload : IPayload
    {
        public byte ByteSize => 96;

        public int Deserialize(byte[] buffer, int index)
        {
                Encoding.ASCII.GetChars(buffer, index,32,Ssid,0);
                index+=32;
                Encoding.ASCII.GetChars(buffer, index,64,Password,0);
                index+=64;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            Encoding.ASCII.GetBytes(Ssid,0,32,buffer,index);
            index+=32;
            Encoding.ASCII.GetBytes(Password,0,64,buffer,index);
            index+=64;
            return ByteSize;
        }

        /// <summary>
        /// Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
        /// 
        /// ssid
        /// </summary>
        public char[] Ssid { get; } = new char[32];
        /// <summary>
        /// Password. Leave it blank for an open AP.
        /// 
        /// password
        /// </summary>
        public char[] Password { get; } = new char[64];
    }
    /// <summary>
    /// Version and capability of protocol version. This message is the response to REQUEST_PROTOCOL_VERSION and is used as part of the handshaking to establish which MAVLink version should be used on the network. Every node should respond to REQUEST_PROTOCOL_VERSION to enable the handshaking. Library implementers should consider adding this into the default decoding state machine to allow the protocol core to respond directly.
    ///  PROTOCOL_VERSION
    /// </summary>
    public class ProtocolVersionPacket: PacketV2<ProtocolVersionPayload>
    {
        public override byte CrcEtra => 217;
        public override int MessageId => 300;
        public override ProtocolVersionPayload Payload { get; } = new ProtocolVersionPayload();

        public override string Name => "PROTOCOL_VERSION";
        public override string ToString()
        {
            var name = "PROTOCOL_VERSION".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  PROTOCOL_VERSION
    /// </summary>
    public class ProtocolVersionPayload : IPayload
    {
        public byte ByteSize => 22;

        public int Deserialize(byte[] buffer, int index)
        {
            Version = BitConverter.ToUInt16(buffer,index);index+=2;
            MinVersion = BitConverter.ToUInt16(buffer,index);index+=2;
            MaxVersion = BitConverter.ToUInt16(buffer,index);index+=2;
            for(var i=0;i<8;i++)
            {
                SpecVersionHash[i] = (byte)buffer[index++];
            }
            for(var i=0;i<8;i++)
            {
                LibraryVersionHash[i] = (byte)buffer[index++];
            }
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(Version).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(MinVersion).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(MaxVersion).CopyTo(buffer, index);index+=2;
            for(var i=0;i<8;i++)
            {
                BitConverter.GetBytes(SpecVersionHash[i]).CopyTo(buffer, index);index+=1;
            }
            for(var i=0;i<8;i++)
            {
                BitConverter.GetBytes(LibraryVersionHash[i]).CopyTo(buffer, index);index+=1;
            }
            return ByteSize;
        }

        /// <summary>
        /// Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
        /// 
        /// version
        /// </summary>
        public ushort Version { get; set; }
        /// <summary>
        /// Minimum MAVLink version supported
        /// 
        /// min_version
        /// </summary>
        public ushort MinVersion { get; set; }
        /// <summary>
        /// Maximum MAVLink version supported (set to the same value as version by default)
        /// 
        /// max_version
        /// </summary>
        public ushort MaxVersion { get; set; }
        /// <summary>
        /// The first 8 bytes (not characters printed in hex!) of the git hash.
        /// 
        /// spec_version_hash
        /// </summary>
        public byte[] SpecVersionHash { get; } = new byte[8];
        /// <summary>
        /// The first 8 bytes (not characters printed in hex!) of the git hash.
        /// 
        /// library_version_hash
        /// </summary>
        public byte[] LibraryVersionHash { get; } = new byte[8];
    }
    /// <summary>
    /// General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message "uavcan.protocol.NodeStatus" for the background information. The UAVCAN specification is available at http://uavcan.org.
    ///  UAVCAN_NODE_STATUS
    /// </summary>
    public class UavcanNodeStatusPacket: PacketV2<UavcanNodeStatusPayload>
    {
        public override byte CrcEtra => 28;
        public override int MessageId => 310;
        public override UavcanNodeStatusPayload Payload { get; } = new UavcanNodeStatusPayload();

        public override string Name => "UAVCAN_NODE_STATUS";
        public override string ToString()
        {
            var name = "UAVCAN_NODE_STATUS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  UAVCAN_NODE_STATUS
    /// </summary>
    public class UavcanNodeStatusPayload : IPayload
    {
        public byte ByteSize => 17;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            UptimeSec = BitConverter.ToUInt32(buffer,index);index+=4;
            VendorSpecificStatusCode = BitConverter.ToUInt16(buffer,index);index+=2;
            Health = (UavcanNodeHealth)buffer[index++];
            Mode = (UavcanNodeMode)buffer[index++];
            SubMode = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(UptimeSec).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(VendorSpecificStatusCode).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes((byte)Health).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)Mode).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(SubMode).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Time since the start-up of the node.
        /// s
        /// uptime_sec
        /// </summary>
        public uint UptimeSec { get; set; }
        /// <summary>
        /// Vendor-specific status information.
        /// 
        /// vendor_specific_status_code
        /// </summary>
        public ushort VendorSpecificStatusCode { get; set; }
        /// <summary>
        /// Generalized node health status.
        /// 
        /// health
        /// </summary>
        public UavcanNodeHealth Health { get; set; }
        /// <summary>
        /// Generalized operating mode.
        /// 
        /// mode
        /// </summary>
        public UavcanNodeMode Mode { get; set; }
        /// <summary>
        /// Not used currently.
        /// 
        /// sub_mode
        /// </summary>
        public byte SubMode { get; set; }
    }
    /// <summary>
    /// General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN service "uavcan.protocol.GetNodeInfo" for the background information. This message should be emitted by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification is available at http://uavcan.org.
    ///  UAVCAN_NODE_INFO
    /// </summary>
    public class UavcanNodeInfoPacket: PacketV2<UavcanNodeInfoPayload>
    {
        public override byte CrcEtra => 95;
        public override int MessageId => 311;
        public override UavcanNodeInfoPayload Payload { get; } = new UavcanNodeInfoPayload();

        public override string Name => "UAVCAN_NODE_INFO";
        public override string ToString()
        {
            var name = "UAVCAN_NODE_INFO".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  UAVCAN_NODE_INFO
    /// </summary>
    public class UavcanNodeInfoPayload : IPayload
    {
        public byte ByteSize => 116;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            UptimeSec = BitConverter.ToUInt32(buffer,index);index+=4;
            SwVcsCommit = BitConverter.ToUInt32(buffer,index);index+=4;
                Encoding.ASCII.GetChars(buffer, index,80,Name,0);
                index+=80;
            HwVersionMajor = (byte)buffer[index++];
            HwVersionMinor = (byte)buffer[index++];
            for(var i=0;i<16;i++)
            {
                HwUniqueId[i] = (byte)buffer[index++];
            }
            SwVersionMajor = (byte)buffer[index++];
            SwVersionMinor = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(UptimeSec).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(SwVcsCommit).CopyTo(buffer, index);index+=4;
            Encoding.ASCII.GetBytes(Name,0,80,buffer,index);
            index+=80;
            BitConverter.GetBytes(HwVersionMajor).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(HwVersionMinor).CopyTo(buffer, index);index+=1;
            for(var i=0;i<16;i++)
            {
                BitConverter.GetBytes(HwUniqueId[i]).CopyTo(buffer, index);index+=1;
            }
            BitConverter.GetBytes(SwVersionMajor).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(SwVersionMinor).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Time since the start-up of the node.
        /// s
        /// uptime_sec
        /// </summary>
        public uint UptimeSec { get; set; }
        /// <summary>
        /// Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
        /// 
        /// sw_vcs_commit
        /// </summary>
        public uint SwVcsCommit { get; set; }
        /// <summary>
        /// Node name string. For example, "sapog.px4.io".
        /// 
        /// name
        /// </summary>
        public char[] Name { get; } = new char[80];
        /// <summary>
        /// Hardware major version number.
        /// 
        /// hw_version_major
        /// </summary>
        public byte HwVersionMajor { get; set; }
        /// <summary>
        /// Hardware minor version number.
        /// 
        /// hw_version_minor
        /// </summary>
        public byte HwVersionMinor { get; set; }
        /// <summary>
        /// Hardware unique 128-bit ID.
        /// 
        /// hw_unique_id
        /// </summary>
        public byte[] HwUniqueId { get; } = new byte[16];
        /// <summary>
        /// Software major version number.
        /// 
        /// sw_version_major
        /// </summary>
        public byte SwVersionMajor { get; set; }
        /// <summary>
        /// Software minor version number.
        /// 
        /// sw_version_minor
        /// </summary>
        public byte SwVersionMinor { get; set; }
    }
    /// <summary>
    /// Request to read the value of a parameter with the either the param_id string id or param_index.
    ///  PARAM_EXT_REQUEST_READ
    /// </summary>
    public class ParamExtRequestReadPacket: PacketV2<ParamExtRequestReadPayload>
    {
        public override byte CrcEtra => 243;
        public override int MessageId => 320;
        public override ParamExtRequestReadPayload Payload { get; } = new ParamExtRequestReadPayload();

        public override string Name => "PARAM_EXT_REQUEST_READ";
        public override string ToString()
        {
            var name = "PARAM_EXT_REQUEST_READ".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  PARAM_EXT_REQUEST_READ
    /// </summary>
    public class ParamExtRequestReadPayload : IPayload
    {
        public byte ByteSize => 20;

        public int Deserialize(byte[] buffer, int index)
        {
            ParamIndex = BitConverter.ToInt16(buffer,index);index+=2;
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
                Encoding.ASCII.GetChars(buffer, index,16,ParamId,0);
                index+=16;
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ParamIndex).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);
            index+=16;
            return ByteSize;
        }

        /// <summary>
        /// Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored)
        /// 
        /// param_index
        /// </summary>
        public short ParamIndex { get; set; }
        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// 
        /// param_id
        /// </summary>
        public char[] ParamId { get; } = new char[16];
    }
    /// <summary>
    /// Request all parameters of this component. After this request, all parameters are emitted.
    ///  PARAM_EXT_REQUEST_LIST
    /// </summary>
    public class ParamExtRequestListPacket: PacketV2<ParamExtRequestListPayload>
    {
        public override byte CrcEtra => 88;
        public override int MessageId => 321;
        public override ParamExtRequestListPayload Payload { get; } = new ParamExtRequestListPayload();

        public override string Name => "PARAM_EXT_REQUEST_LIST";
        public override string ToString()
        {
            var name = "PARAM_EXT_REQUEST_LIST".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  PARAM_EXT_REQUEST_LIST
    /// </summary>
    public class ParamExtRequestListPayload : IPayload
    {
        public byte ByteSize => 2;

        public int Deserialize(byte[] buffer, int index)
        {
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
    }
    /// <summary>
    /// Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows them to re-request missing parameters after a loss or timeout.
    ///  PARAM_EXT_VALUE
    /// </summary>
    public class ParamExtValuePacket: PacketV2<ParamExtValuePayload>
    {
        public override byte CrcEtra => 243;
        public override int MessageId => 322;
        public override ParamExtValuePayload Payload { get; } = new ParamExtValuePayload();

        public override string Name => "PARAM_EXT_VALUE";
        public override string ToString()
        {
            var name = "PARAM_EXT_VALUE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  PARAM_EXT_VALUE
    /// </summary>
    public class ParamExtValuePayload : IPayload
    {
        public byte ByteSize => 149;

        public int Deserialize(byte[] buffer, int index)
        {
            ParamCount = BitConverter.ToUInt16(buffer,index);index+=2;
            ParamIndex = BitConverter.ToUInt16(buffer,index);index+=2;
                Encoding.ASCII.GetChars(buffer, index,16,ParamId,0);
                index+=16;
                Encoding.ASCII.GetChars(buffer, index,128,ParamValue,0);
                index+=128;
            ParamType = (MavParamExtType)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(ParamCount).CopyTo(buffer, index);index+=2;
            BitConverter.GetBytes(ParamIndex).CopyTo(buffer, index);index+=2;
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);
            index+=16;
            Encoding.ASCII.GetBytes(ParamValue,0,128,buffer,index);
            index+=128;
            BitConverter.GetBytes((byte)ParamType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Total number of parameters
        /// 
        /// param_count
        /// </summary>
        public ushort ParamCount { get; set; }
        /// <summary>
        /// Index of this parameter
        /// 
        /// param_index
        /// </summary>
        public ushort ParamIndex { get; set; }
        /// <summary>
        /// Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// 
        /// param_id
        /// </summary>
        public char[] ParamId { get; } = new char[16];
        /// <summary>
        /// Parameter value
        /// 
        /// param_value
        /// </summary>
        public char[] ParamValue { get; } = new char[128];
        /// <summary>
        /// Parameter type.
        /// 
        /// param_type
        /// </summary>
        public MavParamExtType ParamType { get; set; }
    }
    /// <summary>
    /// Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when setting a parameter value and the new value is the same as the current value, you will immediately get a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive a PARAM_ACK_IN_PROGRESS in response.
    ///  PARAM_EXT_SET
    /// </summary>
    public class ParamExtSetPacket: PacketV2<ParamExtSetPayload>
    {
        public override byte CrcEtra => 78;
        public override int MessageId => 323;
        public override ParamExtSetPayload Payload { get; } = new ParamExtSetPayload();

        public override string Name => "PARAM_EXT_SET";
        public override string ToString()
        {
            var name = "PARAM_EXT_SET".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  PARAM_EXT_SET
    /// </summary>
    public class ParamExtSetPayload : IPayload
    {
        public byte ByteSize => 147;

        public int Deserialize(byte[] buffer, int index)
        {
            TargetSystem = (byte)buffer[index++];
            TargetComponent = (byte)buffer[index++];
                Encoding.ASCII.GetChars(buffer, index,16,ParamId,0);
                index+=16;
                Encoding.ASCII.GetChars(buffer, index,128,ParamValue,0);
                index+=128;
            ParamType = (MavParamExtType)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TargetSystem).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(TargetComponent).CopyTo(buffer, index);index+=1;
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);
            index+=16;
            Encoding.ASCII.GetBytes(ParamValue,0,128,buffer,index);
            index+=128;
            BitConverter.GetBytes((byte)ParamType).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// System ID
        /// 
        /// target_system
        /// </summary>
        public byte TargetSystem { get; set; }
        /// <summary>
        /// Component ID
        /// 
        /// target_component
        /// </summary>
        public byte TargetComponent { get; set; }
        /// <summary>
        /// Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// 
        /// param_id
        /// </summary>
        public char[] ParamId { get; } = new char[16];
        /// <summary>
        /// Parameter value
        /// 
        /// param_value
        /// </summary>
        public char[] ParamValue { get; } = new char[128];
        /// <summary>
        /// Parameter type.
        /// 
        /// param_type
        /// </summary>
        public MavParamExtType ParamType { get; set; }
    }
    /// <summary>
    /// Response from a PARAM_EXT_SET message.
    ///  PARAM_EXT_ACK
    /// </summary>
    public class ParamExtAckPacket: PacketV2<ParamExtAckPayload>
    {
        public override byte CrcEtra => 132;
        public override int MessageId => 324;
        public override ParamExtAckPayload Payload { get; } = new ParamExtAckPayload();

        public override string Name => "PARAM_EXT_ACK";
        public override string ToString()
        {
            var name = "PARAM_EXT_ACK".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  PARAM_EXT_ACK
    /// </summary>
    public class ParamExtAckPayload : IPayload
    {
        public byte ByteSize => 146;

        public int Deserialize(byte[] buffer, int index)
        {
                Encoding.ASCII.GetChars(buffer, index,16,ParamId,0);
                index+=16;
                Encoding.ASCII.GetChars(buffer, index,128,ParamValue,0);
                index+=128;
            ParamType = (MavParamExtType)buffer[index++];
            ParamResult = (ParamAck)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            Encoding.ASCII.GetBytes(ParamId,0,16,buffer,index);
            index+=16;
            Encoding.ASCII.GetBytes(ParamValue,0,128,buffer,index);
            index+=128;
            BitConverter.GetBytes((byte)ParamType).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)ParamResult).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// 
        /// param_id
        /// </summary>
        public char[] ParamId { get; } = new char[16];
        /// <summary>
        /// Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
        /// 
        /// param_value
        /// </summary>
        public char[] ParamValue { get; } = new char[128];
        /// <summary>
        /// Parameter type.
        /// 
        /// param_type
        /// </summary>
        public MavParamExtType ParamType { get; set; }
        /// <summary>
        /// Result code.
        /// 
        /// param_result
        /// </summary>
        public ParamAck ParamResult { get; set; }
    }
    /// <summary>
    /// Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
    ///  OBSTACLE_DISTANCE
    /// </summary>
    public class ObstacleDistancePacket: PacketV2<ObstacleDistancePayload>
    {
        public override byte CrcEtra => 23;
        public override int MessageId => 330;
        public override ObstacleDistancePayload Payload { get; } = new ObstacleDistancePayload();

        public override string Name => "OBSTACLE_DISTANCE";
        public override string ToString()
        {
            var name = "OBSTACLE_DISTANCE".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  OBSTACLE_DISTANCE
    /// </summary>
    public class ObstacleDistancePayload : IPayload
    {
        public byte ByteSize => 158;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            for(var i=0;i<72;i++)
            {
                Distances[i] = BitConverter.ToUInt16(buffer,index);index+=2;
            }
            MinDistance = BitConverter.ToUInt16(buffer,index);index+=2;
            MaxDistance = BitConverter.ToUInt16(buffer,index);index+=2;
            SensorType = (MavDistanceSensor)buffer[index++];
            Increment = (byte)buffer[index++];
            return ByteSize;
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
            BitConverter.GetBytes((byte)SensorType).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Increment).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// Distance of obstacles around the UAV with index 0 corresponding to local North. A value of 0 means that the obstacle is right in front of the sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX for unknown/not used. In a array element, one unit corresponds to 1cm.
        /// cm
        /// distances
        /// </summary>
        public ushort[] Distances { get; } = new ushort[72];
        /// <summary>
        /// Minimum distance the sensor can measure.
        /// cm
        /// min_distance
        /// </summary>
        public ushort MinDistance { get; set; }
        /// <summary>
        /// Maximum distance the sensor can measure.
        /// cm
        /// max_distance
        /// </summary>
        public ushort MaxDistance { get; set; }
        /// <summary>
        /// Class id of the distance sensor type.
        /// 
        /// sensor_type
        /// </summary>
        public MavDistanceSensor SensorType { get; set; }
        /// <summary>
        /// Angular width in degrees of each array element.
        /// deg
        /// increment
        /// </summary>
        public byte Increment { get; set; }
    }
    /// <summary>
    /// Odometry message to communicate odometry information with an external interface. Fits ROS REP 147 standard for aerial vehicles (http://www.ros.org/reps/rep-0147.html).
    ///  ODOMETRY
    /// </summary>
    public class OdometryPacket: PacketV2<OdometryPayload>
    {
        public override byte CrcEtra => 58;
        public override int MessageId => 331;
        public override OdometryPayload Payload { get; } = new OdometryPayload();

        public override string Name => "ODOMETRY";
        public override string ToString()
        {
            var name = "ODOMETRY".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  ODOMETRY
    /// </summary>
    public class OdometryPayload : IPayload
    {
        public byte ByteSize => 230;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            X = BitConverter.ToSingle(buffer, index);index+=4;
            Y = BitConverter.ToSingle(buffer, index);index+=4;
            Z = BitConverter.ToSingle(buffer, index);index+=4;
            for(var i=0;i<4;i++)
            {
                Q[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            Vx = BitConverter.ToSingle(buffer, index);index+=4;
            Vy = BitConverter.ToSingle(buffer, index);index+=4;
            Vz = BitConverter.ToSingle(buffer, index);index+=4;
            Rollspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Pitchspeed = BitConverter.ToSingle(buffer, index);index+=4;
            Yawspeed = BitConverter.ToSingle(buffer, index);index+=4;
            for(var i=0;i<21;i++)
            {
                PoseCovariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<21;i++)
            {
                TwistCovariance[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            FrameId = (MavFrame)buffer[index++];
            ChildFrameId = (MavFrame)buffer[index++];
            return ByteSize;
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
            BitConverter.GetBytes((byte)FrameId).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes((byte)ChildFrameId).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X Position
        /// m
        /// x
        /// </summary>
        public float X { get; set; }
        /// <summary>
        /// Y Position
        /// m
        /// y
        /// </summary>
        public float Y { get; set; }
        /// <summary>
        /// Z Position
        /// m
        /// z
        /// </summary>
        public float Z { get; set; }
        /// <summary>
        /// Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
        /// 
        /// q
        /// </summary>
        public float[] Q { get; } = new float[4];
        /// <summary>
        /// X linear speed
        /// m/s
        /// vx
        /// </summary>
        public float Vx { get; set; }
        /// <summary>
        /// Y linear speed
        /// m/s
        /// vy
        /// </summary>
        public float Vy { get; set; }
        /// <summary>
        /// Z linear speed
        /// m/s
        /// vz
        /// </summary>
        public float Vz { get; set; }
        /// <summary>
        /// Roll angular speed
        /// rad/s
        /// rollspeed
        /// </summary>
        public float Rollspeed { get; set; }
        /// <summary>
        /// Pitch angular speed
        /// rad/s
        /// pitchspeed
        /// </summary>
        public float Pitchspeed { get; set; }
        /// <summary>
        /// Yaw angular speed
        /// rad/s
        /// yawspeed
        /// </summary>
        public float Yawspeed { get; set; }
        /// <summary>
        /// Pose (states: x, y, z, roll, pitch, yaw) covariance matrix upper right triangle (first six entries are the first ROW, next five entries are the second ROW, etc.)
        /// 
        /// pose_covariance
        /// </summary>
        public float[] PoseCovariance { get; } = new float[21];
        /// <summary>
        /// Twist (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed) covariance matrix upper right triangle (first six entries are the first ROW, next five entries are the second ROW, etc.)
        /// 
        /// twist_covariance
        /// </summary>
        public float[] TwistCovariance { get; } = new float[21];
        /// <summary>
        /// Coordinate frame of reference for the pose data.
        /// 
        /// frame_id
        /// </summary>
        public MavFrame FrameId { get; set; }
        /// <summary>
        /// Coordinate frame of reference for the velocity in free space (twist) data.
        /// 
        /// child_frame_id
        /// </summary>
        public MavFrame ChildFrameId { get; set; }
    }
    /// <summary>
    /// Describe a trajectory using an array of up-to 5 waypoints in the local frame.
    ///  TRAJECTORY_REPRESENTATION_WAYPOINTS
    /// </summary>
    public class TrajectoryRepresentationWaypointsPacket: PacketV2<TrajectoryRepresentationWaypointsPayload>
    {
        public override byte CrcEtra => 91;
        public override int MessageId => 332;
        public override TrajectoryRepresentationWaypointsPayload Payload { get; } = new TrajectoryRepresentationWaypointsPayload();

        public override string Name => "TRAJECTORY_REPRESENTATION_WAYPOINTS";
        public override string ToString()
        {
            var name = "TRAJECTORY_REPRESENTATION_WAYPOINTS".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  TRAJECTORY_REPRESENTATION_WAYPOINTS
    /// </summary>
    public class TrajectoryRepresentationWaypointsPayload : IPayload
    {
        public byte ByteSize => 229;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            for(var i=0;i<5;i++)
            {
                PosX[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                PosY[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                PosZ[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                VelX[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                VelY[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                VelZ[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                AccX[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                AccY[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                AccZ[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                PosYaw[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                VelYaw[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            ValidPoints = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(PosX[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(PosY[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(PosZ[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(VelX[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(VelY[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(VelZ[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(AccX[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(AccY[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(AccZ[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(PosYaw[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(VelYaw[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(ValidPoints).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X-coordinate of waypoint, set to NaN if not being used
        /// m
        /// pos_x
        /// </summary>
        public float[] PosX { get; } = new float[5];
        /// <summary>
        /// Y-coordinate of waypoint, set to NaN if not being used
        /// m
        /// pos_y
        /// </summary>
        public float[] PosY { get; } = new float[5];
        /// <summary>
        /// Z-coordinate of waypoint, set to NaN if not being used
        /// m
        /// pos_z
        /// </summary>
        public float[] PosZ { get; } = new float[5];
        /// <summary>
        /// X-velocity of waypoint, set to NaN if not being used
        /// m/s
        /// vel_x
        /// </summary>
        public float[] VelX { get; } = new float[5];
        /// <summary>
        /// Y-velocity of waypoint, set to NaN if not being used
        /// m/s
        /// vel_y
        /// </summary>
        public float[] VelY { get; } = new float[5];
        /// <summary>
        /// Z-velocity of waypoint, set to NaN if not being used
        /// m/s
        /// vel_z
        /// </summary>
        public float[] VelZ { get; } = new float[5];
        /// <summary>
        /// X-acceleration of waypoint, set to NaN if not being used
        /// m/s/s
        /// acc_x
        /// </summary>
        public float[] AccX { get; } = new float[5];
        /// <summary>
        /// Y-acceleration of waypoint, set to NaN if not being used
        /// m/s/s
        /// acc_y
        /// </summary>
        public float[] AccY { get; } = new float[5];
        /// <summary>
        /// Z-acceleration of waypoint, set to NaN if not being used
        /// m/s/s
        /// acc_z
        /// </summary>
        public float[] AccZ { get; } = new float[5];
        /// <summary>
        /// Yaw angle, set to NaN if not being used
        /// rad
        /// pos_yaw
        /// </summary>
        public float[] PosYaw { get; } = new float[5];
        /// <summary>
        /// Yaw rate, set to NaN if not being used
        /// rad/s
        /// vel_yaw
        /// </summary>
        public float[] VelYaw { get; } = new float[5];
        /// <summary>
        /// Number of valid points (up-to 5 waypoints are possible)
        /// 
        /// valid_points
        /// </summary>
        public byte ValidPoints { get; set; }
    }
    /// <summary>
    /// Describe a trajectory using an array of up-to 5 bezier points in the local frame.
    ///  TRAJECTORY_REPRESENTATION_BEZIER
    /// </summary>
    public class TrajectoryRepresentationBezierPacket: PacketV2<TrajectoryRepresentationBezierPayload>
    {
        public override byte CrcEtra => 231;
        public override int MessageId => 333;
        public override TrajectoryRepresentationBezierPayload Payload { get; } = new TrajectoryRepresentationBezierPayload();

        public override string Name => "TRAJECTORY_REPRESENTATION_BEZIER";
        public override string ToString()
        {
            var name = "TRAJECTORY_REPRESENTATION_BEZIER".PadLeft(30);

            return $"{name}(" +
                   $"INC:{Convert.ToString(IncompatFlags, 2).PadLeft(8, '0')}|" +
                   $"COM:{Convert.ToString(CompatFlags, 2).PadLeft(8, '0')}|" +
                   $"SEQ:{Sequence:000}|" +
                   $"SYS:{SystemId:000}|" +
                   $"COM:{ComponenId:000}|" +
                   $"MSG:{MessageId:000000}|" +
                   $"{Payload.ToString()})";
        }
    }

    /// <summary>
    ///  TRAJECTORY_REPRESENTATION_BEZIER
    /// </summary>
    public class TrajectoryRepresentationBezierPayload : IPayload
    {
        public byte ByteSize => 109;

        public int Deserialize(byte[] buffer, int index)
        {
            TimeUsec = BitConverter.ToUInt64(buffer,index);index+=8;
            for(var i=0;i<5;i++)
            {
                PosX[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                PosY[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                PosZ[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                Delta[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                PosYaw[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            ValidPoints = (byte)buffer[index++];
            return ByteSize;
        }

        public int Serialize(byte[] buffer, int index)
        {
            BitConverter.GetBytes(TimeUsec).CopyTo(buffer, index);index+=8;
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(PosX[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(PosY[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(PosZ[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(Delta[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<5;i++)
            {
                BitConverter.GetBytes(PosYaw[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(ValidPoints).CopyTo(buffer, index);index+=1;
            return ByteSize;
        }

        /// <summary>
        /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
        /// us
        /// time_usec
        /// </summary>
        public ulong TimeUsec { get; set; }
        /// <summary>
        /// X-coordinate of starting bezier point, set to NaN if not being used
        /// m
        /// pos_x
        /// </summary>
        public float[] PosX { get; } = new float[5];
        /// <summary>
        /// Y-coordinate of starting bezier point, set to NaN if not being used
        /// m
        /// pos_y
        /// </summary>
        public float[] PosY { get; } = new float[5];
        /// <summary>
        /// Z-coordinate of starting bezier point, set to NaN if not being used
        /// m
        /// pos_z
        /// </summary>
        public float[] PosZ { get; } = new float[5];
        /// <summary>
        /// Bezier time horizon, set to NaN if velocity/acceleration should not be incorporated
        /// s
        /// delta
        /// </summary>
        public float[] Delta { get; } = new float[5];
        /// <summary>
        /// Yaw, set to NaN for unchanged
        /// rad
        /// pos_yaw
        /// </summary>
        public float[] PosYaw { get; } = new float[5];
        /// <summary>
        /// Number of valid points (up-to 5 waypoints are possible)
        /// 
        /// valid_points
        /// </summary>
        public byte ValidPoints { get; set; }
    }


#endregion


}
