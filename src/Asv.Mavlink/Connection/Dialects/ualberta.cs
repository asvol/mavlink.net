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

// This code was generate by tool Asv.Mavlink.Gen version v0.1.0-8-g06d785c

using System;
using System.Text;

namespace Asv.Mavlink.V2.Ualberta
{

    public static class UalbertaHelper
    {
        public static void RegisterUalbertaDialect(this IPacketDecoder<IPacketV2<IPayload>> src)
        {
            src.Register(()=>new NavFilterBiasPacket());
            src.Register(()=>new RadioCalibrationPacket());
            src.Register(()=>new UalbertaSysStatusPacket());
        }
    }

#region Enums

    /// <summary>
    /// Available autopilot modes for ualberta uav
    ///  UALBERTA_AUTOPILOT_MODE
    /// </summary>
    public enum UalbertaAutopilotMode
    {
        /// <summary>
        /// Raw input pulse widts sent to output
        /// MODE_MANUAL_DIRECT
        /// </summary>
        ModeManualDirect = 0,
        /// <summary>
        /// Inputs are normalized using calibration, the converted back to raw pulse widths for output
        /// MODE_MANUAL_SCALED
        /// </summary>
        ModeManualScaled = 1,
        /// <summary>
        ///  dfsdfs
        /// MODE_AUTO_PID_ATT
        /// </summary>
        ModeAutoPidAtt = 2,
        /// <summary>
        ///  dfsfds
        /// MODE_AUTO_PID_VEL
        /// </summary>
        ModeAutoPidVel = 3,
        /// <summary>
        ///  dfsdfsdfs
        /// MODE_AUTO_PID_POS
        /// </summary>
        ModeAutoPidPos = 4,
    }

    /// <summary>
    /// Navigation filter mode
    ///  UALBERTA_NAV_MODE
    /// </summary>
    public enum UalbertaNavMode
    {
        /// <summary>
        /// NAV_AHRS_INIT
        /// </summary>
        NavAhrsInit = 0,
        /// <summary>
        /// AHRS mode
        /// NAV_AHRS
        /// </summary>
        NavAhrs = 1,
        /// <summary>
        /// INS/GPS initialization mode
        /// NAV_INS_GPS_INIT
        /// </summary>
        NavInsGpsInit = 2,
        /// <summary>
        /// INS/GPS mode
        /// NAV_INS_GPS
        /// </summary>
        NavInsGps = 3,
    }

    /// <summary>
    /// Mode currently commanded by pilot
    ///  UALBERTA_PILOT_MODE
    /// </summary>
    public enum UalbertaPilotMode
    {
        /// <summary>
        ///  sdf
        /// PILOT_MANUAL
        /// </summary>
        PilotManual = 0,
        /// <summary>
        ///  dfs
        /// PILOT_AUTO
        /// </summary>
        PilotAuto = 1,
        /// <summary>
        ///  Rotomotion mode 
        /// PILOT_ROTO
        /// </summary>
        PilotRoto = 2,
    }


#endregion

#region Messages

    /// <summary>
    /// Accelerometer and Gyro biases from the navigation filter
    ///  NAV_FILTER_BIAS
    /// </summary>
    public class NavFilterBiasPacket: PacketV2<NavFilterBiasPayload>
    {
	    public const int PacketMessageId = 220;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 34;

        public override NavFilterBiasPayload Payload { get; } = new NavFilterBiasPayload();

        public override string Name => "NAV_FILTER_BIAS";
    }

    /// <summary>
    ///  NAV_FILTER_BIAS
    /// </summary>
    public class NavFilterBiasPayload : IPayload
    {
        public byte GetMaxByteSize() => 32;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Usec = BitConverter.ToUInt64(buffer,index);index+=8;
            Accel0 = BitConverter.ToSingle(buffer, index);index+=4;
            Accel1 = BitConverter.ToSingle(buffer, index);index+=4;
            Accel2 = BitConverter.ToSingle(buffer, index);index+=4;
            Gyro0 = BitConverter.ToSingle(buffer, index);index+=4;
            Gyro1 = BitConverter.ToSingle(buffer, index);index+=4;
            Gyro2 = BitConverter.ToSingle(buffer, index);index+=4;
        }

        public int Serialize(byte[] buffer, int index)
        {
		var start = index;
            BitConverter.GetBytes(Usec).CopyTo(buffer, index);index+=8;
            BitConverter.GetBytes(Accel0).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Accel1).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Accel2).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Gyro0).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Gyro1).CopyTo(buffer, index);index+=4;
            BitConverter.GetBytes(Gyro2).CopyTo(buffer, index);index+=4;
            return index - start; // /*PayloadByteSize*/32;
        }

        /// <summary>
        /// Timestamp (microseconds)
        /// OriginName: usec, Units: , IsExtended: false
        /// </summary>
        public ulong Usec { get; set; }
        /// <summary>
        /// b_f[0]
        /// OriginName: accel_0, Units: , IsExtended: false
        /// </summary>
        public float Accel0 { get; set; }
        /// <summary>
        /// b_f[1]
        /// OriginName: accel_1, Units: , IsExtended: false
        /// </summary>
        public float Accel1 { get; set; }
        /// <summary>
        /// b_f[2]
        /// OriginName: accel_2, Units: , IsExtended: false
        /// </summary>
        public float Accel2 { get; set; }
        /// <summary>
        /// b_f[0]
        /// OriginName: gyro_0, Units: , IsExtended: false
        /// </summary>
        public float Gyro0 { get; set; }
        /// <summary>
        /// b_f[1]
        /// OriginName: gyro_1, Units: , IsExtended: false
        /// </summary>
        public float Gyro1 { get; set; }
        /// <summary>
        /// b_f[2]
        /// OriginName: gyro_2, Units: , IsExtended: false
        /// </summary>
        public float Gyro2 { get; set; }
    }
    /// <summary>
    /// Complete set of calibration parameters for the radio
    ///  RADIO_CALIBRATION
    /// </summary>
    public class RadioCalibrationPacket: PacketV2<RadioCalibrationPayload>
    {
	    public const int PacketMessageId = 221;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 71;

        public override RadioCalibrationPayload Payload { get; } = new RadioCalibrationPayload();

        public override string Name => "RADIO_CALIBRATION";
    }

    /// <summary>
    ///  RADIO_CALIBRATION
    /// </summary>
    public class RadioCalibrationPayload : IPayload
    {
        public byte GetMaxByteSize() => 42;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            arraySize = 3;
            for(var i=0;i<arraySize;i++)
            {
                Aileron[i] = BitConverter.ToUInt16(buffer,index);index+=2;
            }
            arraySize = 3;
            for(var i=0;i<arraySize;i++)
            {
                Elevator[i] = BitConverter.ToUInt16(buffer,index);index+=2;
            }
            arraySize = 3;
            for(var i=0;i<arraySize;i++)
            {
                Rudder[i] = BitConverter.ToUInt16(buffer,index);index+=2;
            }
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                Gyro[i] = BitConverter.ToUInt16(buffer,index);index+=2;
            }
            arraySize = /*ArrayLength*/5 - Math.Max(0,((/*PayloadByteSize*/42 - payloadSize - /*ExtendedFieldsLength*/0)/2 /*FieldTypeByteSize*/));
            Pitch = new ushort[arraySize];
            for(var i=0;i<arraySize;i++)
            {
                Pitch[i] = BitConverter.ToUInt16(buffer,index);index+=2;
            }
            arraySize = 5;
            for(var i=0;i<arraySize;i++)
            {
                Throttle[i] = BitConverter.ToUInt16(buffer,index);index+=2;
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
		var start = index;
            for(var i=0;i<Aileron.Length;i++)
            {
                BitConverter.GetBytes(Aileron[i]).CopyTo(buffer, index);index+=2;
            }
            for(var i=0;i<Elevator.Length;i++)
            {
                BitConverter.GetBytes(Elevator[i]).CopyTo(buffer, index);index+=2;
            }
            for(var i=0;i<Rudder.Length;i++)
            {
                BitConverter.GetBytes(Rudder[i]).CopyTo(buffer, index);index+=2;
            }
            for(var i=0;i<Gyro.Length;i++)
            {
                BitConverter.GetBytes(Gyro[i]).CopyTo(buffer, index);index+=2;
            }
            for(var i=0;i<Pitch.Length;i++)
            {
                BitConverter.GetBytes(Pitch[i]).CopyTo(buffer, index);index+=2;
            }
            for(var i=0;i<Throttle.Length;i++)
            {
                BitConverter.GetBytes(Throttle[i]).CopyTo(buffer, index);index+=2;
            }
            return index - start; // /*PayloadByteSize*/42;
        }

        /// <summary>
        /// Aileron setpoints: left, center, right
        /// OriginName: aileron, Units: , IsExtended: false
        /// </summary>
        public ushort[] Aileron { get; } = new ushort[3];
        /// <summary>
        /// Elevator setpoints: nose down, center, nose up
        /// OriginName: elevator, Units: , IsExtended: false
        /// </summary>
        public ushort[] Elevator { get; } = new ushort[3];
        /// <summary>
        /// Rudder setpoints: nose left, center, nose right
        /// OriginName: rudder, Units: , IsExtended: false
        /// </summary>
        public ushort[] Rudder { get; } = new ushort[3];
        /// <summary>
        /// Tail gyro mode/gain setpoints: heading hold, rate mode
        /// OriginName: gyro, Units: , IsExtended: false
        /// </summary>
        public ushort[] Gyro { get; } = new ushort[2];
        /// <summary>
        /// Pitch curve setpoints (every 25%)
        /// OriginName: pitch, Units: , IsExtended: false
        /// </summary>
        public ushort[] Pitch { get; set; } = new ushort[5];
        public byte GetPitchMaxItemsCount() => 5;
        /// <summary>
        /// Throttle curve setpoints (every 25%)
        /// OriginName: throttle, Units: , IsExtended: false
        /// </summary>
        public ushort[] Throttle { get; } = new ushort[5];
    }
    /// <summary>
    /// System status specific to ualberta uav
    ///  UALBERTA_SYS_STATUS
    /// </summary>
    public class UalbertaSysStatusPacket: PacketV2<UalbertaSysStatusPayload>
    {
	    public const int PacketMessageId = 222;
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => 15;

        public override UalbertaSysStatusPayload Payload { get; } = new UalbertaSysStatusPayload();

        public override string Name => "UALBERTA_SYS_STATUS";
    }

    /// <summary>
    ///  UALBERTA_SYS_STATUS
    /// </summary>
    public class UalbertaSysStatusPayload : IPayload
    {
        public byte GetMaxByteSize() => 3;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            Mode = (byte)buffer[index++];
            NavMode = (byte)buffer[index++];
            Pilot = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
		var start = index;
            BitConverter.GetBytes(Mode).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(NavMode).CopyTo(buffer, index);index+=1;
            BitConverter.GetBytes(Pilot).CopyTo(buffer, index);index+=1;
            return index - start; // /*PayloadByteSize*/3;
        }

        /// <summary>
        /// System mode, see UALBERTA_AUTOPILOT_MODE ENUM
        /// OriginName: mode, Units: , IsExtended: false
        /// </summary>
        public byte Mode { get; set; }
        /// <summary>
        /// Navigation mode, see UALBERTA_NAV_MODE ENUM
        /// OriginName: nav_mode, Units: , IsExtended: false
        /// </summary>
        public byte NavMode { get; set; }
        /// <summary>
        /// Pilot mode, see UALBERTA_PILOT_MODE
        /// OriginName: pilot, Units: , IsExtended: false
        /// </summary>
        public byte Pilot { get; set; }
    }


#endregion


}
