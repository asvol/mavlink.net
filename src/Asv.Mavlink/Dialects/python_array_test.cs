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

namespace Asv.Mavlink.V2.PythonArrayTest
{

    public static class PythonArrayTestHelper
    {
        public static void RegisterPythonArrayTestDialect(this IPacketDecoder<IPacketV2<IPayload>> src)
        {
            src.Register(()=>new ArrayTest0Packet());
            src.Register(()=>new ArrayTest1Packet());
            src.Register(()=>new ArrayTest3Packet());
            src.Register(()=>new ArrayTest4Packet());
            src.Register(()=>new ArrayTest5Packet());
            src.Register(()=>new ArrayTest6Packet());
            src.Register(()=>new ArrayTest7Packet());
            src.Register(()=>new ArrayTest8Packet());
        }
    }

#region Enums


#endregion

#region Messages

    /// <summary>
    /// Array test #0.
    ///  ARRAY_TEST_0
    /// </summary>
    public class ArrayTest0Packet: PacketV2<ArrayTest0Payload>
    {
	public const int PacketMessageId = 150;
        public override int MessageId => PacketMessageId;
        public override byte CrcEtra => 26;

        public override ArrayTest0Payload Payload { get; } = new ArrayTest0Payload();

        public override string Name => "ARRAY_TEST_0";
        public override string ToString()
        {
            var name = "ARRAY_TEST_0".PadRight(30);

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
    ///  ARRAY_TEST_0
    /// </summary>
    public class ArrayTest0Payload : IPayload
    {
        public byte MaxByteSize => 33;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            arraySize = /*ArrayLength*/4 - Math.Max(0,((MaxByteSize - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<4;i++)
            {
                ArU32[i] = default(uint);
            }
            for(var i=0;i<arraySize;i++)
            {
                ArU32[i] = BitConverter.ToUInt32(buffer,index);index+=4;
            }
            arraySize = 4;
            for(var i=0;i<arraySize;i++)
            {
                ArU16[i] = BitConverter.ToUInt16(buffer,index);index+=2;
            }
            V1 = (byte)buffer[index++];
            arraySize = 4;
            for(var i=0;i<arraySize;i++)
            {
                ArI8[i] = (sbyte)buffer[index++];
            }
            arraySize = 4;
            for(var i=0;i<arraySize;i++)
            {
                ArU8[i] = (byte)buffer[index++];
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(ArU32[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(ArU16[i]).CopyTo(buffer, index);index+=2;
            }
            BitConverter.GetBytes(V1).CopyTo(buffer, index);index+=1;
            for(var i=0;i<4;i++)
            {
                buffer[index] = (byte)ArI8[i];index+=1;
            }
            for(var i=0;i<4;i++)
            {
                buffer[index] = (byte)ArU8[i];index+=1;
            }
            return MaxByteSize;
        }

        /// <summary>
        /// Value array
        /// OriginName: ar_u32, Units: , IsExtended: false
        /// </summary>
        public uint[] ArU32 { get; } = new uint[4];
        /// <summary>
        /// Value array
        /// OriginName: ar_u16, Units: , IsExtended: false
        /// </summary>
        public ushort[] ArU16 { get; } = new ushort[4];
        /// <summary>
        /// Stub field
        /// OriginName: v1, Units: , IsExtended: false
        /// </summary>
        public byte V1 { get; set; }
        /// <summary>
        /// Value array
        /// OriginName: ar_i8, Units: , IsExtended: false
        /// </summary>
        public sbyte[] ArI8 { get; } = new sbyte[4];
        /// <summary>
        /// Value array
        /// OriginName: ar_u8, Units: , IsExtended: false
        /// </summary>
        public byte[] ArU8 { get; } = new byte[4];
    }
    /// <summary>
    /// Array test #1.
    ///  ARRAY_TEST_1
    /// </summary>
    public class ArrayTest1Packet: PacketV2<ArrayTest1Payload>
    {
	public const int PacketMessageId = 151;
        public override int MessageId => PacketMessageId;
        public override byte CrcEtra => 72;

        public override ArrayTest1Payload Payload { get; } = new ArrayTest1Payload();

        public override string Name => "ARRAY_TEST_1";
        public override string ToString()
        {
            var name = "ARRAY_TEST_1".PadRight(30);

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
    ///  ARRAY_TEST_1
    /// </summary>
    public class ArrayTest1Payload : IPayload
    {
        public byte MaxByteSize => 16;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            arraySize = /*ArrayLength*/4 - Math.Max(0,((MaxByteSize - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<4;i++)
            {
                ArU32[i] = default(uint);
            }
            for(var i=0;i<arraySize;i++)
            {
                ArU32[i] = BitConverter.ToUInt32(buffer,index);index+=4;
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(ArU32[i]).CopyTo(buffer, index);index+=4;
            }
            return MaxByteSize;
        }

        /// <summary>
        /// Value array
        /// OriginName: ar_u32, Units: , IsExtended: false
        /// </summary>
        public uint[] ArU32 { get; } = new uint[4];
    }
    /// <summary>
    /// Array test #3.
    ///  ARRAY_TEST_3
    /// </summary>
    public class ArrayTest3Packet: PacketV2<ArrayTest3Payload>
    {
	public const int PacketMessageId = 153;
        public override int MessageId => PacketMessageId;
        public override byte CrcEtra => 19;

        public override ArrayTest3Payload Payload { get; } = new ArrayTest3Payload();

        public override string Name => "ARRAY_TEST_3";
        public override string ToString()
        {
            var name = "ARRAY_TEST_3".PadRight(30);

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
    ///  ARRAY_TEST_3
    /// </summary>
    public class ArrayTest3Payload : IPayload
    {
        public byte MaxByteSize => 17;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            arraySize = /*ArrayLength*/4 - Math.Max(0,((MaxByteSize - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<4;i++)
            {
                ArU32[i] = default(uint);
            }
            for(var i=0;i<arraySize;i++)
            {
                ArU32[i] = BitConverter.ToUInt32(buffer,index);index+=4;
            }
            V = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(ArU32[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(V).CopyTo(buffer, index);index+=1;
            return MaxByteSize;
        }

        /// <summary>
        /// Value array
        /// OriginName: ar_u32, Units: , IsExtended: false
        /// </summary>
        public uint[] ArU32 { get; } = new uint[4];
        /// <summary>
        /// Stub field
        /// OriginName: v, Units: , IsExtended: false
        /// </summary>
        public byte V { get; set; }
    }
    /// <summary>
    /// Array test #4.
    ///  ARRAY_TEST_4
    /// </summary>
    public class ArrayTest4Packet: PacketV2<ArrayTest4Payload>
    {
	public const int PacketMessageId = 154;
        public override int MessageId => PacketMessageId;
        public override byte CrcEtra => 89;

        public override ArrayTest4Payload Payload { get; } = new ArrayTest4Payload();

        public override string Name => "ARRAY_TEST_4";
        public override string ToString()
        {
            var name = "ARRAY_TEST_4".PadRight(30);

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
    ///  ARRAY_TEST_4
    /// </summary>
    public class ArrayTest4Payload : IPayload
    {
        public byte MaxByteSize => 17;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            arraySize = /*ArrayLength*/4 - Math.Max(0,((MaxByteSize - payloadSize - /*ExtendedFieldsLength*/0)/4 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<4;i++)
            {
                ArU32[i] = default(uint);
            }
            for(var i=0;i<arraySize;i++)
            {
                ArU32[i] = BitConverter.ToUInt32(buffer,index);index+=4;
            }
            V = (byte)buffer[index++];
        }

        public int Serialize(byte[] buffer, int index)
        {
            for(var i=0;i<4;i++)
            {
                BitConverter.GetBytes(ArU32[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(V).CopyTo(buffer, index);index+=1;
            return MaxByteSize;
        }

        /// <summary>
        /// Value array
        /// OriginName: ar_u32, Units: , IsExtended: false
        /// </summary>
        public uint[] ArU32 { get; } = new uint[4];
        /// <summary>
        /// Stub field
        /// OriginName: v, Units: , IsExtended: false
        /// </summary>
        public byte V { get; set; }
    }
    /// <summary>
    /// Array test #5.
    ///  ARRAY_TEST_5
    /// </summary>
    public class ArrayTest5Packet: PacketV2<ArrayTest5Payload>
    {
	public const int PacketMessageId = 155;
        public override int MessageId => PacketMessageId;
        public override byte CrcEtra => 27;

        public override ArrayTest5Payload Payload { get; } = new ArrayTest5Payload();

        public override string Name => "ARRAY_TEST_5";
        public override string ToString()
        {
            var name = "ARRAY_TEST_5".PadRight(30);

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
    ///  ARRAY_TEST_5
    /// </summary>
    public class ArrayTest5Payload : IPayload
    {
        public byte MaxByteSize => 10;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            arraySize = /*ArrayLength*/5 - Math.Max(0,((MaxByteSize - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<5;i++)
            {
                C1[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,C1,0);
                index+=5;
            arraySize = 5;
                Encoding.ASCII.GetChars(buffer, index,arraySize,C2,0);
                index+=5;
        }

        public int Serialize(byte[] buffer, int index)
        {
            Encoding.ASCII.GetBytes(C1,0,5,buffer,index);index+=5;
            Encoding.ASCII.GetBytes(C2,0,5,buffer,index);index+=5;
            return MaxByteSize;
        }

        /// <summary>
        /// Value array
        /// OriginName: c1, Units: , IsExtended: false
        /// </summary>
        public char[] C1 { get; } = new char[5];
        /// <summary>
        /// Value array
        /// OriginName: c2, Units: , IsExtended: false
        /// </summary>
        public char[] C2 { get; } = new char[5];
    }
    /// <summary>
    /// Array test #6.
    ///  ARRAY_TEST_6
    /// </summary>
    public class ArrayTest6Packet: PacketV2<ArrayTest6Payload>
    {
	public const int PacketMessageId = 156;
        public override int MessageId => PacketMessageId;
        public override byte CrcEtra => 14;

        public override ArrayTest6Payload Payload { get; } = new ArrayTest6Payload();

        public override string Name => "ARRAY_TEST_6";
        public override string ToString()
        {
            var name = "ARRAY_TEST_6".PadRight(30);

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
    ///  ARRAY_TEST_6
    /// </summary>
    public class ArrayTest6Payload : IPayload
    {
        public byte MaxByteSize => 91;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArD[i] = BitConverter.ToDouble(buffer, index);index+=8;
            }
            V3 = BitConverter.ToUInt32(buffer,index);index+=4;
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArU32[i] = BitConverter.ToUInt32(buffer,index);index+=4;
            }
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArI32[i] = BitConverter.ToInt32(buffer,index);index+=4;
            }
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArF[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            V2 = BitConverter.ToUInt16(buffer,index);index+=2;
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArU16[i] = BitConverter.ToUInt16(buffer,index);index+=2;
            }
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArI16[i] = BitConverter.ToInt16(buffer,index);index+=2;
            }
            V1 = (byte)buffer[index++];
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArU8[i] = (byte)buffer[index++];
            }
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArI8[i] = (sbyte)buffer[index++];
            }
            arraySize = /*ArrayLength*/32 - Math.Max(0,((MaxByteSize - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<32;i++)
            {
                ArC[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,ArC,0);
                index+=32;
        }

        public int Serialize(byte[] buffer, int index)
        {
            for(var i=0;i<2;i++)
            {
                BitConverter.GetBytes(ArD[i]).CopyTo(buffer, index);index+=8;
            }
            BitConverter.GetBytes(V3).CopyTo(buffer, index);index+=4;
            for(var i=0;i<2;i++)
            {
                BitConverter.GetBytes(ArU32[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<2;i++)
            {
                BitConverter.GetBytes(ArI32[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<2;i++)
            {
                BitConverter.GetBytes(ArF[i]).CopyTo(buffer, index);index+=4;
            }
            BitConverter.GetBytes(V2).CopyTo(buffer, index);index+=2;
            for(var i=0;i<2;i++)
            {
                BitConverter.GetBytes(ArU16[i]).CopyTo(buffer, index);index+=2;
            }
            for(var i=0;i<2;i++)
            {
                BitConverter.GetBytes(ArI16[i]).CopyTo(buffer, index);index+=2;
            }
            BitConverter.GetBytes(V1).CopyTo(buffer, index);index+=1;
            for(var i=0;i<2;i++)
            {
                buffer[index] = (byte)ArU8[i];index+=1;
            }
            for(var i=0;i<2;i++)
            {
                buffer[index] = (byte)ArI8[i];index+=1;
            }
            Encoding.ASCII.GetBytes(ArC,0,32,buffer,index);index+=32;
            return MaxByteSize;
        }

        /// <summary>
        /// Value array
        /// OriginName: ar_d, Units: , IsExtended: false
        /// </summary>
        public double[] ArD { get; } = new double[2];
        /// <summary>
        /// Stub field
        /// OriginName: v3, Units: , IsExtended: false
        /// </summary>
        public uint V3 { get; set; }
        /// <summary>
        /// Value array
        /// OriginName: ar_u32, Units: , IsExtended: false
        /// </summary>
        public uint[] ArU32 { get; } = new uint[2];
        /// <summary>
        /// Value array
        /// OriginName: ar_i32, Units: , IsExtended: false
        /// </summary>
        public int[] ArI32 { get; } = new int[2];
        /// <summary>
        /// Value array
        /// OriginName: ar_f, Units: , IsExtended: false
        /// </summary>
        public float[] ArF { get; } = new float[2];
        /// <summary>
        /// Stub field
        /// OriginName: v2, Units: , IsExtended: false
        /// </summary>
        public ushort V2 { get; set; }
        /// <summary>
        /// Value array
        /// OriginName: ar_u16, Units: , IsExtended: false
        /// </summary>
        public ushort[] ArU16 { get; } = new ushort[2];
        /// <summary>
        /// Value array
        /// OriginName: ar_i16, Units: , IsExtended: false
        /// </summary>
        public short[] ArI16 { get; } = new short[2];
        /// <summary>
        /// Stub field
        /// OriginName: v1, Units: , IsExtended: false
        /// </summary>
        public byte V1 { get; set; }
        /// <summary>
        /// Value array
        /// OriginName: ar_u8, Units: , IsExtended: false
        /// </summary>
        public byte[] ArU8 { get; } = new byte[2];
        /// <summary>
        /// Value array
        /// OriginName: ar_i8, Units: , IsExtended: false
        /// </summary>
        public sbyte[] ArI8 { get; } = new sbyte[2];
        /// <summary>
        /// Value array
        /// OriginName: ar_c, Units: , IsExtended: false
        /// </summary>
        public char[] ArC { get; } = new char[32];
    }
    /// <summary>
    /// Array test #7.
    ///  ARRAY_TEST_7
    /// </summary>
    public class ArrayTest7Packet: PacketV2<ArrayTest7Payload>
    {
	public const int PacketMessageId = 157;
        public override int MessageId => PacketMessageId;
        public override byte CrcEtra => 187;

        public override ArrayTest7Payload Payload { get; } = new ArrayTest7Payload();

        public override string Name => "ARRAY_TEST_7";
        public override string ToString()
        {
            var name = "ARRAY_TEST_7".PadRight(30);

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
    ///  ARRAY_TEST_7
    /// </summary>
    public class ArrayTest7Payload : IPayload
    {
        public byte MaxByteSize => 84;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArD[i] = BitConverter.ToDouble(buffer, index);index+=8;
            }
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArF[i] = BitConverter.ToSingle(buffer, index);index+=4;
            }
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArU32[i] = BitConverter.ToUInt32(buffer,index);index+=4;
            }
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArI32[i] = BitConverter.ToInt32(buffer,index);index+=4;
            }
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArU16[i] = BitConverter.ToUInt16(buffer,index);index+=2;
            }
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArI16[i] = BitConverter.ToInt16(buffer,index);index+=2;
            }
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArU8[i] = (byte)buffer[index++];
            }
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArI8[i] = (sbyte)buffer[index++];
            }
            arraySize = /*ArrayLength*/32 - Math.Max(0,((MaxByteSize - payloadSize - /*ExtendedFieldsLength*/0)/1 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<32;i++)
            {
                ArC[i] = default(char);
            }
                Encoding.ASCII.GetChars(buffer, index,arraySize,ArC,0);
                index+=32;
        }

        public int Serialize(byte[] buffer, int index)
        {
            for(var i=0;i<2;i++)
            {
                BitConverter.GetBytes(ArD[i]).CopyTo(buffer, index);index+=8;
            }
            for(var i=0;i<2;i++)
            {
                BitConverter.GetBytes(ArF[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<2;i++)
            {
                BitConverter.GetBytes(ArU32[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<2;i++)
            {
                BitConverter.GetBytes(ArI32[i]).CopyTo(buffer, index);index+=4;
            }
            for(var i=0;i<2;i++)
            {
                BitConverter.GetBytes(ArU16[i]).CopyTo(buffer, index);index+=2;
            }
            for(var i=0;i<2;i++)
            {
                BitConverter.GetBytes(ArI16[i]).CopyTo(buffer, index);index+=2;
            }
            for(var i=0;i<2;i++)
            {
                buffer[index] = (byte)ArU8[i];index+=1;
            }
            for(var i=0;i<2;i++)
            {
                buffer[index] = (byte)ArI8[i];index+=1;
            }
            Encoding.ASCII.GetBytes(ArC,0,32,buffer,index);index+=32;
            return MaxByteSize;
        }

        /// <summary>
        /// Value array
        /// OriginName: ar_d, Units: , IsExtended: false
        /// </summary>
        public double[] ArD { get; } = new double[2];
        /// <summary>
        /// Value array
        /// OriginName: ar_f, Units: , IsExtended: false
        /// </summary>
        public float[] ArF { get; } = new float[2];
        /// <summary>
        /// Value array
        /// OriginName: ar_u32, Units: , IsExtended: false
        /// </summary>
        public uint[] ArU32 { get; } = new uint[2];
        /// <summary>
        /// Value array
        /// OriginName: ar_i32, Units: , IsExtended: false
        /// </summary>
        public int[] ArI32 { get; } = new int[2];
        /// <summary>
        /// Value array
        /// OriginName: ar_u16, Units: , IsExtended: false
        /// </summary>
        public ushort[] ArU16 { get; } = new ushort[2];
        /// <summary>
        /// Value array
        /// OriginName: ar_i16, Units: , IsExtended: false
        /// </summary>
        public short[] ArI16 { get; } = new short[2];
        /// <summary>
        /// Value array
        /// OriginName: ar_u8, Units: , IsExtended: false
        /// </summary>
        public byte[] ArU8 { get; } = new byte[2];
        /// <summary>
        /// Value array
        /// OriginName: ar_i8, Units: , IsExtended: false
        /// </summary>
        public sbyte[] ArI8 { get; } = new sbyte[2];
        /// <summary>
        /// Value array
        /// OriginName: ar_c, Units: , IsExtended: false
        /// </summary>
        public char[] ArC { get; } = new char[32];
    }
    /// <summary>
    /// Array test #8.
    ///  ARRAY_TEST_8
    /// </summary>
    public class ArrayTest8Packet: PacketV2<ArrayTest8Payload>
    {
	public const int PacketMessageId = 158;
        public override int MessageId => PacketMessageId;
        public override byte CrcEtra => 106;

        public override ArrayTest8Payload Payload { get; } = new ArrayTest8Payload();

        public override string Name => "ARRAY_TEST_8";
        public override string ToString()
        {
            var name = "ARRAY_TEST_8".PadRight(30);

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
    ///  ARRAY_TEST_8
    /// </summary>
    public class ArrayTest8Payload : IPayload
    {
        public byte MaxByteSize => 24;

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
            arraySize = /*ArrayLength*/2 - Math.Max(0,((MaxByteSize - payloadSize - /*ExtendedFieldsLength*/0)/8 /*FieldTypeByteSize*/));
            for(var i=arraySize;i<2;i++)
            {
                ArD[i] = default(double);
            }
            for(var i=0;i<arraySize;i++)
            {
                ArD[i] = BitConverter.ToDouble(buffer, index);index+=8;
            }
            V3 = BitConverter.ToUInt32(buffer,index);index+=4;
            arraySize = 2;
            for(var i=0;i<arraySize;i++)
            {
                ArU16[i] = BitConverter.ToUInt16(buffer,index);index+=2;
            }
        }

        public int Serialize(byte[] buffer, int index)
        {
            for(var i=0;i<2;i++)
            {
                BitConverter.GetBytes(ArD[i]).CopyTo(buffer, index);index+=8;
            }
            BitConverter.GetBytes(V3).CopyTo(buffer, index);index+=4;
            for(var i=0;i<2;i++)
            {
                BitConverter.GetBytes(ArU16[i]).CopyTo(buffer, index);index+=2;
            }
            return MaxByteSize;
        }

        /// <summary>
        /// Value array
        /// OriginName: ar_d, Units: , IsExtended: false
        /// </summary>
        public double[] ArD { get; } = new double[2];
        /// <summary>
        /// Stub field
        /// OriginName: v3, Units: , IsExtended: false
        /// </summary>
        public uint V3 { get; set; }
        /// <summary>
        /// Value array
        /// OriginName: ar_u16, Units: , IsExtended: false
        /// </summary>
        public ushort[] ArU16 { get; } = new ushort[2];
    }


#endregion


}
