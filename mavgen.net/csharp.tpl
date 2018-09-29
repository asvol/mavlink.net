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

// This code was generate by tool {{Tool}} version {{ToolVersion}}

using System;
using System.Text;

namespace Asv.Mavlink.V2.{{ Namespace }}
{

    public static class {{ Namespace }}Helper
    {
        public static void Register{{ Namespace }}Dialect(this IPacketDecoder<IPacketV2<IPayload>> src)
        {
            {%- for msg in Messages -%}
            src.Register(()=>new {{ msg.CamelCaseName }}Packet());
            {%- endfor -%}
        }
    }

#region Enums

{%- for en in Enums -%}
    /// <summary>
    {%- for line in en.Desc -%}
    /// {{ line }}
    {%- endfor -%}
    ///  {{ en.Name }}
    /// </summary>
    public enum {{ en.CamelCaseName }}
    {
    {%- for entry in en.Entries -%}
        /// <summary>
        {%- for line in entry.Desc -%}
        /// {{ line }}
        {%- endfor -%}
        {%- for param in entry.Params -%}
        /// Param {{ param.Index }} - {{ param.Desc }}
        {%- endfor -%}
        /// {{ entry.Name }}
        /// </summary>
        {{ entry.CamelCaseName }} = {{ entry.Value }},
    {%- endfor -%}
    }

{%- endfor -%}

#endregion

#region Messages

{%- for msg in Messages -%}
    /// <summary>
    {%- for line in msg.Desc -%}
    /// {{ line }}
    {%- endfor -%}
    ///  {{ msg.Name }}
    /// </summary>
    public class {{ msg.CamelCaseName }}Packet: PacketV2<{{ msg.CamelCaseName }}Payload>
    {
	    public const int PacketMessageId = {{ msg.Id }};
        public override int MessageId => PacketMessageId;
        public override byte GetCrcEtra() => {{ msg.CrcExtra }};

        public override {{ msg.CamelCaseName }}Payload Payload { get; } = new {{ msg.CamelCaseName }}Payload();

        public override string Name => "{{ msg.Name }}";
    }

    /// <summary>
    ///  {{ msg.Name }}
    /// </summary>
    public class {{ msg.CamelCaseName }}Payload : IPayload
    {
        public byte GetMaxByteSize() => {{ msg.PayloadByteSize }};

        public void Deserialize(byte[] buffer, int offset, int payloadSize)
        {
            var index = offset;
            var endIndex = offset + payloadSize;
            var arraySize = 0;
{%- for field in msg.Fields -%}
            {%- if field.IsExtended -%}
            // extended field '{{ field.CamelCaseName }}' can be empty
            if (index >= endIndex) return;
            {%- endif -%}
    {%- if field.IsEnum -%}
        {%- if field.IsArray -%}
            {%- if field.IsTheLargestArrayInMessage -%}
            arraySize = /*ArrayLength*/{{ field.ArrayLength }} - Math.Max(0,((/*PayloadByteSize*/{{ msg.PayloadByteSize }} - payloadSize - /*ExtendedFieldsLength*/{{ msg.ExtendedFieldsLength }})//*FieldTypeByteSize*/{{ field.FieldTypeByteSize }}));
            for(var i=arraySize;i<{{ field.ArrayLength }};i++)
            {
                {{ field.CamelCaseName }}[i] = ({{ field.EnumCamelCaseName }})default({{ field.Type }})
            }
            {%- else -%}
            arraySize = {{ field.ArrayLength }};
            {%- endif -%}
            for(var i=0;i<arraySize;i++)
            {
                {%- case field.Type -%}
                {%- when 'sbyte' or 'byte'-%}
                {{ field.CamelCaseName }}[i] = ({{ field.EnumCamelCaseName }})buffer[index++];
                {%- when 'short' -%}
                {{ field.CamelCaseName }}[i] = ({{ field.EnumCamelCaseName }})BitConverter.ToInt16(buffer,index);index+=2;
                {%- when 'ushort' -%}
                {{ field.CamelCaseName }}[i] = ({{ field.EnumCamelCaseName }})BitConverter.ToUInt16(buffer,index);index+=2;
                {%- when 'int' -%}
                {{ field.CamelCaseName }}[i] = ({{ field.EnumCamelCaseName }})BitConverter.ToInt32(buffer,index);index+=4;
                {%- when 'uint' -%}
                {{ field.CamelCaseName }}[i] = ({{ field.EnumCamelCaseName }})BitConverter.ToUInt32(buffer,index);index+=4;
                {%- when 'long' -%}
                {{ field.CamelCaseName }}[i] = ({{ field.EnumCamelCaseName }})BitConverter.ToInt64(buffer,index);index+=8;
                {%- when 'ulong' -%}
                {{ field.CamelCaseName }}[i] = ({{ field.EnumCamelCaseName }})BitConverter.ToUInt64(buffer,index);index+=8;
                {%- endcase -%}
            }

        {%- else -%}
            {%- case field.Type -%}
            {%- when 'sbyte' or 'byte' -%}
            {{ field.CamelCaseName }} = ({{ field.EnumCamelCaseName }})buffer[index++];
            {%- when 'short' -%}
            {{ field.CamelCaseName }} = ({{ field.EnumCamelCaseName }})BitConverter.ToInt16(buffer,index);index+=2;
            {%- when 'ushort' -%}
            {{ field.CamelCaseName }} = ({{ field.EnumCamelCaseName }})BitConverter.ToUInt16(buffer,index);index+=2;
            {%- when 'int' -%}
            {{ field.CamelCaseName }} = ({{ field.EnumCamelCaseName }})BitConverter.ToInt32(buffer,index);index+=4;
            {%- when 'uint' -%}
            {{ field.CamelCaseName }} = ({{ field.EnumCamelCaseName }})BitConverter.ToUInt32(buffer,index);index+=4;
            {%- when 'long' -%}
            {{ field.CamelCaseName }} = ({{ field.EnumCamelCaseName }})BitConverter.ToInt64(buffer,index);index+=8;
            {%- when 'ulong' -%}
            {{ field.CamelCaseName }} = ({{ field.EnumCamelCaseName }})BitConverter.ToUInt64(buffer,index);index+=8;
            {%- endcase -%}
        {%- endif -%}
    {%- else -%}
        {%- if field.IsArray -%}
            {%- if field.IsTheLargestArrayInMessage -%}
            arraySize = /*ArrayLength*/{{ field.ArrayLength }} - Math.Max(0,((/*PayloadByteSize*/{{ msg.PayloadByteSize }} - payloadSize - /*ExtendedFieldsLength*/{{ msg.ExtendedFieldsLength }})/{{ field.FieldTypeByteSize }} /*FieldTypeByteSize*/));
            for(var i=arraySize;i<{{ field.ArrayLength }};i++)
            {
                {{ field.CamelCaseName }}[i] = default({{ field.Type }});
            }
            {%- else -%}
            arraySize = {{ field.ArrayLength }};
            {%- endif -%}
            {%- if field.Type == 'char' -%}
                Encoding.ASCII.GetChars(buffer, index,arraySize,{{ field.CamelCaseName }},0);
                index+={{ field.ArrayLength }};
            {%- else -%}
            for(var i=0;i<arraySize;i++)
            {
                {%- case field.Type -%}
                {%- when 'sbyte' or 'byte' -%}
                {{ field.CamelCaseName }}[i] = ({{ field.Type }})buffer[index++];
                {%- when 'short' -%}
                {{ field.CamelCaseName }}[i] = BitConverter.ToInt16(buffer,index);index+=2;
                {%- when 'ushort' -%}
                {{ field.CamelCaseName }}[i] = BitConverter.ToUInt16(buffer,index);index+=2;
                {%- when 'int' -%}
                {{ field.CamelCaseName }}[i] = BitConverter.ToInt32(buffer,index);index+=4;
                {%- when 'uint' -%}
                {{ field.CamelCaseName }}[i] = BitConverter.ToUInt32(buffer,index);index+=4;
                {%- when 'long' -%}
                {{ field.CamelCaseName }}[i] = BitConverter.ToInt64(buffer,index);index+=8;
                {%- when 'ulong' -%}
                {{ field.CamelCaseName }}[i] = BitConverter.ToUInt64(buffer,index);index+=8;
                {%- when 'float' -%}
                {{ field.CamelCaseName }}[i] = BitConverter.ToSingle(buffer, index);index+=4;
                {%- when 'double' -%}
                {{ field.CamelCaseName }}[i] = BitConverter.ToDouble(buffer, index);index+=8;
                {%- endcase -%}
            }
            {%- endif -%}
        {%- else -%}
            {%- if field.Type == 'char' -%}
            {{ field.CamelCaseName }} = Encoding.ASCII.GetChars(buffer,index,1)[0];
            index+=1;
            {%- else -%}
            {%- case field.Type -%}
            {%- when 'sbyte' or 'byte' -%}
            {{ field.CamelCaseName }} = ({{ field.Type }})buffer[index++];
            {%- when 'short' -%}
            {{ field.CamelCaseName }} = BitConverter.ToInt16(buffer,index);index+=2;
            {%- when 'ushort' -%}
            {{ field.CamelCaseName }} = BitConverter.ToUInt16(buffer,index);index+=2;
            {%- when 'int' -%}
            {{ field.CamelCaseName }} = BitConverter.ToInt32(buffer,index);index+=4;
            {%- when 'uint' -%}
            {{ field.CamelCaseName }} = BitConverter.ToUInt32(buffer,index);index+=4;
            {%- when 'long' -%}
            {{ field.CamelCaseName }} = BitConverter.ToInt64(buffer,index);index+=8;
            {%- when 'ulong' -%}
            {{ field.CamelCaseName }} = BitConverter.ToUInt64(buffer,index);index+=8;
            {%- when 'float' -%}
            {{ field.CamelCaseName }} = BitConverter.ToSingle(buffer, index);index+=4;
            {%- when 'double' -%}
            {{ field.CamelCaseName }} = BitConverter.ToDouble(buffer, index);index+=8;
            {%- endcase -%}
            {%- endif -%}
        {%- endif -%}
    {%- endif -%}
{%- endfor -%}
        }

        public int Serialize(byte[] buffer, int index)
        {
{%- for field in msg.Fields -%}
    {%- if field.IsEnum -%}
        {%- if field.IsArray -%}
            for(var i=0;i<{{ field.ArrayLength }};i++)
            {
                {%- case field.Type -%}
                {%- when 'char' or 'double' or 'float'-%}
                ERROR => ENUM as 'char' or 'double' or 'float' ???????
                {%- when 'sbyte' or 'byte' -%}
                buffer[index] = (byte){{ field.CamelCaseName }}[i];index+={{ field.FieldTypeByteSize }};
                {%- when 'ulong' or 'long' or 'uint' or 'int' or 'ushort' or 'short' -%}
                BitConverter.GetBytes(({{ field.Type }}){{ field.CamelCaseName }}[i]).CopyTo(buffer, index);index+={{ field.FieldTypeByteSize }};
                {%- endcase -%}
            }
        {%- else -%}
            {%- case field.Type -%}
            {%- when 'char' or 'double' or 'float'-%}
            ERROR => ENUM as 'char' or 'double' or 'float' ???????
            {%- when 'sbyte' or 'byte' -%}
            buffer[index] = (byte){{ field.CamelCaseName }};index+={{ field.FieldTypeByteSize }};
            {%- when 'ulong' or 'long' or 'uint' or 'int' or 'ushort' or 'short'  -%}
            BitConverter.GetBytes(({{ field.Type }}){{ field.CamelCaseName }}).CopyTo(buffer, index);index+={{ field.FieldTypeByteSize }};
            {%- endcase -%}
        {%- endif -%}
    {%- else -%}
        {%- if field.IsArray -%}
            {%- case field.Type -%}
            {%- when 'char' -%}
            Encoding.ASCII.GetBytes({{ field.CamelCaseName }},0,{{ field.ArrayLength }},buffer,index);index+={{ field.ArrayLength }};
            {%- when 'sbyte' or 'byte' -%}
            for(var i=0;i<{{ field.ArrayLength }};i++)
            {
                buffer[index] = (byte){{ field.CamelCaseName }}[i];index+={{ field.FieldTypeByteSize }};
            }
            {%- when 'double' or 'float' or 'ulong' or 'long' or 'uint' or 'int' or 'ushort' or 'short'  -%}
            for(var i=0;i<{{ field.ArrayLength }};i++)
            {
                BitConverter.GetBytes({{ field.CamelCaseName }}[i]).CopyTo(buffer, index);index+={{ field.FieldTypeByteSize }};
            }
            {%- endcase -%}
        {%- else -%}
            BitConverter.GetBytes({{ field.CamelCaseName }}).CopyTo(buffer, index);index+={{ field.FieldTypeByteSize }};
        {%- endif -%}
    {%- endif -%}
{%- endfor -%}
            return /*PayloadByteSize*/{{ msg.PayloadByteSize }};
        }

    {%- for field in msg.Fields -%}
        /// <summary>
        {%- for line in field.Desc -%}
        /// {{ line }}
        {%- endfor -%}
        /// OriginName: {{ field.Name }}, Units: {{ field.Units }}, IsExtended: {{ field.IsExtended }}
        /// </summary>
    {%- if field.IsEnum -%}
        {%- if field.IsArray -%}
        public {{ field.EnumCamelCaseName }}[] {{ field.CamelCaseName }} { get; } = new {{ field.Type }}[{{ field.ArrayLength }}];
        {%- else -%}
        public {{ field.EnumCamelCaseName }} {{ field.CamelCaseName }} { get; set; }
       {%- endif -%}
    {%- else -%}
        {%- if field.IsArray -%}
        public {{ field.Type }}[] {{ field.CamelCaseName }} { get; } = new {{ field.Type }}[{{ field.ArrayLength }}];
        {%- else -%}
        public {{ field.Type }} {{ field.CamelCaseName }} { get; set; }
        {%- endif -%}
    {%- endif -%}
    {%- endfor -%}
    }
{%- endfor -%}


#endregion


}
