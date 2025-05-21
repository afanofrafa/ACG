using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System.Numerics;

namespace GraphicsLib.Types.JsonConverters
{
    public class Matrix4x4Converter : JsonConverter<Matrix4x4>
    {
        public override Matrix4x4 ReadJson(JsonReader reader, Type objectType, Matrix4x4 existingValue, bool hasExistingValue, JsonSerializer serializer)
        {
            var array = JArray.Load(reader);
            if (array.Count != 16)
            {
                throw new JsonSerializationException("Invalid Matrix4x4 array");
            }

            return new Matrix4x4(
                (float)array[0], (float)array[1], (float)array[2], (float)array[3],
                (float)array[4], (float)array[5], (float)array[6], (float)array[7],
                (float)array[8], (float)array[9], (float)array[10], (float)array[11],
                (float)array[12], (float)array[13], (float)array[14], (float)array[15]
            );
        }

        public override void WriteJson(JsonWriter writer, Matrix4x4 value, JsonSerializer serializer)
        {
            writer.WriteStartArray();
            writer.WriteValue(value.M11);
            writer.WriteValue(value.M12);
            writer.WriteValue(value.M13);
            writer.WriteValue(value.M14);
            writer.WriteValue(value.M21);
            writer.WriteValue(value.M22);
            writer.WriteValue(value.M23);
            writer.WriteValue(value.M24);
            writer.WriteValue(value.M31);
            writer.WriteValue(value.M32);
            writer.WriteValue(value.M33);
            writer.WriteValue(value.M34);
            writer.WriteValue(value.M41);
            writer.WriteValue(value.M42);
            writer.WriteValue(value.M43);
            writer.WriteValue(value.M44);
            writer.WriteEndArray();
        }
    }
    public class Matrix4x4NullableConverter : JsonConverter<Matrix4x4?>
    {
        public override Matrix4x4? ReadJson(JsonReader reader, Type objectType, Matrix4x4? existingValue, bool hasExistingValue, JsonSerializer serializer)
        {
            var array = JArray.Load(reader);
            if (array.Count != 16)
            {
                throw new JsonSerializationException("Invalid Matrix4x4 array");
            }

            return new Matrix4x4(
                (float)array[0], (float)array[1], (float)array[2], (float)array[3],
                (float)array[4], (float)array[5], (float)array[6], (float)array[7],
                (float)array[8], (float)array[9], (float)array[10], (float)array[11],
                (float)array[12], (float)array[13], (float)array[14], (float)array[15]
            );
        }


        public override void WriteJson(JsonWriter writer, Matrix4x4? value, JsonSerializer serializer)
        {
            if (value.HasValue)
            {
                Matrix4x4 matrix = value.Value;
                writer.WriteStartArray();
                writer.WriteValue(matrix.M11);
                writer.WriteValue(matrix.M12);
                writer.WriteValue(matrix.M13);
                writer.WriteValue(matrix.M14);
                writer.WriteValue(matrix.M21);
                writer.WriteValue(matrix.M22);
                writer.WriteValue(matrix.M23);
                writer.WriteValue(matrix.M24);
                writer.WriteValue(matrix.M31);
                writer.WriteValue(matrix.M32);
                writer.WriteValue(matrix.M33);
                writer.WriteValue(matrix.M34);
                writer.WriteValue(matrix.M41);
                writer.WriteValue(matrix.M42);
                writer.WriteValue(matrix.M43);
                writer.WriteValue(matrix.M44);
                writer.WriteEndArray();
            }

        }
    }
}