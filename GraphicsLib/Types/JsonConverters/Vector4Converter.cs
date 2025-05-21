using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System.Numerics;

namespace GraphicsLib.Types.JsonConverters
{
    class Vector4Converter : JsonConverter<Vector4>
    {
        public override Vector4 ReadJson(JsonReader reader, Type objectType, Vector4 existingValue, bool hasExistingValue, JsonSerializer serializer)
        {
            var array = JArray.Load(reader);
            if (array.Count != 4)
            {
                throw new JsonSerializationException("Invalid Vector4 array");
            }
            return new Vector4(
                (float)array[0],
                (float)array[1],
                (float)array[2],
                (float)array[3]
            );
        }

        public override void WriteJson(JsonWriter writer, Vector4 value, JsonSerializer serializer)
        {
            writer.WriteStartArray();
            writer.WriteValue(value.X);
            writer.WriteValue(value.Y);
            writer.WriteValue(value.Z);
            writer.WriteValue(value.W);
            writer.WriteEndArray();
        }
    }
    public class Vector4NullableConverter : JsonConverter<Vector4?>
    {
        public override Vector4? ReadJson(JsonReader reader, Type objectType, Vector4? existingValue, bool hasExistingValue, JsonSerializer serializer)
        {
            var array = JArray.Load(reader);
            if (array.Count != 4)
            {
                throw new JsonSerializationException("Invalid Vector4 array");
            }
            return new Vector4(
                (float)array[0],
                (float)array[1],
                (float)array[2],
                (float)array[3]
            );
        }

        public override void WriteJson(JsonWriter writer, Vector4? value, JsonSerializer serializer)
        {
            if (value.HasValue)
            {
                Vector4 vector = value.Value;
                writer.WriteStartArray();
                writer.WriteValue(vector.X);
                writer.WriteValue(vector.Y);
                writer.WriteValue(vector.Z);
                writer.WriteValue(vector.W);
                writer.WriteEndArray();
            }
        }
    }
}
