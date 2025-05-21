using System;
using System.Numerics;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace GraphicsLib.Types.JsonConverters
{
    public class Vector3Converter : JsonConverter<Vector3>
    {
        public override Vector3 ReadJson(JsonReader reader, Type objectType, Vector3 existingValue, bool hasExistingValue, JsonSerializer serializer)
        {
            var array = JArray.Load(reader);
            if (array.Count != 3)
            {
                throw new JsonSerializationException("Invalid Vector3 array");
            }
            return new System.Numerics.Vector3(
                (float)array[0],
                (float)array[1],
                (float)array[2]
            );
        }

        public override void WriteJson(JsonWriter writer, Vector3 value, JsonSerializer serializer)
        {
            writer.WriteStartArray();
            writer.WriteValue(value.X);
            writer.WriteValue(value.Y);
            writer.WriteValue(value.Z);
            writer.WriteEndArray();
        }
    }
    public class Vector3NullableConverter : JsonConverter<Vector3?>
    {
        public override Vector3? ReadJson(JsonReader reader, Type objectType, Vector3? existingValue, bool hasExistingValue, JsonSerializer serializer)
        {
            var array = JArray.Load(reader);
            if (array.Count != 3)
            {
                throw new JsonSerializationException("Invalid Vector3 array");
            }
            return new System.Numerics.Vector3(
                (float)array[0],
                (float)array[1],
                (float)array[2]
            );
        }
        public override void WriteJson(JsonWriter writer, Vector3? value, JsonSerializer serializer)
        {
            if (value.HasValue)
            {
                Vector3 vector = value.Value;
                writer.WriteStartArray();
                writer.WriteValue(vector.X);
                writer.WriteValue(vector.Y);
                writer.WriteValue(vector.Z);
                writer.WriteEndArray();
            }
        }
    }
}
