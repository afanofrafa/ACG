using Newtonsoft.Json.Linq;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace GraphicsLib.Types.JsonConverters
{
    public class Vector2Converter : JsonConverter<Vector2>
    {
        public override Vector2 ReadJson(JsonReader reader, Type objectType, Vector2 existingValue, bool hasExistingValue, JsonSerializer serializer)
        {
            var array = JArray.Load(reader);
            if (array.Count != 3)
            {
                throw new JsonSerializationException("Invalid Vector3 array");
            }
            return new Vector2(
                (float)array[0],
                (float)array[1]
            );
        }

        public override void WriteJson(JsonWriter writer, Vector2 value, JsonSerializer serializer)
        {
            writer.WriteStartArray();
            writer.WriteValue(value.X);
            writer.WriteValue(value.Y);
            writer.WriteEndArray();
        }
    }
    public class Vector2NullableConverter : JsonConverter<Vector2?>
    {
        public override Vector2? ReadJson(JsonReader reader, Type objectType, Vector2? existingValue, bool hasExistingValue, JsonSerializer serializer)
        {
            var array = JArray.Load(reader);
            if (array.Count != 3)
            {
                throw new JsonSerializationException("Invalid Vector3 array");
            }
            return new Vector2(
                (float)array[0],
                (float)array[1]
            );
        }
        public override void WriteJson(JsonWriter writer, Vector2? value, JsonSerializer serializer)
        {
            if (value.HasValue)
            {
                Vector2 vector = value.Value;
                writer.WriteStartArray();
                writer.WriteValue(vector.X);
                writer.WriteValue(vector.Y);
                writer.WriteEndArray();
            }

        }
    }
}
