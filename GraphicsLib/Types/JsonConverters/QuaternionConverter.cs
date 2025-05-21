using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System.Numerics;
namespace GraphicsLib.Types.JsonConverters
{
    public class QuaternionConverter : JsonConverter<Quaternion>
    {
        public override Quaternion ReadJson(JsonReader reader, Type objectType, Quaternion existingValue, bool hasExistingValue, JsonSerializer serializer)
        {
            var array = JArray.Load(reader);
            if (array.Count != 4)
            {
                throw new JsonSerializationException("Invalid Quaternion array");
            }

            return new Quaternion(
                (float)array[0],
                (float)array[1],
                (float)array[2],
                (float)array[3]
            );
        }

        public override void WriteJson(JsonWriter writer, Quaternion value, JsonSerializer serializer)
        {
            writer.WriteStartArray();
            writer.WriteValue(value.X);
            writer.WriteValue(value.Y);
            writer.WriteValue(value.Z);
            writer.WriteValue(value.W);
            writer.WriteEndArray();
        }
    }
    public class QuaternioNullableConverter : JsonConverter<Quaternion?>
    {
        public override Quaternion? ReadJson(JsonReader reader, Type objectType, Quaternion? existingValue, bool hasExistingValue, JsonSerializer serializer)
        {
            var array = JArray.Load(reader);
            if (array.Count != 4)
            {
                throw new JsonSerializationException("Invalid Quaternion array");
            }

            return new Quaternion(
                (float)array[0],
                (float)array[1],
                (float)array[2],
                (float)array[3]
            );
        }

        public override void WriteJson(JsonWriter writer, Quaternion? value, JsonSerializer serializer)
        {
            if (value.HasValue)
            {
                Quaternion quaternion = value.Value;
                writer.WriteStartArray();
                writer.WriteValue(quaternion.X);
                writer.WriteValue(quaternion.Y);
                writer.WriteValue(quaternion.Z);
                writer.WriteValue(quaternion.W);
                writer.WriteEndArray();
            }

        }
    }
}