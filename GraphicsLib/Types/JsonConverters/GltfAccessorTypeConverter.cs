using GraphicsLib.Types.GltfTypes;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GraphicsLib.Types.JsonConverters
{
    public class GltfAccessorTypeConverter : JsonConverter<GltfAccessorType>
    {
        public override GltfAccessorType ReadJson(JsonReader reader, Type objectType, GltfAccessorType existingValue, bool hasExistingValue, JsonSerializer serializer)
        {
            var accessorType = reader.Value?.ToString() 
                ?? throw new JsonSerializationException("Invalid accessor type");
            return Enum.Parse<GltfAccessorType>(accessorType, true);
        }

        public override void WriteJson(JsonWriter writer, GltfAccessorType value, JsonSerializer serializer)
        {
            writer.WriteValue(value.ToString());
        }
    }
}
