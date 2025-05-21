using GraphicsLib.Types.GltfTypes;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GraphicsLib.Types.JsonConverters
{
    class GltfMaterialAlphaModeConverter : JsonConverter<GltfMaterialAlphaMode>
    {
        public override GltfMaterialAlphaMode ReadJson(JsonReader reader, Type objectType, GltfMaterialAlphaMode existingValue, bool hasExistingValue, JsonSerializer serializer)
        {
            var accessorType = reader.Value?.ToString()
                           ?? throw new JsonSerializationException("Invalid accessor type");
            return Enum.Parse<GltfMaterialAlphaMode>(accessorType, true);
        }

        public override void WriteJson(JsonWriter writer, GltfMaterialAlphaMode value, JsonSerializer serializer)
        {
            writer.WriteValue(value.ToString());
        }
    }
}
