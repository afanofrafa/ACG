using GraphicsLib.Types.GltfTypes;
using Newtonsoft.Json;

namespace GraphicsLib.Types.JsonConverters
{
    class GltfRootConverter : JsonConverter<GltfRoot>
    {
        private string sourcePath;
        public GltfRootConverter(string sourcePath)
        {
            this.sourcePath = sourcePath;
        }
        public override GltfRoot? ReadJson(JsonReader reader, Type objectType, GltfRoot? existingValue, bool hasExistingValue, JsonSerializer serializer)
        {
            JsonSerializerSettings settings = new JsonSerializerSettings
            {
                Converters = [.. serializer.Converters.Where(c => c != this)]
            };

            GltfRoot? gltfRoot = JsonSerializer.CreateDefault(settings).Deserialize<GltfRoot>(reader);
            if (gltfRoot is not null)
            {
                gltfRoot.SourcePath = sourcePath;
                GltfUtils.PreprocessGltfRoot(gltfRoot);
            }
            return gltfRoot;
        }

        public override void WriteJson(JsonWriter writer, GltfRoot? value, JsonSerializer serializer)
        {
            serializer.Serialize(writer, value);
        }
    }
}
