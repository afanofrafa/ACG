using Newtonsoft.Json;

namespace GraphicsLib.Types.GltfTypes
{
    public class gltfScene
    {
        [JsonProperty("nodes")]
        public int[]? Nodes { get; set; }
        [JsonProperty("name")]
        public string? Name { get; set; }
        [JsonProperty("extensions")]
        public Dictionary<string, object>? Extensions { get; set; }
        [JsonProperty("extras")]
        public object? Extras { get; set; }
    }
}