using System.Text.Json.Serialization;

namespace GraphicsLib.Types.GltfTypes
{
    public class GltfRoot : IDisposable
    {
        [JsonPropertyName("extensionsUsed")]
        public List<string>? ExtensionsUsed { get; set; }

        [JsonPropertyName("extensionsRequired")]
        public List<string>? ExtensionsRequired { get; set; }

        [JsonPropertyName("accessors")]
        public List<GltfAccessor>? Accessors { get; set; }

        [JsonPropertyName("animations")]
        public List<GltfAnimation>? Animations { get; set; }

        [JsonPropertyName("asset")]
        public required GltfAsset Asset { get; set; }

        [JsonPropertyName("buffers")]
        public List<GltfBuffer>? Buffers { get; set; }

        [JsonPropertyName("bufferViews")]
        public List<GltfBufferView>? BufferViews { get; set; }

        [JsonPropertyName("cameras")]
        public List<GltfCamera>? Cameras { get; set; }

        [JsonPropertyName("images")]
        public List<GltfImage>? Images { get; set; }

        [JsonPropertyName("materials")]
        public List<GltfMaterial>? Materials { get; set; }

        [JsonPropertyName("meshes")]
        public List<GltfMesh>? Meshes { get; set; }

        [JsonPropertyName("nodes")]
        public List<GltfNode>? Nodes { get; set; }

        [JsonPropertyName("samplers")]
        public List<GltfSampler>? Samplers { get; set; }

        [JsonPropertyName("scene")]
        public int? Scene { get; set; }

        [JsonPropertyName("scenes")]
        public List<gltfScene>? Scenes { get; set; }

        [JsonPropertyName("skins")]
        public List<GltfSkin>? Skins { get; set; }

        [JsonPropertyName("textures")]
        public List<GltfTexture>? Textures { get; set; }

        [JsonPropertyName("extensions")]
        public Dictionary<string, object>? Extensions { get; set; }

        [JsonPropertyName("extras")]
        public object? Extras { get; set; }

        [JsonIgnore]
        public string? SourcePath { get; set; }

        public void Dispose()
        {
            if (Images != null)
            {
                foreach (var image in Images)
                {
                    image.Dispose();
                }
            }
            GC.SuppressFinalize(this);
        }
    }
}
