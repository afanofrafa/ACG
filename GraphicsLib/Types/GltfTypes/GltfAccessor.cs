using Newtonsoft.Json;

namespace GraphicsLib.Types.GltfTypes
{
    public class GltfAccessor
    {
        [JsonProperty("bufferView")]
        public int? BufferViewIndex { get; set; }
        [JsonProperty("byteOffset")]
        public int ByteOffset { get; set; } = 0;
        [JsonProperty("componentType")]
        public required GltfComponentType ComponentType { get; set; }
        [JsonProperty("normalized")]
        public bool? Normalized { get; set; } = false;
        [JsonProperty("count")]
        public required int Count { get; set; }
        [JsonProperty("type")]
        public required GltfAccessorType Type { get; set; }
        [JsonProperty("max")]
        public float[]? Max { get; set; }
        [JsonProperty("min")]
        public float[]? Min { get; set; }
        [JsonProperty("sparse")]
        public GltfSparse? Sparse { get; set; }
        [JsonProperty("name")]
        public string? Name { get; set; }
        [JsonProperty("extensions")]
        public Dictionary<string, object>? Extensions { get; set; }
        [JsonProperty("extras")]
        public object? Extras { get; set; }

        [JsonIgnore]
        public GltfBufferView? BufferView { get; set; }
    }

    public class GltfSparse
    {
    }
    public enum GltfComponentType
    {
        BYTE = 5120,
        UNSIGNED_BYTE = 5121,
        SHORT = 5122,
        UNSIGNED_SHORT = 5123,
        INT = 5124,
        UNSIGNED_INT = 5125,
        FLOAT = 5126,

    }
    public enum GltfAccessorType
    {
        SCALAR,
        VEC2,
        VEC3,
        VEC4,
        MAT2,
        MAT3,
        MAT4
    }
}