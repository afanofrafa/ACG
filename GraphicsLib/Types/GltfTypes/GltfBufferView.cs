using Newtonsoft.Json;

namespace GraphicsLib.Types.GltfTypes
{
    public class GltfBufferView
    {
        [JsonProperty("buffer")]
        public required int BufferIndex { get; set; }
        [JsonProperty("byteOffset")]
        public int ByteOffset { get; set; } = 0;
        [JsonProperty("byteLength")]
        public required int ByteLength { get; set; }
        [JsonProperty("byteStride")]
        public int? ByteStride { get; set; }
        [JsonProperty("target")]
        public GltfTargetType? Target { get; set; }
        [JsonProperty("name")]
        public string? Name { get; set; }
        [JsonProperty("extensions")]
        public Dictionary<string, object>? Extensions { get; set; }
        [JsonProperty("extras")]
        public object? Extras { get; set; }
        [JsonIgnore]
        public ArraySegment<byte> Data { get; set; } = new ArraySegment<byte>();

    }
    public enum GltfTargetType
    {
        ARRAY_BUFFER = 34962,
        ELEMENT_ARRAY_BUFFER = 34963
    }
}