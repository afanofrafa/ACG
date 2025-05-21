using Newtonsoft.Json;

namespace GraphicsLib.Types.GltfTypes
{
    public class GltfSampler
    {
        [JsonProperty("magFilter")]
        public MagnificationFilterMode? MagFilter { get; set; }

        [JsonProperty("minFilter")]
        public MinificationFilterMode? MinFilter { get; set; }

        [JsonProperty("wrapS")]
        public WrappingMode WrapS { get; set; } = WrappingMode.Repeat;

        [JsonProperty("wrapT")]
        public WrappingMode WrapT { get; set; } = WrappingMode.Repeat;

        [JsonProperty("name")]
        public string? Name { get; set; }
        [JsonProperty("extensions")]
        public Dictionary<string, object>? Extensions { get; set; }
        [JsonProperty("extras")]
        public object? Extras { get; set; }
        public Sampler GetSampler()
        {
            return new Sampler(WrapS, WrapT,
                MagFilter ?? MagnificationFilterMode.Nearest,
                MinFilter ?? MinificationFilterMode.Nearest);
        }
    }
    public enum WrappingMode
    {
        ClampToEdge = 33071,
        MirroredRepeat = 33648,
        Repeat = 10497
    }
    public enum MagnificationFilterMode
    {
        Nearest = 9728,
        Linear = 9729
    }
    public enum MinificationFilterMode
    {
        Nearest = 9728,
        Linear = 9729,
        NearestMipmapNearest = 9984,
        LinearMipmapNearest = 9985,
        NearestMipmapLinear = 9986,
        LinearMipmapLinear = 9987
    }
}