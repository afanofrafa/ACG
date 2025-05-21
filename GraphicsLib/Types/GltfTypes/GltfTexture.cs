using Newtonsoft.Json;
using SixLabors.ImageSharp.PixelFormats;
using System.Configuration;

namespace GraphicsLib.Types.GltfTypes
{
    public class GltfTexture
    {
        [JsonProperty("sampler")]
        public int? SamplerIndex { get; set; }

        [JsonProperty("source")]
        public int? ImageSourceIndex { get; set; }
        [JsonProperty("name")]
        public string? Name { get; set; }
        [JsonProperty("extensions")]
        public Dictionary<string, object>? Extensions { get; set; }
        [JsonProperty("extras")]
        public object? Extras { get; set; }
        [JsonIgnore]
        public GltfImage? Image { get; set; }
        [JsonIgnore]
        public GltfSampler? Sampler { get; set; }

        public Sampler GetConvertedSampler()
        {
            if (Sampler == null || Image == null)
            {
                throw new Exception("Sampler or Image is null");
            }
            var image = Image.ImageData;
            var sampler = Sampler.GetSampler();
            Rgba32[] pixels = new Rgba32[image.Height * image.Width];
            image.CopyPixelDataTo(pixels);
            sampler.BindTexture(pixels, image.Width, image.Height);
            return sampler;
        }
    }
}