using Newtonsoft.Json;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.Formats;
using SixLabors.ImageSharp.PixelFormats;
using System.Buffers.Text;
using System.Configuration;
using System.IO;
using System.Numerics;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Image = SixLabors.ImageSharp.Image;

namespace GraphicsLib.Types.GltfTypes
{
    public class GltfImage : IDisposable
    {
        [JsonProperty("uri")]
        public string? UriString { get; set; }

        [JsonProperty("mimeType")]
        public string? MimeType { get; set; }

        [JsonProperty("bufferView")]
        public int? BufferView { get; set; }
        [JsonProperty("name")]
        public string? Name { get; set; }
        [JsonProperty("extensions")]
        public Dictionary<string, object>? Extensions { get; set; }
        [JsonProperty("extras")]
        public object? Extras { get; set; }
        [JsonIgnore]
        public GltfRoot? GltfRoot { get; set; }
        [JsonIgnore]
        private Image<Rgba32>? textureBitmap;
        [JsonIgnore]
        public Image<Rgba32> ImageData { get => GetImageData(); set => textureBitmap = value; }

        private Image<Rgba32> GetImageData()
        {
            if (textureBitmap == null)
            {
                if (UriString == null)
                {
                    throw new ConfigurationErrorsException("Uri is null and Data is null");
                }
                if (UriString.StartsWith("data:"))
                {
                    string base64Data = UriString.Split(',')[1];

                    // Convert the base64 string into a byte array
                    byte[] imageBytes = Convert.FromBase64String(base64Data);

                    // Load the image from the byte array
                    using (var ms = new MemoryStream(imageBytes))
                    {
                        textureBitmap = Image.Load<Rgba32>(ms);
                    }
                }
                else
                {
                    if (GltfRoot?.SourcePath == null)
                        throw new ConfigurationErrorsException("SourcePath for buffer is null.");
                    textureBitmap = Image.Load<Rgba32>(Path.Combine(GltfRoot.SourcePath, UriString));
                }
            }
            return textureBitmap;
        }

        public void Dispose()
        {
            textureBitmap?.Dispose();
            GC.SuppressFinalize(this);
        }
    }
}