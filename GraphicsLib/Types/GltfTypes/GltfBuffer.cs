using Newtonsoft.Json;
using System.Buffers;
using System.Buffers.Text;
using System.Configuration;
using System.IO;

namespace GraphicsLib.Types.GltfTypes
{
    public class GltfBuffer
    {

        [JsonProperty("uri")]
        public string? UriString { get; set; }
        [JsonProperty("byteLength")]
        public required int ByteLength { get; set; }
        [JsonProperty("name")]
        public string? Name { get; set; }
        [JsonProperty("extensions")]
        public Dictionary<string, object>? Extensions { get; set; }
        [JsonProperty("extras")]
        public object? Extras { get; set; }
        [JsonIgnore]

        private byte[]? _data;
        [JsonIgnore]
        public GltfRoot? GltfRoot { get; set; }
        public byte[] Data
        {
            get
            {
                if (_data == null)
                {
                    if (UriString == null)
                    {
                        throw new ConfigurationErrorsException("Uri is null and Data is null");
                    }
                    if (UriString.StartsWith("data:application/octet-stream;base64,"))
                    {
                        int offset = "data:application/octet-stream;base64,".Length;
                        int length = UriString.Length - offset;

                        // Allocate a buffer for the decoded data
                        _data = new byte[Base64.GetMaxDecodedFromUtf8Length(length)];

                        // Decode directly from the substring
                        if (!Convert.TryFromBase64Chars(UriString.AsSpan(offset, length), _data, out _))
                        {
                            throw new FormatException("Invalid base64 string");
                        }
                    }
                    else
                    {
                        if (GltfRoot?.SourcePath == null)
                            throw new ConfigurationErrorsException("SourcePath for buffer is null.");
                        _data = File.ReadAllBytes(Path.Combine(GltfRoot.SourcePath, UriString));
                    }
                }
                return _data;
            }
            set => _data = value;
        }  
    }
}