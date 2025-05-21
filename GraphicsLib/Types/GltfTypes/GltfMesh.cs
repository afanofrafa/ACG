using Newtonsoft.Json;
using System.Configuration;
using System.Numerics;

namespace GraphicsLib.Types.GltfTypes
{
    public class GltfMesh
    {
        [JsonProperty("primitives")]
        public required GtlfMeshPrimitive[] Primitives { get; set; }
        [JsonProperty("weights")]
        public float[]? Weights { get; set; }
        [JsonProperty("name")]
        public string? Name { get; set; }
        [JsonProperty("extensions")]
        public Dictionary<string, object>? Extensions { get; set; }
        [JsonProperty("extras")]
        public object? Extras { get; set; }
    }

    public class GtlfMeshPrimitive
    {
        [JsonProperty("attributes")]
        public required Dictionary<string, int> Attributes { get; set; }
        [JsonProperty("indices")]
        public int? Indices { get; set; }
        [JsonProperty("material")]
        public int? Material { get; set; }
        [JsonProperty("mode")]
        public GltfMeshMode Mode { get; set; } = GltfMeshMode.TRIANGLES;
        [JsonProperty("targets")]
        public object[]? Targets { get; set; }
        [JsonProperty("extensions")]
        public Dictionary<string, object>? Extensions { get; set; }
        [JsonProperty("extras")]
        public object? Extras { get; set; }

        [JsonIgnore]
        public GltfRoot? Root { get; set; }
        [JsonIgnore]
        public Vector3[]? Position { get => GetPosition(); }
        [JsonIgnore]
        public Vector3[]? Normal { get => GetNormals(); }
        [JsonIgnore]
        public int[]? PointIndices { get => GetPointIndices(); }
        [JsonIgnore]
        public Vector4[]? Tangent { get => GetTangents(); }

        public Vector2[]? GetTextureCoords(int v)
        {
            Vector2[]? vectors = null;
            if (Attributes.TryGetValue($"TEXCOORD_{v}", out int index))
            {
                if (Root == null)
                {
                    throw new ConfigurationErrorsException("Root is null.");
                }
                var accessor = Root.Accessors![index];
                object[] data = GltfUtils.GetAccessorData(accessor);
                vectors = new Vector2[accessor.Count];
                for (int i = 0; i < accessor.Count; i++)
                {
                    vectors[i] = (Vector2)data[i];
                }
            }
            return vectors;
        }
        private Vector4[]? GetTangents()
        {
            Vector4[]? vectors = null;
            if (Attributes.TryGetValue("TANGENT", out int index))
            {
                if (Root == null)
                {
                    throw new ConfigurationErrorsException("Root is null.");
                }
                var accessor = Root.Accessors![index];
                object[] data = GltfUtils.GetAccessorData(accessor);
                vectors = new Vector4[accessor.Count];
                for (int i = 0; i < accessor.Count; i++)
                {
                    vectors[i] = (Vector4)data[i];
                }
            }
            return vectors;
        }
        private Vector3[]? GetPosition()
        {
            Vector3[]? vectors = null;
            if (Attributes.ContainsKey("POSITION"))
            {
                if (Root == null)
                {
                    throw new ConfigurationErrorsException("Root is null.");
                }
                var accessor = Root.Accessors![Attributes["POSITION"]];
                object[] uncastedData = GltfUtils.GetAccessorData(accessor);
                vectors = new Vector3[accessor.Count];
                for (int i = 0; i < accessor.Count; i++)
                {
                    vectors[i] = (Vector3)uncastedData[i];
                }
            }
            return vectors;
        }

        private Vector3[]? GetNormals() {
            if (Attributes.TryGetValue("NORMAL", out int index)) {
                if (Root == null) {
                    throw new ConfigurationErrorsException("Root is null.");
                }
                var accessor = Root.Accessors![index];
                object[] data = GltfUtils.GetAccessorData(accessor);
                Vector3[] vectors = new Vector3[accessor.Count];
                for (int i = 0; i < accessor.Count; i++) {
                    vectors[i] = (Vector3)data[i];
                }
                return vectors;
            } else {
                return null;
            }
        }

        private int[]? GetPointIndices()
        {
            int[]? indices = null;
            if (Indices.HasValue)
            {
                if (Root == null)
                {
                    throw new ConfigurationErrorsException("Root is null.");
                }
                var accessor = Root.Accessors![Indices.Value];
                object[] uncastedData = GltfUtils.GetAccessorData(accessor);
                indices = new int[accessor.Count];
                for (int i = 0; i < accessor.Count; i++)
                {
                    indices[i] = (int)uncastedData[i];
                }
            }
            return indices;
        }
    }
    public enum GltfMeshMode
    {
        POINTS = 0,
        LINES = 1,
        LINE_LOOP = 2,
        LINE_STRIP = 3,
        TRIANGLES = 4,
        TRIANGLE_STRIP = 5,
        TRIANGLE_FAN = 6
    }
}