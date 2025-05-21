using Newtonsoft.Json;
using System.Numerics;
using System.Xml.Linq;

namespace GraphicsLib.Types.GltfTypes
{
    public class GltfNode
    {
        [JsonProperty("camera")]
        public int? Camera { get; set; }
        [JsonProperty("children")]
        public int[]? Children { get; set; }
        [JsonProperty("skin")]
        public int? Skin { get; set; }
        [JsonProperty("matrix")]
        public Matrix4x4? Matrix { get; set; }
        [JsonProperty("mesh")]
        public int? Mesh { get; set; }
        [JsonProperty("rotation")]
        public Quaternion? Rotation { get; set; }
        [JsonProperty("scale")]
        public Vector3? Scale { get; set; }
        [JsonProperty("translation")]
        public Vector3? Translation { get; set; }
        [JsonProperty("weights")]
        public float[]? Weights { get; set; }
        [JsonProperty("name")]
        public string? Name { get; set; }
        [JsonProperty("extensions")]
        public Dictionary<string, object>? Extensions { get; set; }
        [JsonProperty("extras")]
        public object? Extras { get; set; }
        [JsonIgnore]
        public GltfNode? Parent { get; set; }
        [JsonIgnore]
        public Matrix4x4 GlobalTransform { get => GetGlobalTransform(); }
        [JsonIgnore]
        public Matrix4x4 LocalTransform { get => GetLocalTransform(); }
        public Matrix4x4 LocalNormalTransform { get => GetLocalNormalTransform(); }
        public Matrix4x4 GlobalNormalTransform { get => GetGlobalNormalTransform(); }

        private Matrix4x4 GetGlobalNormalTransform()
        {
            Matrix4x4.Invert(GlobalTransform, out Matrix4x4 normalTransform);
            normalTransform = Matrix4x4.Transpose(normalTransform);
            return normalTransform;
        }

        private Matrix4x4 GetLocalNormalTransform()
        {
            if (Matrix.HasValue)
            {
                Matrix4x4 transform = Matrix.Value;
                transform.Translation = default;
                return transform;
            }
            else
            {
                var transform = Matrix4x4.Identity;
                if (Scale.HasValue)
                {
                    Vector3 scale = Scale.Value;
                    transform *= Matrix4x4.CreateScale(scale);
                }
                if (Rotation.HasValue)
                {
                    Quaternion rotation = Rotation.Value;
                    transform *= Matrix4x4.CreateFromQuaternion(rotation);
                }
                return transform;
            }
        }

        private Matrix4x4 GetLocalTransform()
        {
            if (Matrix.HasValue)
            {
                return Matrix.Value;
            }
            else
            {
                var transform = Matrix4x4.Identity;
                if (Scale.HasValue)
                {
                    Vector3 scale = Scale.Value;
                    transform *= Matrix4x4.CreateScale(scale);
                }
                if (Rotation.HasValue)
                {
                    Quaternion rotation = Rotation.Value;
                    transform *= Matrix4x4.CreateFromQuaternion(rotation);
                }
                if (Translation.HasValue)
                {
                    Vector3 translation = Translation.Value;
                    transform *= Matrix4x4.CreateTranslation(translation);
                }
                return transform;
            }
        }

        private Matrix4x4 GetGlobalTransform()
        {
            if (Parent == null)
            {
                return LocalTransform;
            }
            else
            {
                return LocalTransform * Parent.GlobalTransform;
            }
        }
    }
}