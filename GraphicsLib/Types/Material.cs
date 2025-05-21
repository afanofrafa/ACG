using GraphicsLib.Types.GltfTypes;
using System.Numerics;

namespace GraphicsLib.Types
{
    public class Material
    {
        public static readonly Material defaultMaterial = new();


        public string name;

        //also known as albedo
        public Vector4 baseColor = new(1, 1, 1, 1);
        public Sampler? baseColorTextureSampler;
        public Sampler? normalTextureSampler;
        public Sampler? metallicRoughnessTextureSampler;
        public Sampler? occlusionTextureSampler;
        public float metallic = 1f;
        public float roughness = 1f;

        public Material()
        {
            name = string.Empty;
        }
        public static Material FromGltfMaterial(GltfMaterial material)
        {
            Material newMaterial = new();
            if (material.Name != null)
                newMaterial.name = material.Name;
            if (material.PbrMetallicRoughness != null)
            {
                var pbr = material.PbrMetallicRoughness;
                newMaterial.metallic = pbr.MetallicFactor;
                newMaterial.roughness = pbr.RoughnessFactor;
                newMaterial.baseColor = pbr.BaseColorFactor;
                //add samplers if they are present
                newMaterial.baseColorTextureSampler = pbr.BaseColorTexture?.GetConvertedSampler();
                newMaterial.metallicRoughnessTextureSampler = pbr.MetallicRoughnessTexture?.GetConvertedSampler();
            }
            //add samplers if they are present
            newMaterial.normalTextureSampler = material.NormalTexture?.GetConvertedSampler();
            newMaterial.occlusionTextureSampler = material.OcclusionTexture?.GetConvertedSampler();

            return newMaterial;
        }
    }
}