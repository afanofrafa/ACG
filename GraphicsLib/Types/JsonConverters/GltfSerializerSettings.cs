using Newtonsoft.Json;
using Newtonsoft.Json.Serialization;

namespace GraphicsLib.Types.JsonConverters
{
    public static class GltfSerializerSettings
    {
        public static JsonSerializerSettings GetSettings(string sourceDirectory)  => new JsonSerializerSettings
        {
            Converters = {new GltfRootConverter(sourceDirectory),
                          new QuaternionConverter(),
                          new QuaternioNullableConverter(),
                          new GltfAccessorTypeConverter(),
                          new Matrix4x4Converter(),
                          new Matrix4x4NullableConverter(),
                          new Vector4Converter(),
                          new Vector4NullableConverter(),
                          new Vector3Converter(),
                          new Vector3NullableConverter(),
                          new Vector2Converter(),
                          new Vector2NullableConverter(),
                          new GltfMaterialAlphaModeConverter()
            },
            ContractResolver = new DefaultContractResolver {
                NamingStrategy = new CamelCaseNamingStrategy {
                    OverrideSpecifiedNames = true
                }
            }
        };
    }
}
