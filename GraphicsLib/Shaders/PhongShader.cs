using GraphicsLib.Primitives;
using GraphicsLib.Types;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Windows.Media.TextFormatting;
using static GraphicsLib.Shaders.PhongShader;

namespace GraphicsLib.Shaders
{
    public class PhongShader : IShader<Vertex>
    {
        public Scene Scene { get => scene; set => SetSceneParams(value); }
        public bool ShadowsEnabled { get => shadowsEnabled; set => SetShadowsEnabled(value); }
        public ShadowMap ShadowMap { get; set; }
        private void SetShadowsEnabled(bool value)
        {
            shadowsEnabled = value;
        }

        private Vector3 ambient; //Вектор фоновогюо освещения
        private Vector3 diffuseColor; //коэффииент рассеянного освещение
        private float specularPower; //коэффициент альфа в степень которого возводится зеркальное освещение.
                                     //Альфа коэффициент блеска поверхности

        private Vector3 lightColor;//коэффициент зеркального освещения
        private float lightIntensity;//Is это цвет зеркального света
        private Vector3 lightPosition; //Позиция истоника света из мировх координат

        private void SetSceneParams(Scene value)
        {
            scene = value;
            if (scene.Obj == null)
                throw new ArgumentException("Scene object is null");
            //caching all values to avoid calling heavy properties
            worldTransform = scene.Obj.transformation.Matrix;
            worldNormalTransform = scene.Obj.transformation.NormalMatrix;
            cameraPos = scene.Camera.Position;
            ambient = scene.AmbientColor * scene.AmbientIntensity;
            diffuseColor = scene.baseDiffuseColor;
            specularPower = scene.SpecularPower;
            lightColor = scene.LightColor * scene.LightIntensity;
            lightIntensity = scene.LightIntensity;
            lightPosition = scene.LightPosition;
        }

        private Matrix4x4 worldTransform;
        private Matrix4x4 worldNormalTransform;
        private Scene scene;
        private bool shadowsEnabled;
        private Vector3 cameraPos;
        public PhongShader()
        {
        }
        public PhongShader(Scene scene)
        {
            Scene = scene;
        }
        // Add shadow-related methods from PhongTexturedShader

        public struct Vertex : IVertex<Vertex>
        {
            public Vector4 Position { get; set; }

            public Vector3 Normal { get; set; }
            public Vector3 WorldPosition { get; set; }
            public static Vertex Lerp(Vertex a, Vertex b, float t)
            {
                return new Vertex
                {
                    Position = Vector4.Lerp(a.Position, b.Position, t),
                    Normal = Vector3.Lerp(a.Normal, b.Normal, t),
                    WorldPosition = Vector3.Lerp(a.WorldPosition, b.WorldPosition, t)
                };
            }
            public static Vertex operator +(Vertex lhs, Vertex rhs)
            {
                return new Vertex
                {
                    Position = lhs.Position + rhs.Position,
                    Normal = lhs.Normal + rhs.Normal,
                    WorldPosition = lhs.WorldPosition + rhs.WorldPosition
                };
            }
            public static Vertex operator -(Vertex lhs, Vertex rhs)
            {
                return new Vertex
                {
                    Position = lhs.Position - rhs.Position,
                    Normal = lhs.Normal - rhs.Normal,
                    WorldPosition = lhs.WorldPosition - rhs.WorldPosition
                };
            }
            public static Vertex operator *(Vertex lhs, float scalar)
            {
                return new Vertex
                {
                    Position = lhs.Position * scalar,
                    Normal = lhs.Normal * scalar,
                    WorldPosition = lhs.WorldPosition * scalar
                };
            }
            public static Vertex operator *(float scalar, Vertex rhs)
            {
                return new Vertex
                {
                    Position = rhs.Position * scalar,
                    Normal = rhs.Normal * scalar,
                    WorldPosition = rhs.WorldPosition * scalar
                };
            }
            public static Vertex operator /(Vertex lhs, float scalar)
            {
                return new Vertex
                {
                    Position = lhs.Position / scalar,
                    Normal = lhs.Normal / scalar,
                    WorldPosition = lhs.WorldPosition / scalar
                };
            }
        }

        public uint PixelShader(Vertex input)
        {
            Vector3 camDir = Vector3.Normalize(cameraPos - input.WorldPosition);
            Vector3 normal = Vector3.Normalize(input.Normal);
            Vector3 lightDir = Vector3.Normalize(lightPosition - input.WorldPosition);
            Vector3 reflectDir = Vector3.Reflect(-lightDir, normal);

            float diffuseFactor = Math.Max(Vector3.Dot(normal, lightDir), 0);
            Vector3 diffuse = diffuseColor * diffuseFactor * lightIntensity;

            float specularFactor = MathF.Pow(Math.Max(Vector3.Dot(reflectDir, camDir), 0), specularPower);
            Vector3 specular = lightColor * specularFactor * lightIntensity;

            Vector3 finalColor = Vector3.Clamp(ambient + diffuse + specular, Vector3.Zero, new Vector3(1, 1, 1));
            uint color = (uint)0xCC << 24
                         | (uint)(finalColor.X * 0x00) << 16
                         | (uint)(finalColor.Y * 0xFF) << 8
                         | (uint)(finalColor.Z * 0x00);
            return color;
        }

        //Позицию вершину переводим в мировую систему(вместе с нормалью)
        public Vertex GetVertexWithWorldPositionFromTriangle(Obj obj, int triangleIndex, int vertexIndex)
        {
            StaticTriangle triangle = obj.triangles[triangleIndex];
            Vertex vertex = default;
            switch (vertexIndex)
            {
                case 0:
                    vertex.Position = Vector4.Transform(new Vector4(triangle.position0, 1), worldTransform);
                    vertex.Normal = Vector3.TransformNormal(triangle.normal0, worldNormalTransform);
                    break;
                case 1:
                    vertex.Position = Vector4.Transform(new Vector4(triangle.position1, 1), worldTransform);
                    vertex.Normal = Vector3.TransformNormal(triangle.normal1, worldNormalTransform);
                    break;
                case 2:
                    vertex.Position = Vector4.Transform(new Vector4(triangle.position2, 1), worldTransform);
                    vertex.Normal = Vector3.TransformNormal(triangle.normal2, worldNormalTransform);
                    break;
                default:
                    throw new ArgumentException("Invalid vertex index");
            }
            vertex.WorldPosition = vertex.Position.AsVector3();
            return vertex;
        }
    }
}