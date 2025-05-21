using GraphicsLib.Primitives;
using GraphicsLib.Types;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using static GraphicsLib.Shaders.PhongTexturedShader;

namespace GraphicsLib.Shaders
{
    public class PhongTexturedShader : IShader<Vertex>
    {
        // Свойство для доступа к сцене, с установкой параметров
        public Scene Scene { get => scene; set => SetSceneParams(value); }

        // Цвет окружающего света (кэшируется для оптимизации)
        private Vector3 ambientLightColor;
        // Степень зеркального отражения (кэшируется)
        private float specularPower;
        // Цвет источника света
        private Vector3 lightColor;
        // Интенсивность света
        private float lightIntensity;
        // Позиция источника света
        private Vector3 lightPosition;
        // Метод для проверки пересечения луча с треугольником с использованием алгоритма Мёллера-Трумбора.
        // Параметры:
        // - origin: Начальная точка луча (Vector3).
        // - direction: Нормализованное направление луча (Vector3).
        // - triangle: Треугольник для проверки пересечения, содержащий три вершины (StaticTriangle).
        // - t: Выходной параметр для расстояния от начала луча до точки пересечения (out float).
        // Возвращает: true, если луч пересекает треугольник и t > EPSILON, false в противном случае.
        private bool IntersectTriangle(Vector3 origin, Vector3 direction, StaticTriangle triangle, out float t)
        {
            // Инициализируем выходной параметр t значением 0. Здесь будет храниться расстояние до точки пересечения, если она найдена.
            t = 0f;

            // Определяем малую константу для обработки проблем с точностью чисел с плавающей запятой.
            // EPSILON используется, чтобы избежать ложных срабатываний из-за численных ошибок, когда значения очень близки к нулю.
            const float EPSILON = 0.000001f;

            // Извлекаем три вершины треугольника из структуры StaticTriangle.
            // v0, v1, v2 представляют позиции вершин треугольника в трёхмерном пространстве.
            Vector3 v0 = triangle.position0;
            Vector3 v1 = triangle.position1;
            Vector3 v2 = triangle.position2;

            // Вычисляем два ребра треугольника, начинающихся из вершины v0.
            // edge1 — вектор от v0 до v1 (v1 - v0).
            // edge2 — вектор от v0 до v2 (v2 - v0).
            // Эти рёбра определяют плоскость треугольника и используются в вычислениях пересечения.
            Vector3 edge1 = v1 - v0;
            Vector3 edge2 = v2 - v0;

            // Вычисляем векторное произведение направления луча и ребра edge2.
            // h = direction × edge2. Этот вектор перпендикулярен как направлению луча, так и ребру edge2.
            // Он используется для определения, параллелен ли луч плоскости треугольника.
            Vector3 h = Vector3.Cross(direction, edge2);

            // Вычисляем скалярное произведение ребра edge1 и вектора h.
            // a = edge1 · h. Этот скаляр представляет проекцию edge1 на h.
            // Он используется для проверки параллельности луча треугольнику и для масштабирования последующих вычислений.
            float a = Vector3.Dot(edge1, h);

            // Проверяем, близко ли a к нулю (в пределах EPSILON).
            // Если |a| < EPSILON, луч почти параллелен плоскости треугольника (h почти перпендикулярен edge1).
            // В этом случае луч не может пересекать треугольник (или пересечение вырождено), поэтому возвращаем false.
            if (a > -EPSILON && a < EPSILON)
                return false;

            // Вычисляем обратное значение a (f = 1/a).
            // Этот коэффициент используется для масштабирования в последующих вычислениях, чтобы избежать многократного деления.
            // Поскольку a ненулевое (проверено выше), деление безопасно.
            float f = 1.0f / a;

            // Вычисляем вектор от вершины треугольника v0 до начала луча.
            // s = origin - v0. Этот вектор представляет относительное положение начала луча относительно треугольника.
            Vector3 s = origin - v0;

            // Вычисляем барицентрическую координату u.
            // u = (s · h) / a = f * (s · h). Это первая барицентрическая координата точки пересечения.
            // u представляет вес вершины v1 в точке пересечения внутри треугольника.
            float u = f * Vector3.Dot(s, h);

            // Проверяем, находится ли u вне допустимого диапазона [0, 1].
            // В барицентрических координатах u должен быть от 0 до 1, чтобы точка пересечения лежала внутри треугольника.
            // Если u < 0 или u > 1, точка пересечения находится вне треугольника, поэтому возвращаем false.
            if (u < 0.0f || u > 1.0f)
                return false;

            // Вычисляем векторное произведение s и ребра edge1.
            // q = s × edge1. Этот вектор перпендикулярен как s, так и edge1.
            // Он используется для вычисления второй барицентрической координаты v.
            Vector3 q = Vector3.Cross(s, edge1);

            // Вычисляем барицентрическую координату v.
            // v = (direction · q) / a = f * (direction · q). Это вторая барицентрическая координата.
            // v представляет вес вершины v2 в точке пересечения внутри треугольника.
            float v = f * Vector3.Dot(direction, q);

            // Проверяем, находится ли v вне допустимого диапазона или u + v > 1.
            // В барицентрических координатах v должен быть >= 0, а u + v <= 1, чтобы точка пересечения лежала внутри треугольника.
            // Если v < 0 или u + v > 1, точка пересечения находится вне треугольника, поэтому возвращаем false.
            if (v < 0.0f || u + v > 1.0f)
                return false;

            // Вычисляем расстояние t до точки пересечения.
            // t = (edge2 · q) / a = f * (edge2 · q). Это расстояние вдоль луча, где он пересекает плоскость треугольника.
            // t показывает, как далеко от начала луча находится точка пересечения.
            t = f * Vector3.Dot(edge2, q);

            // Проверяем, является ли t положительным и больше EPSILON.
            // t > EPSILON гарантирует, что пересечение находится впереди начала луча (не позади) и не слишком близко к нулю (избегая численных ошибок).
            // Если true, луч пересекает треугольник, и t содержит действительное расстояние.
            // Если false, пересечения нет (например, пересечение позади луча или слишком близко к началу).
            return t > EPSILON;
        }
        private bool IsInShadow(Vector3 point, Vector3 lightPos, Obj obj, int currentTriangleIndex = -1)
        {
            float distanceToLight = Vector3.Distance(point, lightPos);

            for (int i = 0; i < obj.triangles.Length; i++)
            {
                //if (i == currentTriangleIndex) continue; // Пропускаем текущий треугольник
                if (IntersectTriangle(point, lightPos, obj.triangles[i], out float t))
                {
                    if (t < distanceToLight)
                        return true;
                }
            }
            return false;
        }
        private void SetSceneParams(Scene value)
        {
            scene = value; // Сохраняем сцену
                           // Проверяем, есть ли объект в сцене
            if (scene.Obj == null)
                throw new ArgumentException("Scene object is null"); // Если нет, выбрасываем исключение
                                                                     // Кэшируем матрицу трансформации объекта
            worldTransform = scene.Obj.transformation.Matrix;
            // Кэшируем матрицу трансформации нормалей
            worldNormalTransform = scene.Obj.transformation.NormalMatrix;
            // Кэшируем позицию камеры
            cameraPos = scene.Camera.Position;
            // Вычисляем и кэшируем цвет окружающего света
            ambientLightColor = scene.AmbientColor * scene.AmbientIntensity;
            // Кэшируем степень зеркального отражения
            specularPower = scene.SpecularPower;
            // Вычисляем и кэшируем цвет света
            lightColor = scene.LightColor * scene.LightIntensity;
            // Кэшируем интенсивность света
            lightIntensity = scene.LightIntensity;
            // Кэшируем позицию света
            lightPosition = scene.LightPosition;
        }

        // Матрица трансформации объекта в мировых координатах
        private Matrix4x4 worldTransform;
        // Матрица трансформации нормалей
        private Matrix4x4 worldNormalTransform;
        // Ссылка на сцену
        private Scene scene;
        // Позиция камеры
        private Vector3 cameraPos;
        // Конструктор по умолчанию
        public PhongTexturedShader()
        {
        }
        // Конструктор с параметром сцены
        public PhongTexturedShader(Scene scene)
        {
            Scene = scene; // Устанавливаем сцену через свойство
        }

        // Структура вершины для шейдера
        public struct Vertex : IVertex<Vertex>
        {
            // Позиция вершины в однородных координатах (x, y, z, w)
            public Vector4 Position { get; set; }
            // Нормаль вершины
            public Vector3 Normal { get; set; }
            // Позиция вершины в мировых координатах
            public Vector3 WorldPosition { get; set; }
            // Текстурные координаты (диффузные)
            public Vector2 Uv { get; set; }
            // Тангент вершины
            public Vector4 Tangent { get; set; }
            // Текстурные координаты для нормалей
            public Vector2 NormalUv { get; set; }
            // Материал вершины
            public Material Material { get; set; }
            // Текстурные координаты для шероховатости
            public Vector2 RoughnessUv { get; set; }
            // Линейная интерполяция между двумя вершинами
            public static Vertex Lerp(Vertex a, Vertex b, float t)
            {
                return new Vertex
                {
                    Position = Vector4.Lerp(a.Position, b.Position, t),
                    Normal = Vector3.Lerp(a.Normal, b.Normal, t),
                    WorldPosition = Vector3.Lerp(a.WorldPosition, b.WorldPosition, t),
                    Uv = Vector2.Lerp(a.Uv, b.Uv, t),
                    Tangent = Vector4.Lerp(a.Tangent, b.Tangent, t),
                    NormalUv = Vector2.Lerp(a.NormalUv, b.NormalUv, t),
                    RoughnessUv = Vector2.Lerp(a.RoughnessUv, b.RoughnessUv, t),
                    Material = a.Material
                };
            }
            public static Vertex operator +(Vertex lhs, Vertex rhs)
            {
                return new Vertex
                {
                    Position = lhs.Position + rhs.Position,
                    Normal = lhs.Normal + rhs.Normal,
                    WorldPosition = lhs.WorldPosition + rhs.WorldPosition,
                    Uv = lhs.Uv + rhs.Uv,
                    Tangent = lhs.Tangent + rhs.Tangent,
                    NormalUv = lhs.NormalUv + rhs.NormalUv,
                    RoughnessUv = lhs.RoughnessUv + rhs.RoughnessUv,
                    Material = lhs.Material
                };
            }
            public static Vertex operator -(Vertex lhs, Vertex rhs)
            {
                return new Vertex
                {
                    Position = lhs.Position - rhs.Position,
                    Normal = lhs.Normal - rhs.Normal,
                    WorldPosition = lhs.WorldPosition - rhs.WorldPosition,
                    Uv = lhs.Uv - rhs.Uv,
                    Tangent = lhs.Tangent - rhs.Tangent,
                    NormalUv = lhs.NormalUv - rhs.NormalUv,
                    RoughnessUv = lhs.RoughnessUv - rhs.RoughnessUv,
                    Material = lhs.Material
                };
            }
            public static Vertex operator *(Vertex lhs, float scalar)
            {
                return new Vertex
                {
                    Position = lhs.Position * scalar,
                    Normal = lhs.Normal * scalar,
                    WorldPosition = lhs.WorldPosition * scalar,
                    Uv = lhs.Uv * scalar,
                    Tangent = lhs.Tangent * scalar,
                    NormalUv = lhs.NormalUv * scalar,
                    RoughnessUv = lhs.RoughnessUv * scalar,
                    Material = lhs.Material
                };
            }
            public static Vertex operator *(float scalar, Vertex rhs)
            {
                return rhs * scalar;
            }
            public static Vertex operator /(Vertex lhs, float scalar)
            {
                return lhs * (1 / scalar);
            }
        }

        // Шейдер для вычисления цвета пикселя
        public uint PixelShader(Vertex input)
        {

            Material material = input.Material;

            Vector3 normal = Vector3.Normalize(input.Normal);
            // Если есть текстура нормалей, используем её
            if (material.normalTextureSampler != null)
            {
                // Знак тангента (нужен для вычисления битангента)
                float sign = input.Tangent.W;
                Vector3 tangent = input.Tangent.AsVector3();
                // Получаем нормаль из текстуры и преобразуем из [0,1] в [-1,1]
                Vector3 tangentSpaceNormal = material.normalTextureSampler.Sample(input.NormalUv).AsVector3();
                tangentSpaceNormal = tangentSpaceNormal * 2 - new Vector3(1, 1, 1); // Декодируем нормаль
                // Вычисляем битангент через векторное произведение
                Vector3 bitangent = sign * Vector3.Cross(normal, tangent);
                // Преобразуем нормаль из касательного пространства в мировое
                normal = Vector3.Normalize(tangent * tangentSpaceNormal.X + bitangent * tangentSpaceNormal.Y + normal * tangentSpaceNormal.Z);
            }
            // Получаем базовые значения шероховатости и металличности
            float roughness = material.roughness;
            float metallic = material.metallic;
            // Если есть текстура металличности/шероховатости, корректируем значения
            if (material.metallicRoughnessTextureSampler != null)
            {
                // Получаем данные из текстуры
                Vector4 metallicRoughness = material.metallicRoughnessTextureSampler.Sample(input.RoughnessUv);
                roughness *= metallicRoughness.Y; // Y — шероховатость
                metallic *= metallicRoughness.Z;  // Z — металличность
            }
            // Получаем базовый диффузный цвет материала
            Vector4 diffuseColor = material.baseColor;
            // Если есть текстура базового цвета, применяем её
            if (material.baseColorTextureSampler != null)
            {
                Vector2 uv = input.Uv; // Берем UV вершины
                                       // Умножаем базовый цвет на цвет из текстуры
                diffuseColor *= material.baseColorTextureSampler.Sample(uv);
            }
            // Вычисляем фоновую (ambient) компоненту освещения
            Vector3 ambient = ambientLightColor * diffuseColor.AsVector3();
            // Вычисляем направление света от вершины к источнику
            bool inShadow = IsInShadow(input.WorldPosition, lightPosition, scene.Obj);

            Vector3 diffuse = Vector3.Zero;
            Vector3 specular = Vector3.Zero;

            if (!inShadow)
            {
                // Вычисляем направление света
                Vector3 lightDir = Vector3.Normalize(lightPosition - input.WorldPosition);
                // Вычисляем коэффициент рассеянного света
                float diffuseFactor = Math.Max(Vector3.Dot(normal, lightDir), 0);
                // Вычисляем рассеянную компоненту
                diffuse = diffuseColor.AsVector3() * (diffuseFactor * lightIntensity);

                // Вычисляем направление к камере
                Vector3 camDir = Vector3.Normalize(cameraPos - input.WorldPosition);
                // Вычисляем направление отражения
                Vector3 reflectDir = Vector3.Reflect(-lightDir, normal);
                // Вычисляем базовый коэффициент зеркального отражения
                float specularBase = Math.Max(Vector3.Dot(reflectDir, camDir), 0);
                // Корректируем шероховатость
                float adjustedRoughness = roughness * (1.0f - metallic * 0.2f);
                float specularPowerLocal = MathF.Pow(2, (((adjustedRoughness) * 10.0f) + 1.0f));
                // Вычисляем коэффициент зеркального отражения
                float specularFactor = (specularBase > 0) ? MathF.Pow(specularBase, specularPowerLocal) : 0;
                // Вычисляем зеркальную компоненту
                specular = lightColor * (specularFactor * lightIntensity);
            }

            // Суммируем все компоненты освещения
            Vector3 finalColor = Vector3.Clamp(ambient + diffuse + specular, Vector3.Zero, new Vector3(1, 1, 1));

            uint color = (uint)(diffuseColor.W * 0xFF) << 24 
                         | (uint)(finalColor.X * 0xFF) << 16
                         | (uint)(finalColor.Y * 0xFF) << 8  
                         | (uint)(finalColor.Z * 0xFF);     
            return color; // Возвращаем цвет пикселя
        }

        // Метод для получения вершины с мировой позицией из треугольника
        public Vertex GetVertexWithWorldPositionFromTriangle(Obj obj, int triangleIndex, int vertexIndex)
        {
            // Получаем треугольник по индексу
            StaticTriangle triangle = obj.triangles[triangleIndex];
            Vertex vertex = default; // Создаем пустую вершину
                                     // В зависимости от индекса вершины (0, 1 или 2) заполняем данные
            switch (vertexIndex)
            {
                case 0: // Первая вершина треугольника
                        // Трансформируем позицию в мировые координаты
                    vertex.Position = Vector4.Transform(new Vector4(triangle.position0, 1), worldTransform);
                    // Трансформируем нормаль
                    vertex.Normal = Vector3.TransformNormal(triangle.normal0, worldNormalTransform);
                    vertex.Uv = triangle.uvCoordinate0;           // Устанавливаем диффузные UV
                    vertex.Tangent = triangle.tangent0;           // Устанавливаем тангент
                    vertex.NormalUv = triangle.normalUvCoordinate0; // Устанавливаем нормальные UV
                    vertex.RoughnessUv = triangle.roughnessUvCoordinate0; // Устанавливаем шероховатые UV
                    break;
                case 1: // Вторая вершина
                    vertex.Position = Vector4.Transform(new Vector4(triangle.position1, 1), worldTransform);
                    vertex.Normal = Vector3.TransformNormal(triangle.normal1, worldNormalTransform);
                    vertex.Uv = triangle.uvCoordinate1;
                    vertex.Tangent = triangle.tangent1;
                    vertex.NormalUv = triangle.normalUvCoordinate1;
                    vertex.RoughnessUv = triangle.roughnessUvCoordinate1;
                    break;
                case 2: // Третья вершина
                    vertex.Position = Vector4.Transform(new Vector4(triangle.position2, 1), worldTransform);
                    vertex.Normal = Vector3.TransformNormal(triangle.normal2, worldNormalTransform);
                    vertex.Uv = triangle.uvCoordinate2;
                    vertex.Tangent = triangle.tangent2;
                    vertex.NormalUv = triangle.normalUvCoordinate2;
                    vertex.RoughnessUv = triangle.roughnessUvCoordinate2;
                    break;
                default:
                    throw new ArgumentException("Invalid vertex index"); // Ошибка, если индекс неверный
            }
            vertex.Material = triangle.material; // Устанавливаем материал треугольника
            vertex.WorldPosition = vertex.Position.AsVector3(); // Преобразуем позицию в Vector3
            return vertex; // Возвращаем готовую вершину
        }
    }
}
