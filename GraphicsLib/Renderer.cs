using GraphicsLib.Primitives;
using GraphicsLib.Shaders;
using GraphicsLib.Types;
using System.Collections.Concurrent;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Windows.Media.Imaging;
using System.Threading.Tasks;

namespace GraphicsLib
{
    public class Renderer
    {
        public Scene Scene { get; set; }
        public bool shadowsEnabled { get; set; }
        public WriteableBitmap? Bitmap { get; set; }
        private Vector4[] projectionSpaceBuffer;
        private int bufferLength;
        private Zbuffer? zBuffer;
        private ZbufferV2? zBufferV2;
        private GBuffer? gBuffer;
        public ShadowMap shadowMap { get; set; } = new ShadowMap(1024, 1024);
        public Renderer(Scene scene)
        {
            Scene = scene;
            projectionSpaceBuffer = [];
            bufferLength = 0;
            Bitmap = default;
            zBuffer = default;
            zBufferV2 = default;
            gBuffer = default;
        }

        public void ClearTriangleCache()
        {
            Pipeline<PhongShader, PhongShader.Vertex>.ClearCache();
            Pipeline<PhongTexturedShader, PhongTexturedShader.Vertex>.ClearCache();
        }

        private void ResizeBuffer(Obj obj)
        {
            int vertexCount = obj.vertices.Length;
            if (projectionSpaceBuffer.Length < vertexCount)
            {
                projectionSpaceBuffer = new Vector4[vertexCount];
            }
            bufferLength = vertexCount;
        }

        private void ResizeAndClearZBuffer()
        {
            if (Bitmap == null)
                return;
            if (zBuffer == null)
            {
                zBuffer = new Zbuffer(Bitmap.PixelWidth, Bitmap.PixelHeight);
            }
            else
            {
                int width = Bitmap.PixelWidth;
                int height = Bitmap.PixelHeight;
                if (zBuffer.Width != width && zBuffer.Height != height)
                {
                    zBuffer = new Zbuffer(Bitmap.PixelWidth, Bitmap.PixelHeight);
                }
                else
                {
                    zBuffer.Clear();
                }
            }
        }

        private void ResizeAndClearZBufferV2()
        {
            if (Bitmap == null)
                return;
            if (zBufferV2 == null)
            {
                zBufferV2 = new ZbufferV2(Bitmap.PixelWidth, Bitmap.PixelHeight);
            }
            else
            {
                int width = Bitmap.PixelWidth;
                int height = Bitmap.PixelHeight;
                if (zBufferV2.Width != width && zBufferV2.Height != height)
                {
                    zBufferV2 = new ZbufferV2(Bitmap.PixelWidth, Bitmap.PixelHeight);
                }
                else
                {
                    zBufferV2.Clear();
                }
            }
        }

        private void ResizeAndClearZBufferWithIndicies()
        {
            if (Bitmap == null)
                return;
            if (gBuffer == null)
            {
                gBuffer = new(Bitmap.PixelWidth, Bitmap.PixelHeight);
            }
            else
            {
                int width = Bitmap.PixelWidth;
                int height = Bitmap.PixelHeight;
                if (gBuffer.Width != width && gBuffer.Height != height)
                {
                    gBuffer = new(Bitmap.PixelWidth, Bitmap.PixelHeight);
                }
                else
                {
                    gBuffer.Clear();
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Render<Shader, Vertex>() where Shader : IShader<Vertex>, new() where Vertex : struct, IVertex<Vertex>
        {
            if (Bitmap == null)
                return 0;
            if (Scene.Obj == null)
                return 0;
            ResizeAndClearZBufferV2();
            Pipeline<Shader, Vertex> pipeline = new(this);
            int count = pipeline.Render();
            return count;
        }

        class Pipeline<Shader, Vertex> where Shader : IShader<Vertex>, new() where Vertex : struct, IVertex<Vertex>
        {
            private readonly Renderer renderer;
            private readonly Shader shader;
            private readonly Scene scene;
            private readonly Matrix4x4 cameraTransform;
            private readonly Matrix4x4 projectionTransform;
            private readonly Matrix4x4 viewPortTransform;
            private readonly int screenWidth;
            private readonly int screenHeight;
            private static readonly ConcurrentDictionary<int, (Vertex p0, Vertex p1, Vertex p2)> triangleCache
                = new ConcurrentDictionary<int, (Vertex p0, Vertex p1, Vertex p2)>();

            public Pipeline(Renderer renderer)
            {
                this.renderer = renderer;
                scene = renderer.Scene;
                shader = new Shader()
                {
                    Scene = scene,
                    ShadowsEnabled = renderer.shadowsEnabled,
                    ShadowMap = renderer.shadowMap
                };
                cameraTransform = scene.Camera.ViewMatrix;
                projectionTransform = scene.Camera.ProjectionMatrix;
                viewPortTransform = scene.Camera.ViewPortMatrix;
                screenHeight = Convert.ToInt32(scene.Camera.ScreenHeight);
                screenWidth = Convert.ToInt32(scene.Camera.ScreenWidth);
            }

            public static void ClearCache()
            {
                triangleCache.Clear();
            }

            public int Render()
            {
                int triangleCount = scene.Obj!.triangles.Length;

                if (triangleCache.IsEmpty)
                {
                    UpdateTriangleCache(triangleCount);
                }
                // Рендеринг карты теней, если тени включены
                //if (renderer.shadowsEnabled)
                //{
                //    RenderShadowMap(triangleCount);
                //}
                Parallel.For(0, triangleCount, AssembleTriangle);
                renderer.Bitmap!.FlushZBufferV2(renderer.zBufferV2!);
                return triangleCount;
            }
            private void RenderShadowMap(int triangleCount)
            {
                renderer.shadowMap.Clear(); // Очищаем карту теней

                Matrix4x4 lightViewProj = scene.LightViewMatrix * scene.LightProjectionMatrix;
                Matrix4x4 lightViewport = Matrix4x4.CreateViewport(0, 0, renderer.shadowMap.Width, renderer.shadowMap.Height, 0, 1);

                for (int i = 0; i < triangleCount; i++)
                {
                    if (!triangleCache.TryGetValue(i, out var cachedTriangle))
                        continue;

                    Vertex p0 = cachedTriangle.p0;
                    Vertex p1 = cachedTriangle.p1;
                    Vertex p2 = cachedTriangle.p2;

                    // Трансформация в пространство света
                    p0.Position = Vector4.Transform(p0.Position, lightViewProj);
                    p1.Position = Vector4.Transform(p1.Position, lightViewProj);
                    p2.Position = Vector4.Transform(p2.Position, lightViewProj);

                    // Проверка ориентации (back-face culling)
                    Vector4 normal = Vector3.Cross((p2.Position - p0.Position).AsVector3(), (p1.Position - p0.Position).AsVector3()).AsVector4();
                    float orientation = Vector4.Dot(normal, p0.Position);
                    if (orientation <= 0)
                        continue;

                    // Проекция в экранное пространство карты теней
                    TransformToShadowMap(ref p0, lightViewport);
                    TransformToShadowMap(ref p1, lightViewport);
                    TransformToShadowMap(ref p2, lightViewport);

                    DrawDepthTriangle(p0, p1, p2);
                }
            }

            private void TransformToShadowMap(ref Vertex vertex, Matrix4x4 viewport)
            {
                float invW = 1 / vertex.Position.W;
                vertex.Position *= invW;
                Vector4 transformedPos = Vector4.Transform(vertex.Position, viewport);
                // Создаём новый Vector4 с сохранением глубины в Z
                vertex.Position = new Vector4(transformedPos.X, transformedPos.Y, invW, transformedPos.W);
            }

            private void DrawDepthTriangle(Vertex p0, Vertex p1, Vertex p2)
            {
                Vertex min = p0, mid = p1, max = p2;
                if (mid.Position.Y < min.Position.Y) (min, mid) = (mid, min);
                if (max.Position.Y < min.Position.Y) (min, max) = (max, min);
                if (max.Position.Y < mid.Position.Y) (mid, max) = (max, mid);

                if (min.Position.Y == mid.Position.Y)
                {
                    if (mid.Position.X < min.Position.X) (min, mid) = (mid, min);
                    DrawFlatTopDepthTriangle(min, mid, max);
                }
                else if (max.Position.Y == mid.Position.Y)
                {
                    if (max.Position.X > mid.Position.X) (mid, max) = (max, mid);
                    DrawFlatBottomDepthTriangle(min, mid, max);
                }
                else
                {
                    float c = (mid.Position.Y - min.Position.Y) / (max.Position.Y - min.Position.Y);
                    Vertex interpolant = Vertex.Lerp(min, max, c);
                    if (interpolant.Position.X > mid.Position.X)
                    {
                        DrawFlatBottomDepthTriangle(min, interpolant, mid);
                        DrawFlatTopDepthTriangle(mid, interpolant, max);
                    }
                    else
                    {
                        DrawFlatBottomDepthTriangle(min, mid, interpolant);
                        DrawFlatTopDepthTriangle(interpolant, mid, max);
                    }
                }
            }

            private void DrawFlatTopDepthTriangle(Vertex leftTop, Vertex rightTop, Vertex bottom)
            {
                float dy = bottom.Position.Y - leftTop.Position.Y;
                Vertex dLeft = (bottom - leftTop) / dy;
                Vertex dRight = (bottom - rightTop) / dy;
                Vertex left = leftTop, right = rightTop;
                DrawDepthScanline(left, right, bottom.Position.Y, dLeft, dRight);
            }

            private void DrawFlatBottomDepthTriangle(Vertex top, Vertex leftBottom, Vertex rightBottom)
            {
                float dy = rightBottom.Position.Y - top.Position.Y;
                Vertex dLeft = (leftBottom - top) / dy;
                Vertex dRight = (rightBottom - top) / dy;
                Vertex left = top, right = top;
                DrawDepthScanline(left, right, rightBottom.Position.Y, dLeft, dRight);
            }

            private void DrawDepthScanline(Vertex left, Vertex right, float yMax, Vertex dLeft, Vertex dRight)
            {
                int yStart = Math.Max((int)Math.Ceiling(left.Position.Y), 0);
                int yEnd = Math.Min((int)Math.Ceiling(yMax), renderer.shadowMap.Height);
                float yPrestep = yStart - left.Position.Y;

                left += dLeft * yPrestep;
                right += dRight * yPrestep;

                for (int y = yStart; y < yEnd; y++)
                {
                    int xStart = Math.Max((int)Math.Ceiling(left.Position.X), 0);
                    int xEnd = Math.Min((int)Math.Ceiling(right.Position.X), renderer.shadowMap.Width);
                    float xPrestep = xStart - left.Position.X;
                    Vertex scan = Vertex.Lerp(left, right, (xStart - left.Position.X) / (right.Position.X - left.Position.X));
                    float dx = right.Position.X - left.Position.X;
                    Vertex dScan = dx > 0 ? (right - left) / dx : default;

                    for (int x = xStart; x < xEnd; x++)
                    {
                        float depth = scan.Position.Z;
                        renderer.shadowMap.TestAndSet(x, y, depth);
                        scan += dScan;
                    }
                    left += dLeft;
                    right += dRight;
                }
            }
            private void UpdateTriangleCache(int triangleCount)
            {
                Obj obj = scene.Obj!;
                for (int i = 0; i < triangleCount; i++)
                {
                    Vertex p0 = shader.GetVertexWithWorldPositionFromTriangle(obj, i, 0);
                    Vertex p1 = shader.GetVertexWithWorldPositionFromTriangle(obj, i, 1);
                    Vertex p2 = shader.GetVertexWithWorldPositionFromTriangle(obj, i, 2);
                    triangleCache[i] = (p0, p1, p2);
                }
            }

            private void AssembleTriangle(int i)
            {
                if (!triangleCache.TryGetValue(i, out var cachedTriangle))
                {
                    return;
                }

                Vertex p0 = cachedTriangle.p0;
                Vertex p1 = cachedTriangle.p1;
                Vertex p2 = cachedTriangle.p2;

                p0.Position = Vector4.Transform(p0.Position, cameraTransform);
                p1.Position = Vector4.Transform(p1.Position, cameraTransform);
                p2.Position = Vector4.Transform(p2.Position, cameraTransform);

                Vector4 normal = Vector3.Cross((p2.Position - p0.Position).AsVector3(), (p1.Position - p0.Position).AsVector3()).AsVector4();
                float orientation = Vector4.Dot(normal, p0.Position);
                if (orientation <= 0)
                    return;

                ProjectTriangle(p0, p1, p2);
            }

            private void ProjectTriangle(Vertex p0, Vertex p1, Vertex p2)
            {
                p0.Position = Vector4.Transform(p0.Position, projectionTransform);
                p1.Position = Vector4.Transform(p1.Position, projectionTransform);
                p2.Position = Vector4.Transform(p2.Position, projectionTransform);
                CullAndClipTriangle(p0, p1, p2);
            }

            private void CullAndClipTriangle(Vertex p0, Vertex p1, Vertex p2)
            {
                if (p0.Position.X > p0.Position.W && p1.Position.X > p1.Position.W && p2.Position.X > p2.Position.W)
                    return;
                if (p0.Position.X < -p0.Position.W && p1.Position.X < -p1.Position.W && p2.Position.X < -p2.Position.W)
                    return;
                if (p0.Position.Y > p0.Position.W && p1.Position.Y > p1.Position.W && p2.Position.Y > p2.Position.W)
                    return;
                if (p0.Position.Y < -p0.Position.W && p1.Position.Y < -p1.Position.W && p2.Position.Y < -p2.Position.W)
                    return;
                if (p0.Position.Z > p0.Position.W && p1.Position.Z > p1.Position.W && p2.Position.Z > p2.Position.W)
                    return;
                if (p0.Position.Z < 0 && p1.Position.Z < 0 && p2.Position.Z < 0)
                    return;

                if (p0.Position.Z < 0)
                {
                    if (p1.Position.Z < 0)
                    {
                        ClipTriangleIntoOne(p0, p1, p2);
                    }
                    else if (p2.Position.Z < 0)
                    {
                        ClipTriangleIntoOne(p0, p2, p1);
                    }
                    else
                    {
                        ClipTriangleIntoTwo(p0, p1, p2);
                    }
                }
                else if (p1.Position.Z < 0)
                {
                    if (p2.Position.Z < 0)
                    {
                        ClipTriangleIntoOne(p1, p2, p0);
                    }
                    else
                    {
                        ClipTriangleIntoTwo(p1, p0, p2);
                    }
                }
                else if (p2.Position.Z < 0)
                {
                    ClipTriangleIntoTwo(p2, p0, p1);
                }
                else
                {
                    ProjectTriangleToViewPort(p0, p1, p2);
                }
            }

            private void ClipTriangleIntoTwo(Vertex pointBehind, Vertex p1, Vertex p2)
            {
                float c0 = (-pointBehind.Position.Z) / (p1.Position.Z - pointBehind.Position.Z);
                float c1 = (-pointBehind.Position.Z) / (p2.Position.Z - pointBehind.Position.Z);
                Vertex leftInterpolant = Vertex.Lerp(pointBehind, p1, c0);
                Vertex rightInterpolant = Vertex.Lerp(pointBehind, p2, c1);
                ProjectTriangleToViewPort(leftInterpolant, p1, p2);
                ProjectTriangleToViewPort(rightInterpolant, leftInterpolant, p2);
            }

            private void ClipTriangleIntoOne(Vertex leftPointBehind, Vertex rightPointBehind, Vertex p2)
            {
                float c0 = (-leftPointBehind.Position.Z) / (p2.Position.Z - leftPointBehind.Position.Z);
                float c1 = (-rightPointBehind.Position.Z) / (p2.Position.Z - rightPointBehind.Position.Z);
                Vertex leftInterpolant = Vertex.Lerp(leftPointBehind, p2, c0);
                Vertex rightInterpolant = Vertex.Lerp(rightPointBehind, p2, c1);
                ProjectTriangleToViewPort(leftInterpolant, p2, rightInterpolant);
            }

            private void ProjectTriangleToViewPort(Vertex p0, Vertex p1, Vertex p2)
            {
                TransformToViewPort(ref p0);
                TransformToViewPort(ref p1);
                TransformToViewPort(ref p2);
                DrawTriangle(p0, p1, p2);
            }

            private void TransformToViewPort(ref Vertex vertex)
            {
                float invZ = 1 / vertex.Position.W;
                vertex *= invZ;
                Vector4 ndcPosition = Vector4.Transform(vertex.Position, viewPortTransform);
                ndcPosition.W = invZ;
                vertex.Position = ndcPosition;
            }

            private void DrawTriangle(Vertex p0, Vertex p1, Vertex p2)
            {
                Vertex min = p0;
                Vertex mid = p1;
                Vertex max = p2;
                if (mid.Position.Y < min.Position.Y)
                {
                    (min, mid) = (mid, min);
                }
                if (max.Position.Y < min.Position.Y)
                {
                    (min, max) = (max, min);
                }
                if (max.Position.Y < mid.Position.Y)
                {
                    (mid, max) = (max, mid);
                }

                if (min.Position.Y == mid.Position.Y)
                {
                    if (mid.Position.X < min.Position.X)
                    {
                        (min, mid) = (mid, min);
                    }
                    DrawFlatTopTriangle(min, mid, max);
                }
                else if (max.Position.Y == mid.Position.Y)
                {
                    if (max.Position.X > mid.Position.X)
                    {
                        (mid, max) = (max, mid);
                    }
                    DrawFlatBottomTriangle(min, mid, max);
                }
                else
                {
                    float c = (mid.Position.Y - min.Position.Y) / (max.Position.Y - min.Position.Y);
                    Vertex interpolant = Vertex.Lerp(min, max, c);
                    if (interpolant.Position.X > mid.Position.X)
                    {
                        DrawFlatBottomTriangle(min, interpolant, mid);
                        DrawFlatTopTriangle(mid, interpolant, max);
                    }
                    else
                    {
                        DrawFlatBottomTriangle(min, mid, interpolant);
                        DrawFlatTopTriangle(interpolant, mid, max);
                    }
                }
            }

            private void DrawFlatTopTriangle(Vertex leftTopPoint, Vertex rightTopPoint, Vertex bottomPoint)
            {
                float dy = bottomPoint.Position.Y - leftTopPoint.Position.Y;
                Vertex dLeftPoint = (bottomPoint - leftTopPoint) / dy;
                Vertex dRightPoint = (bottomPoint - rightTopPoint) / dy;
                Vertex dLineInterpolant = (rightTopPoint - leftTopPoint) / (rightTopPoint.Position.X - leftTopPoint.Position.X);
                Vertex rightPoint = rightTopPoint;
                DrawTriangleScanline(leftTopPoint, rightPoint, bottomPoint.Position.Y, dLeftPoint, dRightPoint, dLineInterpolant);
            }

            private void DrawFlatBottomTriangle(Vertex topPoint, Vertex rightBottomPoint, Vertex leftBottomPoint)
            {
                float dy = rightBottomPoint.Position.Y - topPoint.Position.Y;
                Vertex dRightPoint = (rightBottomPoint - topPoint) / dy;
                Vertex dLeftPoint = (leftBottomPoint - topPoint) / dy;
                Vertex rightPoint = topPoint;
                Vertex dLineInterpolant = (rightBottomPoint - leftBottomPoint) / (rightBottomPoint.Position.X - leftBottomPoint.Position.X);
                DrawTriangleScanline(topPoint, rightPoint, rightBottomPoint.Position.Y, dLeftPoint, dRightPoint, dLineInterpolant);
            }

            private void DrawTriangleScanline(Vertex leftPoint, Vertex rightPoint, float yMax, Vertex dLeftPoint, Vertex dRightPoint, Vertex dLineInterpolant)
            {
                int yStart = Math.Max((int)Math.Ceiling(leftPoint.Position.Y), 0);
                int yEnd = Math.Min((int)Math.Ceiling(yMax), screenHeight);
                float yPrestep = yStart - leftPoint.Position.Y;

                leftPoint += dLeftPoint * yPrestep;
                rightPoint += dRightPoint * yPrestep;

                for (int y = yStart; y < yEnd; y++)
                {
                    int xStart = Math.Max((int)Math.Ceiling(leftPoint.Position.X), 0);
                    int xEnd = Math.Min((int)Math.Ceiling(rightPoint.Position.X), screenWidth);
                    float xPrestep = xStart - leftPoint.Position.X;
                    Vertex lineInterpolant = leftPoint + xPrestep * dLineInterpolant;
                    for (int x = xStart; x < xEnd; x++)
                    {
                        if (renderer.zBufferV2!.Test(x, y, lineInterpolant.Position.Z))
                        {
                            Vertex correctedPoint = lineInterpolant * (1 / lineInterpolant.Position.W);
                            renderer.zBufferV2.TestAndSet(x, y, lineInterpolant.Position.Z, shader.PixelShader(correctedPoint));
                        }
                        lineInterpolant += dLineInterpolant;
                    }
                    leftPoint += dLeftPoint;
                    rightPoint += dRightPoint;
                }
            }
        }

        public int RenderSolid()
        {
            if (Bitmap == null)
                return 0;
            if (Scene.Obj == null)
                return 0;
            Obj obj = Scene.Obj;
            ResizeAndClearZBuffer();

            Matrix4x4 worldTransform = obj.transformation.Matrix;
            Camera mainCamera = Scene.Camera;
            Matrix4x4 cameraTransform = mainCamera.ViewMatrix;
            Matrix4x4 modelToCamera = worldTransform * cameraTransform;
            mainCamera.ScreenHeight = Bitmap.PixelHeight;
            mainCamera.ScreenWidth = Bitmap.PixelWidth;
            Matrix4x4 projectionTransform = mainCamera.ProjectionMatrix;
            Matrix4x4 viewPortTransform = mainCamera.ViewPortMatrix;

            for (int i = 0; i < obj.faces.Length; i++)
            {
                Face triangle = obj.faces[i];
                Vector4 p0 = new Vector4(obj.vertices[triangle.vIndices[0]], 1);
                Vector4 p1 = new Vector4(obj.vertices[triangle.vIndices[1]], 1);
                Vector4 p2 = new Vector4(obj.vertices[triangle.vIndices[2]], 1);
                p0 = Vector4.Transform(p0, modelToCamera);
                p1 = Vector4.Transform(p1, modelToCamera);
                p2 = Vector4.Transform(p2, modelToCamera);
                Vector4 normal = Vector3.Cross((p2 - p0).AsVector3(), (p1 - p0).AsVector3()).AsVector4();
                float orientation = Vector4.Dot(normal, p0);
                if (orientation <= 0)
                    continue;
                p0 = Vector4.Transform(p0, projectionTransform);
                p1 = Vector4.Transform(p1, projectionTransform);
                p2 = Vector4.Transform(p2, projectionTransform);
                if (p0.X > p0.W && p1.X > p1.W && p2.X > p2.W)
                    continue;
                if (p0.X < -p0.W && p1.X < -p1.W && p2.X < -p2.W)
                    continue;
                if (p0.Y > p0.W && p1.Y > p1.W && p2.Y > p2.W)
                    continue;
                if (p0.Y < -p0.W && p1.Y < -p1.W && p2.Y < -p2.W)
                    continue;
                if (p0.Z > p0.W && p1.Z > p1.W && p2.Z > p2.W)
                    continue;
                if (p0.Z < 0 && p1.Z < 0 && p2.Z < 0)
                    continue;
                uint color = 0xFFFFFFFF;
                if (p0.Z < 0)
                {
                    color = 0xFF00FF00;
                    if (p1.Z < 0)
                    {
                        ClipTriangleIntoOne(p0, p1, p2);
                    }
                    else if (p2.Z < 0)
                    {
                        ClipTriangleIntoOne(p0, p2, p1);
                    }
                    else
                    {
                        ClipTriangleIntoTwo(p0, p1, p2);
                    }
                }
                else if (p1.Z < 0)
                {
                    color = 0xFF00FF00;
                    if (p2.Z < 0)
                    {
                        ClipTriangleIntoOne(p1, p2, p0);
                    }
                    else
                    {
                        ClipTriangleIntoTwo(p1, p0, p2);
                    }
                }
                else if (p2.Z < 0)
                {
                    color = 0xFF00FF00;
                    ClipTriangleIntoTwo(p2, p0, p1);
                }
                else
                {
                    ProcessTriangle(p0, p1, p2);
                }

                void ClipTriangleIntoTwo(Vector4 pointBehind, Vector4 p1, Vector4 p2)
                {
                    float c0 = (-pointBehind.Z) / (p1.Z - pointBehind.Z);
                    float c1 = (-pointBehind.Z) / (p2.Z - pointBehind.Z);
                    Vector4 leftInterpolant = Vector4.Lerp(pointBehind, p1, c0);
                    Vector4 rightInterpolant = Vector4.Lerp(pointBehind, p2, c1);
                    ProcessTriangle(leftInterpolant, p1, p2);
                    ProcessTriangle(rightInterpolant, leftInterpolant, p2);
                }

                void ClipTriangleIntoOne(Vector4 leftPointBehind, Vector4 rightPointBehind, Vector4 p2)
                {
                    float c0 = (-leftPointBehind.Z) / (p2.Z - leftPointBehind.Z);
                    float c1 = (-rightPointBehind.Z) / (p2.Z - rightPointBehind.Z);
                    Vector4 leftInterpolant = Vector4.Lerp(leftPointBehind, p2, c0);
                    Vector4 rightInterpolant = Vector4.Lerp(rightPointBehind, p2, c1);
                    ProcessTriangle(leftInterpolant, p2, rightInterpolant);
                }

                void ProcessTriangle(Vector4 p0, Vector4 p1, Vector4 p2)
                {
                    Transform(ref p0);
                    Transform(ref p1);
                    Transform(ref p2);
                    void Transform(ref Vector4 vertex)
                    {
                        float invZ = (1 / vertex.W);
                        float z = vertex.W;
                        vertex *= invZ;
                        vertex = Vector4.Transform(vertex, viewPortTransform);
                        vertex.W = z;
                    }

                    float illumination = 0f;
                    if (triangle.nIndices != null)
                    {
                        for (int i = 0; i < 3; i++)
                        {
                            Vector3 normal = obj.normals[triangle.nIndices[i]];
                            Vector3 translated = Vector3.TransformNormal(normal, worldTransform);
                            Vector3 lightDir = -(Vector3.Transform(obj.vertices[triangle.vIndices[i]], worldTransform) - Scene.Camera.Position);
                            float vertexIllumination = Vector3.Dot(lightDir, translated) / (lightDir.Length() * translated.Length());
                            illumination += vertexIllumination;
                        }
                    }
                    else
                    {
                        Span<Vector3> vectors = [obj.vertices[triangle.vIndices[0]], obj.vertices[triangle.vIndices[1]], obj.vertices[triangle.vIndices[2]]];
                        for (int i = 0; i < vectors.Length; i++)
                        {
                            vectors[i] = Vector3.Transform(vectors[i], worldTransform);
                        }
                        Vector3 normal = Vector3.Cross(vectors[2] - vectors[0], vectors[1] - vectors[0]);
                        Vector3 translated = Vector3.TransformNormal(normal, worldTransform);
                        for (int i = 0; i < 3; i++)
                        {
                            Vector3 lightDir = -(Vector3.Transform(obj.vertices[triangle.vIndices[i]], worldTransform) - Scene.Camera.Position);
                            float vertexIllumination = Vector3.Dot(lightDir, translated) / (lightDir.Length() * translated.Length());
                            illumination += vertexIllumination;
                        }
                    }
                    illumination /= 3;
                    uint rgb = (uint)(illumination * 0xFF);
                    color &= (uint)((0xFF << 24) | (rgb << 16) | (rgb << 8) | rgb);
                    Bitmap.DrawTriangleWithZBuffer((int)mainCamera.ScreenWidth, (int)mainCamera.ScreenHeight, p0, p1, p2, color, zBuffer!);
                }
            }
            return obj.faces.Length;
        }

        public int RenderCarcass()
        {
            if (Bitmap == null)
                return 0;
            if (Scene.Obj == null)
                return 0;
            Obj obj = Scene.Obj;
            ResizeBuffer(obj);

            Matrix4x4 worldTransform = obj.transformation.Matrix;
            Matrix4x4 cameraTransform = Scene.Camera.ViewMatrix;
            int width = Bitmap.PixelWidth;
            int height = Bitmap.PixelHeight;
            float aspectRatio = (float)width / height;
            float fovVertical = MathF.PI / 3;
            float nearPlaneDistance = 0.01f;
            float farPlaneDistance = float.PositiveInfinity;
            float zCoeff = (float.IsPositiveInfinity(farPlaneDistance) ? -1f : farPlaneDistance / (nearPlaneDistance - farPlaneDistance));

            Matrix4x4 projectionTransform = new Matrix4x4(
                1 / MathF.Tan(fovVertical * 0.5f) / aspectRatio, 0, 0, 0,
                0, 1 / MathF.Tan(fovVertical * 0.5f), 0, 0,
                0, 0, zCoeff, -1,
                0, 0, zCoeff * nearPlaneDistance, 0
            );

            float leftCornerX = 0;
            float leftCornerY = 0;
            Matrix4x4 viewPortTransform = new Matrix4x4(
                (float)width / 2, 0, 0, 0,
                0, -(float)height / 2, 0, 0,
                0, 0, 1, 0,
                leftCornerX + (float)width / 2, leftCornerY + (float)height / 2, 0, 1);

            Matrix4x4 modelToProjection = worldTransform * cameraTransform * projectionTransform;
            for (int i = 0; i < bufferLength; i++)
            {
                Vector4 v = new(obj.vertices[i], 1);
                v = Vector4.Transform(v, modelToProjection);
                projectionSpaceBuffer[i] = v;
            }

            Face[] faces = obj.faces;
            int facesCount = faces.Length;
            uint color = 0xFFFF0000;

            for (int i = 0; i < facesCount; i++)
            {
                Face face = faces[i];
                int[] vIndices = face.vIndices;

                for (int j = 0; j < vIndices.Length; j++)
                {
                    int p0 = vIndices[j];
                    int p1 = vIndices[(j + 1) % vIndices.Length];
                    Vector4 v0 = projectionSpaceBuffer[p0];
                    Vector4 v1 = projectionSpaceBuffer[p1];

                    if (v0.X > v0.W && v1.X > v1.W)
                        continue;
                    if (v0.X < -v0.W && v1.X < -v1.W)
                        continue;
                    if (v0.Y > v0.W && v1.Y > v1.W)
                        continue;
                    if (v0.Y < -v0.W && v1.Y < -v1.W)
                        continue;
                    if (v0.Z > v0.W && v1.Z > v1.W)
                        continue;
                    if (v0.Z < 0 && v1.Z < 0)
                        continue;

                    if (v0.Z < 0)
                    {
                        InterpolateV0(ref v0, ref v1);
                        color = 0xFF0000FF;
                    }
                    else if (v1.Z < 0)
                    {
                        InterpolateV0(ref v1, ref v0);
                        color = 0xFF0000FF;
                    }

                    static void InterpolateV0(ref Vector4 v0, ref Vector4 v1)
                    {
                        float coeff = (-v0.Z) / (v1.Z - v0.Z);
                        v0 = Vector4.Lerp(v0, v1, coeff);
                    }

                    v0 = Vector4.Transform(v0, viewPortTransform);
                    v0 *= (1 / v0.W);
                    v1 = Vector4.Transform(v1, viewPortTransform);
                    v1 *= (1 / v1.W);
                    Bitmap.DrawLine(width, height, (int)v0.X, (int)v0.Y, (int)v1.X, (int)v1.Y, color);
                }
            }
            return facesCount;
        }
    }
}