using GraphicsLib.Types.GltfTypes;
using SixLabors.ImageSharp.PixelFormats;
using System.Configuration;
using System.Numerics;

namespace GraphicsLib.Types
{
    //TODO: bake more
    public class Sampler
    {
        private int width;
        private int height;
        private Rgba32[]? textureData;
        private readonly WrappingMode uWrappingMode;
        private readonly WrappingMode vWrappingMode;
        private readonly MagnificationFilterMode magnificationFilterMode;
        private readonly MinificationFilterMode minificationFilterMode;

        public Sampler(WrappingMode uWrappingMode,
            WrappingMode vWrappingMode,
            MagnificationFilterMode magnificationFilterMode,
            MinificationFilterMode minificationFilterMode)
        {
            this.uWrappingMode = uWrappingMode;
            this.vWrappingMode = vWrappingMode;
            this.magnificationFilterMode = magnificationFilterMode;
            this.minificationFilterMode = minificationFilterMode;
        }
        public void BindTexture(Rgba32[] textureData, int width, int height)
        {
            this.textureData = textureData;
            this.width = width;
            this.height = height;
        }
        public void BindTexture(GltfImage gltfImage)
        {
            var textureImage = gltfImage.ImageData;
            Rgba32[] pixels = new Rgba32[textureImage.Height * textureImage.Width];
            textureImage.CopyPixelDataTo(pixels);
            BindTexture(pixels, width, height);
        }
        public Vector4 Sample(Vector2 uv)
        {
            if (textureData == null)
            {
                throw new ConfigurationErrorsException("Texture data is not bound to sampler");
            }
            switch (uWrappingMode)
            {
                case WrappingMode.ClampToEdge:
                    uv.X = float.Clamp(uv.X, 0, 1);
                    break;
                case WrappingMode.Repeat:
                    uv.X = uv.X - MathF.Floor(uv.X);
                    break;
                case WrappingMode.MirroredRepeat:
                    uv.X = uv.X - uv.X % 2;
                    if (uv.X > 1)
                        uv.X = 2 - uv.X;
                    break;
            }
            switch (vWrappingMode)
            {
                case WrappingMode.ClampToEdge:
                    uv.Y = float.Clamp(uv.Y, 0, 1);
                    break;
                case WrappingMode.Repeat:
                    uv.Y = uv.Y - MathF.Floor(uv.Y);
                    break;
                case WrappingMode.MirroredRepeat:
                    uv.Y = uv.Y - uv.Y % 2;
                    if (uv.Y > 1)
                        uv.Y = 2 - uv.Y;
                    break;
            }
            switch (magnificationFilterMode)
            {
                case MagnificationFilterMode.Nearest:
                    {
                        int x = (int)MathF.Round((uv.X * (width - 1)));
                        int y = (int)MathF.Round((uv.Y * (height - 1)));
                        return textureData[y * width + x].ToScaledVector4();
                    }
                case MagnificationFilterMode.Linear:
                    {
                        int x0 = (int)(uv.X * (width - 1));
                        int x1 = int.Clamp(x0 + 1, 0, width - 1);
                        int y0 = (int)(uv.Y * (height - 1));
                        int y1 = int.Clamp(y0 + 1, 0, width - 1);
                        float dx = uv.X * (width - 1) - x0;
                        float dy = uv.Y * (height - 1) - y0;
                        Vector4 topLeft = textureData[y0 * width + x0].ToScaledVector4();
                        Vector4 topRight = textureData[y0 * width + x1].ToScaledVector4();
                        Vector4 bottomLeft = textureData[y1 * width + x0].ToScaledVector4();
                        Vector4 bottomRight = textureData[y1 * width + x1].ToScaledVector4();
                        Vector4 result = Vector4.Lerp(Vector4.Lerp(topLeft, topRight, dx), Vector4.Lerp(bottomLeft, bottomRight, dx), dy);
                        return result;
                    }
            }
            int x2 = (int)(uv.X * (width - 1));
            int y2 = (int)(uv.Y * (height - 1));
            return textureData[y2 * width + x2].ToScaledVector4();
        }
    }
}
