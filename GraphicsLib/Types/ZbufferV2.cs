using System.Numerics;
using System.Runtime.InteropServices;

namespace GraphicsLib.Types
{
    [StructLayout(LayoutKind.Explicit)]
    public struct PixelData
    {
        [FieldOffset(0)]
        public UInt64 data;
        [FieldOffset(0)]
        public float depth = float.PositiveInfinity;
        [FieldOffset(4)]
        public uint color = 0;
        public PixelData()
        {
        }
        public PixelData(float depth, uint color)
        {
            this.depth = depth;
            this.color = color;
        }
        public override bool Equals(object? obj)
        {
            if (obj is PixelData other)
            {
                return depth == other.depth && color == other.color;
            }
            return false;
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(depth, color);
        }
    }
    public class ZbufferV2
    {
        public int Width { get => width; private set => width = value; }
        public int Height { get => height; private set => height = value; }
        private PixelData[] buffer;
        private int width;
        private int height;

        public ZbufferV2(int width, int height)
        {
            this.Width = width;
            this.Height = height;
            buffer = new PixelData[width * height];
            Clear();
        }
        public void Clear()
        {
            Array.Fill<PixelData>(buffer, new PixelData());

        }
        public PixelData At(int pos)
        {
            return buffer[pos];
        }
        public PixelData this[int x, int y]
        {
            get
            {
                if (x < 0 || x >= width || y < 0 || y >= height)
                {
                    throw new ArgumentOutOfRangeException("x or y out of buffer range");
                }
                return buffer[y * width + x];
            }
            set
            {
                if (x < 0 || x >= width || y < 0 || y >= height)
                {
                    throw new ArgumentOutOfRangeException("x or y out of buffer range");
                }
                buffer[y * Width + x] = value;
            }
        }
        public bool Test(int x, int y, float depth)
        {
            if (x < 0 || x >= width || y < 0 || y >= height)
            {
                throw new ArgumentOutOfRangeException("x or y out of buffer range");
            }
            int pos = y * width + x;
            PixelData pixelData = buffer[pos];
            return depth <= pixelData.depth;
        }
        public bool TestAndSet(int x, int y, float depth, uint color)
        {
            if ((color >> 24 & 0xFF) == 0)
                return false;
            if (x < 0 || x >= width || y < 0 || y >= height)
            {
                throw new ArgumentOutOfRangeException("x or y out of buffer range");
            }
            int pos = y * width + x;
            SpinWait spinWait = new();
            PixelData pixelData = default;
            pixelData.color = color;
            pixelData.depth = depth;
            while (true)
            {
                PixelData currentPixel = buffer[pos];
                if (currentPixel.depth < depth)
                {
                    return false;
                }
                if (Interlocked.CompareExchange(ref buffer[pos].data, pixelData.data, currentPixel.data) == currentPixel.data)
                {
                    return true;
                }
                spinWait.SpinOnce();
            }
        }
    }
}
