using System;
using System.Runtime.InteropServices;
using System.Windows.Media.Imaging;

namespace GraphicsLib.Types
{
    public class GBuffer
    {
        public int Width { get => width; private set => width = value; }
        public int Height { get => height; private set => height = value; }
        private float[] depths;
        private int[] triangleIndices;
        private uint[] colors;
        private int width;
        private int height;

        public GBuffer(int width, int height)
        {
            this.Width = width;
            this.Height = height;
            depths = new float[width * height];
            triangleIndices = new int[width * height];
            colors = new uint[width * height];
            Clear();
        }
        public void Clear()
        {
            Array.Fill<int>(triangleIndices, -1);
            Array.Fill<float>(depths, 1f);
            Array.Fill<uint>(colors, 0);
        }
        public int TriangleIndex(int x, int y)
        {
            if (x < 0 || x >= width || y < 0 || y >= height)
            {
                throw new ArgumentOutOfRangeException("x or y out of buffer range");
            }
            return triangleIndices[y * width + x];
        }
        public bool TestAndSet(int x, int y, float depth, int triangleIndex)
        {

            if (x < 0 || x >= width || y < 0 || y >= height)
            {
                throw new ArgumentOutOfRangeException("x or y out of buffer range");
            }
            int pos = y * width + x;
            SpinWait spinWait = new();
            while (true)
            {
                float currentDepth = depths[pos];
                if (currentDepth < depth)
                {
                    return false;
                }
                if (Interlocked.CompareExchange(ref depths[pos], depth, currentDepth) == currentDepth)
                {
                    triangleIndices[pos] = triangleIndex;
                    return true;
                }
                spinWait.SpinOnce();
            }
        }
        public uint GetColor(int x, int y)
        {
            if (x < 0 || x >= width || y < 0 || y >= height)
            {
                throw new ArgumentOutOfRangeException("x or y out of buffer range");
            }
            return colors[y * width + x];
        }
        public void SetColor(int x, int y, uint color)
        {
            if (x < 0 || x >= width || y < 0 || y >= height)
            {
                throw new ArgumentOutOfRangeException("x or y out of buffer range");
            }
            colors[y * width + x] = color;
        }
        public void FlushToBitmap(WriteableBitmap bitmap)
        {
            unsafe
            {
                uint* ptr = (uint*)bitmap.BackBuffer;
                int length = Width * Height;
                for (int i = 0; i < length; i++)
                {
                    ptr[i] = colors[i];
                }
            }
        }
    }
}
