namespace GraphicsLib.Types
{
    public class Zbuffer
    {
        public int Width { get => width; private set => width = value; }
        public int Height { get => height; private set => height = value; }
        private float[] buffer;
        private int width;
        private int height;

        public Zbuffer(int width, int height)
        {
            this.Width = width;
            this.Height = height;
            buffer = new float[width * height];
            Clear();
        }
        public void Clear()
        {
            unsafe
            {
                fixed (float* ptr = buffer)
                {
                    int length = buffer.Length;
                    for (int i = 0; i < length; i++)
                    {
                        ptr[i] = float.PositiveInfinity;
                    }
                }
            }

        }
        public float this[int x, int y]
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
        public bool TestAndSet(int x, int y, float depth)
        {
            if (x < 0 || x >= width || y < 0 || y >= height)
            {
                throw new ArgumentOutOfRangeException("x or y out of buffer range");
            }
            int pos = y * width + x;
            
            float currentdepth = buffer[pos];
            if (currentdepth > depth)
            {
                buffer[pos] = depth;
                return true;
            } else
            {
                return false;
            }

        }
    }
}
