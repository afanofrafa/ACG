public class ShadowMap
{
    private readonly float[] depthBuffer;
    public int Width { get; }
    public int Height { get; }

    public ShadowMap(int width, int height)
    {
        Width = width;
        Height = height;
        depthBuffer = new float[width * height];
        Clear();
    }

    public void Clear()
    {
        Array.Fill(depthBuffer, float.MaxValue);
    }

    public bool TestAndSet(int x, int y, float depth)
    {
        if (x < 0 || x >= Width || y < 0 || y >= Height)
            return false;
        int index = y * Width + x;
        if (depth < depthBuffer[index])
        {
            depthBuffer[index] = depth;
            return true;
        }
        return false;
    }

    public float GetDepth(int x, int y)
    {
        if (x < 0 || x >= Width || y < 0 || y >= Height)
            return float.MaxValue;
        return depthBuffer[y * Width + x];
    }
}