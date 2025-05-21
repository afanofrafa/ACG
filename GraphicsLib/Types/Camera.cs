using System.Numerics;

namespace GraphicsLib.Types
{
    public class Camera
    {
        private float azimuth;
        private float polar;
        private float distance;
        private float nearClipPlane = 0.1f;
        private static readonly Vector3 up = Vector3.UnitY;

        public float Azimuth { get => azimuth; set => SetAzimuth(value); }
        public float Polar { get => polar; set => SetPolar(value); }
        public Vector3 Position { get => GetPosition(); }
        public float Distance { get => distance; set => distance = value > 0 ? value : 1; }
        public Vector3 Target { get; set; }
        public Matrix4x4 ViewMatrix { get => GetViewMatrix(); }

        public Matrix4x4 ProjectionMatrix { get => GetProjectionMatrix(); }
        public Matrix4x4 ViewPortMatrix { get => GetViewPortMatrix(); }
        //дальше лень
        public float NearClipPlane { get => nearClipPlane; set => SetNearClipPlane(value); }
        public float FarClipPlane { get; set; } = float.PositiveInfinity;
        public float FieldOfView { get; set; } = MathF.PI / 3;
        public float ScreenWidth { get; set; } = 0f;
        public float ScreenHeight { get; set; } = 0f;
        private Matrix4x4 GetViewPortMatrix()
        {
            return Matrix4x4.CreateViewport(0, 0, ScreenWidth, ScreenHeight, 0, -1);
        }

        private Matrix4x4 GetProjectionMatrix()
        {
            return Matrix4x4.CreatePerspectiveFieldOfView(FieldOfView, ScreenWidth / ScreenHeight, nearClipPlane, FarClipPlane);
        }
        private Matrix4x4 GetViewMatrix()
        {
            return Matrix4x4.CreateLookAt(Position, Target, up);
        }
        public Camera()
        {
            Azimuth = 0;
            Polar = float.Pi / 2;// float.Pi / 3;
            Distance = 1000;
            Target = Vector3.Zero;
        }
        public Camera(float azimuth, float polar, float distance, Vector3 target)
        {
            Azimuth = azimuth;
            Polar = polar;
            Distance = distance;
            Target = target;
        }
        public void UpdateViewPort(float width, float height)
        {
            ScreenHeight = height;
            ScreenWidth = width;
        }
        private void SetNearClipPlane(float value)
        {
            if (value <= 0 || value >= FarClipPlane)
                throw new ArgumentOutOfRangeException(nameof(value), value, "nearPlane must be between 0 and farPlane");
            nearClipPlane = value;
        }
        private void SetAzimuth(float value)
        {
            azimuth = value;
            int fullcircles = (int)(Azimuth / MathF.Tau);
            if (azimuth < 0)
                fullcircles--;
            azimuth -= fullcircles * MathF.Tau;
        }
        private void SetPolar(float value)
        {
            polar = value;
            if (polar > MathF.PI - 0.01f)
                polar = MathF.PI - 0.01f;
            if (polar < 0.01f)
                polar = 0.01f;
        }
        private Vector3 GetPosition()
        {
            (float sinPolar, float cosPolar) = MathF.SinCos(Polar);
            (float sinAzim, float cosAzim) = MathF.SinCos(Azimuth);
            Vector3 position = default;
            position.Z = Distance * cosAzim * sinPolar;
            position.X = Distance * sinAzim * sinPolar;
            position.Y = Distance * cosPolar;
            position += Target;
            return position;
        }
        public void RotateAroundTargetHorizontal(float angleRadians)
        {
            Azimuth += angleRadians;
        }
        public void RotateAroundTargetVertical(float angleRadians)
        {
            Polar += angleRadians;
        }
        public void MoveTowardTarget(float distance)
        {
            Distance -= distance;
        }
    }
}
