using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace GraphicsLib.Types {
    public struct Objtransformation {

        public float _angleX;
        public float _angleY;
        public float _angleZ;
        public float _scale = 1.0f;

        public float AngleX {
            get {
                return _angleX;
            }
            set {
                _angleX = value;
            }
        }

        public float AngleY;
        public float AngleZ;
        public float Scale {
            get {
                return _scale;
            }
            set {
                _scale = value;
                if (_scale < 0.1f)
                    _scale = 0.1f;
            }
        }

        public Vector3 Offset;
        public Matrix4x4 Matrix { get => GettransformationMatrix(); }
        public Matrix4x4 NormalMatrix { get => GetNormalMatrix(); }

        private Matrix4x4 GetNormalMatrix()
        {
            Matrix4x4 rotateXMatrix = Matrix4x4.CreateRotationX(AngleX);
            Matrix4x4 rotateYMatrix = Matrix4x4.CreateRotationY(AngleY);
            Matrix4x4 rotateZMatrix = Matrix4x4.CreateRotationZ(AngleZ);
            Matrix4x4 invScaleMatrix = Matrix4x4.CreateScale(1f/Scale);
            Matrix4x4 matrix = rotateZMatrix * rotateYMatrix * rotateXMatrix * invScaleMatrix;
            return matrix;
        }

        public Objtransformation() { }

        public Matrix4x4 GettransformationMatrix() {
            Matrix4x4 matrix;
            Matrix4x4 translationMatrix = new Matrix4x4(
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                Offset.X, Offset.Y, Offset.Z, 1
            );
            Matrix4x4 rotateXMatrix = new Matrix4x4(
                1, 0, 0, 0,
                0, MathF.Cos(AngleX), MathF.Sin(AngleX), 0,
                0, -MathF.Sin(AngleX), MathF.Cos(AngleX), 0,
                0, 0, 0, 1
            );
            Matrix4x4 rotateYMatrix = new Matrix4x4(
                MathF.Cos(AngleY), 0, -MathF.Sin(AngleY), 0,
                0, 1, 0, 0,
                MathF.Sin(AngleY), 0, MathF.Cos(AngleY), 0,
                0, 0, 0, 1
            );
            Matrix4x4 rotateZMatrix = new Matrix4x4(
                MathF.Cos(AngleZ), MathF.Sin(AngleZ), 0, 0,
                -MathF.Sin(AngleZ), MathF.Cos(AngleZ), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1
            );
            Matrix4x4 scaleMatrix = new Matrix4x4(
                Scale, 0, 0, 0,
                0, Scale, 0, 0,
                0, 0, Scale, 0,
                0, 0, 0, 1
            );
            matrix =  rotateZMatrix * rotateYMatrix * rotateXMatrix * scaleMatrix * translationMatrix;
            return matrix;
        }

        public void Reset() {
            AngleX = 0.0f;
            AngleY = 0.0f;
            AngleZ = 0.0f;
            Scale = 1.0f;
            Offset = new Vector3();
        }
    }
}
