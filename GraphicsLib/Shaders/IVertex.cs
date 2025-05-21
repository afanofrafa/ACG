using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace GraphicsLib.Shaders
{
    public interface IVertex<T> where T : IVertex<T>
    {
        public Vector4 Position { get; set; }
        abstract static T Lerp(T a, T b, float t);
        public static abstract T operator +(T lhs, T rhs);

        public static abstract T operator -(T lhs, T rhs);

        public static abstract T operator *(T lhs, float scalar);

        public static abstract T operator *(float scalar, T rhs);

        public static abstract T operator /(T lhs, float scalar);
    }
}
