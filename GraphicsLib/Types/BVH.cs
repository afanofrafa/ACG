using System.Numerics;
using GraphicsLib.Primitives;

namespace GraphicsLib
{
    public class BVHNode
    {
        public Vector3 Min;
        public Vector3 Max;
        public List<StaticTriangle> Triangles;
        public BVHNode Left;
        public BVHNode Right;
        public bool IsLeaf => Triangles != null;

        public BVHNode()
        {
            Triangles = new List<StaticTriangle>();
        }

        public bool IntersectAABB(Vector3 origin, Vector3 direction)
        {
            float tMin = 0.0f;
            float tMax = float.MaxValue;

            for (int i = 0; i < 3; i++)
            {
                float invD = 1.0f / direction[i];
                float t0 = (Min[i] - origin[i]) * invD;
                float t1 = (Max[i] - origin[i]) * invD;

                if (invD < 0.0f)
                    (t0, t1) = (t1, t0);

                tMin = Math.Max(tMin, t0);
                tMax = Math.Min(tMax, t1);

                if (tMax <= tMin)
                    return false;
            }

            return true;
        }
    }

    public class BVH
    {
        private BVHNode root;
        private StaticTriangle[] cachedTriangles; // Для получения индекса треугольника

        public BVH(StaticTriangle[] triangles)
        {
            cachedTriangles = triangles;
            root = BuildBVH(triangles, 0, triangles.Length, 0);
        }

        private BVHNode BuildBVH(StaticTriangle[] triangles, int start, int end, int depth)
        {
            BVHNode node = new BVHNode();

            Vector3 min = new Vector3(float.MaxValue);
            Vector3 max = new Vector3(float.MinValue);
            for (int i = start; i < end; i++)
            {
                min = Vector3.Min(min, triangles[i].AABBMin);
                max = Vector3.Max(max, triangles[i].AABBMax);
            }
            node.Min = min;
            node.Max = max;

            int count = end - start;
            if (count <= 4 || depth > 20)
            {
                node.Triangles.AddRange(triangles[start..end]);
                return node;
            }

            Vector3 extent = max - min;
            int axis = extent.X > extent.Y ? (extent.X > extent.Z ? 0 : 2) : (extent.Y > extent.Z ? 1 : 2);
            float split = (min[axis] + max[axis]) * 0.5f;

            int mid = Partition(triangles, start, end, axis, split);

            node.Left = BuildBVH(triangles, start, mid, depth + 1);
            node.Right = BuildBVH(triangles, mid, end, depth + 1);

            return node;
        }

        private int Partition(StaticTriangle[] triangles, int start, int end, int axis, float split)
        {
            int i = start;
            for (int j = start; j < end; j++)
            {
                float centroid = (triangles[j].AABBMin[axis] + triangles[j].AABBMax[axis]) * 0.5f;
                if (centroid < split)
                {
                    (triangles[i], triangles[j]) = (triangles[j], triangles[i]);
                    i++;
                }
            }
            return i == start ? i + 1 : i;
        }

        public bool IntersectRay(Vector3 origin, Vector3 direction, float maxDistance, int skipTriangleIndex, out float t)
        {
            t = float.MaxValue;
            return IntersectNode(root, origin, direction, maxDistance, skipTriangleIndex, ref t);
        }

        private bool IntersectNode(BVHNode node, Vector3 origin, Vector3 direction, float maxDistance, int skipTriangleIndex, ref float t)
        {
            if (!node.IntersectAABB(origin, direction))
                return false;

            if (node.IsLeaf)
            {
                bool hit = false;
                for (int i = 0; i < node.Triangles.Count; i++)
                {
                    int triangleIndex = Array.IndexOf(cachedTriangles, node.Triangles[i]);
                    if (triangleIndex == skipTriangleIndex) continue;
                    if (IntersectTriangle(origin, direction, node.Triangles[i], out float tTri))
                    {
                        if (tTri < t && tTri < maxDistance)
                        {
                            t = tTri;
                            hit = true;
                        }
                    }
                }
                return hit;
            }

            bool hitLeft = IntersectNode(node.Left, origin, direction, maxDistance, skipTriangleIndex, ref t);
            bool hitRight = IntersectNode(node.Right, origin, direction, maxDistance, skipTriangleIndex, ref t);
            return hitLeft || hitRight;
        }

        private bool IntersectTriangle(Vector3 origin, Vector3 direction, StaticTriangle triangle, out float t)
        {
            t = 0f;
            const float EPSILON = 0.000001f;

            Vector3 v0 = triangle.position0;
            Vector3 v1 = triangle.position1;
            Vector3 v2 = triangle.position2;

            Vector3 edge1 = v1 - v0;
            Vector3 edge2 = v2 - v0;

            Vector3 h = Vector3.Cross(direction, edge2);
            float a = Vector3.Dot(edge1, h);

            if (a > -EPSILON && a < EPSILON)
                return false;

            float f = 1.0f / a;
            Vector3 s = origin - v0;
            float u = f * Vector3.Dot(s, h);

            if (u < 0.0f || u > 1.0f)
                return false;

            Vector3 q = Vector3.Cross(s, edge1);
            float v = f * Vector3.Dot(direction, q);

            if (v < 0.0f || u + v > 1.0f)
                return false;

            t = f * Vector3.Dot(edge2, q);
            return t > EPSILON;
        }
    }
}