using GraphicsLib.Types;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace GraphicsLib.Primitives {
    public struct StaticTriangle {

        public Material material;
        public Vector3 position0, position1, position2;
        public Vector3 normal0, normal1, normal2;
        public Vector2 uvCoordinate0, uvCoordinate1, uvCoordinate2;
        public Vector2 normalUvCoordinate0, normalUvCoordinate1, normalUvCoordinate2;
        public Vector4 tangent0, tangent1, tangent2;
        public Vector2 roughnessUvCoordinate0, roughnessUvCoordinate1, roughnessUvCoordinate2;
        public Vector3 AABBMin;
        public Vector3 AABBMax;

        public void ComputeAABB()
        {
            AABBMin = Vector3.Min(position0, Vector3.Min(position1, position2));
            AABBMax = Vector3.Max(position0, Vector3.Max(position1, position2));
        }
        public static StaticTriangle FromFace(Obj obj, Face face)
        {
            StaticTriangle triangle = new StaticTriangle();
            triangle.position0 = obj.vertices[face.vIndices[0]];
            triangle.position1 = obj.vertices[face.vIndices[1]];
            triangle.position2 = obj.vertices[face.vIndices[2]];

            if (face.nIndices != null)
            {
                triangle.normal0 = obj.normals[face.nIndices[0]];
                triangle.normal1 = obj.normals[face.nIndices[1]];
                triangle.normal2 = obj.normals[face.nIndices[2]];
            }

            if (face.tIndices != null)
            {
                triangle.uvCoordinate0 = obj.uvs[face.tIndices[0]];
                triangle.uvCoordinate1 = obj.uvs[face.tIndices[1]];
                triangle.uvCoordinate2 = obj.uvs[face.tIndices[2]];
            }

            if (face.tangentIndicies != null)
            {
                triangle.tangent0 = obj.tangents[face.tangentIndicies[0]];
                triangle.tangent1 = obj.tangents[face.tangentIndicies[1]];
                triangle.tangent2 = obj.tangents[face.tangentIndicies[2]];
            }

            if (face.ntIndicies != null)
            {
                triangle.normalUvCoordinate0 = obj.normalUvs[face.ntIndicies[0]];
                triangle.normalUvCoordinate1 = obj.normalUvs[face.ntIndicies[1]];
                triangle.normalUvCoordinate2 = obj.normalUvs[face.ntIndicies[2]];
            }

            if (face.rtIndicies != null)
            {
                triangle.roughnessUvCoordinate0 = obj.roughnessUvs[face.rtIndicies[0]];
                triangle.roughnessUvCoordinate1 = obj.roughnessUvs[face.rtIndicies[1]];
                triangle.roughnessUvCoordinate2 = obj.roughnessUvs[face.rtIndicies[2]];
            }

            triangle.material = obj.materials[face.MaterialIndex];
            return triangle;
        }
    }
}
