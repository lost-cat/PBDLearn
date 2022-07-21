using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace DefaultNamespace
{
    public class MeshModifier
    {
        private NativeList<float3> m_Vertices;
        private NativeList<float3> m_Normals;
        private NativeList<float2> m_Uvs;

        /// <summary>
        /// 三角形下标
        /// </summary>
        private NativeList<int> m_Indices;

        /// <summary>
        /// 存储每个点所属的边
        /// </summary>
        private List<VertexExtraInfo> m_VertexExtraInfos;

        /// <summary>
        /// 存储当前mesh中的边
        /// </summary>
        private readonly Dictionary<int, Edge> m_Edges = new();

        public NativeArray<float3> Vertices => m_Vertices;
        public NativeArray<int> Indices => m_Indices;
        public IReadOnlyCollection<Edge> Edges => m_Edges.Values;
        public NativeArray<float3> Normals => m_Normals;

        private MeshModifier(NativeList<float3> vertices, NativeList<float3> normals, NativeList<float2> uvs,
            NativeList<int> indices)
        {
            m_Vertices = vertices;
            m_Normals = normals;
            m_Uvs = uvs;
            m_Indices = indices;
            BuildIndex();
        }


        private void CacheEdge(int vIndex0, int vIndex1, int triangleIndex)
        {
            var edgeHash = Edge.GetEdgeHash(vIndex0, vIndex1);
            if (m_Edges.TryGetValue(edgeHash, out var edge))
            {
                edge.TriangleIndexes.Add(triangleIndex);
            }
            else
            {
                edge = new Edge(vIndex0, vIndex1);
                m_Edges.Add(edgeHash, edge);
                edge.TriangleIndexes.Add(triangleIndex);
                m_VertexExtraInfos[vIndex0].EdgeHash.Add(edgeHash);
                m_VertexExtraInfos[vIndex1].EdgeHash.Add(edgeHash);
            }
        }

        private void BuildIndex()
        {
            m_VertexExtraInfos = new List<VertexExtraInfo>(m_Vertices.Length);
            for (var i = 0; i < m_Vertices.Length; i++)
            {
                m_VertexExtraInfos.Add(new VertexExtraInfo());
            }

            var triangleCount = m_Indices.Length / 3;
            for (var i = 0; i < triangleCount; i++)
            {
                var offset = i * 3;
                var i0 = m_Indices[offset];
                var i1 = m_Indices[offset + 1];
                var i2 = m_Indices[offset + 2];

                CacheEdge(i0, i1, i);
                CacheEdge(i1, i2, i);
                CacheEdge(i2, i0, i);
            }
        }


        public static MeshModifier CreateFromMeshFilter(MeshFilter meshFilter)
        {
            var mesh = meshFilter.sharedMesh;
            var vertices = mesh.vertices;
            var uv = mesh.uv;
            var indices = mesh.triangles;

            var verticesList = new NativeList<float3>(vertices.Length, allocator: Allocator.Persistent);
            verticesList.Resize(vertices.Length, NativeArrayOptions.UninitializedMemory);
            var localToWorld = meshFilter.transform.localToWorldMatrix;
            for (var i = 0; i < vertices.Length; i++)
            {
                verticesList[i] = localToWorld.MultiplyPoint3x4(vertices[i]);
            }

            var normals = new NativeList<float3>(vertices.Length, Allocator.Persistent);
            normals.Resize(vertices.Length, NativeArrayOptions.UninitializedMemory);

            var uvList = new NativeList<float2>(uv.Length, Allocator.Persistent);
            uvList.Resize(uv.Length, NativeArrayOptions.UninitializedMemory);
            uvList.AsArray().Reinterpret<Vector2>().CopyFrom(uv);

            var indicesList = new NativeList<int>(indices.Length, Allocator.Persistent);
            indicesList.Resize(indices.Length, NativeArrayOptions.UninitializedMemory);
            indicesList.AsArray().Reinterpret<int>().CopyFrom(indices);

            return new MeshModifier(verticesList, normals, uvList, indicesList);
        }

        /// <summary>
        /// 获取三角形中俩个点之外的另一个点
        /// </summary>
        /// <param name="triangleIndex"></param>
        /// <param name="exceptVertex0"></param>
        /// <param name="exceptVertex1"></param>
        /// <returns></returns>
        public int GetTriangleIndexExcept(int triangleIndex, int exceptVertex0, int exceptVertex1)
        {
            for (int i = 0; i < 3; i++)
            {
                var vertex = m_Indices[triangleIndex * 3 + i];
                if (vertex != exceptVertex0 && vertex != exceptVertex1)
                {
                    return vertex;
                }
            }

            return -1;
        }
        public void Dispose()
        {
            m_Vertices.Dispose();
            m_Uvs.Dispose();
            m_Indices.Dispose();
            m_Normals.Dispose();
        }
    }

    public class Edge
    {
        public readonly int VIndex0;
        public readonly int VIndex1;
        public List<int> TriangleIndexes = new(2);

        public Edge(int vIndex0, int vIndex1)
        {
            VIndex0 = math.min(vIndex0, vIndex1);
            VIndex1 = math.max(vIndex0, vIndex1);
        }


        public override string ToString()
        {
            return $"{nameof(VIndex0)}: {VIndex0}, {nameof(VIndex1)}: {VIndex1}";
        }

        private bool Equals(Edge other)
        {
            return VIndex0 == other.VIndex0 && VIndex1 == other.VIndex1;
        }

        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj)) return false;
            if (ReferenceEquals(this, obj)) return true;
            if (obj.GetType() != this.GetType()) return false;
            return Equals((Edge) obj);
        }

        public override int GetHashCode()
        {
            return (VIndex0 << 16 | VIndex1);
        }

        public static int GetEdgeHash(int vIndex0, int vIndex1)
        {
            if (vIndex0 < vIndex1)
            {
                return (vIndex0 << 16 | vIndex1);
            }
            else
            {
                return (vIndex1 << 16 | vIndex0);
            }
        }

     
    }

    public class VertexExtraInfo
    {
        public List<int> EdgeHash = new();
    }
}