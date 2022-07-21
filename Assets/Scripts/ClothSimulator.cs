using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Util;
using static Unity.Collections.Allocator;

namespace DefaultNamespace
{
    [Flags]
    public enum ConstraintType
    {
        Distance,
        Collision,
        Pin,
    }

    public class ClothSimulator
    {
        public static readonly float3 G = new(0f, -9.8f, 0f);
        public static readonly float3 FieldForce = new(0f, 0f, 5f);
        public const float Density = 1f;
        public const float Damper = 1f;
        public const int MaxConstraintSolveCount = 8;
        public const float CompressStiffness = 0.8f;
        public const float StretchStiffness = 0.8f;
        public const float BendStiffness = 0.1f;

        private NativeList<ConstraintType> m_ConstraintTypes;
        private NativeList<float3> m_Velocities;
        private NativeList<float3> m_PredictPositions;
        private NativeArray<float3> m_PositionCorrects;
        private NativeList<float> m_Masses;

        private NativeList<DistanceConstraintInfo> m_DistanceConstraints;
        private NativeList<BendConstraintInfo> m_BendConstraints;
        private NativeList<CollisionConstraintInfo> m_CollisionConstraints;
        private NativeList<PinConstraintInfo> m_PinConstraints;
        private NativeList<RigidbodyForceApply> m_RigidbodyForceApplies;

        private ColliderGroup m_ColliderDescGroup;
        private readonly Dictionary<int, ICollider> m_ColliderProxies = new();

        private readonly MeshModifier m_Modifier;


        public int VertexCount => m_Modifier.Vertices.Length;
        public NativeArray<float3> Positions => m_Modifier.Vertices;
        public NativeArray<float3> Normals => m_Modifier.Normals;
        public int DistanceCostraintsCount => m_DistanceConstraints.Length;

        private JobHandle m_JobHandle;

        public void Step(float dt)
        {
            var handle = CalculateNormals();
            handle = EstimatePosition(handle, dt);
            handle = CollisionDetect(handle, dt);
            handle = ConstraintsSolve(handle);
            handle = UpdateVelocityAndPosition(handle, dt);
            m_JobHandle = handle;
        }

        public void FinishAllJobs()
        {
            m_JobHandle.Complete();
            RigidBodyVelocityUpdate();
        }

        private void RigidBodyVelocityUpdate()
        {
            foreach (var forceApply in m_RigidbodyForceApplies)
            {
                m_ColliderProxies[forceApply.EntityId].AttachBody
                    .AddForce(forceApply.Velocity, ForceMode.VelocityChange);
            }
        }

        private JobHandle CalculateNormals()
        {
            var job = new NormalCalculateJob
            {
                Indices = m_Modifier.Indices,
                Positions = m_Modifier.Vertices,
                Normals = m_Modifier.Normals
            };
            return job.Schedule();
        }

        private JobHandle EstimatePosition(JobHandle depend, float dt)
        {
            var job = new PositionEstimateJob
            {
                Positions = Positions,
                Normals = m_Modifier.Normals,
                Velocities = m_Velocities,
                Masses = m_Masses,
                PredictPosition = m_PredictPositions,
                FieldForce = FieldForce,
                Damp = Damper,
                DeltaTime = dt
            };
            var jobHandle = job.Schedule(Positions.Length, 64, depend);
            return jobHandle;
        }

        private void UpdateColliderDesc()
        {
            m_ColliderDescGroup.Clear();
            foreach (var (key, value) in m_ColliderProxies)
            {
                value.AddSelfToGroup(m_ColliderDescGroup);
            }
        }

        private JobHandle CollisionDetect(JobHandle depend, float dt)
        {
            UpdateColliderDesc();
            m_RigidbodyForceApplies.Clear();
            var job = new CollisionDetectJob
            {
                DeltaTime = dt,
                Positions = m_Modifier.Vertices,
                PredictPositions = m_PredictPositions,
                Masses = m_Masses,
                ColliderGroup = m_ColliderDescGroup,
                CollisionConstraints = m_CollisionConstraints,
                ConstraintTypes = m_ConstraintTypes,
                RigidbodyForceApplies = m_RigidbodyForceApplies.AsParallelWriter()
            };
            return job.Schedule(Positions.Length, 64, depend);
        }

        private JobHandle ConstraintsSolve(JobHandle depend)
        {
            var jobHandle = depend;
            for (var i = 0; i < MaxConstraintSolveCount; i++)
            {
                jobHandle = SolveDistanceConstraints(jobHandle, i);
                jobHandle = SolveBendConstraint(jobHandle, i);
                jobHandle = SolveFinalConstraintJob(jobHandle, i);
            }

            return jobHandle;
        }

        private float GetStiffnessFromIteration(float stiffness, int iterationCount)
        {
            var di = 1.0f / iterationCount;
            return 1.0f - math.pow(1 - stiffness, di);
        }

        private JobHandle SolveDistanceConstraints(JobHandle depend, int iterationIndex)
        {
            var iterationCount = MaxConstraintSolveCount;
            float di = 1.0f / (iterationCount - iterationIndex);
            var compressStiffness = GetStiffnessFromIteration(CompressStiffness, iterationCount);
            var stretchStiffness = GetStiffnessFromIteration(StretchStiffness, iterationCount);

            var job = new DistanceConstraintsSolveJob
            {
                CompressStiffness = compressStiffness,
                StretchStiffness = stretchStiffness,
                PredictPositions = m_PredictPositions,
                Masses = m_Masses,
                DistanceConstraints = m_DistanceConstraints,
                Di = di,
                PositionCorrects = m_PositionCorrects
            };
            return job.Schedule(m_DistanceConstraints.Length, depend);
        }

        private JobHandle SolveBendConstraint(JobHandle depend, int iterationIndex)
        {
            var di = 1.0f / (MaxConstraintSolveCount - iterationIndex);
            var bendStiffness = GetStiffnessFromIteration(BendStiffness, MaxConstraintSolveCount);
            var job = new BendConstraintSolverJob
            {
                BendConstraints = m_BendConstraints,
                PredictPositions = m_PredictPositions,
                Masses = m_Masses,
                Di = di,
                BendStiffness = bendStiffness,
                PositionsCorrect = m_PositionCorrects
            };
            var jobHandle = job.Schedule(m_BendConstraints.Length, depend);
            return jobHandle;
        }

        private JobHandle SolveFinalConstraintJob(JobHandle depend, int iterationIndex)
        {
            var di = 1.0f / (MaxConstraintSolveCount - iterationIndex);
            var job = new FinalConstraintJobSolver
            {
                IterationCount = MaxConstraintSolveCount,
                IterationIndex = iterationIndex,
                Positions = Positions,
                Masses = m_Masses,
                PinConstraints = m_PinConstraints,
                CollisionConstraints = m_CollisionConstraints,
                ActiveConstraintTypes = m_ConstraintTypes,
                PositionCorrect = m_PositionCorrects,
                PredictPositions = m_PredictPositions
            };
            return job.Schedule(Positions.Length, 64, depend);
        }

        private JobHandle UpdateVelocityAndPosition(JobHandle depend, float dt)
        {
            var job = new UpdateVelocityAndPositionJob
            {
                ConstraintTypes = m_ConstraintTypes,
                PredictPositions = m_PredictPositions,
                CollisionConstraintInfos = m_CollisionConstraints,
                Positions = Positions,
                Velocities = m_Velocities,
                DeltaTime = dt
            };
            return job.Schedule(m_Velocities.Length, 64, depend);
        }


        public ClothSimulator(MeshModifier modifier)
        {
            this.m_Modifier = modifier;

            var vertexCount = modifier.Vertices.Length;
            this.m_PredictPositions = new NativeList<float3>(vertexCount, Persistent);
            m_PredictPositions.AddRange(modifier.Vertices);

            m_Velocities = new NativeList<float3>(vertexCount, Persistent);
            m_Velocities.Resize(vertexCount, NativeArrayOptions.ClearMemory);

            m_CollisionConstraints = new NativeList<CollisionConstraintInfo>(vertexCount, Persistent);
            m_CollisionConstraints.Resize(vertexCount, NativeArrayOptions.ClearMemory);

            m_PinConstraints = new NativeList<PinConstraintInfo>(vertexCount, Persistent);
            m_PinConstraints.Resize(vertexCount, NativeArrayOptions.ClearMemory);

            m_RigidbodyForceApplies = new NativeList<RigidbodyForceApply>(vertexCount * 8, Persistent);

            m_ConstraintTypes = new NativeList<ConstraintType>(vertexCount, Persistent);
            m_ConstraintTypes.Resize(vertexCount, NativeArrayOptions.ClearMemory);

            m_PositionCorrects = new NativeArray<float3>(vertexCount, Persistent);

            m_ColliderDescGroup = new ColliderGroup(Persistent);

            BuildMasses();
            BuildDistanceConstraints();
            BuildBendConstraints();
        }

        private void BuildBendConstraints()
        {
            var edges = m_Modifier.Edges;
            m_BendConstraints = new NativeList<BendConstraintInfo>(edges.Count, Persistent);

            foreach (var edge in edges)
            {
                if (edge.TriangleIndexes.Count == 2)
                {
                    var v2 = m_Modifier.GetTriangleIndexExcept(edge.TriangleIndexes[0], edge.VIndex0, edge.VIndex1);
                    var v3 = m_Modifier.GetTriangleIndexExcept(edge.TriangleIndexes[1], edge.VIndex0, edge.VIndex1);
                    if (v2 < 0 || v3 < 0)
                    {
                        Debug.LogError("Mesh Error");
                        continue;
                    }

                    var bendConstraint = new BendConstraintInfo
                    {
                        VIndex0 = edge.VIndex0,
                        VIndex1 = edge.VIndex1,
                        VIndex2 = v2,
                        VIndex3 = v3
                    };
                    var p0 = Positions[bendConstraint.VIndex0];
                    var p1 = Positions[bendConstraint.VIndex1] - p0;
                    var p2 = Positions[bendConstraint.VIndex2] - p0;
                    var p3 = Positions[bendConstraint.VIndex3] - p0;

                    var n1 = math.normalize(math.cross(p1, p2));
                    var n2 = math.normalize(math.cross(p1, p3));

                    bendConstraint.MaxAngle = math.acos(math.dot(n1, n2));
                    m_BendConstraints.Add(bendConstraint);
                }
            }
        }

        private void BuildMasses()
        {
            var indices = m_Modifier.Indices;
            var vertices = m_Modifier.Vertices;
            m_Masses = new NativeList<float>(VertexCount, Persistent);
            m_Masses.Resize(VertexCount, NativeArrayOptions.ClearMemory);

            for (var i = 0; i < indices.Length / 3; i++)
            {
                var offset = i * 3;
                var i0 = indices[offset];
                var i1 = indices[offset + 1];
                var i2 = indices[offset + 2];
                var v0 = vertices[i0];
                var v1 = vertices[i1];
                var v2 = vertices[i2];
                var area = CollisionUtil.GetArea(v0, v1, v2);
                var m = area * Density;
                var m3 = m / 3f;

                m_Masses[i0] += m3;
                m_Masses[i1] += m3;
                m_Masses[i2] += m3;
            }
        }

        private void BuildDistanceConstraints()
        {
            var edges = m_Modifier.Edges;
            m_DistanceConstraints =
                new NativeList<DistanceConstraintInfo>(edges.Count + m_Modifier.Indices.Length / 6, Persistent);
            foreach (var edge in edges)
            {
                AddDistanceConstraint(edge.VIndex0, edge.VIndex1);
                if (edge.TriangleIndexes.Count == 2)
                {
                    var vIndex0 = m_Modifier.GetTriangleIndexExcept(edge.TriangleIndexes[0], edge.VIndex0, edge.VIndex1);
                    var vIndex1 = m_Modifier.GetTriangleIndexExcept(edge.TriangleIndexes[1], edge.VIndex0, edge.VIndex1);
                    AddDistanceConstraint(vIndex0,vIndex1);
                }
            }
        }

        private void AddDistanceConstraint(int vIndex0, int vIndex1)
        {
            var distance = math.distance(Positions[vIndex0], Positions[vIndex1]);
            m_ConstraintTypes[vIndex0] |= ConstraintType.Distance;
            m_ConstraintTypes[vIndex1] |= ConstraintType.Distance;
            m_DistanceConstraints.Add(new DistanceConstraintInfo
            {
                VIndex0 = vIndex0,
                VIndex1 = vIndex1,
                Length = distance
            });
        }

        public void AddPinConstraint(int vIndex, float3 position)
        {
            m_PinConstraints[vIndex] = new PinConstraintInfo
            {
                Position = position
            };
            m_ConstraintTypes[vIndex] |= ConstraintType.Pin;
        }

        public void AddCollider(ICollider collider)
        {
            m_ColliderProxies.Add(collider.EntityId, collider);
        }

        public void DisPose()
        {
            m_ConstraintTypes.Dispose();
            m_Velocities.Dispose();
            m_PredictPositions.Dispose();
            m_DistanceConstraints.Dispose();
            m_BendConstraints.Dispose();
            m_PinConstraints.Dispose();
            m_CollisionConstraints.Dispose();
            m_Masses.Dispose();
            m_Modifier.Dispose();
            m_ColliderDescGroup.Dispose();
            m_RigidbodyForceApplies.Dispose();
            m_PositionCorrects.Dispose();
        }

        public DistanceConstraintInfo GetDistanceConstraintInfo(int i)
        {
            return m_DistanceConstraints[i];
        }
    }

    internal struct RigidbodyForceApply
    {
        public int EntityId;
        public float3 Velocity;
    }

    public struct RigidBodyColliderDesc<T> where T : IColliderDescription
    {
        public int EntityId;
        public T ColliderDesc;
        public RigidBodyDescription RigidBodyDesc;
    }

    public struct ColliderGroup
    {
        private NativeList<RigidBodyColliderDesc<SphereDescription>> m_SphereColliders;

        public ColliderGroup(Allocator allocator)
        {
            m_SphereColliders = new NativeList<RigidBodyColliderDesc<SphereDescription>>(2, allocator);
        }

        public NativeList<RigidBodyColliderDesc<SphereDescription>> SphereColliders => m_SphereColliders;

        public void AddSphere(SphereDescription sphere, RigidBodyDescription body, int entityId)
        {
            m_SphereColliders.Add(new RigidBodyColliderDesc<SphereDescription>
            {
                EntityId = entityId,
                ColliderDesc = sphere,
                RigidBodyDesc = body
            });
        }

        public void Clear()
        {
            m_SphereColliders.Clear();
        }

        public void Dispose()
        {
            m_SphereColliders.Dispose();
        }
    }
}