using DefaultNamespace;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Util;

[Unity.Burst.BurstCompile]
struct PositionEstimateJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float3> Positions;
    [ReadOnly] public NativeArray<float3> Normals;
    [ReadOnly] public NativeArray<float3> Velocities;
    [ReadOnly] public NativeArray<float> Masses;
    [WriteOnly] public NativeArray<float3> PredictPosition;

    public float3 FieldForce;
    public float Damp;
    public float DeltaTime;

    public void Execute(int index)
    {
        var p = Positions[index];
        var v = Velocities[index];
        var m = Masses[index];
        if (m > 0)
        {
            var normal = Normals[index];
            var fieldForceOnNormal = math.dot(FieldForce, normal) * normal;
            var v1 = v + (fieldForceOnNormal * DeltaTime + ClothSimulator.G * DeltaTime) / m;
            v1 = v1 * math.max(0, (1 - Damp * DeltaTime / m));
            var p1 = v1 * DeltaTime + p;
            PredictPosition[index] = p1;
        }
        else
        {
            PredictPosition[index] = p;
        }
    }
}

[Unity.Burst.BurstCompile]
struct NormalCalculateJob : IJob
{
    [ReadOnly] public NativeArray<int> Indices;
    [ReadOnly] public NativeArray<float3> Positions;

    public NativeArray<float3> Normals;

    public void Execute()
    {
        for (var i = 0; i < Normals.Length; i++)
        {
            Normals[i] = 0;
        }

        for (var i = 0; i < Indices.Length / 3; i++)
        {
            var offset = 3 * i;
            var i0 = Indices[offset];
            var i1 = Indices[offset + 1];
            var i2 = Indices[offset + 2];
            var p0 = Positions[i0];
            var p1 = Positions[i1];
            var p2 = Positions[i2];
            var n = math.normalize(math.cross(p1 - p0, p2 - p0));
            Normals[i0] += n;
            Normals[i1] += n;
            Normals[i2] += n;
        }

        for (var i = 0; i < Normals.Length; i++)
        {
            Normals[i] = math.normalizesafe(Normals[i]);
        }
    }
}

[Unity.Burst.BurstCompile]
struct CollisionDetectJob : IJobParallelFor
{
    [ReadOnly] public float DeltaTime;
    [ReadOnly] public NativeArray<float3> Positions;
    [ReadOnly] public NativeArray<float3> PredictPositions;

    [ReadOnly] public NativeArray<float> Masses;
    [ReadOnly] public ColliderGroup ColliderGroup;

    [WriteOnly] public NativeArray<CollisionConstraintInfo> CollisionConstraints;
    public NativeArray<ConstraintType> ConstraintTypes;
    [WriteOnly] public NativeList<RigidbodyForceApply>.ParallelWriter RigidbodyForceApplies;

    /// <summary>
    /// todo 分步 判断 Broad Phase、 Narrow Phase
    /// </summary>
    /// <param name="index"></param>
    public void Execute(int index)
    {
        var p = Positions[index];
        var predictP = PredictPositions[index];
        var spheres = ColliderGroup.SphereColliders;

        foreach (var sphere in spheres)
        {
            var s = sphere;
            if (CollisionUtil.GetClosePoint(p, s.ColliderDesc, out var contact))
            {
                Debug.Log("detect collision");
                EnableConstraint(index, ref contact, ref s.RigidBodyDesc, s.EntityId);
                return;
            }
        }

        DisableConstraint(index);
    }

    private void EnableConstraint(int index, ref ContactInfo contactInfo, ref RigidBodyDescription description,
        int entityId)
    {
        ConstraintTypes[index] |= ConstraintType.Collision;
        var constraintInfo = new CollisionConstraintInfo
        {
            Point = contactInfo.Point,
            Normal = contactInfo.Normal,
        };
        var m1 = description.Mass;
        if (m1 > 0)
        {
            var restitution = description.Restitution;
            var m0 = Masses[index];
            var v0 = (PredictPositions[index] - Positions[index]) / DeltaTime;
            Debug.Log($"m0:{m0},m1:{m1}");
            var v0Normal = math.dot(v0, contactInfo.Normal) * contactInfo.Normal;
            var v1Normal = math.dot(description.Velocity, contactInfo.Normal) * contactInfo.Normal;

            var v0NewNormal = (restitution + 1) * m1 * v1Normal + v0Normal * (m0 - restitution * m1);
            var v1NewNormal = (restitution + 1) * m0 * v0Normal + v1Normal * (m1 - restitution * m0);

            v0NewNormal /= (m1 + m0);
            v1NewNormal /= (m1 + m0);

            RigidbodyForceApplies.AddNoResize(new RigidbodyForceApply
            {
                EntityId = entityId,
                Velocity = v1NewNormal - v1Normal
            });
            constraintInfo.Velocity = v0 + (v0NewNormal - v0Normal);
        }

        CollisionConstraints[index] = constraintInfo;
    }

    private void DisableConstraint(int index)
    {
        ConstraintTypes[index] &= ~ ConstraintType.Collision;
    }
}

[Unity.Burst.BurstCompile]
struct DistanceConstraintsSolveJob : IJobFor
{
    [ReadOnly] public float CompressStiffness;
    [ReadOnly] public float StretchStiffness;
    [ReadOnly] public NativeArray<float3> PredictPositions;
    [ReadOnly] public NativeArray<float> Masses;
    [ReadOnly] public NativeArray<DistanceConstraintInfo> DistanceConstraints;

    public float Di;
    public NativeArray<float3> PositionCorrects;

    public void Execute(int index)
    {
        var constraint = DistanceConstraints[index];
        var p0 = PredictPositions[constraint.VIndex0];
        var p1 = PredictPositions[constraint.VIndex1];
        var m0 = Masses[constraint.VIndex0];
        var m1 = Masses[constraint.VIndex1];

        var currentDistance = p1 - p0;
        var normal = math.normalize(currentDistance);
        var length = math.length(currentDistance);

        var err = length - constraint.Length;
        float3 correct;
        if (err < 0)
        {
            correct = err * CompressStiffness * normal;
        }
        else
        {
            correct = err * StretchStiffness * normal;
        }

        var totalM = m0 + m1;
        PositionCorrects[constraint.VIndex0] += correct * m0 * Di / totalM;
        PositionCorrects[constraint.VIndex1] -= correct * m1 * Di / totalM;
    }
}

[Unity.Burst.BurstCompile]
struct BendConstraintSolverJob : IJobFor
{
    [ReadOnly] public NativeArray<BendConstraintInfo> BendConstraints;
    [ReadOnly] public NativeArray<float3> PredictPositions;
    [ReadOnly] public NativeArray<float> Masses;
    [ReadOnly] public float Di;
    [ReadOnly] public float BendStiffness;
    public NativeArray<float3> PositionsCorrect;


    public void Execute(int index)
    {
        var constraint = BendConstraints[index];
        var p1 = PredictPositions[constraint.VIndex0];
        var p2 = PredictPositions[constraint.VIndex1] - p1;
        var p3 = PredictPositions[constraint.VIndex2] - p1;
        var p4 = PredictPositions[constraint.VIndex3] - p1;

        var n1 = math.normalize(math.cross(p2, p3));
        var n2 = math.normalize(math.cross(p2, p4));

        var p23Length = math.length(math.cross(p2, p3));
        var p24Length = math.length(math.cross(p2, p4));

        var d = math.dot(n1, n2);
        var q3 = (math.cross(p2, n2) + d * math.cross(n1, p2)) / p23Length;
        var q4 = (math.cross(p2, n1) + d * math.cross(n2, p2)) / p24Length;
        var q2 = -(math.cross(p3, n2) + d * math.cross(n1, p3)) / p23Length -
                 (math.cross(p4, n1) + d * math.cross(n2, p4)) / p24Length;
        var q1 = -(q2 + q3 + q4);

        var w1 = 1.0f / Masses[constraint.VIndex0];
        var w2 = 1.0f / Masses[constraint.VIndex1];
        var w3 = 1.0f / Masses[constraint.VIndex2];
        var w4 = 1.0f / Masses[constraint.VIndex3];

        var sum = w1 * math.lengthsq(q1) + w2 * math.lengthsq(q2) + w3 * math.lengthsq(w3) + w4 * math.lengthsq(w4);
        sum = math.max(0.01f, sum);
        var s = -math.sqrt(1 - d * d) * (math.acos(d) - constraint.MaxAngle) / sum;
        if (math.isfinite(s))
        {
            var dp1 = s * w1 * q1 * Di * BendStiffness;
            var dp2 = s * w2 * q2 * Di * BendStiffness;
            var dp3 = s * w3 * q3 * Di * BendStiffness;
            var dp4 = s * w4 * q4 * Di * BendStiffness;

            PositionsCorrect[constraint.VIndex0] += dp1;
            PositionsCorrect[constraint.VIndex1] += dp2;
            PositionsCorrect[constraint.VIndex2] += dp3;
            PositionsCorrect[constraint.VIndex3] += dp4;
        }
    }
}

[Unity.Burst.BurstCompile]
struct FinalConstraintJobSolver : IJobParallelFor
{
    public int IterationCount;
    public int IterationIndex;
    [ReadOnly] public NativeArray<float3> Positions;

    [ReadOnly] public NativeArray<float> Masses;

    [ReadOnly] public NativeArray<PinConstraintInfo> PinConstraints;
    [ReadOnly] public NativeArray<CollisionConstraintInfo> CollisionConstraints;

    [ReadOnly] public NativeArray<ConstraintType> ActiveConstraintTypes;

    public NativeArray<float3> PositionCorrect;
    public NativeArray<float3> PredictPositions;


    public void Execute(int index)
    {
        var constraintType = ActiveConstraintTypes[index];
        var positionCorrect = PositionCorrect[index];
        //清空修正值 为下一次迭代做准备
        PositionCorrect[index] = 0f;

        if ((constraintType & ConstraintType.Pin) == ConstraintType.Pin)
        {
            PredictPositions[index] = PinConstraints[index].Position;
            return;
        }

        if ((constraintType & ConstraintType.Collision) == ConstraintType.Collision)
        {
            var position = Positions[index];
            var collisionConstraint = CollisionConstraints[index];
            var dp = collisionConstraint.Point - position;
            position += dp * (IterationIndex * 1f / IterationCount);
            PredictPositions[index] = position;
            return;
        }

        PredictPositions[index] += positionCorrect;
    }
}

[Unity.Burst.BurstCompile]
struct UpdateVelocityAndPositionJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<ConstraintType> ConstraintTypes;

    [ReadOnly] public NativeArray<float3> PredictPositions;

    [ReadOnly] public NativeArray<CollisionConstraintInfo> CollisionConstraintInfos;

    public NativeArray<float3> Positions;

    [WriteOnly] public NativeArray<float3> Velocities;
    public float DeltaTime;

    public void Execute(int index)
    {
        if ((ConstraintTypes[index] & ConstraintType.Collision) == ConstraintType.Collision)
        {
            Velocities[index] = CollisionConstraintInfos[index].Velocity;
        }
        else
        {
            Velocities[index] = (PredictPositions[index] - Positions[index]) / DeltaTime;
        }

        Positions[index] = PredictPositions[index];
    }
}