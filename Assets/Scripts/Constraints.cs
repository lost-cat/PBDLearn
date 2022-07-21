using Unity.Mathematics;

namespace DefaultNamespace
{
    public struct DistanceConstraintInfo
    {
        public int VIndex0;
        public int VIndex1;
        public float Length;
    }

    public struct CollisionConstraintInfo
    {
        public float3 Point;
        public float3 Normal;
        public float3 Velocity;
    }

    public struct PinConstraintInfo
    {
        public float3 Position;
    }

    public struct BendConstraintInfo
    {
        public int VIndex0;
        public int VIndex1;
        public int VIndex2;
        public int VIndex3;
        public float MaxAngle;
    }
}