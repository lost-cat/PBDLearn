using Unity.Mathematics;

namespace DefaultNamespace
{
    public interface IColliderDescription
    {
    }

    public struct SphereDescription : IColliderDescription
    {
        public float3 Center;
        public float Radius;
    }

    public struct RigidBodyDescription
    {
        public float Mass;
        public float Restitution;
        public float3 Velocity;
        public float AngleVelocity;
    }
}