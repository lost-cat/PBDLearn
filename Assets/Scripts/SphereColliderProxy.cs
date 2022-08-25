using Unity.Mathematics;
using UnityEngine;
using Util;

namespace DefaultNamespace
{
    [RequireComponent(typeof(SphereCollider))]
    public class SphereColliderProxy : ColliderProxy
    {
        private SphereCollider m_Collider;
        private int m_LastFrame;
        public SphereCollider Collider
        {
            get
            {
                if (!m_Collider)
                {
                    m_Collider = GetComponent<SphereCollider>();
                    return m_Collider;
                }

                return m_Collider;
            }
        }

        private SphereDescription m_Desc;

        public SphereDescription Desc
        {
            get
            {
                if (m_LastFrame == Time.frameCount)
                {
                    return m_Desc;
                }

                m_LastFrame = Time.frameCount;
                var transform1 = transform;
                m_Desc.Center = transform1.position;
                m_Desc.Radius = Collider.radius * transform1.localScale.x;
                return m_Desc;
            }
        }

        public override bool PointInside(float3 p)
        {
            return CollisionUtil.PointInside(p, Desc);
        }

        public override bool GetClosePoint(float3 p, out ContactInfo contact)
        {
            return CollisionUtil.GetClosePoint(p, Desc, out contact);
        }

        public override void AddSelfToGroup(ColliderGroup group)
        {
            RigidBodyDescription body = default;
            if (AttachBody && !AttachBody.isKinematic)
            {
                body.Mass = AttachBody.mass;
                body.Velocity = AttachBody.velocity;
                body.Restitution = 0.3f;
                
            }
            group.AddSphere(Desc, body, EntityId);
        }
    }
}