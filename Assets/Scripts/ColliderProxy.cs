using Unity.Mathematics;
using UnityEngine;
using Util;

namespace DefaultNamespace
{
    public class ColliderProxy : MonoBehaviour, ICollider
    {
        public float ContactOffset = 0.05f;


        private Rigidbody m_Rigidbody;

        private void Awake()
        {
            m_Rigidbody = GetComponent<Rigidbody>();
        }

        public virtual bool PointInside(float3 p)
        {
            return false;
        }

        public virtual bool GetClosePoint(float3 p, out ContactInfo contact)
        {
            contact = default;
            return false;
        }

        public Rigidbody AttachBody => m_Rigidbody;

        public int EntityId => GetInstanceID();

        public virtual bool DetectCollision(float3 fromPoint, float3 toPoint, out ContactInfo contact)
        {
            if (GetClosePoint(fromPoint, out contact))
            {
                contact.Point += contact.Normal * ContactOffset;
                return true;
            }
            else if (GetClosePoint(toPoint, out contact))
            {
                contact.Point += contact.Normal * ContactOffset;
                return true;
            }

            contact = default;
            return false;
        }

        public virtual void AddSelfToGroup(ColliderGroup group)
        {
        }
    }

    public interface ICollider
    {
        public Rigidbody AttachBody { get; }
        public int EntityId { get; }

        public bool DetectCollision(float3 fromPoint, float3 toPoint, out ContactInfo contact);
        public void AddSelfToGroup(ColliderGroup group);
    }
}