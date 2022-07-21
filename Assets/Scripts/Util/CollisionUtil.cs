using DefaultNamespace;
using Unity.Mathematics;

namespace Util
{
    public class CollisionUtil
    {
        /// <summary>
        /// 计算三角形面积
        /// </summary>
        public static float GetArea(float3 p0, float3 p1, float3 p2)
        {
            var v1 = p1 - p0;
            var v2 = p2 - p0;
            var area = math.length(math.cross(v1, v2)) * 0.5f;
            return area;
        }

        public static bool PointInside(float3 p, SphereDescription sphereDescription)
        {
            var d = p - sphereDescription.Center;
            return math.dot(d, d) <= sphereDescription.Radius * sphereDescription.Radius;
        }
        
        /// <summary>
        /// 判断一个点是与球碰撞，发生碰撞的话返回最近点
        /// </summary>
        /// <param name="p"></param>
        /// <param name="sphereDesc"></param>
        /// <param name="contact"></param>
        /// <returns></returns>
        public static bool GetClosePoint(float3 p, SphereDescription sphereDesc, out ContactInfo contact)
        {
            var center2P = p - sphereDesc.Center;
            var distanceSqr = math.dot(center2P, center2P);
            var r2 = sphereDesc.Radius * sphereDesc.Radius;
            if (distanceSqr < r2)
            {
                contact.Normal = math.normalize(center2P);
                contact.Point = sphereDesc.Center + contact.Normal * sphereDesc.Radius;
                return true;
            }

            contact = default;
            return false;
        }
    }

    public struct ContactInfo
    {
        public float3 Point;
        public float3 Normal;
    }
}