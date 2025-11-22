using UnityEngine;

namespace PMC
{
    public static class UnityUtils
    {
        public static Quaternion LookRotation(Vector3 forward, Vector3 upwards)
        {
            if (forward == Vector3.zero)
            {
                return Quaternion.identity;
            }

            return Quaternion.LookRotation(forward, upwards);
        }

        public static Vector3 TriangleNormal(Vector3 a, Vector3 b, Vector3 c)
        {
            var v = Vector3.Cross(a - b, a - c);

            return v.normalized;
        }
    }
}