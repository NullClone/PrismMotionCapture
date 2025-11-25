using UnityEngine;

namespace PMC
{
    public class TimeInterpolate
    {
        // Fields

        private float updateT;
        private double lastT;
        private double currentT;
        private int gotData;


        // Methods

        public void UpdateTime(double nowT)
        {
            updateT = Time.time;
            lastT = currentT;
            currentT = nowT;

            if (gotData < 2)
            {
                ++gotData;
            }
        }

        public float Interpolate()
        {
            if (gotData < 2)
            {
                return 0f;
            }

            var timeDiff = (float)(currentT - lastT);

            if (timeDiff <= 0)
            {
                return 0f;
            }

            return Mathf.Min((Time.time - updateT) / timeDiff, 1.0f);
        }
    }
}