using UnityEngine;

namespace PMC
{
    public class TimeInterpolate
    {
        private float updateT;
        private double lastT;
        private double currentT;
        private int gotData;

        public void UpdateTime(double nowT)
        {
            this.updateT = Time.time;
            this.lastT = this.currentT;
            this.currentT = nowT;
            if (this.gotData >= 2) return;
            ++this.gotData;
        }

        public float Interpolate()
        {
            if (this.gotData < 2) return 0.0f;
            float timeDiff = (float)(this.currentT - this.lastT);
            if (timeDiff <= 0) return 0.0f;
            return Mathf.Min((Time.time - this.updateT) / timeDiff, 1.0f);
        }
    }
}