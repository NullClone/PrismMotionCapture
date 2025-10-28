using System;
using UnityEngine;

namespace PMC
{
    public class OneEuroFilter
    {
        protected float minCutoff;
        protected float beta;
        protected float dcutoff;
        protected float initRate;
        private LowpassFilter xFilt;
        private LowpassFilter dxFilt;
        private bool firstTime;
        private int filterLevel;
        private float rangeParam;

        public int GetFilterLevel() => this.filterLevel;

        public OneEuroFilter(float minCutoff, float beta)
        {
            firstTime = true;
            this.minCutoff = minCutoff;
            this.beta = beta;
            xFilt = new LowpassFilter();
            dxFilt = new LowpassFilter();
            dcutoff = 1f;
        }

        public OneEuroFilter(OneEuroParams oeParams) => this.SetFilterParams(oeParams);

        public void SetFilterParams(OneEuroParams oeParams)
        {
            firstTime = true;
            minCutoff = oeParams.OE_MIN_CUTOFF;
            beta = oeParams.OE_BETA;
            initRate = oeParams.OE_RATE;
            filterLevel = oeParams.filterLevel;
            rangeParam = oeParams.rangeParam;
            xFilt = new LowpassFilter();
            dxFilt = new LowpassFilter();
            dcutoff = 1f;
        }

        public float MinCutoff
        {
            get => minCutoff;
            set => minCutoff = value;
        }

        public float Beta
        {
            get => beta;
            set => beta = value;
        }

        public float Filter(float value) => Filter(value, initRate);

        public float Filter(float value, float rate)
        {
            if (minCutoff < 0.0 || rangeParam < 0.0)
                return value;
            value *= this.rangeParam;
            float x = this.firstTime ? 0.0f : (value - this.xFilt.Last()) * rate;
            if (this.firstTime)
                this.firstTime = false;
            float cutoff = this.minCutoff + (this.beta * Math.Abs(this.dxFilt.Filter(x, this.Alpha(rate, this.dcutoff))));
            return this.xFilt.Filter(value, this.Alpha(rate, cutoff)) / this.rangeParam;
        }

        private float Alpha(float rate, float cutoff)
        {
            return (float)(1.0 / (1.0 + (1.0 / (6.2831854820251465 * (double)cutoff) / (double)(1f / rate))));
        }

        public class OneEuroParams
        {
            public int filterLevel;
            public float rangeParam;
            public float OE_MIN_CUTOFF;
            public float OE_BETA;
            public float OE_RATE;
            public const float OE_MIN_CUTOFF_MIN = 0.0067f;
            public const float OE_MIN_CUTOFF_MAX = 0.0003f;
            public const float OE_BETA_MIN = 0.0042f;
            public const float OE_BETA_MAX = 0.000225f;
            public const float OE_RATE_MIN = 0.425f;
            public const float OE_RATE_MAX = 0.425f;
            public const float FILTER_RANGE_BASE = 60f;

            public OneEuroParams(int filterLevel, float range)
            {
                this.rangeParam = (double)range == 0.0 ? -1f : 60f / range;
                this.filterLevel = filterLevel;
                if (filterLevel <= 0)
                {
                    this.OE_MIN_CUTOFF = -1f;
                    this.OE_BETA = -1f;
                    this.OE_RATE = -1f;
                }
                else
                {
                    float t = Mathf.Clamp01(filterLevel / 100f);
                    this.OE_MIN_CUTOFF = Mathf.Lerp(0.0067f, 0.0003f, t);
                    this.OE_BETA = Mathf.Lerp(0.0042f, 0.000225f, t);
                    this.OE_RATE = Mathf.Lerp(0.425f, 0.425f, t);
                }
            }
        }

        internal class LowpassFilter
        {
            protected bool firstTime;
            protected float hatXPrev;

            public LowpassFilter() => this.firstTime = true;

            public float Last() => this.hatXPrev;

            public float Filter(float x, float alpha)
            {
                float num;
                if (this.firstTime)
                {
                    this.firstTime = false;
                    num = x;
                }
                else
                    num = (float)(((double)alpha * (double)x) + ((1.0 - (double)alpha) * hatXPrev));
                this.hatXPrev = num;
                return num;
            }
        }
    }
}