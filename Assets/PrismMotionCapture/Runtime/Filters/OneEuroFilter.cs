using System;
using UnityEngine;

namespace PMC
{
    public class OneEuroFilter
    {
        private float freq;
        private float minCutoff;
        private float beta;
        private float dCutoff;

        private LowPassFilter x;
        private LowPassFilter dx;
        private float lasttime;

        public float CurrValue { get; protected set; }

        public float PrevValue { get; protected set; }

        private float Alpha(float cutoff)
        {
            float te = 1f / freq;
            float tau = 1f / (2f * Mathf.PI * cutoff);

            return 1f / (1f + (tau / te));
        }

        private void SetFrequency(float f)
        {
            if (f <= 0f)
            {
                Debug.LogError("freq should be > 0");

                return;
            }

            freq = f;
        }

        private void SetMinCutoff(float mc)
        {
            if (mc <= 0f)
            {
                Debug.LogError("mincutoff should be > 0");

                return;
            }

            minCutoff = mc;
        }

        private void SetBeta(float b)
        {
            beta = b;
        }

        private void SetDerivateCutoff(float dc)
        {
            if (dc <= 0f)
            {
                Debug.LogError("dcutoff should be > 0");

                return;
            }

            dCutoff = dc;
        }

        public OneEuroFilter(float freq, float minCutoff = 1f, float beta = 0f, float dCutoff = 1f)
        {
            SetFrequency(freq);
            SetMinCutoff(minCutoff);
            SetBeta(beta);
            SetDerivateCutoff(dCutoff);

            x = new LowPassFilter(Alpha(this.minCutoff));
            dx = new LowPassFilter(Alpha(this.dCutoff));
            lasttime = -1f;

            CurrValue = 0f;
            PrevValue = CurrValue;
        }

        public void UpdateParams(float freq, float minCutoff = 1f, float beta = 0f, float dCutoff = 1f)
        {
            SetFrequency(freq);
            SetMinCutoff(minCutoff);
            SetBeta(beta);
            SetDerivateCutoff(dCutoff);

            x.Alpha = Alpha(this.minCutoff);
            dx.Alpha = Alpha(this.dCutoff);
        }

        public float Filter(float value, float timestamp = -1f)
        {
            PrevValue = CurrValue;

            if (lasttime != -1f && timestamp != -1f)
            {
                freq = 1f / (timestamp - lasttime);
            }

            lasttime = timestamp;

            var dvalue = x.IsInitialized ? (value - x.LastRawValue) * freq : 0f;
            var edvalue = dx.FilterWithAlpha(dvalue, Alpha(dCutoff));

            var cutoff = minCutoff + (beta * Mathf.Abs(edvalue));

            CurrValue = x.FilterWithAlpha(value, Alpha(cutoff));

            return CurrValue;
        }
    }

    public class OneEuroFilter<T> where T : struct
    {
        private readonly Func<T, float, T> filterDelegate;
        private readonly OneEuroFilter[] oneEuroFilters;

        public float Freq { get; protected set; }

        public float MinCutoff { get; protected set; }

        public float Beta { get; protected set; }

        public float DCutoff { get; protected set; }

        public T CurrValue { get; protected set; }

        public T PrevValue { get; protected set; }

        public OneEuroFilter(float freq, float minCutoff = 1f, float beta = 0f, float dCutoff = 1f)
        {
            Freq = freq;
            MinCutoff = minCutoff;
            Beta = beta;
            DCutoff = dCutoff;

            CurrValue = new T();
            PrevValue = new T();

            if (typeof(T) == typeof(Vector2))
            {
                oneEuroFilters = new OneEuroFilter[2];
                filterDelegate = FilterVector2;
            }
            else if (typeof(T) == typeof(Vector3))
            {
                oneEuroFilters = new OneEuroFilter[3];
                filterDelegate = FilterVector3;
            }
            else if (typeof(T) == typeof(Vector4))
            {
                oneEuroFilters = new OneEuroFilter[4];
                filterDelegate = FilterVector4;
            }
            else if (typeof(T) == typeof(Quaternion))
            {
                oneEuroFilters = new OneEuroFilter[4];
                filterDelegate = FilterQuaternion;
            }
            else
            {
                Debug.LogError($"[OneEuroFilter<{typeof(T)}>] {typeof(T)} はサポートされていない型です。");

                filterDelegate = (v, t) => v;

                return;
            }

            for (int i = 0; i < oneEuroFilters.Length; i++)
            {
                oneEuroFilters[i] = new OneEuroFilter(Freq, MinCutoff, Beta, DCutoff);
            }
        }

        public void UpdateParams(float freq, float minCutoff = 1f, float beta = 0f, float dCutoff = 1f)
        {
            Freq = freq;
            MinCutoff = minCutoff;
            Beta = beta;
            DCutoff = dCutoff;

            for (int i = 0; i < oneEuroFilters.Length; i++)
            {
                oneEuroFilters[i].UpdateParams(Freq, MinCutoff, Beta, DCutoff);
            }
        }

        public T Filter(T value, float timestamp = -1f)
        {
            PrevValue = CurrValue;

            CurrValue = filterDelegate(value, timestamp);

            return CurrValue;
        }

        private T FilterVector2(T value, float timestamp)
        {
            var output = Vector2.zero;
            var input = (Vector2)(object)value;

            for (int i = 0; i < oneEuroFilters.Length; i++)
            {
                output[i] = oneEuroFilters[i].Filter(input[i], timestamp);
            }

            return (T)(object)output;
        }

        private T FilterVector3(T value, float timestamp)
        {
            var output = Vector3.zero;
            var input = (Vector3)(object)value;

            for (int i = 0; i < oneEuroFilters.Length; i++)
            {
                output[i] = oneEuroFilters[i].Filter(input[i], timestamp);
            }

            return (T)(object)output;
        }

        private T FilterVector4(T value, float timestamp)
        {
            var output = Vector4.zero;
            var input = (Vector4)(object)value;

            for (int i = 0; i < oneEuroFilters.Length; i++)
            {
                output[i] = oneEuroFilters[i].Filter(input[i], timestamp);
            }

            return (T)(object)output;
        }

        private T FilterQuaternion(T value, float timestamp)
        {
            var output = Quaternion.identity;
            var input = (Quaternion)(object)value;

            var prevQ = new Vector4(oneEuroFilters[0].CurrValue, oneEuroFilters[1].CurrValue, oneEuroFilters[2].CurrValue, oneEuroFilters[3].CurrValue);
            var currQ = new Vector4(input.x, input.y, input.z, input.w);

            if (Vector4.SqrMagnitude(prevQ.normalized - currQ.normalized) > 2.0f)
            {
                input = new Quaternion(-input.x, -input.y, -input.z, -input.w);
            }

            output.x = oneEuroFilters[0].Filter(input.x, timestamp);
            output.y = oneEuroFilters[1].Filter(input.y, timestamp);
            output.z = oneEuroFilters[2].Filter(input.z, timestamp);
            output.w = oneEuroFilters[3].Filter(input.w, timestamp);

            return (T)(object)output.normalized;
        }
    }
}