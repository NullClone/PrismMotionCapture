using UnityEngine;

namespace PMC
{
    public class LowPassFilter
    {
        // Fields

        private float a;
        private float lastFilteredValue;


        // Properties

        public float Alpha
        {
            get { return a; }
            set
            {
                a = Mathf.Clamp01(value);
            }
        }

        public float LastRawValue { get; private set; }

        public bool IsInitialized { get; private set; }


        // Methods

        public LowPassFilter(float alpha, float initval = 0f)
        {
            LastRawValue = lastFilteredValue = initval;
            Alpha = alpha;
            IsInitialized = false;
        }


        public float Filter(float value)
        {
            float result;

            if (IsInitialized)
            {
                result = (a * value) + ((1.0f - a) * lastFilteredValue);
            }
            else
            {
                result = value;

                IsInitialized = true;
            }

            LastRawValue = value;
            lastFilteredValue = result;

            return result;
        }

        public float FilterWithAlpha(float value, float alpha)
        {
            Alpha = alpha;

            return Filter(value);
        }
    }
}