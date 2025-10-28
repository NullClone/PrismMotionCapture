using System.Collections.Generic;
using UnityEngine;

namespace Mediapipe.Unity
{
    public class PoseWorldLandmarkListAnnotationController2 : AnnotationController<PoseLandmarkListAnnotation>
    {
        public float HipHeightMeter = 0.9f;
        public Vector3 Scale = new(1, 1, 1);
        public bool VisualizeZ = true;

        private IReadOnlyList<Landmark> _currentTarget;

        protected override void Start()
        {
            base.Start();

            transform.localPosition = new Vector3(0, HipHeightMeter * Scale.y, 0);
        }

        public void DrawNow(IReadOnlyList<Landmark> target)
        {
            _currentTarget = target;

            SyncNow();
        }

        public void DrawNow(LandmarkList target)
        {
            DrawNow(target?.Landmark);
        }

        public void DrawLater(IReadOnlyList<Landmark> target)
        {
            UpdateCurrentTarget(target, ref _currentTarget);
        }

        public void DrawLater(LandmarkList target)
        {
            DrawLater(target?.Landmark);
        }

        protected override void SyncNow()
        {
            isStale = false;

            annotation.Draw(_currentTarget, Scale, VisualizeZ);
        }
    }
}