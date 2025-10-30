using System.Collections.Generic;

namespace Mediapipe.Unity
{
    public class HandLandmarkListAnnotationController : AnnotationController<HandLandmarkListAnnotation>
    {
        public bool VisualizeZ = false;

        private IReadOnlyList<NormalizedLandmark> _currentTarget;

        protected override void Start()
        {
            annotation.SetHandedness(HandLandmarkListAnnotation.Hand.Left);

            base.Start();
        }

        protected override void LateUpdate()
        {
            base.LateUpdate();
        }

        public void DrawNow(IReadOnlyList<NormalizedLandmark> target)
        {
            _currentTarget = target;

            SyncNow();
        }

        public void DrawNow(NormalizedLandmarkList target)
        {
            DrawNow(target?.Landmark);
        }

        public void DrawLater(IReadOnlyList<NormalizedLandmark> target)
        {
            UpdateCurrentTarget(target, ref _currentTarget);
        }

        public void DrawLater(NormalizedLandmarkList target)
        {
            DrawLater(target?.Landmark);
        }

        protected override void SyncNow()
        {
            isStale = false;

            annotation.Draw(_currentTarget, VisualizeZ);
        }
    }
}
