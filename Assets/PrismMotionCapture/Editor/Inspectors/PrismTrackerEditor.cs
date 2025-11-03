using UnityEditor;

namespace PMC.Editor
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(PrismTracker))]
    sealed class PrismTrackerEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();
        }
    }
}