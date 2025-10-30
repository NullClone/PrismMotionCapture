using UnityEditor;

namespace PMC.Editor
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(PrismTracker))]
    sealed class PrismTrackerEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            EditorGUILayout.LabelField($"{AssetInfo.AssetName} (v{AssetInfo.AssetVersion})", EditorStyles.centeredGreyMiniLabel);
            EditorGUILayout.Space();

            base.OnInspectorGUI();
        }
    }
}