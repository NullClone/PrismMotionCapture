using UnityEditor;

namespace PMC.Editor
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(PrismAvatar))]
    sealed class PrismAvatarEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            EditorGUILayout.LabelField($"{AssetInfo.AssetName} (version {AssetInfo.AssetVersion})", EditorStyles.centeredGreyMiniLabel);
            EditorGUILayout.Space();

            base.OnInspectorGUI();
        }
    }
}