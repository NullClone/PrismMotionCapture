using UnityEditor;
using UnityEngine;

namespace PMC.Editor
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(PrismAvatar))]
    sealed class PrismAvatarEditor : UnityEditor.Editor
    {
        private SerializedProperty m_Script;

        private SerializedProperty _tracker;
        private SerializedProperty _IKType;
        private SerializedProperty _enableTwistRelaxer;
        private SerializedProperty _enableMovement;
        private SerializedProperty _autoWeight;
        private SerializedProperty _landmarkScale;
        private SerializedProperty _handRotationOffset;
        private SerializedProperty _enableKalmanFilter;
        private SerializedProperty _timeInterval;
        private SerializedProperty _noise;

        private UI.Section[] sections;

        private void OnEnable()
        {
            m_Script = serializedObject.FindProperty(nameof(m_Script));

            _tracker = serializedObject.FindProperty(nameof(_tracker));
            _IKType = serializedObject.FindProperty(nameof(_IKType));
            _enableTwistRelaxer = serializedObject.FindProperty(nameof(_enableTwistRelaxer));
            _enableMovement = serializedObject.FindProperty(nameof(_enableMovement));
            _autoWeight = serializedObject.FindProperty(nameof(_autoWeight));
            _landmarkScale = serializedObject.FindProperty(nameof(_landmarkScale));
            _handRotationOffset = serializedObject.FindProperty(nameof(_handRotationOffset));
            _enableKalmanFilter = serializedObject.FindProperty(nameof(_enableKalmanFilter));
            _timeInterval = serializedObject.FindProperty(nameof(_timeInterval));
            _noise = serializedObject.FindProperty(nameof(_noise));

            sections = new UI.Section[]
            {
                new(this, "GENERAL", new GUIContent("General")),
                new(this, "IK", new GUIContent("IK")),
                new(this, "FILTER", new GUIContent("Filter")),
            };
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            using (new EditorGUI.DisabledScope(true))
            {
                EditorGUILayout.PropertyField(m_Script, true);
            }

            EditorGUILayout.Space();

            sections[0].DrawHeader();

            if (EditorGUILayout.BeginFadeGroup(sections[0].anim.faded))
            {
                EditorGUILayout.Space();
                EditorGUILayout.PropertyField(_tracker);
                EditorGUILayout.Space();
                EditorGUILayout.PropertyField(_landmarkScale);
                EditorGUILayout.PropertyField(_handRotationOffset);
                EditorGUILayout.Space();
            }

            EditorGUILayout.EndFadeGroup();

            sections[1].DrawHeader();

            if (EditorGUILayout.BeginFadeGroup(sections[1].anim.faded))
            {
                EditorGUILayout.Space();
                EditorGUILayout.PropertyField(_IKType);
                EditorGUILayout.PropertyField(_enableTwistRelaxer);
                EditorGUILayout.PropertyField(_enableMovement);
                EditorGUILayout.PropertyField(_autoWeight);
                EditorGUILayout.Space();
            }

            EditorGUILayout.EndFadeGroup();

            sections[2].DrawHeader();

            if (EditorGUILayout.BeginFadeGroup(sections[2].anim.faded))
            {
                EditorGUILayout.Space();
                EditorGUILayout.PropertyField(_enableKalmanFilter);
                EditorGUILayout.PropertyField(_timeInterval);
                EditorGUILayout.PropertyField(_noise);
                EditorGUILayout.Space();
            }

            EditorGUILayout.EndFadeGroup();

            //UI.DrawSplitter();

            EditorGUILayout.Space();
            EditorGUILayout.LabelField($"{AssetInfo.AssetName} (v{AssetInfo.AssetVersion})", EditorStyles.centeredGreyMiniLabel);

            serializedObject.ApplyModifiedProperties();
        }
    }
}