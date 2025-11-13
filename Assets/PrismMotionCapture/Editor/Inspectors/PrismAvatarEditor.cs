using UnityEditor;

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
        private SerializedProperty _fingerResetSpeed;
        private SerializedProperty _weightSmoothingSpeed;
        private SerializedProperty _targetRotationSmoothSpeed;
        private SerializedProperty _landmarkScale;
        private SerializedProperty _handRotationOffset;
        private SerializedProperty _enableKalmanFilter;
        private SerializedProperty _timeInterval;
        private SerializedProperty _noise;
        private SerializedProperty _enableOneEuroFilter;
        private SerializedProperty _filterFrequency;
        private SerializedProperty _filterMinCutoff;
        private SerializedProperty _filterBeta;
        private SerializedProperty _filterDcutoff;

        private static bool _IKSettingsFoldout;
        private static bool _filterSettingsFoldout;

        private void OnEnable()
        {
            m_Script = serializedObject.FindProperty(nameof(m_Script));
            _tracker = serializedObject.FindProperty(nameof(_tracker));
            _IKType = serializedObject.FindProperty(nameof(_IKType));
            _enableTwistRelaxer = serializedObject.FindProperty(nameof(_enableTwistRelaxer));
            _enableMovement = serializedObject.FindProperty(nameof(_enableMovement));
            _autoWeight = serializedObject.FindProperty(nameof(_autoWeight));
            _fingerResetSpeed = serializedObject.FindProperty(nameof(_fingerResetSpeed));
            _weightSmoothingSpeed = serializedObject.FindProperty(nameof(_weightSmoothingSpeed));
            _targetRotationSmoothSpeed = serializedObject.FindProperty(nameof(_targetRotationSmoothSpeed));
            _landmarkScale = serializedObject.FindProperty(nameof(_landmarkScale));
            _handRotationOffset = serializedObject.FindProperty(nameof(_handRotationOffset));
            _enableKalmanFilter = serializedObject.FindProperty(nameof(_enableKalmanFilter));
            _timeInterval = serializedObject.FindProperty(nameof(_timeInterval));
            _noise = serializedObject.FindProperty(nameof(_noise));
            _enableOneEuroFilter = serializedObject.FindProperty(nameof(_enableOneEuroFilter));
            _filterFrequency = serializedObject.FindProperty(nameof(_filterFrequency));
            _filterMinCutoff = serializedObject.FindProperty(nameof(_filterMinCutoff));
            _filterBeta = serializedObject.FindProperty(nameof(_filterBeta));
            _filterDcutoff = serializedObject.FindProperty(nameof(_filterDcutoff));
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            using (new EditorGUI.DisabledScope(true))
            {
                EditorGUILayout.PropertyField(m_Script, true);
            }

            EditorGUILayout.Space();
            EditorGUILayout.PropertyField(_tracker);
            EditorGUILayout.Space();
            EditorGUILayout.PropertyField(_IKType);
            EditorGUILayout.Space();

            using (new EditorGUI.IndentLevelScope())
            {
                EditorGUILayout.PropertyField(_landmarkScale);
                EditorGUILayout.PropertyField(_handRotationOffset);
                EditorGUILayout.Space();
            }

            EditorGUILayout.Space();

            var IKSettingsFoldout = EditorGUILayout.Foldout(_IKSettingsFoldout, "IK Settings", true);

            if (IKSettingsFoldout)
            {
                using (new EditorGUI.IndentLevelScope())
                {
                    EditorGUILayout.PropertyField(_enableTwistRelaxer);
                    EditorGUILayout.PropertyField(_enableMovement);
                    EditorGUILayout.PropertyField(_autoWeight);
                    EditorGUILayout.Space();
                    EditorGUILayout.PropertyField(_weightSmoothingSpeed);
                    EditorGUILayout.PropertyField(_targetRotationSmoothSpeed);
                    EditorGUILayout.PropertyField(_fingerResetSpeed);
                    EditorGUILayout.Space();
                }
            }

            _IKSettingsFoldout = IKSettingsFoldout;

            EditorGUILayout.Space();

            var filterSettingsFoldout = EditorGUILayout.Foldout(_filterSettingsFoldout, "Filter Settings", true);

            if (filterSettingsFoldout)
            {
                using (new EditorGUI.IndentLevelScope())
                {
                    EditorGUILayout.PropertyField(_enableKalmanFilter);

                    using (new EditorGUI.DisabledGroupScope(!_enableKalmanFilter.boolValue))
                    {
                        EditorGUILayout.PropertyField(_timeInterval);
                        EditorGUILayout.PropertyField(_noise);
                    }


                    EditorGUILayout.Space();
                    EditorGUILayout.PropertyField(_enableOneEuroFilter);

                    using (new EditorGUI.DisabledGroupScope(!_enableOneEuroFilter.boolValue))
                    {
                        EditorGUILayout.PropertyField(_filterFrequency);
                        EditorGUILayout.PropertyField(_filterMinCutoff);
                        EditorGUILayout.PropertyField(_filterBeta);
                        EditorGUILayout.PropertyField(_filterDcutoff);
                    }

                    EditorGUILayout.Space();
                }
            }

            _filterSettingsFoldout = filterSettingsFoldout;

            EditorGUILayout.Space();

            UI.DrawFoot();

            serializedObject.ApplyModifiedProperties();
        }
    }
}