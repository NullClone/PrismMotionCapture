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
        private SerializedProperty _enableHandTracking;
        private SerializedProperty _enableFaceTracking;
        private SerializedProperty _enableTwistRelaxer;
        private SerializedProperty _enableMovement;
        private SerializedProperty _autoWeight;
        private SerializedProperty _fingerResetSpeed;
        private SerializedProperty _weightSmoothingSpeed;
        private SerializedProperty _targetRotationSmoothSpeed;
        private SerializedProperty _handRotationOffset;
        private SerializedProperty _leftEyeBone;
        private SerializedProperty _rightEyeBone;
        private SerializedProperty _autoBlink;
        private SerializedProperty _linkBlinks;
        private SerializedProperty _allowWinking;
        private SerializedProperty _eyeClosedThreshold;
        private SerializedProperty _eyeOpenedThreshold;
        private SerializedProperty _smartWinkThreshold;
        private SerializedProperty _blinkSmoothing;
        private SerializedProperty _gazeTracking;
        private SerializedProperty _gazeSmoothing;
        private SerializedProperty _gazeStrength;
        private SerializedProperty _gazeFactor;
        private SerializedProperty _mouthOpenSensitivity;
        private SerializedProperty _mouthShapeSensitivity;
        private SerializedProperty _mouthSmoothing;

        private static bool _movementSettingsFoldout;
        private static bool _handSettingsFoldout;
        private static bool _faceSettingsFoldout;
        private static bool _eyeBlinkSettingsFoldout;
        private static bool _gazeSettingsFoldout;
        private static bool _mouthSettingsFoldout;

        private void OnEnable()
        {
            m_Script = serializedObject.FindProperty("m_Script");
            _tracker = serializedObject.FindProperty(nameof(_tracker));
            _IKType = serializedObject.FindProperty(nameof(_IKType));
            _enableHandTracking = serializedObject.FindProperty(nameof(_enableHandTracking));
            _enableFaceTracking = serializedObject.FindProperty(nameof(_enableFaceTracking));
            _enableTwistRelaxer = serializedObject.FindProperty(nameof(_enableTwistRelaxer));
            _enableMovement = serializedObject.FindProperty(nameof(_enableMovement));
            _autoWeight = serializedObject.FindProperty(nameof(_autoWeight));
            _fingerResetSpeed = serializedObject.FindProperty(nameof(_fingerResetSpeed));
            _weightSmoothingSpeed = serializedObject.FindProperty(nameof(_weightSmoothingSpeed));
            _targetRotationSmoothSpeed = serializedObject.FindProperty(nameof(_targetRotationSmoothSpeed));
            _handRotationOffset = serializedObject.FindProperty(nameof(_handRotationOffset));
            _leftEyeBone = serializedObject.FindProperty(nameof(_leftEyeBone));
            _rightEyeBone = serializedObject.FindProperty(nameof(_rightEyeBone));
            _autoBlink = serializedObject.FindProperty(nameof(_autoBlink));
            _linkBlinks = serializedObject.FindProperty(nameof(_linkBlinks));
            _allowWinking = serializedObject.FindProperty(nameof(_allowWinking));
            _eyeClosedThreshold = serializedObject.FindProperty(nameof(_eyeClosedThreshold));
            _eyeOpenedThreshold = serializedObject.FindProperty(nameof(_eyeOpenedThreshold));
            _smartWinkThreshold = serializedObject.FindProperty(nameof(_smartWinkThreshold));
            _blinkSmoothing = serializedObject.FindProperty(nameof(_blinkSmoothing));
            _gazeTracking = serializedObject.FindProperty(nameof(_gazeTracking));
            _gazeSmoothing = serializedObject.FindProperty(nameof(_gazeSmoothing));
            _gazeStrength = serializedObject.FindProperty(nameof(_gazeStrength));
            _gazeFactor = serializedObject.FindProperty(nameof(_gazeFactor));
            _mouthOpenSensitivity = serializedObject.FindProperty(nameof(_mouthOpenSensitivity));
            _mouthShapeSensitivity = serializedObject.FindProperty(nameof(_mouthShapeSensitivity));
            _mouthSmoothing = serializedObject.FindProperty(nameof(_mouthSmoothing));
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            using (new EditorGUI.DisabledScope(true))
            {
                EditorGUILayout.PropertyField(m_Script, true);
            }

            EditorGUILayout.Space();

            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.PropertyField(_tracker);
            EditorGUILayout.PropertyField(_IKType);
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space();

            _movementSettingsFoldout = EditorGUILayout.Foldout(_movementSettingsFoldout, "Movement", true);
            if (_movementSettingsFoldout)
            {
                EditorGUILayout.BeginVertical("box");

                using (new EditorGUI.IndentLevelScope())
                {
                    EditorGUILayout.PropertyField(_enableMovement);
                }

                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.Space();

            _handSettingsFoldout = EditorGUILayout.Foldout(_handSettingsFoldout, "Hand Tracking", true);
            if (_handSettingsFoldout)
            {
                EditorGUILayout.BeginVertical("box");
                using (new EditorGUI.IndentLevelScope())
                {
                    EditorGUILayout.PropertyField(_enableHandTracking);
                    using (new EditorGUI.DisabledGroupScope(!_enableHandTracking.boolValue))
                    {
                        EditorGUILayout.PropertyField(_fingerResetSpeed);
                        EditorGUILayout.PropertyField(_handRotationOffset);
                    }
                }
                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.Space();

            _faceSettingsFoldout = EditorGUILayout.Foldout(_faceSettingsFoldout, "Face Tracking", true);
            if (_faceSettingsFoldout)
            {
                EditorGUILayout.BeginVertical("box");
                using (new EditorGUI.IndentLevelScope())
                {
                    EditorGUILayout.PropertyField(_enableFaceTracking);

                    using (new EditorGUI.DisabledGroupScope(!_enableFaceTracking.boolValue))
                    {
                        EditorGUILayout.Space();

                        _eyeBlinkSettingsFoldout = EditorGUILayout.Foldout(_eyeBlinkSettingsFoldout, "Eye Blink", true);
                        if (_eyeBlinkSettingsFoldout)
                        {
                            EditorGUILayout.PropertyField(_leftEyeBone);
                            EditorGUILayout.PropertyField(_rightEyeBone);
                            EditorGUILayout.PropertyField(_autoBlink);
                            using (new EditorGUI.DisabledGroupScope(_autoBlink.boolValue))
                            {
                                EditorGUILayout.PropertyField(_linkBlinks);
                                EditorGUILayout.PropertyField(_allowWinking);
                                EditorGUILayout.PropertyField(_eyeOpenedThreshold);
                                EditorGUILayout.PropertyField(_eyeClosedThreshold);
                                EditorGUILayout.PropertyField(_smartWinkThreshold);
                                EditorGUILayout.PropertyField(_blinkSmoothing);
                            }
                        }

                        EditorGUILayout.Space();

                        _gazeSettingsFoldout = EditorGUILayout.Foldout(_gazeSettingsFoldout, "Gaze Tracking", true);
                        if (_gazeSettingsFoldout)
                        {
                            EditorGUILayout.PropertyField(_gazeTracking);
                            using (new EditorGUI.DisabledGroupScope(!_gazeTracking.boolValue))
                            {
                                EditorGUILayout.PropertyField(_gazeSmoothing);
                                EditorGUILayout.PropertyField(_gazeStrength);
                                EditorGUILayout.PropertyField(_gazeFactor);
                            }
                        }

                        EditorGUILayout.Space();

                        _mouthSettingsFoldout = EditorGUILayout.Foldout(_mouthSettingsFoldout, "Mouth", true);
                        if (_mouthSettingsFoldout)
                        {
                            EditorGUILayout.PropertyField(_mouthOpenSensitivity);
                            EditorGUILayout.PropertyField(_mouthShapeSensitivity);
                            EditorGUILayout.PropertyField(_mouthSmoothing);
                        }
                    }
                }
                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.Space();

            UI.DrawFoot();

            serializedObject.ApplyModifiedProperties();
        }
    }
}