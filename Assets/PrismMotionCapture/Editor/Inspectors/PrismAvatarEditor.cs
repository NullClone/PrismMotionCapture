using UnityEditor;

namespace PMC.Editor
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(PrismAvatar))]
    sealed class PrismAvatarEditor : UnityEditor.Editor
    {
        private SerializedProperty Tracker;
        private SerializedProperty IKType;
        private SerializedProperty EnableTwistRelaxer;
        private SerializedProperty EnableMovement;
        private SerializedProperty AutoWeight;
        private SerializedProperty WeightSmoothingSpeed;
        private SerializedProperty TargetRotationSmoothSpeed;
        private SerializedProperty EnableHandTracking;
        private SerializedProperty FingerResetSpeed;
        private SerializedProperty HandRotationOffset;
        private SerializedProperty EnableBlink;
        private SerializedProperty AutoBlink;
        private SerializedProperty LinkBlinks;
        private SerializedProperty AllowWinking;
        private SerializedProperty EyeClosedThreshold;
        private SerializedProperty EyeOpenedThreshold;
        private SerializedProperty SmartWinkThreshold;
        private SerializedProperty BlinkSmoothing;
        private SerializedProperty EnableGaze;
        private SerializedProperty GazeSmoothing;
        private SerializedProperty GazeStrength;
        private SerializedProperty EnableMouth;
        private SerializedProperty MouthOpenSensitivity;
        private SerializedProperty MouthShapeSensitivity;
        private SerializedProperty MouthSmoothing;

        private static bool _handSettingsFoldout;
        private static bool _faceSettingsFoldout;
        private static bool _eyeBlinkSettingsFoldout;
        private static bool _gazeSettingsFoldout;
        private static bool _mouthSettingsFoldout;

        private void OnEnable()
        {
            Tracker = serializedObject.FindProperty(nameof(Tracker));
            IKType = serializedObject.FindProperty(nameof(IKType));
            EnableTwistRelaxer = serializedObject.FindProperty(nameof(EnableTwistRelaxer));
            EnableMovement = serializedObject.FindProperty(nameof(EnableMovement));
            AutoWeight = serializedObject.FindProperty(nameof(AutoWeight));
            WeightSmoothingSpeed = serializedObject.FindProperty(nameof(WeightSmoothingSpeed));
            TargetRotationSmoothSpeed = serializedObject.FindProperty(nameof(TargetRotationSmoothSpeed));
            EnableHandTracking = serializedObject.FindProperty(nameof(EnableHandTracking));
            FingerResetSpeed = serializedObject.FindProperty(nameof(FingerResetSpeed));
            HandRotationOffset = serializedObject.FindProperty(nameof(HandRotationOffset));
            EnableBlink = serializedObject.FindProperty(nameof(EnableBlink));
            AutoBlink = serializedObject.FindProperty(nameof(AutoBlink));
            LinkBlinks = serializedObject.FindProperty(nameof(LinkBlinks));
            AllowWinking = serializedObject.FindProperty(nameof(AllowWinking));
            EyeClosedThreshold = serializedObject.FindProperty(nameof(EyeClosedThreshold));
            EyeOpenedThreshold = serializedObject.FindProperty(nameof(EyeOpenedThreshold));
            SmartWinkThreshold = serializedObject.FindProperty(nameof(SmartWinkThreshold));
            BlinkSmoothing = serializedObject.FindProperty(nameof(BlinkSmoothing));
            EnableGaze = serializedObject.FindProperty(nameof(EnableGaze));
            GazeSmoothing = serializedObject.FindProperty(nameof(GazeSmoothing));
            GazeStrength = serializedObject.FindProperty(nameof(GazeStrength));
            EnableMouth = serializedObject.FindProperty(nameof(EnableMouth));
            MouthOpenSensitivity = serializedObject.FindProperty(nameof(MouthOpenSensitivity));
            MouthShapeSensitivity = serializedObject.FindProperty(nameof(MouthShapeSensitivity));
            MouthSmoothing = serializedObject.FindProperty(nameof(MouthSmoothing));
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.PropertyField(Tracker);
            EditorGUILayout.PropertyField(IKType);
            EditorGUILayout.PropertyField(EnableTwistRelaxer);
            EditorGUILayout.PropertyField(EnableMovement);
            EditorGUILayout.PropertyField(AutoWeight);
            EditorGUILayout.PropertyField(WeightSmoothingSpeed);
            EditorGUILayout.PropertyField(TargetRotationSmoothSpeed);
            EditorGUILayout.EndVertical();
            EditorGUILayout.Space();

            _handSettingsFoldout = EditorGUILayout.Foldout(_handSettingsFoldout, "Hand Tracking", true);

            if (_handSettingsFoldout)
            {
                using (new EditorGUI.IndentLevelScope())
                {
                    EditorGUILayout.BeginVertical("box");
                    EditorGUILayout.PropertyField(EnableHandTracking);

                    using (new EditorGUI.DisabledGroupScope(!EnableHandTracking.boolValue))
                    {
                        EditorGUILayout.PropertyField(FingerResetSpeed);
                        EditorGUILayout.PropertyField(HandRotationOffset);
                    }

                    EditorGUILayout.EndVertical();
                }
            }

            EditorGUILayout.Space();

            _faceSettingsFoldout = EditorGUILayout.Foldout(_faceSettingsFoldout, "Face Tracking", true);

            if (_faceSettingsFoldout)
            {
                using (new EditorGUI.IndentLevelScope())
                {
                    EditorGUILayout.BeginVertical("box");

                    _eyeBlinkSettingsFoldout = EditorGUILayout.Foldout(_eyeBlinkSettingsFoldout, "Blink", true);

                    if (_eyeBlinkSettingsFoldout)
                    {
                        EditorGUILayout.PropertyField(EnableBlink);

                        using (new EditorGUI.DisabledGroupScope(!EnableBlink.boolValue))
                        {
                            EditorGUILayout.PropertyField(AutoBlink);

                            using (new EditorGUI.DisabledGroupScope(AutoBlink.boolValue))
                            {
                                EditorGUILayout.PropertyField(LinkBlinks);
                                EditorGUILayout.PropertyField(AllowWinking);
                                EditorGUILayout.PropertyField(EyeClosedThreshold);
                                EditorGUILayout.PropertyField(EyeOpenedThreshold);
                                EditorGUILayout.PropertyField(SmartWinkThreshold);
                                EditorGUILayout.PropertyField(BlinkSmoothing);
                            }
                        }
                    }

                    EditorGUILayout.Space();

                    _gazeSettingsFoldout = EditorGUILayout.Foldout(_gazeSettingsFoldout, "Gaze", true);

                    if (_gazeSettingsFoldout)
                    {
                        EditorGUILayout.PropertyField(EnableGaze);

                        using (new EditorGUI.DisabledGroupScope(!EnableGaze.boolValue))
                        {
                            EditorGUILayout.PropertyField(GazeSmoothing);
                            EditorGUILayout.PropertyField(GazeStrength);
                        }
                    }

                    EditorGUILayout.Space();

                    _mouthSettingsFoldout = EditorGUILayout.Foldout(_mouthSettingsFoldout, "Mouth", true);

                    if (_mouthSettingsFoldout)
                    {
                        EditorGUILayout.PropertyField(EnableMouth);

                        using (new EditorGUI.DisabledGroupScope(!EnableMouth.boolValue))
                        {
                            EditorGUILayout.PropertyField(MouthOpenSensitivity);
                            EditorGUILayout.PropertyField(MouthShapeSensitivity);
                            EditorGUILayout.PropertyField(MouthSmoothing);
                        }
                    }

                    EditorGUILayout.EndVertical();
                }
            }

            EditorGUILayout.Space();

            UI.DrawFoot();

            serializedObject.ApplyModifiedProperties();
        }
    }
}