using UnityEditor;
using UnityEngine;

namespace PMC.Editor
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(PrismTracker))]
    sealed class PrismTrackerEditor : UnityEditor.Editor
    {
        private SerializedProperty ImageSource;
        private SerializedProperty ImageReadMode;
        private SerializedProperty MinFaceDetectionConfidence;
        private SerializedProperty MinFaceSuppressionThreshold;
        private SerializedProperty MinFaceLandmarksConfidence;
        private SerializedProperty MinPoseDetectionConfidence;
        private SerializedProperty MinPoseSuppressionThreshold;
        private SerializedProperty MinPoseLandmarksConfidence;
        private SerializedProperty MinHandLandmarksConfidence;
        private SerializedProperty OutputFaceBlendshapes;
        private SerializedProperty OutputSegmentationMask;
        private SerializedProperty LandmarkScale;
        private SerializedProperty MovementScale;
        private SerializedProperty _enableKalmanFilter;
        private SerializedProperty _timeInterval;
        private SerializedProperty _noise;
        private SerializedProperty _enableOneEuroFilter;
        private SerializedProperty _filterFrequency;
        private SerializedProperty _filterMinCutoff;
        private SerializedProperty _filterBeta;
        private SerializedProperty _filterDcutoff;
        private SerializedProperty _enableGlobalPoseFilter;
        private SerializedProperty _globalPoseFilterFrequency;
        private SerializedProperty _globalPoseFilterMinCutoff;
        private SerializedProperty _globalPoseFilterBeta;
        private SerializedProperty _globalPoseFilterDcutoff;

        private static bool _mediapipeSettingsFoldout;
        private static bool _trackingSettingsFoldout;
        private static bool _filterSettingsFoldout;

        private void OnEnable()
        {
            ImageSource = serializedObject.FindProperty(nameof(ImageSource));
            ImageReadMode = serializedObject.FindProperty(nameof(ImageReadMode));
            MinFaceDetectionConfidence = serializedObject.FindProperty(nameof(MinFaceDetectionConfidence));
            MinFaceSuppressionThreshold = serializedObject.FindProperty(nameof(MinFaceSuppressionThreshold));
            MinFaceLandmarksConfidence = serializedObject.FindProperty(nameof(MinFaceLandmarksConfidence));
            MinPoseDetectionConfidence = serializedObject.FindProperty(nameof(MinPoseDetectionConfidence));
            MinPoseSuppressionThreshold = serializedObject.FindProperty(nameof(MinPoseSuppressionThreshold));
            MinPoseLandmarksConfidence = serializedObject.FindProperty(nameof(MinPoseLandmarksConfidence));
            MinHandLandmarksConfidence = serializedObject.FindProperty(nameof(MinHandLandmarksConfidence));
            OutputFaceBlendshapes = serializedObject.FindProperty(nameof(OutputFaceBlendshapes));
            OutputSegmentationMask = serializedObject.FindProperty(nameof(OutputSegmentationMask));
            LandmarkScale = serializedObject.FindProperty(nameof(LandmarkScale));
            MovementScale = serializedObject.FindProperty(nameof(MovementScale));
            _enableKalmanFilter = serializedObject.FindProperty(nameof(_enableKalmanFilter));
            _timeInterval = serializedObject.FindProperty(nameof(_timeInterval));
            _noise = serializedObject.FindProperty(nameof(_noise));
            _enableOneEuroFilter = serializedObject.FindProperty(nameof(_enableOneEuroFilter));
            _filterFrequency = serializedObject.FindProperty(nameof(_filterFrequency));
            _filterMinCutoff = serializedObject.FindProperty(nameof(_filterMinCutoff));
            _filterBeta = serializedObject.FindProperty(nameof(_filterBeta));
            _filterDcutoff = serializedObject.FindProperty(nameof(_filterDcutoff));
            _enableGlobalPoseFilter = serializedObject.FindProperty(nameof(_enableGlobalPoseFilter));
            _globalPoseFilterFrequency = serializedObject.FindProperty(nameof(_globalPoseFilterFrequency));
            _globalPoseFilterMinCutoff = serializedObject.FindProperty(nameof(_globalPoseFilterMinCutoff));
            _globalPoseFilterBeta = serializedObject.FindProperty(nameof(_globalPoseFilterBeta));
            _globalPoseFilterDcutoff = serializedObject.FindProperty(nameof(_globalPoseFilterDcutoff));
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.PropertyField(ImageSource);
            EditorGUILayout.PropertyField(ImageReadMode);
            EditorGUILayout.EndVertical();
            EditorGUILayout.Space();

            _mediapipeSettingsFoldout = EditorGUILayout.Foldout(_mediapipeSettingsFoldout, "Mediapipe Settings", true);

            if (_mediapipeSettingsFoldout)
            {
                using (new EditorGUI.IndentLevelScope())
                {
                    EditorGUILayout.BeginVertical("box");
                    EditorGUILayout.PropertyField(MinFaceDetectionConfidence, new GUIContent("Face Detection"));
                    EditorGUILayout.PropertyField(MinFaceSuppressionThreshold, new GUIContent("Face Suppression"));
                    EditorGUILayout.PropertyField(MinFaceLandmarksConfidence, new GUIContent("Face Landmarks"));
                    EditorGUILayout.Space();
                    EditorGUILayout.PropertyField(MinPoseDetectionConfidence, new GUIContent("Pose Detection"));
                    EditorGUILayout.PropertyField(MinPoseSuppressionThreshold, new GUIContent("Pose Suppression"));
                    EditorGUILayout.PropertyField(MinPoseLandmarksConfidence, new GUIContent("Pose Landmarks"));
                    EditorGUILayout.Space();
                    EditorGUILayout.PropertyField(MinHandLandmarksConfidence, new GUIContent("Hand Landmarks"));
                    EditorGUILayout.Space();
                    EditorGUILayout.PropertyField(OutputFaceBlendshapes, new GUIContent("Enable Face Blendshapes"));
                    EditorGUILayout.PropertyField(OutputSegmentationMask, new GUIContent("Enable Segmentation Mask"));
                    EditorGUILayout.EndVertical();
                }
            }

            EditorGUILayout.Space();

            _trackingSettingsFoldout = EditorGUILayout.Foldout(_trackingSettingsFoldout, "Tracking Settings", true);

            if (_trackingSettingsFoldout)
            {
                EditorGUILayout.BeginVertical("box");
                EditorGUILayout.PropertyField(LandmarkScale);
                EditorGUILayout.PropertyField(MovementScale);
                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.Space();

            _filterSettingsFoldout = EditorGUILayout.Foldout(_filterSettingsFoldout, "Filter Settings", true);

            if (_filterSettingsFoldout)
            {
                using (new EditorGUI.IndentLevelScope())
                {
                    EditorGUILayout.BeginVertical("box");
                    EditorGUILayout.LabelField("Landmark Filter", EditorStyles.boldLabel);
                    EditorGUILayout.Space();
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

                    EditorGUILayout.EndVertical();
                    EditorGUILayout.Space();
                    EditorGUILayout.BeginVertical("box");
                    EditorGUILayout.PropertyField(_enableGlobalPoseFilter);

                    using (new EditorGUI.DisabledGroupScope(!_enableGlobalPoseFilter.boolValue))
                    {
                        EditorGUILayout.PropertyField(_globalPoseFilterFrequency);
                        EditorGUILayout.PropertyField(_globalPoseFilterMinCutoff);
                        EditorGUILayout.PropertyField(_globalPoseFilterBeta);
                        EditorGUILayout.PropertyField(_globalPoseFilterDcutoff);
                    }

                    EditorGUILayout.EndVertical();
                }
            }

            EditorGUILayout.Space();

            EditorGUILayout.LabelField($"{AssetInfo.AssetName} (v{AssetInfo.AssetVersion})", EditorStyles.centeredGreyMiniLabel);

            serializedObject.ApplyModifiedProperties();
        }
    }
}