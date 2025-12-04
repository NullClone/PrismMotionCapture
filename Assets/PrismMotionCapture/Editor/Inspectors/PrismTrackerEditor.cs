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
        private SerializedProperty Framerate;
        private SerializedProperty MinFaceDetectionConfidence;
        private SerializedProperty MinFaceSuppressionThreshold;
        private SerializedProperty MinFaceLandmarksConfidence;
        private SerializedProperty MinPoseDetectionConfidence;
        private SerializedProperty MinPoseSuppressionThreshold;
        private SerializedProperty MinPoseLandmarksConfidence;
        private SerializedProperty MinHandLandmarksConfidence;
        private SerializedProperty OutputFaceBlendshapes;
        private SerializedProperty OutputSegmentationMask;
        private SerializedProperty FlipHorizontally;
        private SerializedProperty FlipVertically;
        private SerializedProperty LandmarkScale;
        private SerializedProperty MovementScale;
        private SerializedProperty _enableKalmanFilter;
        private SerializedProperty _timeInterval;
        private SerializedProperty _noise;
        private SerializedProperty _enableOneEuroFilter;
        private SerializedProperty _filterMinCutoff;
        private SerializedProperty _filterBeta;
        private SerializedProperty _filterDcutoff;
        private SerializedProperty _enableGlobalPoseFilter;
        private SerializedProperty _globalPoseFilterMinCutoff;
        private SerializedProperty _globalPoseFilterBeta;
        private SerializedProperty _globalPoseFilterDcutoff;
        private SerializedProperty ShowTrackingFPS;
        private SerializedProperty ShowLandmark;
        private SerializedProperty LandmarkRadius;
        private SerializedProperty LandmarkPosition;
        private SerializedProperty LeftLandmarkColor;
        private SerializedProperty RightLandmarkColor;
        private SerializedProperty ConnectionColor;

        private static bool _mediapipeSettingsFoldout;
        private static bool _trackingSettingsFoldout;
        private static bool _filterSettingsFoldout;
        private static bool _debugSettingsFoldout;

        private void OnEnable()
        {
            ImageSource = serializedObject.FindProperty(nameof(ImageSource));
            ImageReadMode = serializedObject.FindProperty(nameof(ImageReadMode));
            Framerate = serializedObject.FindProperty(nameof(Framerate));
            MinFaceDetectionConfidence = serializedObject.FindProperty(nameof(MinFaceDetectionConfidence));
            MinFaceSuppressionThreshold = serializedObject.FindProperty(nameof(MinFaceSuppressionThreshold));
            MinFaceLandmarksConfidence = serializedObject.FindProperty(nameof(MinFaceLandmarksConfidence));
            MinPoseDetectionConfidence = serializedObject.FindProperty(nameof(MinPoseDetectionConfidence));
            MinPoseSuppressionThreshold = serializedObject.FindProperty(nameof(MinPoseSuppressionThreshold));
            MinPoseLandmarksConfidence = serializedObject.FindProperty(nameof(MinPoseLandmarksConfidence));
            MinHandLandmarksConfidence = serializedObject.FindProperty(nameof(MinHandLandmarksConfidence));
            OutputFaceBlendshapes = serializedObject.FindProperty(nameof(OutputFaceBlendshapes));
            OutputSegmentationMask = serializedObject.FindProperty(nameof(OutputSegmentationMask));
            FlipHorizontally = serializedObject.FindProperty(nameof(FlipHorizontally));
            FlipVertically = serializedObject.FindProperty(nameof(FlipVertically));
            LandmarkScale = serializedObject.FindProperty(nameof(LandmarkScale));
            MovementScale = serializedObject.FindProperty(nameof(MovementScale));
            _enableKalmanFilter = serializedObject.FindProperty(nameof(_enableKalmanFilter));
            _timeInterval = serializedObject.FindProperty(nameof(_timeInterval));
            _noise = serializedObject.FindProperty(nameof(_noise));
            _enableOneEuroFilter = serializedObject.FindProperty(nameof(_enableOneEuroFilter));
            _filterMinCutoff = serializedObject.FindProperty(nameof(_filterMinCutoff));
            _filterBeta = serializedObject.FindProperty(nameof(_filterBeta));
            _filterDcutoff = serializedObject.FindProperty(nameof(_filterDcutoff));
            _enableGlobalPoseFilter = serializedObject.FindProperty(nameof(_enableGlobalPoseFilter));
            _globalPoseFilterMinCutoff = serializedObject.FindProperty(nameof(_globalPoseFilterMinCutoff));
            _globalPoseFilterBeta = serializedObject.FindProperty(nameof(_globalPoseFilterBeta));
            _globalPoseFilterDcutoff = serializedObject.FindProperty(nameof(_globalPoseFilterDcutoff));
            ShowTrackingFPS = serializedObject.FindProperty(nameof(ShowTrackingFPS));
            ShowLandmark = serializedObject.FindProperty(nameof(ShowLandmark));
            LandmarkRadius = serializedObject.FindProperty(nameof(LandmarkRadius));
            LandmarkPosition = serializedObject.FindProperty(nameof(LandmarkPosition));
            LeftLandmarkColor = serializedObject.FindProperty(nameof(LeftLandmarkColor));
            RightLandmarkColor = serializedObject.FindProperty(nameof(RightLandmarkColor));
            ConnectionColor = serializedObject.FindProperty(nameof(ConnectionColor));
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.PropertyField(ImageSource);
            EditorGUILayout.PropertyField(ImageReadMode);
            EditorGUILayout.PropertyField(Framerate);
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
                    EditorGUILayout.Space();
                    EditorGUILayout.PropertyField(FlipHorizontally);
                    EditorGUILayout.PropertyField(FlipVertically);
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
                        EditorGUILayout.PropertyField(_globalPoseFilterMinCutoff);
                        EditorGUILayout.PropertyField(_globalPoseFilterBeta);
                        EditorGUILayout.PropertyField(_globalPoseFilterDcutoff);
                    }

                    EditorGUILayout.EndVertical();
                }
            }

            EditorGUILayout.Space();

            _debugSettingsFoldout = EditorGUILayout.Foldout(_debugSettingsFoldout, "Debug Settings", true);

            if (_debugSettingsFoldout)
            {
                using (new EditorGUI.IndentLevelScope())
                {
                    EditorGUILayout.BeginVertical("box");
                    EditorGUILayout.PropertyField(ShowTrackingFPS);
                    EditorGUILayout.PropertyField(ShowLandmark);
                    EditorGUILayout.Space();
                    EditorGUILayout.PropertyField(LandmarkRadius);
                    EditorGUILayout.PropertyField(LandmarkPosition);
                    EditorGUILayout.Space();
                    EditorGUILayout.PropertyField(LeftLandmarkColor);
                    EditorGUILayout.PropertyField(RightLandmarkColor);
                    EditorGUILayout.PropertyField(ConnectionColor);
                    EditorGUILayout.EndVertical();
                }
            }

            EditorGUILayout.Space();
            EditorGUILayout.LabelField($"{AssetInfo.AssetName} (v{AssetInfo.AssetVersion})", EditorStyles.centeredGreyMiniLabel);

            serializedObject.ApplyModifiedProperties();
        }
    }
}