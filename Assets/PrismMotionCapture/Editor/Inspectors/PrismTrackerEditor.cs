using UnityEditor;
using UnityEngine;

namespace PMC.Editor
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(PrismTracker))]
    sealed class PrismTrackerEditor : UnityEditor.Editor
    {
        private SerializedProperty m_Script;
        private SerializedProperty _imageSource;
        private SerializedProperty _imageReadMode;
        private SerializedProperty _modelAssetPath;
        private SerializedProperty _minFaceDetectionConfidence;
        private SerializedProperty _minFaceSuppressionThreshold;
        private SerializedProperty _minFaceLandmarksConfidence;
        private SerializedProperty _minPoseDetectionConfidence;
        private SerializedProperty _minPoseSuppressionThreshold;
        private SerializedProperty _minPoseLandmarksConfidence;
        private SerializedProperty _minHandLandmarksConfidence;
        private SerializedProperty _outputFaceBlendshapes;
        private SerializedProperty _outputSegmentationMask;
        private SerializedProperty _landmarkScale;
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

        private static bool _trackingSettingsFoldout;
        private static bool _landmarkFilterSettingsFoldout;
        private static bool _globalPoseFilterSettingsFoldout;

        private void OnEnable()
        {
            m_Script = serializedObject.FindProperty("m_Script");
            _imageSource = serializedObject.FindProperty(nameof(_imageSource));
            _imageReadMode = serializedObject.FindProperty(nameof(_imageReadMode));
            _modelAssetPath = serializedObject.FindProperty(nameof(_modelAssetPath));
            _minFaceDetectionConfidence = serializedObject.FindProperty(nameof(_minFaceDetectionConfidence));
            _minFaceSuppressionThreshold = serializedObject.FindProperty(nameof(_minFaceSuppressionThreshold));
            _minFaceLandmarksConfidence = serializedObject.FindProperty(nameof(_minFaceLandmarksConfidence));
            _minPoseDetectionConfidence = serializedObject.FindProperty(nameof(_minPoseDetectionConfidence));
            _minPoseSuppressionThreshold = serializedObject.FindProperty(nameof(_minPoseSuppressionThreshold));
            _minPoseLandmarksConfidence = serializedObject.FindProperty(nameof(_minPoseLandmarksConfidence));
            _minHandLandmarksConfidence = serializedObject.FindProperty(nameof(_minHandLandmarksConfidence));
            _outputFaceBlendshapes = serializedObject.FindProperty(nameof(_outputFaceBlendshapes));
            _outputSegmentationMask = serializedObject.FindProperty(nameof(_outputSegmentationMask));
            _landmarkScale = serializedObject.FindProperty(nameof(_landmarkScale));
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

            using (new EditorGUI.DisabledScope(true))
            {
                EditorGUILayout.PropertyField(m_Script, true);
            }

            EditorGUILayout.Space();

            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.PropertyField(_imageSource);
            EditorGUILayout.PropertyField(_imageReadMode);
            EditorGUILayout.PropertyField(_modelAssetPath);
            EditorGUILayout.PropertyField(_landmarkScale);
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space();

            _trackingSettingsFoldout = EditorGUILayout.Foldout(_trackingSettingsFoldout, "Tracking Confidence", true);
            if (_trackingSettingsFoldout)
            {
                EditorGUILayout.BeginVertical("box");
                using (new EditorGUI.IndentLevelScope())
                {
                    EditorGUILayout.PropertyField(_minFaceDetectionConfidence, new GUIContent("Face Detection"));
                    EditorGUILayout.PropertyField(_minFaceSuppressionThreshold, new GUIContent("Face Suppression"));
                    EditorGUILayout.PropertyField(_minFaceLandmarksConfidence, new GUIContent("Face Landmarks"));
                    EditorGUILayout.Space();
                    EditorGUILayout.PropertyField(_minPoseDetectionConfidence, new GUIContent("Pose Detection"));
                    EditorGUILayout.PropertyField(_minPoseSuppressionThreshold, new GUIContent("Pose Suppression"));
                    EditorGUILayout.PropertyField(_minPoseLandmarksConfidence, new GUIContent("Pose Landmarks"));
                    EditorGUILayout.Space();
                    EditorGUILayout.PropertyField(_minHandLandmarksConfidence, new GUIContent("Hand Landmarks"));
                    EditorGUILayout.Space();
                    EditorGUILayout.PropertyField(_outputFaceBlendshapes, new GUIContent("Enable Face Blendshapes"));
                    EditorGUILayout.PropertyField(_outputSegmentationMask, new GUIContent("Enable Segmentation Mask"));
                }
                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.Space();

            _landmarkFilterSettingsFoldout = EditorGUILayout.Foldout(_landmarkFilterSettingsFoldout, "Landmark Filter Settings", true);
            if (_landmarkFilterSettingsFoldout)
            {
                EditorGUILayout.BeginVertical("box");
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
                }
                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.Space();

            _globalPoseFilterSettingsFoldout = EditorGUILayout.Foldout(_globalPoseFilterSettingsFoldout, "Global Pose Filter Settings", true);
            if (_globalPoseFilterSettingsFoldout)
            {
                EditorGUILayout.BeginVertical("box");
                using (new EditorGUI.IndentLevelScope())
                {
                    EditorGUILayout.PropertyField(_enableGlobalPoseFilter, new GUIContent("Enable Global Pose Filter"));
                    using (new EditorGUI.DisabledGroupScope(!_enableGlobalPoseFilter.boolValue))
                    {
                        EditorGUILayout.PropertyField(_globalPoseFilterFrequency);
                        EditorGUILayout.PropertyField(_globalPoseFilterMinCutoff);
                        EditorGUILayout.PropertyField(_globalPoseFilterBeta);
                        EditorGUILayout.PropertyField(_globalPoseFilterDcutoff);
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