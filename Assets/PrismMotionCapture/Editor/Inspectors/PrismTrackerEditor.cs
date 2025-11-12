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
        private SerializedProperty _preview;

        private static bool _trackingSettingsFoldout;

        private void OnEnable()
        {
            m_Script = serializedObject.FindProperty(nameof(m_Script));
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
            _preview = serializedObject.FindProperty(nameof(_preview));
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            using (new EditorGUI.DisabledScope(true))
            {
                EditorGUILayout.PropertyField(m_Script, true);
            }

            EditorGUILayout.Space();

            EditorGUILayout.PropertyField(_imageSource);

            EditorGUILayout.PropertyField(_imageReadMode);
            EditorGUILayout.Space();
            EditorGUILayout.PropertyField(_modelAssetPath);
            EditorGUILayout.Space();

            var trackingSettingsFoldout = EditorGUILayout.Foldout(_trackingSettingsFoldout, "Tracking Settings", true);

            if (trackingSettingsFoldout)
            {
                using (new EditorGUI.IndentLevelScope())
                {
                    EditorGUILayout.PropertyField(_minFaceDetectionConfidence, new GUIContent("Face Detection Confidence"));
                    EditorGUILayout.PropertyField(_minFaceSuppressionThreshold, new GUIContent("Face Suppression Threshold"));
                    EditorGUILayout.PropertyField(_minFaceLandmarksConfidence, new GUIContent("Face Landmarks Confidence"));
                    EditorGUILayout.PropertyField(_minPoseDetectionConfidence, new GUIContent("Pose Detection Confidence"));
                    EditorGUILayout.PropertyField(_minPoseSuppressionThreshold, new GUIContent("Pose Suppression Threshold"));
                    EditorGUILayout.PropertyField(_minPoseLandmarksConfidence, new GUIContent("Pose Landmarks Confidence"));
                    EditorGUILayout.PropertyField(_minHandLandmarksConfidence, new GUIContent("Hand Landmarks Confidence"));
                    EditorGUILayout.Space();
                    EditorGUILayout.PropertyField(_outputFaceBlendshapes, new GUIContent("Enable Face Blendshapes"));
                    EditorGUILayout.PropertyField(_outputSegmentationMask, new GUIContent("Enable Segmentation Mask"));
                    EditorGUILayout.Space();
                }
            }

            _trackingSettingsFoldout = trackingSettingsFoldout;

            EditorGUILayout.Space();

            EditorGUILayout.PropertyField(_preview);

            EditorGUILayout.Space();

            UI.DrawFoot();

            serializedObject.ApplyModifiedProperties();
        }
    }
}