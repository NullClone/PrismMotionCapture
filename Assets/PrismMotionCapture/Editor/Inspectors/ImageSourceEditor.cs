using System;
using UnityEditor;
using UnityEngine;

namespace PMC.Editor
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(ImageSource))]
    sealed class ImageSourceEditor : UnityEditor.Editor
    {
        private SerializedProperty SourceType;
        private SerializedProperty Texture2D;
        private SerializedProperty VideoPlayer;
        private SerializedProperty WebcamName;
        private SerializedProperty WebcamFrameRate;
        private SerializedProperty WebcamResolution;
        private SerializedProperty RenderMode;
        private SerializedProperty RenderTexture;
        private SerializedProperty Renderer;
        private SerializedProperty UseAutoSelect;
        private SerializedProperty PropertyName;
        private SerializedProperty RawImage;

        private void OnEnable()
        {
            SourceType = serializedObject.FindProperty(nameof(SourceType));
            Texture2D = serializedObject.FindProperty(nameof(Texture2D));
            VideoPlayer = serializedObject.FindProperty(nameof(VideoPlayer));
            WebcamName = serializedObject.FindProperty(nameof(WebcamName));
            WebcamFrameRate = serializedObject.FindProperty(nameof(WebcamFrameRate));
            WebcamResolution = serializedObject.FindProperty(nameof(WebcamResolution));
            RenderMode = serializedObject.FindProperty(nameof(RenderMode));
            RenderTexture = serializedObject.FindProperty(nameof(RenderTexture));
            Renderer = serializedObject.FindProperty(nameof(Renderer));
            UseAutoSelect = serializedObject.FindProperty(nameof(UseAutoSelect));
            PropertyName = serializedObject.FindProperty(nameof(PropertyName));
            RawImage = serializedObject.FindProperty(nameof(RawImage));
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.PropertyField(SourceType);

            using (new EditorGUI.IndentLevelScope())
            {
                switch ((SourceType)SourceType.enumValueIndex)
                {
                    case 0:
                        {
                            EditorGUILayout.PropertyField(Texture2D);

                            break;
                        }
                    case (SourceType)1:
                        {
                            EditorGUILayout.PropertyField(VideoPlayer);

                            break;
                        }
                    case (SourceType)2:
                        {
                            EditorGUILayout.BeginHorizontal();

                            using (new EditorGUI.DisabledGroupScope(true))
                            {
                                EditorGUILayout.PropertyField(WebcamName, new GUIContent("Device Name"));
                            }

                            var rect = EditorGUILayout.GetControlRect(false, GUILayout.Width(60));

                            if (EditorGUI.DropdownButton(rect, new GUIContent("Select"), FocusType.Keyboard))
                            {
                                var menu = new GenericMenu();

                                foreach (var device in WebCamTexture.devices)
                                {
                                    menu.AddItem(new GUIContent(device.name), false,
                                        () =>
                                        {
                                            serializedObject.Update();

                                            WebcamName.stringValue = device.name;

                                            serializedObject.ApplyModifiedProperties();
                                        });
                                }

                                menu.DropDown(rect);
                            }

                            EditorGUILayout.EndHorizontal();
                            EditorGUILayout.PropertyField(WebcamFrameRate, new GUIContent("Frame Rate"));
                            EditorGUILayout.PropertyField(WebcamResolution, new GUIContent("Resolution"));

                            break;
                        }
                }
            }

            EditorGUILayout.Space();
            EditorGUILayout.PropertyField(RenderMode);

            switch ((RenderMode)RenderMode.enumValueIndex)
            {
                case (RenderMode)1:
                    {
                        EditorGUILayout.PropertyField(RenderTexture);

                        break;
                    }
                case (RenderMode)2:
                    {
                        EditorGUILayout.PropertyField(Renderer);

                        var renderer = (Renderer)Renderer.objectReferenceValue;

                        if (renderer == null) break;

                        var material = renderer.sharedMaterial;

                        if (material == null) break;

                        EditorGUILayout.PropertyField(UseAutoSelect);

                        using (new EditorGUI.DisabledGroupScope(UseAutoSelect.boolValue))
                        {
                            var names = material.GetPropertyNames(MaterialPropertyType.Texture);

                            var selectedIndex = Array.IndexOf(names, PropertyName.stringValue);

                            if (selectedIndex < 0) selectedIndex = 0;

                            var index = EditorGUILayout.Popup("Material Property", selectedIndex, names);

                            PropertyName.stringValue = names[index];
                        }

                        break;
                    }
                case (RenderMode)3:
                    {
                        EditorGUILayout.PropertyField(RawImage);

                        break;
                    }
            }

            serializedObject.ApplyModifiedProperties();
        }
    }
}