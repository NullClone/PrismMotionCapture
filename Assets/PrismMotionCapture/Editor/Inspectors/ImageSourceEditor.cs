using System;
using UnityEditor;
using UnityEngine;

namespace PMC.Editor
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(ImageSource))]
    sealed class ImageSourceEditor : UnityEditor.Editor
    {
        // Properties

        private SerializedProperty _sourceType;
        private SerializedProperty _isFlipX;
        private SerializedProperty _texture;
        private SerializedProperty _videoPlayer;
        private SerializedProperty _webcamName;
        private SerializedProperty _webcamFrameRate;
        private SerializedProperty _webcamResolution;
        private SerializedProperty _renderMode;
        private SerializedProperty _renderTexture;
        private SerializedProperty _renderer;
        private SerializedProperty _useAutoSelect;
        private SerializedProperty _propertyName;
        private SerializedProperty _rawImage;


        // Methods

        public override void OnInspectorGUI()
        {
            var instance = (ImageSource)target;

            serializedObject.Update();

            EditorGUI.BeginDisabledGroup(Application.isPlaying);

            EditorGUILayout.PropertyField(_sourceType);

            EditorGUI.indentLevel++;

            var sourceType = (SourceType)_sourceType.enumValueIndex;

            switch (sourceType)
            {
                case SourceType.Texture:
                    {
                        EditorGUILayout.PropertyField(_texture);
                        EditorGUILayout.PropertyField(_isFlipX);

                        break;
                    }
                case SourceType.Video:
                    {
                        EditorGUILayout.PropertyField(_videoPlayer);
                        EditorGUILayout.PropertyField(_isFlipX);

                        break;
                    }
                case SourceType.Webcam:
                    {
                        EditorGUILayout.BeginHorizontal();

                        EditorGUI.BeginDisabledGroup(true);

                        EditorGUILayout.PropertyField(_webcamName, new GUIContent("Device Name"));

                        EditorGUI.EndDisabledGroup();

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

                                        _webcamName.stringValue = device.name;

                                        serializedObject.ApplyModifiedProperties();
                                    });
                            }

                            menu.DropDown(rect);
                        }

                        EditorGUILayout.EndHorizontal();

                        EditorGUILayout.PropertyField(_webcamFrameRate, new GUIContent("Frame Rate"));
                        EditorGUILayout.PropertyField(_webcamResolution, new GUIContent("Resolution"));
                        EditorGUILayout.PropertyField(_isFlipX);

                        break;
                    }
            }

            EditorGUI.indentLevel--;

            EditorGUI.EndDisabledGroup();

            EditorGUILayout.Space();
            EditorGUILayout.PropertyField(_renderMode);

            var renderMode = (RenderMode)_renderMode.enumValueIndex;

            switch (renderMode)
            {
                case RenderMode.RenderTexture:
                    {
                        EditorGUILayout.PropertyField(_renderTexture);

                        break;
                    }
                case RenderMode.Renderer:
                    {
                        EditorGUILayout.PropertyField(_renderer);

                        if (instance.Renderer == null) break;

                        var material = instance.Renderer.sharedMaterial;

                        if (material)
                        {
                            EditorGUILayout.PropertyField(_useAutoSelect);

                            EditorGUI.indentLevel++;

                            EditorGUI.BeginDisabledGroup(_useAutoSelect.boolValue);

                            var names = material.GetPropertyNames(MaterialPropertyType.Texture);

                            var selectedIndex = Array.IndexOf(names, _propertyName.stringValue);

                            if (selectedIndex < 0)
                            {
                                selectedIndex = 0;
                            }

                            var index = EditorGUILayout.Popup("Material Property", selectedIndex, names);

                            _propertyName.stringValue = names[index];

                            EditorGUI.EndDisabledGroup();

                            EditorGUI.indentLevel--;
                        }

                        break;
                    }
                case RenderMode.RawImage:
                    {
                        EditorGUILayout.PropertyField(_rawImage);

                        break;
                    }
            }

            serializedObject.ApplyModifiedProperties();
        }


        private void OnEnable()
        {
            _sourceType = serializedObject.FindProperty("_sourceType");
            _isFlipX = serializedObject.FindProperty("_isFlipX");
            _texture = serializedObject.FindProperty("_texture");
            _videoPlayer = serializedObject.FindProperty("_videoPlayer");
            _webcamName = serializedObject.FindProperty("_webcamName");
            _webcamFrameRate = serializedObject.FindProperty("_webcamFrameRate");
            _webcamResolution = serializedObject.FindProperty("_webcamResolution");
            _renderMode = serializedObject.FindProperty("_renderMode");
            _renderTexture = serializedObject.FindProperty("_renderTexture");
            _renderer = serializedObject.FindProperty("_renderer");
            _useAutoSelect = serializedObject.FindProperty("_useAutoSelect");
            _propertyName = serializedObject.FindProperty("_propertyName");
            _rawImage = serializedObject.FindProperty("_rawImage");
        }
    }
}