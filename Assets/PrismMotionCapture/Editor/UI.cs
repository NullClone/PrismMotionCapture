using System;
using UnityEditor;
using UnityEditor.AnimatedValues;
using UnityEngine;
using UnityEngine.Events;

namespace PMC.Editor
{
    public static class UI
    {
        public class Section
        {
            public bool Expanded
            {
                get { return SessionState.GetBool(id, false); }
                set { SessionState.SetBool(id, value); }
            }
            public AnimBool anim;

            public readonly string id;
            public GUIContent title;

            public Section(UnityAction action, string id, GUIContent title)
            {
                this.id = id;
                this.title = title;

                anim = new AnimBool(true);
                anim.valueChanged.AddListener(action);
                anim.speed = 12f;
                anim.target = Expanded;
            }

            public Section(UnityEditor.Editor owner, string id, GUIContent title) : this(owner.Repaint, id, title) { }

            public void DrawHeader()
            {
                UI.DrawHeader(title, Expanded, () => Expanded = !Expanded);

                anim.target = Expanded;
            }
        }

        private const float HeaderHeight = 25f;

        public static void DrawSplitter(bool isBoxed = false)
        {
            var rect = GUILayoutUtility.GetRect(1f, 1f);
            float xMin = rect.xMin;

            rect = ToFullWidth(rect);

            if (isBoxed)
            {
                rect.xMin = xMin == 7.0 ? 4.0f : EditorGUIUtility.singleLineHeight;
                rect.width -= 1;
            }

            if (Event.current.type != EventType.Repaint)
                return;

            EditorGUI.DrawRect(rect, !EditorGUIUtility.isProSkin
                ? new Color(0.6f, 0.6f, 0.6f, 1.333f)
                : new Color(0.12f, 0.12f, 0.12f, 1.333f));
        }

        private static Rect ToFullWidth(Rect rect)
        {
            rect.xMin = 0f;
            rect.width += 4f;
            return rect;
        }

        public static bool DrawHeader(GUIContent content, bool isExpanded, Action clickAction = null)
        {
            DrawSplitter();

            Rect backgroundRect = GUILayoutUtility.GetRect(1f, HeaderHeight);

            var labelRect = backgroundRect;
            labelRect.xMin += 12f;
            labelRect.xMax -= 20f + 16 + 5;

            var foldoutRect = backgroundRect;
            foldoutRect.xMin -= 8f;
            foldoutRect.y += 0f;
            foldoutRect.width = HeaderHeight;
            foldoutRect.height = HeaderHeight;

            backgroundRect.xMin = 0f;
            backgroundRect.width += 4f;

            float backgroundTint = EditorGUIUtility.isProSkin ? 0.1f : 1f;

            if (backgroundRect.Contains(Event.current.mousePosition))
            {
                backgroundTint *= EditorGUIUtility.isProSkin ? 1.5f : 0.9f;
            }

            EditorGUI.DrawRect(backgroundRect, new Color(backgroundTint, backgroundTint, backgroundTint, 0.2f));

            EditorGUI.LabelField(labelRect, content, EditorStyles.boldLabel);

            GUI.Label(foldoutRect, new GUIContent(isExpanded ? "-" : "="), EditorStyles.boldLabel);

            var e = Event.current;

            if (e.type == EventType.MouseDown)
            {
                if (backgroundRect.Contains(e.mousePosition))
                {
                    if (e.button == 0)
                    {
                        isExpanded = !isExpanded;

                        clickAction?.Invoke();
                    }

                    e.Use();
                }
            }

            return isExpanded;
        }
    }
}