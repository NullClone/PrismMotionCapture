using UnityEditor;
using UnityEngine;

namespace PMC.Editor
{
    public static class UI
    {
        public static void DrawFoot()
        {
            EditorGUILayout.LabelField($"{AssetInfo.AssetName} (v{AssetInfo.AssetVersion})", EditorStyles.centeredGreyMiniLabel);
        }

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
    }
}