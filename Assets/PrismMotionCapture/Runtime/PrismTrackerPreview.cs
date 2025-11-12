using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Color = UnityEngine.Color;

namespace PMC
{
    [AddComponentMenu("Prism Motion Capture/Prism Tracker Preview")]
    public sealed class PrismTrackerPreview : MonoBehaviour
    {
        // Fields

        [SerializeField] private float _landmarkRadius = 0.01f;
        [SerializeField] private Color _leftLandmarkColor = Color.green;
        [SerializeField] private Color _rightLandmarkColor = Color.cyan;
        [SerializeField] private Color _centerLandmarkColor = Color.white;
        [SerializeField] private Color _connectionColor = Color.red;
        [SerializeField] private Vector3 _landmarkPosition = Vector3.zero;
        [SerializeField] private Vector3 _landmarkScale = Vector3.one;


        private static readonly HashSet<int> _LeftLandmarks = new() { 1, 2, 3, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31 };

        private static readonly HashSet<int> _RightLandmarks = new() { 4, 5, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32 };

        private static readonly List<(int, int)> _Connections = new() {
            // Left Eye
            (0, 1), (1, 2), (2, 3), (3, 7),
            // Right Eye
            (0, 4), (4, 5), (5, 6), (6, 8),
            // Lips
            (9, 10),
            // Left Arm
            (11, 13), (13, 15),
            // Left Hand
            (15, 17), (15, 19), (15, 21), (17, 19),
            // Right Arm
            (12, 14), (14, 16),
            // Right Hand
            (16, 18), (16, 20), (16, 22), (18, 20),
            // Torso
            (11, 12), (12, 24), (24, 23), (23, 11),
            // Left Leg
            (23, 25), (25, 27), (27, 29), (27, 31), (29, 31),
            // Right Leg
            (24, 26), (26, 28), (28, 30), (28, 32), (30, 32),
        };


        // Properties

        public IReadOnlyList<Mediapipe.Tasks.Components.Containers.Landmark> Landmarks { get; set; }

        public BodyParts Mask { get; set; } = BodyParts.All;

        public bool VisualizeZ { get; set; } = true;


        // Methods

        private void OnDrawGizmos()
        {
            if (!Application.isPlaying || !enabled) return;

            if (Landmarks == null) return;

            var landmarks = Landmarks.ToList();

            if (landmarks.Count > 0)
            {
                Gizmos.color = _connectionColor;

                foreach (var conn in _Connections)
                {
                    if (IsVisible(conn.Item1, Mask) && IsVisible(conn.Item2, Mask))
                    {
                        if (conn.Item1 >= landmarks.Count || conn.Item2 >= landmarks.Count) continue;

                        Gizmos.DrawLine(
                            GetLandmarkPosition(landmarks[conn.Item1]),
                            GetLandmarkPosition(landmarks[conn.Item2]));
                    }
                }

                for (int i = 0; i < landmarks.Count; i++)
                {
                    if (IsVisible(i, Mask))
                    {
                        Gizmos.color = GetLandmarkColor(i);
                        Gizmos.DrawSphere(GetLandmarkPosition(landmarks[i]), _landmarkRadius);
                    }
                }
            }
        }

        private bool IsVisible(int index, BodyParts mask)
        {
            if (mask.HasFlag(BodyParts.All))
            {
                return true;
            }
            if (mask == BodyParts.None)
            {
                return false;
            }

            if (index >= 0 && index <= 10) // Face
            {
                return mask.HasFlag(BodyParts.Face);
            }
            if (index == 13) // Left Elbow
            {
                return mask.HasFlag(BodyParts.LeftArm);
            }
            if (index == 15 || index == 17 || index == 19 || index == 21) // Left Hand
            {
                return mask.HasFlag(BodyParts.LeftHand);
            }
            if (index == 14) // Right Elbow
            {
                return mask.HasFlag(BodyParts.RightArm);
            }
            if (index == 16 || index == 18 || index == 20 || index == 22) // Right Hand
            {
                return mask.HasFlag(BodyParts.RightHand);
            }
            if (index >= 25 && index <= 32) // Lower Body
            {
                return mask.HasFlag(BodyParts.LowerBody);
            }

            return true;
        }

        private Color GetLandmarkColor(int index)
        {
            if (_LeftLandmarks.Contains(index))
            {
                return _leftLandmarkColor;
            }

            if (_RightLandmarks.Contains(index))
            {
                return _rightLandmarkColor;
            }

            return _centerLandmarkColor;
        }

        private Vector3 GetLandmarkPosition(Mediapipe.Tasks.Components.Containers.Landmark lm)
        {
            var localPos = new Vector3(
                lm.x,
                lm.y,
                VisualizeZ ? lm.z : 0f
            );

            return Vector3.Scale(localPos, _landmarkScale) + _landmarkPosition;
        }
    }

    [Flags]
    public enum BodyParts : short
    {
        None = 0,
        Face = 1,
        // Torso = 2,
        LeftArm = 4,
        LeftHand = 8,
        RightArm = 16,
        RightHand = 32,
        LowerBody = 64,
        All = 127,
    }
}