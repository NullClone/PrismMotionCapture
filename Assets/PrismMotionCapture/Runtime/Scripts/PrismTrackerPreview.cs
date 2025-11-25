#if UNITY_EDITOR

using System.Collections.Generic;
using UnityEngine;
using Color = UnityEngine.Color;

namespace PMC
{
    [AddComponentMenu("Prism Motion Capture/Prism Tracker Preview")]
    public sealed class PrismTrackerPreview : MonoBehaviour
    {
        // Fields

        [SerializeField] private float _landmarkRadius = 0.01f;
        [SerializeField] private Color _leftLandmarkColor = new(1f, 0.5f, 0f, 1f);
        [SerializeField] private Color _rightLandmarkColor = Color.cyan;
        [SerializeField] private Color _connectionColor = Color.white;
        [SerializeField] private Vector3 LandmarkPosition = Vector3.zero;

        private PrismTracker _tracker;

        private Vector3[] _positions;

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


        // Methods

        private void Reset()
        {
            Awake();
        }

        private void Awake()
        {
            if (_tracker == null)
            {
                _tracker = gameObject.GetComponent<PrismTracker>();
            }
        }

        private void OnDrawGizmos()
        {
            if (!Application.isPlaying || _tracker == null || !enabled) return;

            _positions = _tracker.LocalAvatarSpacePoints;

            if (_positions == null) return;

            Gizmos.color = _connectionColor;

            foreach (var conn in _Connections)
            {
                Gizmos.DrawLine(_positions[conn.Item1] + LandmarkPosition, _positions[conn.Item2] + LandmarkPosition);
            }

            for (int i = 0; i < _positions.Length; i++)
            {
                if (_LeftLandmarks.Contains(i))
                {
                    Gizmos.color = _leftLandmarkColor;
                }

                if (_RightLandmarks.Contains(i))
                {
                    Gizmos.color = _rightLandmarkColor;
                }

                Gizmos.DrawSphere(_positions[i] + LandmarkPosition, _landmarkRadius);
            }
        }
    }
}

#endif