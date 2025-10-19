using UnityEngine;

namespace PMC
{
    public sealed class KeypointLine : MonoBehaviour
    {
        // Fields

        [SerializeField] private LineRenderer _lineRenderer;
        [SerializeField] private Keypoint start;
        [SerializeField] private Keypoint end;
        [SerializeField] private Color color;
        [SerializeField] private float width;


        // Methods

        private void Awake()
        {
            if (_lineRenderer == null) return;

            _lineRenderer.startColor = color;
            _lineRenderer.endColor = color;
            _lineRenderer.startWidth = width;
            _lineRenderer.endWidth = width;
        }

        private void Update()
        {
            if (_lineRenderer == null) return;

            _lineRenderer.SetPosition(0, start.Position);
            _lineRenderer.SetPosition(1, end.Position);

            _lineRenderer.gameObject.SetActive(start.IsActive && end.IsActive);
        }
    }
}