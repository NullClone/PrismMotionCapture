using UnityEngine;

namespace PMC
{
    public sealed class Keypoint : MonoBehaviour
    {
        // Properties

        public Vector3 Position => _position;

        public bool IsActive => _isActive;


        // Fields

        [SerializeField] private LineRenderer _outerCircle;
        [SerializeField] private LineRenderer _innerCircle;
        [SerializeField] private Color _outerColor;
        [SerializeField] private Color _innerColor;
        [SerializeField] private float _outerWidth;
        [SerializeField] private float _innerWidth;

        private Vector3 _position;
        private bool _isActive;
        private bool _isInitialized;


        // Methods

        public void SetValue(Vector3 position, bool active)
        {
            if (!_isInitialized) return;

            _isActive = active;
            _position = position;

            gameObject.SetActive(_isActive);

            _outerCircle.SetPosition(0, _position);
            _outerCircle.SetPosition(1, _position);
            _innerCircle.SetPosition(0, _position);
            _innerCircle.SetPosition(1, _position);
        }


        private void Awake()
        {
            if (_outerCircle == null || _innerCircle == null) return;

            _outerCircle.startColor = _outerColor;
            _outerCircle.endColor = _outerColor;
            _outerCircle.startWidth = _outerWidth;
            _outerCircle.endWidth = _outerWidth;
            _innerCircle.startColor = _innerColor;
            _innerCircle.endColor = _innerColor;
            _innerCircle.startWidth = _innerWidth;
            _innerCircle.endWidth = _innerWidth;

            _isInitialized = true;
        }
    }
}