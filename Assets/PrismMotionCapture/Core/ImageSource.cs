using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Video;

namespace PMC
{
    [AddComponentMenu("Prism Motion Capture/Image Source")]
    public sealed class ImageSource : MonoBehaviour
    {
        // Properties

        public RenderTexture Texture => _buffer;

        public Renderer Renderer => _renderer;

        public Vector2 Resolution => _resolution;


        // Fields

        [SerializeField] private SourceType _sourceType = SourceType.Texture;
        [SerializeField] private bool _isFlipX = false;
        [SerializeField] private Texture2D _texture;
        [SerializeField] private VideoPlayer _videoPlayer;
        [SerializeField] private string _webcamName = "";
        [SerializeField] private int _webcamFrameRate = 30;
        [SerializeField] private Vector2Int _webcamResolution = new(1920, 1080);
        [SerializeField] private RenderMode _renderMode = RenderMode.None;
        [SerializeField] private RenderTexture _renderTexture;
        [SerializeField] private Renderer _renderer;
        [SerializeField] private bool _useAutoSelect = true;
        [SerializeField] private string _propertyName;
        [SerializeField] private RawImage _rawImage;

        private WebCamTexture _webcam;
        private RenderTexture _buffer;
        private Vector2Int _resolution;


        // Methods

        private void Awake()
        {
            switch (_sourceType)
            {
                case SourceType.Texture:
                    {
                        if (_texture)
                        {
                            _resolution = new Vector2Int(_texture.width, _texture.height);

                            _buffer = new RenderTexture(_resolution.x, _resolution.y, 0);

                            TextureBlit(_texture, _buffer, _isFlipX);
                        }

                        break;
                    }
                case SourceType.Video:
                    {
                        if (_videoPlayer)
                        {
                            _resolution = new Vector2Int((int)_videoPlayer.width, (int)_videoPlayer.height);

                            _buffer = new RenderTexture(_resolution.x, _resolution.y, 0);
                        }

                        break;
                    }
                case SourceType.Webcam:
                    {
                        _resolution = _webcamResolution;

                        _buffer = new RenderTexture(_resolution.x, _resolution.y, 0);

                        _webcam = new WebCamTexture(
                            _webcamName,
                            _resolution.x,
                            _resolution.y,
                            _webcamFrameRate);

                        _webcam.Play();

                        break;
                    }
            }

            switch (_renderMode)
            {
                case RenderMode.Renderer:
                    {
                        var aspect = (float)_resolution.x / _resolution.y;

                        if (_renderer.transform.localScale.x / _renderer.transform.localScale.y != aspect)
                        {
                            _renderer.transform.localScale = new Vector3(
                            _renderer.transform.localScale.x * aspect,
                            _renderer.transform.localScale.y,
                            _renderer.transform.localScale.z);
                        }

                        break;
                    }
                case RenderMode.RawImage:
                    {
                        var aspect = (float)_resolution.x / _resolution.y;

                        if (_rawImage.rectTransform.localScale.x / _rawImage.rectTransform.localScale.y != aspect)
                        {
                            _rawImage.rectTransform.localScale = new Vector3(
                            _rawImage.rectTransform.localScale.x * aspect,
                            _rawImage.rectTransform.localScale.y,
                            _rawImage.rectTransform.localScale.z);
                        }

                        break;
                    }
            }
        }

        private void Update()
        {
            switch (_sourceType)
            {
                case SourceType.Video:
                    {
                        if (_videoPlayer && _videoPlayer.texture)
                        {
                            TextureBlit(_videoPlayer.texture, _buffer, _isFlipX);
                        }

                        break;
                    }
                case SourceType.Webcam:
                    {
                        if (_webcam && _webcam.didUpdateThisFrame)
                        {
                            TextureBlit(_webcam, _buffer, _isFlipX);
                        }

                        break;
                    }
            }

            switch (_renderMode)
            {
                case RenderMode.RenderTexture:
                    {
                        if (_renderTexture)
                        {
                            _renderTexture = _buffer;
                        }

                        break;
                    }
                case RenderMode.Renderer:
                    {
                        if (_renderer && _renderer.material)
                        {
                            if (_useAutoSelect)
                            {
                                _renderer.material.mainTexture = _buffer;
                            }
                            else
                            {
                                _renderer.material.SetTexture(_propertyName, _buffer);
                            }
                        }

                        break;
                    }
                case RenderMode.RawImage:
                    {
                        if (_rawImage && _rawImage.material)
                        {
                            _rawImage.texture = _buffer;
                        }

                        break;
                    }
            }
        }

        private void OnDestroy()
        {
            if (_webcam != null)
            {
                Destroy(_webcam);

                _webcam = null;
            }

            if (_buffer != null)
            {
                Destroy(_buffer);

                _buffer = null;
            }
        }

        private void TextureBlit(Texture srcTexture, RenderTexture dstTexture, bool isFlipX = false)
        {
            if (srcTexture == null || dstTexture == null) return;

            var aspect1 = (float)srcTexture.width / srcTexture.height;
            var aspect2 = (float)dstTexture.width / dstTexture.height;

            var scale = Vector2.Min(Vector2.one, new Vector2(aspect2 / aspect1, aspect1 / aspect2));
            if (isFlipX) scale.x *= -1;

            var offset = (Vector2.one - scale) / 2;

            Graphics.Blit(srcTexture, dstTexture, scale, offset);
        }
    }

    public enum SourceType
    {
        Texture,
        Video,
        Webcam,
    }

    public enum RenderMode
    {
        None,
        RenderTexture,
        Renderer,
        RawImage,
    }
}