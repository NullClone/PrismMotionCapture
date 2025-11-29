using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Video;

namespace PMC
{
    [AddComponentMenu("Prism Motion Capture/Image Source")]
    public sealed class ImageSource : MonoBehaviour
    {
        // Fields

        public SourceType SourceType = SourceType.Texture;
        public Texture2D Texture2D;
        public VideoPlayer VideoPlayer;
        public string WebcamName;
        public int WebcamFrameRate = 30;
        public Vector2Int WebcamResolution = new(1920, 1080);
        public RenderMode RenderMode = RenderMode.None;
        public RenderTexture RenderTexture;
        public Renderer Renderer;
        public bool UseAutoSelect = true;
        public string PropertyName;
        public RawImage RawImage;

        private WebCamTexture _webcam;
        private RenderTexture _buffer;
        private Vector2Int _resolution;


        // Properties

        public RenderTexture Texture => _buffer;

        public Vector2 Resolution => _resolution;


        // Methods

        private void Awake()
        {
            switch (SourceType)
            {
                case SourceType.Texture:
                    {
                        if (Texture2D)
                        {
                            _resolution = new Vector2Int(Texture2D.width, Texture2D.height);

                            _buffer = new RenderTexture(_resolution.x, _resolution.y, 0);

                            TextureBlit(Texture2D, _buffer);
                        }

                        break;
                    }
                case SourceType.Video:
                    {
                        if (VideoPlayer)
                        {
                            _resolution = new Vector2Int((int)VideoPlayer.width, (int)VideoPlayer.height);

                            _buffer = new RenderTexture(_resolution.x, _resolution.y, 0);
                        }

                        break;
                    }
                case SourceType.Webcam:
                    {
                        _resolution = WebcamResolution;

                        _buffer = new RenderTexture(_resolution.x, _resolution.y, 0);

                        var isFound = false;

                        foreach (var device in WebCamTexture.devices)
                        {
                            if (device.name == WebcamName)
                            {
                                isFound = true;
                            }
                        }

                        if (!isFound)
                        {
                            WebcamName = WebCamTexture.devices[0].name;
                        }

                        _webcam = new WebCamTexture(WebcamName, _resolution.x, _resolution.y, WebcamFrameRate);
                        _webcam.Play();

                        break;
                    }
            }

            switch (RenderMode)
            {
                case RenderMode.Renderer:
                    {
                        var aspect = (float)_resolution.x / _resolution.y;

                        if (Renderer.transform.localScale.x / Renderer.transform.localScale.y != aspect)
                        {
                            Renderer.transform.localScale = new Vector3(
                                Renderer.transform.localScale.x * aspect,
                                Renderer.transform.localScale.y,
                                Renderer.transform.localScale.z);
                        }

                        break;
                    }
                case RenderMode.RawImage:
                    {
                        var aspect = (float)_resolution.x / _resolution.y;

                        if (RawImage.rectTransform.localScale.x / RawImage.rectTransform.localScale.y != aspect)
                        {
                            RawImage.rectTransform.localScale = new Vector3(
                                RawImage.rectTransform.localScale.x * aspect,
                                RawImage.rectTransform.localScale.y,
                                RawImage.rectTransform.localScale.z);
                        }

                        break;
                    }
            }
        }

        private void Update()
        {
            switch (SourceType)
            {
                case SourceType.Video:
                    {
                        if (VideoPlayer && VideoPlayer.texture)
                        {
                            TextureBlit(VideoPlayer.texture, _buffer);
                        }

                        break;
                    }
                case SourceType.Webcam:
                    {
                        if (_webcam && _webcam.didUpdateThisFrame)
                        {
                            TextureBlit(_webcam, _buffer);
                        }

                        break;
                    }
            }

            switch (RenderMode)
            {
                case RenderMode.RenderTexture:
                    {
                        if (RenderTexture)
                        {
                            RenderTexture = _buffer;
                        }

                        break;
                    }
                case RenderMode.Renderer:
                    {
                        if (Renderer && Renderer.material)
                        {
                            if (UseAutoSelect)
                            {
                                Renderer.material.mainTexture = _buffer;
                            }
                            else
                            {
                                Renderer.material.SetTexture(PropertyName, _buffer);
                            }
                        }

                        break;
                    }
                case RenderMode.RawImage:
                    {
                        if (RawImage && RawImage.material)
                        {
                            RawImage.texture = _buffer;
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


        private static void TextureBlit(Texture srcTexture, RenderTexture dstTexture)
        {
            if (srcTexture == null || dstTexture == null) return;

            var aspect1 = (float)srcTexture.width / srcTexture.height;
            var aspect2 = (float)dstTexture.width / dstTexture.height;

            var scale = Vector2.Min(Vector2.one, new Vector2(aspect2 / aspect1, aspect1 / aspect2));

            var offset = (Vector2.one - scale) / 2;

            Graphics.Blit(srcTexture, dstTexture, scale, offset);
        }
    }
}