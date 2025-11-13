using Mediapipe;
using Mediapipe.Tasks.Core;
using Mediapipe.Tasks.Vision.HolisticLandmarker;
using Mediapipe.Unity;
using Mediapipe.Unity.Experimental;
using System;
using System.Collections;
using System.Collections.Concurrent;
using System.IO;
using System.Threading;
using UnityEngine;
using UnityEngine.Rendering;
using RunningMode = Mediapipe.Tasks.Vision.Core.RunningMode;
using Stopwatch = System.Diagnostics.Stopwatch;

namespace PMC
{
    [AddComponentMenu("Prism Motion Capture/Prism Tracker")]
    public sealed class PrismTracker : MonoBehaviour
    {
        // Fields

        [SerializeField] private ImageSource _imageSource;
        [SerializeField] private ImageReadMode _imageReadMode = ImageReadMode.CPUAsync;
        [SerializeField] private string _modelAssetPath = "holistic_landmarker.bytes";
        [SerializeField, Range(0f, 1f)] private float _minFaceDetectionConfidence = 0.5f;
        [SerializeField, Range(0f, 1f)] private float _minFaceSuppressionThreshold = 0.5f;
        [SerializeField, Range(0f, 1f)] private float _minFaceLandmarksConfidence = 0.5f;
        [SerializeField, Range(0f, 1f)] private float _minPoseDetectionConfidence = 0.5f;
        [SerializeField, Range(0f, 1f)] private float _minPoseSuppressionThreshold = 0.5f;
        [SerializeField, Range(0f, 1f)] private float _minPoseLandmarksConfidence = 0.5f;
        [SerializeField, Range(0f, 1f)] private float _minHandLandmarksConfidence = 0.5f;
        [SerializeField] private bool _outputFaceBlendshapes = true;
        [SerializeField] private bool _outputSegmentationMask = true;
        [SerializeField] private PrismTrackerPreview _preview;
        [SerializeField, HideInInspector] private BaseOptions.Delegate _delegate;
        [SerializeField, HideInInspector] private RunningMode _runningMode = RunningMode.LIVE_STREAM;

        private HolisticLandmarker _holisticLandmarker;
        private TextureFramePool _textureFramePool;
        private Stopwatch _stopwatch;

        private bool _initialized = false;
        private int _resultCallbackCount = 0;
        private float _fpsTimer = 0f;


        // Properties

        public int MediaPipeFPS { get; private set; }

        public ConcurrentQueue<HolisticLandmarkerResult> ResultQueue { get; private set; } = new();


        // Events

        public event Action<HolisticLandmarkerResult> OnCallback;


        // Methods

        private void Awake()
        {
            if (_imageSource == null || !enabled) return;

            ResourceUtil.EnableCustomResolver();

            var assetPath = Path.Combine(Application.streamingAssetsPath, _modelAssetPath);
            var assetName = Path.GetFileName(assetPath);

            if (File.Exists(assetPath))
            {
                ResourceUtil.SetAssetPath(assetName, assetPath);
            }
            else
            {
                Debug.LogError($"Model asset not found at path: {assetPath}");

                return;
            }

#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN || UNITY_EDITOR_OSX || UNITY_STANDALONE_OSX
            _delegate = BaseOptions.Delegate.CPU;
#else
            _delegate = BaseOptions.Delegate.GPU;
#endif

            var baseOptions = new BaseOptions(_delegate, assetName);

            var options = new HolisticLandmarkerOptions(
                baseOptions,
                _runningMode,
                _minFaceDetectionConfidence,
                _minFaceSuppressionThreshold,
                _minFaceLandmarksConfidence,
                _minPoseDetectionConfidence,
                _minPoseSuppressionThreshold,
                _minPoseLandmarksConfidence,
                _minHandLandmarksConfidence,
                _outputFaceBlendshapes,
                _outputSegmentationMask,
                resultCallback: (_runningMode == RunningMode.LIVE_STREAM) ? ResultCallback : null);

            _holisticLandmarker = HolisticLandmarker.CreateFromOptions(options, GpuManager.GpuResources);

            _textureFramePool = new TextureFramePool((int)_imageSource.Resolution.x, (int)_imageSource.Resolution.y, TextureFormat.RGBA32, 10);

            _initialized = true;
        }

        private IEnumerator Start()
        {
            if (!_initialized) yield break;

            _stopwatch = Stopwatch.StartNew();

            var req = (AsyncGPUReadbackRequest)default;

            var waitUntilReqDone = new WaitUntil(() => req.done);
            var waitForEndOfFrame = new WaitForEndOfFrame();

            var canUseGpuImage = SystemInfo.graphicsDeviceType == GraphicsDeviceType.OpenGLES3 && GpuManager.GpuResources != null;

            using var glContext = canUseGpuImage ? GpuManager.GetGlContext() : null;

            while (true)
            {
                if (_textureFramePool.TryGetTextureFrame(out var textureFrame))
                {
                    Image image = default;

                    switch (_imageReadMode)
                    {
                        case ImageReadMode.CPU:
                            {
                                yield return waitForEndOfFrame;

                                textureFrame.ReadTextureOnCPU(_imageSource.Texture);
                                image = textureFrame.BuildCPUImage();
                                textureFrame.Release();

                                break;
                            }
                        case ImageReadMode.CPUAsync:
                            {
                                req = textureFrame.ReadTextureAsync(_imageSource.Texture);

                                yield return waitUntilReqDone;

                                if (req.hasError)
                                {
                                    Debug.LogWarning($"Failed to read texture from the image source");

                                    continue;
                                }

                                image = textureFrame.BuildCPUImage();

                                textureFrame.Release();

                                break;
                            }
                        case ImageReadMode.GPU:
                            {
                                if (!canUseGpuImage)
                                {
                                    throw new Exception("ImageReadMode.GPU is not supported");
                                }

                                textureFrame.ReadTextureOnGPU(_imageSource.Texture);
                                image = textureFrame.BuildGPUImage(glContext);

                                yield return waitForEndOfFrame;

                                break;
                            }
                    }

                    _holisticLandmarker.DetectAsync(image, _stopwatch.ElapsedTicks / 10000);

                    image.Dispose();

                    if (_imageReadMode == ImageReadMode.GPU)
                    {
                        textureFrame.Release();
                    }
                }
                else
                {
                    yield return new WaitForEndOfFrame();
                }
            }
        }

        private void Update()
        {
            if (ResultQueue.TryDequeue(out var result))
            {
                _preview.Landmarks = result.poseWorldLandmarks.landmarks;

                OnCallback?.Invoke(result);
            }

            _fpsTimer += Time.deltaTime;

            if (_fpsTimer >= 1f)
            {
                MediaPipeFPS = Interlocked.Exchange(ref _resultCallbackCount, 0);

                _fpsTimer -= 1f;
            }
        }

        private void OnDestroy()
        {
            _holisticLandmarker?.Close();
            _holisticLandmarker = null;

            _textureFramePool?.Dispose();
            _textureFramePool = null;

            _stopwatch?.Stop();
        }

        private void ResultCallback(in HolisticLandmarkerResult holisticLandmarkerResult, Image image, long timestampMillisec)
        {
            ResultQueue.Enqueue(holisticLandmarkerResult);

            Interlocked.Increment(ref _resultCallbackCount);
        }
    }

    public enum ImageReadMode
    {
        CPU,
        CPUAsync,
        GPU,
    }
}