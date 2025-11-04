using Mediapipe;
using Mediapipe.Tasks.Core;
using Mediapipe.Tasks.Vision.HolisticLandmarker;
using Mediapipe.Unity;
using Mediapipe.Unity.Experimental;
using System;
using System.Collections;
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
        [Space]
        [SerializeField] private BaseOptions.Delegate _delegate = BaseOptions.Delegate.CPU;
        [SerializeField] private RunningMode _runningMode = RunningMode.LIVE_STREAM;
        [SerializeField] private AssetLoaderType _assetLoaderType = AssetLoaderType.Local;
        [SerializeField] private string _modelAssetPath = "holistic_landmarker.bytes";
        [SerializeField] private ImageReadMode _imageReadMode = ImageReadMode.CPUAsync;
        [Space]
        [SerializeField, Range(0f, 1f)] private float _minFaceDetectionConfidence = 0.5f;
        [SerializeField, Range(0f, 1f)] private float _minFaceSuppressionThreshold = 0.5f;
        [SerializeField, Range(0f, 1f)] private float _minFaceLandmarksConfidence = 0.5f;
        [SerializeField, Range(0f, 1f)] private float _minPoseDetectionConfidence = 0.5f;
        [SerializeField, Range(0f, 1f)] private float _minPoseSuppressionThreshold = 0.5f;
        [SerializeField, Range(0f, 1f)] private float _minPoseLandmarksConfidence = 0.5f;
        [SerializeField, Range(0f, 1f)] private float _minHandLandmarksConfidence = 0.5f;
        [SerializeField] private bool _outputFaceBlendshapes = false;
        [SerializeField] private bool _outputSegmentationMask = false;

        private HolisticLandmarker _holisticLandmarker;
        private TextureFramePool _textureFramePool;
        private Stopwatch _stopwatch;


        public event Action<HolisticLandmarkerResult> OnCallback;


        // Methods

        private IEnumerator Start()
        {
            if (_imageSource == null) yield break;

            IResourceManager manager = _assetLoaderType switch
            {
                AssetLoaderType.StreamingAssets => new StreamingAssetsResourceManager(),
                AssetLoaderType.AssetBundle => new AssetBundleResourceManager(""),
                AssetLoaderType.Local => new LocalResourceManager(),
                _ => throw new ArgumentOutOfRangeException(),
            };

            yield return manager.PrepareAssetAsync(_modelAssetPath);

            if (_delegate != BaseOptions.Delegate.CPU)
            {
#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN || UNITY_EDITOR_OSX || UNITY_STANDALONE_OSX
                Debug.LogWarning("GPU delegate selected on PC platform. Forcing fallback to CPU.");

                _delegate = BaseOptions.Delegate.CPU;
#endif
            }

            var baseOptions = new BaseOptions(_delegate, _modelAssetPath);

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

            AsyncGPUReadbackRequest req = default;
            HolisticLandmarkerResult result = default;

            var waitUntilReqDone = new WaitUntil(() => req.done);
            var waitForEndOfFrame = new WaitForEndOfFrame();

            var canUseGpuImage = SystemInfo.graphicsDeviceType == GraphicsDeviceType.OpenGLES3 && GpuManager.GpuResources != null;

            using var glContext = canUseGpuImage ? GpuManager.GetGlContext() : null;

            _stopwatch = new Stopwatch();
            _stopwatch.Start();

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

                    switch (_runningMode)
                    {
                        case RunningMode.IMAGE:
                            {
                                if (_holisticLandmarker.TryDetect(image, ref result))
                                {
                                    OnCallback?.Invoke(result);
                                }
                                else
                                {
                                    OnCallback?.Invoke(default);
                                }

                                result.segmentationMask?.Dispose();

                                break;
                            }
                        case RunningMode.VIDEO:
                            {
                                if (_holisticLandmarker.TryDetectForVideo(image, _stopwatch.ElapsedTicks / 10000, ref result))
                                {
                                    OnCallback?.Invoke(result);
                                }
                                else
                                {
                                    OnCallback?.Invoke(default);
                                }

                                result.segmentationMask?.Dispose();

                                break;
                            }
                        case RunningMode.LIVE_STREAM:
                            {
                                _holisticLandmarker.DetectAsync(image, _stopwatch.ElapsedTicks / 10000);

                                break;
                            }
                    }

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
            OnCallback?.Invoke(holisticLandmarkerResult);
        }
    }

    public enum ImageReadMode
    {
        CPU,
        CPUAsync,
        GPU,
    }

    public enum AssetLoaderType
    {
        StreamingAssets,
        AssetBundle,
        Local,
    }
}