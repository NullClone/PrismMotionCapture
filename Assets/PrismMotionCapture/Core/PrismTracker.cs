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
        [SerializeField] private BaseOptions.Delegate _delegate;
        [SerializeField] private RunningMode _runningMode;

        private HolisticLandmarker _holisticLandmarker;
        private TextureFramePool _textureFramePool;
        private Stopwatch _stopwatch;

        private const string MODEL_PATH = "holistic_landmarker.bytes";


        public event Action<HolisticLandmarkerResult> OnCallback;


        // Methods

        private IEnumerator Start()
        {
            if (_imageSource == null) yield break;

            IResourceManager manager = new LocalResourceManager();

            yield return manager.PrepareAssetAsync(MODEL_PATH);

            var baseOptions = new BaseOptions(_delegate, MODEL_PATH);

            var options = new HolisticLandmarkerOptions(
                baseOptions: baseOptions,
                runningMode: _runningMode,
                minFaceDetectionConfidence: 0.5f,
                minFaceSuppressionThreshold: 0.5f,
                minFaceLandmarksConfidence: 0.5f,
                minPoseDetectionConfidence: 0.5f,
                minPoseSuppressionThreshold: 0.5f,
                minPoseLandmarksConfidence: 0.5f,
                minHandLandmarksConfidence: 0.5f,
                outputFaceBlendshapes: false,
                outputSegmentationMask: false,
                resultCallback: (_runningMode == RunningMode.LIVE_STREAM) ? ResultCallback : null);

            _holisticLandmarker = HolisticLandmarker.CreateFromOptions(options);

            _textureFramePool = new TextureFramePool((int)_imageSource.Resolution.x, (int)_imageSource.Resolution.y);

            AsyncGPUReadbackRequest req = default;

            var waitUntilReqDone = new WaitUntil(() => req.done);
            var waitForEndOfFrame = new WaitForEndOfFrame();

            _stopwatch = new Stopwatch();
            _stopwatch.Start();

            while (true)
            {
                if (_textureFramePool.TryGetTextureFrame(out var textureFrame))
                {
                    req = textureFrame.ReadTextureAsync(_imageSource.Texture);

                    yield return waitUntilReqDone;

                    if (req.hasError)
                    {
                        Debug.LogWarning($"Failed to read texture from the image source");

                        continue;
                    }

                    var image = textureFrame.BuildCPUImage();

                    textureFrame.Release();

                    var timestampMillisec = _stopwatch.ElapsedTicks / 10000;

                    _holisticLandmarker.DetectAsync(image, timestampMillisec);
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
}