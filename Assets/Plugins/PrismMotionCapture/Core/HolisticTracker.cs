using Google.Protobuf;
using Mediapipe;
using Mediapipe.Unity;
using Mediapipe.Unity.Experimental;
using System;
using System.Collections;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEngine;
using UnityEngine.Events;
using Stopwatch = System.Diagnostics.Stopwatch;

namespace PMC
{
    public sealed class HolisticTracker : MonoBehaviour
    {
        // Fields

        [SerializeField] private ImageSource _imageSource;
        [Space]
        [SerializeField] private InferenceMode _inferenceMode = InferenceMode.CPU;
        [SerializeField] private ModelComplexity modelComplexity = ModelComplexity.Full;
        [SerializeField] private bool _refineFaceLandmarks = true;
        [SerializeField] private bool _smoothLandmarks = true;
        [SerializeField] private bool _enableSegmentation = false;
        [SerializeField] private bool _smoothSegmentation = false;
        [SerializeField, Range(0f, 1f)] private float _detectionConfidence = 0.5f;
        [SerializeField, Range(0f, 1f)] private float _trackingConfidence = 0.5f;
        [Space]
        [SerializeField] private TextAsset _CPUConfig;
        [SerializeField] private TextAsset _GPUConfig;
        [SerializeField] private TextAsset _OpenGLESConfig;

        private CalculatorGraph _calculatorGraph;
        private TextureFramePool _textureFramePool;
        private Stopwatch _stopwatch;
        private OutputStream<Detection> _poseDetectionStream;
        private OutputStream<NormalizedLandmarkList> _poseLandmarksStream;
        private OutputStream<NormalizedLandmarkList> _faceLandmarksStream;
        private OutputStream<NormalizedLandmarkList> _leftHandLandmarksStream;
        private OutputStream<NormalizedLandmarkList> _rightHandLandmarksStream;
        private OutputStream<LandmarkList> _poseWorldLandmarksStream;
        private OutputStream<ImageFrame> _segmentationMaskStream;
        private OutputStream<NormalizedRect> _poseRoiStream;

        private const long _timeoutMicrosec = 100000;


        // Events

        private event EventHandler<OutputStream<Detection>.OutputEventArgs> OnPoseDetectionOutput
        {
            add => _poseDetectionStream.AddListener(value, _timeoutMicrosec);
            remove => _poseDetectionStream.RemoveListener(value);
        }

        private event EventHandler<OutputStream<NormalizedLandmarkList>.OutputEventArgs> OnPoseLandmarksOutput
        {
            add => _poseLandmarksStream.AddListener(value, _timeoutMicrosec);
            remove => _poseLandmarksStream.RemoveListener(value);
        }

        private event EventHandler<OutputStream<NormalizedLandmarkList>.OutputEventArgs> OnFaceLandmarksOutput
        {
            add => _faceLandmarksStream.AddListener(value, _timeoutMicrosec);
            remove => _faceLandmarksStream.RemoveListener(value);
        }

        private event EventHandler<OutputStream<NormalizedLandmarkList>.OutputEventArgs> OnLeftHandLandmarksOutput
        {
            add => _leftHandLandmarksStream.AddListener(value, _timeoutMicrosec);
            remove => _leftHandLandmarksStream.RemoveListener(value);
        }

        private event EventHandler<OutputStream<NormalizedLandmarkList>.OutputEventArgs> OnRightHandLandmarksOutput
        {
            add => _rightHandLandmarksStream.AddListener(value, _timeoutMicrosec);
            remove => _rightHandLandmarksStream.RemoveListener(value);
        }

        private event EventHandler<OutputStream<LandmarkList>.OutputEventArgs> OnPoseWorldLandmarksOutput
        {
            add => _poseWorldLandmarksStream.AddListener(value, _timeoutMicrosec);
            remove => _poseWorldLandmarksStream.RemoveListener(value);
        }

        private event EventHandler<OutputStream<ImageFrame>.OutputEventArgs> OnSegmentationMaskOutput
        {
            add => _segmentationMaskStream.AddListener(value, _timeoutMicrosec);
            remove => _segmentationMaskStream.RemoveListener(value);
        }

        private event EventHandler<OutputStream<NormalizedRect>.OutputEventArgs> OnPoseRoiOutput
        {
            add => _poseRoiStream.AddListener(value, _timeoutMicrosec);
            remove => _poseRoiStream.RemoveListener(value);
        }


        // Actions

        public UnityAction<Detection> OnPoseDetectionAction { get; set; }

        public UnityAction<NormalizedLandmarkList> OnPoseLandmarksAction { get; set; }

        public UnityAction<NormalizedLandmarkList> OnFaceLandmarksAction { get; set; }

        public UnityAction<NormalizedLandmarkList> OnLeftHandLandmarksAction { get; set; }

        public UnityAction<NormalizedLandmarkList> OnRightHandLandmarksAction { get; set; }

        public UnityAction<LandmarkList> OnPoseWorldLandmarksAction { get; set; }

        public UnityAction<ImageFrame> OnSegmentationMaskAction { get; set; }

        public UnityAction<NormalizedRect> OnPoseRoiAction { get; set; }


        // Methods

        private void Awake()
        {
            _stopwatch = new Stopwatch();
            _stopwatch.Start();
        }

        private IEnumerator Start()
        {
            if (_inferenceMode == InferenceMode.GPU)
            {
                Debug.LogWarning("GPU not supported.");

                _inferenceMode = InferenceMode.CPU;
            }

            IResourceManager resourceManager = new StreamingAssetsResourceManager();

            var faceLandmarkPath = _refineFaceLandmarks ? "face_landmark_with_attention.bytes" : "face_landmark.bytes";
            var poseLandmarkPath = modelComplexity switch
            {
                ModelComplexity.Lite => "pose_landmark_lite.bytes",
                ModelComplexity.Full => "pose_landmark_full.bytes",
                ModelComplexity.Heavy => "pose_landmark_heavy.bytes",

                _ => throw new InternalException($"Invalid model complexity: {modelComplexity}"),
            };

            yield return resourceManager.PrepareAssetAsync("face_detection_short_range.bytes");
            yield return resourceManager.PrepareAssetAsync(faceLandmarkPath);
            yield return resourceManager.PrepareAssetAsync("iris_landmark.bytes");
            yield return resourceManager.PrepareAssetAsync("hand_landmark_full.bytes");
            yield return resourceManager.PrepareAssetAsync("hand_recrop.bytes");
            yield return resourceManager.PrepareAssetAsync("handedness.txt");
            yield return resourceManager.PrepareAssetAsync("palm_detection_full.bytes");
            yield return resourceManager.PrepareAssetAsync("pose_detection.bytes");
            yield return resourceManager.PrepareAssetAsync(poseLandmarkPath);

            _calculatorGraph = new CalculatorGraph();

            _poseDetectionStream = new OutputStream<Detection>(_calculatorGraph, "pose_detection", true);
            _poseLandmarksStream = new OutputStream<NormalizedLandmarkList>(_calculatorGraph, "pose_landmarks", true);
            _faceLandmarksStream = new OutputStream<NormalizedLandmarkList>(_calculatorGraph, "face_landmarks", true);
            _leftHandLandmarksStream = new OutputStream<NormalizedLandmarkList>(_calculatorGraph, "left_hand_landmarks", true);
            _rightHandLandmarksStream = new OutputStream<NormalizedLandmarkList>(_calculatorGraph, "right_hand_landmarks", true);
            _poseWorldLandmarksStream = new OutputStream<LandmarkList>(_calculatorGraph, "pose_world_landmarks", true);
            _segmentationMaskStream = new OutputStream<ImageFrame>(_calculatorGraph, "segmentation_mask", true);
            _poseRoiStream = new OutputStream<NormalizedRect>(_calculatorGraph, "pose_roi", true);

            using var validatedGraphConfig = new ValidatedGraphConfig();

            var config = _inferenceMode switch
            {
                InferenceMode.CPU => _CPUConfig,
                InferenceMode.GPU => _GPUConfig,
                InferenceMode.OpenGLES => _OpenGLESConfig,

                _ => throw new IndexOutOfRangeException(),
            };

            var baseConfig = CalculatorGraphConfig.Parser.ParseFromTextFormat(config.text);

            validatedGraphConfig.Initialize(baseConfig);

            var extensionRegistry = new ExtensionRegistry()
            {
                TensorsToDetectionsCalculatorOptions.Extensions.Ext,
                ThresholdingCalculatorOptions.Extensions.Ext
            };

            var cannonicalizedConfig = validatedGraphConfig.Config(extensionRegistry);

            var poseDetectionCalculatorPattern = new Regex("__posedetection[a-z]+__TensorsToDetectionsCalculator$");
            var tensorsToDetectionsCalculators = cannonicalizedConfig.Node.Where((node) => poseDetectionCalculatorPattern.Match(node.Name).Success).ToList();

            var poseTrackingCalculatorPattern = new Regex("tensorstoposelandmarksandsegmentation__ThresholdingCalculator$");
            var thresholdingCalculators = cannonicalizedConfig.Node.Where((node) => poseTrackingCalculatorPattern.Match(node.Name).Success).ToList();

            foreach (var calculator in tensorsToDetectionsCalculators)
            {
                if (calculator.Options.HasExtension(TensorsToDetectionsCalculatorOptions.Extensions.Ext))
                {
                    var options = calculator.Options.GetExtension(TensorsToDetectionsCalculatorOptions.Extensions.Ext);

                    options.MinScoreThresh = _detectionConfidence;
                }
            }

            foreach (var calculator in thresholdingCalculators)
            {
                if (calculator.Options.HasExtension(ThresholdingCalculatorOptions.Extensions.Ext))
                {
                    var options = calculator.Options.GetExtension(ThresholdingCalculatorOptions.Extensions.Ext);

                    options.Threshold = _trackingConfidence;
                }
            }

            _calculatorGraph.Initialize(cannonicalizedConfig);

            OnPoseDetectionOutput += OnPoseDetection;
            OnPoseLandmarksOutput += OnPoseLandmarks;
            OnFaceLandmarksOutput += OnFaceLandmarks;
            OnLeftHandLandmarksOutput += OnLeftHandLandmarks;
            OnRightHandLandmarksOutput += OnRightHandLandmarks;
            OnPoseWorldLandmarksOutput += OnPoseWorldLandmarks;
            OnSegmentationMaskOutput += OnSegmentationMask;
            OnPoseRoiOutput += OnPoseRoi;

            var packet = new PacketMap();

            packet.Emplace("input_rotation", Packet.CreateInt(0));
            packet.Emplace("input_horizontally_flipped", Packet.CreateBool(false));
            packet.Emplace("input_vertically_flipped", Packet.CreateBool(false));

            packet.Emplace("output_rotation", Packet.CreateInt(0));
            packet.Emplace("output_horizontally_flipped", Packet.CreateBool(false));
            packet.Emplace("output_vertically_flipped", Packet.CreateBool(false));

            packet.Emplace("refine_face_landmarks", Packet.CreateBool(_refineFaceLandmarks));
            packet.Emplace("model_complexity", Packet.CreateInt((int)modelComplexity));
            packet.Emplace("smooth_landmarks", Packet.CreateBool(_smoothLandmarks));
            packet.Emplace("enable_segmentation", Packet.CreateBool(_enableSegmentation));
            packet.Emplace("smooth_segmentation", Packet.CreateBool(_smoothSegmentation));

            _calculatorGraph.StartRun(packet);

            _textureFramePool = new TextureFramePool((int)_imageSource.Resolution.x, (int)_imageSource.Resolution.y);

            while (true)
            {
                if (_textureFramePool.TryGetTextureFrame(out var textureFrame))
                {
                    var req = textureFrame.ReadTextureAsync(_imageSource.Texture);

                    yield return new WaitUntil(() => req.done);

                    if (req.hasError)
                    {
                        Debug.LogWarning($"Failed to read texture from the image source");

                        continue;
                    }

                    var imageFrame = textureFrame.BuildImageFrame();

                    textureFrame.Release();

                    var imageFramePacket = Packet.CreateImageFrameAt(imageFrame, _stopwatch.ElapsedTicks / (TimeSpan.TicksPerMillisecond / 1000));

                    _calculatorGraph.AddPacketToInputStream("input_video", imageFramePacket);
                }
            }
        }

        private void OnDestroy()
        {
            OnPoseDetectionOutput -= OnPoseDetection;
            OnPoseLandmarksOutput -= OnPoseLandmarks;
            OnFaceLandmarksOutput -= OnFaceLandmarks;
            OnLeftHandLandmarksOutput -= OnLeftHandLandmarks;
            OnRightHandLandmarksOutput -= OnRightHandLandmarks;
            OnPoseWorldLandmarksOutput -= OnPoseWorldLandmarks;
            OnSegmentationMaskOutput -= OnSegmentationMask;
            OnPoseRoiOutput -= OnPoseRoi;

            _textureFramePool?.Dispose();
            _textureFramePool = null;

            if (_calculatorGraph != null)
            {
                try
                {
                    _calculatorGraph.CloseAllPacketSources();
                }
                catch (BadStatusException e)
                {
                    Debug.LogError(e);
                }

                try
                {
                    _calculatorGraph.WaitUntilDone();
                }
                catch (BadStatusException e)
                {
                    Debug.LogError(e);
                }

                _calculatorGraph.Dispose();
                _calculatorGraph = null;
            }

            if (_stopwatch != null && _stopwatch.IsRunning)
            {
                _stopwatch.Stop();
            }

            _poseDetectionStream?.Dispose();
            _poseDetectionStream = null;

            _poseLandmarksStream?.Dispose();
            _poseLandmarksStream = null;

            _faceLandmarksStream?.Dispose();
            _faceLandmarksStream = null;

            _leftHandLandmarksStream?.Dispose();
            _leftHandLandmarksStream = null;

            _rightHandLandmarksStream?.Dispose();
            _rightHandLandmarksStream = null;

            _poseWorldLandmarksStream?.Dispose();
            _poseWorldLandmarksStream = null;

            _segmentationMaskStream?.Dispose();
            _segmentationMaskStream = null;

            _poseRoiStream?.Dispose();
            _poseRoiStream = null;
        }


        private void OnPoseDetection(object stream, OutputStream<Detection>.OutputEventArgs eventArgs) => OnPoseDetectionAction?.Invoke(eventArgs.Get());

        private void OnPoseLandmarks(object stream, OutputStream<NormalizedLandmarkList>.OutputEventArgs eventArgs) => OnPoseLandmarksAction?.Invoke(eventArgs.Get());

        private void OnFaceLandmarks(object stream, OutputStream<NormalizedLandmarkList>.OutputEventArgs eventArgs) => OnFaceLandmarksAction?.Invoke(eventArgs.Get());

        private void OnLeftHandLandmarks(object stream, OutputStream<NormalizedLandmarkList>.OutputEventArgs eventArgs) => OnLeftHandLandmarksAction?.Invoke(eventArgs.Get());

        private void OnRightHandLandmarks(object stream, OutputStream<NormalizedLandmarkList>.OutputEventArgs eventArgs) => OnRightHandLandmarksAction?.Invoke(eventArgs.Get());

        private void OnPoseWorldLandmarks(object stream, OutputStream<LandmarkList>.OutputEventArgs eventArgs) => OnPoseWorldLandmarksAction?.Invoke(eventArgs.Get());

        private void OnSegmentationMask(object stream, OutputStream<ImageFrame>.OutputEventArgs eventArgs) => OnSegmentationMaskAction?.Invoke(eventArgs.packet?.Get());

        private void OnPoseRoi(object stream, OutputStream<NormalizedRect>.OutputEventArgs eventArgs) => OnPoseRoiAction?.Invoke(eventArgs.Get());
    }

    public enum InferenceMode
    {
        None,
        CPU,
        GPU,
        OpenGLES,
    }

    public enum ModelComplexity
    {
        Lite = 0,
        Full = 1,
        Heavy = 2,
    }
}