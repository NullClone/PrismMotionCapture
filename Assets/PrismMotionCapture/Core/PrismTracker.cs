using Google.Protobuf;
using Mediapipe;
using Mediapipe.Unity;
using Mediapipe.Unity.Experimental;
using System;
using System.Collections;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEngine;
using Stopwatch = System.Diagnostics.Stopwatch;

namespace PMC
{
    [AddComponentMenu("Prism Motion Capture/Prism Tracker")]
    public sealed class PrismTracker : MonoBehaviour
    {
        // Fields

        [Header("Input Settings")]
        [SerializeField] private ImageSource _imageSource;

        [Header("Inference Settings")]
        [SerializeField] private InferenceMode _inferenceMode = InferenceMode.CPU;
        [SerializeField] private ModelComplexity modelComplexity = ModelComplexity.Full;
        [SerializeField, Range(0f, 1f)] private float _detectionConfidence = 0.5f;
        [SerializeField, Range(0f, 1f)] private float _trackingConfidence = 0.5f;

        [Header("Holistic Options")]
        [SerializeField] private bool _refineFaceLandmarks = true;
        [SerializeField] private bool _smoothLandmarks = true;
        [SerializeField] private bool _enableSegmentation = false;
        [SerializeField] private bool _smoothSegmentation = false;

        [Header("Debug")]
        [SerializeField] private PoseWorldLandmarkListAnnotationController2 _poseWorldLandmarkListAnnotationController2;
        [SerializeField] private HandLandmarkListAnnotationController _handLandmarkListAnnotationController;

        [SerializeField, HideInInspector] private TextAsset _CPUConfig;
        [SerializeField, HideInInspector] private TextAsset _GPUConfig;
        [SerializeField, HideInInspector] private TextAsset _OpenGLESConfig;

        private CalculatorGraph _calculatorGraph;
        private TextureFramePool _textureFramePool;
        private Stopwatch _stopwatch = new();

        private OutputStream<Detection> _poseDetectionStream;
        private OutputStream<NormalizedLandmarkList> _poseLandmarksStream;
        private OutputStream<NormalizedLandmarkList> _faceLandmarksStream;
        private OutputStream<NormalizedLandmarkList> _leftHandLandmarksStream;
        private OutputStream<NormalizedLandmarkList> _rightHandLandmarksStream;
        private OutputStream<LandmarkList> _poseWorldLandmarksStream;
        private OutputStream<ImageFrame> _segmentationMaskStream;
        private OutputStream<NormalizedRect> _poseRoiStream;

        public event Action<Detection> OnPoseDetection;
        public event Action<NormalizedLandmarkList> OnPoseLandmarks;
        public event Action<NormalizedLandmarkList> OnFaceLandmarks;
        public event Action<NormalizedLandmarkList> OnLeftHandLandmarks;
        public event Action<NormalizedLandmarkList> OnRightHandLandmarks;
        public event Action<LandmarkList> OnPoseWorldLandmarks;
        public event Action<ImageFrame> OnSegmentationMask;
        public event Action<NormalizedRect> OnPoseRoi;

        private const string PoseDetectionStream = "pose_detection";
        private const string PoseLandmarksStream = "pose_landmarks";
        private const string FaceLandmarksStream = "face_landmarks";
        private const string LeftHandLandmarksStream = "left_hand_landmarks";
        private const string RightHandLandmarksStream = "right_hand_landmarks";
        private const string PoseWorldLandmarksStream = "pose_world_landmarks";
        private const string SegmentationMaskStream = "segmentation_mask";
        private const string PoseRoiStream = "pose_roi";
        private const string InputVideoStream = "input_video";
        private const long TimeoutMicrosec = 100000;

        private static readonly Regex PoseDetectionCalculatorPattern = new("__posedetection[a-z]+__TensorsToDetectionsCalculator$");
        private static readonly Regex PoseTrackingCalculatorPattern = new("tensorstoposelandmarksandsegmentation__ThresholdingCalculator$");


        // Methods

        private void Awake()
        {
            if (_poseWorldLandmarkListAnnotationController2 != null)
            {
                _poseWorldLandmarkListAnnotationController2.rotationAngle = RotationAngle.Rotation180;
                _poseWorldLandmarkListAnnotationController2.isMirrored = true;
                _poseWorldLandmarkListAnnotationController2.VisualizeZ = true;
            }

            if (_handLandmarkListAnnotationController != null)
            {
                _handLandmarkListAnnotationController.rotationAngle = RotationAngle.Rotation180;
                _handLandmarkListAnnotationController.isMirrored = true;
            }
        }

        private void OnEnable()
        {
            OnPoseWorldLandmarks += OnPoseWorldLandmarksUpdate;
            OnLeftHandLandmarks += OnLeftHandLandmarksUpdate;
        }

        private IEnumerator Start()
        {
            _stopwatch.Start();

            if (_inferenceMode == InferenceMode.GPU)
            {
                Debug.LogWarning("GPU inference is not supported on this platform. Falling back to CPU.");

                _inferenceMode = InferenceMode.CPU;
            }

            yield return PrepareResources();

            InitializeGraph();

            yield return StartRun();
        }

        private IEnumerator PrepareResources()
        {
            IResourceManager resourceManager = new LocalResourceManager();

            var faceLandmarkPath = _refineFaceLandmarks ? "face_landmark_with_attention.bytes" : "face_landmark.bytes";
            var poseLandmarkPath = modelComplexity switch
            {
                ModelComplexity.Lite => "pose_landmark_lite.bytes",
                ModelComplexity.Full => "pose_landmark_full.bytes",
                ModelComplexity.Heavy => "pose_landmark_heavy.bytes",
                _ => throw new ArgumentOutOfRangeException(nameof(modelComplexity), $"Invalid model complexity: {modelComplexity}"),
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
        }

        private IEnumerator StartRun()
        {
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
                if (!_textureFramePool.TryGetTextureFrame(out var textureFrame))
                {
                    yield return null;

                    continue;
                }

                var req = textureFrame.ReadTextureAsync(_imageSource.Texture);

                yield return new WaitUntil(() => req.done);

                if (req.hasError)
                {
                    Debug.LogWarning("Failed to read texture from the image source.");

                    textureFrame.Release();

                    continue;
                }

                var imageFrame = textureFrame.BuildImageFrame();
                textureFrame.Release();

                var timestamp = _stopwatch.ElapsedTicks / (TimeSpan.TicksPerMillisecond / 1000);
                var imageFramePacket = Packet.CreateImageFrameAt(imageFrame, timestamp);

                _calculatorGraph.AddPacketToInputStream(InputVideoStream, imageFramePacket);
            }
        }

        private void InitializeGraph()
        {
            _calculatorGraph = new CalculatorGraph();

            _poseDetectionStream = new OutputStream<Detection>(_calculatorGraph, PoseDetectionStream, true);
            _poseLandmarksStream = new OutputStream<NormalizedLandmarkList>(_calculatorGraph, PoseLandmarksStream, true);
            _faceLandmarksStream = new OutputStream<NormalizedLandmarkList>(_calculatorGraph, FaceLandmarksStream, true);
            _leftHandLandmarksStream = new OutputStream<NormalizedLandmarkList>(_calculatorGraph, LeftHandLandmarksStream, true);
            _rightHandLandmarksStream = new OutputStream<NormalizedLandmarkList>(_calculatorGraph, RightHandLandmarksStream, true);
            _poseWorldLandmarksStream = new OutputStream<LandmarkList>(_calculatorGraph, PoseWorldLandmarksStream, true);
            _segmentationMaskStream = new OutputStream<ImageFrame>(_calculatorGraph, SegmentationMaskStream, true);
            _poseRoiStream = new OutputStream<NormalizedRect>(_calculatorGraph, PoseRoiStream, true);

            _calculatorGraph.Initialize(GetGraphConfig());

            _poseDetectionStream.AddListener(OnPoseDetectionOutput, TimeoutMicrosec);
            _poseLandmarksStream.AddListener(OnPoseLandmarksOutput, TimeoutMicrosec);
            _faceLandmarksStream.AddListener(OnFaceLandmarksOutput, TimeoutMicrosec);
            _leftHandLandmarksStream.AddListener(OnLeftHandLandmarksOutput, TimeoutMicrosec);
            _rightHandLandmarksStream.AddListener(OnRightHandLandmarksOutput, TimeoutMicrosec);
            _poseWorldLandmarksStream.AddListener(OnPoseWorldLandmarksOutput, TimeoutMicrosec);
            _segmentationMaskStream.AddListener(OnSegmentationMaskOutput, TimeoutMicrosec);
            _poseRoiStream.AddListener(OnPoseRoiOutput, TimeoutMicrosec);
        }

        private CalculatorGraphConfig GetGraphConfig()
        {
            var configAsset = _inferenceMode switch
            {
                InferenceMode.CPU => _CPUConfig,
                InferenceMode.GPU => _GPUConfig,
                InferenceMode.OpenGLES => _OpenGLESConfig,
                _ => throw new ArgumentOutOfRangeException(nameof(_inferenceMode), "Invalid inference mode specified."),
            };

            var baseConfig = CalculatorGraphConfig.Parser.ParseFromTextFormat(configAsset.text);

            using var validatedGraphConfig = new ValidatedGraphConfig();

            validatedGraphConfig.Initialize(baseConfig);

            var extensionRegistry = new ExtensionRegistry()
            {
                TensorsToDetectionsCalculatorOptions.Extensions.Ext,
                ThresholdingCalculatorOptions.Extensions.Ext
            };

            var canonicalizedConfig = validatedGraphConfig.Config(extensionRegistry);

            foreach (var node in canonicalizedConfig.Node.Where(node => PoseDetectionCalculatorPattern.IsMatch(node.Name)))
            {
                if (node.Options.HasExtension(TensorsToDetectionsCalculatorOptions.Extensions.Ext))
                {
                    var options = node.Options.GetExtension(TensorsToDetectionsCalculatorOptions.Extensions.Ext);
                    options.MinScoreThresh = _detectionConfidence;
                }
            }

            foreach (var node in canonicalizedConfig.Node.Where(node => PoseTrackingCalculatorPattern.IsMatch(node.Name)))
            {
                if (node.Options.HasExtension(ThresholdingCalculatorOptions.Extensions.Ext))
                {
                    var options = node.Options.GetExtension(ThresholdingCalculatorOptions.Extensions.Ext);
                    options.Threshold = _trackingConfidence;
                }
            }

            return canonicalizedConfig;
        }

        private void OnDisable()
        {
            OnPoseWorldLandmarks -= OnPoseWorldLandmarksUpdate;
            OnLeftHandLandmarks -= OnLeftHandLandmarksUpdate;
        }

        private void OnDestroy()
        {
            _stopwatch?.Stop();

            _textureFramePool?.Dispose();

            if (_calculatorGraph != null)
            {
                try
                {
                    _calculatorGraph.CloseAllPacketSources();
                    _calculatorGraph.WaitUntilDone();
                }
                catch (BadStatusException e)
                {
                    Debug.LogError($"Failed to shutdown CalculatorGraph: {e}");
                }
                finally
                {
                    _calculatorGraph.Dispose();
                }
            }

            _poseDetectionStream?.Dispose();
            _poseLandmarksStream?.Dispose();
            _faceLandmarksStream?.Dispose();
            _leftHandLandmarksStream?.Dispose();
            _rightHandLandmarksStream?.Dispose();
            _poseWorldLandmarksStream?.Dispose();
            _segmentationMaskStream?.Dispose();
            _poseRoiStream?.Dispose();
        }


        private void OnPoseDetectionOutput(object _, OutputStream<Detection>.OutputEventArgs eventArgs) => OnPoseDetection?.Invoke(eventArgs.Get());

        private void OnPoseLandmarksOutput(object _, OutputStream<NormalizedLandmarkList>.OutputEventArgs eventArgs) => OnPoseLandmarks?.Invoke(eventArgs.Get());

        private void OnFaceLandmarksOutput(object _, OutputStream<NormalizedLandmarkList>.OutputEventArgs eventArgs) => OnFaceLandmarks?.Invoke(eventArgs.Get());

        private void OnLeftHandLandmarksOutput(object _, OutputStream<NormalizedLandmarkList>.OutputEventArgs eventArgs) => OnLeftHandLandmarks?.Invoke(eventArgs.Get());

        private void OnRightHandLandmarksOutput(object _, OutputStream<NormalizedLandmarkList>.OutputEventArgs eventArgs) => OnRightHandLandmarks?.Invoke(eventArgs.Get());

        private void OnPoseWorldLandmarksOutput(object _, OutputStream<LandmarkList>.OutputEventArgs eventArgs) => OnPoseWorldLandmarks?.Invoke(eventArgs.Get());

        private void OnSegmentationMaskOutput(object _, OutputStream<ImageFrame>.OutputEventArgs eventArgs) => OnSegmentationMask?.Invoke(eventArgs.packet?.Get());

        private void OnPoseRoiOutput(object _, OutputStream<NormalizedRect>.OutputEventArgs eventArgs) => OnPoseRoi?.Invoke(eventArgs.Get());


        private void OnPoseWorldLandmarksUpdate(LandmarkList landmarkList) => _poseWorldLandmarkListAnnotationController2?.DrawLater(landmarkList);

        private void OnLeftHandLandmarksUpdate(NormalizedLandmarkList landmarkList) => _handLandmarkListAnnotationController?.DrawLater(landmarkList);
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
        Lite,
        Full,
        Heavy,
    }
}