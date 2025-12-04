using Mediapipe;
using Mediapipe.Tasks.Core;
using Mediapipe.Tasks.Vision.HolisticLandmarker;
using Mediapipe.Unity;
using Mediapipe.Unity.Experimental;
using OpenCvSharp;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering;
using Color = UnityEngine.Color;
using Rect = UnityEngine.Rect;
using RunningMode = Mediapipe.Tasks.Vision.Core.RunningMode;
using Stopwatch = System.Diagnostics.Stopwatch;

namespace PMC
{
    [AddComponentMenu("Prism Motion Capture/Prism Tracker")]
    public sealed class PrismTracker : MonoBehaviour
    {
        // Fields

        public ImageSource ImageSource;
        private BaseOptions.Delegate _delegate;
        public TextAsset ModelAsset;
        public ImageReadMode ImageReadMode = ImageReadMode.CPU;

        [Range(1f, 60f)] public float Framerate = 30f;
        private float _trackingFPS = 0f;
        private long _lastFrameTick = 0;

        [Range(0f, 1f)] public float MinFaceDetectionConfidence = 0.5f;
        [Range(0f, 1f)] public float MinFaceSuppressionThreshold = 0.5f;
        [Range(0f, 1f)] public float MinFaceLandmarksConfidence = 0.5f;
        [Range(0f, 1f)] public float MinPoseDetectionConfidence = 0.5f;
        [Range(0f, 1f)] public float MinPoseSuppressionThreshold = 0.5f;
        [Range(0f, 1f)] public float MinPoseLandmarksConfidence = 0.5f;
        [Range(0f, 1f)] public float MinHandLandmarksConfidence = 0.5f;
        public bool OutputFaceBlendshapes = true;
        public bool OutputSegmentationMask = false;

        public bool FlipHorizontally = false;
        public bool FlipVertically = false;

        public Vector3 LandmarkScale = Vector3.one;
        public Vector3 MovementScale = Vector3.one;

        [SerializeField] private bool _enableKalmanFilter = true;
        [SerializeField] private float _timeInterval = 0.45f;
        [SerializeField] private float _noise = 0.4f;
        [SerializeField] private bool _enableOneEuroFilter = true;
        [SerializeField] private float _filterMinCutoff = 1f;
        [SerializeField] private float _filterBeta = 0.1f;
        [SerializeField] private float _filterDcutoff = 1f;
        [SerializeField] private bool _enableGlobalPoseFilter = true;
        [SerializeField] private float _globalPoseFilterMinCutoff = 1f;
        [SerializeField] private float _globalPoseFilterBeta = 0.1f;
        [SerializeField] private float _globalPoseFilterDcutoff = 1f;

        public bool ShowTrackingFPS = true;
        public bool ShowLandmark = true;
        public float LandmarkRadius = 0.01f;
        public Vector3 LandmarkPosition = Vector3.zero;
        public Color LeftLandmarkColor = new(1f, 0.5f, 0f, 1f);
        public Color RightLandmarkColor = new(0f, 1f, 1f, 1f);
        public Color ConnectionColor = new(1f, 1f, 1f, 1f);

        private HolisticLandmarker _holisticLandmarker;
        private TextureFramePool _textureFramePool;
        private Stopwatch _stopwatch;

        private double _fx, _fy, _cx, _cy;

        private Mat _camMatrix;
        private Mat _distCoeffs;
        private Mat _rvec;
        private Mat _tvec;
        private Mat _rotationMatrix;
        private Mat _pnpObjectPointMat;
        private Mat _pnpCamSpacePointMat;

        private Matrix4x4 _invertYM;
        private Matrix4x4 _invertZM;

        private bool _initialized = false;

        private OneEuroFilter<Vector3> _globalAvatarPositionOneEuroFilter;


        private readonly List<Point3f> _objectPointsList = new();
        private readonly List<Point2f> _imagePointsList = new();

        private readonly int[] _pnpLandmarkIndices = new int[]
        {
            (int)PoseLandmark.LeftShoulder,
            (int)PoseLandmark.RightShoulder,
            (int)PoseLandmark.LeftHip,
            (int)PoseLandmark.RightHip,
        };

        private readonly Vector3[] _positions = new Vector3[PoseLandmarkCount];

        private static readonly HashSet<int> _LeftLandmarks = new() { 1, 2, 3, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31 };
        private static readonly HashSet<int> _RightLandmarks = new() { 4, 5, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32 };
        private static readonly List<(int, int)> _Connections = new() {
            (0, 1), (1, 2), (2, 3), (3, 7),(0, 4), (4, 5), (5, 6),
            (6, 8),(9, 10),(11, 13), (13, 15),(15, 17), (15, 19), (15, 21),
            (17, 19),(12, 14), (14, 16),(16, 18), (16, 20), (16, 22),(18, 20),
            (11, 12), (12, 24), (24, 23), (23, 11),(23, 25), (25, 27), (27, 29),
            (27, 31), (29, 31),(24, 26), (26, 28), (28, 30), (28, 32), (30, 32),};


        public const string ModelAssetPath = "holistic_landmarker.bytes";

        public const int PoseLandmarkCount = 33;
        public const int HandLandmarkCount = 21;
        public const int FaceLandmarkCount = 478;
        public const int FaceBlendShapeCount = 52;


        // Properties        

        public bool ActiveFaceLandmark { get; private set; }

        public bool ActivePoseLandmark { get; private set; }

        public bool ActivePoseWorldLandmark { get; private set; }

        public bool ActiveLeftHandLandmark { get; private set; }

        public bool ActiveLeftHandWorldLandmark { get; private set; }

        public bool ActiveRightHandLandmark { get; private set; }

        public bool ActiveRightHandWorldLandmark { get; private set; }

        public bool ActiveFaceBlendShapes { get; private set; }

        public Landmark[] FaceLandmarks { get; private set; } = new Landmark[FaceLandmarkCount].Select(x => new Landmark()).ToArray();

        public Landmark[] PoseLandmarks { get; private set; } = new Landmark[PoseLandmarkCount].Select(x => new Landmark()).ToArray();

        public Landmark[] PoseWorldLandmarks { get; private set; } = new Landmark[PoseLandmarkCount].Select(x => new Landmark()).ToArray();

        public Landmark[] LeftHandLandmarks { get; private set; } = new Landmark[HandLandmarkCount].Select(x => new Landmark()).ToArray();

        public Landmark[] LeftHandWorldLandmarks { get; private set; } = new Landmark[HandLandmarkCount].Select(x => new Landmark()).ToArray();

        public Landmark[] RightHandLandmarks { get; private set; } = new Landmark[HandLandmarkCount].Select(x => new Landmark()).ToArray();

        public Landmark[] RightHandWorldLandmarks { get; private set; } = new Landmark[HandLandmarkCount].Select(x => new Landmark()).ToArray();

        public float[] FaceBlendShapes { get; private set; } = new float[FaceBlendShapeCount];

        public Vector3 GlobalAvatarPosition { get; private set; }


        // Methods

        private void Awake()
        {
            if (ImageSource == null || !enabled) return;

#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN || UNITY_EDITOR_OSX || UNITY_STANDALONE_OSX
            _delegate = BaseOptions.Delegate.CPU;
#else
            _delegate = BaseOptions.Delegate.GPU;
#endif

            var baseOptions = new BaseOptions(_delegate, ModelAssetPath, ModelAsset.bytes);

            var options = new HolisticLandmarkerOptions(
                baseOptions,
                RunningMode.LIVE_STREAM,
                MinFaceDetectionConfidence,
                MinFaceSuppressionThreshold,
                MinFaceLandmarksConfidence,
                MinPoseDetectionConfidence,
                MinPoseSuppressionThreshold,
                MinPoseLandmarksConfidence,
                MinHandLandmarksConfidence,
                OutputFaceBlendshapes,
                OutputSegmentationMask,
                resultCallback: ResultCallback);

            _holisticLandmarker = HolisticLandmarker.CreateFromOptions(options, GpuManager.GpuResources);

            _textureFramePool = new TextureFramePool((int)ImageSource.Resolution.x, (int)ImageSource.Resolution.y, TextureFormat.RGBA32, 10);

            _stopwatch = Stopwatch.StartNew();

            InitializeFilter();
            InitializePnP();

            _initialized = true;
        }

        private IEnumerator Start()
        {
            if (!_initialized) yield break;

            var req = (AsyncGPUReadbackRequest)default;

            var waitUntilReqDone = new WaitUntil(() => req.done);
            var waitForEndOfFrame = new WaitForEndOfFrame();

            var canUseGpuImage = SystemInfo.graphicsDeviceType == GraphicsDeviceType.OpenGLES3 && GpuManager.GpuResources != null;

            using var glContext = canUseGpuImage ? GpuManager.GetGlContext() : null;

            var nextFrameTime = Time.time;

            while (true)
            {
                var interval = 1f / Framerate;

                if (Time.time < nextFrameTime)
                {
                    yield return null;
                    continue;
                }

                nextFrameTime = Time.time + interval;

                if (_textureFramePool.TryGetTextureFrame(out var textureFrame))
                {
                    Image image = default;

                    switch (ImageReadMode)
                    {
                        case ImageReadMode.CPU:
                            {
                                req = textureFrame.ReadTextureAsync(ImageSource.Texture, FlipHorizontally, FlipVertically);

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

                                textureFrame.ReadTextureOnGPU(ImageSource.Texture, FlipHorizontally, FlipVertically);
                                image = textureFrame.BuildGPUImage(glContext);

                                yield return waitForEndOfFrame;

                                break;
                            }
                    }

                    _holisticLandmarker.DetectAsync(image, _stopwatch.ElapsedTicks / 10000);

                    image.Dispose();

                    if (ImageReadMode == ImageReadMode.GPU)
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

        private void OnGUI()
        {
            if (ShowTrackingFPS || !enabled)
            {
                const int Padding = 10;

                var size = Mathf.RoundToInt(0.01f * Screen.width);
                var style = new GUIStyle
                {
                    alignment = TextAnchor.MiddleRight,
                    fontSize = size,
                };

                style.normal.textColor = Color.green;

                var rect = new Rect(Padding, Screen.height - Padding - size, Screen.width - (2f * Padding), size);

                GUI.Label(rect, $"{Mathf.RoundToInt(_trackingFPS)} FPS", style);
            }
        }

        private void OnDrawGizmos()
        {
            if (!Application.isPlaying || !enabled) return;

            if (ShowLandmark)
            {
                //var positions = PoseWorldLandmarks.Select(t => t.Position).ToArray();
                var positions = _positions;

                if (positions == null) return;

                Gizmos.color = ConnectionColor;

                foreach (var conn in _Connections)
                {
                    Gizmos.DrawLine(positions[conn.Item1] + LandmarkPosition, positions[conn.Item2] + LandmarkPosition);
                }

                for (int i = 0; i < positions.Length; i++)
                {
                    if (_LeftLandmarks.Contains(i))
                    {
                        Gizmos.color = LeftLandmarkColor;
                    }

                    if (_RightLandmarks.Contains(i))
                    {
                        Gizmos.color = RightLandmarkColor;
                    }

                    Gizmos.DrawSphere(positions[i] + LandmarkPosition, LandmarkRadius);
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

            _camMatrix?.Dispose();
            _distCoeffs?.Dispose();
            _rvec?.Dispose();
            _tvec?.Dispose();
            _rotationMatrix?.Dispose();
            _pnpObjectPointMat?.Dispose();
            _pnpCamSpacePointMat?.Dispose();
        }

        private void InitializeFilter()
        {
            foreach (var landmark in PoseWorldLandmarks)
            {
                if (_enableKalmanFilter)
                {
                    landmark.KalmanFilter = new KalmanFilter();
                    landmark.KalmanFilter.SetParameter(_timeInterval, _noise);
                    landmark.KalmanFilter.Predict();
                }

                if (_enableOneEuroFilter)
                {
                    landmark.OneEuroFilter = new OneEuroFilter<Vector3>(Framerate, _filterMinCutoff, _filterBeta, _filterDcutoff);
                }
            }

            _globalAvatarPositionOneEuroFilter = new OneEuroFilter<Vector3>(Framerate, _globalPoseFilterMinCutoff, _globalPoseFilterBeta, _globalPoseFilterDcutoff);
        }

        private void InitializePnP()
        {
            var max_d = Mathf.Max(ImageSource.Resolution.x, ImageSource.Resolution.y);
            _fx = max_d;
            _fy = max_d;
            _cx = ImageSource.Resolution.x / 2f;
            _cy = ImageSource.Resolution.y / 2f;

            _camMatrix = new Mat(3, 3, MatType.CV_64FC1);
            _camMatrix.Set(0, 0, _fx);
            _camMatrix.Set(0, 1, 0d);
            _camMatrix.Set(0, 2, _cx);
            _camMatrix.Set(1, 0, 0d);
            _camMatrix.Set(1, 1, _fy);
            _camMatrix.Set(1, 2, _cy);
            _camMatrix.Set(2, 0, 0d);
            _camMatrix.Set(2, 1, 0d);
            _camMatrix.Set(2, 2, 1.0d);

            _distCoeffs = new Mat(4, 1, MatType.CV_64FC1);
            _distCoeffs.SetTo(Scalar.All(0));

            _rvec = new Mat(3, 1, MatType.CV_64FC1);
            _tvec = new Mat(3, 1, MatType.CV_64FC1);
            _rotationMatrix = new Mat(3, 3, MatType.CV_64FC1);
            _pnpObjectPointMat = new Mat(3, 1, MatType.CV_64FC1);
            _pnpCamSpacePointMat = new Mat(3, 1, MatType.CV_64FC1);

            _invertYM = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(1, -1, 1));
            _invertZM = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(1, 1, -1));
        }

        private void UpdatePnP(HolisticLandmarkerResult result)
        {
            if (!ActivePoseLandmark || !ActivePoseWorldLandmark) return;

            _objectPointsList.Clear();
            _imagePointsList.Clear();

            foreach (int i in _pnpLandmarkIndices)
            {
                var p3d = result.poseWorldLandmarks.landmarks[i];
                var p2d = result.poseLandmarks.landmarks[i];

                _objectPointsList.Add(new Point3f(p3d.x, -p3d.y, p3d.z));
                _imagePointsList.Add(new Point2f(p2d.x * ImageSource.Resolution.x, p2d.y * ImageSource.Resolution.y));
            }

            Cv2.SolvePnPRansac(
                InputArray.Create(_objectPointsList),
                InputArray.Create(_imagePointsList),
                _camMatrix,
                _distCoeffs,
                _rvec,
                _tvec,
                true,
                100,
                8f,
                0.99,
                null,
                SolvePnPFlags.Iterative);

            Cv2.Rodrigues(_rvec, _rotationMatrix);

            for (int i = 0; i < PoseLandmarkCount; i++)
            {
                var p3d = result.poseWorldLandmarks.landmarks[i];
                var p2d = result.poseLandmarks.landmarks[i];

                _pnpObjectPointMat.Set(0, 0, (double)p3d.x);
                _pnpObjectPointMat.Set(1, 0, (double)-p3d.y);
                _pnpObjectPointMat.Set(2, 0, (double)p3d.z);

                Cv2.Gemm(_rotationMatrix, _pnpObjectPointMat, 1.0, _tvec, 1.0, _pnpCamSpacePointMat, GemmFlags.None);

                var z = (float)_pnpCamSpacePointMat.At<double>(2, 0);
                var x = (float)(((p2d.x * ImageSource.Resolution.x) - _cx) * z / _fx);
                var y = (float)(((p2d.y * ImageSource.Resolution.y) - _cy) * z / _fy);

                _positions[i] = new Vector3(x, y, 0);
            }

            var hipCenterPos = (_positions[(int)PoseLandmark.LeftHip] + _positions[(int)PoseLandmark.RightHip]) / 2f;

            GlobalAvatarPosition = Vector3.Scale(hipCenterPos, MovementScale);

            if (_enableGlobalPoseFilter)
            {
                GlobalAvatarPosition = _globalAvatarPositionOneEuroFilter.Filter(GlobalAvatarPosition, (float)_stopwatch.Elapsed.TotalSeconds);
            }
        }

        private void ResultCallback(in HolisticLandmarkerResult holisticLandmarkerResult, Image image, long timestampMillisec)
        {
            ActiveFaceLandmark = Set(FaceLandmarks, holisticLandmarkerResult.faceLandmarks.landmarks);
            ActivePoseLandmark = Set(PoseLandmarks, holisticLandmarkerResult.poseLandmarks.landmarks);
            ActivePoseWorldLandmark = Set(PoseWorldLandmarks, holisticLandmarkerResult.poseWorldLandmarks.landmarks);
            ActiveLeftHandLandmark = Set(LeftHandLandmarks, holisticLandmarkerResult.leftHandLandmarks.landmarks);
            ActiveLeftHandWorldLandmark = Set(LeftHandWorldLandmarks, holisticLandmarkerResult.leftHandWorldLandmarks.landmarks);
            ActiveRightHandLandmark = Set(RightHandLandmarks, holisticLandmarkerResult.rightHandLandmarks.landmarks);
            ActiveRightHandWorldLandmark = Set(RightHandWorldLandmarks, holisticLandmarkerResult.rightHandWorldLandmarks.landmarks);
            ActiveFaceBlendShapes = holisticLandmarkerResult.faceBlendshapes.categories != null && holisticLandmarkerResult.faceBlendshapes.categories.Count > 0;

            UpdatePnP(holisticLandmarkerResult);

            if (ActiveFaceBlendShapes)
            {
                for (int i = 0; i < FaceBlendShapes.Length; i++)
                {
                    FaceBlendShapes[i] = holisticLandmarkerResult.faceBlendshapes.categories[i].score;
                }
            }

            foreach (var landmark in PoseWorldLandmarks)
            {
                if (_enableKalmanFilter)
                {
                    landmark.Position = landmark.KalmanFilter.Update(landmark.Position);
                }

                if (_enableOneEuroFilter)
                {
                    landmark.Position = landmark.OneEuroFilter.Filter(landmark.Position, (float)_stopwatch.Elapsed.TotalSeconds);
                }
            }


            var currentTick = _stopwatch.ElapsedTicks;

            if (_lastFrameTick != 0)
            {
                var elapsedSeconds = (float)(currentTick - _lastFrameTick) / Stopwatch.Frequency;

                if (elapsedSeconds > 0)
                {
                    _trackingFPS = 1f / elapsedSeconds;
                }
            }

            _lastFrameTick = currentTick;
        }

        private bool Set(Landmark[] landmarks, List<Mediapipe.Tasks.Components.Containers.NormalizedLandmark> normalizedLandmarks)
        {
            if (normalizedLandmarks == null || normalizedLandmarks.Count == 0) return false;

            for (int i = 0; i < normalizedLandmarks.Count; i++)
            {
                landmarks[i].Set(normalizedLandmarks[i]);

                landmarks[i].Position = Vector3.Scale(landmarks[i].Position, LandmarkScale);
            }

            return true;
        }

        private bool Set(Landmark[] landmarks, List<Mediapipe.Tasks.Components.Containers.Landmark> normalizedLandmarks)
        {
            if (normalizedLandmarks == null || normalizedLandmarks.Count == 0) return false;

            for (int i = 0; i < normalizedLandmarks.Count; i++)
            {
                landmarks[i].Set(normalizedLandmarks[i]);

                landmarks[i].Position = Vector3.Scale(landmarks[i].Position, LandmarkScale);
            }

            return normalizedLandmarks != null;
        }
    }
}