using Mediapipe;
using Mediapipe.Tasks.Core;
using Mediapipe.Tasks.Vision.HolisticLandmarker;
using Mediapipe.Unity;
using Mediapipe.Unity.Experimental;
using OpenCVForUnity.Calib3dModule;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.UnityIntegration;
using PMC.Utilities;
using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
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
        [SerializeField] private bool _useMainThread = true;
        [SerializeField] private Vector3 _landmarkScale = new(1f, 1f, 1f);
        [SerializeField] private bool _enableKalmanFilter = true;
        [SerializeField] private float _timeInterval = 0.45f;
        [SerializeField] private float _noise = 0.4f;
        [SerializeField] private bool _enableOneEuroFilter = true;
        [SerializeField] private float _filterFrequency = 120f;
        [SerializeField] private float _filterMinCutoff = 1f;
        [SerializeField] private float _filterBeta = 0.1f;
        [SerializeField] private float _filterDcutoff = 1f;
        [SerializeField] private bool _enableGlobalPoseFilter = true;
        [SerializeField] private float _globalPoseFilterFrequency = 120f;
        [SerializeField] private float _globalPoseFilterMinCutoff = 1f;
        [SerializeField] private float _globalPoseFilterBeta = 0.1f;
        [SerializeField] private float _globalPoseFilterDcutoff = 1f;
        [SerializeField, HideInInspector] private BaseOptions.Delegate _delegate;
        [SerializeField, HideInInspector] private RunningMode _runningMode = RunningMode.LIVE_STREAM;

        private HolisticLandmarker _holisticLandmarker;
        private TextureFramePool _textureFramePool;
        private Stopwatch _stopwatch;

        private int _resultCallbackCount = 0;
        private float _fpsTimer = 0f;
        private bool _initialized = false;

        private Mat _camMatrix;
        private MatOfDouble _distCoeffs;
        private MatOfPoint3f _objectPoints;
        private MatOfPoint2f _imagePoints;
        private Mat _rvec;
        private Mat _tvec;
        private Mat _rotationMatrix;
        private Mat _pnpObjectPointMat;
        private Mat _pnpCamSpacePointMat;

        private Matrix4x4 _invertYM;
        private Matrix4x4 _invertZM;
        private Matrix4x4 _transformationM;
        private Matrix4x4 _ARM;

        private double _fx, _fy, _cx, _cy;
        private double[] _tvecArr;

        private readonly KalmanFilter[] _kalmanFilters = new KalmanFilter[PoseLandmarkCount];
        private OneEuroFilter<Vector3> _globalAvatarPositionOneEuroFilter;
        private OneEuroFilter<Quaternion> _globalAvatarRotationOneEuroFilter;
        private readonly OneEuroFilter<Vector3>[] _oneEuroFilters = new OneEuroFilter<Vector3>[PoseLandmarkCount];

        private readonly List<Point3> _objectPointsList = new();
        private readonly List<Point> _imagePointsList = new();

        private readonly Vector3[] _adjustedCameraSpacePoints = new Vector3[PoseLandmarkCount];

        private readonly int[] _pnpLandmarkIndices = new int[]
        {
            (int)PoseLandmark.Nose,
            (int)PoseLandmark.LeftEye, (int)PoseLandmark.RightEye,
            (int)PoseLandmark.LeftEar, (int)PoseLandmark.RightEar,
            (int)PoseLandmark.LeftShoulder, (int)PoseLandmark.RightShoulder,
            (int)PoseLandmark.LeftHip, (int)PoseLandmark.RightHip,
        };


        public const int PoseLandmarkCount = 33;
        public const int HandLandmarkCount = 21;
        public const int FaceLandmarkCount = 478;


        // Properties

        public Landmark[] FaceLandmarks { get; private set; } = new Landmark[FaceLandmarkCount];

        public Landmark[] PoseLandmarks { get; private set; } = new Landmark[PoseLandmarkCount];

        public Landmark[] PoseWorldLandmarks { get; private set; } = new Landmark[PoseLandmarkCount];

        public Landmark[] LeftHandLandmarks { get; private set; } = new Landmark[HandLandmarkCount];

        public Landmark[] LeftHandWorldLandmarks { get; private set; } = new Landmark[HandLandmarkCount];

        public Landmark[] RightHandLandmarks { get; private set; } = new Landmark[HandLandmarkCount];

        public Landmark[] RightHandWorldLandmarks { get; private set; } = new Landmark[HandLandmarkCount];

        public Vector3[] LocalAvatarSpacePoints { get; private set; } = new Vector3[PoseLandmarkCount];


        public bool ActiveFaceLandmark { get; private set; }

        public bool ActivePoseLandmark { get; private set; }

        public bool ActivePoseWorldLandmark { get; private set; }

        public bool ActiveLeftHandLandmark { get; private set; }

        public bool ActiveLeftHandWorldLandmark { get; private set; }

        public bool ActiveRightHandLandmark { get; private set; }

        public bool ActiveRightHandWorldLandmark { get; private set; }


        public Vector3 GlobalAvatarPosition { get; private set; } = Vector3.zero;

        public Quaternion GlobalAvatarRotation { get; private set; } = Quaternion.identity;

        public ConcurrentQueue<HolisticLandmarkerResult> ResultQueue { get; private set; } = new();

        public int MediaPipeFPS { get; private set; }


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

            InitializeLandmark(FaceLandmarks);
            InitializeLandmark(PoseLandmarks);
            InitializeLandmark(PoseWorldLandmarks);
            InitializeLandmark(LeftHandLandmarks);
            InitializeLandmark(LeftHandWorldLandmarks);
            InitializeLandmark(RightHandLandmarks);
            InitializeLandmark(RightHandWorldLandmarks);

            if (_enableKalmanFilter)
            {
                for (int i = 0; i < _kalmanFilters.Length; i++)
                {
                    _kalmanFilters[i] = new KalmanFilter();

                    _kalmanFilters[i].SetParameter(_timeInterval, _noise);
                    _kalmanFilters[i].Predict();
                }
            }

            if (_enableOneEuroFilter)
            {
                for (int i = 0; i < _kalmanFilters.Length; i++)
                {
                    _oneEuroFilters[i] = new OneEuroFilter<Vector3>(_filterFrequency, _filterMinCutoff, _filterBeta, _filterDcutoff);
                }
            }

            // Initialize global pose filters with separate parameters
            _globalAvatarPositionOneEuroFilter = new OneEuroFilter<Vector3>(_globalPoseFilterFrequency, _globalPoseFilterMinCutoff, _globalPoseFilterBeta, _globalPoseFilterDcutoff);
            _globalAvatarRotationOneEuroFilter = new OneEuroFilter<Quaternion>(_globalPoseFilterFrequency, _globalPoseFilterMinCutoff, _globalPoseFilterBeta, _globalPoseFilterDcutoff);

            _initialized = true;
        }

        private IEnumerator Start()
        {
            if (!_initialized) yield break;

            InitializePnP();

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
            if (_useMainThread)
            {
                if (ResultQueue.TryDequeue(out var result))
                {
                    Set(result);

                    UpdatePnP(result);

                    OnCallback?.Invoke(result);
                }
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

            _camMatrix?.Dispose();
            _distCoeffs?.Dispose();
            _objectPoints?.Dispose();
            _imagePoints?.Dispose();
            _rvec?.Dispose();
            _tvec?.Dispose();
            _rotationMatrix?.Dispose();
            _pnpObjectPointMat?.Dispose();
            _pnpCamSpacePointMat?.Dispose();
        }

        private void InitializeLandmark(Landmark[] landmarks)
        {
            for (int i = 0; i < landmarks.Length; i++)
            {
                landmarks[i] = new Landmark();

                if (_enableKalmanFilter)
                {
                    landmarks[i].KalmanFilter.SetParameter(_timeInterval, _noise);
                    landmarks[i].KalmanFilter.Predict();
                }

                if (_enableOneEuroFilter)
                {
                    landmarks[i].OneEuroFilter = new OneEuroFilter<Vector3>(_filterFrequency, _filterMinCutoff, _filterBeta, _filterDcutoff);
                }
            }
        }

        private void InitializePnP()
        {
            var max_d = Mathf.Max(_imageSource.Resolution.x, _imageSource.Resolution.y);
            _fx = max_d;
            _fy = max_d;
            _cx = _imageSource.Resolution.x / 2.0f;
            _cy = _imageSource.Resolution.y / 2.0f;

            _camMatrix = new Mat(3, 3, CvType.CV_64FC1);
            _camMatrix.put(0, 0, _fx);
            _camMatrix.put(0, 1, 0);
            _camMatrix.put(0, 2, _cx);
            _camMatrix.put(1, 0, 0);
            _camMatrix.put(1, 1, _fy);
            _camMatrix.put(1, 2, _cy);
            _camMatrix.put(2, 0, 0);
            _camMatrix.put(2, 1, 0);
            _camMatrix.put(2, 2, 1.0f);

            _distCoeffs = new MatOfDouble(0, 0, 0, 0);
            _objectPoints = new MatOfPoint3f();
            _imagePoints = new MatOfPoint2f();
            _rvec = new Mat(3, 1, CvType.CV_64FC1);
            _tvec = new Mat(3, 1, CvType.CV_64FC1);
            _rotationMatrix = new Mat(3, 3, CvType.CV_64FC1);

            _invertYM = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(1, -1, 1));
            _invertZM = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(1, 1, -1));
            _transformationM = new Matrix4x4();

            _pnpObjectPointMat = new Mat(3, 1, CvType.CV_64FC1);
            _pnpCamSpacePointMat = new Mat(3, 1, CvType.CV_64FC1);
            _tvecArr = new double[3];
        }

        private void UpdatePnP(HolisticLandmarkerResult result)
        {
            if (result.poseLandmarks.landmarks == null || result.poseLandmarks.landmarks.Count == 0 ||
                result.poseWorldLandmarks.landmarks == null || result.poseWorldLandmarks.landmarks.Count == 0) return;

            _objectPointsList.Clear();
            _imagePointsList.Clear();

            foreach (int i in _pnpLandmarkIndices)
            {
                var p3d = result.poseWorldLandmarks.landmarks[i];
                _objectPointsList.Add(new Point3(p3d.x, -p3d.y, p3d.z));

                var p2d = result.poseLandmarks.landmarks[i];
                _imagePointsList.Add(new Point(p2d.x * _imageSource.Resolution.x, p2d.y * _imageSource.Resolution.y));
            }

            _objectPoints.fromList(_objectPointsList);
            _imagePoints.fromList(_imagePointsList);

            Calib3d.solvePnP(_objectPoints, _imagePoints, _camMatrix, _distCoeffs, _rvec, _tvec, true, Calib3d.SOLVEPNP_SQPNP);
            Calib3d.Rodrigues(_rvec, _rotationMatrix);

            _tvec.get(0, 0, _tvecArr);

            for (int i = 0; i < PoseLandmarkCount; i++)
            {
                var p3d = result.poseWorldLandmarks.landmarks[i];
                _pnpObjectPointMat.put(0, 0, p3d.x);
                _pnpObjectPointMat.put(1, 0, -p3d.y);
                _pnpObjectPointMat.put(2, 0, p3d.z);

                Core.gemm(_rotationMatrix, _pnpObjectPointMat, 1.0, _tvec, 1.0, _pnpCamSpacePointMat);

                var z = _pnpCamSpacePointMat.get(2, 0)[0];

                var p2d = result.poseLandmarks.landmarks[i];
                var x_pixel = p2d.x * _imageSource.Resolution.x;
                var y_pixel = p2d.y * _imageSource.Resolution.y;

                var x = (float)((x_pixel - _cx) * z / _fx);
                var y = (float)((y_pixel - _cy) * z / _fy);

                _adjustedCameraSpacePoints[i] = new Vector3(x, y, (float)z);
            }

            var hipCenterPos = (_adjustedCameraSpacePoints[(int)PoseLandmark.LeftHip] + _adjustedCameraSpacePoints[(int)PoseLandmark.RightHip]) / 2f;

            Calib3d.Rodrigues(_rvec, _rotationMatrix);

            var rot = OpenCVARUtils.ConvertRvecToRot(_rvec);

            _transformationM = Matrix4x4.TRS(hipCenterPos, rot, Vector3.one);
            _ARM = _invertYM * _transformationM * _invertYM;
            _ARM = _ARM * _invertYM * _invertZM;

            GlobalAvatarPosition = _ARM.GetColumn(3);
            GlobalAvatarRotation = UnityUtils.LookRotation(_ARM.GetColumn(2), _ARM.GetColumn(1));

            if (_enableGlobalPoseFilter)
            {
                GlobalAvatarPosition = _globalAvatarPositionOneEuroFilter.Filter(GlobalAvatarPosition, (float)_stopwatch.Elapsed.TotalSeconds);
                GlobalAvatarRotation = _globalAvatarRotationOneEuroFilter.Filter(GlobalAvatarRotation, (float)_stopwatch.Elapsed.TotalSeconds);
            }

            var localRotInverse = Quaternion.Inverse(GlobalAvatarRotation);

            for (int i = 0; i < PoseLandmarkCount; i++)
            {
                LocalAvatarSpacePoints[i] = Vector3.Scale(localRotInverse * (_adjustedCameraSpacePoints[i] - hipCenterPos), _landmarkScale);

                if (_enableKalmanFilter)
                {
                    LocalAvatarSpacePoints[i] = _kalmanFilters[i].Update(LocalAvatarSpacePoints[i]);
                }

                if (_enableOneEuroFilter)
                {
                    LocalAvatarSpacePoints[i] = _oneEuroFilters[i].Filter(LocalAvatarSpacePoints[i], (float)_stopwatch.Elapsed.TotalSeconds);
                }
            }
        }

        private void ResultCallback(in HolisticLandmarkerResult holisticLandmarkerResult, Image image, long timestampMillisec)
        {
            if (_useMainThread)
            {
                ResultQueue.Enqueue(holisticLandmarkerResult);
            }
            else
            {
                Set(holisticLandmarkerResult);

                OnCallback?.Invoke(holisticLandmarkerResult);
            }


            Interlocked.Increment(ref _resultCallbackCount);
        }

        private void Set(HolisticLandmarkerResult result)
        {
            ActiveFaceLandmark = Set(FaceLandmarks, result.faceLandmarks.landmarks);
            ActivePoseLandmark = Set(PoseLandmarks, result.poseLandmarks.landmarks);
            ActivePoseWorldLandmark = Set(PoseWorldLandmarks, result.poseWorldLandmarks.landmarks);
            ActiveLeftHandLandmark = Set(LeftHandLandmarks, result.leftHandLandmarks.landmarks);
            ActiveLeftHandWorldLandmark = Set(LeftHandWorldLandmarks, result.leftHandWorldLandmarks.landmarks);
            ActiveRightHandLandmark = Set(RightHandLandmarks, result.rightHandLandmarks.landmarks);
            ActiveRightHandWorldLandmark = Set(RightHandWorldLandmarks, result.rightHandWorldLandmarks.landmarks);
        }

        private bool Set(Landmark[] landmarks, List<Mediapipe.Tasks.Components.Containers.NormalizedLandmark> normalizedLandmarks)
        {
            if (normalizedLandmarks == null || normalizedLandmarks.Count == 0) return false;

            for (int i = 0; i < normalizedLandmarks.Count; i++)
            {
                landmarks[i].Set(normalizedLandmarks[i]);

                landmarks[i].Position = Vector3.Scale(landmarks[i].Position, _landmarkScale);

                if (_enableKalmanFilter)
                {
                    landmarks[i].Position = landmarks[i].KalmanFilter.Update(landmarks[i].Position);
                }

                if (_enableOneEuroFilter)
                {
                    landmarks[i].Position = landmarks[i].OneEuroFilter.Filter(landmarks[i].Position, (float)_stopwatch.Elapsed.TotalSeconds);
                }
            }

            return true;
        }

        private bool Set(Landmark[] landmarks, List<Mediapipe.Tasks.Components.Containers.Landmark> normalizedLandmarks)
        {
            if (normalizedLandmarks == null || normalizedLandmarks.Count == 0) return false;

            for (int i = 0; i < normalizedLandmarks.Count; i++)
            {
                landmarks[i].Set(normalizedLandmarks[i]);

                landmarks[i].Position = Vector3.Scale(landmarks[i].Position, _landmarkScale);

                if (_enableKalmanFilter)
                {
                    landmarks[i].Position = landmarks[i].KalmanFilter.Update(landmarks[i].Position);
                }

                if (_enableOneEuroFilter)
                {
                    landmarks[i].Position = landmarks[i].OneEuroFilter.Filter(landmarks[i].Position, (float)_stopwatch.Elapsed.TotalSeconds);
                }
            }

            return normalizedLandmarks != null;
        }
    }

    public enum ImageReadMode
    {
        CPU,
        CPUAsync,
        GPU,
    }
}