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
using System.Collections.Generic;
using System.Linq;
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

        public ImageSource ImageSource;
        private BaseOptions.Delegate _delegate;
        public TextAsset ModelAsset;
        public ImageReadMode ImageReadMode = ImageReadMode.CPUAsync;

        [Range(0f, 1f)] public float MinFaceDetectionConfidence = 0.5f;
        [Range(0f, 1f)] public float MinFaceSuppressionThreshold = 0.5f;
        [Range(0f, 1f)] public float MinFaceLandmarksConfidence = 0.5f;
        [Range(0f, 1f)] public float MinPoseDetectionConfidence = 0.5f;
        [Range(0f, 1f)] public float MinPoseSuppressionThreshold = 0.5f;
        [Range(0f, 1f)] public float MinPoseLandmarksConfidence = 0.5f;
        [Range(0f, 1f)] public float MinHandLandmarksConfidence = 0.5f;
        public bool OutputFaceBlendshapes = true;
        public bool OutputSegmentationMask = false;

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

        //public Vector3 LandmarkScale = Vector3.one;

        private HolisticLandmarker _holisticLandmarker;
        private TextureFramePool _textureFramePool;
        private Stopwatch _stopwatch;

        private double _fx, _fy, _cx, _cy;

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

        private bool _initialized = false;

        [HideInInspector] private int _resultCallbackCount = 0;
        [HideInInspector] private float _fpsTimer = 0f;

        private OneEuroFilter<Vector3> _globalAvatarPositionOneEuroFilter;
        private OneEuroFilter<Quaternion> _globalAvatarRotationOneEuroFilter;


        private readonly KalmanFilter[] _kalmanFilters = new KalmanFilter[PoseLandmarkCount];
        private readonly OneEuroFilter<Vector3>[] _oneEuroFilters = new OneEuroFilter<Vector3>[PoseLandmarkCount];

        private readonly List<Point3> _objectPointsList = new();
        private readonly List<Point> _imagePointsList = new();

        private readonly int[] _pnpLandmarkIndices = new int[]
        {
            (int)PoseLandmark.Nose,
            (int)PoseLandmark.LeftEye, (int)PoseLandmark.RightEye,
            (int)PoseLandmark.LeftEar, (int)PoseLandmark.RightEar,
            (int)PoseLandmark.LeftShoulder, (int)PoseLandmark.RightShoulder,
            (int)PoseLandmark.LeftHip, (int)PoseLandmark.RightHip,
        };


        public const string ModelAssetPath = "holistic_landmarker.bytes";

        public const int PoseLandmarkCount = 33;
        public const int HandLandmarkCount = 21;
        public const int FaceLandmarkCount = 478;
        public const int FaceBlendShapeCount = 52;


        // Properties

        public Vector3 GlobalAvatarPosition { get; private set; } = Vector3.zero;

        public Quaternion GlobalAvatarRotation { get; private set; } = Quaternion.identity;

        public int MediaPipeFPS { get; private set; }

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

        public Vector3[] LocalAvatarSpacePoints { get; private set; } = new Vector3[PoseLandmarkCount];


        // Events

        public event Action<HolisticLandmarkerResult> OnCallback;


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
                    landmark.OneEuroFilter = new OneEuroFilter<Vector3>(_filterFrequency, _filterMinCutoff, _filterBeta, _filterDcutoff);
                }
            }

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

            _globalAvatarPositionOneEuroFilter = new OneEuroFilter<Vector3>(_globalPoseFilterFrequency, _globalPoseFilterMinCutoff, _globalPoseFilterBeta, _globalPoseFilterDcutoff);
            _globalAvatarRotationOneEuroFilter = new OneEuroFilter<Quaternion>(_globalPoseFilterFrequency, _globalPoseFilterMinCutoff, _globalPoseFilterBeta, _globalPoseFilterDcutoff);


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

            while (true)
            {
                if (_textureFramePool.TryGetTextureFrame(out var textureFrame))
                {
                    Image image = default;

                    switch (ImageReadMode)
                    {
                        case ImageReadMode.CPU:
                            {
                                yield return waitForEndOfFrame;

                                textureFrame.ReadTextureOnCPU(ImageSource.Texture);
                                image = textureFrame.BuildCPUImage();
                                textureFrame.Release();

                                break;
                            }
                        case ImageReadMode.CPUAsync:
                            {
                                req = textureFrame.ReadTextureAsync(ImageSource.Texture);

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

                                textureFrame.ReadTextureOnGPU(ImageSource.Texture);
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

        private void Update()
        {
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

        private void InitializePnP()
        {
            var max_d = Mathf.Max(ImageSource.Resolution.x, ImageSource.Resolution.y);
            _fx = max_d;
            _fy = max_d;
            _cx = ImageSource.Resolution.x / 2f;
            _cy = ImageSource.Resolution.y / 2f;

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
            _pnpObjectPointMat = new Mat(3, 1, CvType.CV_64FC1);
            _pnpCamSpacePointMat = new Mat(3, 1, CvType.CV_64FC1);

            _invertYM = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(1, -1, 1));
            _invertZM = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(1, 1, -1));
            _transformationM = new Matrix4x4();
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

            UpdatePnP(holisticLandmarkerResult);

            Interlocked.Increment(ref _resultCallbackCount);

            OnCallback?.Invoke(holisticLandmarkerResult);
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

                _objectPointsList.Add(new Point3(p3d.x, -p3d.y, p3d.z));
                _imagePointsList.Add(new Point(p2d.x * ImageSource.Resolution.x, p2d.y * ImageSource.Resolution.y));
            }

            _objectPoints.fromList(_objectPointsList);
            _imagePoints.fromList(_imagePointsList);

            Calib3d.solvePnP(_objectPoints, _imagePoints, _camMatrix, _distCoeffs, _rvec, _tvec, true, Calib3d.SOLVEPNP_SQPNP);
            Calib3d.Rodrigues(_rvec, _rotationMatrix);

            _tvec.get(0, 0, new double[3]);

            var adjustedCameraSpacePoints = new Vector3[PoseLandmarkCount];

            for (int i = 0; i < PoseLandmarkCount; i++)
            {
                var p3d = result.poseWorldLandmarks.landmarks[i];
                var p2d = result.poseLandmarks.landmarks[i];

                _pnpObjectPointMat.put(0, 0, p3d.x);
                _pnpObjectPointMat.put(1, 0, -p3d.y);
                _pnpObjectPointMat.put(2, 0, p3d.z);

                Core.gemm(_rotationMatrix, _pnpObjectPointMat, 1.0, _tvec, 1.0, _pnpCamSpacePointMat);

                var z = _pnpCamSpacePointMat.get(2, 0)[0];
                var x = (float)(((p2d.x * ImageSource.Resolution.x) - _cx) * z / _fx);
                var y = (float)(((p2d.y * ImageSource.Resolution.y) - _cy) * z / _fy);

                adjustedCameraSpacePoints[i] = new Vector3(x, y, (float)z);
            }

            var hipCenterPos = (adjustedCameraSpacePoints[(int)PoseLandmark.LeftHip] + adjustedCameraSpacePoints[(int)PoseLandmark.RightHip]) / 2f;

            Calib3d.Rodrigues(_rvec, _rotationMatrix);

            var rot = OpenCVARUtils.ConvertRvecToRot(_rvec);

            _transformationM = Matrix4x4.TRS(hipCenterPos, rot, Vector3.one);

            var ARM = _invertYM * _transformationM * _invertYM * _invertYM * _invertZM;

            GlobalAvatarPosition = ARM.GetColumn(3);
            GlobalAvatarRotation = UnityUtils.LookRotation(ARM.GetColumn(2), ARM.GetColumn(1));

            if (_enableGlobalPoseFilter)
            {
                GlobalAvatarPosition = _globalAvatarPositionOneEuroFilter.Filter(GlobalAvatarPosition, (float)_stopwatch.Elapsed.TotalSeconds);
                GlobalAvatarRotation = _globalAvatarRotationOneEuroFilter.Filter(GlobalAvatarRotation, (float)_stopwatch.Elapsed.TotalSeconds);
            }

            var localRotInverse = Quaternion.Inverse(GlobalAvatarRotation);

            for (int i = 0; i < PoseLandmarkCount; i++)
            {
                LocalAvatarSpacePoints[i] = localRotInverse * (adjustedCameraSpacePoints[i] - hipCenterPos);
                //LocalAvatarSpacePoints[i] = Vector3.Scale(localRotInverse * (adjustedCameraSpacePoints[i] - hipCenterPos), LandmarkScale);

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

        private bool Set(Landmark[] landmarks, List<Mediapipe.Tasks.Components.Containers.NormalizedLandmark> normalizedLandmarks)
        {
            if (normalizedLandmarks == null || normalizedLandmarks.Count == 0) return false;

            for (int i = 0; i < normalizedLandmarks.Count; i++)
            {
                landmarks[i].Set(normalizedLandmarks[i]);

                //landmarks[i].Position = Vector3.Scale(landmarks[i].Position, LandmarkScale);
            }

            return true;
        }

        private bool Set(Landmark[] landmarks, List<Mediapipe.Tasks.Components.Containers.Landmark> normalizedLandmarks)
        {
            if (normalizedLandmarks == null || normalizedLandmarks.Count == 0) return false;

            for (int i = 0; i < normalizedLandmarks.Count; i++)
            {
                landmarks[i].Set(normalizedLandmarks[i]);

                //landmarks[i].Position = Vector3.Scale(landmarks[i].Position, LandmarkScale);
            }

            return normalizedLandmarks != null;
        }
    }
}