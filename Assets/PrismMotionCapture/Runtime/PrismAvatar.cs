using Mediapipe.Tasks.Vision.HolisticLandmarker;
using RootMotion;
using RootMotion.FinalIK;
using System.Collections.Generic;
using UnityEngine;
using Stopwatch = System.Diagnostics.Stopwatch;

namespace PMC
{
    [AddComponentMenu("Prism Motion Capture/Prism Avatar")]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Animator))]
    public sealed class PrismAvatar : MonoBehaviour
    {
        // Fields

        [SerializeField] private PrismTracker _tracker;
        [SerializeField] private IKType _IKType = IKType.VRIK;
        [SerializeField] private bool _enableTwistRelaxer = true;
        [SerializeField] private bool _enableMovement = false;
        [SerializeField] private bool _autoWeight = true;
        [SerializeField] private float _fingerResetSpeed = 5f;
        [SerializeField] private float _weightSmoothingSpeed = 10f;
        [SerializeField] private float _targetRotationSmoothSpeed = 20.0f;
        [SerializeField] private Vector3 _landmarkScale = new(1f, 1f, -1f);
        [SerializeField] private Vector3 _handRotationOffset = new(0f, 90f, 0f);
        [SerializeField] private bool _enableKalmanFilter = true;
        [SerializeField] private float _timeInterval = 0.45f;
        [SerializeField] private float _noise = 0.4f;
        [SerializeField] private bool _enableOneEuroFilter = true;
        [SerializeField] private float _filterFrequency = 120.0f;
        [SerializeField] private float _filterMinCutoff = 1.0f;
        [SerializeField] private float _filterBeta = 0.1f;
        [SerializeField] private float _filterDcutoff = 1.0f;

        private Animator _animator;
        private VRIK _VRIK;
        private FullBodyBipedIK _FBBIK;
        private FBBIKHeadEffector _headEffector;
        private TwistRelaxer _twistRelaxer;

        private Transform _root;
        private Transform _headTarget;
        private Transform _pelvisTarget;
        private Transform _leftHandTarget;
        private Transform _rightHandTarget;
        private Transform _leftArmBendGoal;
        private Transform _rightArmBendGoal;
        private Transform _leftShoulderTarget;
        private Transform _rightShoulderTarget;
        private Transform _leftFootTarget;
        private Transform _rightFootTarget;
        private Transform _leftLegBendGoal;
        private Transform _rightLegBendGoal;
        private Transform _leftThighTarget;
        private Transform _rightThighTarget;

        private Vector3 _baseScale;
        private Vector3 _basePosition;
        private Vector3 _baseTrackingPosition;
        private Vector3 _pelvisBasePosition;
        private Quaternion _inverseLeftHandRotation;
        private Quaternion _inverseRightHandRotation;

        private float _currentLeftLegWeight = 0f;
        private float _currentRightLegWeight = 0f;
        private float _currentLeftBendWeight = 0f;
        private float _currentRightBendWeight = 0f;

        private float _sittingHeight;
        private bool _activeFaceLandmark;
        private bool _activePoseLandmark;
        private bool _activePoseWorldLandmark;
        private bool _activeLeftHandLandmark;
        private bool _activeLeftHandWorldLandmark;
        private bool _activeRightHandLandmark;
        private bool _activeRightHandWorldLandmark;

        private Stopwatch _stopwatch;

        private readonly Landmark[] _faceLandmarks = new Landmark[FaceLandmarkCount];
        private readonly Landmark[] _poseLandmarks = new Landmark[PoseLandmarkCount];
        private readonly Landmark[] _poseWorldLandmarks = new Landmark[PoseLandmarkCount];
        private readonly Landmark[] _leftHandLandmarks = new Landmark[HandLandmarkCount];
        private readonly Landmark[] _leftHandWorldLandmarks = new Landmark[HandLandmarkCount];
        private readonly Landmark[] _rightHandLandmarks = new Landmark[HandLandmarkCount];
        private readonly Landmark[] _rightHandWorldLandmarks = new Landmark[HandLandmarkCount];

        private readonly Dictionary<HumanBodyBones, Quaternion> _initialLocalRotations = new();
        private readonly Dictionary<HumanBodyBones, Vector3> _initialBoneDirections = new();

        public const int PoseLandmarkCount = 33;
        public const int HandLandmarkCount = 21;
        public const int FaceLandmarkCount = 478;


        // Methods

        [ContextMenu("Execute Calibration (Play Mode Only)")]
        public void ExecuteCalibration()
        {
            if (!Application.isPlaying) return;

            var distance = Vector3.Distance(Vector3.zero, _poseWorldLandmarks[(int)PoseLandmark.Nose].Position);

            _landmarkScale *= _sittingHeight / distance;
        }

        [ContextMenu("Reset Position (Play Mode Only)")]
        public void ResetPosition()
        {
            if (!Application.isPlaying) return;

            _baseTrackingPosition = Vector3.zero;
        }


        private void Awake()
        {
            if (!enabled) return;

            _animator = gameObject.GetComponent<Animator>();

            if (_IKType == IKType.VRIK)
            {
                if (!gameObject.TryGetComponent(out _VRIK))
                {
                    _VRIK = gameObject.AddComponent<VRIK>();
                }
            }

            if (_IKType == IKType.FBBIK)
            {
                if (!gameObject.TryGetComponent(out _FBBIK))
                {
                    _FBBIK = gameObject.AddComponent<FullBodyBipedIK>();
                }
            }

            if (_enableTwistRelaxer)
            {
                if (!gameObject.TryGetComponent(out _twistRelaxer))
                {
                    _twistRelaxer = gameObject.AddComponent<TwistRelaxer>();
                }
            }

            if (_tracker != null)
            {
                _tracker.OnCallback += OnCallback;
            }

            if (_IKType == IKType.VRIK)
            {
                _VRIK.solver.OnPreUpdate += OnPreVRIK;
            }

            if (_IKType == IKType.FBBIK)
            {
                _FBBIK.solver.OnPreUpdate += OnPreFBBIK;
            }

            InitializeLandmark(_faceLandmarks);
            InitializeLandmark(_poseLandmarks);
            InitializeLandmark(_poseWorldLandmarks);
            InitializeLandmark(_leftHandLandmarks);
            InitializeLandmark(_leftHandWorldLandmarks);
            InitializeLandmark(_rightHandLandmarks);
            InitializeLandmark(_rightHandWorldLandmarks);
        }

        private void Start()
        {
            if (!enabled) return;

            _stopwatch = Stopwatch.StartNew();

            CreateTarget();

            InitializeFinger();

            if (_IKType == IKType.VRIK)
            {
                InitializeVRIK();
            }

            if (_IKType == IKType.FBBIK)
            {
                InitializeFBBIK();
            }
        }

        private void OnDestroy()
        {
            if (!enabled) return;

            if (_tracker != null)
            {
                _tracker.OnCallback -= OnCallback;
            }

            if (_IKType == IKType.VRIK)
            {
                _VRIK.solver.OnPreUpdate -= OnPreVRIK;
            }

            if (_IKType == IKType.FBBIK)
            {
                _FBBIK.solver.OnPreUpdate -= OnPreFBBIK;
            }
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

        private void OnPreVRIK()
        {
            UpdateTarget();
            UpdateVRIK();
            UpdateFinger();
        }

        private void OnPreFBBIK()
        {
            _root.transform.localRotation = _FBBIK.references.root.rotation;

            UpdateTarget();
            UpdateFBBIK();

            if (_enableMovement && _activePoseLandmark)
            {
                var c_Hip = (_poseLandmarks[(int)PoseLandmark.LeftHip].Position + _poseLandmarks[(int)PoseLandmark.RightHip].Position) / 2;

                if (_baseTrackingPosition == Vector3.zero)
                {
                    _baseTrackingPosition = c_Hip;
                }

                _FBBIK.references.root.position = c_Hip - _baseTrackingPosition + _basePosition;

                _root.transform.localPosition = _FBBIK.references.root.position + _pelvisBasePosition;

                //_FBBIK.references.root.position = _root.transform.localPosition - _pelvisBasePosition;
            }

            UpdateFinger();
        }

        private void InitializeVRIK()
        {
            _VRIK.AutoDetectReferences();

            _VRIK.solver.FixTransforms();

            _VRIK.solver.IKPositionWeight = 0f;

            _basePosition = _VRIK.references.root.position;
            _baseScale = _VRIK.references.root.localScale;
            _pelvisBasePosition = _VRIK.references.pelvis.localPosition;
            _sittingHeight = Vector3.Distance(_VRIK.references.head.position, _VRIK.references.pelvis.position);

            _root.position = _VRIK.references.pelvis.position;
            _headTarget.position = _VRIK.references.head.position;
            _pelvisTarget.position = _VRIK.references.pelvis.position;
            _leftHandTarget.position = _VRIK.references.leftHand.position;
            _rightHandTarget.position = _VRIK.references.rightHand.position;
            _leftArmBendGoal.position = _VRIK.references.leftForearm.position;
            _rightArmBendGoal.position = _VRIK.references.rightForearm.position;
            _leftFootTarget.position = _VRIK.references.leftFoot.position;
            _rightFootTarget.position = _VRIK.references.rightFoot.position;
            _leftLegBendGoal.position = _VRIK.references.leftCalf.position;
            _rightLegBendGoal.position = _VRIK.references.rightCalf.position;

            _VRIK.solver.spine.minHeadHeight = 0f;
            _VRIK.solver.spine.bodyPosStiffness = 0.55f;
            _VRIK.solver.spine.bodyRotStiffness = 0.1f;
            _VRIK.solver.spine.neckStiffness = 0.2f;
            _VRIK.solver.spine.moveBodyBackWhenCrouching = 0.5f;
            _VRIK.solver.spine.headClampWeight = 0f;
            _VRIK.solver.spine.maxRootAngle = 20f;
            //_VRIK.solver.spine.maintainPelvisPosition = 1f;
            _VRIK.solver.plantFeet = true;

            _VRIK.solver.spine.headTarget = _headTarget;
            _VRIK.solver.spine.pelvisTarget = _pelvisTarget;
            _VRIK.solver.leftArm.target = _leftHandTarget;
            _VRIK.solver.rightArm.target = _rightHandTarget;
            _VRIK.solver.leftArm.bendGoal = _leftArmBendGoal;
            _VRIK.solver.rightArm.bendGoal = _rightArmBendGoal;
            _VRIK.solver.leftLeg.target = _leftFootTarget;
            _VRIK.solver.rightLeg.target = _rightFootTarget;
            _VRIK.solver.leftLeg.bendGoal = _leftLegBendGoal;
            _VRIK.solver.rightLeg.bendGoal = _rightLegBendGoal;

            _VRIK.solver.spine.positionWeight = 0.5f;
            _VRIK.solver.spine.rotationWeight = 1f;
            _VRIK.solver.spine.pelvisPositionWeight = 0f;
            _VRIK.solver.spine.pelvisRotationWeight = 0.7f;
            _VRIK.solver.leftArm.positionWeight = 1f;
            _VRIK.solver.leftArm.rotationWeight = 1f;
            _VRIK.solver.leftArm.bendGoalWeight = 1f;
            _VRIK.solver.rightArm.positionWeight = 1f;
            _VRIK.solver.rightArm.rotationWeight = 1f;
            _VRIK.solver.rightArm.bendGoalWeight = 1f;
            _VRIK.solver.leftLeg.positionWeight = 1f;
            _VRIK.solver.leftLeg.rotationWeight = 1f;
            _VRIK.solver.leftLeg.bendGoalWeight = 1f;
            _VRIK.solver.rightLeg.positionWeight = 1f;
            _VRIK.solver.rightLeg.rotationWeight = 1f;
            _VRIK.solver.rightLeg.bendGoalWeight = 1f;

            //_VRIK.solver.leftArm.shoulderRotationMode = IKSolverVR.Arm.ShoulderRotationMode.FromTo;
            _VRIK.solver.leftArm.shoulderRotationWeight = 0.3f;
            _VRIK.solver.leftArm.shoulderTwistWeight = 0.7f;

            //_VRIK.solver.rightArm.shoulderRotationMode = IKSolverVR.Arm.ShoulderRotationMode.FromTo;
            _VRIK.solver.rightArm.shoulderRotationWeight = 0.3f;
            _VRIK.solver.rightArm.shoulderTwistWeight = 0.7f;

            _VRIK.solver.locomotion.footDistance = 0.1f;
            _VRIK.solver.locomotion.stepThreshold = 0.2f;
            _VRIK.solver.locomotion.angleThreshold = 60f;
            _VRIK.solver.locomotion.maxVelocity = 0.4f;
            _VRIK.solver.locomotion.velocityFactor = 0.4f;
            _VRIK.solver.locomotion.rootSpeed = 20f;
            _VRIK.solver.locomotion.stepSpeed = 3f;
            _VRIK.solver.locomotion.weight = 1.0f;

            if (_enableTwistRelaxer)
            {
                _twistRelaxer.ik = _VRIK;
                _twistRelaxer.twistSolvers = new TwistSolver[4];
                _twistRelaxer.twistSolvers[0] = new TwistSolver(_VRIK.references.leftUpperArm);
                _twistRelaxer.twistSolvers[1] = new TwistSolver(_VRIK.references.rightUpperArm);
                _twistRelaxer.twistSolvers[2] = new TwistSolver(_VRIK.references.leftForearm);
                _twistRelaxer.twistSolvers[3] = new TwistSolver(_VRIK.references.rightForearm);
                _twistRelaxer.twistSolvers[2].children = new Transform[1] { _VRIK.references.leftHand };
                _twistRelaxer.twistSolvers[3].children = new Transform[1] { _VRIK.references.rightHand };
                _twistRelaxer.twistSolvers[0].parentChildCrossfade = 0f;
                _twistRelaxer.twistSolvers[1].parentChildCrossfade = 0f;
                _twistRelaxer.twistSolvers[2].parentChildCrossfade = 0.5f;
                _twistRelaxer.twistSolvers[3].parentChildCrossfade = 0.5f;
            }

            _VRIK.solver.IKPositionWeight = 1f;
        }

        private void InitializeFBBIK()
        {
            var references = new BipedReferences();

            BipedReferences.AutoDetectReferences(ref references, transform, new BipedReferences.AutoDetectParams(true, true));

            _FBBIK.SetReferences(references, references.pelvis);

            _FBBIK.solver.FixTransforms();

            _FBBIK.solver.iterations = 4;
            _FBBIK.solver.IKPositionWeight = 0f;

            _basePosition = _FBBIK.references.root.position;
            _baseScale = _FBBIK.references.root.localScale;
            _pelvisBasePosition = _FBBIK.references.pelvis.localPosition;
            _sittingHeight = Vector3.Distance(_FBBIK.references.head.position, _FBBIK.references.pelvis.position);

            _root.position = _FBBIK.references.pelvis.position;
            _headTarget.position = _FBBIK.references.head.position;
            _pelvisTarget.position = _FBBIK.references.pelvis.position;
            _leftHandTarget.position = _FBBIK.references.leftHand.position;
            _rightHandTarget.position = _FBBIK.references.rightHand.position;
            _leftArmBendGoal.position = _FBBIK.references.leftForearm.position;
            _rightArmBendGoal.position = _FBBIK.references.rightForearm.position;
            _leftShoulderTarget.position = _FBBIK.references.leftUpperArm.position;
            _rightShoulderTarget.position = _FBBIK.references.rightUpperArm.position;
            _leftFootTarget.position = _FBBIK.references.leftFoot.position;
            _rightFootTarget.position = _FBBIK.references.rightFoot.position;
            _leftLegBendGoal.position = _FBBIK.references.leftCalf.position;
            _rightLegBendGoal.position = _FBBIK.references.rightCalf.position;
            _leftThighTarget.position = _FBBIK.references.leftThigh.position;
            _rightThighTarget.position = _FBBIK.references.rightThigh.position;

            _FBBIK.solver.bodyEffector.target = _pelvisTarget;
            _FBBIK.solver.leftHandEffector.target = _leftHandTarget;
            _FBBIK.solver.rightHandEffector.target = _rightHandTarget;
            _FBBIK.solver.leftArmChain.bendConstraint.bendGoal = _leftArmBendGoal;
            _FBBIK.solver.rightArmChain.bendConstraint.bendGoal = _rightArmBendGoal;
            _FBBIK.solver.leftShoulderEffector.target = _leftShoulderTarget;
            _FBBIK.solver.rightShoulderEffector.target = _rightShoulderTarget;
            _FBBIK.solver.leftFootEffector.target = _leftFootTarget;
            _FBBIK.solver.rightFootEffector.target = _rightFootTarget;
            _FBBIK.solver.leftLegChain.bendConstraint.bendGoal = _leftLegBendGoal;
            _FBBIK.solver.rightLegChain.bendConstraint.bendGoal = _rightLegBendGoal;
            _FBBIK.solver.leftThighEffector.target = _leftThighTarget;
            _FBBIK.solver.rightThighEffector.target = _rightThighTarget;

            _FBBIK.solver.bodyEffector.positionWeight = 1f;
            _FBBIK.solver.bodyEffector.rotationWeight = 1f;
            _FBBIK.solver.leftHandEffector.positionWeight = 1f;
            _FBBIK.solver.leftHandEffector.rotationWeight = 1f;
            _FBBIK.solver.rightHandEffector.positionWeight = 1f;
            _FBBIK.solver.rightHandEffector.rotationWeight = 1f;
            _FBBIK.solver.leftArmChain.bendConstraint.weight = 1f;
            _FBBIK.solver.rightArmChain.bendConstraint.weight = 1f;
            _FBBIK.solver.leftShoulderEffector.positionWeight = 1f;
            _FBBIK.solver.leftShoulderEffector.rotationWeight = 1f;
            _FBBIK.solver.rightShoulderEffector.positionWeight = 1f;
            _FBBIK.solver.rightShoulderEffector.rotationWeight = 1f;
            _FBBIK.solver.leftFootEffector.positionWeight = 1f;
            _FBBIK.solver.leftFootEffector.rotationWeight = 1f;
            _FBBIK.solver.rightFootEffector.positionWeight = 1f;
            _FBBIK.solver.rightFootEffector.rotationWeight = 1f;
            _FBBIK.solver.leftLegChain.bendConstraint.weight = 1f;
            _FBBIK.solver.rightLegChain.bendConstraint.weight = 1f;
            _FBBIK.solver.leftThighEffector.positionWeight = 1f;
            _FBBIK.solver.leftThighEffector.rotationWeight = 1f;
            _FBBIK.solver.rightThighEffector.positionWeight = 1f;
            _FBBIK.solver.rightThighEffector.rotationWeight = 1f;

            _FBBIK.solver.leftArmChain.bendConstraint.weight = 1f;
            _FBBIK.solver.rightArmChain.bendConstraint.weight = 1f;

            _FBBIK.solver.headMapping.maintainRotationWeight = 1f;
            _FBBIK.solver.leftArmMapping.maintainRotationWeight = 1f;
            _FBBIK.solver.rightArmMapping.maintainRotationWeight = 1f;
            _FBBIK.solver.leftLegMapping.maintainRotationWeight = 1f;
            _FBBIK.solver.rightLegMapping.maintainRotationWeight = 1f;

            _FBBIK.solver.bodyEffector.effectChildNodes = true;

            _headEffector = _headTarget.gameObject.AddComponent<FBBIKHeadEffector>();
            _headEffector.ik = _FBBIK;
            _headEffector.positionWeight = 1f;
            _headEffector.bodyWeight = 0.8f;
            _headEffector.thighWeight = 0.8f;
            _headEffector.rotationWeight = 1f;
            _headEffector.bodyClampWeight = 0f;
            _headEffector.headClampWeight = 0f;
            _headEffector.bendWeight = 1f;

            if (_enableTwistRelaxer)
            {
                _twistRelaxer.ik = _FBBIK;
                _twistRelaxer.twistSolvers = new TwistSolver[4];
                _twistRelaxer.twistSolvers[0] = new TwistSolver(_FBBIK.references.leftUpperArm);
                _twistRelaxer.twistSolvers[1] = new TwistSolver(_FBBIK.references.rightUpperArm);
                _twistRelaxer.twistSolvers[2] = new TwistSolver(_FBBIK.references.leftForearm);
                _twistRelaxer.twistSolvers[3] = new TwistSolver(_FBBIK.references.rightForearm);
                _twistRelaxer.twistSolvers[2].children = new Transform[1] { _FBBIK.references.leftHand };
                _twistRelaxer.twistSolvers[3].children = new Transform[1] { _FBBIK.references.rightHand };
                _twistRelaxer.twistSolvers[0].parentChildCrossfade = 0f;
                _twistRelaxer.twistSolvers[1].parentChildCrossfade = 0f;
                _twistRelaxer.twistSolvers[2].parentChildCrossfade = 0.5f;
                _twistRelaxer.twistSolvers[3].parentChildCrossfade = 0.5f;
            }

            _FBBIK.solver.IKPositionWeight = 1f;
        }

        private void InitializeFinger()
        {
            foreach (var bone in new HumanBodyBones[]
                {
                    HumanBodyBones.LeftLittleDistal,
                    HumanBodyBones.LeftLittleIntermediate,
                    HumanBodyBones.LeftLittleProximal,
                    HumanBodyBones.LeftRingDistal,
                    HumanBodyBones.LeftRingIntermediate,
                    HumanBodyBones.LeftRingProximal,
                    HumanBodyBones.LeftMiddleDistal,
                    HumanBodyBones.LeftMiddleIntermediate,
                    HumanBodyBones.LeftMiddleProximal,
                    HumanBodyBones.LeftIndexDistal,
                    HumanBodyBones.LeftIndexIntermediate,
                    HumanBodyBones.LeftIndexProximal,
                    HumanBodyBones.LeftThumbDistal,
                    HumanBodyBones.LeftThumbIntermediate,
                    HumanBodyBones.LeftThumbProximal,
                    HumanBodyBones.RightLittleDistal,
                    HumanBodyBones.RightLittleIntermediate,
                    HumanBodyBones.RightLittleProximal,
                    HumanBodyBones.RightRingDistal,
                    HumanBodyBones.RightRingIntermediate,
                    HumanBodyBones.RightRingProximal,
                    HumanBodyBones.RightMiddleDistal,
                    HumanBodyBones.RightMiddleIntermediate,
                    HumanBodyBones.RightMiddleProximal,
                    HumanBodyBones.RightIndexDistal,
                    HumanBodyBones.RightIndexIntermediate,
                    HumanBodyBones.RightIndexProximal,
                    HumanBodyBones.RightThumbDistal,
                    HumanBodyBones.RightThumbIntermediate,
                    HumanBodyBones.RightThumbProximal,
                })
            {
                var transform = _animator.GetBoneTransform(bone);

                if (transform == null) continue;

                _initialLocalRotations.Add(bone, transform.localRotation);

                if (!_initialBoneDirections.ContainsKey(bone))
                {
                    if (transform.childCount > 0)
                    {
                        _initialBoneDirections.Add(bone, transform.InverseTransformPoint(transform.GetChild(0).position).normalized);
                    }
                    else
                    {
                        if (transform.parent != null)
                        {
                            _initialBoneDirections.Add(bone, transform.parent.InverseTransformPoint(transform.position).normalized);
                        }
                        else
                        {
                            Debug.LogWarning($"Cannot determine initial bone direction for {bone}.");
                        }
                    }
                }
            }

            var leftHandTransform = _animator.GetBoneTransform(HumanBodyBones.LeftHand);
            var rightHandTransform = _animator.GetBoneTransform(HumanBodyBones.RightHand);
            var lMidProx = _animator.GetBoneTransform(HumanBodyBones.LeftMiddleProximal);
            var rMidProx = _animator.GetBoneTransform(HumanBodyBones.RightMiddleProximal);

            if (leftHandTransform != null && lMidProx != null)
            {
                _initialLocalRotations.Add(HumanBodyBones.LeftHand, leftHandTransform.localRotation);

                var forward = (leftHandTransform.position - lMidProx.position).normalized;

                _inverseLeftHandRotation = Quaternion.Inverse(LookRotation(forward, leftHandTransform.up)) * leftHandTransform.rotation;
            }

            if (rightHandTransform != null && rMidProx != null)
            {
                _initialLocalRotations.Add(HumanBodyBones.RightHand, rightHandTransform.localRotation);

                var forward = (rightHandTransform.position - rMidProx.position).normalized;

                _inverseRightHandRotation = Quaternion.Inverse(LookRotation(forward, rightHandTransform.up)) * rightHandTransform.rotation;
            }
        }

        private void CreateTarget()
        {
            _root = new GameObject($"{name} (Tracking Space)").transform;
            _headTarget = new GameObject("Head Target").transform;
            _pelvisTarget = new GameObject("Pelvis Target").transform;
            _leftHandTarget = new GameObject("Left Hand Target").transform;
            _rightHandTarget = new GameObject("Right Hand Target").transform;
            _leftArmBendGoal = new GameObject("Left Arm Bend Goal").transform;
            _rightArmBendGoal = new GameObject("Right Arm Bend Goal").transform;
            _leftShoulderTarget = new GameObject("Left Shoulder Target").transform;
            _rightShoulderTarget = new GameObject("Right Shoulder Target").transform;
            _leftFootTarget = new GameObject("Left Foot Target").transform;
            _rightFootTarget = new GameObject("Right Foot Target").transform;
            _leftLegBendGoal = new GameObject("Left Leg Bend Goal").transform;
            _rightLegBendGoal = new GameObject("Right Leg Bend Goal").transform;
            _leftThighTarget = new GameObject("Left Thigh Target").transform;
            _rightThighTarget = new GameObject("Right Thigh Target").transform;

            _headTarget.parent = _root;
            _pelvisTarget.parent = _root;
            _leftHandTarget.parent = _root;
            _rightHandTarget.parent = _root;
            _leftArmBendGoal.parent = _root;
            _rightArmBendGoal.parent = _root;
            _leftShoulderTarget.parent = _root;
            _rightShoulderTarget.parent = _root;
            _leftFootTarget.parent = _root;
            _rightFootTarget.parent = _root;
            _leftLegBendGoal.parent = _root;
            _rightLegBendGoal.parent = _root;
            _leftThighTarget.parent = _root;
            _rightThighTarget.parent = _root;
        }

        private void UpdateTarget()
        {
            if (!_activePoseWorldLandmark) return;

            var rotDelta = Time.deltaTime * _targetRotationSmoothSpeed;

            // --------------------------------------------------
            // Body Rotation (Stabilized version)
            // --------------------------------------------------

            var centerHipPosition = (_poseWorldLandmarks[(int)PoseLandmark.LeftHip].Position + _poseWorldLandmarks[(int)PoseLandmark.RightHip].Position) / 2f;
            var centerShoulderPosition = (_poseWorldLandmarks[(int)PoseLandmark.LeftShoulder].Position + _poseWorldLandmarks[(int)PoseLandmark.RightShoulder].Position) / 2f;

            var bodyRightVector = (_poseWorldLandmarks[(int)PoseLandmark.RightHip].Position - _poseWorldLandmarks[(int)PoseLandmark.LeftHip].Position).normalized;
            var bodyUpVector = (centerShoulderPosition - centerHipPosition).normalized;
            var bodyForwardVector = Vector3.Cross(bodyRightVector, bodyUpVector).normalized;

            var targetPelvisRotation = LookRotation(bodyForwardVector, bodyUpVector);
            _pelvisTarget.localRotation = Quaternion.Slerp(_pelvisTarget.localRotation, targetPelvisRotation, rotDelta);


            // --------------------------------------------------
            // Head Rotation
            // --------------------------------------------------

            var centerEyePosition = (_poseWorldLandmarks[(int)PoseLandmark.LeftEye].Position + _poseWorldLandmarks[(int)PoseLandmark.RightEye].Position) / 2f;
            var centerEarPosition = (_poseWorldLandmarks[(int)PoseLandmark.LeftEar].Position + _poseWorldLandmarks[(int)PoseLandmark.RightEar].Position) / 2f;

            var headRightVector = (_poseWorldLandmarks[(int)PoseLandmark.RightEar].Position - _poseWorldLandmarks[(int)PoseLandmark.LeftEar].Position).normalized;
            var headForwardVector = (centerEyePosition - centerEarPosition).normalized;
            var headUpVector = Vector3.Cross(headForwardVector, headRightVector).normalized;

            var targetHeadRotation = LookRotation(headForwardVector, headUpVector);
            _headTarget.localRotation = Quaternion.Slerp(_headTarget.localRotation, targetHeadRotation, rotDelta);


            // --------------------------------------------------
            // Left Hand Rotation
            // --------------------------------------------------

            Quaternion targetLeftHandRotation;

            if (!_activeLeftHandWorldLandmark)
            {
                var centerLeftHandPosition = (_poseWorldLandmarks[(int)PoseLandmark.LeftIndex].Position + _poseWorldLandmarks[(int)PoseLandmark.LeftPinky].Position) / 2f;
                var leftHandForwardVector = (centerLeftHandPosition - _poseWorldLandmarks[(int)PoseLandmark.LeftWrist].Position).normalized;
                var leftHandRightVector = (_poseWorldLandmarks[(int)PoseLandmark.LeftIndex].Position - _poseWorldLandmarks[(int)PoseLandmark.LeftPinky].Position).normalized;
                var leftHandUpVector = Vector3.Cross(leftHandForwardVector, leftHandRightVector).normalized;

                targetLeftHandRotation = LookRotation(leftHandForwardVector, leftHandUpVector) * Quaternion.Euler(_handRotationOffset);
            }
            else
            {
                var leftHandWristToMiddle = _leftHandWorldLandmarks[(int)HandLandmark.MiddleFingerMcp].Position - _leftHandWorldLandmarks[(int)HandLandmark.Wrist].Position;
                var leftHandPinkyToIndex = _leftHandWorldLandmarks[(int)HandLandmark.IndexFingerMcp].Position - _leftHandWorldLandmarks[(int)HandLandmark.PinkyMcp].Position;
                var leftHandUpVector = Vector3.Cross(leftHandPinkyToIndex, leftHandWristToMiddle).normalized;
                var leftHandForwardVector = Vector3.Cross(leftHandUpVector, leftHandPinkyToIndex).normalized;

                targetLeftHandRotation = LookRotation(leftHandForwardVector, -leftHandUpVector) * Quaternion.Euler(_handRotationOffset);
            }

            _leftHandTarget.localRotation = Quaternion.Slerp(_leftHandTarget.localRotation, targetLeftHandRotation, rotDelta);


            // --------------------------------------------------
            // Right Hand Rotation
            // --------------------------------------------------

            Quaternion targetRightHandRotation;

            if (!_activeRightHandWorldLandmark)
            {
                var centerRightHandPosition = (_poseWorldLandmarks[(int)PoseLandmark.RightIndex].Position + _poseWorldLandmarks[(int)PoseLandmark.RightPinky].Position) / 2f;
                var rightHandForwardVector = (centerRightHandPosition - _poseWorldLandmarks[(int)PoseLandmark.RightWrist].Position).normalized;
                var rightHandRightVector = (_poseWorldLandmarks[(int)PoseLandmark.RightIndex].Position - _poseWorldLandmarks[(int)PoseLandmark.RightPinky].Position).normalized;
                var rightHandUpVector = Vector3.Cross(-rightHandForwardVector, rightHandRightVector).normalized;

                targetRightHandRotation = LookRotation(-rightHandForwardVector, rightHandUpVector) * Quaternion.Euler(_handRotationOffset);
            }
            else
            {
                var rightHandWristToMiddle = _rightHandWorldLandmarks[(int)HandLandmark.MiddleFingerMcp].Position - _rightHandWorldLandmarks[(int)HandLandmark.Wrist].Position;
                var rightHandPinkyToIndex = _rightHandWorldLandmarks[(int)HandLandmark.IndexFingerMcp].Position - _rightHandWorldLandmarks[(int)HandLandmark.PinkyMcp].Position;
                var rightHandUpVector = Vector3.Cross(rightHandPinkyToIndex, rightHandWristToMiddle).normalized;
                var rightHandForwardVector = Vector3.Cross(rightHandUpVector, rightHandPinkyToIndex).normalized;

                targetRightHandRotation = LookRotation(-rightHandForwardVector, rightHandUpVector) * Quaternion.Euler(_handRotationOffset);
            }

            _rightHandTarget.localRotation = Quaternion.Slerp(_rightHandTarget.localRotation, targetRightHandRotation, rotDelta);


            // --------------------------------------------------
            // Foot Rotation
            // --------------------------------------------------

            var leftFootForwardVector = (_poseWorldLandmarks[(int)PoseLandmark.LeftFootIndex].Position - _poseWorldLandmarks[(int)PoseLandmark.LeftHeel].Position).normalized;
            var rightFootForwardVector = (_poseWorldLandmarks[(int)PoseLandmark.RightFootIndex].Position - _poseWorldLandmarks[(int)PoseLandmark.RightHeel].Position).normalized;

            var targetLeftFootRotation = LookRotation(leftFootForwardVector, Vector3.up);
            var targetRightFootRotation = LookRotation(rightFootForwardVector, Vector3.up);

            _leftFootTarget.localRotation = Quaternion.Slerp(_leftFootTarget.localRotation, targetLeftFootRotation, rotDelta);
            _rightFootTarget.localRotation = Quaternion.Slerp(_rightFootTarget.localRotation, targetRightFootRotation, rotDelta);


            // --------------------------------------------------
            // Position
            // --------------------------------------------------

            _headTarget.localPosition = _poseWorldLandmarks[(int)PoseLandmark.Nose].Position;

            _leftHandTarget.localPosition = _poseWorldLandmarks[(int)PoseLandmark.LeftWrist].Position;
            _rightHandTarget.localPosition = _poseWorldLandmarks[(int)PoseLandmark.RightWrist].Position;

            _leftArmBendGoal.localPosition = _poseWorldLandmarks[(int)PoseLandmark.LeftElbow].Position;
            _rightArmBendGoal.localPosition = _poseWorldLandmarks[(int)PoseLandmark.RightElbow].Position;

            _leftShoulderTarget.localPosition = _poseWorldLandmarks[(int)PoseLandmark.LeftShoulder].Position;
            _rightShoulderTarget.localPosition = _poseWorldLandmarks[(int)PoseLandmark.RightShoulder].Position;

            _leftFootTarget.localPosition = _poseWorldLandmarks[(int)PoseLandmark.LeftHeel].Position;
            _rightFootTarget.localPosition = _poseWorldLandmarks[(int)PoseLandmark.RightHeel].Position;

            _leftLegBendGoal.localPosition = _poseWorldLandmarks[(int)PoseLandmark.LeftKnee].Position;
            _rightLegBendGoal.localPosition = _poseWorldLandmarks[(int)PoseLandmark.RightKnee].Position;

            _leftThighTarget.localPosition = _poseWorldLandmarks[(int)PoseLandmark.LeftHip].Position;
            _rightThighTarget.localPosition = _poseWorldLandmarks[(int)PoseLandmark.RightHip].Position;
        }

        private void UpdateFinger()
        {
            if (_activeLeftHandWorldLandmark)
            {
                var lHandTransform = _animator.GetBoneTransform(HumanBodyBones.LeftHand);

                var lHandUp = TriangleNormal(
                                        _leftHandWorldLandmarks[(int)HandLandmark.Wrist].Position,
                                        _leftHandWorldLandmarks[(int)HandLandmark.PinkyMcp].Position,
                                        _leftHandWorldLandmarks[(int)HandLandmark.IndexFingerMcp].Position);
                var lHandForward = (_leftHandWorldLandmarks[(int)HandLandmark.Wrist].Position - _leftHandWorldLandmarks[(int)HandLandmark.MiddleFingerMcp].Position).normalized;

                lHandTransform.rotation = LookRotation(lHandForward, lHandUp) * _inverseLeftHandRotation;

                var invLeftHandWorldRot = Quaternion.Inverse(lHandTransform.rotation);

                ComputeFingerRotation(HumanBodyBones.LeftThumbProximal, HandLandmark.ThumbCmc, invLeftHandWorldRot);
                ComputeFingerRotation(HumanBodyBones.LeftIndexProximal, HandLandmark.IndexFingerMcp, invLeftHandWorldRot);
                ComputeFingerRotation(HumanBodyBones.LeftMiddleProximal, HandLandmark.MiddleFingerMcp, invLeftHandWorldRot);
                ComputeFingerRotation(HumanBodyBones.LeftRingProximal, HandLandmark.RingFingerMcp, invLeftHandWorldRot);
                ComputeFingerRotation(HumanBodyBones.LeftLittleProximal, HandLandmark.PinkyMcp, invLeftHandWorldRot);
            }
            else
            {
                ResetHandBones(false);
            }

            if (_activeRightHandWorldLandmark)
            {
                var rHandTransform = _animator.GetBoneTransform(HumanBodyBones.RightHand);

                var rHandUp = TriangleNormal(
                                        _rightHandWorldLandmarks[(int)HandLandmark.Wrist].Position,
                                        _rightHandWorldLandmarks[(int)HandLandmark.IndexFingerMcp].Position,
                                        _rightHandWorldLandmarks[(int)HandLandmark.PinkyMcp].Position);
                var rHandForward = (_rightHandWorldLandmarks[(int)HandLandmark.Wrist].Position - _rightHandWorldLandmarks[(int)HandLandmark.MiddleFingerMcp].Position).normalized;

                rHandTransform.rotation = LookRotation(rHandForward, rHandUp) * _inverseRightHandRotation;

                var invRightHandWorldRot = Quaternion.Inverse(rHandTransform.rotation);

                ComputeFingerRotation(HumanBodyBones.RightThumbProximal, HandLandmark.ThumbCmc, invRightHandWorldRot);
                ComputeFingerRotation(HumanBodyBones.RightIndexProximal, HandLandmark.IndexFingerMcp, invRightHandWorldRot);
                ComputeFingerRotation(HumanBodyBones.RightMiddleProximal, HandLandmark.MiddleFingerMcp, invRightHandWorldRot);
                ComputeFingerRotation(HumanBodyBones.RightRingProximal, HandLandmark.RingFingerMcp, invRightHandWorldRot);
                ComputeFingerRotation(HumanBodyBones.RightLittleProximal, HandLandmark.PinkyMcp, invRightHandWorldRot);
            }
            else
            {
                ResetHandBones(true);
            }
        }

        private void UpdateVRIK()
        {
            if (!_activePoseWorldLandmark) return;

            _VRIK.references.root.position = new Vector3(_pelvisTarget.position.x, _VRIK.references.root.position.y, _pelvisTarget.position.z);

            var pelvisTargetRight = Quaternion.Inverse(_pelvisTarget.rotation) * _VRIK.references.root.right;

            _VRIK.references.root.rotation = Quaternion.LookRotation(Vector3.Cross(_pelvisTarget.rotation * pelvisTargetRight, _VRIK.references.root.up));
            _VRIK.references.pelvis.position = Vector3.Lerp(_VRIK.references.pelvis.position, _pelvisTarget.position, _VRIK.solver.spine.pelvisPositionWeight);
            _VRIK.references.pelvis.rotation = Quaternion.Slerp(_VRIK.references.pelvis.rotation, _pelvisTarget.rotation, _VRIK.solver.spine.pelvisRotationWeight);

            if (_autoWeight)
            {
                var targetLeftLegWeight = _poseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility ?? 0f;
                var targetRightLegWeight = _poseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility ?? 0f;
                var targetLeftBendWeight = (_poseWorldLandmarks[(int)PoseLandmark.LeftKnee].Visibility * _poseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility) ?? 0f;
                var targetRightBendWeight = (_poseWorldLandmarks[(int)PoseLandmark.RightKnee].Visibility * _poseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility) ?? 0f;

                var lerpDelta = Time.deltaTime * _weightSmoothingSpeed;

                _currentLeftLegWeight = Mathf.Lerp(_currentLeftLegWeight, targetLeftLegWeight, lerpDelta);
                _currentRightLegWeight = Mathf.Lerp(_currentRightLegWeight, targetRightLegWeight, lerpDelta);
                _currentLeftBendWeight = Mathf.Lerp(_currentLeftBendWeight, targetLeftBendWeight, lerpDelta);
                _currentRightBendWeight = Mathf.Lerp(_currentRightBendWeight, targetRightBendWeight, lerpDelta);

                _VRIK.solver.leftLeg.positionWeight = _currentLeftLegWeight;
                _VRIK.solver.leftLeg.rotationWeight = _currentLeftLegWeight;
                _VRIK.solver.leftLeg.bendGoalWeight = _currentLeftBendWeight;

                _VRIK.solver.rightLeg.positionWeight = _currentRightLegWeight;
                _VRIK.solver.rightLeg.rotationWeight = _currentRightLegWeight;
                _VRIK.solver.rightLeg.bendGoalWeight = _currentRightBendWeight;
            }
            else
            {
                if (_poseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility > 0.5f)
                {
                    _VRIK.solver.leftLeg.positionWeight = 1f;
                    _VRIK.solver.leftLeg.rotationWeight = 1f;
                }
                else
                {
                    _VRIK.solver.leftLeg.positionWeight = 0f;
                    _VRIK.solver.leftLeg.rotationWeight = 0f;
                }

                if (_poseWorldLandmarks[(int)PoseLandmark.LeftKnee].Visibility > 0.5f)
                {
                    _VRIK.solver.leftLeg.bendGoalWeight = 1f;
                }
                else
                {
                    _VRIK.solver.leftLeg.bendGoalWeight = 0f;
                }

                if (_poseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility > 0.5f)
                {
                    _VRIK.solver.rightLeg.positionWeight = 1f;
                    _VRIK.solver.rightLeg.rotationWeight = 1f;
                }
                else
                {
                    _VRIK.solver.rightLeg.positionWeight = 0f;
                    _VRIK.solver.rightLeg.rotationWeight = 0f;
                }

                if (_poseWorldLandmarks[(int)PoseLandmark.RightKnee].Visibility > 0.5f)
                {
                    _VRIK.solver.rightLeg.bendGoalWeight = 1f;
                }
                else
                {
                    _VRIK.solver.rightLeg.bendGoalWeight = 0f;
                }
            }
        }

        private void UpdateFBBIK()
        {
            if (!_activePoseWorldLandmark) return;

            if (_autoWeight)
            {
                _FBBIK.solver.leftFootEffector.positionWeight = _poseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility ?? 0f;
                _FBBIK.solver.leftFootEffector.rotationWeight = _poseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility ?? 0f;
                _FBBIK.solver.rightFootEffector.positionWeight = _poseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility ?? 0f;
                _FBBIK.solver.rightFootEffector.rotationWeight = _poseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility ?? 0f;
                _FBBIK.solver.leftLegChain.bendConstraint.weight = _poseWorldLandmarks[(int)PoseLandmark.LeftKnee].Visibility ?? 0f;
                _FBBIK.solver.rightLegChain.bendConstraint.weight = _poseWorldLandmarks[(int)PoseLandmark.RightKnee].Visibility ?? 0f;
                _FBBIK.solver.leftThighEffector.positionWeight = _poseWorldLandmarks[(int)PoseLandmark.LeftHip].Visibility ?? 0f;
                _FBBIK.solver.leftThighEffector.rotationWeight = _poseWorldLandmarks[(int)PoseLandmark.LeftHip].Visibility ?? 0f;
                _FBBIK.solver.rightThighEffector.positionWeight = _poseWorldLandmarks[(int)PoseLandmark.RightHip].Visibility ?? 0f;
                _FBBIK.solver.rightThighEffector.rotationWeight = _poseWorldLandmarks[(int)PoseLandmark.RightHip].Visibility ?? 0f;
            }
            else
            {
                if (_poseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility > 0.5f)
                {
                    _FBBIK.solver.leftFootEffector.positionWeight = 1f;
                    _FBBIK.solver.leftFootEffector.rotationWeight = 1f;
                }
                else
                {
                    _FBBIK.solver.leftFootEffector.positionWeight = 0f;
                    _FBBIK.solver.leftFootEffector.rotationWeight = 0f;
                }

                if (_poseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility > 0.5f)
                {
                    _FBBIK.solver.rightFootEffector.positionWeight = 1f;
                    _FBBIK.solver.rightFootEffector.rotationWeight = 1f;
                }
                else
                {
                    _FBBIK.solver.rightFootEffector.positionWeight = 0f;
                    _FBBIK.solver.rightFootEffector.rotationWeight = 0f;
                }

                if (_poseWorldLandmarks[(int)PoseLandmark.LeftKnee].Visibility > 0.5f)
                {
                    _FBBIK.solver.leftLegChain.bendConstraint.weight = 1f;
                }
                else
                {
                    _FBBIK.solver.leftLegChain.bendConstraint.weight = 0f;
                }

                if (_poseWorldLandmarks[(int)PoseLandmark.RightKnee].Visibility > 0.5f)
                {
                    _FBBIK.solver.rightLegChain.bendConstraint.weight = 1f;
                }
                else
                {
                    _FBBIK.solver.rightLegChain.bendConstraint.weight = 0f;
                }

                if (_poseWorldLandmarks[(int)PoseLandmark.LeftHip].Visibility > 0.5f)
                {
                    _FBBIK.solver.leftThighEffector.positionWeight = 1f;
                    _FBBIK.solver.leftThighEffector.rotationWeight = 1f;
                }
                else
                {
                    _FBBIK.solver.leftThighEffector.positionWeight = 0f;
                    _FBBIK.solver.leftThighEffector.rotationWeight = 0f;
                }

                if (_poseWorldLandmarks[(int)PoseLandmark.RightHip].Visibility > 0.5f)
                {
                    _FBBIK.solver.rightThighEffector.positionWeight = 1f;
                    _FBBIK.solver.rightThighEffector.rotationWeight = 1f;
                }
                else
                {
                    _FBBIK.solver.rightThighEffector.positionWeight = 0f;
                    _FBBIK.solver.rightThighEffector.rotationWeight = 0f;
                }
            }
        }

        private void ResetHandBones(bool isRightHand = false)
        {
            var handBones = isRightHand ?
                new[] {
                    HumanBodyBones.RightLittleDistal,
                    HumanBodyBones.RightLittleIntermediate,
                    HumanBodyBones.RightLittleProximal,
                    HumanBodyBones.RightRingDistal,
                    HumanBodyBones.RightRingIntermediate,
                    HumanBodyBones.RightRingProximal,
                    HumanBodyBones.RightMiddleDistal,
                    HumanBodyBones.RightMiddleIntermediate,
                    HumanBodyBones.RightMiddleProximal,
                    HumanBodyBones.RightIndexDistal,
                    HumanBodyBones.RightIndexIntermediate,
                    HumanBodyBones.RightIndexProximal,
                    HumanBodyBones.RightThumbDistal,
                    HumanBodyBones.RightThumbIntermediate,
                    HumanBodyBones.RightThumbProximal,
                } :
                new[] {
                    HumanBodyBones.LeftLittleDistal,
                    HumanBodyBones.LeftLittleIntermediate,
                    HumanBodyBones.LeftLittleProximal,
                    HumanBodyBones.LeftRingDistal,
                    HumanBodyBones.LeftRingIntermediate,
                    HumanBodyBones.LeftRingProximal,
                    HumanBodyBones.LeftMiddleDistal,
                    HumanBodyBones.LeftMiddleIntermediate,
                    HumanBodyBones.LeftMiddleProximal,
                    HumanBodyBones.LeftIndexDistal,
                    HumanBodyBones.LeftIndexIntermediate,
                    HumanBodyBones.LeftIndexProximal,
                    HumanBodyBones.LeftThumbDistal,
                    HumanBodyBones.LeftThumbIntermediate,
                    HumanBodyBones.LeftThumbProximal,
                };

            foreach (var bone in handBones)
            {
                if (_initialLocalRotations.TryGetValue(bone, out Quaternion initialRotation))
                {
                    var transform = _animator.GetBoneTransform(bone);

                    if (transform != null)
                    {
                        transform.localRotation = Quaternion.Slerp(
                            transform.localRotation,
                            initialRotation,
                            Time.deltaTime * _fingerResetSpeed
                        );
                    }
                }
            }
        }

        private void ComputeFingerRotation(HumanBodyBones proximalBone, HandLandmark firstLandmark, Quaternion handWorldRotation)
        {
            for (int i = 0; i < 3; i++)
            {
                var bone = proximalBone + i;

                if (_initialLocalRotations.TryGetValue(bone, out Quaternion initialLocalRotation))
                {
                    var transform = _animator.GetBoneTransform(bone);

                    if (transform == null) continue;

                    var isRightHand = bone.ToString().StartsWith("Right");

                    var landmarks = isRightHand ? _rightHandWorldLandmarks : _leftHandWorldLandmarks;

                    var worldTargetDirection = (landmarks[(int)firstLandmark + i + 1].Position - landmarks[(int)firstLandmark + i].Position).normalized;

                    if (worldTargetDirection == Vector3.zero) continue;

                    var initialBoneLocalTargetDirection = Quaternion.Inverse(initialLocalRotation) * handWorldRotation * worldTargetDirection;

                    transform.localRotation = Quaternion.FromToRotation(_initialBoneDirections[bone], initialBoneLocalTargetDirection) * initialLocalRotation;

                    handWorldRotation = Quaternion.Inverse(transform.localRotation) * handWorldRotation;
                }
            }
        }


        private void OnCallback(HolisticLandmarkerResult result)
        {
            _activeFaceLandmark = Set(_faceLandmarks, result.faceLandmarks.landmarks);
            _activePoseLandmark = Set(_poseLandmarks, result.poseLandmarks.landmarks);
            _activePoseWorldLandmark = Set(_poseWorldLandmarks, result.poseWorldLandmarks.landmarks);
            _activeLeftHandLandmark = Set(_leftHandLandmarks, result.leftHandLandmarks.landmarks);
            _activeLeftHandWorldLandmark = Set(_leftHandWorldLandmarks, result.leftHandWorldLandmarks.landmarks);
            _activeRightHandLandmark = Set(_rightHandLandmarks, result.rightHandLandmarks.landmarks);
            _activeRightHandWorldLandmark = Set(_rightHandWorldLandmarks, result.rightHandWorldLandmarks.landmarks);
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


        private static Quaternion LookRotation(Vector3 forward, Vector3 upwards)
        {
            if (forward == Vector3.zero)
            {
                return Quaternion.identity;
            }

            return Quaternion.LookRotation(forward, upwards);
        }

        private static Vector3 TriangleNormal(Vector3 a, Vector3 b, Vector3 c)
        {
            var v = Vector3.Cross(a - b, a - c);

            return v.normalized;
        }
    }

    public enum IKType
    {
        [InspectorName("VR IK")] VRIK,
        [InspectorName("Full Body Biped IK")] FBBIK,
    }
}