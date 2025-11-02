using Mediapipe;
using RootMotion;
using RootMotion.FinalIK;
using System.Collections.Generic;
using UnityEngine;

namespace PMC
{
    [AddComponentMenu("Prism Motion Capture/Prism Avatar")]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Animator))]
    public sealed class PrismAvatar : MonoBehaviour
    {
        // Fields

        [Space]
        [SerializeField] private PrismTracker _tracker;

        [Header("IK Settings")]
        [SerializeField] private IKType _IKType = IKType.FBBIK;
        [SerializeField] private bool _enableTwistRelaxer = true;
        [SerializeField] private bool _enableMovement = true;
        [SerializeField] private bool _autoWeight = true;
        [Space]
        [SerializeField] private Vector3 _landmarkScale = new(1f, 1f, -1f);
        [SerializeField] private Vector3 _handRotationOffset = new(0f, 90f, 0f);

        [Header("Kalman Filter Settings")]
        [SerializeField] private bool _enableKalmanFilter = true;
        [SerializeField] private float _timeInterval = 0.45f;
        [SerializeField] private float _noise = 0.4f;

        private Animator _animator;

        private VRIK _VRIK;
        private VRIKRootController _rootController;

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
        private Vector3 _pelvisBasePosition;

        private float _sittingHeight;
        private bool _activePoseLandmark;
        private bool _activeLeftHandLandmark;
        private bool _activeRightHandLandmark;

        private readonly Landmark[] _poseLandmarks = new Landmark[(int)PoseLandmark.Count];
        private readonly Landmark[] _poseWorldLandmarks = new Landmark[(int)PoseLandmark.Count];
        private readonly Landmark[] _leftHandLandmarks = new Landmark[(int)HandLandmark.Count];
        private readonly Landmark[] _rightHandLandmarks = new Landmark[(int)HandLandmark.Count];

        private readonly Dictionary<HumanBodyBones, Quaternion> _initialLocalRotations = new();
        private readonly Dictionary<HumanBodyBones, Quaternion> _inverseRotations = new();
        private readonly Dictionary<HumanBodyBones, Vector3> _initialBoneDirections = new();


        // Methods

        [ContextMenu("Execute Calibration (Play Mode Only)")]
        public void ExecuteCalibration()
        {
            if (!Application.isPlaying) return;

            var distance = Vector3.Distance(Vector3.zero, _poseWorldLandmarks[(int)PoseLandmark.Nose].Position);

            _landmarkScale *= _sittingHeight / distance;
        }


        private void Awake()
        {
            _animator = gameObject.GetComponent<Animator>();

            if (_IKType == IKType.VRIK)
            {
                if (!gameObject.TryGetComponent(out _VRIK))
                {
                    _VRIK = gameObject.AddComponent<VRIK>();
                }

                if (!gameObject.TryGetComponent(out _rootController))
                {
                    _rootController = gameObject.AddComponent<VRIKRootController>();
                }
            }

            if (_IKType == IKType.FBBIK)
            {
                if (!gameObject.TryGetComponent(out _FBBIK))
                {
                    _FBBIK = gameObject.AddComponent<FullBodyBipedIK>();
                }
            }

            if (_enableTwistRelaxer && _IKType == IKType.FBBIK)
            {
                if (!gameObject.TryGetComponent(out _twistRelaxer))
                {
                    _twistRelaxer = gameObject.AddComponent<TwistRelaxer>();
                }
            }

            for (int i = 0; i < _poseLandmarks.Length; i++)
            {
                if (_poseLandmarks[i] == null)
                {
                    _poseLandmarks[i] = new Landmark();
                }

                if (_enableKalmanFilter)
                {
                    _poseLandmarks[i].KalmanFilter = new KalmanFilter();
                    _poseLandmarks[i].KalmanFilter.SetParameter(_timeInterval, _noise);
                    _poseLandmarks[i].KalmanFilter.Predict();
                }
            }

            for (int i = 0; i < _poseWorldLandmarks.Length; i++)
            {
                if (_poseWorldLandmarks[i] == null)
                {
                    _poseWorldLandmarks[i] = new Landmark();
                }

                if (_enableKalmanFilter)
                {
                    _poseWorldLandmarks[i].KalmanFilter = new KalmanFilter();
                    _poseWorldLandmarks[i].KalmanFilter.SetParameter(_timeInterval, _noise);
                    _poseWorldLandmarks[i].KalmanFilter.Predict();
                }
            }

            for (int i = 0; i < _leftHandLandmarks.Length; i++)
            {
                if (_leftHandLandmarks[i] == null)
                {
                    _leftHandLandmarks[i] = new Landmark();
                }

                if (_enableKalmanFilter)
                {
                    _leftHandLandmarks[i].KalmanFilter = new KalmanFilter();
                    _leftHandLandmarks[i].KalmanFilter.SetParameter(_timeInterval, _noise);
                    _leftHandLandmarks[i].KalmanFilter.Predict();
                }
            }

            for (int i = 0; i < _rightHandLandmarks.Length; i++)
            {
                if (_rightHandLandmarks[i] == null)
                {
                    _rightHandLandmarks[i] = new Landmark();
                }

                if (_enableKalmanFilter)
                {
                    _rightHandLandmarks[i].KalmanFilter = new KalmanFilter();
                    _rightHandLandmarks[i].KalmanFilter.SetParameter(_timeInterval, _noise);
                    _rightHandLandmarks[i].KalmanFilter.Predict();
                }
            }
        }

        private void Start()
        {
            if (_animator == null) return;

            if (_tracker != null)
            {
                _tracker.OnPoseLandmarks += OnPoseLandmarks;
                _tracker.OnPoseWorldLandmarks += OnPoseWorldLandmarks;
                _tracker.OnLeftHandLandmarks += OnLeftHandLandmarks;
                _tracker.OnRightHandLandmarks += OnRightHandLandmarks;
            }

            CreateTarget();

            InitializeFinger();

            if (_IKType == IKType.VRIK)
            {
                _VRIK.solver.OnPreUpdate += OnPreVRIK;

                InitializeVRIK();
            }

            if (_IKType == IKType.FBBIK)
            {
                _FBBIK.solver.OnPreUpdate += OnPreFBBIK;

                InitializeFBBIK();
            }
        }

        private void OnDestroy()
        {
            if (_tracker != null)
            {
                _tracker.OnPoseLandmarks -= OnPoseLandmarks;
                _tracker.OnPoseWorldLandmarks -= OnPoseWorldLandmarks;
                _tracker.OnLeftHandLandmarks -= OnLeftHandLandmarks;
                _tracker.OnRightHandLandmarks -= OnRightHandLandmarks;
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

        private void OnPreVRIK()
        {
            if (_activePoseLandmark)
            {
                UpdateTarget();

                UpdateVRIK();
            }

            UpdateFinger();
        }

        private void OnPreFBBIK()
        {
            _root.transform.localRotation = _FBBIK.references.root.rotation;

            if (_activePoseLandmark)
            {
                UpdateTarget();

                UpdateFBBIK();

                if (_enableMovement)
                {
                    var c_Hip = (_poseLandmarks[(int)PoseLandmark.LeftHip].Position + _poseLandmarks[(int)PoseLandmark.RightHip].Position) / 2;

                    _root.transform.localPosition = new Vector3(c_Hip.x, c_Hip.y, c_Hip.z) + _basePosition;

                    _FBBIK.references.root.position = _root.transform.localPosition - _pelvisBasePosition;
                }
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
            _VRIK.solver.spine.maintainPelvisPosition = 0f;
            _VRIK.solver.plantFeet = false;

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

            _VRIK.solver.spine.positionWeight = 1f;
            _VRIK.solver.spine.rotationWeight = 1f;
            _VRIK.solver.spine.pelvisPositionWeight = 1f;
            _VRIK.solver.spine.pelvisRotationWeight = 1f;
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

            _VRIK.solver.leftArm.shoulderRotationMode = IKSolverVR.Arm.ShoulderRotationMode.FromTo;
            _VRIK.solver.leftArm.shoulderRotationWeight = 0.3f;
            _VRIK.solver.leftArm.shoulderTwistWeight = 0.7f;

            _VRIK.solver.rightArm.shoulderRotationMode = IKSolverVR.Arm.ShoulderRotationMode.FromTo;
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

            _rootController.Calibrate();

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

            _FBBIK.solver.leftArmChain.bendConstraint.weight = 0.7f;
            _FBBIK.solver.rightArmChain.bendConstraint.weight = 0.7f;

            _FBBIK.solver.headMapping.maintainRotationWeight = 1f;
            _FBBIK.solver.leftArmMapping.maintainRotationWeight = 1f;
            _FBBIK.solver.rightArmMapping.maintainRotationWeight = 1f;
            _FBBIK.solver.leftLegMapping.maintainRotationWeight = 1f;
            _FBBIK.solver.rightLegMapping.maintainRotationWeight = 1f;

            _FBBIK.solver.bodyEffector.effectChildNodes = false;

            _headEffector = _headTarget.gameObject.AddComponent<FBBIKHeadEffector>();
            _headEffector.ik = _FBBIK;
            _headEffector.positionWeight = 1f;
            _headEffector.rotationWeight = 1f;
            _headEffector.bodyClampWeight = 0f;
            _headEffector.headClampWeight = 0f;

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
                _twistRelaxer.twistSolvers[2].parentChildCrossfade = 1f;
                _twistRelaxer.twistSolvers[3].parentChildCrossfade = 1f;
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

                if (!_initialLocalRotations.ContainsKey(bone))
                {
                    _initialLocalRotations.Add(bone, transform.localRotation);
                }

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
                            _initialBoneDirections.Add(bone, Vector3.up);
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
                _initialLocalRotations.TryAdd(HumanBodyBones.LeftHand, leftHandTransform.localRotation);

                var forward = (leftHandTransform.position - lMidProx.position).normalized;
                var up = leftHandTransform.up;

                if (forward != Vector3.zero && up != Vector3.zero)
                {
                    _inverseRotations.TryAdd(HumanBodyBones.LeftHand,
                        Quaternion.Inverse(Quaternion.LookRotation(forward, up)) * leftHandTransform.rotation);
                }
            }

            if (rightHandTransform != null && rMidProx != null)
            {
                _initialLocalRotations.TryAdd(HumanBodyBones.RightHand, rightHandTransform.localRotation);

                var forward = (rightHandTransform.position - rMidProx.position).normalized;
                var up = rightHandTransform.up;

                if (forward != Vector3.zero && up != Vector3.zero)
                {
                    _inverseRotations.TryAdd(HumanBodyBones.RightHand,
                        Quaternion.Inverse(Quaternion.LookRotation(forward, up)) * rightHandTransform.rotation);
                }
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
            // --------------------------------------------------
            // Body Rotation
            // --------------------------------------------------

            var centerHipPosition = (_poseWorldLandmarks[(int)PoseLandmark.LeftHip].Position + _poseWorldLandmarks[(int)PoseLandmark.RightHip].Position) / 2f;
            var centerShoulderPosition = (_poseWorldLandmarks[(int)PoseLandmark.LeftShoulder].Position + _poseWorldLandmarks[(int)PoseLandmark.RightShoulder].Position) / 2f;

            var bodyRightVector = (_poseWorldLandmarks[(int)PoseLandmark.RightShoulder].Position - _poseWorldLandmarks[(int)PoseLandmark.LeftShoulder].Position).normalized;
            var bodyUpVector = (centerShoulderPosition - centerHipPosition).normalized;
            var bodyForwardVector = Vector3.Cross(bodyRightVector, bodyUpVector).normalized;

            var bodyRotation = LookRotation(bodyForwardVector, bodyUpVector);

            _pelvisTarget.localRotation = bodyRotation;


            // --------------------------------------------------
            // Head Rotation
            // --------------------------------------------------

            var centerEyePosition = (_poseWorldLandmarks[(int)PoseLandmark.LeftEye].Position + _poseWorldLandmarks[(int)PoseLandmark.RightEye].Position) / 2f;
            var centerEarPosition = (_poseWorldLandmarks[(int)PoseLandmark.LeftEar].Position + _poseWorldLandmarks[(int)PoseLandmark.RightEar].Position) / 2f;

            var headRightVector = (_poseWorldLandmarks[(int)PoseLandmark.RightEar].Position - _poseWorldLandmarks[(int)PoseLandmark.LeftEar].Position).normalized;
            var headForwardVector = (centerEyePosition - centerEarPosition).normalized;
            var headUpVector = Vector3.Cross(headForwardVector, headRightVector).normalized;

            _headTarget.localRotation = LookRotation(headForwardVector, headUpVector);


            // --------------------------------------------------
            // Left Hand Rotation
            // --------------------------------------------------

            if (!_activeLeftHandLandmark)
            {
                var centerLeftHandPosition = (_poseWorldLandmarks[(int)PoseLandmark.LeftIndex].Position + _poseWorldLandmarks[(int)PoseLandmark.LeftPinky].Position) / 2f;
                var leftHandForwardVector = (centerLeftHandPosition - _poseWorldLandmarks[(int)PoseLandmark.LeftWrist].Position).normalized;
                var leftHandRightVector = (_poseWorldLandmarks[(int)PoseLandmark.LeftIndex].Position - _poseWorldLandmarks[(int)PoseLandmark.LeftPinky].Position).normalized;
                var leftHandUpVector = Vector3.Cross(leftHandForwardVector, leftHandRightVector).normalized;

                _leftHandTarget.localRotation = LookRotation(leftHandForwardVector, leftHandUpVector) * Quaternion.Euler(_handRotationOffset);
            }
            else
            {
                var leftHandWristToMiddle = _leftHandLandmarks[(int)HandLandmark.MiddleFingerMcp].Position - _leftHandLandmarks[(int)HandLandmark.Wrist].Position;
                var leftHandPinkyToIndex = _leftHandLandmarks[(int)HandLandmark.IndexFingerMcp].Position - _leftHandLandmarks[(int)HandLandmark.PinkyMcp].Position;
                var leftHandUpVector = Vector3.Cross(leftHandPinkyToIndex, leftHandWristToMiddle).normalized;
                var leftHandForwardVector = Vector3.Cross(leftHandUpVector, leftHandPinkyToIndex).normalized;

                _leftHandTarget.localRotation = LookRotation(leftHandForwardVector, -leftHandUpVector) * Quaternion.Euler(_handRotationOffset);
            }


            // --------------------------------------------------
            // Right Hand Rotation
            // --------------------------------------------------

            if (!_activeRightHandLandmark)
            {
                var centerRightHandPosition = (_poseWorldLandmarks[(int)PoseLandmark.RightIndex].Position + _poseWorldLandmarks[(int)PoseLandmark.RightPinky].Position) / 2f;
                var rightHandForwardVector = (centerRightHandPosition - _poseWorldLandmarks[(int)PoseLandmark.RightWrist].Position).normalized;
                var rightHandRightVector = (_poseWorldLandmarks[(int)PoseLandmark.RightIndex].Position - _poseWorldLandmarks[(int)PoseLandmark.RightPinky].Position).normalized;
                var rightHandUpVector = Vector3.Cross(-rightHandForwardVector, rightHandRightVector).normalized;

                _rightHandTarget.localRotation = LookRotation(-rightHandForwardVector, rightHandUpVector) * Quaternion.Euler(_handRotationOffset);
            }
            else
            {
                var rightHandWristToMiddle = _rightHandLandmarks[(int)HandLandmark.MiddleFingerMcp].Position - _rightHandLandmarks[(int)HandLandmark.Wrist].Position;
                var rightHandPinkyToIndex = _rightHandLandmarks[(int)HandLandmark.IndexFingerMcp].Position - _rightHandLandmarks[(int)HandLandmark.PinkyMcp].Position;
                var rightHandUpVector = Vector3.Cross(rightHandPinkyToIndex, rightHandWristToMiddle).normalized;
                var rightHandForwardVector = Vector3.Cross(rightHandUpVector, rightHandPinkyToIndex).normalized;

                _rightHandTarget.localRotation = LookRotation(-rightHandForwardVector, rightHandUpVector) * Quaternion.Euler(_handRotationOffset);
            }


            // --------------------------------------------------
            // Foot Rotation
            // --------------------------------------------------

            var leftFootForwardVector = (_poseWorldLandmarks[(int)PoseLandmark.LeftFootIndex].Position - _poseWorldLandmarks[(int)PoseLandmark.LeftHeel].Position).normalized;
            var rightFootForwardVector = (_poseWorldLandmarks[(int)PoseLandmark.RightFootIndex].Position - _poseWorldLandmarks[(int)PoseLandmark.RightHeel].Position).normalized;

            _leftFootTarget.localRotation = LookRotation(leftFootForwardVector, Vector3.up);
            _rightFootTarget.localRotation = LookRotation(rightFootForwardVector, Vector3.up);


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
            if (_activeLeftHandLandmark)
            {
                var lHandTransform = _animator.GetBoneTransform(HumanBodyBones.LeftHand);

                if (lHandTransform != null && _inverseRotations.TryGetValue(HumanBodyBones.LeftHand, out var invRot))
                {
                    var lHandPos = _leftHandLandmarks[(int)HandLandmark.Wrist].Position;
                    var lMidMcpPos = _leftHandLandmarks[(int)HandLandmark.MiddleFingerMcp].Position;
                    var lIndexMcpPos = _leftHandLandmarks[(int)HandLandmark.IndexFingerMcp].Position;
                    var lPinkyMcpPos = _leftHandLandmarks[(int)HandLandmark.PinkyMcp].Position;

                    var lHandUp = TriangleNormal(lHandPos, lPinkyMcpPos, lIndexMcpPos);
                    var lHandForward = (lHandPos - lMidMcpPos).normalized;

                    lHandTransform.rotation = LookRotation(lHandForward, lHandUp) * invRot;
                }

                var invLeftHandWorldRot = Quaternion.Inverse(lHandTransform.rotation);

                ComputeFingerRotation(HumanBodyBones.LeftThumbProximal, HandLandmark.ThumbCmc, invLeftHandWorldRot);

                ApplyFingerAngle(HumanBodyBones.LeftIndexProximal, HandLandmark.Wrist, HandLandmark.IndexFingerMcp, HandLandmark.IndexFingerPip);
                ApplyFingerAngle(HumanBodyBones.LeftIndexIntermediate, HandLandmark.IndexFingerMcp, HandLandmark.IndexFingerPip, HandLandmark.IndexFingerDip);
                ApplyFingerAngle(HumanBodyBones.LeftIndexDistal, HandLandmark.IndexFingerPip, HandLandmark.IndexFingerDip, HandLandmark.IndexFingerTip);

                ApplyFingerAngle(HumanBodyBones.LeftMiddleProximal, HandLandmark.Wrist, HandLandmark.MiddleFingerMcp, HandLandmark.MiddleFingerPip);
                ApplyFingerAngle(HumanBodyBones.LeftMiddleIntermediate, HandLandmark.MiddleFingerMcp, HandLandmark.MiddleFingerPip, HandLandmark.MiddleFingerDip);
                ApplyFingerAngle(HumanBodyBones.LeftMiddleDistal, HandLandmark.MiddleFingerPip, HandLandmark.MiddleFingerDip, HandLandmark.MiddleFingerTip);

                ApplyFingerAngle(HumanBodyBones.LeftRingProximal, HandLandmark.Wrist, HandLandmark.RingFingerMcp, HandLandmark.RingFingerPip);
                ApplyFingerAngle(HumanBodyBones.LeftRingIntermediate, HandLandmark.RingFingerMcp, HandLandmark.RingFingerPip, HandLandmark.RingFingerDip);
                ApplyFingerAngle(HumanBodyBones.LeftRingDistal, HandLandmark.RingFingerPip, HandLandmark.RingFingerDip, HandLandmark.RingFingerTip);

                ApplyFingerAngle(HumanBodyBones.LeftLittleProximal, HandLandmark.Wrist, HandLandmark.PinkyMcp, HandLandmark.PinkyPip);
                ApplyFingerAngle(HumanBodyBones.LeftLittleIntermediate, HandLandmark.PinkyMcp, HandLandmark.PinkyPip, HandLandmark.PinkyDip);
                ApplyFingerAngle(HumanBodyBones.LeftLittleDistal, HandLandmark.PinkyPip, HandLandmark.PinkyDip, HandLandmark.PinkyTip);
            }

            if (_activeRightHandLandmark)
            {
                var rHandTransform = _animator.GetBoneTransform(HumanBodyBones.RightHand);

                if (rHandTransform != null && _inverseRotations.TryGetValue(HumanBodyBones.RightHand, out var invRot))
                {
                    var rHandPos = _rightHandLandmarks[(int)HandLandmark.Wrist].Position;
                    var rMidMcpPos = _rightHandLandmarks[(int)HandLandmark.MiddleFingerMcp].Position;
                    var rIndexMcpPos = _rightHandLandmarks[(int)HandLandmark.IndexFingerMcp].Position;
                    var rPinkyMcpPos = _rightHandLandmarks[(int)HandLandmark.PinkyMcp].Position;

                    var rHandUp = TriangleNormal(rHandPos, rIndexMcpPos, rPinkyMcpPos);
                    var rHandForward = (rHandPos - rMidMcpPos).normalized;

                    rHandTransform.rotation = LookRotation(rHandForward, rHandUp) * invRot;
                }

                var invRightHandWorldRot = Quaternion.Inverse(rHandTransform.rotation);

                ComputeFingerRotation(HumanBodyBones.RightThumbProximal, HandLandmark.ThumbCmc, invRightHandWorldRot);

                ApplyFingerAngle(HumanBodyBones.RightIndexProximal, HandLandmark.Wrist, HandLandmark.IndexFingerMcp, HandLandmark.IndexFingerPip);
                ApplyFingerAngle(HumanBodyBones.RightIndexIntermediate, HandLandmark.IndexFingerMcp, HandLandmark.IndexFingerPip, HandLandmark.IndexFingerDip);
                ApplyFingerAngle(HumanBodyBones.RightIndexDistal, HandLandmark.IndexFingerPip, HandLandmark.IndexFingerDip, HandLandmark.IndexFingerTip);

                ApplyFingerAngle(HumanBodyBones.RightMiddleProximal, HandLandmark.Wrist, HandLandmark.MiddleFingerMcp, HandLandmark.MiddleFingerPip);
                ApplyFingerAngle(HumanBodyBones.RightMiddleIntermediate, HandLandmark.MiddleFingerMcp, HandLandmark.MiddleFingerPip, HandLandmark.MiddleFingerDip);
                ApplyFingerAngle(HumanBodyBones.RightMiddleDistal, HandLandmark.MiddleFingerPip, HandLandmark.MiddleFingerDip, HandLandmark.MiddleFingerTip);

                ApplyFingerAngle(HumanBodyBones.RightRingProximal, HandLandmark.Wrist, HandLandmark.RingFingerMcp, HandLandmark.RingFingerPip);
                ApplyFingerAngle(HumanBodyBones.RightRingIntermediate, HandLandmark.RingFingerMcp, HandLandmark.RingFingerPip, HandLandmark.RingFingerDip);
                ApplyFingerAngle(HumanBodyBones.RightRingDistal, HandLandmark.RingFingerPip, HandLandmark.RingFingerDip, HandLandmark.RingFingerTip);

                ApplyFingerAngle(HumanBodyBones.RightLittleProximal, HandLandmark.Wrist, HandLandmark.PinkyMcp, HandLandmark.PinkyPip);
                ApplyFingerAngle(HumanBodyBones.RightLittleIntermediate, HandLandmark.PinkyMcp, HandLandmark.PinkyPip, HandLandmark.PinkyDip);
                ApplyFingerAngle(HumanBodyBones.RightLittleDistal, HandLandmark.PinkyPip, HandLandmark.PinkyDip, HandLandmark.PinkyTip);
            }
        }

        private void ComputeFingerRotation(HumanBodyBones proximalBone, HandLandmark firstLandmark, Quaternion handWorldRotation)
        {
            for (int i = 0; i < 3; i++)
            {
                var bone = proximalBone + i;

                if (_initialLocalRotations.ContainsKey(bone))
                {
                    var transform = _animator.GetBoneTransform(bone);

                    if (transform == null) continue;

                    var isRightHand = bone.ToString().StartsWith("Right");

                    var landmarks = isRightHand ? _rightHandLandmarks : _leftHandLandmarks;

                    var worldTargetDirection = (landmarks[(int)firstLandmark + i + 1].Position - landmarks[(int)firstLandmark + i].Position).normalized;

                    if (worldTargetDirection == Vector3.zero) continue;

                    var deltaRotation = Quaternion.FromToRotation(_initialBoneDirections[bone], handWorldRotation * worldTargetDirection);

                    transform.localRotation = _initialLocalRotations[bone] * deltaRotation;

                    handWorldRotation *= Quaternion.Inverse(deltaRotation);
                }
            }
        }

        private void ApplyFingerAngle(HumanBodyBones bone, HandLandmark parentLandmark, HandLandmark jointLandmark, HandLandmark childLandmark)
        {
            var transform = _animator.GetBoneTransform(bone);

            if (transform == null) return;

            var isRightHand = bone.ToString().StartsWith("Right");

            var landmarks = isRightHand ? _rightHandLandmarks : _leftHandLandmarks;

            var angle = Mathf.Min(Vector3.Angle(
                landmarks[(int)jointLandmark].Position - landmarks[(int)parentLandmark].Position,
                landmarks[(int)childLandmark].Position - landmarks[(int)jointLandmark].Position), 90f);

            if (_initialLocalRotations.TryGetValue(bone, out Quaternion initRot))
            {
                transform.localRotation = initRot * Quaternion.Euler(0, 0, isRightHand ? -angle : angle);
            }
        }

        private void UpdateVRIK()
        {
            if (_autoWeight)
            {
                _VRIK.solver.leftLeg.positionWeight = _poseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility;
                _VRIK.solver.leftLeg.rotationWeight = _poseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility;
                _VRIK.solver.leftLeg.bendGoalWeight = _poseWorldLandmarks[(int)PoseLandmark.LeftKnee].Visibility;
                _VRIK.solver.rightLeg.positionWeight = _poseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility;
                _VRIK.solver.rightLeg.rotationWeight = _poseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility;
                _VRIK.solver.rightLeg.bendGoalWeight = _poseWorldLandmarks[(int)PoseLandmark.RightKnee].Visibility;
            }
            else
            {
                if (_poseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility > 0.5f &&
                    _poseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility > 0.5f)
                {
                    _VRIK.solver.locomotion.weight = 0f;
                }
                else
                {
                    _VRIK.solver.locomotion.weight = 1f;
                }

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
            if (_autoWeight)
            {
                _FBBIK.solver.leftFootEffector.positionWeight = _poseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility;
                _FBBIK.solver.leftFootEffector.rotationWeight = _poseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility;
                _FBBIK.solver.rightFootEffector.positionWeight = _poseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility;
                _FBBIK.solver.rightFootEffector.rotationWeight = _poseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility;
                _FBBIK.solver.leftLegChain.bendConstraint.weight = _poseWorldLandmarks[(int)PoseLandmark.LeftKnee].Visibility;
                _FBBIK.solver.rightLegChain.bendConstraint.weight = _poseWorldLandmarks[(int)PoseLandmark.RightKnee].Visibility;
                _FBBIK.solver.leftThighEffector.positionWeight = _poseWorldLandmarks[(int)PoseLandmark.LeftHip].Visibility;
                _FBBIK.solver.leftThighEffector.rotationWeight = _poseWorldLandmarks[(int)PoseLandmark.LeftHip].Visibility;
                _FBBIK.solver.rightThighEffector.positionWeight = _poseWorldLandmarks[(int)PoseLandmark.RightHip].Visibility;
                _FBBIK.solver.rightThighEffector.rotationWeight = _poseWorldLandmarks[(int)PoseLandmark.RightHip].Visibility;
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

        private void OnPoseLandmarks(NormalizedLandmarkList landmarkList)
        {
            _activePoseLandmark = landmarkList != null;

            if (landmarkList == null) return;

            var landmark = landmarkList.Landmark;

            if (landmark == null) return;

            for (int i = 0; i < landmark.Count; i++)
            {
                _poseLandmarks[i].Set(landmark[i]);

                _poseLandmarks[i].Position = Vector3.Scale(_poseLandmarks[i].Position, _landmarkScale);

                if (_enableKalmanFilter)
                {
                    _poseLandmarks[i].Position = _poseLandmarks[i].KalmanFilter.Update(_poseLandmarks[i].Position);
                }
            }
        }

        private void OnPoseWorldLandmarks(LandmarkList landmarkList)
        {
            if (landmarkList == null) return;

            var landmark = landmarkList.Landmark;

            if (landmark == null) return;

            for (int i = 0; i < landmark.Count; i++)
            {
                _poseWorldLandmarks[i].Set(landmark[i]);

                _poseWorldLandmarks[i].Position = Vector3.Scale(_poseWorldLandmarks[i].Position, _landmarkScale);

                if (_enableKalmanFilter)
                {
                    _poseWorldLandmarks[i].Position = _poseWorldLandmarks[i].KalmanFilter.Update(_poseWorldLandmarks[i].Position);
                }
            }
        }

        private void OnLeftHandLandmarks(NormalizedLandmarkList landmarkList)
        {
            _activeLeftHandLandmark = landmarkList != null;

            if (landmarkList == null) return;

            var landmark = landmarkList.Landmark;

            if (landmark == null) return;

            for (int i = 0; i < landmark.Count; i++)
            {
                _leftHandLandmarks[i].Set(landmark[i]);

                _leftHandLandmarks[i].Position = Vector3.Scale(_leftHandLandmarks[i].Position, _landmarkScale);

                if (_enableKalmanFilter)
                {
                    _leftHandLandmarks[i].Position = _leftHandLandmarks[i].KalmanFilter.Update(_leftHandLandmarks[i].Position);
                }
            }
        }

        private void OnRightHandLandmarks(NormalizedLandmarkList landmarkList)
        {
            _activeRightHandLandmark = landmarkList != null;

            if (landmarkList == null) return;

            var landmark = landmarkList.Landmark;

            if (landmark == null) return;

            for (int i = 0; i < landmark.Count; i++)
            {
                _rightHandLandmarks[i].Set(landmark[i]);

                _rightHandLandmarks[i].Position = Vector3.Scale(_rightHandLandmarks[i].Position, _landmarkScale);

                if (_enableKalmanFilter)
                {
                    _rightHandLandmarks[i].Position = _rightHandLandmarks[i].KalmanFilter.Update(_rightHandLandmarks[i].Position);
                }
            }
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

            v.Normalize();

            return v;
        }
    }

    public enum IKType
    {
        None,
        VRIK,
        FBBIK,
    }
}