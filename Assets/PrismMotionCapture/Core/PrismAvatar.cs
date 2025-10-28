using Mediapipe;
using RootMotion;
using RootMotion.FinalIK;
using System.Collections.Generic;
using UnityEngine;
using Color = UnityEngine.Color;

namespace PMC
{
    [AddComponentMenu("Prism Motion Capture/Prism Avatar")]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Animator))]
    public sealed class PrismAvatar : MonoBehaviour
    {
        // Fields

        [SerializeField] private PrismTracker _tracker;
        [SerializeField] private IKType _IKType = IKType.FBBIK;
        [SerializeField] private bool _enableMovement = true;
        [SerializeField] private bool _autoWeight = true;
        [SerializeField] private Vector3 _landmarkScale = new(1f, 1f, -1f);
        [SerializeField] private Vector3 _handRotationOffset = new(0f, 90f, 0f);
        [SerializeField] private Vector3 _leftHandRotationOffset = new(0f, 90f, 0f);
        [SerializeField] private bool _enableKalmanFilter = true;
        [SerializeField] private float _timeInterval = 0.45f;
        [SerializeField] private float _noise = 0.4f;
        [SerializeField] private bool _executeCalibration = false;

        private Animator _animator;

        private VRIK _VRIK;
        private VRIKRootController _rootController;

        private FullBodyBipedIK _FBBIK;
        private FBBIKHeadEffector _headEffector;

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

        private Landmark[] _poseLandmarks;
        private Landmark[] _poseWorldLandmarks;
        private Landmark[] _leftHandLandmarks;
        private Landmark[] _rightHandLandmarks;

        private Vector3 _baseScale;
        private Vector3 _basePosition;
        private Vector3 _pelvisBasePosition;

        private float _sittingHeight;
        private bool _activePoseLandmark;
        private bool _activeLeftHandLandmark;
        private bool _activeRightHandLandmark;

        private Dictionary<HumanBodyBones, Quaternion> _initialLocalRotations = new();


        private static readonly HumanBodyBones[] FingerBones = new HumanBodyBones[]
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
        };


        // Methods

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

            _poseLandmarks = new Landmark[(int)PoseLandmark.Count];
            _poseWorldLandmarks = new Landmark[(int)PoseLandmark.Count];
            _leftHandLandmarks = new Landmark[(int)HandLandmark.Count];
            _rightHandLandmarks = new Landmark[(int)HandLandmark.Count];

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

            _executeCalibration = false;
        }

        private void Update()
        {
            // Editor Only
            ExecuteCalibration();
        }

        private void OnDestroy()
        {
            if (_tracker != null)
            {
                _tracker.OnPoseLandmarks -= OnPoseLandmarks;
                _tracker.OnPoseWorldLandmarks -= OnPoseWorldLandmarks;
                _tracker.OnLeftHandLandmarks -= OnLeftHandLandmarks;
                _tracker.OnRightHandLandmarks += OnRightHandLandmarks;
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

        private void OnDrawGizmos()
        {
            if (!Application.isPlaying) return;

            if (_activeLeftHandLandmark)
            {
                var leftHandWristToMiddle = _leftHandLandmarks[(int)HandLandmark.MiddleFingerMcp].Position - _leftHandLandmarks[(int)HandLandmark.Wrist].Position;
                var leftHandPinkyToIndex = _leftHandLandmarks[(int)HandLandmark.IndexFingerMcp].Position - _leftHandLandmarks[(int)HandLandmark.PinkyMcp].Position;
                var leftHandUpVector = Vector3.Cross(leftHandPinkyToIndex, leftHandWristToMiddle).normalized;
                var leftHandForwardVector = Vector3.Cross(leftHandUpVector, leftHandPinkyToIndex).normalized;

                var rotation = LookRotation(leftHandForwardVector, -leftHandUpVector) * Quaternion.Euler(_handRotationOffset);

                var length = 0.25f;

                Gizmos.color = new Color(0f, 0f, 1f, 1f);
                Gizmos.DrawRay(_animator.GetBoneTransform(HumanBodyBones.LeftHand).position, rotation * (length * Vector3.forward));

                Gizmos.color = new Color(0f, 1f, 0f, 1f);
                Gizmos.DrawRay(_animator.GetBoneTransform(HumanBodyBones.LeftHand).position, rotation * (length * Vector3.up));

                Gizmos.color = new Color(1f, 0f, 0f, 1f);
                Gizmos.DrawRay(_animator.GetBoneTransform(HumanBodyBones.LeftHand).position, rotation * (length * Vector3.right));
            }
        }

        private void OnPreVRIK()
        {
            if (_activePoseLandmark)
            {
                UpdateTarget();

                UpdateVRIK();
            }
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

            _FBBIK.solver.IKPositionWeight = 1f;

            foreach (var bone in FingerBones)
            {
                var transform = _animator.GetBoneTransform(bone);

                if (transform == null) continue;

                _initialLocalRotations.Add(bone, transform.localRotation);
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
            if (_activeRightHandLandmark)
            {
                SetFingerRotationFromTo(HumanBodyBones.RightThumbProximal, HandLandmark.Wrist, HandLandmark.ThumbCmc, HandLandmark.ThumbMcp);
                SetFingerRotationFromTo(HumanBodyBones.RightThumbIntermediate, HandLandmark.ThumbCmc, HandLandmark.ThumbMcp, HandLandmark.ThumbIp);
                SetFingerRotationFromTo(HumanBodyBones.RightThumbDistal, HandLandmark.ThumbMcp, HandLandmark.ThumbIp, HandLandmark.ThumbTip);

                SetFingerRotationFromTo(HumanBodyBones.RightIndexProximal, HandLandmark.Wrist, HandLandmark.IndexFingerMcp, HandLandmark.IndexFingerPip);
                SetFingerRotationFromTo(HumanBodyBones.RightIndexIntermediate, HandLandmark.IndexFingerMcp, HandLandmark.IndexFingerPip, HandLandmark.IndexFingerDip);
                SetFingerRotationFromTo(HumanBodyBones.RightIndexDistal, HandLandmark.IndexFingerPip, HandLandmark.IndexFingerDip, HandLandmark.IndexFingerTip);

                SetFingerRotationFromTo(HumanBodyBones.RightMiddleProximal, HandLandmark.Wrist, HandLandmark.MiddleFingerMcp, HandLandmark.MiddleFingerPip);
                SetFingerRotationFromTo(HumanBodyBones.RightMiddleIntermediate, HandLandmark.MiddleFingerMcp, HandLandmark.MiddleFingerPip, HandLandmark.MiddleFingerDip);
                SetFingerRotationFromTo(HumanBodyBones.RightMiddleDistal, HandLandmark.MiddleFingerPip, HandLandmark.MiddleFingerDip, HandLandmark.MiddleFingerTip);

                SetFingerRotationFromTo(HumanBodyBones.RightRingProximal, HandLandmark.Wrist, HandLandmark.RingFingerMcp, HandLandmark.RingFingerPip);
                SetFingerRotationFromTo(HumanBodyBones.RightRingIntermediate, HandLandmark.RingFingerMcp, HandLandmark.RingFingerPip, HandLandmark.RingFingerDip);
                SetFingerRotationFromTo(HumanBodyBones.RightRingDistal, HandLandmark.RingFingerPip, HandLandmark.RingFingerDip, HandLandmark.RingFingerTip);

                SetFingerRotationFromTo(HumanBodyBones.RightLittleProximal, HandLandmark.Wrist, HandLandmark.PinkyMcp, HandLandmark.PinkyPip);
                SetFingerRotationFromTo(HumanBodyBones.RightLittleIntermediate, HandLandmark.PinkyMcp, HandLandmark.PinkyPip, HandLandmark.PinkyDip);
                SetFingerRotationFromTo(HumanBodyBones.RightLittleDistal, HandLandmark.PinkyPip, HandLandmark.PinkyDip, HandLandmark.PinkyTip);
            }
        }

        private void SetFingerRotationFromTo(HumanBodyBones boneId, HandLandmark parentStartId, HandLandmark childStartId, HandLandmark childEndId)
        {
            var bone = _animator.GetBoneTransform(boneId);

            if (bone == null || !_initialLocalRotations.ContainsKey(boneId)) return;

            var isLeftHand = boneId.ToString().StartsWith("Left");
            var landmarks = isLeftHand ? _leftHandLandmarks : _rightHandLandmarks;

            // ----------------------------------------------------------------------
            // 1. 方向ベクトルの計算
            // ----------------------------------------------------------------------
            // a) Forwardベクトル（回転後に指が向くべき方向）: 親の関節から子の関節へ
            var forwardDirection = (landmarks[(int)childStartId].Position - landmarks[(int)parentStartId].Position).normalized;

            // b) Upベクトル（ねじれを定義する基準となる方向）
            // 手の平面からUpベクトルを計算するために、手首と指の付け根（MCP）のランドマークを使用します。

            Vector3 upVector;
            if (isLeftHand)
            {
                // 左手の甲の平面を基準にする
                var wristToPinky = landmarks[(int)HandLandmark.PinkyMcp].Position - landmarks[(int)HandLandmark.Wrist].Position;
                var wristToIndex = landmarks[(int)HandLandmark.IndexFingerMcp].Position - landmarks[(int)HandLandmark.Wrist].Position;

                // 外積で平面に垂直なベクトルを計算 (左手のUpは通常、手のひら側を向く)
                upVector = Vector3.Cross(wristToIndex, wristToPinky).normalized;
            }
            else
            {
                // 右手の甲の平面を基準にする
                var wristToPinky = landmarks[(int)HandLandmark.PinkyMcp].Position - landmarks[(int)HandLandmark.Wrist].Position;
                var wristToIndex = landmarks[(int)HandLandmark.IndexFingerMcp].Position - landmarks[(int)HandLandmark.Wrist].Position;

                // 外積で平面に垂直なベクトルを計算 (右手のUpは通常、手の甲側を向く)
                upVector = Vector3.Cross(wristToPinky, wristToIndex).normalized;
            }

            if (forwardDirection == Vector3.zero || upVector == Vector3.zero) return;


            // ----------------------------------------------------------------------
            // 2. LookRotationで目標回転を計算
            // ----------------------------------------------------------------------
            // forwardDirection: 指が向くべき方向（回転後のZ軸の目標）
            // upVector: 指の曲げ方向（回転後のY軸の目標、またはその近く）
            var targetRotation = LookRotation(forwardDirection, upVector);


            // ----------------------------------------------------------------------
            // 3. 初期回転を基準にローカル回転を適用
            // ----------------------------------------------------------------------
            // targetRotation (World Space)をローカル回転に変換するために、
            // 親の回転の逆数を乗算して相対的な回転を取り出す必要がある場合がありますが、
            // ここでは単純に初期回転を基準にした相対回転を適用する一般的なアプローチを取ります。

            // Quaternion.Inverse(_animator.GetBoneTransform(HumanBodyBones.ParentOfThisBone).rotation) * targetRotation * _initialLocalRotations[boneId];
            // のような複雑な計算が必要な場合もありますが、
            // Simple IKでは、初期回転を基準にしたFromTo/LookRotationで十分なことも多いです。

            // より安定させるため、FromToRotationのロジックに近づけて、
            // 「親ボーンの初期回転」からの相対的なねじれを修正した回転として適用します。
            // ※ここでは既存の FromToRotation の実装を LookRotation で置き換えるシンプルな対応とします。

            // targetRotation: ワールド空間で目指すべき指の姿勢
            // bone.parent.rotation: 親ボーンの現在のワールド回転

            // ワールド姿勢をローカル姿勢に変換
            Quaternion localTargetRotation = Quaternion.Inverse(bone.parent.rotation) * targetRotation;

            // 初期ローカル回転にターゲットローカル回転を乗算
            // ただし、LookRotationは絶対的なワールド回転を与えるため、
            // ここではターゲットローカル回転を直接適用します。
            bone.localRotation = localTargetRotation;
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

        private void ExecuteCalibration()
        {
            if (_executeCalibration)
            {
                var distance = Vector3.Distance(Vector3.zero, _poseWorldLandmarks[(int)PoseLandmark.Nose].Position);

                _landmarkScale *= _sittingHeight / distance;

                _executeCalibration = false;
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
    }

    public enum IKType
    {
        None,
        VRIK,
        FBBIK,
    }
}