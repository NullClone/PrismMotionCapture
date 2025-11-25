using PMC.Utilities;
using RootMotion;
using RootMotion.FinalIK;
using System.Collections.Generic;
using UnityEngine;
using UniVRM10;

namespace PMC
{
    [AddComponentMenu("Prism Motion Capture/Prism Avatar")]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Animator))]
    public sealed class PrismAvatar : MonoBehaviour
    {
        // Fields

        public PrismTracker Tracker;
        private Vrm10Instance _vrm;
        private Animator _animator;
        private VRIK _VRIK;
        private FullBodyBipedIK _FBBIK;
        private TwistRelaxer _twistRelaxer;
        private FBBIKHeadEffector _headEffector;
        public IKType IKType = IKType.VRIK;

        public bool EnableTwistRelaxer = true;
        public bool EnableMovement = true;
        public bool AutoWeight = true;
        public float WeightSmoothingSpeed = 10f;
        public float TargetRotationSmoothSpeed = 20f;

        public bool EnableHandTracking = true;
        public float FingerResetSpeed = 5f;
        public Vector3 HandRotationOffset = new(0f, 90f, 0f);

        public bool EnableBlink = true;
        public bool AutoBlink = false;
        public bool LinkBlinks = true;
        public bool AllowWinking = true;
        [Range(0f, 1f)] public float EyeClosedThreshold = 0.65f;
        [Range(0f, 1f)] public float EyeOpenedThreshold = 0.15f;
        [Range(0f, 1f)] public float SmartWinkThreshold = 0.85f;
        [Range(0f, 1f)] public float BlinkSmoothing = 0.75f;

        public bool EnableGaze = true;
        [Range(0f, 1f)] public float GazeSmoothing = 0.6f;
        [Range(0f, 1f)] public float GazeStrength = 1f;

        public bool EnableMouth = true;
        [Range(0f, 2f)] public float MouthOpenSensitivity = 1f;
        [Range(0f, 2f)] public float MouthShapeSensitivity = 1f;
        [Range(0f, 1f)] public float MouthSmoothing = 0.6f;

        private bool _initialized = false;

        private Transform _root;
        private Transform _headTarget;
        private Transform _pelvisTarget;
        private Transform _chestGoal;
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

        private Vector3 _basePosition;
        private Vector3 _baseTrackingPosition;
        private Vector3 _pelvisBasePosition;
        private Quaternion _inverseLeftHandRotation;
        private Quaternion _inverseRightHandRotation;

        private float _currentLeftLegWeight = 0f;
        private float _currentRightLegWeight = 0f;
        private float _currentLeftBendWeight = 0f;
        private float _currentRightBendWeight = 0f;

        private float _currentBlinkLeft;
        private float _currentBlinkRight;
        private float _currentLookUp;
        private float _currentLookDown;
        private float _currentLookLeft;
        private float _currentLookRight;

        private float _lastBlinkLeft;
        private float _lastBlinkRight;
        private float _lastLookUp;
        private float _lastLookDown;
        private float _lastLookLeft;
        private float _lastLookRight;

        private bool _blinking;
        private float _blinkTimer;
        private float _nextBlink = 4f;
        private float _blinkState;


        private readonly TimeInterpolate _blinkInterpolate = new();
        private readonly TimeInterpolate _gazeInterpolate = new();
        private readonly TimeInterpolate _mouthInterpolate = new();

        private readonly Dictionary<HumanBodyBones, Quaternion> _initialLocalRotations = new();
        private readonly Dictionary<HumanBodyBones, Vector3> _initialBoneDirections = new();


        // Methods

        private void Awake()
        {
            if (!enabled || Tracker == null) return;

            _vrm = gameObject.GetComponent<Vrm10Instance>();

            if (_vrm == null) return;

            _animator = gameObject.GetComponent<Animator>();

            if (IKType == IKType.VRIK)
            {
                if (!gameObject.TryGetComponent(out _VRIK))
                {
                    _VRIK = gameObject.AddComponent<VRIK>();
                }
            }

            if (IKType == IKType.FBBIK)
            {
                if (!gameObject.TryGetComponent(out _FBBIK))
                {
                    _FBBIK = gameObject.AddComponent<FullBodyBipedIK>();
                }
            }

            if (EnableTwistRelaxer)
            {
                if (!gameObject.TryGetComponent(out _twistRelaxer))
                {
                    _twistRelaxer = gameObject.AddComponent<TwistRelaxer>();
                }
            }

            if (IKType == IKType.VRIK)
            {
                _VRIK.solver.OnPreUpdate += OnPreVRIK;
            }

            if (IKType == IKType.FBBIK)
            {
                _FBBIK.solver.OnPreUpdate += OnPreFBBIK;
            }

            _initialized = true;
        }

        private void Start()
        {
            if (!_initialized) return;

            CreateTarget();

            InitializeFinger();

            if (IKType == IKType.VRIK)
            {
                InitializeVRIK();
            }

            if (IKType == IKType.FBBIK)
            {
                InitializeFBBIK();
            }
        }

        private void OnDestroy()
        {
            if (IKType == IKType.VRIK && _VRIK)
            {
                _VRIK.solver.OnPreUpdate -= OnPreVRIK;
            }

            if (IKType == IKType.FBBIK && _FBBIK)
            {
                _FBBIK.solver.OnPreUpdate -= OnPreFBBIK;
            }
        }

        private void OnPreVRIK()
        {
            UpdateMovement();

            //var pelvisTargetRight = Quaternion.Inverse(_pelvisTarget.rotation) * _VRIK.references.root.right;
            //_VRIK.references.pelvis.position = Vector3.Lerp(_VRIK.references.pelvis.position, _pelvisTarget.position, _VRIK.solver.spine.pelvisPositionWeight);
            //_VRIK.references.pelvis.rotation = Quaternion.Slerp(_VRIK.references.pelvis.rotation, _pelvisTarget.rotation, _VRIK.solver.spine.pelvisRotationWeight);

            UpdateTarget();
            UpdateVRIK();
            UpdateFinger();

            if (_vrm)
            {
                UpdateBlink();
                UpdateGaze();
                UpdateMouth();
            }
        }

        private void OnPreFBBIK()
        {
            UpdateMovement();
            UpdateTarget();
            UpdateFBBIK();
            UpdateFinger();

            if (_vrm)
            {
                UpdateBlink();
                UpdateGaze();
                UpdateMouth();
            }
        }

        private void CreateTarget()
        {
            _root = new GameObject($"{name} (Tracking Space)").transform;
            _headTarget = new GameObject("Head Target").transform;
            _pelvisTarget = new GameObject("Pelvis Target").transform;
            _chestGoal = new GameObject("Chest Goal").transform;
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
            _chestGoal.parent = _root;
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

                _inverseLeftHandRotation = Quaternion.Inverse(UnityUtils.LookRotation(forward, leftHandTransform.up)) * leftHandTransform.rotation;
            }

            if (rightHandTransform != null && rMidProx != null)
            {
                _initialLocalRotations.Add(HumanBodyBones.RightHand, rightHandTransform.localRotation);

                var forward = (rightHandTransform.position - rMidProx.position).normalized;

                _inverseRightHandRotation = Quaternion.Inverse(UnityUtils.LookRotation(forward, rightHandTransform.up)) * rightHandTransform.rotation;
            }
        }

        private void InitializeVRIK()
        {
            _VRIK.AutoDetectReferences();

            _VRIK.solver.FixTransforms();

            _VRIK.solver.IKPositionWeight = 0f;

            _basePosition = _VRIK.references.root.position;
            _pelvisBasePosition = _VRIK.references.pelvis.localPosition;

            _root.position = _VRIK.references.pelvis.position;
            _headTarget.position = _VRIK.references.head.position;
            _pelvisTarget.position = _VRIK.references.pelvis.position;
            _chestGoal.position = _VRIK.references.chest.position;
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
            _VRIK.solver.spine.maintainPelvisPosition = 0.2f;
            _VRIK.solver.plantFeet = false;

            _VRIK.solver.spine.headTarget = _headTarget;
            _VRIK.solver.spine.pelvisTarget = _pelvisTarget;
            _VRIK.solver.spine.chestGoal = _chestGoal;
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
            _VRIK.solver.spine.pelvisPositionWeight = 1f;
            _VRIK.solver.spine.pelvisRotationWeight = 1f;
            _VRIK.solver.spine.chestGoalWeight = 1f;
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

            if (EnableTwistRelaxer)
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
            _pelvisBasePosition = _FBBIK.references.pelvis.localPosition;

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

            if (EnableTwistRelaxer)
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

        private void UpdateMovement()
        {
            if (Tracker.ActivePoseWorldLandmark)
            {
                for (int i = 0; i < PrismTracker.PoseLandmarkCount; i++)
                {
                    Tracker.PoseWorldLandmarks[i].Position = Tracker.LocalAvatarSpacePoints[i];
                }
            }

            if (EnableMovement)
            {
                var pos = Tracker.GlobalAvatarPosition;

                pos.y *= -1f;
                pos.z *= -1f;

                if (Tracker.GlobalAvatarPosition != Vector3.zero && _baseTrackingPosition == Vector3.zero)
                {
                    _baseTrackingPosition = pos;

                    _basePosition.y += _pelvisBasePosition.y;
                }

                _root.position = pos - _baseTrackingPosition + _basePosition;
                _root.rotation = Tracker.GlobalAvatarRotation;

                if (IKType == IKType.VRIK)
                {
                    _VRIK.references.root.position = _root.position - _pelvisBasePosition;
                }
                else
                {
                    _FBBIK.references.root.position = _root.position - _pelvisBasePosition;
                }
            }
        }

        private void UpdateTarget()
        {
            if (!Tracker.ActivePoseWorldLandmark) return;

            var rotDelta = Time.deltaTime * TargetRotationSmoothSpeed;

            // --------------------------------------------------
            // Body Rotation (Stabilized version)
            // --------------------------------------------------

            var centerHipPosition = (Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftHip].Position + Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightHip].Position) / 2f;
            var centerShoulderPosition = (Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftShoulder].Position + Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightShoulder].Position) / 2f;

            var bodyRightVector = (Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightHip].Position - Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftHip].Position).normalized;
            var bodyUpVector = (centerShoulderPosition - centerHipPosition).normalized;
            var bodyForwardVector = Vector3.Cross(bodyRightVector, bodyUpVector).normalized;

            var targetPelvisRotation = UnityUtils.LookRotation(bodyForwardVector, bodyUpVector);
            _pelvisTarget.localRotation = Quaternion.Slerp(_pelvisTarget.localRotation, targetPelvisRotation, rotDelta);


            // --------------------------------------------------
            // Head Rotation
            // --------------------------------------------------

            var centerEyePosition = (Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftEye].Position + Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightEye].Position) / 2f;
            var centerEarPosition = (Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftEar].Position + Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightEar].Position) / 2f;

            var headRightVector = (Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightEar].Position - Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftEar].Position).normalized;
            var headForwardVector = (centerEyePosition - centerEarPosition).normalized;
            var headUpVector = Vector3.Cross(headForwardVector, headRightVector).normalized;

            var targetHeadRotation = UnityUtils.LookRotation(headForwardVector, headUpVector);
            _headTarget.localRotation = Quaternion.Slerp(_headTarget.localRotation, targetHeadRotation, rotDelta);


            // --------------------------------------------------
            // Chest
            // --------------------------------------------------

            var leftShoulderPos = Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftShoulder].Position;
            var rightShoulderPos = Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightShoulder].Position;
            var chestPos = (leftShoulderPos + rightShoulderPos) * 0.5f;

            _chestGoal.localPosition = chestPos;


            // --------------------------------------------------
            // Left Hand Rotation
            // --------------------------------------------------

            Quaternion targetLeftHandRotation;

            if (!Tracker.ActiveLeftHandWorldLandmark || !EnableHandTracking)
            {
                var centerLeftHandPosition = (Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftIndex].Position + Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftPinky].Position) / 2f;
                var leftHandForwardVector = (centerLeftHandPosition - Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftWrist].Position).normalized;
                var leftHandRightVector = (Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftIndex].Position - Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftPinky].Position).normalized;
                var leftHandUpVector = Vector3.Cross(leftHandForwardVector, leftHandRightVector).normalized;

                targetLeftHandRotation = UnityUtils.LookRotation(leftHandForwardVector, leftHandUpVector) * Quaternion.Euler(HandRotationOffset);
            }
            else
            {
                var leftHandWristToMiddle = Tracker.LeftHandWorldLandmarks[(int)HandLandmark.MiddleFingerMcp].Position - Tracker.LeftHandWorldLandmarks[(int)HandLandmark.Wrist].Position;
                var leftHandPinkyToIndex = Tracker.LeftHandWorldLandmarks[(int)HandLandmark.IndexFingerMcp].Position - Tracker.LeftHandWorldLandmarks[(int)HandLandmark.PinkyMcp].Position;
                var leftHandUpVector = Vector3.Cross(leftHandPinkyToIndex, leftHandWristToMiddle).normalized;
                var leftHandForwardVector = Vector3.Cross(leftHandUpVector, leftHandPinkyToIndex).normalized;

                targetLeftHandRotation = UnityUtils.LookRotation(leftHandForwardVector, -leftHandUpVector) * Quaternion.Euler(HandRotationOffset);
            }

            _leftHandTarget.localRotation = Quaternion.Slerp(_leftHandTarget.localRotation, targetLeftHandRotation, rotDelta);


            // --------------------------------------------------
            // Right Hand Rotation
            // --------------------------------------------------

            Quaternion targetRightHandRotation;

            if (!Tracker.ActiveRightHandWorldLandmark || !EnableHandTracking)
            {
                var centerRightHandPosition = (Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightIndex].Position + Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightPinky].Position) / 2f;
                var rightHandForwardVector = (centerRightHandPosition - Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightWrist].Position).normalized;
                var rightHandRightVector = (Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightIndex].Position - Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightPinky].Position).normalized;
                var rightHandUpVector = Vector3.Cross(-rightHandForwardVector, rightHandRightVector).normalized;

                targetRightHandRotation = UnityUtils.LookRotation(-rightHandForwardVector, rightHandUpVector) * Quaternion.Euler(HandRotationOffset);
            }
            else
            {
                var rightHandWristToMiddle = Tracker.RightHandWorldLandmarks[(int)HandLandmark.MiddleFingerMcp].Position - Tracker.RightHandWorldLandmarks[(int)HandLandmark.Wrist].Position;
                var rightHandPinkyToIndex = Tracker.RightHandWorldLandmarks[(int)HandLandmark.IndexFingerMcp].Position - Tracker.RightHandWorldLandmarks[(int)HandLandmark.PinkyMcp].Position;
                var rightHandUpVector = Vector3.Cross(rightHandPinkyToIndex, rightHandWristToMiddle).normalized;
                var rightHandForwardVector = Vector3.Cross(rightHandUpVector, rightHandPinkyToIndex).normalized;

                targetRightHandRotation = UnityUtils.LookRotation(-rightHandForwardVector, rightHandUpVector) * Quaternion.Euler(HandRotationOffset);
            }

            _rightHandTarget.localRotation = Quaternion.Slerp(_rightHandTarget.localRotation, targetRightHandRotation, rotDelta);


            // --------------------------------------------------
            // Foot Rotation
            // --------------------------------------------------

            var leftFootForwardVector = (Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftFootIndex].Position - Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftHeel].Position).normalized;
            var rightFootForwardVector = (Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightFootIndex].Position - Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightHeel].Position).normalized;

            var targetLeftFootRotation = UnityUtils.LookRotation(leftFootForwardVector, Vector3.up);
            var targetRightFootRotation = UnityUtils.LookRotation(rightFootForwardVector, Vector3.up);

            _leftFootTarget.localRotation = Quaternion.Slerp(_leftFootTarget.localRotation, targetLeftFootRotation, rotDelta);
            _rightFootTarget.localRotation = Quaternion.Slerp(_rightFootTarget.localRotation, targetRightFootRotation, rotDelta);


            // --------------------------------------------------
            // Position
            // --------------------------------------------------

            _headTarget.localPosition = Tracker.PoseWorldLandmarks[(int)PoseLandmark.Nose].Position;

            _leftHandTarget.localPosition = Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftWrist].Position;
            _rightHandTarget.localPosition = Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightWrist].Position;

            _leftArmBendGoal.localPosition = Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftElbow].Position;
            _rightArmBendGoal.localPosition = Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightElbow].Position;

            _leftShoulderTarget.localPosition = Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftShoulder].Position;
            _rightShoulderTarget.localPosition = Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightShoulder].Position;

            _leftFootTarget.localPosition = Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftHeel].Position;
            _rightFootTarget.localPosition = Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightHeel].Position;

            _leftLegBendGoal.localPosition = Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftKnee].Position;
            _rightLegBendGoal.localPosition = Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightKnee].Position;

            _leftThighTarget.localPosition = Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftHip].Position;
            _rightThighTarget.localPosition = Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightHip].Position;
        }

        private void UpdateVRIK()
        {
            if (!Tracker.ActivePoseWorldLandmark) return;

            if (AutoWeight)
            {
                var targetLeftLegWeight = Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility ?? 0f;
                var targetRightLegWeight = Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility ?? 0f;
                var targetLeftBendWeight = (Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftKnee].Visibility * Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility) ?? 0f;
                var targetRightBendWeight = (Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightKnee].Visibility * Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility) ?? 0f;

                var lerpDelta = Time.deltaTime * WeightSmoothingSpeed;

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
                if (Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility > 0.5f)
                {
                    _VRIK.solver.leftLeg.positionWeight = 1f;
                    _VRIK.solver.leftLeg.rotationWeight = 1f;
                }
                else
                {
                    _VRIK.solver.leftLeg.positionWeight = 0f;
                    _VRIK.solver.leftLeg.rotationWeight = 0f;
                }

                if (Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftKnee].Visibility > 0.5f)
                {
                    _VRIK.solver.leftLeg.bendGoalWeight = 1f;
                }
                else
                {
                    _VRIK.solver.leftLeg.bendGoalWeight = 0f;
                }

                if (Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility > 0.5f)
                {
                    _VRIK.solver.rightLeg.positionWeight = 1f;
                    _VRIK.solver.rightLeg.rotationWeight = 1f;
                }
                else
                {
                    _VRIK.solver.rightLeg.positionWeight = 0f;
                    _VRIK.solver.rightLeg.rotationWeight = 0f;
                }

                if (Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightKnee].Visibility > 0.5f)
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
            if (!Tracker.ActivePoseWorldLandmark) return;

            if (AutoWeight)
            {
                _FBBIK.solver.leftFootEffector.positionWeight = Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility ?? 0f;
                _FBBIK.solver.leftFootEffector.rotationWeight = Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility ?? 0f;
                _FBBIK.solver.rightFootEffector.positionWeight = Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility ?? 0f;
                _FBBIK.solver.rightFootEffector.rotationWeight = Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility ?? 0f;
                _FBBIK.solver.leftLegChain.bendConstraint.weight = Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftKnee].Visibility ?? 0f;
                _FBBIK.solver.rightLegChain.bendConstraint.weight = Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightKnee].Visibility ?? 0f;
                _FBBIK.solver.leftThighEffector.positionWeight = Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftHip].Visibility ?? 0f;
                _FBBIK.solver.leftThighEffector.rotationWeight = Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftHip].Visibility ?? 0f;
                _FBBIK.solver.rightThighEffector.positionWeight = Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightHip].Visibility ?? 0f;
                _FBBIK.solver.rightThighEffector.rotationWeight = Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightHip].Visibility ?? 0f;
            }
            else
            {
                if (Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftHeel].Visibility > 0.5f)
                {
                    _FBBIK.solver.leftFootEffector.positionWeight = 1f;
                    _FBBIK.solver.leftFootEffector.rotationWeight = 1f;
                }
                else
                {
                    _FBBIK.solver.leftFootEffector.positionWeight = 0f;
                    _FBBIK.solver.leftFootEffector.rotationWeight = 0f;
                }

                if (Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightHeel].Visibility > 0.5f)
                {
                    _FBBIK.solver.rightFootEffector.positionWeight = 1f;
                    _FBBIK.solver.rightFootEffector.rotationWeight = 1f;
                }
                else
                {
                    _FBBIK.solver.rightFootEffector.positionWeight = 0f;
                    _FBBIK.solver.rightFootEffector.rotationWeight = 0f;
                }

                if (Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftKnee].Visibility > 0.5f)
                {
                    _FBBIK.solver.leftLegChain.bendConstraint.weight = 1f;
                }
                else
                {
                    _FBBIK.solver.leftLegChain.bendConstraint.weight = 0f;
                }

                if (Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightKnee].Visibility > 0.5f)
                {
                    _FBBIK.solver.rightLegChain.bendConstraint.weight = 1f;
                }
                else
                {
                    _FBBIK.solver.rightLegChain.bendConstraint.weight = 0f;
                }

                if (Tracker.PoseWorldLandmarks[(int)PoseLandmark.LeftHip].Visibility > 0.5f)
                {
                    _FBBIK.solver.leftThighEffector.positionWeight = 1f;
                    _FBBIK.solver.leftThighEffector.rotationWeight = 1f;
                }
                else
                {
                    _FBBIK.solver.leftThighEffector.positionWeight = 0f;
                    _FBBIK.solver.leftThighEffector.rotationWeight = 0f;
                }

                if (Tracker.PoseWorldLandmarks[(int)PoseLandmark.RightHip].Visibility > 0.5f)
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

        private void UpdateFinger()
        {
            if (!EnableHandTracking) return;

            if (Tracker.ActiveLeftHandWorldLandmark)
            {
                var lHandTransform = _animator.GetBoneTransform(HumanBodyBones.LeftHand);

                var lHandUp = UnityUtils.TriangleNormal(
                                        Tracker.LeftHandWorldLandmarks[(int)HandLandmark.Wrist].Position,
                                        Tracker.LeftHandWorldLandmarks[(int)HandLandmark.PinkyMcp].Position,
                                        Tracker.LeftHandWorldLandmarks[(int)HandLandmark.IndexFingerMcp].Position);
                var lHandForward = (Tracker.LeftHandWorldLandmarks[(int)HandLandmark.Wrist].Position - Tracker.LeftHandWorldLandmarks[(int)HandLandmark.MiddleFingerMcp].Position).normalized;

                lHandTransform.rotation = UnityUtils.LookRotation(lHandForward, lHandUp) * _inverseLeftHandRotation;

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

            if (Tracker.ActiveRightHandWorldLandmark)
            {
                var rHandTransform = _animator.GetBoneTransform(HumanBodyBones.RightHand);

                var rHandUp = UnityUtils.TriangleNormal(
                                        Tracker.RightHandWorldLandmarks[(int)HandLandmark.Wrist].Position,
                                        Tracker.RightHandWorldLandmarks[(int)HandLandmark.IndexFingerMcp].Position,
                                        Tracker.RightHandWorldLandmarks[(int)HandLandmark.PinkyMcp].Position);
                var rHandForward = (Tracker.RightHandWorldLandmarks[(int)HandLandmark.Wrist].Position - Tracker.RightHandWorldLandmarks[(int)HandLandmark.MiddleFingerMcp].Position).normalized;

                rHandTransform.rotation = UnityUtils.LookRotation(rHandForward, rHandUp) * _inverseRightHandRotation;

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

        private void UpdateBlink()
        {
            if (!EnableBlink || !Tracker.ActiveFaceBlendShapes) return;

            if (AutoBlink)
            {
                if (_blinking)
                {
                    _blinkState += Time.deltaTime * 20f;

                    if (_blinkState >= 2.0)
                    {
                        _blinking = false;
                        _blinkState = 0.0f;
                        _blinkTimer = Time.time;
                        _nextBlink = Random.Range(1f, 4f);
                    }
                }
                else if (Time.time - _blinkTimer > _nextBlink)
                {
                    _blinking = true;
                }

                var weight = _blinkState <= 1.0 ? _blinkState : 2f - _blinkState;

                _vrm.Runtime.Expression.SetWeight(ExpressionKey.Blink, weight);
            }
            else
            {
                var targetLeft = CalculateBlinkValueFromARKit(Tracker.FaceBlendShapes[(int)FaceBlendShapes.EyeBlinkLeft], EyeOpenedThreshold, EyeClosedThreshold);
                var targetRight = CalculateBlinkValueFromARKit(Tracker.FaceBlendShapes[(int)FaceBlendShapes.EyeBlinkRight], EyeOpenedThreshold, EyeClosedThreshold);

                var t = _blinkInterpolate.Interpolate();
                var smoothedLeft = Mathf.Lerp(_lastBlinkLeft, _currentBlinkLeft, t);
                var smoothedRight = Mathf.Lerp(_lastBlinkRight, _currentBlinkRight, t);

                if (LinkBlinks && !AllowWinking)
                {
                    var weight = Mathf.Max(smoothedLeft, smoothedRight);

                    _vrm.Runtime.Expression.SetWeight(ExpressionKey.Blink, weight);
                }
                else
                {
                    _vrm.Runtime.Expression.SetWeight(ExpressionKey.BlinkLeft, smoothedLeft);
                    _vrm.Runtime.Expression.SetWeight(ExpressionKey.BlinkRight, smoothedRight);
                }

                _blinkInterpolate.UpdateTime(Time.timeAsDouble);

                if (LinkBlinks && AllowWinking && Mathf.Abs(targetRight - targetLeft) < SmartWinkThreshold)
                {
                    var maxBlink = Mathf.Max(targetLeft, targetRight);

                    targetLeft = maxBlink;
                    targetRight = maxBlink;
                }

                _lastBlinkLeft = smoothedLeft;
                _lastBlinkRight = smoothedRight;

                _currentBlinkLeft = Mathf.Lerp(_currentBlinkLeft, targetLeft, 1f - BlinkSmoothing);
                _currentBlinkRight = Mathf.Lerp(_currentBlinkRight, targetRight, 1f - BlinkSmoothing);
            }
        }

        private void UpdateGaze()
        {
            if (!EnableGaze || !Tracker.ActiveFaceBlendShapes) return;

            var targetLookUp = (Tracker.FaceBlendShapes[(int)FaceBlendShapes.EyeLookUpLeft] + Tracker.FaceBlendShapes[(int)FaceBlendShapes.EyeLookUpRight]) / 2f;
            var targetLookDown = (Tracker.FaceBlendShapes[(int)FaceBlendShapes.EyeLookDownLeft] + Tracker.FaceBlendShapes[(int)FaceBlendShapes.EyeLookDownRight]) / 2f;
            var targetLookLeft = (Tracker.FaceBlendShapes[(int)FaceBlendShapes.EyeLookInLeft] + Tracker.FaceBlendShapes[(int)FaceBlendShapes.EyeLookOutRight]) / 2f;
            var targetLookRight = (Tracker.FaceBlendShapes[(int)FaceBlendShapes.EyeLookOutLeft] + Tracker.FaceBlendShapes[(int)FaceBlendShapes.EyeLookInRight]) / 2f;

            var t = _gazeInterpolate.Interpolate();

            var smoothedLookUp = Mathf.Lerp(_lastLookUp, targetLookUp, 1f - GazeSmoothing);
            var smoothedLookDown = Mathf.Lerp(_lastLookDown, targetLookDown, 1f - GazeSmoothing);
            var smoothedLookLeft = Mathf.Lerp(_lastLookLeft, targetLookLeft, 1f - GazeSmoothing);
            var smoothedLookRight = Mathf.Lerp(_lastLookRight, targetLookRight, 1f - GazeSmoothing);

            _lastLookUp = smoothedLookUp;
            _lastLookDown = smoothedLookDown;
            _lastLookLeft = smoothedLookLeft;
            _lastLookRight = smoothedLookRight;

            _currentLookUp = smoothedLookUp;
            _currentLookDown = smoothedLookDown;
            _currentLookLeft = smoothedLookLeft;
            _currentLookRight = smoothedLookRight;

            _gazeInterpolate.UpdateTime(Time.timeAsDouble);

            var lookUpDown = _currentLookUp - _currentLookDown;
            var lookLeftRight = _currentLookRight - _currentLookLeft;

            _vrm.Runtime.LookAt.SetYawPitchManually(GazeStrength * lookLeftRight, GazeStrength * lookUpDown);
        }

        private void UpdateMouth()
        {
            if (!EnableMouth || !Tracker.ActiveFaceBlendShapes) return;

            var t = _mouthInterpolate.Interpolate();

            var mouthOpen = Tracker.FaceBlendShapes[(int)FaceBlendShapes.JawOpen] * MouthOpenSensitivity;
            var au = Mathf.Lerp(_vrm.Runtime.Expression.GetWeight(ExpressionKey.Aa), mouthOpen, 1f - MouthSmoothing);

            _vrm.Runtime.Expression.SetWeight(ExpressionKey.Aa, au);
            _vrm.Runtime.Expression.SetWeight(ExpressionKey.Ih, 0);
            _vrm.Runtime.Expression.SetWeight(ExpressionKey.Ou, 0);
            _vrm.Runtime.Expression.SetWeight(ExpressionKey.Ee, 0);
            _vrm.Runtime.Expression.SetWeight(ExpressionKey.Oh, 0);

            var smile = (Tracker.FaceBlendShapes[(int)FaceBlendShapes.MouthSmileLeft] + Tracker.FaceBlendShapes[(int)FaceBlendShapes.MouthSmileRight]) / 2f * MouthShapeSensitivity;
            var frown = (Tracker.FaceBlendShapes[(int)FaceBlendShapes.MouthFrownLeft] + Tracker.FaceBlendShapes[(int)FaceBlendShapes.MouthFrownRight]) / 2f * MouthShapeSensitivity;
            var pucker = Tracker.FaceBlendShapes[(int)FaceBlendShapes.MouthPucker] * MouthShapeSensitivity;
            var funnel = Tracker.FaceBlendShapes[(int)FaceBlendShapes.MouthFunnel] * MouthShapeSensitivity;
            var shapeTotal = smile + frown + pucker + funnel;

            if (shapeTotal > 1.0f)
            {
                smile /= shapeTotal;
                frown /= shapeTotal;
                pucker /= shapeTotal;
                funnel /= shapeTotal;
            }

            _vrm.Runtime.Expression.SetWeight(ExpressionKey.Happy, 0);
            _vrm.Runtime.Expression.SetWeight(ExpressionKey.Sad, 0);
            _vrm.Runtime.Expression.SetWeight(ExpressionKey.Relaxed, 0);
            _vrm.Runtime.Expression.SetWeight(ExpressionKey.Surprised, 0);
            _vrm.Runtime.Expression.SetWeight(ExpressionKey.Happy, Mathf.Lerp(_vrm.Runtime.Expression.GetWeight(ExpressionKey.Happy), smile, 1f - MouthSmoothing));
            _vrm.Runtime.Expression.SetWeight(ExpressionKey.Sad, Mathf.Lerp(_vrm.Runtime.Expression.GetWeight(ExpressionKey.Sad), frown, 1f - MouthSmoothing));
            _vrm.Runtime.Expression.SetWeight(ExpressionKey.Oh, Mathf.Max(_vrm.Runtime.Expression.GetWeight(ExpressionKey.Oh), pucker));
            _vrm.Runtime.Expression.SetWeight(ExpressionKey.Ou, Mathf.Max(_vrm.Runtime.Expression.GetWeight(ExpressionKey.Ou), funnel));

            _mouthInterpolate.UpdateTime(Time.timeAsDouble);
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
                            Time.deltaTime * FingerResetSpeed
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

                    var landmarks = isRightHand ? Tracker.RightHandWorldLandmarks : Tracker.LeftHandWorldLandmarks;

                    var worldTargetDirection = (landmarks[(int)firstLandmark + i + 1].Position - landmarks[(int)firstLandmark + i].Position).normalized;

                    if (worldTargetDirection == Vector3.zero) continue;

                    var initialBoneLocalTargetDirection = Quaternion.Inverse(initialLocalRotation) * handWorldRotation * worldTargetDirection;

                    transform.localRotation = Quaternion.FromToRotation(_initialBoneDirections[bone], initialBoneLocalTargetDirection) * initialLocalRotation;

                    handWorldRotation = Quaternion.Inverse(transform.localRotation) * handWorldRotation;
                }
            }
        }

        private static float CalculateBlinkValueFromARKit(float arkitScore, float openedThreshold, float closedThreshold)
        {
            if (arkitScore >= openedThreshold)
            {
                if (arkitScore <= closedThreshold)
                {
                    return (arkitScore - openedThreshold) / (closedThreshold - openedThreshold);
                }

                return 1f;
            }

            return 0f;
        }
    }
}