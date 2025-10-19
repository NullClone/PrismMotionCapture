using System.Collections.Generic;
using UnityEngine;

namespace VMC
{
    public class AnimationController
    {
        public class AnimationItem
        {
            public float Time { get; set; } //アニメーションにかける時間
            public float StartValue { get; set; }
            public float EndValue { get; set; }
            public System.Action<float> SetAction { get; set; }
            public System.Func<float> TimeInitializer { get; set; }

            public void RunAction(float value)
            {
                SetAction?.Invoke(value);
            }

            public void Initialize()
            {
                if (TimeInitializer != null)
                {
                    Time = TimeInitializer();
                }
            }
        }

        private bool isStart = false;
        private float startTime = 0.0f;
        public System.Action ResetAction { get; set; }

        public List<AnimationItem> AnimationItems = new List<AnimationItem>();

        public Dictionary<float, AnimationItem> CurrentAnimationItems = new Dictionary<float, AnimationItem>(); //Key:開始時間


        private AnimationItem EndLastItem = null;
        private AnimationItem CurrentItem = null;

        private void InitializeAnimation()
        {
            CurrentAnimationItems.Clear();
            EndLastItem = null;
            CurrentItem = null;
            var starttime = 0.0f;
            foreach (var item in AnimationItems)
            {
                item.Initialize();
                CurrentAnimationItems.Add(starttime, item);
                starttime += item.Time == 0.0f ? 0.0001f : item.Time;
            }
        }

        public void AddResetAction(System.Action resetAction)
        {
            ResetAction = resetAction;
        }

        public void AddWait(float? time, System.Func<float> timeInitializer = null)
        {
            AddAnimation(time, 0.0f, 0.0f, null, timeInitializer);
        }

        public void AddAnimation(float? time, float startValue, float endValue, System.Action<float> setAction, System.Func<float> timeInitializer = null)
        {
            AnimationItems.Add(new AnimationItem { Time = time ?? 0.0f, StartValue = startValue, EndValue = endValue, SetAction = setAction, TimeInitializer = timeInitializer });
        }

        public void Reset()
        {
            StopAnimations();
            ResetAction?.Invoke();
        }

        public void ClearAnimations()
        {
            AnimationItems.Clear();
        }

        public void StopAnimations()
        {
            isStart = false;
            lastitem = null;
        }

        private AnimationItem lastitem = null;

        public bool Next()
        {
            if (isStart == false)
            {
                isStart = true;
                startTime = Time.time;
                InitializeAnimation();
            }

            var elapsedTime = Time.time - startTime;
            var addTime = 0.0f;
            foreach (var item in CurrentAnimationItems)
            {
                addTime = item.Key + item.Value.Time; //すべてのアニメーションの時間+今のアニメーション時間
                if (addTime >= elapsedTime)
                {//経過時間がまだアニメーションの終了時間に届いていない間(アニメーション中)
                 //Debug.Log($"AnimationTime:{elapsedTime}");
                    if (lastitem != null && EndLastItem != lastitem)
                    {//前回のアニメーションが終わりまで行ってない場合があるので100％で実行
                        lastitem.RunAction(lastitem.EndValue);
                        EndLastItem = lastitem;
                    }
                    if (CurrentItem != item.Value)
                    {
                        if (lastitem != null)
                        {//前回のアニメーションが終わりまで行ってない場合があるので100％で実行
                            lastitem.RunAction(lastitem.EndValue);
                            EndLastItem = lastitem;
                        }
                        //新しいアニメーションになったときには時間にかかわらずきちんと最初の値を使う
                        item.Value.RunAction(item.Value.StartValue);
                        CurrentItem = item.Value;
                    }
                    else
                    {
                        var currentTime = item.Value.Time + (elapsedTime - addTime);
                        var setvalue = item.Value.StartValue + ((item.Value.EndValue - item.Value.StartValue) * (currentTime / item.Value.Time));
                        item.Value.RunAction(setvalue);
                    }
                    lastitem = item.Value;
                    return true;
                }
            }

            //最後までアニメーションしたとき
            if (lastitem != null)
            {//最後のアニメーションが終わりまで行ってない場合があるので100％で実行
                lastitem.RunAction(lastitem.EndValue);
            }
            isStart = false;
            return false;
        }
    }

    public class HandController : MonoBehaviour
    {

        private AnimationController leftAnimationController;
        private AnimationController rightAnimationController;
        private bool doLeftAnimation = false;
        private bool doRightAnimation = false;

        //指のデフォルト角度取得
        public void SetDefaultAngle(Animator animator)
        {
            FingerTransforms.Clear();
            FingerDefaultVectors.Clear();
            foreach (var bone in FingerBones)
            {
                var transform = animator.GetBoneTransform(bone);
                FingerTransforms.Add(transform);
                if (transform == null)
                {
                    FingerDefaultVectors.Add(Vector3.zero);
                }
                else
                {
                    FingerDefaultVectors.Add(new Vector3(transform.rotation.x, transform.rotation.y, transform.rotation.z));
                }
            }
        }

        public void SetNaturalPose()
        {
            var handAngles = new List<int> { -16, -16, -17, 1, -16, -16, -20, 3, -16, -25, -10, 1, -22, -12, -21, 2, -24, -51, -9, 15 };
            SetHandEulerAngles(true, true, CalcHandEulerAngles(handAngles));
        }


        private List<HumanBodyBones> FingerBones = new List<HumanBodyBones>
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

        private List<Transform> FingerTransforms = new List<Transform>();
        private List<Vector3> FingerDefaultVectors = new List<Vector3>();

        public void SetHandAngle(bool LeftEnable, bool RightEnable, List<int> angles, float animationTime)
        {
            if (leftAnimationController == null) leftAnimationController = new AnimationController();
            if (rightAnimationController == null) rightAnimationController = new AnimationController();

            var startEulers = GetHandEulerAngles();
            if (startEulers == null) return;
            var endEulers = CalcHandEulerAngles(angles);

            if (LeftEnable)
            {
                leftAnimationController.StopAnimations();
                leftAnimationController.ClearAnimations();

                leftAnimationController.AddAnimation(animationTime, 0.0f, 1.0f, v => SetHandEulerAngles(true, false, eulersLerp(startEulers, endEulers, v)));

                doLeftAnimation = true;
            }
            if (RightEnable)
            {
                rightAnimationController.StopAnimations();
                rightAnimationController.ClearAnimations();

                rightAnimationController.AddAnimation(animationTime, 0.0f, 1.0f, v => SetHandEulerAngles(false, true, eulersLerp(startEulers, endEulers, v)));

                doRightAnimation = true;
            }
        }

        /*
        private List<Vector3> eulersLerp(List<Vector3> startEulers, List<Vector3> endEulers, float t)
        {
            var eulers = new List<Vector3>();
            for (int i = 0; i < startEulers.Count; i++)
            {
                eulers.Add(Vector3.Lerp(startEulers[i], endEulers[i], t));
            }
            return eulers;
        }
        */
        private List<Vector3> eulersLerp(List<Vector3> startEulers, List<Vector3> endEulers, float t)
        {
            var eulers = new List<Vector3>();
            for (int i = 0; i < startEulers.Count; i++)
            {
                var calcStart = new Vector3(startEulers[i].x > 180 ? startEulers[i].x - 360 : startEulers[i].x, startEulers[i].y > 180 ? startEulers[i].y - 360 : startEulers[i].y, startEulers[i].z > 180 ? startEulers[i].z - 360 : startEulers[i].z);
                var calcEnd = new Vector3(endEulers[i].x > 180 ? endEulers[i].x - 360 : endEulers[i].x, endEulers[i].y > 180 ? endEulers[i].y - 360 : endEulers[i].y, endEulers[i].z > 180 ? endEulers[i].z - 360 : endEulers[i].z);
                eulers.Add(Vector3.Lerp(calcStart, calcEnd, t));
            }
            return eulers;
        }


        public void SetHandEulerAngles(bool LeftEnable, bool RightEnable, List<Vector3> Eulers)
        {
            if (FingerTransforms.Count == 0) return;
            var handBonesCount = FingerBones.Count / 2;
            if (LeftEnable)
            {
                for (int i = 0; i < handBonesCount; i++)
                {
                    if (FingerTransforms[i] != null) FingerTransforms[i].localRotation = Quaternion.Euler(Eulers[i]);
                }
            }
            if (RightEnable)
            {
                for (int i = 0; i < handBonesCount; i++)
                {
                    if (FingerTransforms[i + handBonesCount] != null) FingerTransforms[i + handBonesCount].localRotation = Quaternion.Euler(Eulers[i + handBonesCount]);
                }
            }
        }

        public List<Vector3> GetHandEulerAngles()
        {
            var handBonesCount = FingerBones.Count;
            if (FingerTransforms.Count != handBonesCount) return null;
            var eulers = new List<Vector3>();
            for (int i = 0; i < handBonesCount; i++)
            {
                if (FingerTransforms[i] != null)
                {
                    eulers.Add(FingerTransforms[i].localRotation.eulerAngles);
                }
                else
                {
                    eulers.Add(Vector3.zero);
                }
            }
            return eulers;
        }

        public List<Vector3> CalcHandEulerAngles(List<int> angles)
        {
            if (FingerDefaultVectors == null || FingerDefaultVectors.Count == 0) return null;
            var handBonesCount = FingerBones.Count / 2;
            var eulers = new Vector3[FingerBones.Count];
            for (int i = 0; i < handBonesCount; i += 3)
            {
                if (i >= 12)
                { //親指
                    var vector = FingerDefaultVectors[i + 2]; //第三関節
                    var angle = (-angles[(i / 3 * 4) + 2]) / 90.0f; //-90が1.0 -45は0.5 -180は2.0
                    var sideangle = angles[(i / 3 * 4) + 3];
                    var ax = angle * 0.0f;
                    var ay = (angle * 0.0f) + sideangle;
                    var az = (float)angles[(i / 3 * 4) + 2];
                    eulers[i + 2] = new Vector3(vector.x + ax, vector.y - ay, vector.z - az);

                    vector = FingerDefaultVectors[i + 1]; //第二関節
                    angle = (-angles[(i / 3 * 4) + 1]) / 90.0f;
                    ax = angle * 38f;
                    ay = angle * 38f;
                    az = angle * -15f;
                    eulers[i + 1] = new Vector3(vector.x + ax, vector.y - ay, vector.z - az);

                    vector = FingerDefaultVectors[i]; //第一関節
                    angle = (-angles[i / 3 * 4]) / 90.0f;
                    ax = angle * 34f;
                    ay = angle * 56f;
                    az = angle * -7f;
                    eulers[i] = new Vector3(vector.x + ax, vector.y - ay, vector.z - az);
                }
                else
                {
                    var vector = FingerDefaultVectors[i + 2]; //第三関節
                    var angle = angles[(i / 3 * 4) + 2];
                    var sideangle = angles[(i / 3 * 4) + 3];
                    eulers[i + 2] = new Vector3(vector.x, vector.y - sideangle, vector.z - angle);

                    vector = FingerDefaultVectors[i + 1]; //第二関節
                    angle = angles[(i / 3 * 4) + 1];
                    eulers[i + 1] = new Vector3(vector.x, vector.y/* - sideangle*/, vector.z - angle);

                    vector = FingerDefaultVectors[i]; //第一関節
                    angle = angles[i / 3 * 4];
                    eulers[i] = new Vector3(vector.x, vector.y/* - sideangle*/, vector.z - angle);
                }
            }
            for (int i = 0; i < handBonesCount; i += 3)
            {
                if (i >= 12)
                { //親指
                    var vector = FingerDefaultVectors[i + 2]; //第三関節
                    var angle = (-angles[(i / 3 * 4) + 2]) / 90.0f; //-90が1.0 -45は0.5 -180は2.0
                    var sideangle = angles[(i / 3 * 4) + 3];
                    var ax = angle * 0.0f;
                    var ay = (angle * 0.0f) + sideangle;
                    var az = (float)angles[(i / 3 * 4) + 2];
                    eulers[i + handBonesCount + 2] = new Vector3(vector.x + ax, vector.y + ay, vector.z + az);

                    vector = FingerDefaultVectors[i + 1]; //第二関節
                    angle = (-angles[(i / 3 * 4) + 1]) / 90.0f;
                    ax = angle * 38f;
                    ay = angle * 38f;
                    az = angle * -15f;
                    eulers[i + handBonesCount + 1] = new Vector3(vector.x + ax, vector.y + ay, vector.z + az);

                    vector = FingerDefaultVectors[i]; //第一関節
                    angle = (-angles[i / 3 * 4]) / 90.0f;
                    ax = angle * 34f;
                    ay = angle * 56f;
                    az = angle * -7f;
                    eulers[i + handBonesCount] = new Vector3(vector.x + ax, vector.y + ay, vector.z + az);
                }
                else
                {
                    var vector = FingerDefaultVectors[i + 2]; //第三関節
                    var angle = angles[(i / 3 * 4) + 2];
                    var sideangle = angles[(i / 3 * 4) + 3];
                    eulers[i + handBonesCount + 2] = new Vector3(vector.x, vector.y + sideangle, vector.z + angle);

                    vector = FingerDefaultVectors[i + 1]; //第二関節
                    angle = angles[(i / 3 * 4) + 1];
                    eulers[i + handBonesCount + 1] = new Vector3(vector.x, vector.y/* + sideangle*/, vector.z + angle);

                    vector = FingerDefaultVectors[i]; //第一関節
                    angle = angles[i / 3 * 4];
                    eulers[i + handBonesCount] = new Vector3(vector.x, vector.y/* + sideangle*/, vector.z + angle);
                }
            }
            return new List<Vector3>(eulers);
        }

        // Update is called once per frame
        void Update()
        {
            if (doLeftAnimation && leftAnimationController?.Next() == false)
            {
                doLeftAnimation = false;
            }
            if (doRightAnimation && rightAnimationController?.Next() == false)
            {
                doRightAnimation = false;
            }
        }
    }
}