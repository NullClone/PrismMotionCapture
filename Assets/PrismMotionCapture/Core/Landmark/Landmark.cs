using UnityEngine;

namespace PMC
{
    public class Landmark
    {
        // Properties

        public string Name { get; set; }

        public float? Presence { get; set; }

        public float? Visibility { get; set; }

        public Vector3 Position { get; set; }

        public KalmanFilter KalmanFilter { get; set; }


        // Methods

        public void Set(Mediapipe.Tasks.Components.Containers.Landmark landmark)
        {
            Name = landmark.name;
            Presence = landmark.presence;
            Visibility = landmark.visibility;
            Position = new Vector3(landmark.x, landmark.y, landmark.z);
        }

        public void Set(Mediapipe.Tasks.Components.Containers.NormalizedLandmark landmark)
        {
            Name = landmark.name;
            Presence = landmark.presence;
            Visibility = landmark.visibility;
            Position = new Vector3(landmark.x, landmark.y, landmark.z);
        }
    }
}