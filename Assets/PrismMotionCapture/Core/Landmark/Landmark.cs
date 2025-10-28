using UnityEngine;

namespace PMC
{
    public class Landmark
    {
        // Properties

        public Vector3 Position { get; set; }

        public float Visibility { get; set; }

        public float Presence { get; set; }

        public KalmanFilter KalmanFilter { get; set; }


        // Methods

        public void Set(Mediapipe.Landmark landmark)
        {
            Position = new Vector3(landmark.X, landmark.Y, landmark.Z);
            Visibility = landmark.Visibility;
            Presence = landmark.Presence;
        }

        public void Set(Mediapipe.NormalizedLandmark landmark)
        {
            Position = new Vector3(landmark.X, landmark.Y, landmark.Z);
            Visibility = landmark.Visibility;
            Presence = landmark.Presence;
        }
    }
}