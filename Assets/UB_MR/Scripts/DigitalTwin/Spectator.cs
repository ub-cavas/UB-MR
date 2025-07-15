using UnityEngine;
using Unity.Cinemachine;

namespace CAVAS.UB_MR.DT
{
    public class Spectator : DigitalTwin
    {
        [SerializeField] CinemachineCamera mTrackCam;
        ObservableObjectTracker tracker;
        int idx = 0;

        void Start()
        {
            tracker = FindFirstObjectByType<ObservableObjectTracker>();
            Spectate(this);
        }


        public void Spectate(Observable inTarget)
        {
            this.mTrackCam.Target.TrackingTarget = ((MonoBehaviour)inTarget).transform;
        }

        public Observable Next()
        {
            if (tracker.ObservableNetworkObjects.Count == 0)
                return null;

            idx += 1;
            if (idx >= tracker.ObservableNetworkObjects.Count)
                idx = 0;

            Observable next = tracker.ObservableNetworkObjects[idx].GetComponent<Observable>();
            Spectate(next);
            return next;
        }

        public Observable Previous()
        {
            if (tracker.ObservableNetworkObjects.Count == 0)
                return null;

            idx -= 1;
            if (idx < 0)
                idx = tracker.ObservableNetworkObjects.Count - 1;

            Observable prev = tracker.ObservableNetworkObjects[idx].GetComponent<Observable>();
            Spectate(prev);
            return prev;
        }
    }
}
