using UnityEngine;
using System.Collections;
using CAVAS.UB_MR.DT;

namespace Assets.UB_MR.Scripts.SimpleTraffic
{
    public class TrafficAgent : DigitalTwin
    {

        public override void OnNetworkSpawn()
        {
            base.OnNetworkSpawn();
            if (IsOwner)
            {
                TrafficManager trafficManager = FindFirstObjectByType<TrafficManager>();
                Transform[] waypoints = trafficManager.GetWaypoints();
                float velocity = trafficManager.GetVelocity();
                StartCoroutine(WaypointFollow(waypoints, velocity));
            }
        }

        IEnumerator WaypointFollow(Transform[] transforms, float velocity)
        {
            Transform startTransform = transform;
            float currentProgress = 0f;

            // Move from current position to first waypoint
            float distance;
            float duration;
            float elapsedTime;
            // Teleport to first waypoint
            transform.position = transforms[0].position;

          
            while (true)
            {
                // Move through each subsequent waypoint
                for (int i = 1; i < transforms.Length; i++)
                {
                    Transform fromTransform = transforms[i - 1];
                    Transform toTransform = transforms[i];

                    distance = Vector3.Distance(fromTransform.position, toTransform.position);
                    duration = distance / velocity;

                    elapsedTime = 0f;
                    currentProgress = 0f;

                    while (elapsedTime < duration)
                    {
                        elapsedTime += Time.deltaTime;
                        currentProgress = elapsedTime / duration;
                        Vector3 currentPos = Vector3.Lerp(fromTransform.position, toTransform.position, currentProgress);
                        transform.position = currentPos;

                        yield return null;
                    }

                    transform.position = toTransform.position;
                    transform.rotation = toTransform.rotation; 
                    currentProgress = 1f;
                }

                // Move back to the first waypoint
                Transform finalTransform = transforms[transforms.Length - 1];
                Transform firstTransform = transforms[0];

                distance = Vector3.Distance(finalTransform.position, firstTransform.position);
                duration = distance / velocity;

                elapsedTime = 0f;
                currentProgress = 0f;

                while (elapsedTime < duration)
                {
                    elapsedTime += Time.deltaTime;
                    currentProgress = elapsedTime / duration;
                    Vector3 currentPos = Vector3.Lerp(finalTransform.position, firstTransform.position, currentProgress);
                    transform.position = currentPos;

                    yield return null;
                }

                transform.position = firstTransform.position;
                transform.rotation = firstTransform.rotation; 
                currentProgress = 1f;
            }
        }
    }
}

