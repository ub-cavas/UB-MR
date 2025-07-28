using System.Collections;
using Assets.UB_MR.Scripts.SimpleTraffic;
using UnityEngine;

namespace Assets.UB_MR.Scripts.SimpleTraffic
{
    public class TrafficManager : MonoBehaviour
    {
        [SerializeField] GameObject mTrafficAgentPrefab;
        [SerializeField] int mNumberOfAgents = 10;
        [SerializeField] float mVelocity = 15f;
        [SerializeField] Transform[] mWaypoints;

        IEnumerator Start()
        {
            int spawnedAgents = 0;
            while (spawnedAgents < mNumberOfAgents)
            {
                if (mTrafficAgentPrefab != null && mWaypoints.Length > 0)
                {
                    TrafficAgent agent = Instantiate(mTrafficAgentPrefab).GetComponent<TrafficAgent>();
                    StartCoroutine(agent.WaypointFollow(this.mWaypoints, this.mVelocity)); // Move to the first waypoint over 5 seconds
                    spawnedAgents++;
                    yield return new WaitForSeconds(5f); // Wait for 5 seconds before spawning the next agent
                }
            }
        }
    }
}
