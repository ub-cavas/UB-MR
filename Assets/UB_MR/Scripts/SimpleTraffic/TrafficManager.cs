using System.Collections;
using Unity.Netcode;
using UnityEngine;

namespace Assets.UB_MR.Scripts.SimpleTraffic
{
    public class TrafficManager : NetworkBehaviour
    {
        [SerializeField] GameObject mTrafficAgentPrefab;
        [SerializeField] int mNumberOfAgents = 10;
        [SerializeField] float mVelocity = 15f;
        [SerializeField] Transform[] mWaypoints;

        public override void OnNetworkSpawn()
        {
            base.OnNetworkSpawn();
            if (IsServer)
            {
                Debug.Log("Spawning Traffic Agents");
                StartCoroutine(SpawnAgents());
            }
        }

        public IEnumerator SpawnAgents(ulong client_id = 0)
        {
            int spawnedAgents = 0;
            while (spawnedAgents < mNumberOfAgents)
            {
                if (mTrafficAgentPrefab != null && mWaypoints.Length > 0)
                {
                    GameObject instance = Instantiate(mTrafficAgentPrefab, mWaypoints[0].position, Quaternion.identity);
                    var instanceNetworkObject = instance.GetComponent<NetworkObject>();
                    instanceNetworkObject.SpawnWithOwnership(client_id);
                    spawnedAgents++;
                    yield return new WaitForSeconds(5f); // Wait for 5 seconds before spawning the next agent
                }
            }
        }

        public Transform[] GetWaypoints()
        {
            return mWaypoints;
        }

        public float GetVelocity()
        {
            return mVelocity;
        }
    }
}
