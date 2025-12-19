using System.Collections.Generic;
using UnityEngine;

namespace UB_MR.Redis_Networking
{
    public class TrafficRenderer : MonoBehaviour
    {
        [SerializeField] TrafficReceiver receiver;

        [Header("Prefabs")]
        public GameObject defaultVehicle;
        public GameObject defaultWalker;

        Dictionary<string, GameObject> actors = new();

        Queue<TrafficReceiver.TrafficActorData> CREATE_Q = new();
        Queue<TrafficReceiver.TrafficActorData> UPDATE_Q = new();
        Queue<TrafficReceiver.TrafficActorData> REMOVE_Q = new();

        void Start()
        {
            receiver.OnSpawnTrafficActor += a => CREATE_Q.Enqueue(a);
            receiver.OnTrafficActorUpdate += a => UPDATE_Q.Enqueue(a);
            receiver.OnDespawnTrafficActor += a => REMOVE_Q.Enqueue(a);
        }

        void Update()
        {
            while (CREATE_Q.TryDequeue(out var a))
            {
                GameObject prefab =
                    a.actor_type == "walker"
                        ? defaultWalker
                        : defaultVehicle;

                GameObject go = Instantiate(
                    prefab,
                    a.Position(),
                    a.Orientation()
                );

                actors[a.Id] = go;
            }

            while (UPDATE_Q.TryDequeue(out var a))
            {
                if (!actors.ContainsKey(a.Id)) continue;

                actors[a.Id].transform.SetPositionAndRotation(
                    a.Position(),
                    a.Orientation()
                );
            }

            while (REMOVE_Q.TryDequeue(out var a))
            {
                if (!actors.ContainsKey(a.Id)) continue;

                Destroy(actors[a.Id]);
                actors.Remove(a.Id);
            }
        }
    }
}