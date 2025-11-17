using UnityEngine;
using CAVAS.UB_MR.DT;
using CAVAS.UB_MR.Config;
using CAVAS.UB_MR.DT.Vehicle;

namespace CAVAS.UB_MR
{
    public class ModuleBoot : MonoBehaviour
    {
        [SerializeField] GameObject AgentPrefab;

        void Start()
        {
            SpawnActiveAgent();
        }

        DT.Agent SpawnActiveAgent()
        {
            return SpawnAgent(ConfigurationManager.GetConfiguration().Item1);
        }

        DT.Agent SpawnAgent(Config.Agent inAgent)
        {
            DT.Agent agent;
            GameObject newAgent = Instantiate(AgentPrefab, new Vector3(0,0,0), Quaternion.identity);
            if (inAgent.type == AgentType.AutonomousVehicle)
                agent = newAgent.AddComponent<AutonomousVehicle>();
            else
                agent = newAgent.AddComponent<DT.Agent>();

            agent.Setup(inAgent);
            return agent;
        }
    }
}
