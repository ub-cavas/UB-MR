using CAVAS.UB_MR.Config;
using CAVAS.UI;
using UnityEngine;

namespace CAVAS.UB_MR.UI
{
    public class AgentEditMenu : ScrollPanel
    {
        [SerializeField] SensorEditPanel sensorEditPanel;
        Agent agent;

        public void SetAgent(Agent inAgent)
        {
            agent = inAgent;
        }

        public override void LoadPanel()
        {
            if (agent != null)
            {
                SetTitle(agent.name);
                RemoveAllButtons();
                foreach (string name in agent.sensors.Keys)
                    AddButton(name);
            }
            base.LoadPanel();
        }

        
        protected override void OnAddClicked()
        {
            UI_Manager.LoadPanel(sensorEditPanel);
        }

        protected override void OnEditClicked()
        {
            sensorEditPanel.SetSensor(agent.sensors[GetSelectedButtonLabel()]);
            UI_Manager.LoadPanel(sensorEditPanel);
        }

        protected override void OnRemoveClicked()
        {
            throw new System.NotImplementedException();
        }
    }
}
