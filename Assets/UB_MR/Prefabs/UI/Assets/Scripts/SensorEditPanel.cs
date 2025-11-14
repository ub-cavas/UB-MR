using System;
using CAVAS.UB_MR.Config;
using CAVAS.UI;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UB_MR.UI
{
    public class SensorEditPanel : Panel
    {
        [SerializeField] Button saveButton;
        [Space]
        [SerializeField] TMP_Dropdown sensorType;
        [SerializeField] TMP_InputField sensorName;
        [SerializeField] TMP_InputField sensorTopic;

        [Header("Offsets")]
        [SerializeField] TMP_InputField pos_off_x;
        [SerializeField] TMP_InputField pos_off_y;
        [SerializeField] TMP_InputField pos_off_z;
        [Space]
        [SerializeField] TMP_InputField roll;
        [SerializeField] TMP_InputField pitch;
        [SerializeField] TMP_InputField yaw;

        [Header("Panels")]
        [SerializeField] AgentEditMenu agentEditMenu;

        Agent agent;
        Sensor sensor;

        public void SetSensor(Sensor inSensor, Agent inAgent)
        {
            agent = inAgent;
            sensor = inSensor;
        }

        public override void LoadPanel()
        {
            base.LoadPanel();
            
            saveButton.onClick.AddListener(SaveSensor);
            if (sensor is null)
            {
                sensor = new Sensor();
                sensor.name = "New Sensor";
                sensorName.text = String.Empty;
                sensorTopic.text = String.Empty;
                pos_off_x.text = String.Empty;
                pos_off_y.text = String.Empty;
                pos_off_z.text = String.Empty;
                roll.text = String.Empty;
                pitch.text = String.Empty;
                yaw.text = String.Empty;
            }
            else
            {
                // Prepopulate Data
                sensorName.text = sensor.name;
                sensorTopic.text = sensor.topic;
                int index = sensorType.options.FindIndex(option => option.text == sensor.type.ToString());
                sensorType.value = index;
                sensorType.RefreshShownValue();

                pos_off_x.text = sensor.position.x.ToString();
                pos_off_y.text = sensor.position.y.ToString();
                pos_off_z.text = sensor.position.z.ToString();
                roll.text = sensor.rotation.x.ToString();
                pitch.text = sensor.rotation.y.ToString();
                yaw.text = sensor.rotation.z.ToString();
            }
            SetTitle(sensor.name);
        }

        public override void UnloadPanel()
        {
            saveButton.onClick.RemoveAllListeners();
            base.UnloadPanel();
        }

        void SaveSensor()
        {
            // Update sensor values
            sensor.name = sensorName.text;
            sensor.topic = sensorTopic.text;
            // Get sensor type
            string label = sensorType.options[sensorType.value].text;
            if (Enum.TryParse(label, ignoreCase: true, out SensorType type))
                sensor.type = type;
            else
                Debug.LogWarning($"'{label}' is not a valid SensorType");
            // Transform
            sensor.position = new Vector3(float.Parse(pos_off_x.text), float.Parse(pos_off_y.text), float.Parse(pos_off_z.text));
            sensor.rotation = new Vector3(float.Parse(roll.text), float.Parse(pitch.text), float.Parse(yaw.text));
            // Save
            agent.sensors[sensor.name] = sensor;
            ConfigurationManager.SaveToJSON(agent);
            UI_Manager.LoadPanel(agentEditMenu);
        }
    }
}
