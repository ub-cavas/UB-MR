using UnityEngine;
using CAVAS.UB_MR.Config;
using UnityEngine.UI;
using TMPro;
using System;


namespace CAVAS.UB_MR.UI.Vehicle
{
    public class VehicleSetupMenu : MonoBehaviour
    {   
        [Header("NAME")]
        [SerializeField] GameObject vehicleNamePanel;
        [SerializeField] TMP_InputField nameInputField;
        [SerializeField] Button confirmNameButton;

        [Header("VEHICLE OVERVIEW")]
        [SerializeField] GameObject vehicleOverviewPanel;
        [SerializeField] TextMeshProUGUI title;
        [SerializeField] SensorScroller sensorButtonScroller;
        [SerializeField] Button confirmButton;
        [SerializeField] Button addSensorButton;
        [SerializeField] Button removeSensorButton;

        [Header("SENSOR")]
        [SerializeField] GameObject sensorPanel;
        [SerializeField] Button saveSensorButton;
        [SerializeField] TMP_InputField sensorName;
        [SerializeField] TMP_InputField sensorTopic;
        [SerializeField] TMP_Dropdown sensorType;
        [SerializeField] TMP_InputField posOffX;
        [SerializeField] TMP_InputField posOffY;
        [SerializeField] TMP_InputField posOffZ;
        [SerializeField] TMP_InputField roll;
        [SerializeField] TMP_InputField pitch;
        [SerializeField] TMP_InputField yaw;


        Config.Agent activeAgent;
        Config.Sensor activeSensor;

        void OnEnable()
        {
            // Buttons
            confirmNameButton.onClick.AddListener(FindOrCreateVehicle);
            confirmButton.onClick.AddListener(SaveVehicle);
            
            removeSensorButton.onClick.AddListener(RemoveSensor);

            saveSensorButton.onClick.AddListener(SaveSensor);
            saveSensorButton.onClick.AddListener(OpenEditAgentMenu);

            addSensorButton.onClick.AddListener(sensorButtonScroller.AddSensor);
            sensorButtonScroller.OnSensorClicked += OpenSensorMenu;
        }

        void OnDisable()
        {
            // Buttons
            confirmNameButton.onClick.RemoveAllListeners();
            confirmButton.onClick.RemoveAllListeners();
            addSensorButton.onClick.RemoveAllListeners();
            removeSensorButton.onClick.RemoveAllListeners();
            saveSensorButton.onClick.RemoveAllListeners();

            sensorButtonScroller.OnSensorClicked -= OpenSensorMenu;
        }

        void RemoveSensor()
        {
            //TODO: delete sensor
        }

        void OpenEditAgentMenu()
        {
            OpenEditAgentMenu(activeAgent);
        }

        void OpenEditAgentMenu(Config.Agent inAgent)
        {
            activeAgent = inAgent;
            // Disable other menus
            this.vehicleNamePanel.SetActive(false);
            this.sensorPanel.SetActive(false);
            // Enable necessary components
            this.vehicleOverviewPanel.SetActive(true);
            // Set title text
            title.text = inAgent.name;
            // Clear old buttons
            sensorButtonScroller.RemoveAllElements();
            // Create new buttons for each sensor
            foreach (string name in inAgent.sensors.Keys)
                sensorButtonScroller.AddElement(name);
        }

        void OpenSensorMenu(string inSensorName)
        {
            // Disable other submenus
            this.vehicleNamePanel.SetActive(false);
            this.vehicleOverviewPanel.SetActive(false);
            // Enable sub-menu
            this.sensorPanel.SetActive(true);
            Sensor sensor = new Sensor();
            // New Sensor
            if (inSensorName is null)
            {
                // Name (Default)
                sensor.name = "Sensor_1";
                sensorName.text = sensor.name;
                this.activeSensor = sensor;
                return; 
            }
            // Prepopulate if existing sensor
            if (activeAgent.sensors.ContainsKey(inSensorName))
            {
                this.activeSensor = activeAgent.sensors[inSensorName];
                // Name
                sensorName.text = sensor.name;
                // Topic
                sensorTopic.text = sensor.topic;
                // Type
                int index = sensorType.options.FindIndex(option => option.text == sensor.type.ToString());
                sensorType.value = index;
                sensorType.RefreshShownValue();
                // Offsets
                posOffX.text = sensor.position.x.ToString();
                posOffY.text = sensor.position.y.ToString();
                posOffZ.text = sensor.position.z.ToString();
                roll.text = sensor.rotation.x.ToString();
                pitch.text = sensor.rotation.y.ToString();
                yaw.text = sensor.rotation.z.ToString();
            }
        }

        void SaveSensor()
        {
            // Update sensor values
            this.activeSensor.name = sensorName.text;
            this.activeSensor.topic = sensorTopic.text;

            // Get sensor type
            string label = sensorType.options[sensorType.value].text;
            if (Enum.TryParse(label, ignoreCase: true, out SensorType sensor))
                this.activeSensor.type = sensor;
            else
                Debug.LogWarning($"'{label}' is not a valid SensorType");
            
             
            this.activeSensor.position = new Vector3(float.Parse(posOffX.text), float.Parse(posOffY.text), float.Parse(posOffZ.text));
            this.activeSensor.rotation = new Vector3(float.Parse(roll.text), float.Parse(pitch.text), float.Parse(yaw.text));

            activeAgent.sensors[activeSensor.name] = activeSensor;
            print(this.activeSensor.position);
            SaveVehicle();
        }
        
        void SaveVehicle()
        {
            ConfigurationManager.SaveToJSON(activeAgent);
        }
        
        public void FindOrCreateVehicle()
        {
            name = nameInputField.text;
            Config.Agent agent = ConfigurationManager.LoadFromJSON(name);
            if (agent is not null)
                OpenEditAgentMenu(agent);
            else if (name != "") //TODO: this empty input check logic should be better
            {
                Config.Agent newAgent = new Config.Agent();
                newAgent.name = name;
                OpenEditAgentMenu(newAgent);
            }
            else
            {
                Debug.LogWarning("Name cannot be empty");
            }
        }
    }
}
