using UnityEngine;
using CAVAS.UB_MR.Config;
using UnityEngine.UI;
using TMPro;
using System;


namespace CAVAS.UB_MR.UI.Vehicle
{
    public class VehicleSetupMenu : MonoBehaviour
    {
        public Action OnAddSensor;
        public Action<Config.Agent, Config.Sensor> OnEditSensor;

        
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
            addSensorButton.onClick.AddListener(AddSensor);
            removeSensorButton.onClick.AddListener(RemoveSensor);
            saveSensorButton.onClick.AddListener(SaveSensor);

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

        void AddSensor()
        {
            //TODO: Open sensor add menu
            OnAddSensor?.Invoke();
        }

        void EditSensor()
        {
            //TODO: Open sensor edit menu
            //OnEditSensor?.Invoke();
        }

        void RemoveSensor()
        {
            //TODO: delete sensor
        }

        void OpenEditVehicleMenu(Config.Agent inVehicle)
        {
            activeAgent = inVehicle;
            // Disable name menu
            this.vehicleNamePanel.SetActive(false);
            // Enable necessary components
            this.vehicleOverviewPanel.SetActive(true);
            // Set title text
            title.text = inVehicle.name;
            // Clear old buttons
            sensorButtonScroller.RemoveAllElements();
            // Create new buttons for each sensor
            foreach (string name in inVehicle.sensors.Keys)
                sensorButtonScroller.AddElement(name);
        }

        void OpenSensorMenu(string inSensorName)
        {
            // Disable other submenus
            this.vehicleNamePanel.SetActive(false);
            this.vehicleOverviewPanel.SetActive(false);
            // Enable sub-menu
            this.sensorPanel.SetActive(true);
            Sensor activeSensor = new Sensor();
            // Prepopulate if existing sensor
            if (activeAgent.sensors.ContainsKey(inSensorName))
            {
                activeSensor = activeAgent.sensors[inSensorName];
                // Name
                sensorName.text = activeSensor.name;
                // Topic
                sensorTopic.text = activeSensor.topic;
                // Type
                int index = sensorType.options.FindIndex(option => option.text == activeSensor.type.ToString());
                sensorType.value = index;
                sensorType.RefreshShownValue();
                // Offsets
                posOffX.text = activeSensor.position.x.ToString();
                posOffY.text = activeSensor.position.y.ToString();
                posOffZ.text = activeSensor.position.z.ToString();
                roll.text = activeSensor.rotation.x.ToString();
                pitch.text = activeSensor.rotation.y.ToString();
                yaw.text = activeSensor.rotation.z.ToString();
            }
        }

        void SaveSensor()
        {
            activeAgent.sensors[activeSensor.name] = activeSensor;
            SaveVehicle();
        }
        void SaveVehicle()
        {
            ConfigurationManager.SaveToJSON(activeAgent);
        }
        
        public void FindOrCreateVehicle()
        {
            name = nameInputField.text;
            Config.Agent vehicle = ConfigurationManager.LoadFromJSON(name);
            if (vehicle is not null)
                OpenEditVehicleMenu(vehicle);
            else if (name != "") //TODO: this empty input check logic should be better
            {
                Config.Agent newVehicle = new Config.Agent();
                vehicle.name = name;
                OpenEditVehicleMenu(newVehicle);
            }
            else
            {
                Debug.LogWarning("Name cannot be empty");
            }
        }
    }
}
