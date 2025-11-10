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
        public Action<Config.Vehicle, Config.Sensor> OnEditSensor;

        
        [Header("NAME")]
        [SerializeField] GameObject vehicleNamePanel;
        [SerializeField] InputField nameInputField;
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
        [SerializeField] InputField sensorName;
        [SerializeField] InputField sensorTopic;
        [SerializeField] Dropdown sensorType;
        [SerializeField] InputField posOffX;
        [SerializeField] InputField posOffY;
        [SerializeField] InputField posOffZ;
        [SerializeField] InputField roll;
        [SerializeField] InputField pitch;
        [SerializeField] InputField yaw;


        Config.Vehicle activeVehicle;
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

        void OpenEditVehicleMenu(Config.Vehicle inVehicle)
        {
            activeVehicle = inVehicle;
            // Disable name menu
            this.vehicleNamePanel.SetActive(false);
            // Enable necessary components
            this.vehicleOverviewPanel.SetActive(true);
            // Set title text
            title.text = inVehicle.vehicle_name;
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
            if (activeVehicle.sensors.ContainsKey(inSensorName))
            {
                activeSensor = activeVehicle.sensors[inSensorName];
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
            activeVehicle.sensors[activeSensor.name] = activeSensor;
        }
        void SaveVehicle()
        {
            JSONManager.SaveToJSON(activeVehicle);
        }
        
        public void FindOrCreateVehicle()
        {
            name = nameInputField.text;
            Config.Vehicle vehicle = JSONManager.LoadFromJSON(name);
            if (vehicle is not null)
                OpenEditVehicleMenu(vehicle);
            else if (name != "") //TODO: this empty input check logic should be better
            {
                Config.Vehicle newVehicle = new Config.Vehicle();
                vehicle.vehicle_name = name;
                OpenEditVehicleMenu(newVehicle);
            }
            else
            {
                Debug.LogWarning("Name cannot be empty");
            }
        }
    }
}
