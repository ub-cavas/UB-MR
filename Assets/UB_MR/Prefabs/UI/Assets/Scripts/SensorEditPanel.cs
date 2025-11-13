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

        Sensor sensor;

        public void SetSensor(Sensor inSensor)
        {
            sensor = inSensor;
        }

        public override void LoadPanel()
        {
            if (sensor is null)
                sensor.name = "Sensor_1";
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
            
            base.LoadPanel();
        }
    }
}
