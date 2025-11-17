using CAVAS.UB_MR.Config;
using Unity.Cinemachine;
using UnityEngine;

namespace CAVAS.UB_MR.DT
{
    public class Agent : MonoBehaviour
    {
        Transform baseLink;
        Transform spectatorCameras;
        HUD hud;
        CinemachineCamera[] cinemachineCameras;
        int camIdx = 0;

        protected virtual void OnEnable()
        {
            
        }

        protected virtual void OnDisable()
        {
            hud.OnNextSpectatorCamera -= NextSpectatorCamera;
            hud.OnPrevSpectatorCamera -= PreviousSpectatorCamera;
        }

        public void Setup(Config.Agent inAgent)
        {
            this.gameObject.name = inAgent.name;

            // Sensors
            baseLink = transform.Find("base_link");
            foreach (Sensor sensor in inAgent.sensors.Values)
            {
                GameObject sensorGO = GameObject.Instantiate(new GameObject(), baseLink);
                sensorGO.name = sensor.name;
                sensorGO.transform.SetLocalPositionAndRotation(sensor.position, Quaternion.Euler(sensor.rotation));
            }
                
            // Spectator Cameras
            spectatorCameras = transform.Find("spectator_cameras");
            cinemachineCameras = spectatorCameras.GetComponentsInChildren<CinemachineCamera>(true);
            DisableAllSpectatorCameras();
            EnableSpectatorCamera(cinemachineCameras[camIdx], true);

            // HUD
            if (hud is null)
                hud = new HUD();
            hud.OnNextSpectatorCamera += NextSpectatorCamera;
            hud.OnPrevSpectatorCamera += PreviousSpectatorCamera;
            
        }

        protected void DisableAllSpectatorCameras()
        {
            foreach (CinemachineCamera cam in cinemachineCameras)
                EnableSpectatorCamera(cam, false);
        }

        protected void EnableSpectatorCamera(CinemachineCamera inCamera, bool inEnable)
        {
            DisableAllSpectatorCameras();
            inCamera.gameObject.SetActive(inEnable);
        }

        public void NextSpectatorCamera()
        {
            camIdx = (camIdx + 1) % cinemachineCameras.Length;
        }

        public void PreviousSpectatorCamera()
        {
            camIdx = (camIdx - 1) % cinemachineCameras.Length;
        }
    }
}
