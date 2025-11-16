using Unity.Cinemachine;
using UnityEngine;

namespace CAVAS.UB_MR.Agent
{
    public class DigitalTwin : MonoBehaviour
    {
        HUD hud;
        CinemachineCamera[] cinemachineCameras;
        int camIdx = 0;

        protected virtual void OnEnable()
        {
            // Cinemachine Cameras
            cinemachineCameras = GetComponentsInChildren<CinemachineCamera>(true);
            DisableAllSpectatorCameras();
            EnableSpectatorCamera(cinemachineCameras[camIdx], true);

            // HUD
            if (hud is null)
                hud = new HUD();
            hud.OnNextSpectatorCamera += NextSpectatorCamera;
            hud.OnPrevSpectatorCamera += PreviousSpectatorCamera;
        }

        protected virtual void OnDisable()
        {
            hud.OnNextSpectatorCamera -= NextSpectatorCamera;
            hud.OnPrevSpectatorCamera -= PreviousSpectatorCamera;
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
