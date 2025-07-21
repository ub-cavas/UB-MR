using UnityEngine;
using Unity.Netcode;
using Unity.Cinemachine;
using System;

namespace CAVAS.UB_MR.DT
{
    public interface Observable
    {
    }

    public class DigitalTwin : NetworkBehaviour, Observable
    {
        public static event Action OnSpawn;

        protected CinemachineCamera[] mCameras;
        protected Canvas mHUD;

        void Awake()
        {
            this.mCameras = GetComponentsInChildren<CinemachineCamera>(true);
            this.mHUD = GetComponentInChildren<Canvas>(true);
        }

        public override void OnNetworkSpawn()
        {
            OnSpawn?.Invoke();
            if (!IsOwner)
            {
                EnableCameras(false);
                EnableUI(false);
            }
        }

        void EnableCameras(bool enable)
        {
            if (this.mCameras == null || this.mCameras.Length == 0)
            {
                this.mCameras = GetComponentsInChildren<CinemachineCamera>(true);
            }


            foreach (var cam in this.mCameras)
            {
                cam.gameObject.SetActive(enable);
            }
        }

        void EnableUI(bool enable)
        {
            if (this.mHUD == null)
            {
                this.mHUD = GetComponentInChildren<Canvas>(true);
            }
            if (this.mHUD != null)
            {
                this.mHUD.enabled = enable;
            }
        }



    }
}
