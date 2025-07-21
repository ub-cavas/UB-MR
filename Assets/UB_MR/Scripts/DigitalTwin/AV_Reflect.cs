using UnityEngine;

namespace CAVAS.UB_MR.DT
{
    [RequireComponent(typeof(Rigidbody))]
    public class AV_Reflect : AutonomousVehicle
    {

        void Update()
        {
            SnapUpdate();
        }

        void SnapUpdate()
        {
            this.transform.position = this.mWorldPosition;
            this.transform.rotation = this.mWorldRotation;
        }
    }
}
