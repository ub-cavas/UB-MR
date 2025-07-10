using UnityEngine;

namespace CAVAS.UB_MR.DT
{
    [RequireComponent(typeof(Rigidbody))]
    public class DT_Reflect : DigitalTwin
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
