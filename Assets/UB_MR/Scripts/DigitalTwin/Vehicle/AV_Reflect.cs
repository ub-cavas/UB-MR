using UnityEngine;

namespace CAVAS.UB_MR.DT.Vehicle
{
    public class AV_Reflect : DynamicAgent
    {
        protected override void Update()
        {
            base.Update();
            SnapUpdate();
        }

        void SnapUpdate()
        {
            this.transform.position = WorldPosition();
            this.transform.rotation = WorldRotation();
        }
    }
}
