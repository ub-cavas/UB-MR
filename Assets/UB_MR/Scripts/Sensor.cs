using UnityEngine;

namespace CAVAS.UB_MR
{
    public abstract class SensorModifier
    {
        public abstract void CleanUp();
        public abstract void Publish();
    }
}

