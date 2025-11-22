using UnityEngine;

namespace CAVAS.UB_MR
{
    public abstract class Sensor
    {
        public abstract void CleanUp();
        public abstract void Publish();
    }
}

