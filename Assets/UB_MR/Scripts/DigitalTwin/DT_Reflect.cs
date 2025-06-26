using UnityEngine;

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
