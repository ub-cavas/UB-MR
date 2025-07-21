using System.Collections.Generic;
using UnityEngine;

public class VirtualObjectDetector
{
    static List<VirtualObject> sVirtualObjects;
    Transform mTransform;

    public VirtualObjectDetector(Transform inTransform)
    {
        if (sVirtualObjects == null)
        {
            sVirtualObjects = new List<VirtualObject>();
        }
        this.mTransform = inTransform;
    }

    public static void AddVirtualObjectToDatabase(VirtualObject vObj)
    {
        if (sVirtualObjects == null)
        {
            sVirtualObjects = new List<VirtualObject>();
        }
        if (sVirtualObjects.Contains(vObj) == false)
        {
            sVirtualObjects.Add(vObj);
        }
    }

    public static void UpdateVirtualObjectDatabase()
    {
        ClearVirtualObjectDatabase();
        VirtualObject[] objects = GameObject.FindObjectsByType<VirtualObject>(FindObjectsSortMode.None);
        // Add new virtual objects
        foreach (VirtualObject vObj in objects)
        {
            if (sVirtualObjects.Contains(vObj) == false)
                sVirtualObjects.Add(vObj);
        }
    }

    public static void ClearVirtualObjectDatabase()
    {
        sVirtualObjects.Clear();
    }


    public List<VirtualObject> GetNearbyObstacles(float radius = 30f)
    {
        List<VirtualObject> nearbyObjects = new List<VirtualObject>();
        foreach (VirtualObject vObj in sVirtualObjects)
        {
            if (Vector3.Distance(vObj.transform.position, this.mTransform.position) <= radius)
                nearbyObjects.Add(vObj);
        }
        return nearbyObjects;
    }
}
