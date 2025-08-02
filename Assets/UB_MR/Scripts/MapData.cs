using UnityEngine;
using ROS2;
using robot_localization.srv;

public class MapData : MonoBehaviour
{
    [SerializeField] double origin_latitude = 0; 
    [SerializeField] double origin_longitude = 0; 
    [SerializeField] double origin_altitude = 0;


    ROS2Node mNode;

    // To Do: Turn this node into a Service

    void Start()
    {
        if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
        {
            this.mNode = ROS2_Bridge.ROS_CORE.CreateNode("Unity_Map");
            SetDatum();
        }
    }

    void SetDatum()
    {
        IClient<SetDatum_Request, SetDatum_Response> setDatumClient = this.mNode.CreateClient<SetDatum_Request, SetDatum_Response>("/datum");
        SetDatum_Request request = new SetDatum_Request();
        request.Geo_pose.Position.Latitude = origin_latitude;
        request.Geo_pose.Position.Longitude = origin_longitude;
        request.Geo_pose.Position.Altitude = origin_altitude;
        request.Geo_pose.Orientation.X = 0;
        request.Geo_pose.Orientation.Y = 0;
        request.Geo_pose.Orientation.Z = 0;
        request.Geo_pose.Orientation.W = 1;
        var response = setDatumClient.Call(request);
        if (response != null)
        {
            Debug.Log("Datum set successfully");
        }
    }
}

