using UnityEngine;
using ROS2;

public class MapData : MonoBehaviour
{
    [SerializeField] string origin_topic = "map_origin";
    [SerializeField] double origin_latitude = 0; 
    [SerializeField] double origin_longitude = 0; 
    [SerializeField] double origin_altitude = 0;


    ROS2Node mNode;
    
    IPublisher<sensor_msgs.msg.NavSatFix> mMapOriginPublisher;

    // To Do: Turn this node into a Service

    void Start()
    {
        if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
        {
            this.mNode = ROS2_Bridge.ROS_CORE.CreateNode("Map_Info");
            this.mMapOriginPublisher = this.mNode.CreatePublisher<sensor_msgs.msg.NavSatFix>(origin_topic);
        }
    }

    void Update()
    {
       
        if (ROS2_Bridge.ROS_CORE.Ok())
        {
            PublishMapOrigin();
        }
       
    }


    void PublishMapOrigin()
    {
        sensor_msgs.msg.NavSatFix msg = new sensor_msgs.msg.NavSatFix();
        msg.Latitude = origin_latitude;
        msg.Longitude = origin_longitude;
        msg.Altitude = origin_altitude;
        this.mMapOriginPublisher.Publish(msg);
    }

    // TODO: Bounding Box publisher of object within 30m of the ego-vehicle
    void PublishBoundingBoxes()
    {
        
    }

    // TODO: Publish disparity between expected location (GNSS) and predicted location (Rigidbody Physics)
    // this could be used later in the pipeline to correct the error via a Kalman Filter or similar
    void PublishError()
    {

    }
}

