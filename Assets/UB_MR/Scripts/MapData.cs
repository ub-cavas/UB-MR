using UnityEngine;
using ROS2;

public class MapData : MonoBehaviour
{
    [SerializeField] string origin_topic = "map_origin";
    [SerializeField] string origin = "GNSS origin data"; // Placeholder for GNSS origin data


    ROS2Node mNode;
    
    IPublisher<std_msgs.msg.String> mMapOriginPublisher;

    // To Do: Turn this node into a Service

    void Start()
    {
        if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
        {
            this.mNode = ROS2_Bridge.ROS_CORE.CreateNode("Map_Info");
            this.mMapOriginPublisher = this.mNode.CreatePublisher<std_msgs.msg.String>(origin_topic);
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
        std_msgs.msg.String msg = new std_msgs.msg.String();
        msg.Data = origin;
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

