using UnityEngine;
using ROS2;

public class ROS2_Bridge
{
    public static ROS2UnityCore ROS_CORE;

    ROS2_Bridge()
    {
        if (ROS_CORE == null)
        {
            ROS_CORE = new ROS2UnityCore();
        }
    }
}
