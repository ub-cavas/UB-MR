using UnityEngine;
using geometry_msgs.msg;

public static class Ros2UnityConversions
{
    /// <summary>
    /// Convert a Unity Vector3 to a ROS2 Point.
    /// </summary>
    public static UnityEngine.Vector3 UnityToRosPoint(UnityEngine.Vector3 unityPoint)
    {
        return new UnityEngine.Vector3
        {
            x = unityPoint.z,      // Unity forward → ROS X (forward)
            y = -unityPoint.x,     // Unity right → ROS Y (left)
            z = unityPoint.y       // Unity up → ROS Z (up)
        };
    }

    /// <summary>
    /// Convert a ROS2 Point to a Unity Vector3
    /// </summary>
    public static UnityEngine.Vector3 RosToUnityPoint(UnityEngine.Vector3 rosPoint)
    {
        return new UnityEngine.Vector3(
            -rosPoint.y,   // ROS left → Unity right
            rosPoint.z,     // ROS up → Unity up
            rosPoint.x      // ROS forward → Unity forward
        );
    }
}

