using UnityEngine;
using ROS2;

public static class ROS2_Bridge
{
    // backing field
    private static ROS2UnityCore _rosCore;

    public static ROS2UnityCore ROS_CORE
    {
        get
        {
            if (_rosCore == null)
            {
                _rosCore = new ROS2UnityCore();
            }
            return _rosCore;
        }
    }
}
