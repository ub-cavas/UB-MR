using System;

namespace CAVAS.UB_MR.DT.Sensors
{
    /// <summary>
    /// Selects how virtual objects are injected into the perception stack.
    /// Treated as a bit field so both methods can run at once for A/B comparison.
    /// </summary>
    [Flags]
    public enum VirtualObjectDetectionMode
    {
        /// <summary>No virtual objects reach the stack. Useful as a control run.</summary>
        None = 0,
        /// <summary>Carve virtual objects into the live LiDAR point cloud via SDF raymarching.</summary>
        LidarModification = 1 << 0,
        /// <summary>Publish virtual objects directly as ground truth detected objects.</summary>
        BoundingBoxInjection = 1 << 1,
        /// <summary>Run both injection methods simultaneously.</summary>
        Both = LidarModification | BoundingBoxInjection,
    }

    public static class VirtualObjectDetectionModes
    {
        /// <summary>
        /// Canonical ordering used to populate runtime dropdowns. Keeping this in one
        /// place means the UI and any serialized default always agree on the option list.
        /// </summary>
        public static readonly VirtualObjectDetectionMode[] Ordered =
        {
            VirtualObjectDetectionMode.LidarModification,
            VirtualObjectDetectionMode.BoundingBoxInjection,
            VirtualObjectDetectionMode.Both,
            VirtualObjectDetectionMode.None,
        };

        public static string DisplayName(VirtualObjectDetectionMode inMode)
        {
            switch (inMode)
            {
                case VirtualObjectDetectionMode.LidarModification: return "LiDAR Modification";
                case VirtualObjectDetectionMode.BoundingBoxInjection: return "Bounding Box Injection";
                case VirtualObjectDetectionMode.Both: return "Both";
                case VirtualObjectDetectionMode.None: return "None";
                default: return inMode.ToString();
            }
        }

        public static bool Includes(this VirtualObjectDetectionMode inMode, VirtualObjectDetectionMode inFlag)
        {
            return (inMode & inFlag) == inFlag && inFlag != VirtualObjectDetectionMode.None;
        }

        public static int IndexOf(VirtualObjectDetectionMode inMode)
        {
            for (int i = 0; i < Ordered.Length; i++)
            {
                if (Ordered[i] == inMode)
                    return i;
            }
            return 0;
        }
    }
}
