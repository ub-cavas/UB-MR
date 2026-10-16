using sensor_msgs.msg;

namespace CAVAS.UB_MR.DT.Sensors.Lidar
{
    internal static class PointCloudValidation
    {
        internal static bool HasValidPayload(PointCloud2 message)
        {
            if (message?.Data == null || message.Fields == null || message.Width == 0 || message.Height == 0 ||
                message.Point_step == 0 || message.Width > int.MaxValue || message.Height > int.MaxValue ||
                message.Point_step > int.MaxValue || message.Row_step > int.MaxValue) return false;
            long rowBytes = (long)message.Width * message.Point_step;
            long required = (long)(message.Height - 1) * message.Row_step + rowBytes;
            return message.Row_step >= rowBytes && required <= message.Data.LongLength;
        }
        internal static bool HasCoordinates(PointCloud2 message)
        {
            if (!HasValidPayload(message)) return false;
            bool x = false, y = false, z = false;
            foreach (var field in message.Fields)
            {
                if (field == null || field.Datatype != PointField.FLOAT32) continue;
                if (field.Name != "x" && field.Name != "y" && field.Name != "z") continue;
                if ((long)field.Offset + sizeof(float) > message.Point_step || field.Count < 1) return false;
                if (field.Name == "x") x = true;
                if (field.Name == "y") y = true;
                if (field.Name == "z") z = true;
            }
            return x && y && z;
        }
    }
}
