using CAVAS.UB_MR.Telemetry;
using UnityEditor;

namespace CAVAS.UB_MR.Editor
{
    public static class ExperimentRecordingMenu
    {
        const string MenuPath = "UB-MR/Telemetry/Record Editor experiments";

        [MenuItem(MenuPath)]
        static void Toggle()
        {
            ExperimentRecording.EditorLoggingEnabled = !ExperimentRecording.EditorLoggingEnabled;
            Menu.SetChecked(MenuPath, ExperimentRecording.EditorLoggingEnabled);
        }

        [MenuItem(MenuPath, true)]
        static bool Validate()
        {
            Menu.SetChecked(MenuPath, ExperimentRecording.EditorLoggingEnabled);
            // Pick the setting before Play so each experiment covers a complete session.
            return !EditorApplication.isPlayingOrWillChangePlaymode;
        }
    }
}
