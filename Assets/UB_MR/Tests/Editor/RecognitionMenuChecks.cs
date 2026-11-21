using System;
using System.Linq;
using TMPro;
using CAVAS.UB_MR.Config;
using CAVAS.UB_MR.Modules.MainMenu;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UB_MR.Tests
{
    [InitializeOnLoad]
    public static class RecognitionMenuChecks
    {
        const string Key = "UBMR.RecognitionMenuCheck";
        static double readyAt;
        static int stage;

        static RecognitionMenuChecks()
        {
            EditorApplication.playModeStateChanged += state =>
            {
                if (state == PlayModeStateChange.EnteredPlayMode && SessionState.GetBool(Key, false))
                {
                    readyAt = EditorApplication.timeSinceStartup + 3;
                    stage = 0;
                    EditorApplication.update += Tick;
                }
            };
        }

        public static void Run()
        {
            EditorSceneManager.OpenScene("Assets/UB_MR/Modules/Main-Menu/Main-Menu.unity");
            SessionState.SetBool(Key, true);
            EditorApplication.EnterPlaymode();
        }

        static void Tick()
        {
            if (EditorApplication.timeSinceStartup < readyAt) return;
            try
            {
                if (stage == 0)
                {
                    var menu = UnityEngine.Object.FindFirstObjectByType<AgentEditMenu>(FindObjectsInactive.Include);
                    if (menu == null) throw new Exception("Active agent editor was not found in Main-Menu scene.");
                    menu.SetAgent(new Config.Agent { name = "Recognition preview", recognition = new VirtualObjectRecognitionSettings
                        { mode = VirtualObjectRecognitionMode.LidarModificationAndBoundingBoxInjection, useSimTime = true } });
                    UI_Manager.LoadPanel(menu);
                    var button = menu.transform.Find("Recognition settings").GetComponent<Button>();
                    button.onClick.Invoke();
                    Canvas.ForceUpdateCanvases();
                    if (!menu.transform.Find("Virtual object recognition").gameObject.activeInHierarchy)
                        throw new Exception("Recognition settings did not open.");
                    var panel = menu.transform.Find("Virtual object recognition");
                    var dropdowns = panel.GetComponentsInChildren<TMP_Dropdown>(true);
                    var mode = dropdowns[0];
                    var fields = panel.GetComponentsInChildren<TMP_InputField>(true);
                    if (mode.options.Count != 3) throw new Exception("Expected three recognition choices.");
                    if (!Application.persistentDataPath.StartsWith("/tmp/"))
                        throw new Exception("Menu check requires XDG_CONFIG_HOME under /tmp to isolate saved agents.");
                    for (int i = 0; i < 3; i++)
                    {
                        mode.value = i;
                        if (fields.Any(field => field.interactable != (i != 0)) || dropdowns[1].interactable != (i != 0))
                            throw new Exception("Box field availability mismatch for mode " + i);
                        panel.GetComponentsInChildren<Button>(true).First(control =>
                            control.GetComponentInChildren<TextMeshProUGUI>(true)?.text == "Save settings").onClick.Invoke();
                        var saved = ConfigurationManager.LoadFromJSON("Recognition preview");
                        if ((int)saved.recognition.mode != i) throw new Exception("Mode did not save: " + i);
                        menu.SetAgent(saved);
                        button.onClick.Invoke();
                        if (mode.value != i) throw new Exception("Mode did not reopen: " + i);
                    }
                    fields[0].text = "/test/combined_boxes";
                    fields[1].text = "17";
                    fields[2].text = "123";
                    dropdowns[1].value = 1;
                    panel.GetComponentsInChildren<Button>(true).First(button =>
                        button.GetComponentInChildren<TextMeshProUGUI>(true)?.text == "Save settings").onClick.Invoke();
                    var restored = ConfigurationManager.LoadFromJSON("Recognition preview");
                    if (restored.recognition.mode != VirtualObjectRecognitionMode.LidarModificationAndBoundingBoxInjection ||
                        restored.recognition.boundingBoxTopic != "/test/combined_boxes" || restored.recognition.publishRateHz != 17 ||
                        restored.recognition.detectionRadiusMeters != 123 || !restored.recognition.useSimTime)
                        throw new Exception("Combined settings did not save/reopen.");
                    menu.SetAgent(restored);
                    button.onClick.Invoke();
                    if (mode.value != 2 || fields[0].text != "/test/combined_boxes")
                        throw new Exception("Combined controls did not reopen.");
                    ConfigurationManager.RemoveAgent("Recognition preview");
                    stage = 1;
                    readyAt = EditorApplication.timeSinceStartup + 2;
                }
                else if (stage == 1)
                {
                    var menu = UnityEngine.Object.FindFirstObjectByType<AgentEditMenu>();
                    var canvas = menu.GetComponentInParent<Canvas>();
                    var cameraObject = new GameObject("Menu capture camera");
                    var camera = cameraObject.AddComponent<Camera>();
                    camera.orthographic = true;
                    camera.clearFlags = CameraClearFlags.SolidColor;
                    camera.backgroundColor = Color.black;
                    var target = new RenderTexture(1200, 1000, 24);
                    camera.targetTexture = target;
                    canvas.renderMode = RenderMode.ScreenSpaceCamera;
                    canvas.worldCamera = camera;
                    canvas.planeDistance = 1;
                    Canvas.ForceUpdateCanvases();
                    camera.Render();
                    var previous = RenderTexture.active;
                    RenderTexture.active = target;
                    var texture = new Texture2D(1200, 1000, TextureFormat.RGB24, false);
                    texture.ReadPixels(new Rect(0, 0, 1200, 1000), 0, 0);
                    texture.Apply();
                    System.IO.File.WriteAllBytes("/tmp/ub-mr-recognition-menu.png", texture.EncodeToPNG());
                    RenderTexture.active = previous;
                    camera.targetTexture = null;
                    target.Release();
                    UnityEngine.Object.Destroy(texture);
                    UnityEngine.Object.Destroy(target);
                    UnityEngine.Object.Destroy(cameraObject);
                    stage = 2;
                    readyAt = EditorApplication.timeSinceStartup + 3;
                }
                else
                {
                    if (!System.IO.File.Exists("/tmp/ub-mr-recognition-menu.png"))
                        throw new Exception("Menu screenshot was not captured.");
                    SessionState.SetBool(Key, false);
                    EditorApplication.update -= Tick;
                    Debug.Log("UB-MR menu checks PASSED; screenshot: /tmp/ub-mr-recognition-menu.png");
                    EditorApplication.Exit(0);
                }
            }
            catch (Exception exception)
            {
                SessionState.SetBool(Key, false);
                EditorApplication.update -= Tick;
                Debug.LogException(exception);
                EditorApplication.Exit(1);
            }
        }
    }
}
