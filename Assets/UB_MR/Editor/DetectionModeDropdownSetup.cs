using System.Collections.Generic;
using CAVAS.UB_MR.DT.Sensors;
using TMPro;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace CAVAS.UB_MR.EditorTools
{
    /// <summary>
    /// One-click setup for the runtime virtual object detection mode selector.
    /// The map panel lives in the scene as an instance of the shared Panel prefab, so the
    /// dropdown is added to the scene instance (as a prefab override) rather than to the
    /// prefab asset, which every other panel also derives from.
    /// </summary>
    public static class DetectionModeDropdownSetup
    {
        const string MENU_PATH = "UB-MR/Setup/Add Detection Mode Dropdown";
        const string DROPDOWN_NAME = "DetectionMode_Dropdown";
        const string LABEL_NAME = "DetectionMode_Label";
        const string FIELD_NAME = "detectionMode";

        const string STANDARD_SPRITE_PATH = "UI/Skin/UISprite.psd";
        const string BACKGROUND_SPRITE_PATH = "UI/Skin/Background.psd";
        const string INPUT_FIELD_BACKGROUND_PATH = "UI/Skin/InputFieldBackground.psd";
        const string KNOB_PATH = "UI/Skin/Knob.psd";
        const string CHECKMARK_PATH = "UI/Skin/Checkmark.psd";
        const string DROPDOWN_ARROW_PATH = "UI/Skin/DropdownArrow.psd";
        const string MASK_PATH = "UI/Skin/UIMask.psd";

        [MenuItem(MENU_PATH)]
        public static void AddDetectionModeDropdown()
        {
            MapPanel panel = Object.FindFirstObjectByType<MapPanel>(FindObjectsInactive.Include);
            if (panel == null)
            {
                EditorUtility.DisplayDialog(
                    "Detection Mode Dropdown",
                    "No MapPanel was found in the open scene. Open the scene that contains the map panel and try again.",
                    "OK");
                return;
            }

            SerializedObject serializedPanel = new SerializedObject(panel);
            SerializedProperty field = serializedPanel.FindProperty(FIELD_NAME);
            if (field == null)
            {
                Debug.LogError($"MapPanel has no serialized field named '{FIELD_NAME}'. Scripts may still be compiling.");
                return;
            }

            if (field.objectReferenceValue != null)
            {
                Selection.activeObject = field.objectReferenceValue;
                EditorGUIUtility.PingObject(field.objectReferenceValue);
                Debug.Log("Detection mode dropdown is already wired up; selecting the existing one.");
                return;
            }

            GameObject dropdownGO = TMP_DefaultControls.CreateDropdown(GetStandardResources());
            dropdownGO.name = DROPDOWN_NAME;
            Undo.RegisterCreatedObjectUndo(dropdownGO, "Add Detection Mode Dropdown");

            GameObject labelGO = TMP_DefaultControls.CreateText(GetStandardResources());
            labelGO.name = LABEL_NAME;
            Undo.RegisterCreatedObjectUndo(labelGO, "Add Detection Mode Label");
            TextMeshProUGUI label = labelGO.GetComponent<TextMeshProUGUI>();
            if (label != null)
            {
                label.text = "Detection Mode";
                label.fontSize = 18f;
                label.alignment = TextAlignmentOptions.MidlineLeft;
            }

            Vector2 anchoredPosition = NextFreeSlot(panel.transform as RectTransform);
            Attach(labelGO, panel.transform, anchoredPosition, new Vector2(200f, 30f));
            Attach(dropdownGO, panel.transform, anchoredPosition + new Vector2(0f, -34f), new Vector2(260f, 34f));

            // Preload the options so the panel reads correctly in the editor. MapPanel also
            // rebuilds them at runtime, so the two can never drift apart.
            TMP_Dropdown dropdown = dropdownGO.GetComponent<TMP_Dropdown>();
            List<string> labels = new List<string>();
            foreach (VirtualObjectDetectionMode mode in VirtualObjectDetectionModes.Ordered)
                labels.Add(VirtualObjectDetectionModes.DisplayName(mode));
            dropdown.ClearOptions();
            dropdown.AddOptions(labels);
            dropdown.SetValueWithoutNotify(VirtualObjectDetectionModes.IndexOf(VirtualObjectDetectionMode.Both));
            dropdown.RefreshShownValue();

            Undo.RecordObject(panel, "Wire Detection Mode Dropdown");
            field.objectReferenceValue = dropdown;
            serializedPanel.ApplyModifiedProperties();
            PrefabUtility.RecordPrefabInstancePropertyModifications(panel);

            EditorSceneManager.MarkSceneDirty(panel.gameObject.scene);
            Selection.activeGameObject = dropdownGO;
            EditorGUIUtility.PingObject(dropdownGO);
            Debug.Log($"Added '{DROPDOWN_NAME}' under '{panel.name}' and wired it to MapPanel.{FIELD_NAME}. Save the scene to keep it.");
        }

        static void Attach(GameObject inChild, Transform inParent, Vector2 inAnchoredPosition, Vector2 inSize)
        {
            Undo.SetTransformParent(inChild.transform, inParent, "Parent Detection Mode UI");
            inChild.layer = inParent.gameObject.layer;

            RectTransform rect = inChild.GetComponent<RectTransform>();
            rect.localScale = Vector3.one;
            rect.localRotation = Quaternion.identity;
            rect.anchorMin = new Vector2(0f, 1f);
            rect.anchorMax = new Vector2(0f, 1f);
            rect.pivot = new Vector2(0f, 1f);
            rect.sizeDelta = inSize;
            rect.anchoredPosition3D = new Vector3(inAnchoredPosition.x, inAnchoredPosition.y, 0f);
        }

        /// <summary>
        /// Finds empty space beneath the panel's existing controls so the new widget does not
        /// land on top of the offset fields, whatever layout the panel happens to use.
        /// </summary>
        static Vector2 NextFreeSlot(RectTransform inPanel)
        {
            const float LEFT_MARGIN = 40f;
            const float GAP = 24f;

            if (inPanel == null)
                return new Vector2(LEFT_MARGIN, -60f);

            float lowest = 0f;
            bool found = false;
            for (int i = 0; i < inPanel.childCount; i++)
            {
                RectTransform child = inPanel.GetChild(i) as RectTransform;
                if (child == null)
                    continue;

                // Top edge of the child expressed in the panel's top-left space.
                float top = child.anchoredPosition.y + (1f - child.pivot.y) * child.rect.height;
                float bottom = top - child.rect.height;
                if (!found || bottom < lowest)
                {
                    lowest = bottom;
                    found = true;
                }
            }

            return new Vector2(LEFT_MARGIN, found ? lowest - GAP : -60f);
        }

        static TMP_DefaultControls.Resources GetStandardResources()
        {
            TMP_DefaultControls.Resources resources = new TMP_DefaultControls.Resources();
            resources.standard = AssetDatabase.GetBuiltinExtraResource<Sprite>(STANDARD_SPRITE_PATH);
            resources.background = AssetDatabase.GetBuiltinExtraResource<Sprite>(BACKGROUND_SPRITE_PATH);
            resources.inputField = AssetDatabase.GetBuiltinExtraResource<Sprite>(INPUT_FIELD_BACKGROUND_PATH);
            resources.knob = AssetDatabase.GetBuiltinExtraResource<Sprite>(KNOB_PATH);
            resources.checkmark = AssetDatabase.GetBuiltinExtraResource<Sprite>(CHECKMARK_PATH);
            resources.dropdown = AssetDatabase.GetBuiltinExtraResource<Sprite>(DROPDOWN_ARROW_PATH);
            resources.mask = AssetDatabase.GetBuiltinExtraResource<Sprite>(MASK_PATH);
            return resources;
        }
    }
}
