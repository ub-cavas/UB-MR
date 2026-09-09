using System;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using CAVAS.UI;
using CAVAS.UB_MR.DT.Sensors;

public class MapPanel : Panel
{
    [Header("Virtual Object Detection")]
    [SerializeField] TMP_Dropdown detectionMode;

    /// <summary>
    /// Raised when the operator picks a different virtual object injection method.
    /// </summary>
    public event Action<VirtualObjectDetectionMode> OnDetectionModeChanged;

    bool warnedMissingDetectionModeDropdown;

    [Space]

    [Header("Offsets")]
    [SerializeField] TMP_InputField pos_x;
    [SerializeField] TMP_InputField pos_y;
    [SerializeField] TMP_InputField pos_z;

    [Space]
    [SerializeField] TMP_InputField rot_x;
    [SerializeField] TMP_InputField rot_y;
    [SerializeField] TMP_InputField rot_z;

    // Wired on enable rather than in LoadPanel: the panel can be shown either through
    // UI_Manager or by simply being active at scene start, and the dropdown has to work
    // in both cases.
    void OnEnable()
    {
        if (!TryGetDetectionModeDropdown(out TMP_Dropdown dropdown))
            return;
        PopulateDetectionModeOptions(dropdown);
        dropdown.onValueChanged.RemoveListener(HandleDetectionModeChanged);
        dropdown.onValueChanged.AddListener(HandleDetectionModeChanged);
    }

    void OnDisable()
    {
        if (detectionMode != null)
            detectionMode.onValueChanged.RemoveListener(HandleDetectionModeChanged);
    }

    // -------- VIRTUAL OBJECT DETECTION --------
    public VirtualObjectDetectionMode GetDetectionMode()
    {
        if (detectionMode == null)
            return VirtualObjectDetectionMode.Both;
        int index = Mathf.Clamp(detectionMode.value, 0, VirtualObjectDetectionModes.Ordered.Length - 1);
        return VirtualObjectDetectionModes.Ordered[index];
    }

    /// <summary>
    /// Reflects a mode into the dropdown without raising OnDetectionModeChanged, so callers
    /// pushing their own state in do not get it echoed straight back at them.
    /// </summary>
    public void SetDetectionMode(VirtualObjectDetectionMode inMode)
    {
        if (!TryGetDetectionModeDropdown(out TMP_Dropdown dropdown))
            return;
        PopulateDetectionModeOptions(dropdown);
        dropdown.SetValueWithoutNotify(VirtualObjectDetectionModes.IndexOf(inMode));
        dropdown.RefreshShownValue();
    }

    void PopulateDetectionModeOptions(TMP_Dropdown inDropdown)
    {
        List<string> labels = new List<string>(VirtualObjectDetectionModes.Ordered.Length);
        foreach (VirtualObjectDetectionMode mode in VirtualObjectDetectionModes.Ordered)
            labels.Add(VirtualObjectDetectionModes.DisplayName(mode));

        // Only rebuild when the authored options do not already match, so the current
        // selection survives a panel reload.
        if (inDropdown.options.Count == labels.Count)
        {
            bool matches = true;
            for (int i = 0; i < labels.Count; i++)
            {
                if (inDropdown.options[i].text != labels[i])
                {
                    matches = false;
                    break;
                }
            }
            if (matches)
                return;
        }

        int previous = inDropdown.value;
        inDropdown.ClearOptions();
        inDropdown.AddOptions(labels);
        inDropdown.SetValueWithoutNotify(Mathf.Clamp(previous, 0, labels.Count - 1));
        inDropdown.RefreshShownValue();
    }

    void HandleDetectionModeChanged(int inIndex)
    {
        OnDetectionModeChanged?.Invoke(GetDetectionMode());
    }

    bool TryGetDetectionModeDropdown(out TMP_Dropdown outDropdown)
    {
        outDropdown = detectionMode;
        if (outDropdown != null)
            return true;
        if (!warnedMissingDetectionModeDropdown)
        {
            warnedMissingDetectionModeDropdown = true;
            Debug.LogWarning($"[{name}] No detection mode dropdown assigned; virtual object injection stays on the module's default.");
        }
        return false;
    }

    // -------- GETTERS --------
    public Vector3 GetMapPosition()
    {
        return new Vector3(
            ParseFloat(pos_x.text),
            ParseFloat(pos_y.text),
            ParseFloat(pos_z.text)
        );
    }

    public Quaternion GetMapRotation()
    {
        return Quaternion.Euler(GetMapRotationEuler());
    }

    public Vector3 GetMapRotationEuler()
    {
        return new Vector3(
            ParseFloat(rot_x.text),
            ParseFloat(rot_y.text),
            ParseFloat(rot_z.text)
        );
    }



    // -------- SETTERS --------
    public void SetMapPosition(Vector3 position)
    {
        pos_x.text = position.x.ToString();
        pos_y.text = position.y.ToString();
        pos_z.text = position.z.ToString();
    }

    public void SetMapRotation(Vector3 rotation)
    {
        rot_x.text = rotation.x.ToString();
        rot_y.text = rotation.y.ToString();
        rot_z.text = rotation.z.ToString();
    }

    // -------- HELPER --------
    float ParseFloat(string value)
    {
        float result;
        if (!float.TryParse(value, out result))
        {
            result = 0f;
        }
        return result;
    }
}
