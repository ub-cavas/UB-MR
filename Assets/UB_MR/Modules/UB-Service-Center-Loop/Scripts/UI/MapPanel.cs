using UnityEngine;
using TMPro;
using CAVAS.UI;
using CAVAS.UB_MR.UI;
using UnityEngine.UI;

public class MapPanel : Panel
{
    [Header("Offsets")]
    [SerializeField] TMP_InputField pos_x;
    [SerializeField] TMP_InputField pos_y;
    [SerializeField] TMP_InputField pos_z;

    [Space]
    [SerializeField] TMP_InputField rot_x;
    [SerializeField] TMP_InputField rot_y;
    [SerializeField] TMP_InputField rot_z;

    bool styled;

    void Start() => ApplyHudStyle();

    internal void ApplyHudStyle()
    {
        if (styled) return;
        styled = true;
        var canvas = GetComponentInParent<Canvas>();
        if (canvas != null) HudTheme.ScaleCanvas(canvas);
        // Keep the serialized input fields and their listeners; replace only their layout.
        foreach (Transform child in transform) child.gameObject.SetActive(false);
        var window = new HudWindow((RectTransform)transform, "Map alignment", "RUNTIME  /  WORLD", HudTheme.Purple, 352, 312);
        AddVectorRow(window.Body, "POSITION", "meters", 14, new[] { pos_x, pos_y, pos_z });
        AddVectorRow(window.Body, "ROTATION", "degrees", 118, new[] { rot_x, rot_y, rot_z });
        HudTheme.Label(window.Body, "Offsets apply immediately to the simulation.", 18, 222, 316, 18, 10, HudTheme.Muted);
    }

    static void AddVectorRow(RectTransform parent, string name, string unit, float y, TMP_InputField[] fields)
    {
        var card = HudTheme.Rect(name, parent);
        HudTheme.Place(card, 14, y, 324, 92); HudTheme.Surface(card, HudTheme.Card);
        HudTheme.Label(card, name, 12, 10, 210, 18, 11, HudTheme.Purple, true);
        var units = HudTheme.Label(card, unit, 226, 10, 86, 18, 10, HudTheme.Muted);
        units.alignment = TextAlignmentOptions.TopRight;
        for (int i = 0; i < fields.Length; i++)
        {
            float x = 12 + i * 103;
            HudTheme.Label(card, new[] { "X", "Y", "Z" }[i], x, 32, 94, 14, 9, HudTheme.Muted, true);
            var field = fields[i];
            field.transform.SetParent(card, false); field.gameObject.SetActive(true);
            HudTheme.Place((RectTransform)field.transform, x, 49, 94, 31);
            field.targetGraphic = HudTheme.Surface((RectTransform)field.transform, HudTheme.Line, true);
            HudTheme.Interaction(field);
            field.customCaretColor = true; field.caretColor = HudTheme.Cyan;
            field.selectionColor = new Color(.30f, .82f, .94f, .3f);
            foreach (var label in field.GetComponentsInChildren<TMP_Text>(true))
            {
                label.font = TMP_Settings.defaultFontAsset;
                label.fontSize = 14; label.enableAutoSizing = false;
                label.fontStyle = FontStyles.Normal; label.color = HudTheme.Ink;
                label.alignment = TextAlignmentOptions.MidlineLeft;
            }
            field.textViewport.anchorMin = Vector2.zero; field.textViewport.anchorMax = Vector2.one;
            field.textViewport.offsetMin = new Vector2(8, 3); field.textViewport.offsetMax = new Vector2(-8, -3);
        }
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
