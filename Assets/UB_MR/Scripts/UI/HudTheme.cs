using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UB_MR.UI
{
    // Shared visual language for simulation overlays.
    internal static class HudTheme
    {
        internal static readonly Color Ink = new(.91f, .95f, .98f);
        internal static readonly Color Muted = new(.57f, .65f, .73f);
        internal static readonly Color Cyan = new(.30f, .82f, .94f);
        internal static readonly Color Amber = new(.94f, .70f, .37f);
        internal static readonly Color Purple = new(.72f, .65f, .98f);
        internal static readonly Color Mint = new(.42f, .84f, .69f);
        internal static readonly Color Card = new(.065f, .085f, .115f, .98f);
        internal static readonly Color Line = new(.15f, .19f, .24f);
        internal static readonly Color Background = new(.035f, .047f, .065f, .97f);
        internal static readonly Color Header = new(.06f, .082f, .11f);

        internal static void ScaleCanvas(Canvas canvas)
        {
            var scaler = canvas.GetComponent<CanvasScaler>() ?? canvas.gameObject.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1280, 720);
            scaler.matchWidthOrHeight = 1;
        }

        internal static RectTransform Rect(string name, Transform parent)
        {
            var rect = (RectTransform)new GameObject(name, typeof(RectTransform)).transform;
            rect.SetParent(parent, false);
            return rect;
        }

        internal static void Place(RectTransform rect, float x, float y, float width, float height)
        {
            rect.localScale = Vector3.one;
            rect.anchorMin = rect.anchorMax = rect.pivot = new Vector2(0, 1);
            rect.anchoredPosition = new Vector2(x, -y);
            rect.sizeDelta = new Vector2(width, height);
        }

        internal static Image Surface(RectTransform rect, Color color, bool hit = false)
        {
            var image = rect.GetComponent<Image>() ?? rect.gameObject.AddComponent<Image>();
            image.sprite = null;
            image.color = color;
            image.raycastTarget = hit;
            return image;
        }

        internal static void Frame(RectTransform rect)
        {
            Surface(rect, Background, true);
            var shadow = rect.gameObject.AddComponent<Shadow>();
            shadow.effectColor = new Color(0, 0, 0, .30f);
            shadow.effectDistance = new Vector2(3, -4);
            var border = rect.gameObject.AddComponent<Outline>();
            border.effectColor = new Color(.55f, .68f, .8f, .12f);
            border.effectDistance = Vector2.one;
        }

        internal static TextMeshProUGUI Label(Transform parent, string text, float x, float y,
            float width, float height, float size, Color color, bool bold = false)
        {
            var label = Rect(text, parent).gameObject.AddComponent<TextMeshProUGUI>();
            label.font = TMP_Settings.defaultFontAsset;
            label.text = text; label.fontSize = size; label.color = color;
            label.fontStyle = bold ? FontStyles.Bold : FontStyles.Normal;
            label.raycastTarget = false; label.richText = false;
            Place(label.rectTransform, x, y, width, height);
            return label;
        }

        internal static void Interaction(Selectable control)
        {
            var colors = control.colors;
            colors.normalColor = Color.white;
            colors.highlightedColor = new Color(1.3f, 1.3f, 1.3f);
            colors.selectedColor = new Color(1.5f, 1.5f, 1.5f);
            colors.pressedColor = new Color(.85f, .85f, .85f);
            control.colors = colors;
        }
    }
}
