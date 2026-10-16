using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UB_MR.UI
{
    // Reusable left-side shell; collapsing leaves the content and its state alive.
    internal sealed class HudWindow
    {
        readonly RectTransform panel, header, accent;
        readonly TextMeshProUGUI eyebrow, title, toggle;
        readonly float width, height;
        internal RectTransform Body { get; }
        internal bool Expanded { get; private set; } = true;

        internal HudWindow(RectTransform panel, string title, string eyebrow, Color color, float width, float height)
        {
            this.panel = panel; this.width = width; this.height = height;
            HudTheme.Place(panel, 18, 18, width, height);
            HudTheme.Frame(panel);
            header = HudTheme.Rect("Header", panel);
            HudTheme.Place(header, 0, 0, width, 64);
            var button = header.gameObject.AddComponent<Button>();
            button.targetGraphic = HudTheme.Surface(header, HudTheme.Header, true);
            HudTheme.Interaction(button); button.onClick.AddListener(Toggle);
            accent = HudTheme.Rect("Accent", header);
            HudTheme.Place(accent, 0, 0, width, 2); HudTheme.Surface(accent, color);
            this.eyebrow = HudTheme.Label(header, eyebrow, 16, 12, width - 60, 14, 9, HudTheme.Muted);
            this.eyebrow.characterSpacing = 1.8f;
            this.title = HudTheme.Label(header, title, 16, 28, width - 60, 28, 22, HudTheme.Ink, true);
            toggle = HudTheme.Label(header, "−", width - 42, 21, 28, 30, 22, HudTheme.Muted);
            toggle.alignment = TextAlignmentOptions.Center;
            Body = HudTheme.Rect("Body", panel);
            HudTheme.Place(Body, 0, 64, width, height - 64);
        }

        internal void Toggle()
        {
            Expanded = !Expanded;
            Body.gameObject.SetActive(Expanded); eyebrow.gameObject.SetActive(Expanded);
            float w = Expanded ? width : 200;
            panel.sizeDelta = new Vector2(w, Expanded ? height : 42);
            header.sizeDelta = new Vector2(w, Expanded ? 64 : 42);
            accent.sizeDelta = new Vector2(w, 2);
            title.fontSize = Expanded ? 22 : 15;
            HudTheme.Place(title.rectTransform, 16, Expanded ? 28 : 10, w - 60, Expanded ? 28 : 24);
            HudTheme.Place(toggle.rectTransform, w - 42, Expanded ? 21 : 5, 28, 30);
            toggle.text = Expanded ? "−" : "+";
        }
    }
}
