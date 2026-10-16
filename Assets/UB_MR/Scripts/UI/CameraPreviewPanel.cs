using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UB_MR.UI
{
    internal sealed class CameraPreviewPanel : MonoBehaviour
    {
        RawImage preview;
        AspectRatioFitter aspect;
        TextMeshProUGUI status;
        Texture lastTexture;
        int lastWidth, lastHeight;

        internal static void Attach(RawImage image)
        {
            if (image == null) return;
            var canvas = image.GetComponentInParent<Canvas>();
            if (canvas != null) HudTheme.ScaleCanvas(canvas);
            var panel = HudTheme.Rect("Camera preview HUD", image.transform.parent);
            var view = panel.gameObject.AddComponent<CameraPreviewPanel>();
            var window = new HudWindow(panel, "Camera preview", "RUNTIME  /  VISION", HudTheme.Mint, 384, 312);
            var surface = HudTheme.Rect("Image frame", window.Body);
            HudTheme.Place(surface, 14, 14, 356, 200); HudTheme.Surface(surface, HudTheme.Card);
            view.status = HudTheme.Label(window.Body, "Waiting for camera image", 18, 222, 348, 18, 10, HudTheme.Muted);
            image.transform.SetParent(surface, false);
            image.raycastTarget = false;
            view.preview = image;
            view.aspect = image.GetComponent<AspectRatioFitter>() ?? image.gameObject.AddComponent<AspectRatioFitter>();
            view.aspect.aspectMode = AspectRatioFitter.AspectMode.FitInParent;
            view.Refresh();
        }

        void LateUpdate() => Refresh();

        void Refresh()
        {
            if (preview == null) return;
            var texture = preview.texture;
            preview.enabled = texture != null;
            if (texture == lastTexture && (texture == null || (texture.width == lastWidth && texture.height == lastHeight))) return;
            lastTexture = texture;
            if (texture == null) { status.text = "Waiting for camera image"; return; }
            lastWidth = texture.width; lastHeight = texture.height;
            aspect.aspectRatio = (float)lastWidth / Mathf.Max(1, lastHeight);
            status.text = $"{lastWidth} × {lastHeight}  ·  Camera image";
        }
    }
}
