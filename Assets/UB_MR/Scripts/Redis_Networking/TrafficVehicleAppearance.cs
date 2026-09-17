using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

namespace UB_MR.Redis_Networking
{
    public sealed class TrafficVehicleAppearance : MonoBehaviour
    {
        [Serializable]
        public struct PaintBinding
        {
            public Renderer renderer;
            public int materialIndex;
            public string colorProperty;
        }

        [SerializeField] private PaintBinding[] paintBindings = Array.Empty<PaintBinding>();
        private Material[] originals;
        private TrafficMaterialCache cache;
        private Color32 appliedColor;
        private bool hasColor;
        public IReadOnlyList<PaintBinding> PaintBindings => paintBindings;
        public static string Property(PaintBinding binding) =>
            string.IsNullOrEmpty(binding.colorProperty) ? "_BaseColor" : binding.colorProperty;

        public bool Validate(out string error)
        {
            if (paintBindings == null || paintBindings.Length == 0)
            { error = "No paint bindings configured."; return false; }
            var slots = new HashSet<(Renderer, int)>();
            foreach (var binding in paintBindings)
            {
                if (binding.renderer == null || !binding.renderer.transform.IsChildOf(transform))
                { error = "Paint renderer must belong to this vehicle."; return false; }
                var materials = binding.renderer.sharedMaterials;
                if (binding.materialIndex < 0 || binding.materialIndex >= materials.Length
                    || materials[binding.materialIndex] == null
                    || !materials[binding.materialIndex].HasProperty(Property(binding)))
                { error = $"Invalid paint slot/property on {binding.renderer.name}."; return false; }
                if (!slots.Add((binding.renderer, binding.materialIndex)))
                { error = "Paint slots must not be bound twice."; return false; }
            }
            error = null;
            return true;
        }

        public static bool TryParseColor(string value, out Color32 color)
        {
            color = default;
            var parts = value?.Split(',');
            if (parts == null || parts.Length != 3) return false;
            if (!byte.TryParse(parts[0].Trim(), NumberStyles.None, CultureInfo.InvariantCulture, out byte r)
                || !byte.TryParse(parts[1].Trim(), NumberStyles.None, CultureInfo.InvariantCulture, out byte g)
                || !byte.TryParse(parts[2].Trim(), NumberStyles.None, CultureInfo.InvariantCulture, out byte b)) return false;
            color = new Color32(r, g, b, 255);
            return true;
        }

        public void ApplyColor(string value, TrafficMaterialCache owner)
        {
            if (originals == null)
            {
                if (!Validate(out string error))
                { Debug.LogError($"{name}: {error}", this); return; }
                originals = new Material[paintBindings.Length];
                for (int i = 0; i < originals.Length; i++)
                    originals[i] = paintBindings[i].renderer.sharedMaterials[paintBindings[i].materialIndex];
            }
            bool valid = TryParseColor(value, out var color);
            if (hasColor && valid && color.Equals(appliedColor) && cache == owner) return;
            ReleaseColor();
            if (!valid) return;
            cache = owner;
            appliedColor = color;
            for (int i = 0; i < paintBindings.Length; i++)
            {
                var binding = paintBindings[i];
                SetMaterial(binding, cache.Acquire(originals[i], Property(binding), color));
            }
            hasColor = true;
        }

        public void ReleaseColor()
        {
            if (!hasColor) return;
            for (int i = 0; i < paintBindings.Length; i++)
            {
                var binding = paintBindings[i];
                if (binding.renderer != null) SetMaterial(binding, originals[i]);
                cache.Release(originals[i], Property(binding), appliedColor);
            }
            hasColor = false;
            cache = null;
        }

        private static void SetMaterial(PaintBinding binding, Material material)
        {
            var materials = binding.renderer.sharedMaterials;
            materials[binding.materialIndex] = material;
            binding.renderer.sharedMaterials = materials;
        }

        private void OnDisable() => ReleaseColor();
        private void OnDestroy() => ReleaseColor();
    }
}
