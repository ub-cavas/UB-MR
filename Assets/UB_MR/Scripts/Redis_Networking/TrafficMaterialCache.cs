using System;
using System.Collections.Generic;
using UnityEngine;

namespace UB_MR.Redis_Networking
{
    /// <summary>Owned by one TrafficRenderer. Source materials are never modified.</summary>
    public sealed class TrafficMaterialCache : IDisposable
    {
        private sealed class Variant
        {
            public Material material;
            public int references;
        }

        private readonly Dictionary<(Material, string, Color32), Variant> variants = new();
        private bool disposed;
        public int Count => variants.Count;

        public Material Acquire(Material source, string property, Color32 color)
        {
            if (disposed) throw new ObjectDisposedException(nameof(TrafficMaterialCache));
            var key = (source, property, color);
            if (!variants.TryGetValue(key, out var variant))
            {
                var material = new Material(source) { name = $"{source.name} (traffic {color.r},{color.g},{color.b})" };
                Color paint = color;
                paint.a = source.GetColor(property).a;
                material.SetColor(property, paint);
                variant = new Variant { material = material };
                variants.Add(key, variant);
            }
            variant.references++;
            return variant.material;
        }

        public void Release(Material source, string property, Color32 color)
        {
            var key = (source, property, color);
            if (!variants.TryGetValue(key, out var variant)) return;
            if (--variant.references > 0) return;
            variants.Remove(key);
            DestroyOwned(variant.material);
        }

        public void Dispose()
        {
            foreach (var variant in variants.Values) DestroyOwned(variant.material);
            variants.Clear();
            disposed = true;
        }

        internal static void DestroyOwned(UnityEngine.Object value)
        {
            if (Application.isPlaying) UnityEngine.Object.Destroy(value);
            else UnityEngine.Object.DestroyImmediate(value);
        }
    }
}
