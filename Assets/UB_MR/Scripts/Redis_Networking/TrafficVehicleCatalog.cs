using System;
using System.Collections.Generic;
using UnityEngine;

namespace UB_MR.Redis_Networking
{
    [CreateAssetMenu(menuName = "UB-MR/Traffic vehicle catalog")]
    public sealed class TrafficVehicleCatalog : ScriptableObject
    {
        [Serializable]
        public struct Entry
        {
            public string blueprintId;
            public GameObject prefab;
        }

        [SerializeField] private Entry[] entries = Array.Empty<Entry>();
        private Dictionary<string, GameObject> lookup;
        private string validationError;
        public IReadOnlyList<Entry> Entries => entries ?? Array.Empty<Entry>();

        public bool Validate(out string error)
        {
            var ids = new HashSet<string>(StringComparer.Ordinal);
            foreach (var entry in entries ?? Array.Empty<Entry>())
            {
                if (string.IsNullOrWhiteSpace(entry.blueprintId))
                { error = "Catalog contains an empty blueprint ID."; return false; }
                if (!ids.Add(entry.blueprintId))
                { error = $"Duplicate blueprint ID: {entry.blueprintId}"; return false; }
                if (entry.prefab == null)
                { error = $"Missing prefab: {entry.blueprintId}"; return false; }
            }
            error = null;
            return true;
        }

        public bool Initialize(out string error)
        {
            if (lookup == null)
            {
                lookup = new Dictionary<string, GameObject>(StringComparer.Ordinal);
                if (Validate(out validationError))
                    foreach (var entry in entries ?? Array.Empty<Entry>())
                        lookup.Add(entry.blueprintId, entry.prefab);
            }
            error = validationError;
            return error == null;
        }

        public bool TryResolve(string blueprintId, out GameObject prefab)
        {
            prefab = null;
            return Initialize(out _) && !string.IsNullOrEmpty(blueprintId)
                && lookup.TryGetValue(blueprintId, out prefab) && prefab != null;
        }

        private void OnEnable() => lookup = null;
        private void OnValidate() => lookup = null;
    }
}
