// Copyright 2020 The MathWorks, Inc.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

#if UNITY_EDITOR

////////////////////////////////////////////////////////////////////////////////
// Scriptable Object to store and save RoadRunner import settings
namespace MathWorks
{
	public class ImportSettings : ScriptableObject
	{
		// Save in plugin editor folder
		public const string SettingsFilePath = "Assets/RoadRunner/Editor/ImportSettings.asset";

		// Default to rotating the scene 180 to match RoadRunner when looking top-down
		[SerializeField]
		internal bool RotateScene180 = true;

		internal static ImportSettings GetOrCreateSettings()
		{
			var settings = AssetDatabase.LoadAssetAtPath<ImportSettings>(SettingsFilePath);

			if (settings == null)
			{
				settings = ScriptableObject.CreateInstance<ImportSettings>();
				settings.RotateScene180 = true;
				AssetDatabase.CreateAsset(settings, SettingsFilePath);
				AssetDatabase.SaveAssets();
			}
			return settings;
		}
	}
}

#endif
