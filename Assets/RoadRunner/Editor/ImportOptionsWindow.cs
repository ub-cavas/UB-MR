// Copyright 2020 The MathWorks, Inc.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

#if UNITY_EDITOR

////////////////////////////////////////////////////////////////////////////////
// Import options window
namespace MathWorks
{
	public class ImportOptionsWindow : EditorWindow
	{
		static ImportSettings ImportSettings_;
		

		[MenuItem("Window/RoadRunner/Import Options")]
		public static void ShowWindow()
		{
			ImportSettings_ = MathWorks.ImportSettings.GetOrCreateSettings();
			var window = EditorWindow.GetWindow(typeof(ImportOptionsWindow));

			window.minSize = new Vector2(500, 200);
			window.maxSize = new Vector2(500, 200);
		}

		// Set the title
		public void OnEnable()
		{
			titleContent.text = "Import Settings";
		}

		private void OnGUI()
		{
			// Try to load existing settings
			if (ImportSettings_ == null)
			{
				ImportSettings_ = MathWorks.ImportSettings.GetOrCreateSettings();
			}

			GUILayout.BeginHorizontal();

			// Display logo on left
			GUILayout.Label(MathWorks.Styles.GetLogo(), MathWorks.Styles.IconStyle);

			GUILayout.BeginVertical();

			GUILayout.Label("Import Settings", EditorStyles.boldLabel);
			ImportSettings_.RotateScene180 = EditorGUILayout.Toggle(new GUIContent("Rotate Scene",
				"Automatically rotate the scene 180 degrees so Z+ is north"), ImportSettings_.RotateScene180);

			GUILayout.EndVertical();

			GUILayout.EndHorizontal();
		}
	}
}

#endif