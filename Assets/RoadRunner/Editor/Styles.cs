// Copyright 2020 The MathWorks, Inc.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

#if UNITY_EDITOR

////////////////////////////////////////////////////////////////////////////////
// Class for RoadRunner plugin Editor window styles.
namespace MathWorks
{
	public class Styles
	{
		public static readonly GUIStyle ScrollStyle = new GUIStyle(GUI.skin.scrollView)
		{
			normal = new GUIStyleState()
			{
				background = MakeBackgroundTexture(new Color(0.7f, 0.7f, 0.7f))
			}
		};

		public static readonly GUIStyle MessageBoxStyle = new GUIStyle(GUI.skin.textArea)
		{
			margin = new RectOffset(10, 10, 10, 10),
			normal = new GUIStyleState()
			{
				background = MakeBackgroundTexture(new Color(0.5f, 0.5f, 0.5f))
			}
		};

		public static readonly GUIStyle IconStyle = new GUIStyle(GUI.skin.label)
		{
			margin = new RectOffset(10, 10, 10, 10),
			fixedWidth = 96
		};

		public static readonly GUIStyle WarningMessageStyle = new GUIStyle(GUI.skin.label)
		{
			normal = new GUIStyleState()
			{
				textColor = Color.yellow
			}
		};

		public static readonly GUIStyle ErrorMessageStyle = new GUIStyle(GUI.skin.label)
		{
			normal = new GUIStyleState()
			{
				textColor = new Color(0.9f, 0.0f, 0.0f)
			}
		};

		// Get logo from plugin Editor folder
		static private Texture RoadRunnerLogo;

		public static Texture GetLogo()
		{
			if (RoadRunnerLogo != null)
				return RoadRunnerLogo;

            RoadRunnerLogo = AssetDatabase.LoadAssetAtPath<Texture>("Assets/RoadRunner/Editor/Logo.png");
			return RoadRunnerLogo;
		}


		// Programmatically create a 1x1 texture to hold the background color
		// This is to work around the fact that GUI backgrounds must be a texture.
		private static Texture2D MakeBackgroundTexture(Color col)
		{
			Color[] pix = new Color[1];
			pix[0] = col;
			Texture2D result = new Texture2D(1, 1);
			result.SetPixels(pix);
			result.Apply();
			return result;
		}
	}
}

#endif
