// Copyright 2020 The MathWorks, Inc.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;


#if UNITY_EDITOR

////////////////////////////////////////////////////////////////////////////////
// This editor script controls the output window for the RoadRunnerImporter
// script. It will display all the messages logged to it, with different styles
// for logs, warnings, and errors. Other scripts should use AddLogMessage,
// AddWarningMessage, and AddErrorMessage to add to the output.
namespace MathWorks
{
	public class ImportWindow : EditorWindow
	{

		// Enum for different message types
		enum MessageType
		{
			eLog,
			eWarning,
			eError
		}

		// Class to hold the text and message type
		private class LogMessage
		{
			public string Text;
			public MessageType Type;

			public LogMessage(string mesg, MessageType type)
			{
				Text = mesg;
				Type = type;
			}
		}

		// Singleton to hold the current window
		static ImportWindow CurrWindow;

		// Variables for the window
		Vector2 ScrollPosition;
		static List<LogMessage> LogMessages = new List<LogMessage>();

		// Set the title
		public void OnEnable()
		{
			titleContent.text = "MathWorks RoadRunner Importer Plugin";
		}

		// Display as a floating window, initializing if necessary
		public static void ShowWindow()
		{
			if(!CurrWindow)
				CurrWindow = ScriptableObject.CreateInstance(typeof(ImportWindow)) as ImportWindow;
			
			CurrWindow.minSize = new Vector2(750, 300);
			CurrWindow.ShowUtility();
		}

		// Functions to add messages to the log
		public static void AddLogMessage(string mesg) { LogMessages.Add(new LogMessage(mesg, MessageType.eLog)); }
		public static void AddWarningMessage(string mesg) { LogMessages.Add(new LogMessage(mesg, MessageType.eWarning)); }
		public static void AddErrorMessage(string mesg) { LogMessages.Add(new LogMessage(mesg, MessageType.eError)); }

		// Clear the messages
		public static void ClearLogMessages() { LogMessages.Clear(); }

		// Set the position to the bottom so the most recent message is visible
		public static void ScrollToBottom() { CurrWindow.ScrollPosition.y = CurrWindow.position.height; }

		// Display the RoadRunner output window
		void OnGUI()
		{
			GUILayout.BeginHorizontal();

			// Display logo on left
			GUILayout.Label(MathWorks.Styles.GetLogo(), MathWorks.Styles.IconStyle);

			// Output log section
			GUILayout.BeginVertical(MathWorks.Styles.MessageBoxStyle);

			// Log title
			GUILayout.Label("Output Log", EditorStyles.boldLabel);

			// Scroll view to disply the log messages
			ScrollPosition = GUILayout.BeginScrollView(ScrollPosition, MathWorks.Styles.ScrollStyle);
			foreach (LogMessage mesg in LogMessages)
			{
				GUIStyle currStyle = EditorStyles.label; // Start with default label style
				switch (mesg.Type)
				{
					case MessageType.eWarning:
						currStyle = MathWorks.Styles.WarningMessageStyle;
						break;
					case MessageType.eError:
						currStyle = MathWorks.Styles.ErrorMessageStyle;
						break;
					default:
						break;
				}
				GUILayout.Label("> " + mesg.Text, currStyle);
			}

			GUILayout.EndScrollView();
			GUILayout.EndVertical();
			GUILayout.EndHorizontal();

			// Add close button 
			GUILayout.BeginHorizontal();
			GUILayout.Space(position.width - 65); // Align to right side
			if (GUILayout.Button("Close", EditorStyles.miniButtonRight))
			{
				Close();
			}
			GUILayout.EndHorizontal();
		}

		// Redraw even when window is not in focus 
		void OnInspectorUpdate()
		{
			Repaint();
		}

		// Clear log messages when the window is closed
		private void OnDestroy()
		{
			ClearLogMessages();
		}
	}
}

#endif
