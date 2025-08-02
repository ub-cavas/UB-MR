// Copyright 2020 The MathWorks, Inc.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Xml;
using System.Xml.Serialization;
using System.IO;
using System.Text.RegularExpressions;
using UnityEngine.Rendering;

using UnityEngine.Experimental.Rendering;

#if UNITY_EDITOR

////////////////////////////////////////////////////////////////////////////////
// This editor script will automatically fix up fbx files from RoadRunner
// into Unity, and adds colliders to all geometry.
////////////////////////////////////////////////////////////////////////////////
// Asset processing class
[InitializeOnLoad]
public class RoadRunnerImporter : AssetPostprocessor
{

#if !UNITY_2017_2_OR_NEWER
	static RoadRunnerImporter()
	{
		Debug.LogWarning("RoadRunnerImporter only supports Unity version 2017.2 and newer. Models may not be imported correctly.");
	}
#endif

	// Version number string
	static string VersionName = "1.4.1";

	// Max supported rrdata version
	static int PluginVersion = 5;

	static int MarkingRenderQueueBase = 2100;
	static int TransparentRenderQueue = 4500;

	// Maps from fbx paths to their metadata for the current import (cleared when asset progress bar finished)
	static Dictionary<string, MathWorks.Metadata> FbxFileToMetadataMap = new Dictionary<string, MathWorks.Metadata>();

	// Maps texture file names to their texture info for the current import (cleared when asset progress bar finished)
	static Dictionary<string, MathWorks.TextureInfo> NameToTextureInfoMap = new Dictionary<string, MathWorks.TextureInfo>();

	// Set of directory names to check if the metadata files it contains have already been processed
	static HashSet<string> ProcessedDirectories = new HashSet<string>();

	// Error Flags
	static bool MissingShaderErrorFlag = false;
	static bool MissingRoadRunnerShaderErrorFlag = false;

	////////////////////////////////////////////////////////////////////////////////
	// Check to see if regular materials should be used
	bool UseStandardPipeline()
	{
#if HAS_HDRP
		if(GraphicsSettings.currentRenderPipeline == null)
			return true;
		else
			return false;
#else
		return true;
#endif
	}

	bool UseHDRP()
	{
#if HAS_HDRP
		if(GraphicsSettings.currentRenderPipeline.GetType().ToString().Contains("HighDefinition"))
			return true;
		else
			return false;
#else
		return false;
#endif
	}



	////////////////////////////////////////////////////////////////////////////////
	// Replace colons and slashes to match Unity import
	string FixFilename(string filename)
	{
		if (filename != null)
		{
			filename = string.Join("_", filename.Split(Path.GetInvalidFileNameChars()));
		}
		return filename;
	}

	////////////////////////////////////////////////////////////////////////////////
	// Load all RoadRunner XML files into memory if not loaded already
	void LoadXml(string importedAssetPath)
	{
		string assetDirectory = Path.GetDirectoryName(importedAssetPath); // cut off file name

		if (ProcessedDirectories.Contains(assetDirectory))
		{
			return;
		}

		ProcessedDirectories.Add(assetDirectory);

		// Find all RoadRunner XML files
		string[] rrDataFiles = System.IO.Directory.GetFiles(assetDirectory, "*" + MathWorks.Metadata.FileExtension);

		if (rrDataFiles.Length != 0)
		{
			// Display Import Window
			MathWorks.ImportWindow.ShowWindow();
		}

		foreach (string xmlPath in rrDataFiles)
		{
			// Fix path name to only have forward slashes to be consistent with Unity
			string fixedXmlPath = xmlPath.Replace('\\', '/');
			string assetName = fixedXmlPath.Substring(0, fixedXmlPath.Length - MathWorks.Metadata.FileExtension.Length); // get rid of .rrData.xml
			assetName += ".fbx";
			if (FbxFileToMetadataMap.ContainsKey(assetName)) //check if the map contains the fbx file
			{
				continue;
			}

			MathWorks.Metadata rrData;
			XmlSerializer xmlSerializer = new XmlSerializer(typeof(MathWorks.Metadata));
			XmlTextReader xmlReader;
			try
			{
				MathWorks.ImportWindow.AddLogMessage("Found metadata: " + fixedXmlPath);
				xmlReader = new XmlTextReader(fixedXmlPath);
				rrData = xmlSerializer.Deserialize(xmlReader) as MathWorks.Metadata;
			}
			catch (System.Exception e)
			{
				MathWorks.ImportWindow.AddLogMessage("Failed to import " + fixedXmlPath);
				Debug.LogError(fixedXmlPath + " parsing failed. Try re-exporting from RoadRunner.\n" + e.ToString());
				continue;
			}


			MathWorks.ImportWindow.AddLogMessage("Importing material data");
			foreach (MathWorks.Material mat in rrData.Materials)
			{
				// Change strings to match Unity Settings
				mat.Name = FixFilename(mat.Name);

				// Add to maps for fast lookup later
				if (mat.DiffuseMap != null)
				{
					mat.DiffuseMap = FixFilename(mat.DiffuseMap);
					NameToTextureInfoMap[mat.DiffuseMap] = new MathWorks.TextureInfo(MathWorks.TextureInfo.EnumTextureType.eDiffuse);
				}

				if (mat.NormalMap != null)
				{
					mat.NormalMap = FixFilename(mat.NormalMap);
					NameToTextureInfoMap[mat.NormalMap] = new MathWorks.TextureInfo(MathWorks.TextureInfo.EnumTextureType.eNormal);
				}

				if (mat.SpecularMap != null)
				{
					mat.SpecularMap = FixFilename(mat.SpecularMap);
					NameToTextureInfoMap[mat.SpecularMap] = new MathWorks.TextureInfo(MathWorks.TextureInfo.EnumTextureType.eSpecular);
				}

				rrData.NameToMaterialMap[mat.Name] = mat;
			}

			// Add the associated fbx file to the map of loaded files
			FbxFileToMetadataMap[assetName] = rrData;

			// Version check
			if (rrData.Version > PluginVersion)
			{
				var mesg = fixedXmlPath + " has a version newer than the plugin. Installed plugin version "
					+ VersionName + " only supports rrdata versions up to " + PluginVersion
					+ ". Please update the plugin if there are issues with the import";

				MathWorks.ImportWindow.AddErrorMessage(mesg);
				Debug.LogWarning(mesg);
			}

			// Backwards compatibility
			if (rrData.Version < (int)MathWorks.MetadataVersions.SignalFormatChange)
			{
				if (rrData.SignalData_ != null)
				{
					// Shallow copy to new location
					rrData.Junctions = new List<MathWorks.Junction>(rrData.SignalData_.Junctions);
					rrData.SignalAssets = new List<MathWorks.SignalAsset>(rrData.SignalData_.SignalAssets);
				}
			}
		}
	}

	////////////////////////////////////////////////////////////////////////////////
	// Parse color from string
	Color ParseColor(string colorString)
	{
		string[] colorCompoments = colorString.Split(',');
		if (colorCompoments.Length != 3) // Only parse RGB colors
		{
			throw new System.ArgumentException("Color string must have exactly 3 comma separated values", "colorString");
		}

		float r = float.Parse(colorCompoments[0]);
		float g = float.Parse(colorCompoments[1]);
		float b = float.Parse(colorCompoments[2]);

		return new Color(r, g, b, 1);
	}

	////////////////////////////////////////////////////////////////////////////////
	// Clear out maps to let GC clean up memory
	static void OnPostprocessAllAssets(string[] importedAssets, string[] deletedAssets, string[] movedAssets, string[] movedFromAssetPaths)
	{
		FbxFileToMetadataMap.Clear();
		NameToTextureInfoMap.Clear();
		ProcessedDirectories.Clear();
	}

	////////////////////////////////////////////////////////////////////////////////
	// Load the xml and reset flags
	void OnPreprocessModel()
	{
#if UNITY_2020_1_OR_NEWER
		var modelImport = assetImporter as ModelImporter;
		modelImport.materialImportMode = ModelImporterMaterialImportMode.ImportStandard;
#endif
		MissingShaderErrorFlag = false;
		MissingRoadRunnerShaderErrorFlag = false;
		LoadXml(assetPath);
	}
	////////////////////////////////////////////////////////////////////////////////
	// Copied from StandardShaderGUI.cs from the builtin shaders archive
	public enum BlendMode
	{
		Opaque,
		Cutout,
		Fade,       // Old school alpha-blending mode, fresnel does not affect amount of transparency
		Transparent // Physically plausible transparency mode, implemented as alpha pre-multiply
	}




	////////////////////////////////////////////////////////////////////////////////
	// Load the texture asset from the string, and set the texture property in the material
	void SetMaterialTexture(Material material, string textureFile, string propertyName, Vector2 textureScale)
	{
		if (textureFile == null)
		{
			return;
		}
		string texPathName = Path.GetDirectoryName(assetPath) + "/" + textureFile;

		Texture2D texture = AssetDatabase.LoadAssetAtPath<Texture2D>(texPathName);
		if (texture != null)
		{
			material.SetTexture(propertyName, texture);
			material.SetTextureScale(propertyName, textureScale);
		}
		else
		{
			Debug.LogWarning("For material " + material.name + ", " + texPathName + " could not be found.");
		}
	}


	////////////////////////////////////////////////////////////////////////////////
	// Load the texture asset from the string, and set the texture property in the material
	// Texture scale set separately
	void SetHDRPMaterialTexture(Material material, string textureFile, string propertyName)
	{

		if (textureFile == null)
		{
			return;
		}
		string texPathName = Path.GetDirectoryName(assetPath) + "/" + textureFile;

		Texture2D texture = AssetDatabase.LoadAssetAtPath<Texture2D>(texPathName);

		if (texture != null)
		{
			material.SetTexture(propertyName, texture);
		}
		else
		{
			Debug.LogWarning("For material " + material.name + ", " + texPathName + " could not be found.");
		}
	}

	////////////////////////////////////////////////////////////////////////////////
	// Parse the color from the string, and set the color property in the material
	void SetMaterialColor(Material material, string colorString, float alpha, string propertyName)
	{
		if (colorString == null)
		{
			return;
		}

		Color color = Color.magenta; // Default to magenta if ParseColor fails
		try
		{
			color = ParseColor(colorString);
			if (alpha < 0.0 || alpha > 1.0)
			{
				throw new System.ArgumentException("Color alpha must be between 0 and 1", "colorString");
			}
		}
		catch (System.Exception e)
		{
			Debug.LogWarning("Error when parsing color of " + propertyName + "\n" + e.ToString());
		}
		color.a = alpha;
		material.SetColor(propertyName, color);
	}

	////////////////////////////////////////////////////////////////////////////////
	// Set a float property in the material
	void SetMaterialFloat(Material material, float value, string propertyName)
	{
		material.SetFloat(propertyName, value);
	}

	////////////////////////////////////////////////////////////////////////////////
	// Fixes the default Unity shader importing.
	// - Adds the specular map if it exists
	// - Fix transparency issues
	void OnPostprocessMaterial(Material mat)
	{
		MathWorks.Metadata rrData;
		FbxFileToMetadataMap.TryGetValue(assetPath, out rrData);
		// Check if there is RoadRunner data associated with the fbx
		if (rrData == null)
		{
			return;
		}

		// Load the material and set the specular map
		MathWorks.Material rrMat;
		rrData.NameToMaterialMap.TryGetValue(mat.name, out rrMat);
		if (rrMat == null)
		{
			return;
		}

		if (UseStandardPipeline())
		{
			// Legacy graphics
			Shader shader;
			bool isTransparent = (rrMat.TransparentMap != null) || (rrMat.TransparencyFactor > 0);

			// Prioritize using the marking shader for decals
			if (rrMat.IsDecal == "true")
			{
				shader = Shader.Find("Custom/RoadRunnerShaderMarking");
				mat.EnableKeyword("RR_TRANSPARENT");
			}
			else if (rrMat.TwoSided == "true")
			{
				shader = Shader.Find("Custom/RoadRunnerShaderTransparentTwoSided");
				mat.EnableKeyword("RR_LEAVES");
			}
			else
			{
				shader = Shader.Find("Custom/RoadRunnerShaderTransparent");
				mat.EnableKeyword("RR_TRANSPARENT");
			}


			Vector2 textureScale = new Vector2(rrMat.TextureScaleU, rrMat.TextureScaleV);

			if (isTransparent && shader != null)
			{
				// Use custom transparent shader
				mat.shader = shader;

				// If the object is transparent, adjust the cutoff so it won't be written to the zbuffer
				// and the object's alpha won't be modifed in the surface shader
				if (rrMat.TransparencyFactor > 0)
				{
					SetMaterialFloat(mat, 1.0f, "_Cutoff");
				}

				// Set textures
				SetMaterialTexture(mat, rrMat.DiffuseMap, "_MainTex", textureScale);
				SetMaterialTexture(mat, rrMat.NormalMap, "_BumpMap", textureScale);
				SetMaterialTexture(mat, rrMat.SpecularMap, "_RoughnessMap", textureScale);

				// Set colors
				SetMaterialColor(mat, rrMat.DiffuseColor, 1.0f - rrMat.TransparencyFactor, "_DiffuseColor");

				// Set floats
				//SetMaterialFloat(mat, rrMat.Roughness, "_RoughnessFactor");
				SetMaterialFloat(mat, rrMat.Emission, "_Emission");
				SetMaterialColor(mat, rrMat.EmissionColor, 1.0f, "_EmissionColor");

				// Set keywords for fallback shader
				mat.DisableKeyword("_ALPHATEST_ON");
				mat.EnableKeyword("_ALPHABLEND_ON");
				mat.DisableKeyword("_ALPHAPREMULTIPLY_ON");

				if (PluginVersion >= 1)
				{
					if (rrMat.IsDecal == "true")
					{
						// offset render queue from after "Opaque" default to receive shadows
						mat.renderQueue = MarkingRenderQueueBase + rrMat.DrawQueue;
					}
					else
					{
						// Render non-Decal materials last
						mat.renderQueue = TransparentRenderQueue;
					}
				}
			}
			else if ((shader = Shader.Find("Custom/RoadRunnerShader")) != null)
			{
				mat.shader = shader;

				// Note: RoadRunner is in the middle of converting to a roughness-metallic setup, so some material may not look right
				// Set textures
				SetMaterialTexture(mat, rrMat.DiffuseMap, "_DiffuseMap", textureScale);
				SetMaterialTexture(mat, rrMat.NormalMap, "_NormalMap", textureScale);
				SetMaterialTexture(mat, rrMat.SpecularMap, "_RoughnessMap", textureScale);

				// Set colors
				SetMaterialColor(mat, rrMat.DiffuseColor, 1.0f - rrMat.TransparencyFactor, "_DiffuseColor");

				// Set floats
				// Note: The default RoadRunner materials have not been tuned to the new setup yet
				//SetMaterialFloat(mat, rrMat.Roughness, "_RoughnessFactor");
				SetMaterialFloat(mat, rrMat.Emission, "_Emission");
				SetMaterialColor(mat, rrMat.EmissionColor, 1.0f, "_EmissionColor");
			}
			else
			{
				if (!MissingRoadRunnerShaderErrorFlag)
				{
					MissingRoadRunnerShaderErrorFlag = true;
					Debug.LogWarning("RoadRunner shaders could not be found.");
				}

				// Fallback to built-in shader if custom shaders not found
				shader = Shader.Find("Standard (Roughness setup)");
				if (shader == null)
				{
					if (!MissingShaderErrorFlag)
					{
						MissingShaderErrorFlag = true;
						Debug.LogError("Standard (Roughness setup) shader could not be found.");
					}
					return;
				}

				mat.shader = shader;

				if (rrMat.SpecularMap != null)
				{
					string specPathName = Path.GetDirectoryName(assetPath) + "/" + rrMat.SpecularMap;

					Texture2D texture = AssetDatabase.LoadAssetAtPath(specPathName, typeof(Texture2D)) as Texture2D;
					if (texture != null)
					{
						mat.EnableKeyword("_SPECGLOSSMAP");
						mat.SetTexture("_SpecGlossMap", texture);
					}
					else
					{
						Debug.LogWarning("Specular Map could not be found at " + specPathName + " for: " + mat.name);
					}
				}

				SetMaterialColor(mat, rrMat.DiffuseColor, 1.0f - rrMat.TransparencyFactor, "_Color");

				if (isTransparent)
				{
					// Change to Fade mode. Based off StandardShaderGUI.cs from the builtin shaders archive
					mat.SetFloat("_Mode", (float)BlendMode.Fade);
					mat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
					mat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
					mat.SetInt("_ZWrite", 0);
					mat.DisableKeyword("_ALPHATEST_ON");
					mat.EnableKeyword("_ALPHABLEND_ON");
					mat.DisableKeyword("_ALPHAPREMULTIPLY_ON");
					mat.renderQueue = 3000; // Value of UnityEngine.Rendering.RenderQueue.Transparent as defined in version 5.4+
				}
			}
		}
		else if (UseHDRP())
		{
			bool isRoughness = string.IsNullOrEmpty(rrMat.SpecularMap);

			Shader shader;
			bool isTransparent = (rrMat.TransparentMap != null) || (rrMat.TransparencyFactor > 0);
			if (isTransparent)
			{
				if (rrMat.IsDecal == "true")
				{
					if (isRoughness)
						shader = Shader.Find("Shader Graphs/RR_HDRP_Roughness_Decal");
					else
						shader = Shader.Find("Shader Graphs/RR_HDRP_Specular_Decal");
				}
				else
				{
					if (isRoughness)
						shader = Shader.Find("Shader Graphs/RR_HDRP_Roughness_Transparent");
					else
						shader = Shader.Find("Shader Graphs/RR_HDRP_Specular_Transparent");
				}
				mat.shader = shader;

				// Set to transparent mode
				mat.SetFloat("_SurfaceType", 1);
				if (rrMat.IsDecal == "true")
				{
					mat.SetFloat("_ZWrite", 0);
					mat.SetFloat("_TransparentZWrite", 0);
					mat.SetFloat("_TransparentSortPriority", rrMat.DrawQueue);
					mat.SetFloat("_EnableBlendModePreserveSpecularLighting", 0);
				}
				else
				{
					mat.SetFloat("_AlphaCutoffEnable", 1);
					mat.SetFloat("_ZWrite", 1);
					mat.SetFloat("_TransparentZWrite", 1);
					mat.SetFloat("_EnableBlendModePreserveSpecularLighting", 0);
				}

				if (rrMat.TwoSided == "true")
				{
					mat.SetInt("_DoubleSidedEnable", 1);
				}
				else
				{
					mat.SetInt("_DoubleSidedEnable", 0);
				}



			}
			else
			{
				if (isRoughness)
					shader = Shader.Find("Shader Graphs/RR_HDRP_Roughness_Opaque");
				else
					shader = Shader.Find("Shader Graphs/RR_HDRP_Specular_Opaque");
				mat.shader = shader;
			}

#if HAS_HDRP
			UnityEditor.Rendering.HighDefinition.HDShaderUtils.ResetMaterialKeywords(mat);
#endif

			SetHDRPMaterialTexture(mat, rrMat.DiffuseMap, "_DiffuseMap");
			SetHDRPMaterialTexture(mat, rrMat.NormalMap, "_NormalMap");
			SetHDRPMaterialTexture(mat, rrMat.RoughnessMap, "_RoughnessMap");

			// Set colors
			SetMaterialColor(mat, rrMat.DiffuseColor, 1.0f - rrMat.TransparencyFactor, "_DiffuseColor");
			SetMaterialColor(mat, rrMat.SpecularColor, 1.0f, "_SpecularColor");
			SetMaterialColor(mat, rrMat.EmissionColor, 1.0f, "_EmissionColorHDRP");

			// Set floats
			SetMaterialFloat(mat, rrMat.Roughness, "_RoughnessFactor");
			SetMaterialFloat(mat, rrMat.Emission, "_Emission");

			SetMaterialFloat(mat, rrMat.TextureScaleU, "_TextureScaleU");
			SetMaterialFloat(mat, rrMat.TextureScaleV, "_TextureScaleV");
		}
		else
		{
			Debug.LogWarning("Only the Standard and High Definition render pipelines are currently supported.");
		}



	}

	////////////////////////////////////////////////////////////////////////////////
	// Fixes texture import settings for normal maps
	void OnPreprocessTexture()
	{
		// Load all RoadRunner XML files
		LoadXml(assetPath);

		string filename = Path.GetFileName(assetPath);
		MathWorks.TextureInfo texInfo;
		NameToTextureInfoMap.TryGetValue(filename, out texInfo);
		// Setting normal map import settings
		if (texInfo != null && texInfo.Type == MathWorks.TextureInfo.EnumTextureType.eNormal)
		{
			TextureImporter textureImporter = (TextureImporter)assetImporter;
			textureImporter.textureType = TextureImporterType.NormalMap;
		}
	}

	// Store LOD data for later processing
	void OnPostprocessGameObjectWithUserProperties(
		GameObject gameObject,
		string[] propNames,
		object[] values)
	{
		for (int i = 0; i < propNames.Length; i++)
		{
			string propName = propNames[i];
			switch (propName)
			{
				case "LodThresholds":
					// Convert string into float list
					string[] thresholds = values[i].ToString().Split(';');
					List<float> lodThresholds = new List<float>();
					foreach (string thresholdStr in thresholds)
					{
						float threshold;
						if (float.TryParse(thresholdStr, out threshold))
						{
							lodThresholds.Add(threshold);
						}
						else
						{

							Debug.LogError("Failed to parse threshold: " + thresholdStr);
						}
					}

					// Add component to temporarily store data
					MathWorks.LodImportData importData = gameObject.AddComponent<MathWorks.LodImportData>();
					importData.LodThresholds = lodThresholds;
					break;
				default:
					break;
			}
		}
	}

	////////////////////////////////////////////////////////////////////////////////
	// Setup LODgroup from imported data
	void SetupLods(GameObject gameObject)
	{
		// Set up LODs
		foreach (MathWorks.LodImportData lodImportData in gameObject.GetComponentsInChildren<MathWorks.LodImportData>())
		{
			GameObject lodGameObject = lodImportData.gameObject;
			int numThresholds = lodImportData.LodThresholds.Count;

			LODGroup lodGroup = lodGameObject.AddComponent<LODGroup>();
			LOD[] lods = new LOD[numThresholds];

			for (int j = 0; j < numThresholds; j++)
			{
				float threshold = lodImportData.LodThresholds[j] / 100.0f; // Convert from percentage
				Renderer[] renderers = lodGameObject.transform.GetChild(j).GetComponentsInChildren<Renderer>();
				lods[j] = new LOD(threshold, renderers);
			}

			lodGroup.SetLODs(lods);
			lodGroup.RecalculateBounds();

			// Delete temp data
			UnityEngine.Object.DestroyImmediate(lodImportData);
		}
	}

	////////////////////////////////////////////////////////////////////////////////
	// Create the traffic controller for each junction, and set up the references
	void OnPostprocessModel(GameObject go)
	{
		MathWorks.Metadata rrData;
		FbxFileToMetadataMap.TryGetValue(assetPath, out rrData);
		// Check if there is RoadRunner data associated with the fbx
		if (rrData == null)
		{
			return;
		}

		MathWorks.ImportWindow.AddLogMessage("Importing traffic signals");

		// Rotate 180 if specified
		var settings = MathWorks.ImportSettings.GetOrCreateSettings();
		if (settings.RotateScene180)
		{
			var modelImport = assetImporter as ModelImporter;
			if (modelImport.preserveHierarchy)
			{
				if (go.transform.childCount != 0)
				{
					go.transform.GetChild(0).transform.Rotate(0, 0, 180);
				}
			}
			else
			{
				go.transform.Rotate(0, 0, 180);
			}
		}

		// Add colliders to all game objects with mesh
		AddColliders(go);

		// Create map for easy repeated lookup of uuid to game object
		Dictionary<string, GameObject> uuidToGameObjectMap = CreateUuidGameObjectMap(go);

		// Set up uuid map for signal assets and convert imported strings to bool
		foreach (MathWorks.SignalAsset signalAsset in rrData.SignalAssets)
		{
			rrData.UuidToSignalAssetMap[signalAsset.Id] = signalAsset;

			foreach (MathWorks.SignalConfiguration signalConfig in signalAsset.SignalConfigurations)
			{
				foreach (MathWorks.LightBulbState lightBulbState in signalConfig.LightBulbStates)
				{

					if (string.Equals(lightBulbState.StateString, "On", System.StringComparison.OrdinalIgnoreCase)
						|| string.Equals(lightBulbState.StateString, "True", System.StringComparison.OrdinalIgnoreCase))
					{
						lightBulbState.State = MathWorks.EnumBulbState.eOn;
					}
					else if (string.Equals(lightBulbState.StateString, "Blinking", System.StringComparison.OrdinalIgnoreCase))
					{
						lightBulbState.State = MathWorks.EnumBulbState.eBlinking;
					}
					else
					{
						lightBulbState.State = MathWorks.EnumBulbState.eOff;
					}
				}
			}
		}

		foreach (MathWorks.Junction junction in rrData.Junctions)
		{
			// Create a new GameObject to attach the signal controller to
			GameObject junctionGameObj = new GameObject("JunctionController" + junction.Id);
			junctionGameObj.transform.SetParent(go.transform);
			// TODO: set the position to the average of all its signals and add a gizmo

			MathWorks.TrafficJunction junctionComponent = junctionGameObj.AddComponent<MathWorks.TrafficJunction>();
			junctionComponent.JunctionId = junction.Id;
			// Set up the game object refernce within the lightbulb state it is setup for the TrafficJunction script
			foreach (MathWorks.SignalPhase signalPhase in junction.SignalPhases)
			{
				foreach (MathWorks.Interval interval in signalPhase.Intervals)
				{
					foreach (MathWorks.SignalState signalState in interval.SignalStates)
					{
						MathWorks.SignalAsset signalAsset;
						if (!rrData.UuidToSignalAssetMap.TryGetValue(signalState.SignalAssetId, out signalAsset))
						{
							Debug.LogWarning("Signal Asset ID: " + signalState.SignalAssetId + " could not be found in the metadata file.");
							continue;
						}

						MathWorks.SignalConfiguration ConfigurationRef;

						if (signalState.Configuration >= signalAsset.SignalConfigurations.Count || signalState.Configuration < 0)
						{
							Debug.LogWarning("Signal Configuration " + signalState.Configuration + " for: " + signalState.Id + " is outside the range of the configuration list");
							continue;
						}
						ConfigurationRef = signalAsset.SignalConfigurations[signalState.Configuration];

						GameObject signalInstance;

						if (!uuidToGameObjectMap.TryGetValue(signalState.Id, out signalInstance))
						{
							// Only print error if this is not an individually tiled export
							Regex regex = new Regex(@".*_Tile_[0-9]+_[0-9]+");
							if (!regex.IsMatch(go.name))
							{
								Debug.LogWarning("Signal ID: " + signalState.Id + " could not be found.");
							}
							continue;
						}

						if (signalInstance.transform.childCount != 1)
						{
							Debug.LogWarning("Signal ID: " + signalState.Id + " is invalid.");
							continue;
						}
						// Find the transform node for the signal instance, the signal root node will be the only node under it.
						GameObject signalGameObj = signalInstance.transform.GetChild(0).gameObject;

						foreach (MathWorks.LightBulbState lightBulbState in ConfigurationRef.LightBulbStates)
						{
							if (rrData.Version < (int)MathWorks.MetadataVersions.FlashingBulb)
							{
								GameObject lightBulb = FindByNamePrefix(signalGameObj, lightBulbState.Name);
								if (lightBulb == null)
								{
									Debug.LogWarning("LightBulb " + lightBulbState.Name + " could not be found under " + signalState.Id);
								}
								else
								{
									MathWorks.LightInstanceState lightInstanceState = new MathWorks.LightInstanceState();
									lightInstanceState.LegacyRef = lightBulb;
									lightInstanceState.State = lightBulbState.State;
									signalState.LightInstanceStates.Add(lightInstanceState);
								}
							}
							else
							{
								GameObject onBulb = FindByNamePrefix(signalGameObj, lightBulbState.Name + "_on");
								GameObject offBulb = FindByNamePrefix(signalGameObj, lightBulbState.Name + "_off");

								if (onBulb == null || offBulb == null)
								{
									Debug.LogWarning("LightBulb " + lightBulbState.Name + " could not be found under " + signalState.Id);
								}
								else
								{
									MathWorks.LightInstanceState lightInstanceState = new MathWorks.LightInstanceState();
									lightInstanceState.OnRef = onBulb;
									lightInstanceState.OffRef = offBulb;
									lightInstanceState.State = lightBulbState.State;
									signalState.LightInstanceStates.Add(lightInstanceState);
								}
							}
						}
					}
				}
			}

			// Set the phase list in the script
			junctionComponent.SetPhases(junction.SignalPhases);
		}

		// Strip out automatically added lods at base level
		if (go.GetComponent<LODGroup>() != null)
		{
			LODGroup lodGroup = go.GetComponent<LODGroup>();
			UnityEngine.Object.DestroyImmediate(lodGroup);
		}

		SetupLods(go);

		MathWorks.ImportWindow.AddLogMessage("Finished importing " + go.name);
		MathWorks.ImportWindow.ScrollToBottom();
	}

	////////////////////////////////////////////////////////////////////////////////
	// Create a dictionary that maps UUIDs to GameObject if it has one prefixed in the name
	Dictionary<string, GameObject> CreateUuidGameObjectMap(GameObject root)
	{
		Dictionary<string, GameObject> ret = new Dictionary<string, GameObject>();

		// Depth first traversal through the tree
		Stack<GameObject> nextGameObjects = new Stack<GameObject>();
		nextGameObjects.Push(root);
		while (nextGameObjects.Count != 0)
		{
			GameObject go = nextGameObjects.Pop();

			// Check if name starts with guid
			Regex regex = new Regex(@"^[{(]?[0-9A-F]{8}[-]?([0-9A-F]{4}[-]?){3}[0-9A-F]{12}[)}]?", RegexOptions.IgnoreCase);

			MatchCollection matches = regex.Matches(go.name);

			// Should only be one match
			foreach (Match match in matches)
			{
				// Only prop instances should have uuid prefixed in the name
				if (!ret.ContainsKey(match.Value))
				{
					ret.Add(match.Value, go);
				}
			}

			foreach (Transform child in go.transform)
			{
				nextGameObjects.Push(child.gameObject);
			}
		}

		return ret;
	}

	////////////////////////////////////////////////////////////////////////////////
	// Find a GameObject starting with a particular string within the parent's descendants
	// Returns the first GameObject found that matches
	GameObject FindByNamePrefix(GameObject parent, string prefix)
	{
		foreach (Transform child in parent.transform)
		{
			Regex regex = new Regex(@"^" + prefix + @".*(_[0-9]+)?Node.*");

			if (regex.IsMatch(child.gameObject.name))
			{
				return child.gameObject;
			}

			GameObject childCheck = FindByNamePrefix(child.gameObject, prefix);

			if (childCheck != null)
			{
				return childCheck;
			}
		}
		return null;
	}

	////////////////////////////////////////////////////////////////////////////////
	// Add colliders to entire scene
	void AddColliders(GameObject gameObject)
	{
		// Add collider if game object has mesh data
		if (gameObject.GetComponent<MeshRenderer>())
			gameObject.AddComponent<MeshCollider>();

		foreach (Transform child in gameObject.transform)
		{
			AddColliders(child.gameObject);
		}

	}
}

#endif