// Copyright 2020 The MathWorks, Inc.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Xml;
using System.Xml.Serialization;

////////////////////////////////////////////////////////////////////////////////
// This file holds all the classes needed to deserialize the XML into classes.
namespace MathWorks
{
	// Enum to describe major XML format changes between versions for
	// backward compatibility
	public enum MetadataVersions
	{
		SignalFormatChange = 3,
		FlashingBulb = 5
	}

	public enum EnumBulbState
	{
		eOn, eOff, eBlinking
	};

	// Classes for XML deserialization
	////////////////////////////////////////////////////////////////////////////////
	// Root node holding the data
	[XmlRoot("RoadRunnerMetadata")]
	public class Metadata
	{
		[XmlArray("MaterialList"), XmlArrayItem("Material")]
		public List<Material> Materials = new List<Material>();

		// For backwards compatibility
		[XmlElement("SignalData")]
		public SignalData SignalData_;

		[XmlAttribute("Version")]
		public int Version = 0;

		// Version 3 changes (replaces SignalData)
		[XmlArray("SignalConfigurations"), XmlArrayItem("Signal")]
		public List<SignalAsset> SignalAssets;

		[XmlArray("Signalization"), XmlArrayItem("Junction")]
		public List<Junction> Junctions;

		// Additional dictionaries for fast lookup during import
		[XmlIgnore]
		public Dictionary<string, Material> NameToMaterialMap = new Dictionary<string, Material>();

		[XmlIgnore]
		public Dictionary<string, SignalAsset> UuidToSignalAssetMap = new Dictionary<string, SignalAsset>();

		[XmlIgnore]
		public const string FileExtension = ".rrdata.xml";
	}

	////////////////////////////////////////////////////////////////////////////////
	// Material class - Holds data from material information in RoadRunner
	public class Material
	{
		[XmlElement("Name")]
		public string Name;

		// Texture maps stored as image file names
		[XmlElement("DiffuseMap")]
		public string DiffuseMap;
		[XmlElement("NormalMap")]
		public string NormalMap;
		[XmlElement("SpecularMap")]
		public string SpecularMap;
		[XmlElement("TransparentColor")]
		public string TransparentMap;

		// Colors stored as comma-separated decimal values
		[XmlElement("AmbientColor")]
		public string AmbientColor;
		[XmlElement("DiffuseColor")]
		public string DiffuseColor;
		[XmlElement("SpecularColor")]
		public string SpecularColor;

		// Various scalar values
		[XmlElement("Roughness")]
		public float Roughness;
		[XmlElement("SpecularFactor")]
		public float SpecularFactor;
		[XmlElement("TransparencyFactor")]
		public float TransparencyFactor;
		[XmlElement("Emission")]
		public float Emission;
		[XmlElement("EmissionColor")]
		public string EmissionColor;
		[XmlElement("TextureScaleU")]
		public float TextureScaleU;
		[XmlElement("TextureScaleV")]
		public float TextureScaleV;

		[XmlElement("TwoSided")]
		public string TwoSided;

		// Version 1 additions
		[XmlElement("DrawQueue")]
		public int DrawQueue = 0;
		[XmlElement("ShadowCaster")]
		public string ShadowCaster = "true";
		[XmlElement("IsDecal")]
		public string IsDecal = "false";

		// Version 2 additions
		[XmlElement("SegmentationType")]
		public string SegmentationType;

		// Version 4 additions
		[XmlElement("RoughnessMap")]
		public string RoughnessMap;
	}

	////////////////////////////////////////////////////////////////////////////////
	// Holds info about textures
	public class TextureInfo
	{
		public enum EnumTextureType { eDiffuse, eNormal, eSpecular };
		public EnumTextureType Type;

		public TextureInfo(EnumTextureType type)
		{
			this.Type = type;
		}
	}

	// Classes for signal data
	////////////////////////////////////////////////////////////////////////////////
	// SignalData class - Holds SignalAssets and Junctions
	[System.Serializable]
	public class SignalData
	{
		[SerializeField]
		[XmlArray("SignalAssets"), XmlArrayItem("Signal")]
		public List<SignalAsset> SignalAssets = new List<SignalAsset>();

		[SerializeField]
		[XmlElement("Junction")]
		public List<Junction> Junctions = new List<Junction>();
	}

	////////////////////////////////////////////////////////////////////////////////
	// SignalAsset class - Holds ID and configurations
	[System.Serializable]
	public class SignalAsset
	{
		[SerializeField]
		[XmlElement("ID")]
		public string Id;

		[SerializeField]
		[XmlElement("Configuration")]
		public List<SignalConfiguration> SignalConfigurations = new List<SignalConfiguration>();
	}

	////////////////////////////////////////////////////////////////////////////////
	// SignalConfiguration class - Holds LightStates.
	[System.Serializable]
	public class SignalConfiguration
	{
		[SerializeField]
		[XmlElement("Name")]
		public string Name;

		[SerializeField]
		[XmlElement("LightState")]
		public List<LightBulbState> LightBulbStates = new List<LightBulbState>();
	}

	////////////////////////////////////////////////////////////////////////////////
	// LightBulb class - Holds information on which meshes/lights should be enabled
	[System.Serializable]
	public class LightBulbState
	{
		[SerializeField]
		[XmlElement("Name")]
		public string Name;

		[XmlElement("OnMesh")]
		public string OnMesh;

		[XmlElement("OffMesh")]
		public string OffMesh;

		[XmlElement("State")]
		public string StateString;

		[SerializeField]
		[XmlIgnore]
		public EnumBulbState State;
	}

	////////////////////////////////////////////////////////////////////////////////
	// Junction class - Holds its ID and a list of signal phases
	[System.Serializable]
	public class Junction
	{
		[SerializeField]
		[XmlElement("ID")]
		public string Id;

		[SerializeField]
		[XmlElement("SignalPhase")]
		public List<SignalPhase> SignalPhases = new List<SignalPhase>();
	}

	////////////////////////////////////////////////////////////////////////////////
	// SignalPhase class - Holds a list of Signal states and Intervals
	[System.Serializable]
	public class SignalPhase
	{
		[SerializeField]
		[XmlElement("Interval")]
		public List<Interval> Intervals = new List<Interval>();
	}

	////////////////////////////////////////////////////////////////////////////////
	// Interval class - Holds interval duration and the states for the signals
	[System.Serializable]
	public class Interval
	{
		[SerializeField]
		[XmlElement("Time")]
		public float Time;

		[SerializeField]
		[XmlElement("Signal")]
		public List<SignalState> SignalStates = new List<SignalState>();
	}

	////////////////////////////////////////////////////////////////////////////////
	// Signal class - The state of a signal for all its light bulbs for a specific phase
	[System.Serializable]
	public class SignalState
	{
		[SerializeField]
		[XmlElement("ID")]
		public string Id;

		[SerializeField]
		[XmlElement("SignalAsset")]
		public string SignalAssetId;

		[SerializeField]
		[XmlElement("ConfigurationIndex")]
		public int Configuration;

		// List of game object references and state for use in runtime
		[XmlIgnore]
		[SerializeField]
		public List<LightInstanceState> LightInstanceStates = new List<LightInstanceState>();
	}

	////////////////////////////////////////////////////////////////////////////////
	// LightInstanceState class - For the TrafficJunction script to easily set the light states
	[System.Serializable]
	public class LightInstanceState
	{
		public GameObject OffRef;
		public GameObject OnRef;
		public GameObject LegacyRef;
		public EnumBulbState State;
	}
}
