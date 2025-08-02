// Copyright 2020 The MathWorks, Inc.


using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Temporary data holder for import process
namespace MathWorks
{
	public class LodImportData : MonoBehaviour
	{
		public List<float> LodThresholds;

		// Start is called before the first frame update
		void Start()
		{
			Debug.LogError("LOD data has not been imported correctly.");
		}
	}
}
