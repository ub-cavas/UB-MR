// Copyright 2020 The MathWorks, Inc.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

////////////////////////////////////////////////////////////////////////////////
// This script controls traffic signals based off the imported RoadRunner
// metadata.
namespace MathWorks
{
	public class TrafficJunction : MonoBehaviour
	{
		// The list of phases will be set within RoadRunnerImporter script during import time
		public List<SignalPhase> Phases = new List<SignalPhase>();

		private float Timer = 0f;
		private int CurrentPhase = 0;
		private int CurrentInterval = 0;

		// UUID of the junction
		public string JunctionId;
		public static Dictionary<string, TrafficJunction> JunctionIdToController = new Dictionary<string, TrafficJunction>();

		// Mode to determine if the traffic should run by itself or depend on an external
		// script to call SetPhaseAndInterval.
		public enum Mode
		{
			eStandAlone,
			eNetwork
		}

		public Mode CurrentMode = Mode.eStandAlone;

		////////////////////////////////////////////////////////////////////////////////

		void Awake()
		{
			// Add self to dictionary
			JunctionIdToController[JunctionId] = this;
		}

		////////////////////////////////////////////////////////////////////////////////
		// Check the timer and change the lights in the junction
		void Update()
		{

			if (Phases.Count == 0)
				return;

			SignalPhase signalPhase = GetCurrentPhase();
			Interval interval = GetCurrentInterval();

			// Set each light bulb's state based off the current phase and interval
			foreach (SignalState signalState in interval.SignalStates)
			{
				foreach (LightInstanceState lightInstanceState in signalState.LightInstanceStates)
				{
					bool state;
					if (lightInstanceState.State == MathWorks.EnumBulbState.eOff) state = false;
					else if (lightInstanceState.State == MathWorks.EnumBulbState.eOn) state = true;
					else
					{
						if (Timer % 1 >= 0.5f) state = false;
						else state = true;
					}
					if (lightInstanceState.LegacyRef) lightInstanceState.LegacyRef.SetActive(state);
					else
					{
						lightInstanceState.OffRef.SetActive(!state);
						lightInstanceState.OnRef.SetActive(state);
					}
				}
			}

			if (CurrentMode == Mode.eNetwork)
				return;

			Timer += Time.deltaTime;

			// Update current state
			if (Timer > interval.Time)
			{
				// Reset timer
				Timer = 0;
				CurrentInterval++;
				if (CurrentInterval >= signalPhase.Intervals.Count)
				{
					CurrentInterval = 0;

					CurrentPhase++;
					if (CurrentPhase >= Phases.Count)
					{
						CurrentPhase = 0;
					}
				}
			}
		}

		////////////////////////////////////////////////////////////////////////////////

		SignalPhase GetCurrentPhase()
		{
			return Phases[CurrentPhase];
		}

		////////////////////////////////////////////////////////////////////////////////

		Interval GetCurrentInterval()
		{
			return GetCurrentPhase().Intervals[CurrentInterval];
		}

		////////////////////////////////////////////////////////////////////////////////

		public void SetPhases(List<SignalPhase> signalPhases)
		{
			Phases = signalPhases;
		}

		////////////////////////////////////////////////////////////////////////////////

		public void SetPhaseAndInterval(int phase, int interval)
		{
			CurrentMode = Mode.eNetwork;

			if (phase >= Phases.Count || interval >= Phases[phase].Intervals.Count)
			{
				Debug.LogWarning("SetPhaseAndInterval given parameters out of range.");
				return;
			}
			CurrentPhase = phase;
			CurrentInterval = interval;
		}

	}

}
