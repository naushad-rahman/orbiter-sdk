// ==============================================================
//                 ORBITER MODULE: Atlantis
//                  Part of the ORBITER SDK
//          Copyright (C) 2001-2003 Martin Schweiger
//                   All rights reserved
//
// Atlantis_SRB.cpp
// Reference implementation of Atlantis SRB(Space Shuttle - Solid
// Rocket Booster) vessel class module
// Note: This module takes control of the SRB after separation
// from the Shuttle's main tank.
// ==============================================================

#define ORBITER_MODULE
#include "Atlantis.h"
#include "math.h"
#include "stdio.h"

// ==============================================================
// Specialised vessel class Atlantis_SRB
// ==============================================================

// Constructor
Atlantis_SRB::Atlantis_SRB (OBJHANDLE hObj)
: VESSEL2(hObj)
{
	// preload mesh
	hSRBMesh = oapiLoadMeshGlobal ("Atlantis_srb");
}

// reconstruct liftoff time from fuel level
void Atlantis_SRB::SetRefTime (void)
{
	extern int SRB_nt;
	extern double SRB_Seq[], SRB_Prop[], SRB_PrpSCL[];

	int i;
	double fuel = GetFuelMass()/GetMaxFuelMass();
	for (i = 1; i < SRB_nt; i++)
		if (fuel >= SRB_Prop[i]) break;
	double met = SRB_Seq[i] + (fuel-SRB_Prop[i])/SRB_PrpSCL[i-1];
	t0 = oapiGetSimTime()-met;
}

// ==============================================================
// Callback functions
// ==============================================================

// Set SRB class specs
void Atlantis_SRB::clbkSetClassCaps (FILEHANDLE cfg)
{
	extern PARTICLESTREAMSPEC srb_contrail, srb_exhaust;
	PARTICLESTREAMSPEC srb_bolt = {
		0, 8.0, 20, 0.0, 0.1, 0.3, 16, 3.0, PARTICLESTREAMSPEC::EMISSIVE,
		PARTICLESTREAMSPEC::LVL_LIN, 0, 1,
		PARTICLESTREAMSPEC::ATM_FLAT, 1, 1
	};

	SetEnableFocus (false);
	// SRB cannot receive input focus

	// *********************** physical parameters *********************************

	SetSize (23.0);
	SetEmptyMass (SRB_EMPTY_MASS);
	SetCW (0.1, 0.3, 1.4, 1.4);
	SetCrossSections (_V(162.1,162.1,26.6));
	SetRotDrag (_V(0.7,0.7,0.1));
	SetPMI (_V(154.3,154.3,1.83));
	//SetGravityGradientDamping (10.0);
	SetTouchdownPoints (_V(0,-2,-21.1), _V(-2,1,-21.1), _V(2,1,-21.1));
	SetLiftCoeffFunc (0);

	// ************************* propellant specs **********************************

	ph_main = CreatePropellantResource (SRB_MAX_PROPELLANT_MASS);
	// Note that the SRB instance is only created after separation from
	// the main assembly, so the actual fuel mass will always be
	// much smaller

	// *********************** thruster definitions ********************************

	// main engine
	th_main = CreateThruster (_V(0,0,-21), _V(0,0,1), SRB_THRUST, ph_main, SRB_ISP0, SRB_ISP1);
	SURFHANDLE tex = oapiRegisterExhaustTexture ("Exhaust2");
	srb_exhaust.tex = oapiRegisterParticleTexture ("Contrail2");
	AddExhaust (th_main, 16.0, 2.0, tex);
	AddExhaustStream (th_main, _V(0,0,-30), &srb_contrail);
	AddExhaustStream (th_main, _V(0,0,-25), &srb_exhaust);

	// separation bolts
	th_bolt = CreateThruster (_V(0,0,1.2), _V(1,0,0), 3e6, ph_main, 1e7);
	// for simplicity, the separation bolts directly use SRB propellant. We give
	// them an insanely high ISP to avoid significant propellant drainage

	AddExhaust (th_bolt, 0.7, 0.1, _V(-2.1,0,-8), _V(-1,0,0));
	AddExhaust (th_bolt, 0.7, 0.1, _V(-2.1,0,11), _V(-1,0,0));
	AddExhaustStream (th_bolt, _V(-2.1,0,0), &srb_bolt);

	// ************************ visual parameters **********************************

	AddMesh (hSRBMesh);

	bMainEngine = true;
	bSeparationEngine = true;
	srb_separation_time = oapiGetSimTime();
}

// Finish setup
void Atlantis_SRB::clbkPostCreation ()
{
	SetRefTime ();	// reconstruct ignition time from fuel level
}

// Simulation time step
void Atlantis_SRB::clbkPostStep (double simt, double simdt, double mjd)
{
	extern void GetSRB_State (double, double&, double&);
	//sprintf (oapiDebugString(), "SRB mass = %f", GetMass());
	if (bMainEngine) {
		double met = simt-t0;
		if (met >= SRB_CUTOUT_TIME) {
			SetThrusterLevel (th_main, 0);
			bMainEngine = false;
			// After the propellant is burnt out we should be airborne.
			// Now we can prepare touchdown points for "landing"
			SetTouchdownPoints (_V(0,9,3), _V(-1,1,-3), _V(1,1,-3));
		} else {
			double thrust_level, prop_level;
			GetSRB_State (met, thrust_level, prop_level);
			SetThrusterLevel (th_main, thrust_level);
		}
		if (bSeparationEngine) {
			static double bolt_t = 0.5;
			double srb_dt = simt - srb_separation_time;
			if (srb_dt > bolt_t) {
				DelThruster (th_bolt);
				bSeparationEngine = false;
			} else {
				SetThrusterLevel (th_bolt, sqrt (1.0-srb_dt/bolt_t));
			}
		}
	}
	if (GetAltitude() < 0.0) oapiDeleteVessel (GetHandle());
}

// ==============================================================
// API interface
// ==============================================================

// Initialisation
DLLCLBK VESSEL *ovcInit (OBJHANDLE hvessel, int flightmodel)
{
	return new Atlantis_SRB (hvessel);
}

// Cleanup
DLLCLBK void ovcExit (VESSEL *vessel)
{
	if (vessel) delete (Atlantis_SRB*)vessel;
}
