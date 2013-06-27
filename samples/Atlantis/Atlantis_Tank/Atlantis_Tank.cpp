// ==============================================================
//                 ORBITER MODULE: Atlantis
//                  Part of the ORBITER SDK
//          Copyright (C) 2001-2003 Martin Schweiger
//                   All rights reserved
//
// Atlantis_Tank.cpp
// Reference implementation of Atlantis Tank vessel class module
// Note: This module takes control of the tank after separation
// from the orbiter.
// ==============================================================

#define ORBITER_MODULE
#include "Atlantis.h"
#include "math.h"

// ==============================================================
// Specialised vessel class Atlantis_SRB
// ==============================================================

// Constructor
Atlantis_Tank::Atlantis_Tank (OBJHANDLE hObj)
: VESSEL2(hObj)
{
	// preload mesh
	hTankMesh = oapiLoadMeshGlobal ("Atlantis_tank");
}

// ==============================================================
// Callback functions
// ==============================================================

// Set Tank class specs
void Atlantis_Tank::clbkSetClassCaps (FILEHANDLE cfg)
{
	SetEnableFocus (false);
	// Tank cannot receive input focus

	SetSize (24.0);
	SetEmptyMass (TANK_EMPTY_MASS);

	SetMaxFuelMass (TANK_MAX_PROPELLANT_MASS);
	// Note that the Tank instance is only created after separation from
	// the orbiter, so the actual fuel mass will always be much smaller

	SetISP (5000.0);

	SetMaxThrust (ENGINE_MAIN, 0);
	SetMaxThrust (ENGINE_RETRO, 0);
	SetMaxThrust (ENGINE_HOVER, 0);
	SetMaxThrust (ENGINE_ATTITUDE, 0);
	// Tank has no engines of its own

	SetCW (0.2, 0.3, 1.2, 1.2);
	VECTOR3 cs = {412.1,411.8,72.7};
	SetCrossSections (cs);
	VECTOR3 rd = {0.5,0.5,0.1};
	SetRotDrag (rd);
	VECTOR3 pmi = {145.6,145.6,10.5};
	SetPMI (pmi);
	SetPitchMomentScale (1e-4);
	SetYawMomentScale (1e-4);

	VECTOR3 co = {0,0,0};
	SetCameraOffset (co);
	// Note that the camera offset should not be required
	// since the Tank doesn't define a 'cockpit'

	SetCOG_elev (-5.0);
	SetTouchdownPoints (_V(0,9,3), _V(-1,1,-3), _V(1,1,-3));
	SetLiftCoeffFunc (0);

	AddMesh (hTankMesh);
}

// Simulation time step
void Atlantis_Tank::clbkPostStep (double simt, double simdt, double mjd)
{
	if (GetAltitude() < 0.0) oapiDeleteVessel (GetHandle());
}

// ==============================================================
// API interface
// ==============================================================

// Initialisation
DLLCLBK VESSEL *ovcInit (OBJHANDLE hvessel, int flightmodel)
{
	return new Atlantis_Tank (hvessel);
}

// Cleanup
DLLCLBK void ovcExit (VESSEL *vessel)
{
	if (vessel) delete (Atlantis_Tank*)vessel;
}
