// ==============================================================
//                 ORBITER MODULE:  ShuttleA
//                  Part of the ORBITER SDK
//          Copyright (C) 2002-2004 Martin Schweiger
//                   All rights reserved
//
// ShuttleA.h
// Class interface for Shuttle-A vessel class module
// ==============================================================

#ifndef __SHUTTLEA_H
#define __SHUTTLEA_H

#include "orbitersdk.h"

// ==========================================================
// Some vessel class caps
// ==========================================================

//56000 Liters per box... @20 tones per box * 6 boxes+ 13 tons the skeleton

const double EMPTY_MASS = 13000;
// Empty mass (excluding fuel) [kg]

//fuel tanks are: 1.7radius and 4m length = 36315 Liters / tank
//2 fuel tanks + internal one = 10000 Liters 
// Total = 82631 Liters for fuel
// LOX LH2 density  0.865g/cc =>
const double MAX_MAIN_FUEL = 71500.0; //oau !?
// Max fuel capacity: main tank [kg]

const double MAX_RCS_FUEL = 2700.0;
// Max fuel capacity: RCS tank [kg]

const double MAX_MAIN_THRUST = 2128720.0/2.0;
// Main engine thrust [N] per engine

const double MAX_HOVER_THRUST = 1489950.0/2.0;
// Hover engine thrust [N] per engine

const double MAX_RETRO_THRUST = 770000.0/2.0;
// Max thrust [N] of retro/hover pods per engine

const double MAX_RCS_THRUST = 4500.0;
// Max thrust [N] for each attitude thruster

const double ISP = 3e4;
// Sea-level Isp (fuel-specific impulse) for all thrusters [m/s] 

const double ISP_P0 =3.3e4;
// A better thrust/ISP in vacuum for all but RCS

const double DOCK_OPERATING_SPEED = 0.05;
// Opening/closing speed of docking hatch mechanism (1/sec)
// => cycle = 20 sec

const double AIRLOCK_OPERATING_SPEED = 0.1;
// Opening/closing speed of outer airlock (1/sec)
// => cycle = 10 sec

const double GEAR_OPERATING_SPEED = 0.25;
// Opening/closing speed of gear (1/sec)
// => cycle = 4 sec

const double POD_ROTREQUEST_SPEED = 0.4;
// speed at which the pod tilt switches react to preset changes

const double POD_ROT_SPEED = 0.2;
// speed at which pods actually rotate

const int nsurf = 6; // number of bitmap handles

const double MAX_GRAPPLING_DIST = 0.5f;
const double MAX_GRAPPLING_ANG  = 0.9f; //(cos(angle)>0.9f)
//distance for payload grappling.. being a bit generous here

// ==========================================================
// Interface for derived vessel class: ShuttleA
// ==========================================================

class ShuttleA: public VESSEL3 {
public:
	ShuttleA (OBJHANDLE hObj, int fmodel);
	~ShuttleA ();
	void ReleaseSurfaces ();
	void InitPanel (int panel);
	void RotatePod (UINT which, UINT mode);
	void PresetPod (UINT which, double angle);
	void SetPodAngle (UINT which, double angle);
	inline double GetPodAngle (UINT which) { return pod_angle[which]; }

	void RedrawPanel_MFDButton (SURFHANDLE surf, int mfd, int side);
	void RedrawPanel_Navmode (SURFHANDLE surf);
	bool RedrawPanel_Throttle (SURFHANDLE surf);
	bool RedrawPanel_Hover (SURFHANDLE surf);
	bool RedrawPanel_Podlevel (SURFHANDLE surf);
	bool RedrawPanel_EngineIndicator (SURFHANDLE surf);
	bool RedrawPanel_PodangleIndicator (SURFHANDLE surf);
	void RedrawPanel_Fuelstatus (SURFHANDLE surf, int part);
	void RedrawPanel_CargoOpen(SURFHANDLE surf);
	void RedrawPanel_CargoPresent(SURFHANDLE surf);

	SURFHANDLE srf[nsurf];
	PROPELLANT_HANDLE ph_main, ph_rcs;
	THRUSTER_HANDLE th_main[2], th_hover[2], th_pod[2];
	MESHHANDLE vcmesh_tpl,exmesh_tpl;
	UINT podswitch[2];
	double dock_proc, lock_proc,gear_proc;
	enum DoorStatus { DOOR_CLOSED, DOOR_OPEN, DOOR_CLOSING, DOOR_OPENING } dock_status, lock_status,gear_status;

	void ActivateDockingPort (DoorStatus action);
	void RevertDockingPort ();
	void ActivateAirlock (DoorStatus action);
	void RevertAirlock ();
	void ActivateLandingGear (DoorStatus action);
	void RevertLandingGear();
	void ActivateCargo (int status);

	// Overloaded callback functions
	void clbkSetClassCaps (FILEHANDLE cfg);
	void clbkLoadStateEx (FILEHANDLE scn, void *vs);
	void clbkSaveState (FILEHANDLE scn);
	void clbkPostCreation ();
	bool clbkPlaybackEvent (double simt, double event_t, const char *event_type, const char *event);
	void clbkPostStep (double simt, double simdt, double mjd);
	void clbkMFDMode (int mfd, int mode);
	void clbkNavMode (int mode, bool active);
	int  clbkConsumeBufferedKey (DWORD key, bool down, char *kstate);
	int  clbkConsumeDirectKey (char *kstate);
	bool clbkLoadPanel (int id);
	bool clbkPanelMouseEvent (int id, int event, int mx, int my);
	bool clbkPanelRedrawEvent (int id, int event, SURFHANDLE surf);
	bool clbkDrawHUD (int mode, const HUDPAINTSPEC *hps, oapi::Sketchpad *skp);
	void clbkRenderHUD (int mode, const HUDPAINTSPEC *hps, SURFHANDLE hTex);
	bool clbkLoadVC (int id);
	bool clbkVCRedrawEvent (int id, int event, SURFHANDLE surf);
	bool clbkVCMouseEvent (int id, int event, VECTOR3 &p);

private:
	void DefineAnimations ();
	bool ToggleGrapple (int grapple);
	double payload_mass;
	void ComputePayloadMass();
	ATTACHMENTHANDLE  payload_attachment[6];

	THGROUP_HANDLE thg_main, thg_hover, thg_pod;
	double pod_angle[2], pod_angle_request[2];
	
	int cargo_open[6];		//Cargo control buttons status
	int cargo_arm_status;

	UINT anim_pod[2], anim_dock, anim_lock;
	UINT sliderpos_main[2], sliderpos_hovr[2];
	UINT sliderpos_retro[2], sliderpos_auxhovr[2], sliderpos_pod[2];
    UINT anim_gear;

	UINT sliderpos_main_v[2], sliderpos_hovr_v[2];
	UINT sliderpos_retro_v[2], sliderpos_auxhovr_v[2], sliderpos_pod_v[2];

	UINT anim_pod_thrust_left;      // VC pod thruster animation
	UINT anim_pod_thrust_right;      // VC pod thruster animation

	UINT anim_main_thrust_left;
	UINT anim_main_thrust_right;

	UINT anim_hover_thrust_left;
	UINT anim_hover_thrust_right;

    UINT anim_pod_angle;
	UINT anim_rcs_mode;

	UINT anim_dock_switch;
	UINT anim_airlock_switch;
	UINT anim_gear_switch;
	UINT anim_cargo_switch;

	void RedrawVC_ThPOD();
	void RedrawVC_ThMain();
	void RedrawVC_ThHover();
};

typedef struct {
	HINSTANCE hDLL;
	HFONT hFont[1];
	HPEN hPen[2];
	HBRUSH hBrush[1];
} GDIParams;

#define AID_MFD1_LBUTTONS      0
#define AID_MFD1_RBUTTONS      1
#define AID_MFD1_BBUTTONS      2
#define AID_MFD2_LBUTTONS      3
#define AID_MFD2_RBUTTONS      4
#define AID_MFD2_BBUTTONS      5

#define AID_ENGINEMAIN         6
#define AID_ENGINEHOVER        7
#define AID_ENGINEPODLEVEL     8
#define AID_ENGINEINDICATOR    9
#define AID_PODANGLEINDICATOR 10
#define AID_PODANGLESWITCH    11
#define AID_PODANGLEPRESET    12
#define AID_NAVMODE           13
#define AID_ATTITUDEMODE      14

#define AID_FUELSTATUS1       20
#define AID_FUELSTATUS2       21
#define AID_FUELSTATUS3       22
#define AID_FUELSTATUS4       23
#define AID_AIRLOCK1SWITCH    24
#define AID_AIRLOCK1INDICATOR 25
#define AID_DOCKSWITCH        26
#define AID_DOCKINDICATOR     27

#define AID_GEARSWITCH		  28
#define AID_GEARINDICATOR	  29

#define AID_CARGO_OPEN		  30
#define AID_GARGOARMSWITCH	  31
#define AID_CARGOARMINDICATOR 32
#define AID_CARGOPRESENT	  33 // TODO
#endif // !__SHUTTLEA_H