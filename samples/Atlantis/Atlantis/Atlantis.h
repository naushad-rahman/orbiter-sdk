// ==============================================================
//                 ORBITER MODULE: Atlantis
//                  Part of the ORBITER SDK
//          Copyright (C) 2001-2003 Martin Schweiger
//                   All rights reserved
//
// Atlantis.h
// Class interface of Atlantis (Space Shuttle) vessel class
// module and associated subclasses (SRB, tank)
// ==============================================================

#ifndef __ATLANTIS_H
#define __ATLANTIS_H

#include "orbitersdk.h"
#include <math.h>

// ==========================================================
// Some Orbiter-related parameters
// ==========================================================

const double ORBITER_EMPTY_MASS = 104326.0;
// Orbiter empty mass [kg]

const double ORBITER_MAX_PROPELLANT_MASS = 21600.0;
// Amount of fuel the orbiter can hold in internal OMS tanks

const double ORBITER_MAIN_THRUST = 1668652.0 * 1.25;
// Vacuum thrust rating per main engine [N] (x3 for total)
// assuming vacuum thrust is 5/4 liftoff thrust

const double ORBITER_OMS_THRUST = 26700.0;
// Vacuum thrust per unit for Orbital Maneuvering System [N] (x2 for total)

const double ORBITER_RCS_THRUST = 7740.0;
// Vacuum thrust rating for attitude thrusters (Reaction Control System) [N]

const double ORBITER_MAIN_ISP0 = 5000.0;
const double ORBITER_MAIN_ISP1 = 4000.0;
// Vacuum and sea-level fuel-specific impulse for orbiter main engines [m/s]
// using H2/O2 (hydrogen/oxygen)
// The correct ISP0 is about 3532 m/s!

const double ORBITER_OMS_ISP0 = 3100;
const double ORBITER_OMS_ISP1 = 2500;
// Vacuum and sea-level fuel-specific impulse for Orbital Maneuvering System [m/s]
// using MMH/N2O4 (monomethyl hydrazine/nitrogen tetroxide)

const double ORBITER_RCS_ISP0 = 5000.0;
const double ORBITER_RCS_ISP1 = 4000.0;
// Vacuum and sea-level fuel-specific impulse for Reaction Control System [m/s]

const double GEAR_OPERATING_SPEED = 0.3;
// Opening/closing speed of landing gear (1/sec)
// => gear cycle ~ 3 sec

const double DOOR_OPERATING_SPEED = 0.007353;
// Opening/closing speed of payload bay doors (1/sec)
// This contains the door opening sequence (63 sec for each door) and an
// interval of 10 sec between the two door operations

const double RAD_OPERATING_SPEED = 0.025;
// Deployment/stowing speed of radiators (1/sec)
// => radiator cycle = 40 sec

const double RADLATCH_OPERATING_SPEED = 0.2;
// Release/engaging speed of radiator latches (1/sec)
// => radiator latch cycle = 5 sec

const double KU_OPERATING_SPEED = 0.0435;
// Deployment speed of the Ku Band antenna (1/sec)
// cycle is 23 sec

const double SPEEDBRAKE_OPERATING_SPEED = 0.20284;
// Deployment speed of the speedbrake (1/sec)
// cycle is 4.93 sec

const double ARM_OPERATING_SPEED = 0.005;
// RMS arm joint rotation speed (rad/sec)

const VECTOR3 ORBITER_CS = {234.8,389.1,68.2};
// Orbiter cross sections (projections into principal axes) [m^2]

const VECTOR3 ORBITER_CS_GEAR = {10.0,0.0,3.0};
// Contribution of fully extended landing gear to cross sections

const double ORBITER_CW[4] = {0.13, 0.5, 1.4, 1.4};
// Orbiter wind resistance coefficients in principal directions (+z, -z, +-x, +-y)

const double ORBITER_CW_GEAR[4] = {0.04, 0.04, 0.05, 0.0};
// Contribution of fully extended landing gear to wind resistance

const double MAX_GRAPPLING_DIST = 0.5;
// max distance between RMS tip and grappling point for successful grappling

// ==========================================================
// Some Tank-related parameters
// ==========================================================

const double TANK_MAX_PROPELLANT_MASS = 719115.0;
// Main tank propellant mass [kg]

const double TANK_EMPTY_MASS = 35425.0;
// Main tank empty mass

// ==========================================================
// Some SRB-related parameters
// ==========================================================

const double SRB_MAX_PROPELLANT_MASS = 502126.0;
// SRB propellant mass [kg]

const double SRB_EMPTY_MASS = 87543.0;
// SRB empty mass [kg]

const double SRB_ISP0 = 3574.68;
const double SRB_ISP1 = 2859.74;
// SRB vacuum and sea-level fuel-specific impulse [m/s]

const double SRB_THRUST = 1202020.0*9.81 * 1.25;
// Vacuum SRB thrust per unit [N]

const double MAX_ATT_LAUNCH = 1e5;
const double MAX_ROLL_SRB = 2.5e5;
// Max attitude thrust during launch phase (temporary)

const double SRB_STABILISATION_TIME = 4.0;
// MET: -SRB ignition

const double SRB_SEPARATION_TIME = 126.0;
// MET: SRB separation

const double SRB_CUTOUT_TIME = 135.0;
// MET: engine shutdown

// ==========================================================
// Mesh offsets for various configurations
// ==========================================================

const VECTOR3 OFS_ZERO             = { 0.0, 0.0,  0.0  };
const VECTOR3 OFS_LAUNCH_ORBITER   = { 0.0, 6.22,-7.795};
const VECTOR3 OFS_LAUNCH_TANK      = { 0.0,-1.91, 5.72 };
const VECTOR3 OFS_LAUNCH_RIGHTSRB  = { 6.2,-1.91,-5.68 };
const VECTOR3 OFS_LAUNCH_LEFTSRB   = {-6.2,-1.91,-5.68 };
const VECTOR3 OFS_WITHTANK_ORBITER = { 0.0, 4.79,-9.185};
const VECTOR3 OFS_WITHTANK_TANK    = { 0.0,-3.34, 4.33 };
const VECTOR3 ORBITER_DOCKPOS      = { 0.0, 2.44,10.44 };
const VECTOR3 OFS_MMU              = {0,2.44,10.44};

// ==========================================================
// Mesh group indices for some components
// ==========================================================

const UINT MESH_UNDEFINED = (UINT)-1;

// ==========================================================
// panel area identifiers
// ==========================================================

// define MFD function buttons
#define AID_CDR1_BUTTONS   1
#define AID_CDR2_BUTTONS   2
#define AID_PLT1_BUTTONS   3
#define AID_PLT2_BUTTONS   4
#define AID_MFD1_BUTTONS   5
#define AID_MFD2_BUTTONS   6
#define AID_MFD3_BUTTONS   7
#define AID_MFD4_BUTTONS   8
#define AID_MFD5_BUTTONS   9
#define AID_MFDA_BUTTONS  10
// D. Beachy: define power buttons
#define AID_CDR1_PWR      11
#define AID_CDR2_PWR      12
#define AID_PLT1_PWR      13
#define AID_PLT2_PWR      14
#define AID_MFD1_PWR      15
#define AID_MFD2_PWR      16
#define AID_MFD3_PWR      17
#define AID_MFD4_PWR      18
#define AID_MFD5_PWR      19
#define AID_MFDA_PWR      20
// MFD brightness buttons
#define AID_CDR1_BRT      21
#define AID_CDR2_BRT      22
#define AID_PLT1_BRT      23
#define AID_PLT2_BRT      24
#define AID_MFD1_BRT      25
#define AID_MFD2_BRT      26
#define AID_MFD3_BRT      27
#define AID_MFD4_BRT      28
#define AID_MFD5_BRT      29
#define AID_MFDA_BRT      30
// Panel R13L (payload bay operations)
#define AID_R13L_MIN     100
#define AID_R13L         100
#define AID_R13L_TKBK1   101
#define AID_R13L_TKBK2   102
#define AID_R13L_TKBK3   103
#define AID_R13L_TKBK4   104
#define AID_R13L_TKBK5   105
#define AID_R13L_TKBK6   106
#define AID_R13L_MAX     120

typedef struct {
	HINSTANCE hDLL;
	SURFHANDLE tkbk_label;
	HFONT font[1];
} GDIParams;

// ==========================================================
// Interface for derived vessel class: Atlantis
// ==========================================================

class Atlantis: public VESSEL3 {
	friend class PayloadBayOp;
	friend BOOL CALLBACK RMS_DlgProc (HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
public:
	AnimState::Action gear_status, spdb_status;
	Atlantis (OBJHANDLE hObj, int fmodel);
	~Atlantis();
	void SetLaunchConfiguration (void);
	void SetPostLaunchConfiguration (double srbtime);
	void SetOrbiterTankConfiguration (void);
	void SetOrbiterConfiguration (void);
	void DefineAnimations (void);
	void AddOrbiterVisual (const VECTOR3 &ofs);
	void AddTankVisual (const VECTOR3 &ofs);
	void AddSRBVisual (int which, const VECTOR3 &ofs);
	void SeparateBoosters (double srb_time);
	void SeparateTank (void);
	void SeparateMMU (void);
	void ToggleGrapple (void);
	void ToggleArrest (void);
	void SetGearParameters (double state);
	void Jettison ();
	void UpdateMesh ();
	void ClearMeshes ();
	void SetBayDoorPosition (double pos);
	void SetRadiatorPosition (double pos);
	void SetRadLatchPosition (double pos) {}
	void SetKuAntennaPosition (double pos);
	void OperateLandingGear (AnimState::Action action);
	void RevertLandingGear ();
	void OperateSpeedbrake (AnimState::Action action);
	void RevertSpeedbrake ();
	void SetAnimationArm (UINT anim, double state);

	void RegisterVC_CdrMFD ();
	void RegisterVC_PltMFD ();
	void RegisterVC_CntMFD ();
	void RegisterVC_AftMFD ();
	void RedrawPanel_MFDButton (SURFHANDLE surf, int mfd);

	int status; // 0=launch configuration
	            // 1=SRB's engaged
	            // 2=SRB's separated
	            // 3=Tank separated (orbiter only)
	bool bManualSeparate; // flag for user-induced booster or tank separation

	double t0;          // reference time: designated liftoff time
	WORD srb_id1, srb_id2;

	double gear_proc; // landing gear deployment state (0=retracted, 1=deployed)
	//double kubd_proc; // Ku-band antenna deployment state (0=retracted, 1=deployed)
	double spdb_proc; // Speedbrake deployment state (0=retracted, 1=deployed)
	double ldoor_drag, rdoor_drag; // drag components from open cargo doors
	bool center_arm;
	bool arm_moved;
	double center_arm_t;
	bool do_eva;
	bool do_plat;
	bool do_cargostatic;
	VECTOR3 orbiter_ofs;
	VECTOR3 ofs_sts_sat;
	VECTOR3 cargo_static_ofs;
	VISHANDLE vis;      // handle for visual - note: we assume that only one visual per object is created!
	MESHHANDLE hOrbiterMesh, hOrbiterCockpitMesh, hOrbiterVCMesh, hTankMesh, hSRBMesh; // mesh handles
	char cargo_static_mesh_name[256];
	ATTACHMENTHANDLE sat_attach, rms_attach;
	VECTOR3 arm_tip[3];

	// Overloaded callback functions
	void clbkSetClassCaps (FILEHANDLE cfg);
	void clbkSetStateEx (const void *status);
	void clbkLoadStateEx (FILEHANDLE scn, void *vs);
	void clbkSaveState (FILEHANDLE scn);
	void clbkFocusChanged (bool getfocus, OBJHANDLE hNewVessel, OBJHANDLE hOldVessel);
	void clbkPostStep (double simt, double simdt, double mjd);
	bool clbkPlaybackEvent (double simt, double event_t, const char *event_type, const char *event);
	int  clbkConsumeBufferedKey (DWORD key, bool down, char *kstate);
	void clbkVisualCreated (VISHANDLE vis, int refcount);
	void clbkVisualDestroyed (VISHANDLE vis, int refcount);
	void clbkAnimate (double simt);
	void clbkMFDMode (int mfd, int mode);
	bool clbkLoadGenericCockpit ();
	bool clbkLoadVC (int id);
	bool clbkVCMouseEvent (int id, int event, VECTOR3 &p);
	bool clbkVCRedrawEvent (int id, int event, SURFHANDLE surf);
	bool clbkDrawHUD (int mode, const HUDPAINTSPEC *hps, oapi::Sketchpad *skp);

	PayloadBayOp *plop; // control and status of payload bay operations

private:
	void AutoMainGimbal();
	void LaunchClamps();
	void CreateAttControls (double th_pitch, double th_roll, double th_yaw, double isp0, double isp1);
	void CreateAttControls_Launch();
	void CreateAttControls_RCS();
	bool SatGrappled() const { return GetAttachmentStatus (rms_attach) != 0; }
	bool SatStowed() const { return GetAttachmentStatus (sat_attach) != 0; }
	ATTACHMENTHANDLE CanArrest() const;

	UINT anim_door;                            // handle for cargo door animation
	UINT anim_rad;                             // handle for radiator animation
	UINT anim_gear;                            // handle for landing gear animation
	UINT anim_kubd;                            // handle for Ku-band antenna animation
	UINT anim_elev;                            // handle for elevator animation
	UINT anim_laileron;						   // handle for left aileron animation
	UINT anim_raileron;						   // handle for right aileron animation
	UINT anim_rudder;						   // handle for rudder animation
	UINT anim_spdb;                            // handle for speed brake animation
	UINT mesh_orbiter;                         // index for orbiter mesh
	UINT mesh_cockpit;                         // index for cockpit mesh for external view
	UINT mesh_vc;                              // index for virtual cockpit mesh
	UINT mesh_tank;                            // index for external tank mesh
	UINT mesh_srb[2];                          // index for SRB meshes
	PROPELLANT_HANDLE ph_oms, ph_tank, ph_srb; // handles for propellant resources
	THRUSTER_HANDLE th_main[3];                // handles for orbiter main engines
	THRUSTER_HANDLE th_srb[2];                 // handles for SRB engines
	THGROUP_HANDLE thg_main, thg_srb;          // handles for thruster groups

	// RMS arm animation status
	ANIMATIONCOMPONENT_HANDLE hAC_arm, hAC_sat, hAC_satref;
	MGROUP_TRANSFORM *rms_anim[6];
	UINT anim_arm_sy, anim_arm_sp, anim_arm_ep, anim_arm_wp, anim_arm_wy, anim_arm_wr;
	double arm_sy, arm_sp, arm_ep, arm_wp, arm_wy, arm_wr;

	MGROUP_TRANSFORM *sat_anim, *sat_ref;

	bool reset_sat;
	OBJHANDLE hMMU, hSAT;
	bool render_cockpit;
	VCHUDSPEC huds;
	EXTMFDSPEC mfds[10];
	double mfdbright[10];

	LightEmitter *engine_light;
	double engine_light_level;
};

// ==========================================================
// Interface for derived vessel class: Atlantis_SRB
// ==========================================================

class Atlantis_SRB: public VESSEL2 {
public:
	Atlantis_SRB (OBJHANDLE hObj);
	// Construct interface from existing object

	void SetRefTime (void);

	// Overloaded callback functions
	void clbkSetClassCaps (FILEHANDLE cfg);
	void clbkPostStep (double simt, double simdt, double mjd);
	void clbkPostCreation ();

private:
	double t0;                  // reference time: liftoff
	double srb_separation_time; // simulation time at which SRB separation was initiated
	bool bMainEngine;           // main engine firing?
	bool bSeparationEngine;     // separation thrusters firing?
	MESHHANDLE hSRBMesh;
	PROPELLANT_HANDLE ph_main;  // handle for propellant resource
	THRUSTER_HANDLE th_main;    // engine handle
	THRUSTER_HANDLE th_bolt;    // separation bolt
};

// ==========================================================
// Interface for derived vessel class: Atlantis_Tank
// ==========================================================

class Atlantis_Tank: public VESSEL2 {
public:
	Atlantis_Tank (OBJHANDLE hObj);
	// Construct interface from existing object

	// Overloaded callback functions
	void clbkSetClassCaps (FILEHANDLE cfg);
	void clbkPostStep (double simt, double simdt, double mjd);

private:
	MESHHANDLE hTankMesh;
};

#endif // !__ATLANTIS_H
