// ==============================================================
//                 ORBITER MODULE: Atlantis
//                  Part of the ORBITER SDK
//          Copyright (C) 2001-2003 Martin Schweiger
//                   All rights reserved
//
// Atlantis.cpp
// Reference implementation of Atlantis (Space Shuttle) vessel
// class module
//
// RMS, grappling and MMU capabilities by Robert Conley
// ==============================================================

#define STRICT 1
#define ORBITER_MODULE
#include "Atlantis.h"
#include "PlBayOp.h"
#include "DlgCtrl.h"
#include "meshres.h"
#include "meshres_vc.h"
#include "resource.h"
#include <stdio.h>
#include <fstream>

#ifdef _DEBUG
    // D. Beachy: for BoundsChecker debugging
    extern int GrowStack();
#endif

#define LOADBMP(id) (LoadBitmap (g_Param.hDLL, MAKEINTRESOURCE (id)))

// ==============================================================
// Global (class-wide) parameters

GDIParams g_Param;

char *ActionString[5] = {"STOPPED", "ISCLOSED", "ISOPEN", "CLOSE", "OPEN"};

HELPCONTEXT g_hc = {
	"html/vessels/Atlantis.chm",
	0,
	"html/vessels/Atlantis.chm::/Atlantis.hhc",
	"html/vessels/Atlantis.chm::/Atlantis.hhk"
};


// ==============================================================
// Local prototypes

BOOL CALLBACK Atlantis_DlgProc (HWND, UINT, WPARAM, LPARAM);
BOOL CALLBACK RMS_DlgProc (HWND, UINT, WPARAM, LPARAM);
extern void GetSRB_State (double met, double &thrust_level, double &prop_level);

// ==============================================================
// Airfoil coefficient functions
// Return lift, moment and zero-lift drag coefficients as a
// function of angle of attack (alpha or beta)
// ==============================================================

// 1. vertical lift component (wings and body)

void VLiftCoeff (double aoa, double M, double Re, double *cl, double *cm, double *cd)
{
	static const double step = RAD*15.0;
	static const double istep = 1.0/step;
	static const int nabsc = 25;
	static const double CL[nabsc] = {0.1, 0.17, 0.2, 0.2, 0.17, 0.1, 0, -0.11, -0.24, -0.38,  -0.5,  -0.5, -0.02, 0.6355,    0.63,   0.46, 0.28, 0.13, 0.0, -0.16, -0.26, -0.29, -0.24, -0.1, 0.1};
	static const double CM[nabsc] = {  0,    0,   0,   0,    0,   0, 0,     0,    0,0.002,0.004, 0.0025,0.0012,      0,-0.0012,-0.0007,    0,    0,   0,     0,     0,     0,     0,    0,   0};
	// lift and moment coefficients from -180 to 180 in 15 degree steps.
	// This uses a documented lift slope of 0.0437/deg, everything else is rather ad-hoc

	aoa += PI;
	int idx = max (0, min (23, (int)(aoa*istep)));
	double d = aoa*istep - idx;
	*cl = CL[idx] + (CL[idx+1]-CL[idx])*d;
	*cm = CM[idx] + (CM[idx+1]-CM[idx])*d;
	*cd = 0.06 + oapiGetInducedDrag (*cl, 2.266, 0.6);
}

// 2. horizontal lift component (vertical stabiliser and body)

void HLiftCoeff (double beta, double M, double Re, double *cl, double *cm, double *cd)
{
	static const double step = RAD*22.5;
	static const double istep = 1.0/step;
	static const int nabsc = 17;
	static const double CL[nabsc] = {0, 0.2, 0.3, 0.2, 0, -0.2, -0.3, -0.2, 0, 0.2, 0.3, 0.2, 0, -0.2, -0.3, -0.2, 0};

	beta += PI;
	int idx = max (0, min (15, (int)(beta*istep)));
	double d = beta*istep - idx;
	*cl = CL[idx] + (CL[idx+1]-CL[idx])*d;
	*cm = 0.0;
	*cd = 0.02 + oapiGetInducedDrag (*cl, 1.5, 0.6);
}

// ==============================================================
// Specialised vessel class Atlantis
// ==============================================================

// --------------------------------------------------------------
// Constructor
// --------------------------------------------------------------
Atlantis::Atlantis (OBJHANDLE hObj, int fmodel)
: VESSEL3 (hObj, fmodel)
{
#ifdef _DEBUG
        // D. Beachy: for BoundsChecker debugging
        GrowStack();
#endif
	int i;

	plop            = new PayloadBayOp (this);
	status          = 3;
	gear_status     = AnimState::CLOSED;
	gear_proc       = 0.0;
	ldoor_drag      = rdoor_drag = 0.0;
	spdb_status     = AnimState::CLOSED;
	spdb_proc       = 0.0;

	engine_light_level = 0.0;
	COLOUR4 col_diff = {1,1,1,0};
	COLOUR4 col_zero = {0,0,0,0};
	engine_light = AddPointLight (_V(0,0,-25), 300, 2e-4, 0, 4e-4, col_diff, col_zero, col_zero);
	engine_light->SetIntensityRef (&engine_light_level);

	mesh_orbiter    = MESH_UNDEFINED;
	mesh_cockpit    = MESH_UNDEFINED;
	mesh_vc         = MESH_UNDEFINED;
	mesh_tank       = MESH_UNDEFINED;
	mesh_srb[0] = mesh_srb[1] = MESH_UNDEFINED;

	vis             = NULL;
	
	reset_sat       = false;
	render_cockpit  = false;

	int mfdgrp[10] = {
		GRP_CDR1_VC,GRP_CDR2_VC,GRP_PLT1_VC,GRP_PLT2_VC,
		GRP_MFD1_VC, GRP_MFD2_VC, GRP_MFD3_VC, GRP_MFD4_VC, GRP_MFD5_VC,
		GRP_MFD_aft_VC};
	for (i = 0; i < 10; i++) {
		mfds[i].ngroup   = mfdgrp[i];
		mfds[i].flag     = MFD_SHOWMODELABELS;
		mfds[i].nbt1     = 5;
		mfds[i].nbt2     = 0;
		mfds[i].bt_yofs  = 256/6;
		mfds[i].bt_ydist = 256/7;
	}
	for (i = 0; i < 10; i++)
		mfdbright[i] =  1.0;
	huds.ngroup       = GRP_VirtualHUD_VC;
	huds.size         = 0.176558;

	// propellant resources
	ph_oms          = NULL;
	ph_tank         = NULL;
	ph_srb          = NULL;
	thg_main        = NULL;
	thg_srb         = NULL;

	// preload meshes
	hOrbiterMesh        = oapiLoadMeshGlobal ("Atlantis\\Atlantis");
	hOrbiterCockpitMesh = oapiLoadMeshGlobal ("Atlantis\\AtlantisCockpit");
	hOrbiterVCMesh      = oapiLoadMeshGlobal ("Atlantis\\AtlantisVC");
	hTankMesh           = oapiLoadMeshGlobal ("Atlantis_tank");
	hSRBMesh            = oapiLoadMeshGlobal ("Atlantis_srb");


	DefineAnimations();
	center_arm      = false;
	arm_moved       = false;
	bManualSeparate = false;
	ofs_sts_sat     = _V(0,0,0);      
	do_eva          = false;
	do_plat         = false;
	do_cargostatic  = false;
	vis             = NULL;
	sat_attach      = NULL;
	rms_attach      = NULL;
	cargo_static_ofs   =_V(0,0,0);

	// default arm status: stowed
	arm_sy = 0.5;
	arm_sp = 0.0;
	arm_ep = 0.0;
	arm_wp = 0.5;
	arm_wy = 0.5;
	arm_wr = 0.5;
	arm_tip[0] = _V(-2.26,1.71,-6.5);
	arm_tip[1] = _V(-2.26,1.71,-7.5);
	arm_tip[2] = _V(-2.26,2.71,-6.5);
}

// --------------------------------------------------------------
// Destructor
// --------------------------------------------------------------
Atlantis::~Atlantis ()
{
	delete plop;
	int i;
	for (i = 0; i < 6; i++) delete rms_anim[i];
}

// --------------------------------------------------------------
// launch configuration setup (orbiter + tank + SRB)
// --------------------------------------------------------------
void Atlantis::SetLaunchConfiguration (void)
{
	int i;
	extern PARTICLESTREAMSPEC srb_contrail, srb_exhaust;

	// *********************** physical parameters *********************************

	SetSize (30.0);
	SetEmptyMass (ORBITER_EMPTY_MASS + TANK_EMPTY_MASS + 2*SRB_EMPTY_MASS);
	SetCW (0.2, 0.5, 1.5, 1.5);
	SetWingAspect (0.7);
	SetWingEffectiveness (2.5);
	SetCrossSections (_V(687.4,849.5,189.4));
	SetRotDrag (_V(0.7,0.1,0.3));
	SetPMI (_V(179.1,176.8,29.3));
	SetTrimScale (0.05);
	SetLiftCoeffFunc (0); // simplification: we assume no lift during launch phase
	SetTouchdownPoints (_V(0,-10,-53.3), _V(-7,7,-53.3), _V(7,7,-53.3));

	// ************************* propellant specs **********************************

	if (!ph_oms)  ph_oms  = CreatePropellantResource (ORBITER_MAX_PROPELLANT_MASS); // OMS propellant
	if (!ph_tank) ph_tank = CreatePropellantResource (TANK_MAX_PROPELLANT_MASS);    // main tank
	if (!ph_srb)  ph_srb  = CreatePropellantResource (SRB_MAX_PROPELLANT_MASS*2.0); // SRB's
	SetDefaultPropellantResource (ph_tank); // display main tank level in generic HUD

	// *********************** thruster definitions ********************************

	// The main and SRB thrusters are defined so as to minimise the angular momentum
	// when engaged at maximum thrust. Dynamic gimbaling is not required until the SRBs
	// start to lose power since the centre of gravity is assumed static.
	// However the resulting linear force vector has a component in +y ("up") direction

	ClearThrusterDefinitions();

	// orbiter main thrusters
	th_main[0] = CreateThruster (OFS_LAUNCH_ORBITER+_V(-1.6,-0.2,-16.0), _V( 0.04994,0.0,0.99875), ORBITER_MAIN_THRUST, ph_tank, ORBITER_MAIN_ISP0, ORBITER_MAIN_ISP1);
	th_main[1] = CreateThruster (OFS_LAUNCH_ORBITER+_V( 1.6,-0.2,-16.0), _V(-0.04994,0.0,0.99875), ORBITER_MAIN_THRUST, ph_tank, ORBITER_MAIN_ISP0, ORBITER_MAIN_ISP1);
	th_main[2] = CreateThruster (OFS_LAUNCH_ORBITER+_V( 0.0, 3.2,-15.5), _V( 0.0,-0.13,1), ORBITER_MAIN_THRUST, ph_tank, ORBITER_MAIN_ISP0, ORBITER_MAIN_ISP1);
	thg_main = CreateThrusterGroup (th_main, 3, THGROUP_MAIN);
	SURFHANDLE tex_main = oapiRegisterExhaustTexture ("Exhaust_atsme");
	for (i = 0; i < 3; i++) AddExhaust (th_main[i], 30.0, 2.0, tex_main);

	// SRBs
	th_srb[0] = CreateThruster (OFS_LAUNCH_RIGHTSRB+_V(0.0,0.0,-21.8), _V(0,0.023643,0.999720), SRB_THRUST, ph_srb, SRB_ISP0, SRB_ISP1);
	th_srb[1] = CreateThruster (OFS_LAUNCH_LEFTSRB +_V(0.0,0.0,-21.8), _V(0,0.023643,0.999720), SRB_THRUST, ph_srb, SRB_ISP0, SRB_ISP1);
	thg_srb = CreateThrusterGroup (th_srb, 2, THGROUP_USER);
	SURFHANDLE tex = oapiRegisterExhaustTexture ("Exhaust2");
	srb_exhaust.tex = oapiRegisterParticleTexture ("Contrail2");
	srb_contrail.tex = oapiRegisterParticleTexture ("Contrail4");
	for (i = 0; i < 2; i++) AddExhaust (th_srb[i], 16.0, 2.0, tex);
	AddExhaustStream (th_srb[0], OFS_LAUNCH_RIGHTSRB+_V(0,0,/*-30*/-50), &srb_contrail);
	AddExhaustStream (th_srb[1], OFS_LAUNCH_LEFTSRB+_V(0,0,/*-30*/-50), &srb_contrail);
	AddExhaustStream (th_srb[0], OFS_LAUNCH_RIGHTSRB+_V(0,0,-25), &srb_exhaust);
	AddExhaustStream (th_srb[1], OFS_LAUNCH_LEFTSRB+_V(0,0,-25), &srb_exhaust);

	// attitude - this is temporary
	// attitude adjustment during launch phase should really be done via SRB gimbaling
	CreateAttControls_Launch();

	// ************************* aerodynamics **************************************

	ClearVariableDragElements ();
	CreateVariableDragElement (&spdb_proc, 5, OFS_LAUNCH_ORBITER+_V(0, 7.5, -14)); // speedbrake drag
	CreateVariableDragElement (&gear_proc, 2, OFS_LAUNCH_ORBITER+_V(0,-3,0));      // landing gear drag
	CreateVariableDragElement (&rdoor_drag, 7, OFS_LAUNCH_ORBITER+_V(2.9,0,10));   // right cargo door drag
	CreateVariableDragElement (&ldoor_drag, 7, OFS_LAUNCH_ORBITER+_V(-2.9,0,10));  // right cargo door drag

	// ************************ visual parameters **********************************

	AddOrbiterVisual (OFS_LAUNCH_ORBITER);
	AddTankVisual    (OFS_LAUNCH_TANK);
	AddSRBVisual     (0, OFS_LAUNCH_RIGHTSRB);
	AddSRBVisual     (1, OFS_LAUNCH_LEFTSRB);

	status = 0;
}

// --------------------------------------------------------------
// Configuration after launch, before SRB separation
// --------------------------------------------------------------
void Atlantis::SetPostLaunchConfiguration (double met)
{
	SetLaunchConfiguration();
	t0 = -met; // reference time (liftoff)
	status = 1;
}

// --------------------------------------------------------------
// Configuration after booster separation (orbiter + tank)
// --------------------------------------------------------------
void Atlantis::SetOrbiterTankConfiguration (void)
{
	int i;
	VECTOR3 ofs;
	
	// *********************** physical parameters *********************************

	SetSize (28.8);
	SetEmptyMass (ORBITER_EMPTY_MASS + TANK_EMPTY_MASS);
	SetCW (0.2, 0.5, 1.5, 1.5);
	SetWingAspect (0.7);
	SetWingEffectiveness (2.5);
	SetCrossSections (_V(646.1,603.0,141.5));
	SetRotDrag (_V(0.7,0.1,0.3));
	SetPMI (_V(168.2,154.1,24.2));
	SetTrimScale (0.05);
	SetLiftCoeffFunc (0); // simplification: we assume no lift during launch phase
	SetTouchdownPoints (_V(0,-5,30), _V(-10,-10,-30), _V(10,0,-30));

	// ************************* propellant specs **********************************

	if (!ph_oms)  ph_oms  = CreatePropellantResource (ORBITER_MAX_PROPELLANT_MASS); // OMS propellant
	if (!ph_tank) ph_tank = CreatePropellantResource (TANK_MAX_PROPELLANT_MASS);    // main tank
	SetDefaultPropellantResource (ph_tank); // display main tank level in generic HUD

	// *********************** thruster definitions ********************************

	// Orbiter main engines
	// The thruster directions are adjusted so as to generate no angular moment
	// i.e. sum_{i=1}^3 thrusterpos_i x thrusterdir_i = 0
	// This assumes that all 3 main engines generate the same amount of thrust at
	// all times

	ofs = OFS_WITHTANK_ORBITER;
	if (thg_main) { // main engines already defined - just modify parameters
		SetThrusterRef (th_main[0], ofs+_V(-1.6,-0.2,-16.0));
		SetThrusterDir (th_main[0], _V( 0.0624,-0.1789,0.9819));
		SetThrusterRef (th_main[1], ofs+_V( 1.6,-0.2,-16.0));
		SetThrusterDir (th_main[1], _V(-0.0624,-0.1789,0.9819));
		SetThrusterRef (th_main[2], ofs+_V( 0.0, 3.2,-15.5));
		SetThrusterDir (th_main[2], _V( 0.0,   -0.308046,0.951372));
	} else {        // create main engines
		th_main[0] = CreateThruster (ofs+_V(-1.6,-0.2,-16.0), _V( 0.0624,-0.1789,0.9819), ORBITER_MAIN_THRUST, ph_tank, ORBITER_MAIN_ISP0, ORBITER_MAIN_ISP1);
		th_main[1] = CreateThruster (ofs+_V( 1.6,-0.2,-16.0), _V(-0.0624,-0.1789,0.9819), ORBITER_MAIN_THRUST, ph_tank, ORBITER_MAIN_ISP0, ORBITER_MAIN_ISP1);
		th_main[2] = CreateThruster (ofs+_V( 0.0, 3.2,-15.5), _V( 0.0,   -0.308046,0.951372), ORBITER_MAIN_THRUST, ph_tank, ORBITER_MAIN_ISP0, ORBITER_MAIN_ISP1);
		thg_main = CreateThrusterGroup (th_main, 3, THGROUP_MAIN);
		SURFHANDLE tex_main = oapiRegisterExhaustTexture ("Exhaust_atsme");
		for (i = 0; i < 3; i++) AddExhaust (th_main[i], 30.0, 2.0, tex_main);
	}
	if (!ThrusterGroupDefined (THGROUP_ATT_PITCHUP))
		CreateAttControls_Launch();

	// ************************* aerodynamics **************************************

	ClearVariableDragElements ();
	CreateVariableDragElement (&spdb_proc, 5, ofs+_V(0, 7.5, -14)); // speedbrake drag
	CreateVariableDragElement (&gear_proc, 2, ofs+_V(0,-3,0));      // landing gear drag
	CreateVariableDragElement (&rdoor_drag, 7, ofs+_V(2.9,0,10));   // right cargo door drag
	CreateVariableDragElement (&ldoor_drag, 7, ofs+_V(-2.9,0,10));  // right cargo door drag

	// ************************ visual parameters **********************************

	AddOrbiterVisual (OFS_WITHTANK_ORBITER);
	AddTankVisual    (OFS_WITHTANK_TANK);

	status = 2;
}

// --------------------------------------------------------------
// Configuration after tank separation (orbiter only)
// --------------------------------------------------------------
void Atlantis::SetOrbiterConfiguration (void)
{
	int i;

	// *********************** physical parameters *********************************

	SetSize (19.6);
	SetEmptyMass (ORBITER_EMPTY_MASS);
	VECTOR3 r[2] = {{0,0,10},{0,0,-8}};
	SetPMI (_V(78.2,82.1,10.7));
	SetGravityGradientDamping (20.0);
	SetTrimScale (0.05);
	SetCW (ORBITER_CW[0], ORBITER_CW[1], ORBITER_CW[2], ORBITER_CW[3]);
	SetCrossSections (ORBITER_CS);
	SetGearParameters (gear_proc);

	if (GetFlightModel() >= 1) { // realistic flight model
	//	SetPitchMomentScale (1e-5);
	//	SetYawMomentScale (2e-5);
		if (gear_status == AnimState::OPEN) {
			SetCW (ORBITER_CW[0]+ORBITER_CW_GEAR[0],
				   ORBITER_CW[1]+ORBITER_CW_GEAR[1],
				   ORBITER_CW[2]+ORBITER_CW_GEAR[2],
				   ORBITER_CW[3]+ORBITER_CW_GEAR[3]);
			SetCrossSections (ORBITER_CS+ORBITER_CS_GEAR);
		}
	}

	// ************************* aerodynamics **************************************

	SetRotDrag (_V(0.43,0.43,0.29)); // angular drag

	CreateAirfoil (LIFT_VERTICAL,   _V(0,0,-0.5), VLiftCoeff, 20, 270, 2.266);
	CreateAirfoil (LIFT_HORIZONTAL, _V(0,0,-4), HLiftCoeff, 20,  50, 1.5);

	CreateControlSurface (AIRCTRL_ELEVATOR, 5.0, 1.5, _V( 0, 0,  -15), AIRCTRL_AXIS_XPOS, anim_elev);
	CreateControlSurface (AIRCTRL_RUDDER,   2.0, 1.5, _V( 0, 3,  -16), AIRCTRL_AXIS_YPOS, anim_rudder);
	CreateControlSurface (AIRCTRL_AILERON,  3.0, 1.5, _V( 7,-0.5,-15), AIRCTRL_AXIS_XPOS, anim_raileron);
	CreateControlSurface (AIRCTRL_AILERON,  3.0, 1.5, _V(-7,-0.5,-15), AIRCTRL_AXIS_XNEG, anim_laileron);

	ClearVariableDragElements ();
	CreateVariableDragElement (&spdb_proc, 5, _V(0, 7.5, -14)); // speedbrake drag
	CreateVariableDragElement (&gear_proc, 2, _V(0,-3,0));      // landing gear drag
	CreateVariableDragElement (&rdoor_drag, 7, _V(2.9,0,10));   // right cargo door drag
	CreateVariableDragElement (&ldoor_drag, 7, _V(-2.9,0,10));  // right cargo door drag

	SetADCtrlMode (7);

	// ************************* particle streams **********************************

	PARTICLESTREAMSPEC rps = {
		0, 20, 20, 0, 0.03, 0.5, 100, 3, PARTICLESTREAMSPEC::DIFFUSE,
		PARTICLESTREAMSPEC::LVL_FLAT, 1, 1, PARTICLESTREAMSPEC::ATM_PLIN, 6e7, 12e7
	};
	AddReentryStream (&rps);

	// ************************* propellant specs **********************************

	if (!ph_oms) ph_oms  = CreatePropellantResource (ORBITER_MAX_PROPELLANT_MASS); // OMS propellant
	SetDefaultPropellantResource (ph_oms); // display OMS tank level in generic HUD

	// *********************** thruster definitions ********************************

	// OMS (Orbital Manouevering System)
	th_main[0] = CreateThruster (_V(-2.7,3.7,-13.8), _V( 0.03,-0.25885,0.96545), ORBITER_OMS_THRUST, ph_oms, ORBITER_OMS_ISP0, ORBITER_OMS_ISP1);
	th_main[1] = CreateThruster (_V( 2.7,3.7,-13.8), _V(-0.03,-0.25885,0.96545), ORBITER_OMS_THRUST, ph_oms, ORBITER_OMS_ISP0, ORBITER_OMS_ISP1);
	thg_main = CreateThrusterGroup (th_main, 2, THGROUP_MAIN);
	for (i = 0; i < 2; i++) AddExhaust (th_main[i], 4.0, 0.5);

	// RCS (Reaction Control System)
	CreateAttControls_RCS ();

	// ************************ visual parameters **********************************

	AddOrbiterVisual (OFS_ZERO);

	status = 3;
}

// --------------------------------------------------------------
// Generic attitude controls
// --------------------------------------------------------------
void Atlantis::CreateAttControls (double th_pitch, double th_roll, double th_yaw, double isp0, double isp1)
{
	THRUSTER_HANDLE hAtt[2];
	hAtt[0] = CreateThruster (_V(0,0, 25), _V(0, 1,0), th_pitch, ph_tank, isp0, isp1);
	hAtt[1] = CreateThruster (_V(0,0,-25), _V(0,-1,0), th_pitch, ph_tank, isp0, isp1);
	CreateThrusterGroup (hAtt, 2, THGROUP_ATT_PITCHUP);
	hAtt[0] = CreateThruster (_V(0,0, 25), _V(0,-1,0), th_pitch, ph_tank, isp0, isp1);
	hAtt[1] = CreateThruster (_V(0,0,-25), _V(0, 1,0), th_pitch, ph_tank, isp0, isp1);
	CreateThrusterGroup (hAtt, 2, THGROUP_ATT_PITCHDOWN);
	hAtt[0] = CreateThruster (_V( 8,0, 0), _V(0, 1,0), th_roll,  ph_tank, isp0, isp1);
	hAtt[1] = CreateThruster (_V(-8,0, 0), _V(0,-1,0), th_roll,  ph_tank, isp0, isp1);
	CreateThrusterGroup (hAtt, 2, THGROUP_ATT_BANKLEFT);
	hAtt[0] = CreateThruster (_V( 8,0, 0), _V(0,-1,0), th_roll,  ph_tank, isp0, isp1);
	hAtt[1] = CreateThruster (_V(-8,0, 0), _V(0, 1,0), th_roll,  ph_tank, isp0, isp1);
	CreateThrusterGroup (hAtt, 2, THGROUP_ATT_BANKRIGHT);
	hAtt[0] = CreateThruster (_V(0,0, 10), _V( 1,0,0), th_yaw,   ph_tank, isp0, isp1);
	hAtt[1] = CreateThruster (_V(0,0,-10), _V(-1,0,0), th_yaw,   ph_tank, isp0, isp1);
	CreateThrusterGroup (hAtt, 2, THGROUP_ATT_YAWRIGHT);
	hAtt[0] = CreateThruster (_V(0,0, 10), _V(-1,0,0), th_yaw,   ph_tank, isp0, isp1);
	hAtt[1] = CreateThruster (_V(0,0,-10), _V( 1,0,0), th_yaw,   ph_tank, isp0, isp1);
	CreateThrusterGroup (hAtt, 2, THGROUP_ATT_YAWLEFT);
}

// --------------------------------------------------------------
// Attitude controls during launch phase
// --------------------------------------------------------------
void Atlantis::CreateAttControls_Launch()
{
	// we should only require pitch and bank, but yaw is also defined to make KILLROT work
	// no linear attitude modes are defined at this stage
	CreateAttControls (MAX_ATT_LAUNCH, MAX_ROLL_SRB, MAX_ATT_LAUNCH, ORBITER_MAIN_ISP0, ORBITER_MAIN_ISP1);
}

// --------------------------------------------------------------
// Attitude controls (RCS) during orbital phase
// --------------------------------------------------------------
void Atlantis::CreateAttControls_RCS()
{
	SURFHANDLE tex_rcs = oapiRegisterExhaustTexture ("Exhaust_atrcs");
	const double eh = 6.0;             // exhaust length scale
	const double ew1 = 0.4, ew2 = 0.8; // exhaust width scales

	// set of attitude thrusters (idealised). The arrangement is such that no angular
	// momentum is created in linear mode, and no linear momentum is created in rotational mode.
	THRUSTER_HANDLE th_att_rot[4], th_att_lin[4];
	th_att_rot[0] = th_att_lin[0] = CreateThruster (_V(0,0, 15.5), _V(0, 1,0), ORBITER_RCS_THRUST, ph_oms, ORBITER_RCS_ISP0, ORBITER_RCS_ISP1);
	th_att_rot[1] = th_att_lin[3] = CreateThruster (_V(0,0,-15.5), _V(0,-1,0), ORBITER_RCS_THRUST, ph_oms, ORBITER_RCS_ISP0, ORBITER_RCS_ISP1);
	th_att_rot[2] = th_att_lin[2] = CreateThruster (_V(0,0, 15.5), _V(0,-1,0), ORBITER_RCS_THRUST, ph_oms, ORBITER_RCS_ISP0, ORBITER_RCS_ISP1);
	th_att_rot[3] = th_att_lin[1] = CreateThruster (_V(0,0,-15.5), _V(0, 1,0), ORBITER_RCS_THRUST, ph_oms, ORBITER_RCS_ISP0, ORBITER_RCS_ISP1);
	CreateThrusterGroup (th_att_rot,   2, THGROUP_ATT_PITCHUP);
	CreateThrusterGroup (th_att_rot+2, 2, THGROUP_ATT_PITCHDOWN);
	CreateThrusterGroup (th_att_lin,   2, THGROUP_ATT_UP);
	CreateThrusterGroup (th_att_lin+2, 2, THGROUP_ATT_DOWN);

	AddExhaust (th_att_rot[0], eh, ew1, _V( 1.60,-0.20, 18.78), _V( 0.4339,-0.8830,-0.1793), tex_rcs);//F2D
	AddExhaust (th_att_rot[0], eh, ew1, _V( 1.68,-0.18, 18.40), _V( 0.4339,-0.8830,-0.1793), tex_rcs);//F4D
	AddExhaust (th_att_rot[0], eh, ew1, _V(-1.55,-0.20, 18.78), _V(-0.4339,-0.8830,-0.1793), tex_rcs);//F1D
	AddExhaust (th_att_rot[0], eh, ew1, _V(-1.63,-0.18, 18.40), _V(-0.4339,-0.8830,-0.1793), tex_rcs);//F3D

	AddExhaust (th_att_rot[1], eh, ew1, _V(-3.46, 3.20,-12.30), _V(0, 1,0), tex_rcs);//L4U
	AddExhaust (th_att_rot[1], eh, ew1, _V(-3.46, 3.20,-12.70), _V(0, 1,0), tex_rcs);//L2U
	AddExhaust (th_att_rot[1], eh, ew1, _V(-3.46, 3.20,-13.10), _V(0, 1,0), tex_rcs);//L1U

	AddExhaust (th_att_rot[1], eh, ew1, _V( 3.43, 3.20,-12.30), _V(0, 1,0), tex_rcs);//R4U
	AddExhaust (th_att_rot[1], eh, ew1, _V( 3.43, 3.20,-12.70), _V(0, 1,0), tex_rcs);//R2U
	AddExhaust (th_att_rot[1], eh, ew1, _V( 3.43, 3.20,-13.10), _V(0, 1,0), tex_rcs);//R1U

	AddExhaust (th_att_rot[2], eh, ew1, _V(-0.4 , 1.10, 18.3 ), _V(0, 1,0), tex_rcs);//F1U	
	AddExhaust (th_att_rot[2], eh, ew1, _V( 0.0 , 1.15 ,18.3 ), _V(0, 1,0), tex_rcs);//F3U
	AddExhaust (th_att_rot[2], eh, ew1, _V( 0.4 , 1.10, 18.3 ), _V(0, 1,0), tex_rcs);//F2U

	AddExhaust (th_att_rot[3], eh, ew1, _V(-3.1 , 1.55,-12.45), _V(-0.2844,-0.9481,-0.1422), tex_rcs);//L4D
	AddExhaust (th_att_rot[3], eh, ew1, _V(-3.1 , 1.6 ,-12.8 ), _V(-0.2844,-0.9481,-0.1422), tex_rcs);//L2D
	AddExhaust (th_att_rot[3], eh, ew1, _V(-3.1 , 1.65,-13.15), _V(-0.2844,-0.9481,-0.1422), tex_rcs);//L3D

	AddExhaust (th_att_rot[3], eh, ew1, _V( 3.15, 1.55,-12.45), _V( 0.2844,-0.9481,-0.1422), tex_rcs);//R4D
	AddExhaust (th_att_rot[3], eh, ew1, _V( 3.15, 1.6 ,-12.8 ), _V( 0.2844,-0.9481,-0.1422), tex_rcs);//R2D
	AddExhaust (th_att_rot[3], eh, ew1, _V( 3.15, 1.65,-13.15), _V( 0.2844,-0.9481,-0.1422), tex_rcs);//R3D

	th_att_rot[0] = th_att_lin[0] = CreateThruster (_V(0,0, 15.5), _V(-1,0,0), ORBITER_RCS_THRUST, ph_oms, ORBITER_RCS_ISP0, ORBITER_RCS_ISP1);
	th_att_rot[1] = th_att_lin[3] = CreateThruster (_V(0,0,-15.5), _V( 1,0,0), ORBITER_RCS_THRUST, ph_oms, ORBITER_RCS_ISP0, ORBITER_RCS_ISP1);
	th_att_rot[2] = th_att_lin[2] = CreateThruster (_V(0,0, 15.5), _V( 1,0,0), ORBITER_RCS_THRUST, ph_oms, ORBITER_RCS_ISP0, ORBITER_RCS_ISP1);
	th_att_rot[3] = th_att_lin[1] = CreateThruster (_V(0,0,-15.5), _V(-1,0,0), ORBITER_RCS_THRUST, ph_oms, ORBITER_RCS_ISP0, ORBITER_RCS_ISP1);
	CreateThrusterGroup (th_att_rot,   2, THGROUP_ATT_YAWLEFT);
	CreateThrusterGroup (th_att_rot+2, 2, THGROUP_ATT_YAWRIGHT);
	CreateThrusterGroup (th_att_lin,   2, THGROUP_ATT_LEFT);
	CreateThrusterGroup (th_att_lin+2, 2, THGROUP_ATT_RIGHT);

	AddExhaust (th_att_rot[0], eh, ew2, _V( 1.8 ,-0.3 , 18.0 ), _V( 1,0,0), tex_rcs);//F4R
	AddExhaust (th_att_rot[0], eh, ew2, _V( 1.75, 0.1 , 18.05), _V( 1,0,0), tex_rcs);//F2R
	AddExhaust (th_att_rot[2], eh, ew2, _V(-1.7 ,-0.3 , 18.0 ), _V(-1,0,0), tex_rcs);//F1L
	AddExhaust (th_att_rot[2], eh, ew2, _V(-1.65,-0.1 , 18.05), _V(-1,0,0), tex_rcs);//F3L

	AddExhaust (th_att_rot[1], eh, ew2, _V(-4.0 , 2.35,-12.35), _V(-1,0,0), tex_rcs);//L4L
	AddExhaust (th_att_rot[1], eh, ew2, _V(-4.0 , 2.35,-12.6 ), _V(-1,0,0), tex_rcs);//L2L
	AddExhaust (th_att_rot[1], eh, ew2, _V(-4.0 , 2.35,-13.0 ), _V(-1,0,0), tex_rcs);//L3L
	AddExhaust (th_att_rot[1], eh, ew2, _V(-4.0 , 2.35,-13.35), _V(-1,0,0), tex_rcs);//L1L

	AddExhaust (th_att_rot[3], eh, ew2, _V( 4.0 , 2.35,-12.35), _V( 1,0,0), tex_rcs);//R4R
	AddExhaust (th_att_rot[3], eh, ew2, _V( 4.0 , 2.35,-12.6 ), _V( 1,0,0), tex_rcs);//R2R
	AddExhaust (th_att_rot[3], eh, ew2, _V( 4.0 , 2.35,-13.0 ), _V( 1,0,0), tex_rcs);//R3R
	AddExhaust (th_att_rot[3], eh, ew2, _V( 4.0,  2.35,-13.35), _V( 1,0,0), tex_rcs);//R1R

	th_att_rot[0] = CreateThruster (_V( 2.7,0,0), _V(0, 1,0), ORBITER_RCS_THRUST, ph_oms, ORBITER_RCS_ISP0, ORBITER_RCS_ISP1);
	th_att_rot[1] = CreateThruster (_V(-2.7,0,0), _V(0,-1,0), ORBITER_RCS_THRUST, ph_oms, ORBITER_RCS_ISP0, ORBITER_RCS_ISP1);
	th_att_rot[2] = CreateThruster (_V(-2.7,0,0), _V(0, 1,0), ORBITER_RCS_THRUST, ph_oms, ORBITER_RCS_ISP0, ORBITER_RCS_ISP1);
	th_att_rot[3] = CreateThruster (_V( 2.7,0,0), _V(0,-1,0), ORBITER_RCS_THRUST, ph_oms, ORBITER_RCS_ISP0, ORBITER_RCS_ISP1);
	CreateThrusterGroup (th_att_rot,   2, THGROUP_ATT_BANKLEFT);
	CreateThrusterGroup (th_att_rot+2, 2, THGROUP_ATT_BANKRIGHT);

	AddExhaust (th_att_rot[0], eh, ew1, _V( 1.60,-0.20, 18.78), _V( 0.4339,-0.8830,-0.1793), tex_rcs);//F2D
	AddExhaust (th_att_rot[0], eh, ew1, _V( 1.68,-0.18, 18.40), _V( 0.4339,-0.8830,-0.1793), tex_rcs);//F4D
	AddExhaust (th_att_rot[2], eh, ew1, _V(-1.55,-0.20, 18.78), _V(-0.4339,-0.8830,-0.1793), tex_rcs);//F1D
	AddExhaust (th_att_rot[2], eh, ew1, _V(-1.63,-0.18, 18.40), _V(-0.4339,-0.8830,-0.1793), tex_rcs);//F3D

	AddExhaust (th_att_rot[1], eh, ew1, _V(-3.46, 3.20,-12.30), _V(0, 1,0), tex_rcs);//L4U
	AddExhaust (th_att_rot[1], eh, ew1, _V(-3.46, 3.20,-12.70), _V(0, 1,0), tex_rcs);//L2U
	AddExhaust (th_att_rot[1], eh, ew1, _V(-3.46, 3.20,-13.10), _V(0, 1,0), tex_rcs);//L1U

	AddExhaust (th_att_rot[3], eh, ew1, _V( 3.43, 3.20,-12.30), _V(0, 1,0), tex_rcs);//R4U
	AddExhaust (th_att_rot[3], eh, ew1, _V( 3.43, 3.20,-12.70), _V(0, 1,0), tex_rcs);//R2U
	AddExhaust (th_att_rot[3], eh, ew1, _V( 3.43, 3.20,-13.10), _V(0, 1,0), tex_rcs);//R1U

	AddExhaust (th_att_rot[2], eh, ew1, _V(-3.1 , 1.55,-12.45), _V(-0.2844,-0.9481,-0.1422), tex_rcs);//L4D
	AddExhaust (th_att_rot[2], eh, ew1, _V(-3.1 , 1.6 ,-12.8 ), _V(-0.2844,-0.9481,-0.1422), tex_rcs);//L2D
	AddExhaust (th_att_rot[2], eh, ew1, _V(-3.1 , 1.65,-13.15), _V(-0.2844,-0.9481,-0.1422), tex_rcs);//L3D

	AddExhaust (th_att_rot[0], eh, ew1, _V( 3.15, 1.55,-12.45), _V( 0.2844,-0.9481,-0.1422), tex_rcs);//R4D
	AddExhaust (th_att_rot[0], eh, ew1, _V( 3.15, 1.6 ,-12.8 ), _V( 0.2844,-0.9481,-0.1422), tex_rcs);//R2D
	AddExhaust (th_att_rot[0], eh, ew1, _V( 3.15, 1.65,-13.15), _V( 0.2844,-0.9481,-0.1422), tex_rcs);//R3D

	th_att_lin[0] = CreateThruster (_V(0,0,-16), _V(0,0, 1), ORBITER_RCS_THRUST, ph_oms, ORBITER_RCS_ISP0, ORBITER_RCS_ISP1);
	th_att_lin[1] = CreateThruster (_V(0,0, 16), _V(0,0,-1), ORBITER_RCS_THRUST, ph_oms, ORBITER_RCS_ISP0, ORBITER_RCS_ISP1);
	CreateThrusterGroup (th_att_lin,   1, THGROUP_ATT_FORWARD);
	CreateThrusterGroup (th_att_lin+1, 1, THGROUP_ATT_BACK);

	AddExhaust (th_att_lin[0], eh, ew1, _V(-3.59, 2.8 ,-13.6 ), _V(0,0,-1), tex_rcs);//L1A
	AddExhaust (th_att_lin[0], eh, ew1, _V(-3.27, 2.8 ,-13.6 ), _V(0,0,-1), tex_rcs);//L3A
	AddExhaust (th_att_lin[0], eh, ew1, _V( 3.64, 2.8 ,-13.6 ), _V(0,0,-1), tex_rcs);//R1A
	AddExhaust (th_att_lin[0], eh, ew1, _V( 3.27, 2.8 ,-13.6 ), _V(0,0,-1), tex_rcs);//R3A

	AddExhaust (th_att_lin[1], eh, ew1, _V( 0.0 , 0.75, 19.2 ), _V(0, 0.0499, 0.9988), tex_rcs);//F3F
	AddExhaust (th_att_lin[1], eh, ew1, _V(-0.4 , 0.7 , 19.2 ), _V(0, 0.0499, 0.9988), tex_rcs);//F1F
	AddExhaust (th_att_lin[1], eh, ew1, _V( 0.4 , 0.7 , 19.2 ), _V(0, 0.0499, 0.9988), tex_rcs);//F2F
}

// --------------------------------------------------------------
// Define animation sequences for moving parts
// --------------------------------------------------------------
void Atlantis::DefineAnimations (void)
{
	UINT midx = 1; // mesh index for all external animations
	UINT vidx = 2; // mesh index for all VC animations

	// ***** 1. Cargo door and radiator animations *****

	static UINT RCargoDoorGrp[4] = {GRP_cargodooroutR,GRP_cargodoorinR,GRP_radiatorFR,GRP_radiatorBR};
	static MGROUP_ROTATE RCargoDoor (midx, RCargoDoorGrp, 4,
		_V(2.88, 1.3, 0), _V(0,0,1), (float)(-175.5*RAD));
	static UINT LCargoDoorGrp[4] = {GRP_cargodooroutL,GRP_cargodoorinL,GRP_radiatorFL,GRP_radiatorBL};
	static MGROUP_ROTATE LCargoDoor (midx, LCargoDoorGrp, 4,
		_V(-2.88, 1.3, 0), _V(0,0,1), (float)(175.5*RAD));

	anim_door = CreateAnimation (0);
	AddAnimationComponent (anim_door, 0.0, 0.4632, &RCargoDoor);
	AddAnimationComponent (anim_door, 0.5368, 1.0, &LCargoDoor);

	static UINT RRadiatorGrp[1] = {GRP_radiatorFR};
	static MGROUP_ROTATE RRadiator (midx, RRadiatorGrp, 1,
		_V(2.88, 1.3, 0), _V(0,0,1), (float)(35.5*RAD));
	static UINT LRadiatorGrp[1] = {GRP_radiatorFL};
	static MGROUP_ROTATE LRadiator (midx, LRadiatorGrp, 1,
		_V(-2.88, 1.3, 0), _V(0,0,1), (float)(-35.5*RAD));

	anim_rad = CreateAnimation (0);
	AddAnimationComponent (anim_rad, 0, 1, &RRadiator);
	AddAnimationComponent (anim_rad, 0, 1, &LRadiator);

	// ***** 2. Landing gear animation *****

	static UINT LNosewheelDoorGrp[1] = {GRP_nosedoorL};
	static MGROUP_ROTATE LNosewheelDoor (midx, LNosewheelDoorGrp, 1,
		_V(-0.78, -2.15, 17), _V(0, 0.195, 0.981), (float)(-60.0*RAD));
	static UINT RNosewheelDoorGrp[1] = {GRP_nosedoorR};
	static MGROUP_ROTATE RNosewheelDoor (midx, RNosewheelDoorGrp, 1,
		_V(0.78, -2.15, 17), _V(0, 0.195, 0.981), (float)(60.0*RAD));
	static UINT NosewheelGrp[2] = {GRP_nosewheel,GRP_nosegear};
	static MGROUP_ROTATE Nosewheel (midx, NosewheelGrp, 2,
		_V(0.0, -2.01, 18.11), _V(1, 0, 0), (float)(94.5*RAD));
	static UINT RGearDoorGrp[1] = {GRP_geardoorR};
	static MGROUP_ROTATE RGearDoor (midx, RGearDoorGrp, 1,
		_V(4.35, -2.64, -1.69), _V(0, 0.02, 0.9), (float)(96.2*RAD));
	static UINT LGearDoorGrp[1] = {GRP_geardoorL};
	static MGROUP_ROTATE LGearDoor (midx, LGearDoorGrp, 1,
		_V(-4.35, -2.64, -1.69), _V(0, 0.02, 0.9), (float)(-96.2*RAD));
	static UINT MainGearGrp[4] = {GRP_wheelR,GRP_gearR,GRP_wheelL,GRP_gearL};
	static MGROUP_ROTATE MainGear (midx, MainGearGrp, 4,
		_V(0, -2.66, -3.68), _V(1, 0, 0), (float)(94.5*RAD));

	anim_gear = CreateAnimation (0);
	AddAnimationComponent (anim_gear, 0,   0.5, &LNosewheelDoor);
	AddAnimationComponent (anim_gear, 0,   0.5, &RNosewheelDoor);
	AddAnimationComponent (anim_gear, 0.4, 1.0, &Nosewheel);
	AddAnimationComponent (anim_gear, 0,   0.5, &RGearDoor);
	AddAnimationComponent (anim_gear, 0,   0.5, &LGearDoor);
	AddAnimationComponent (anim_gear, 0.4, 1.0, &MainGear);

	// ***** 3. Ku-band antenna animation *****

	static UINT KuBand1Grp[3] = {GRP_startrackers,GRP_KUband1,GRP_KUband2};
	static MGROUP_ROTATE KuBand1 (midx, KuBand1Grp, 3,
		_V(2.85, 0.85, 0), _V(0,0,1), (float)(-18*RAD));
	static UINT KuBand2Grp[1] = {GRP_KUband2};
	static MGROUP_ROTATE KuBand2 (midx, KuBand2Grp, 1,
		_V(2.78, 1.7, 0), _V(0,0,1), (float)(-90*RAD));
	static UINT KuBand3Grp[2] = {GRP_KUband1,GRP_KUband2};
	static MGROUP_ROTATE KuBand3 (midx, KuBand3Grp, 2,
		_V(2.75, 2.05, 11.47), _V(0,1,0), (float)(-113*RAD));

	anim_kubd = CreateAnimation (0);
	AddAnimationComponent (anim_kubd, 0,     0.333, &KuBand1);
	AddAnimationComponent (anim_kubd, 0.333, 0.667, &KuBand2);
	AddAnimationComponent (anim_kubd, 0.667, 0.999, &KuBand3);

	// ***** 4. Elevator animation of elevons *****

	static UINT ElevGrp[4] = {GRP_flapR,GRP_flapL,GRP_aileronL,GRP_aileronR};
	static MGROUP_ROTATE Elevator (midx, ElevGrp, 4,
		_V(0,-2.173,-8.84), _V(1,0,0), (float)(30.0*RAD));
	anim_elev = CreateAnimation (0.5);
	AddAnimationComponent (anim_elev, 0, 1, &Elevator);

	// ***** 5. Aileron animation of elevons *****

	static UINT LAileronGrp[2] = {GRP_flapL,GRP_aileronL};
	static MGROUP_ROTATE LAileron (midx, LAileronGrp, 2,
		_V(0,-2.173,-8.84), _V(-1,0,0), (float)(10.0*RAD));
	static UINT RAileronGrp[2] = {GRP_flapR,GRP_aileronR};
	static MGROUP_ROTATE RAileron (midx, RAileronGrp, 2,
		_V(0,-2.173,-8.84), _V(1,0,0), (float)(10.0*RAD));
	anim_laileron = CreateAnimation (0.5);
	AddAnimationComponent (anim_laileron, 0, 1, &LAileron);
	anim_raileron = CreateAnimation (0.5);
	AddAnimationComponent (anim_raileron, 0, 1, &RAileron);

	// ***** 6. Rudder animation *****

	static UINT RudderGrp[2] = {GRP_rudderR,GRP_rudderL};
	static MGROUP_ROTATE Rudder (midx, RudderGrp, 2,
		_V(0,5.77,-12.17), _V(-0.037,0.833,-0.552), (float)(-54.2*RAD));
	anim_rudder = CreateAnimation (0.5);
	AddAnimationComponent (anim_rudder, 0, 1, &Rudder);

	// ***** 7. Speedbrake animation *****

	static UINT SB1Grp[1] = {GRP_rudderR};
	static MGROUP_ROTATE SB1 (midx, SB1Grp, 1,
		_V(0.32,5.77,-12.17), _V(-0.037,0.833,-0.552), (float)(-49.3*RAD));
	static UINT SB2Grp[1] = {GRP_rudderL};
	static MGROUP_ROTATE SB2 (midx, SB2Grp, 1,
		_V(-0.32,5.77,-12.17), _V(0.037,0.833,-0.552), (float)(49.3*RAD));

	anim_spdb = CreateAnimation (0);
	AddAnimationComponent (anim_spdb, 0, 1, &SB1);
	AddAnimationComponent (anim_spdb, 0, 1, &SB2);

	// ***** 8. RMS arm animation *****
	// Note that the animation components can't be declared static here, since
	// their rotation parameters are modified by the respective parent transforms

	ANIMATIONCOMPONENT_HANDLE parent;

	static UINT RMSShoulderYawGrp[1] = {GRP_Shoulder};
	rms_anim[0] = new MGROUP_ROTATE (midx, RMSShoulderYawGrp, 1,
		_V(-2.26, 1.5, 10), _V(0, 1, 0), (float)(-360*RAD)); // -180 .. +180
	anim_arm_sy = CreateAnimation (0.5);
	parent = AddAnimationComponent (anim_arm_sy, 0, 1, rms_anim[0]);

	static UINT RMSShoulderPitchGrp[1] = {GRP_Humerus};
	rms_anim[1] = new MGROUP_ROTATE (midx, RMSShoulderPitchGrp, 1,
		_V(-2.26, 1.8, 10), _V(1, 0, 0), (float)(147*RAD)); // -2 .. +145
	anim_arm_sp = CreateAnimation (0.0136);
	parent = AddAnimationComponent (anim_arm_sp, 0, 1, rms_anim[1], parent);

	static UINT RMSElbowPitchGrp[3] = {GRP_radii,GRP_RMScamera,GRP_RMScamera_pivot};
	rms_anim[2] = new MGROUP_ROTATE (midx, RMSElbowPitchGrp, 3,
		_V(-2.26,1.7,3.3), _V(1,0,0), (float)(-162*RAD)); // -160 .. +2
	anim_arm_ep = CreateAnimation (0.0123);
	parent = AddAnimationComponent (anim_arm_ep, 0, 1, rms_anim[2], parent);

	static UINT RMSWristPitchGrp[1] = {GRP_wrist};
	rms_anim[3] = new MGROUP_ROTATE (midx, RMSWristPitchGrp, 1,
		_V(-2.26,1.7,-3.55), _V(1,0,0), (float)(240*RAD)); // -120 .. +120
	anim_arm_wp = CreateAnimation (0.5);
	parent = AddAnimationComponent (anim_arm_wp, 0, 1, rms_anim[3], parent);

	static UINT RMSWristYawGrp[1] = {GRP_endeffecter};
	rms_anim[4] = new MGROUP_ROTATE (midx, RMSWristYawGrp, 1,
		_V(-2.26,1.7,-4.9), _V(0,1,0), (float)(-240*RAD)); // -120 .. +120
	anim_arm_wy = CreateAnimation (0.5);
	parent = AddAnimationComponent (anim_arm_wy, 0, 1, rms_anim[4], parent);

	rms_anim[5] = new MGROUP_ROTATE (LOCALVERTEXLIST, MAKEGROUPARRAY(arm_tip), 3,
		_V(-2.26,1.7,-6.5), _V(0,0,1), (float)(894*RAD)); // -447 .. +447
	anim_arm_wr = CreateAnimation (0.5);
	hAC_arm = AddAnimationComponent (anim_arm_wr, 0, 1, rms_anim[5], parent);

	// ======================================================
	// VC animation definitions
	// ======================================================
	plop->DefineAnimations (vidx);
}

void Atlantis::AddOrbiterVisual (const VECTOR3 &ofs)
{
	orbiter_ofs = ofs;
	huds.hudcnt = _V(ofs.x-0.671257, ofs.y+2.523535, ofs.z+14.969);

	if (mesh_orbiter == MESH_UNDEFINED) {

		// ***** Load meshes

		mesh_cockpit = AddMesh (hOrbiterCockpitMesh, &ofs);
		SetMeshVisibilityMode (mesh_cockpit, MESHVIS_EXTERNAL);

		mesh_orbiter = AddMesh (hOrbiterMesh, &ofs);
		SetMeshVisibilityMode (mesh_orbiter, MESHVIS_EXTERNAL|MESHVIS_VC|MESHVIS_EXTPASS);

		mesh_vc = AddMesh (hOrbiterVCMesh, &ofs);
		SetMeshVisibilityMode (mesh_vc, MESHVIS_VC);

		for (int i = 0; i < 10; i++) mfds[i].nmesh = mesh_vc;
		huds.nmesh = mesh_vc;

		if (do_cargostatic) {
			VECTOR3 cofs = ofs + cargo_static_ofs;
			AddMesh (cargo_static_mesh_name, &cofs);
		}
		if (do_plat) {
			VECTOR3 plat_ofs = _V(-2.59805, 1.69209, -5.15524);
			AddMesh("shuttle_eva_plat", &plat_ofs);
		}

		// ***** Docking definitions

		SetDockParams (ofs+ORBITER_DOCKPOS, _V(0,1,0), _V(0,0,-1));

		// ***** Attachment definitions

		if (!sat_attach) sat_attach = CreateAttachment (false, ofs+ofs_sts_sat, _V(0,1,0), _V(0,0,1), "X");
		if (!rms_attach) rms_attach = CreateAttachment (false, ofs+arm_tip[0], arm_tip[1]-arm_tip[0], arm_tip[2]-arm_tip[0], "G", true);

		// ***** Cockpit camera definition

		SetCameraOffset (_V(ofs.x-0.67,ofs.y+2.55,ofs.z+14.4));
		oapiVCRegisterHUD (&huds); // register changes in HUD parameters
	}
}

void Atlantis::AddTankVisual (const VECTOR3 &ofs)
{
	if (mesh_tank == MESH_UNDEFINED) {

		// ***** Load mesh
		mesh_tank = AddMesh (hTankMesh, &ofs);
		SetMeshVisibilityMode (mesh_tank, MESHVIS_ALWAYS|MESHVIS_EXTPASS);
	}
}

void Atlantis::AddSRBVisual (int which, const VECTOR3 &ofs)
{
	if (mesh_srb[which] == MESH_UNDEFINED) {

		// ***** Load mesh
		mesh_srb[which] = AddMesh (hSRBMesh, &ofs);
		SetMeshVisibilityMode (mesh_srb[which], MESHVIS_ALWAYS|MESHVIS_EXTPASS);
		int id = AddExhaustRef (EXHAUST_CUSTOM, _V(ofs.x, ofs.y, ofs.z-21.8), 0, 0);
		if (which) srb_id1 = id;
		else       srb_id2 = id;
	}
}

void Atlantis::SeparateBoosters (double met)
{
	// Create SRB's as individual objects
	VESSELSTATUS2 vs;
	VESSELSTATUS2::FUELSPEC fuel;
	VESSELSTATUS2::THRUSTSPEC thrust;
	memset (&vs, 0, sizeof(vs));
	vs.version = 2;
	GetStatusEx (&vs);
	vs.flag = VS_FUELLIST | VS_THRUSTLIST;
	vs.fuel = &fuel;
	vs.nfuel = 1;
	vs.fuel->idx = 0;
	vs.thruster = &thrust;
	vs.nthruster = 1;
	vs.thruster->idx = 0;
	GetSRB_State (met, vs.thruster->level, vs.fuel->level);
	Local2Rel (OFS_LAUNCH_RIGHTSRB, vs.rpos);
	vs.arot.z += 0.25*PI;
	vs.status = 0;
	char name[256];
	strcpy (name, GetName()); strcat (name, "-SRB1");
	oapiCreateVesselEx (name, "Atlantis_SRB", &vs);
	Local2Rel (OFS_LAUNCH_LEFTSRB, vs.rpos);
	vs.arot.z -= 1.5*PI;
	name[strlen(name)-1] = '2';
	oapiCreateVesselEx (name, "Atlantis_SRB", &vs);

	// Remove SRB's from Shuttle instance
	DelPropellantResource (ph_srb);
	DelThrusterGroup (thg_srb, THGROUP_USER, true);

	// remove srb meshes and shift cg
	DelMesh(mesh_srb[1]);
	DelMesh(mesh_srb[0]);
	ShiftCG (OFS_LAUNCH_ORBITER-OFS_WITHTANK_ORBITER);

	// reconfigure
	RecordEvent ("JET", "SRB");
	SetOrbiterTankConfiguration();
}

void Atlantis::SeparateTank (void)
{
	// Create Tank as individual object
	VESSELSTATUS2 vs;
	memset (&vs, 0, sizeof (vs));
	vs.version = 2;
	GetStatusEx (&vs);
	vs.flag = VS_FUELRESET | VS_THRUSTRESET;
	VECTOR3 ofs = OFS_WITHTANK_TANK;
	if (Playback()) // necessary because during playback the CG shift occurs before separation
		ofs -= OFS_WITHTANK_ORBITER;
	VECTOR3 rofs;
	VECTOR3 vel = {0,-1,0};
	static const double dv = 0.3; // tank separation velocity
	Local2Rel (ofs, vs.rpos);
	GlobalRot (vel, rofs);
	vs.rvel += rofs*dv;
	vs.vrot.x = -0.005;
	vs.status = 0;
	char name[256];
	strcpy (name, GetName()); strcat (name, "-Tank");
	oapiCreateVesselEx (name, "Atlantis_Tank", &vs);

	// Remove Tank from shuttle instance
	DelPropellantResource (ph_tank);

	// main engines are done
	DelThrusterGroup (thg_main, THGROUP_MAIN, true);

	// clear launch attitude control system
	DelThrusterGroup (THGROUP_ATT_PITCHUP, true);
	DelThrusterGroup (THGROUP_ATT_PITCHDOWN, true);
	DelThrusterGroup (THGROUP_ATT_BANKLEFT, true);
	DelThrusterGroup (THGROUP_ATT_BANKRIGHT, true);
	DelThrusterGroup (THGROUP_ATT_YAWLEFT, true);
	DelThrusterGroup (THGROUP_ATT_YAWRIGHT, true);

	// remove tank mesh and shift cg
	DelMesh (mesh_tank);
	ShiftCG (OFS_WITHTANK_ORBITER);

	// reconfigure
	RecordEvent ("JET", "ET");
	SetOrbiterConfiguration ();
}

void Atlantis::ToggleGrapple (void)
{
	HWND hDlg;
	OBJHANDLE hV = GetAttachmentStatus (rms_attach);

	if (hV) {  // release satellite

		ATTACHMENTHANDLE hAtt = CanArrest();
		DetachChild (rms_attach);
		if (hDlg = oapiFindDialog (g_Param.hDLL, IDD_RMS)) {
			SetWindowText (GetDlgItem (hDlg, IDC_GRAPPLE), "Grapple");
			EnableWindow (GetDlgItem (hDlg, IDC_STOW), TRUE);
		}
		// check whether the object being ungrappled is ready to be clamped into the payload bay
		if (hAtt) {
			AttachChild (hV, sat_attach, hAtt);
			if (hDlg) {
				SetWindowText (GetDlgItem (hDlg, IDC_PAYLOAD), "Purge");
				EnableWindow (GetDlgItem (hDlg, IDC_PAYLOAD), TRUE);
			}
		}

#ifdef UNDEF
		VECTOR3 pos, dir, rot, gbay, gpos;
		GetAttachmentParams (sat_attach, pos, dir, rot);
		Local2Global (pos, gbay);
		VESSEL *v = oapiGetVesselInterface (hV);
		DWORD nAttach = v->AttachmentCount (true);
		for (DWORD j = 0; j < nAttach; j++) { // now scan all attachment points
			ATTACHMENTHANDLE hAtt = v->GetAttachmentHandle (true, j);
			v->GetAttachmentParams (hAtt, pos, dir, rot);
			v->Local2Global (pos, gpos);
			if (dist (gpos, gbay) < MAX_GRAPPLING_DIST) {
				AttachChild (hV, sat_attach, hAtt);
				return;
			}
		}
#endif

	} else {             // grapple satellite

		VECTOR3 gpos, grms, pos, dir, rot;
		Local2Global (orbiter_ofs+arm_tip[0], grms);  // global position of RMS tip
		
		// Search the complete vessel list for a grappling candidate.
		// Not very scalable ...
		for (DWORD i = 0; i < oapiGetVesselCount(); i++) {
			OBJHANDLE hV = oapiGetVesselByIndex (i);
			if (hV == GetHandle()) continue; // we don't want to grapple ourselves ...
			oapiGetGlobalPos (hV, &gpos);
			if (dist (gpos, grms) < oapiGetSize (hV)) { // in range
				VESSEL *v = oapiGetVesselInterface (hV);
				DWORD nAttach = v->AttachmentCount (true);
				for (DWORD j = 0; j < nAttach; j++) { // now scan all attachment points of the candidate
					ATTACHMENTHANDLE hAtt = v->GetAttachmentHandle (true, j);
					const char *id = v->GetAttachmentId (hAtt);
					if (strncmp (id, "GS", 2)) continue; // attachment point not compatible
					v->GetAttachmentParams (hAtt, pos, dir, rot);
					v->Local2Global (pos, gpos);
					if (dist (gpos, grms) < MAX_GRAPPLING_DIST) { // found one!
						// check whether satellite is currently clamped into payload bay
						if (hV == GetAttachmentStatus (sat_attach))
							DetachChild (sat_attach);
						AttachChild (hV, rms_attach, hAtt);
						if (hDlg = oapiFindDialog (g_Param.hDLL, IDD_RMS)) {
							SetWindowText (GetDlgItem (hDlg, IDC_GRAPPLE), "Release");
							EnableWindow (GetDlgItem (hDlg, IDC_STOW), FALSE);
						}
						return;
					}
				}
			}
		}

	}
}

void Atlantis::ToggleArrest (void)
{
	HWND hDlg;
	if (SatStowed()) { // purge satellite
		DetachChild (sat_attach, 0.1);
		if (hDlg = oapiFindDialog (g_Param.hDLL, IDD_RMS)) {
			SetWindowText (GetDlgItem (hDlg, IDC_PAYLOAD), "Arrest");
			EnableWindow (GetDlgItem (hDlg, IDC_PAYLOAD), CanArrest() ? TRUE:FALSE);
		}
	} else if (CanArrest()) {           // try to arrest satellite
		ToggleGrapple();
	}
}

// check whether the currently grappled object can be stowed in the cargo bay
ATTACHMENTHANDLE Atlantis::CanArrest (void) const
{
	OBJHANDLE hV = GetAttachmentStatus (rms_attach);
	if (!hV) return 0;
	VESSEL *v = oapiGetVesselInterface (hV);
	DWORD nAttach = v->AttachmentCount (true);
	VECTOR3 pos, dir, rot, gpos, gbay;
	GetAttachmentParams (sat_attach, pos, dir, rot);
	Local2Global (pos, gbay);
	for (DWORD j = 0; j < nAttach; j++) {
		ATTACHMENTHANDLE hAtt = v->GetAttachmentHandle (true, j);
		if (strncmp (v->GetAttachmentId (hAtt), "XS", 2)) continue; // attachment point not compatible
		v->GetAttachmentParams (hAtt, pos, dir, rot);
		v->Local2Global (pos, gpos);
		if (dist (gpos, gbay) < MAX_GRAPPLING_DIST) {
			return hAtt;
		}
	}
	return 0;
}

void Atlantis::SeparateMMU (void)
{
	// Create MMU at docking port
	DOCKHANDLE hDock = GetDockHandle (0);
	if (GetDockStatus(hDock)) return; // something is already attached to this docking port

	int i;
	char name[256];
	OBJHANDLE hVessel;
	for (i = 0; ; i++) {
		sprintf (name, "%s-MMU-%d", GetName(), i+1);
		hVessel = oapiGetVesselByName(name);
		if (!hVessel) break;
	}

	VESSELSTATUS vs;
	GetStatus (vs);
	hMMU = oapiCreateVessel (name, "Nasa_MMU", vs);
	Dock (hMMU, 0, 0, 1);
	oapiSetFocusObject (hMMU);
}

void Atlantis::AutoMainGimbal ()
{
	// Implement automatic gimbal adjustment for main engines to correct for
	// SRB thrust variations. We adjust only the upper engine

	double        F_srb  = SRB_THRUST         *GetThrusterLevel (th_srb[0]);
	double        F_main = ORBITER_MAIN_THRUST*GetThrusterLevel (th_main[0]);
	if (F_main) {
		double        M_srb =   2.51951112*F_srb;  // angular moment from SRB thrust
		double        M_m2  = -12.02495   *F_main; // angular moment from the two lower main engines in neutral position
		double        M_0   = M_srb + M_m2;
		static double ry    =   9.42;  // upper main thruster: y-offset
		static double rz    = -23.295; // upper main thruster: z-offset
		static double ry2   = ry*ry;
		static double rz2   = rz*rz;
		double        F2    = F_main*F_main;
		double        M2    = M_0*M_0;
		double        term  = -M2 + F2*(ry2+rz2);
		double        arg1  = max (0.0, rz2 * term);
		double        scale = 1.0/(F_main*(ry2+rz2));
		double        dz    = (M_0*ry + sqrt (arg1))*scale;
		double		  arg2  = max (0.0, ry2 * term);
		double        dy    =-(M_0*rz + sqrt (arg2))*scale;
		if (!arg1 || !arg2) {
			double len = _hypot(dy,dz);
			dy /= len, dz /= len;
		}
		SetThrusterDir (th_main[2], _V(0,dy,dz));
	}
}

void Atlantis::LaunchClamps ()
{
	VECTOR3 F, T, r = _V(0,0,0), Fc = _V(0,0,0), Tc = _V(0,0,0);
	GetThrusterMoment (th_srb[0], F, T);
	Fc.z = -2*F.z;
	Tc.x =  2*T.x;
	GetThrusterMoment (th_main[0], F, T);
	Fc.z -= 2*F.z;
	Fc.y -= 2*F.y;
	Tc.x += 2*T.x;
	GetThrusterMoment (th_main[2], F, T);
	Fc.z -= F.z;
	Fc.y -= F.y;
	Tc.x += T.x;
	r.z = (Fc.y ? Tc.x/Fc.y : 0);
	AddForce (Fc, r);
}

void Atlantis::SetGearParameters (double state)
{
	if (state == 1.0) { // gear fully deployed
		SetTouchdownPoints (_V(0,-4,17), _V(-3.96,-5.6,-4.3), _V(3.96,-5.6,-4.3)); // gear wheel tips
		SetSurfaceFrictionCoeff (0.05, 0.4);
	} else {
		SetTouchdownPoints (_V(0,-2.5,14), _V(-8,-2.8,-9), _V(8,-2.8,-9)); // belly landing
		SetSurfaceFrictionCoeff (0.4, 0.4);
	}
}

void Atlantis::Jettison ()
{
	switch (status) {
	case 0:
	case 3:               // nothing to do
		break;
	case 1:               // abandon boosters
		SeparateBoosters (oapiGetSimTime()-t0); 
		break;
	case 2:               // abandon tank
		SeparateTank();
		break;
	}
}

// Update moving parts of the orbiter's visual: payload bay doors and gear
// This should only be called when the visual exists, e.g. from within
// clbkVisualCreated or clbkAnimate

void Atlantis::UpdateMesh ()
{
	// update animation states
	SetAnimation (anim_gear, gear_proc);
	SetAnimation (anim_spdb, spdb_proc);
	SetAnimation (anim_door, plop->BayDoorStatus.pos);
	SetAnimation (anim_rad,  plop->RadiatorStatus.pos);
	SetAnimation (anim_kubd, plop->KuAntennaStatus.pos);

	SetAnimationArm (anim_arm_sy, arm_sy);
	SetAnimationArm (anim_arm_sp, arm_sp);
	SetAnimationArm (anim_arm_ep, arm_ep);
	SetAnimationArm (anim_arm_wp, arm_wp);
	SetAnimationArm (anim_arm_wy, arm_wy);
	SetAnimationArm (anim_arm_wr, arm_wr);

	// update MFD brightness
	if (vis) {
		int i;
		MESHHANDLE hMesh = GetMesh (vis, mesh_vc);
		for (i = 0; i < 10; i++) {
			MATERIAL *mat = oapiMeshMaterial (hMesh, 10+i);
			mat->emissive.r = mat->emissive.g = mat->emissive.b = (float)mfdbright[i];
		}
	}
}

void Atlantis::ClearMeshes ()
{
	VESSEL::ClearMeshes();
	mesh_orbiter = MESH_UNDEFINED;
	mesh_cockpit = MESH_UNDEFINED;
	mesh_vc      = MESH_UNDEFINED;
	mesh_tank    = MESH_UNDEFINED;
	mesh_srb[0] = mesh_srb[1] = MESH_UNDEFINED;
}

void Atlantis::SetBayDoorPosition (double pos)
{
	SetAnimation (anim_door, pos);
	rdoor_drag = sqrt (min (1.0, pos*3.0));
	ldoor_drag = sqrt (min (1.0, max(0.0, pos-0.3656)*3.0));
}

void Atlantis::SetRadiatorPosition (double pos)
{
	SetAnimation (anim_rad, pos);
}

void Atlantis::SetKuAntennaPosition (double pos)
{
	SetAnimation (anim_kubd, pos);
}

void Atlantis::OperateLandingGear (AnimState::Action action)
{
	if (status < 3) return;
	// operate landing gear only once the orbiter is free from the tank

	gear_status = action;
	RecordEvent ("GEAR", action == AnimState::CLOSING ? "UP" : "DOWN");
}

void Atlantis::RevertLandingGear ()
{
	if (status < 3) return;
	// operate landing gear only once the orbiter is free from the tank

	OperateLandingGear (gear_status == AnimState::CLOSED || gear_status == AnimState::CLOSING ?
		AnimState::OPENING : AnimState::CLOSING);
}

void Atlantis::OperateSpeedbrake (AnimState::Action action)
{
	spdb_status = action;
	RecordEvent ("SPEEDBRAKE", action == AnimState::CLOSING ? "CLOSE" : "OPEN");
}

void Atlantis::RevertSpeedbrake (void)
{
	OperateSpeedbrake (spdb_status == AnimState::CLOSED || spdb_status == AnimState::CLOSING ?
		AnimState::OPENING : AnimState::CLOSING);
}

void Atlantis::SetAnimationArm (UINT anim, double state)
{
	SetAnimation (anim, state);
	arm_moved = true;

	HWND hDlg;
	if (!SatStowed() && (hDlg = oapiFindDialog (g_Param.hDLL, IDD_RMS))) {
		SetWindowText (GetDlgItem (hDlg, IDC_PAYLOAD), "Arrest");
		EnableWindow (GetDlgItem (hDlg, IDC_PAYLOAD), CanArrest() ? TRUE : FALSE);
	}
}

void Atlantis::RedrawPanel_MFDButton (SURFHANDLE surf, int mfd)
{
	HDC hDC = oapiGetDC (surf);

	// D. Beachy: BUGFIX: if MFD powered off, cover separator lines and do not paint buttons
    if (oapiGetMFDMode(mfd) == MFD_NONE) {
        RECT r = { 0,0,255,13 };
        FillRect(hDC, &r, (HBRUSH)GetStockObject(BLACK_BRUSH));
    } else {   // MFD powered on
		HFONT pFont = (HFONT)SelectObject (hDC, g_Param.font[0]);
		SetTextColor (hDC, RGB(0,255,216));
		SetTextAlign (hDC, TA_CENTER);
		SetBkMode (hDC, TRANSPARENT);
		const char *label;
		int x = 24;

		for (int bt = 0; bt < 5; bt++) {
			if (label = oapiMFDButtonLabel (mfd, bt)) {
				TextOut (hDC, x, 1, label, strlen(label));
				x += 42;
			} else break;
		}
		TextOut (hDC, 234, 1, "PG", 2);
		SelectObject (hDC, pFont);
	}

	oapiReleaseDC (surf, hDC);
}

// ==============================================================
// Overloaded callback functions
// ==============================================================

// --------------------------------------------------------------
// Set vessel class capabilities from config file
// --------------------------------------------------------------
void Atlantis::clbkSetClassCaps (FILEHANDLE cfg)
{
	if (!oapiReadItem_bool (cfg, "RenderCockpit", render_cockpit))
		render_cockpit = false;
}

// --------------------------------------------------------------
// Set status from a VESSELSTATUS2 structure
// --------------------------------------------------------------
void Atlantis::clbkSetStateEx (const void *status)
{
	// default parameter initialisation
	DefSetStateEx (status);

	// reset vessel-specific parameters to defaults
	//status = 3;
	SetOrbiterConfiguration();
}

// --------------------------------------------------------------
// Read status from scenario file
// --------------------------------------------------------------
void Atlantis::clbkLoadStateEx (FILEHANDLE scn, void *vs)
{
	int action;
    char *line;
	double met = 0.0; // mission elapsed time
	double srbtime = 0.0;
	double sts_sat_x = 0.0;
	double sts_sat_y = 0.0;
	double sts_sat_z = 0.0;
	spdb_status = AnimState::CLOSED; spdb_proc = 0.0;

	while (oapiReadScenario_nextline (scn, line)) {
        if (!_strnicmp (line, "CONFIGURATION", 13)) {
            sscanf (line+13, "%d", &status);
		} else if (!_strnicmp (line, "MET", 3)) {
			sscanf (line+3, "%lf", &met);
		} else if (!_strnicmp (line, "GEAR", 4)) {
			sscanf (line+4, "%d%lf", &action, &gear_proc);
			gear_status = (AnimState::Action)(action+1);
		} else if (!_strnicmp (line, "SPEEDBRAKE", 10)) {
			sscanf (line+10, "%d%lf", &action, &spdb_proc);
			spdb_status = (AnimState::Action)(action+1);
		} else if (!_strnicmp (line, "SRB_IGNITION_TIME", 17)) {
			sscanf (line+17, "%lf", &srbtime);
		} else if (!_strnicmp (line, "SAT_OFS_X", 9)) {
			sscanf (line+9, "%lf", &sts_sat_x);
		} else if (!_strnicmp (line, "SAT_OFS_Y", 9)) {
			sscanf (line+9, "%lf", &sts_sat_y);
		} else if (!_strnicmp (line, "SAT_OFS_Z", 9)) {
			sscanf (line+9, "%lf", &sts_sat_z);
		} else if (!_strnicmp (line, "CARGO_STATIC_MESH", 17)) {
			sscanf (line+17, "%s", cargo_static_mesh_name);
			do_cargostatic = true;
		} else if (!_strnicmp (line, "CARGO_STATIC_OFS", 16)) {
			sscanf (line+16, "%lf%lf%lf", &cargo_static_ofs.x, &cargo_static_ofs.y, &cargo_static_ofs.z);
		} else if (!_strnicmp (line, "ARM_STATUS", 10)) {
			sscanf (line+10, "%lf%lf%lf%lf%lf%lf", &arm_sy, &arm_sp, &arm_ep, &arm_wp, &arm_wy, &arm_wr);
        } else {
			if (plop->ParseScenarioLine (line)) continue; // offer the line to bay door operations
            ParseScenarioLineEx (line, vs);
			// unrecognised option - pass to Orbiter's generic parser
        }
    }
	ofs_sts_sat.x=sts_sat_x;
	ofs_sts_sat.y=sts_sat_y;
	ofs_sts_sat.z=sts_sat_z;

	ClearMeshes();
	switch (status) {
	case 0:
		SetLaunchConfiguration();
		break;
	case 1:
		SetPostLaunchConfiguration (met);
		break;
	case 2:
		SetOrbiterTankConfiguration();
		break;
	case 3:
		SetOrbiterConfiguration();
		break;
	}

	UpdateMesh ();
}

// --------------------------------------------------------------
// Write status to scenario file
// --------------------------------------------------------------
void Atlantis::clbkSaveState (FILEHANDLE scn)
{
	char cbuf[256];

	// default vessel parameters
	VESSEL3::clbkSaveState (scn);

	// custom parameters
	oapiWriteScenario_int (scn, "CONFIGURATION", status);

	if (status == 1)
		oapiWriteScenario_float (scn, "MET", oapiGetSimTime()-t0);

	sprintf (cbuf, "%d %0.4f", gear_status-1, gear_proc);
	oapiWriteScenario_string (scn, "GEAR", cbuf);

	if (spdb_status != AnimState::CLOSED) {
		sprintf (cbuf, "%d %0.4f", spdb_status-1, spdb_proc);
		oapiWriteScenario_string (scn, "SPEEDBRAKE", cbuf);
	}

	sprintf (cbuf, "%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f", arm_sy, arm_sp, arm_ep, arm_wp, arm_wy, arm_wr);
	oapiWriteScenario_string (scn, "ARM_STATUS", cbuf);

	oapiWriteScenario_float (scn, "SAT_OFS_X", ofs_sts_sat.x);
	oapiWriteScenario_float (scn, "SAT_OFS_Y", ofs_sts_sat.y);
	oapiWriteScenario_float (scn, "SAT_OFS_Z", ofs_sts_sat.z);

	if (do_cargostatic) {
		oapiWriteScenario_string (scn, "CARGO_STATIC_MESH", cargo_static_mesh_name);
		if (cargo_static_ofs.x || cargo_static_ofs.y || cargo_static_ofs.z)
			oapiWriteScenario_vec (scn, "CARGO_STATIC_OFS", cargo_static_ofs);
	}

	// save bay door operations status
	plop->SaveState (scn);
}

// --------------------------------------------------------------
// Vessel gains or loses input focus
// --------------------------------------------------------------
void Atlantis::clbkFocusChanged (bool getfocus, OBJHANDLE newv, OBJHANDLE oldv)
{
	if (getfocus) {
		oapiDisableMFDMode (MFD_LANDING);
		// no VTOL MFD mode for Atlantis
	}
}

// --------------------------------------------------------------
// Simulation time step
// --------------------------------------------------------------
void Atlantis::clbkPostStep (double simt, double simdt, double mjd)
{
	double met;
	int i;
	OBJHANDLE hvessel;

	engine_light_level = GetThrusterGroupLevel (THGROUP_MAIN);

	switch (status) {
	case 0: // launch configuration
		if (GetEngineLevel (ENGINE_MAIN) > 0.95) {
			status = 1; // launch
			t0 = simt + SRB_STABILISATION_TIME;   // store designated liftoff time
			RecordEvent ("STATUS", "SRB_IGNITION");
		} else
			AutoMainGimbal();
		break;
	case 1: // SRB's ignited
		met = simt-t0;
		if (met > SRB_SEPARATION_TIME && !Playback() || bManualSeparate) { // separate boosters
			SeparateBoosters (met);
			bManualSeparate = false;
		} else {
			// extract current thrust level and propellant level as a function of time
			double thrust_level, prop_level;
			GetSRB_State (met, thrust_level, prop_level);
			for (i = 0; i < 2; i++)
				SetThrusterLevel (th_srb[i], thrust_level);
			if (met < 0.0) LaunchClamps ();
			AutoMainGimbal();
		}
		break;

	case 2: // post SRB separation
		if (GetPropellantMass (ph_tank) < 1.0 && !Playback() || bManualSeparate) {
			SeparateTank();
			bManualSeparate = false;
		}
		break;
	case 3: // post tank separation
		if (bManualSeparate && GetAttachmentStatus (sat_attach)) {
			DetachChild (sat_attach, 0.1);
			bManualSeparate = false;
		}

		if (do_eva) {
			char name[256];
			strcpy (name, GetName()); strcat (name, "-MMU");
			hvessel = oapiGetVesselByName (name);
			if (!hvessel) {
				SeparateMMU ();
			}
			do_eva = false;
		};
		break;
	}

	// Execute payload bay operations
	plop->Step (simt, simdt);

	// ***** Animate landing gear *****

	if (gear_status >= AnimState::CLOSING) {
		double da = simdt * GEAR_OPERATING_SPEED;
		if (gear_status == AnimState::CLOSING) { // retract gear
			if (gear_proc > 0.0) gear_proc = max (0.0, gear_proc-da);
			else                 gear_status = AnimState::CLOSED;
		} else {                           // deploy gear
			if (gear_proc < 1.0) gear_proc = min (1.0, gear_proc+da);
			else                 gear_status = AnimState::OPEN;
		}
		SetAnimation (anim_gear, gear_proc);
		SetGearParameters (gear_proc);
	}

	// ***** Animate speedbrake *****

	if (spdb_status >= AnimState::CLOSING) {
		double da = simdt * SPEEDBRAKE_OPERATING_SPEED;
		if (spdb_status == AnimState::CLOSING) { // retract brake
			if (spdb_proc > 0.0) spdb_proc = max (0.0, spdb_proc-da);
			else                 spdb_status = AnimState::CLOSED;
		} else {                           // deploy antenna
			if (spdb_proc < 1.0) spdb_proc = min (1.0, spdb_proc+da);
			else                 spdb_status = AnimState::OPEN;
		}
		SetAnimation (anim_spdb, spdb_proc);
	}

	// ***** Stow RMS arm *****

	if (center_arm) {
		double t0 = oapiGetSimTime();
		double dt = t0 - center_arm_t;       // time step
		double da = ARM_OPERATING_SPEED*dt;  // total rotation angle

		// work from the wrist down to the shoulder
		if (da && (arm_wr != 0.5)) {    // zero wrist roll
			if (da >= fabs(arm_wr-0.5)) // finished
				arm_wr = 0.5, da -= fabs(arm_wr-0.5);
			else
				arm_wr -= (arm_wr > 0.5 ? da:-da), da = 0;
			SetAnimationArm (anim_arm_wr, arm_wr);
		}
		if (da && (arm_wy != 0.5)) {    // zero wrist yaw
			if (da >= fabs(arm_wy-0.5)) // finished
				arm_wy = 0.5, da -= fabs(arm_wy-0.5);
			else
				arm_wy -= (arm_wy > 0.5 ? da:-da), da = 0;
			SetAnimationArm (anim_arm_wy, arm_wy);
		}
		if (da && (arm_wp != 0.5)) {    // zero wrist pitch
			if (da >= fabs(arm_wp-0.5)) // finished
				arm_wp = 0.5, da -= fabs(arm_wp-0.5);
			else
				arm_wp -= (arm_wp > 0.5 ? da:-da), da = 0;
			SetAnimationArm (anim_arm_wp, arm_wp);
		}
		if (da && arm_ep) {             // zero elbow pitch
			if (da >= arm_ep)           // finished
				arm_ep = 0.0, da -= arm_ep;
			else
				arm_ep -= da, da = 0;
			SetAnimationArm (anim_arm_ep, arm_ep);
		}
		if (da && (arm_sy != 0.5)) {    // zero shoulder yaw
			if (da >= fabs(arm_sy-0.5)) // finished
				arm_sy = 0.5, da -= fabs(arm_sy-0.5);
			else
				arm_sy -= (arm_sy > 0.5 ? da:-da), da = 0;
			SetAnimationArm (anim_arm_sy, arm_sy);
		}
		if (da && arm_sp) {             // zero shoulder pitch
			if (da >= arm_sp)           // finished
				arm_sp = 0.0, da -= arm_sp;
			else
				arm_sp -= da, da = 0;
			SetAnimationArm (anim_arm_sp, arm_sp);
		}
		center_arm_t = t0;
		if (da) {
			center_arm = false; // finished stowing
			HWND hDlg = oapiFindDialog (g_Param.hDLL, IDD_RMS);
			if (hDlg) EnableWindow (GetDlgItem (hDlg, IDC_GRAPPLE), TRUE);
		}
	}

	if (arm_moved) {
		SetAttachmentParams (rms_attach, orbiter_ofs+arm_tip[0], arm_tip[1]-arm_tip[0], arm_tip[2]-arm_tip[0]);
		arm_moved = false;
	}
}

// --------------------------------------------------------------
// Respond to playback event
// --------------------------------------------------------------
bool Atlantis::clbkPlaybackEvent (double simt, double event_t, const char *event_type, const char *event)
{
	if (!_stricmp (event_type, "JET")) {
		if (!_stricmp (event, "SRB")) {
			bManualSeparate = true;
			return true;
		}
		else if (!_stricmp (event, "ET")) {
			bManualSeparate = true;
			return true;
		}
	} else if (!_stricmp (event_type, "STATUS")) {
		if (!_stricmp (event, "SRB_IGNITION")) {
			status = 1;
			t0 = event_t + SRB_STABILISATION_TIME;
			return true;
		}
	} else if (!_stricmp (event_type, "ADJUST_LAUNCHTIME")) {
		sscanf (event, "%lf", &t0);
		return true;
	} else if (!_stricmp (event_type, "CARGODOOR")) {
		plop->SetDoorAction (!_stricmp (event, "CLOSE") ? AnimState::CLOSING : AnimState::OPENING, true);
		return true;
	} else if (!_stricmp (event_type, "GEAR")) {
		OperateLandingGear (!_stricmp (event, "UP") ? AnimState::CLOSING : AnimState::OPENING);
		return true;
	} else if (!_stricmp (event_type,"SPEEDBRAKE")) {
		OperateSpeedbrake (!_stricmp (event, "CLOSE") ? AnimState::CLOSING : AnimState::OPENING);
		return true;
	} else if (!_stricmp (event_type, "KUBAND")) {
		plop->SetKuAntennaAction (!_stricmp (event, "CLOSE") ? AnimState::CLOSING : AnimState::OPENING);
		return true;
	}

	return false;
}

// --------------------------------------------------------------
// Atlantis mesh loaded
// --------------------------------------------------------------
void Atlantis::clbkVisualCreated (VISHANDLE _vis, int refcount)
{
	if (refcount > 1) return; // we don't support more than one visual per object
	vis = _vis;

#ifdef UNDEF
	// note: orbiter re-applies the animations to the mesh before calling
	// clbkVisualCreated, so updating the mesh here is not necessary

	// reset grappling point
	arm_tip[0] = _V(-2.26,1.71,-6.5);
	arm_tip[1] = _V(-2.26,1.71,-7.5);
	arm_tip[2] = _V(-2.26,2.71,-6.5);

	UpdateMesh ();
#endif
}

// --------------------------------------------------------------
// Atlantis mesh discarded
// --------------------------------------------------------------
void Atlantis::clbkVisualDestroyed (VISHANDLE _vis, int refcount)
{
	if (vis == _vis) vis = NULL;
}

// --------------------------------------------------------------
// Update mesh animation state
// --------------------------------------------------------------
void Atlantis::clbkAnimate (double simt)
{
	UpdateMesh ();
}

// --------------------------------------------------------------
// Respond to MFD mode change
// --------------------------------------------------------------
void Atlantis::clbkMFDMode (int mfd, int mode)
{
	oapiVCTriggerRedrawArea (-1, AID_CDR1_BUTTONS+mfd-MFD_LEFT);
}

// --------------------------------------------------------------
// Load generic glass cockpit mode
// --------------------------------------------------------------
bool Atlantis::clbkLoadGenericCockpit ()
{
	SetCameraOffset (_V(orbiter_ofs.x-0.67,orbiter_ofs.y+2.55,orbiter_ofs.z+14.4));
	SetCameraDefaultDirection (_V(0,0,1));
	return true;
}

// --------------------------------------------------------------
// register VC buttons for the 2 commander MFDs
// (accessible from commander position only)
// --------------------------------------------------------------
void Atlantis::RegisterVC_CdrMFD ()
{
	// activate MFD function buttons
	oapiVCSetAreaClickmode_Quadrilateral (AID_CDR1_BUTTONS, _V(-0.9239,2.0490,15.0595)+orbiter_ofs, _V(-0.7448,2.0490,15.0595)+orbiter_ofs,  _V(-0.9239,2.0280,15.0595)+orbiter_ofs, _V(-0.7448,2.0280,15.0595)+orbiter_ofs);
	oapiVCSetAreaClickmode_Quadrilateral (AID_CDR2_BUTTONS, _V(-0.6546,2.0490,15.0595)+orbiter_ofs, _V(-0.4736,2.0490,15.0595)+orbiter_ofs,  _V(-0.6546,2.0280,15.0595)+orbiter_ofs, _V(-0.4736,2.0280,15.0595)+orbiter_ofs);

    // D. Beachy: register+activate MFD power buttons
    const double powerButtonRadius = 0.0075; // radius of power button on each MFD
	oapiVCRegisterArea (AID_CDR1_PWR, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea (AID_CDR2_PWR, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_ONREPLAY);
    oapiVCSetAreaClickmode_Spherical(AID_CDR1_PWR, _V(-0.950, 2.060, 15.060)+orbiter_ofs, powerButtonRadius);  
    oapiVCSetAreaClickmode_Spherical(AID_CDR2_PWR, _V(-0.680, 2.060, 15.060)+orbiter_ofs, powerButtonRadius);  

	// register+activate MFD brightness buttons
	oapiVCRegisterArea (AID_CDR1_BRT, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea (AID_CDR2_BRT, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY);
	oapiVCSetAreaClickmode_Quadrilateral (AID_CDR1_BRT, _V(-0.729,2.0675,15.060)+orbiter_ofs, _V(-0.714,2.0675,15.060)+orbiter_ofs, _V(-0.729,2.0525,15.060)+orbiter_ofs, _V(-0.714,2.0525,15.060)+orbiter_ofs);
	oapiVCSetAreaClickmode_Quadrilateral (AID_CDR2_BRT, _V(-0.459,2.0675,15.060)+orbiter_ofs, _V(-0.444,2.0675,15.060)+orbiter_ofs, _V(-0.459,2.0525,15.060)+orbiter_ofs, _V(-0.444,2.0525,15.060)+orbiter_ofs);
}

// --------------------------------------------------------------
// register VC buttons for the 2 pilot MFDs
// (accessible from pilot position only)
// --------------------------------------------------------------
void Atlantis::RegisterVC_PltMFD ()
{
	// activate MFD function buttons
	oapiVCSetAreaClickmode_Quadrilateral (AID_PLT1_BUTTONS, _V(0.4759,2.0490,15.0595)+orbiter_ofs, _V(0.6568,2.0490,15.0595)+orbiter_ofs,  _V(0.4759,2.0280,15.0595)+orbiter_ofs, _V(0.6568,2.0280,15.0595)+orbiter_ofs);
	oapiVCSetAreaClickmode_Quadrilateral (AID_PLT2_BUTTONS, _V(0.7461,2.0490,15.0595)+orbiter_ofs, _V(0.9271,2.0490,15.0595)+orbiter_ofs,  _V(0.7461,2.0280,15.0595)+orbiter_ofs, _V(0.9271,2.0280,15.0595)+orbiter_ofs);

    // D. Beachy: register+activate MFD power buttons
    const double powerButtonRadius = 0.0075; // radius of power button on each MFD
	oapiVCRegisterArea (AID_PLT1_PWR, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea (AID_PLT2_PWR, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_ONREPLAY);
    oapiVCSetAreaClickmode_Spherical(AID_PLT1_PWR, _V( 0.450, 2.060, 15.060)+orbiter_ofs, powerButtonRadius);  
    oapiVCSetAreaClickmode_Spherical(AID_PLT2_PWR, _V( 0.720, 2.060, 15.060)+orbiter_ofs, powerButtonRadius);  

	// register+activate MFD brightness buttons
	oapiVCRegisterArea (AID_PLT1_BRT, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea (AID_PLT2_BRT, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY);
	oapiVCSetAreaClickmode_Quadrilateral (AID_PLT1_BRT, _V(0.671,2.0675,15.060)+orbiter_ofs, _V(0.686,2.0675,15.060)+orbiter_ofs, _V(0.671,2.0525,15.060)+orbiter_ofs, _V(0.686,2.0525,15.060)+orbiter_ofs);
	oapiVCSetAreaClickmode_Quadrilateral (AID_PLT2_BRT, _V(0.941,2.0675,15.060)+orbiter_ofs, _V(0.956,2.0675,15.060)+orbiter_ofs, _V(0.941,2.0525,15.060)+orbiter_ofs, _V(0.956,2.0525,15.060)+orbiter_ofs);
}

// --------------------------------------------------------------
// register VC buttons for the 5 MFDs on the central panel
// (accessible from commander and pilot positions)
// --------------------------------------------------------------
void Atlantis::RegisterVC_CntMFD ()
{
	// activate MFD function buttons
	oapiVCSetAreaClickmode_Quadrilateral (AID_MFD1_BUTTONS, _V(-0.3579,2.1451,15.0863)+orbiter_ofs, _V(-0.1770,2.1451,15.0863)+orbiter_ofs, _V(-0.3579,2.1241,15.0863)+orbiter_ofs, _V(-0.1770,2.1241,15.0863)+orbiter_ofs);
	oapiVCSetAreaClickmode_Quadrilateral (AID_MFD2_BUTTONS, _V(-0.3579,1.9143,15.0217)+orbiter_ofs, _V(-0.1770,1.9143,15.0217)+orbiter_ofs, _V(-0.3579,1.8933,15.0217)+orbiter_ofs, _V(-0.1770,1.8933,15.0217)+orbiter_ofs);
	oapiVCSetAreaClickmode_Quadrilateral (AID_MFD3_BUTTONS, _V(-0.0888,2.0288,15.0538)+orbiter_ofs, _V(0.0922,2.0288,15.0538)+orbiter_ofs, _V(-0.0888,2.0078,15.0538)+orbiter_ofs, _V(0.0922,2.0078,15.0538)+orbiter_ofs);
	oapiVCSetAreaClickmode_Quadrilateral (AID_MFD4_BUTTONS, _V(0.1795,2.1451,15.0863)+orbiter_ofs, _V(0.3604,2.1451,15.0863)+orbiter_ofs, _V(0.1795,2.1241,15.0863)+orbiter_ofs, _V(0.3604,2.1241,15.0863)+orbiter_ofs);
	oapiVCSetAreaClickmode_Quadrilateral (AID_MFD5_BUTTONS, _V(0.1795,1.9143,15.0217)+orbiter_ofs, _V(0.3604,1.9143,15.0217)+orbiter_ofs, _V(0.1795,1.8933,15.0217)+orbiter_ofs, _V(0.3604,1.8933,15.0217)+orbiter_ofs);

    // D. Beachy: register+activate MFD power buttons
    const double powerButtonRadius = 0.0075; // radius of power button on each MFD
	oapiVCRegisterArea (AID_MFD1_PWR, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea (AID_MFD2_PWR, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea (AID_MFD3_PWR, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea (AID_MFD4_PWR, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea (AID_MFD5_PWR, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_ONREPLAY);
    oapiVCSetAreaClickmode_Spherical(AID_MFD1_PWR, _V(-0.383, 2.153, 15.090)+orbiter_ofs, powerButtonRadius);  
    oapiVCSetAreaClickmode_Spherical(AID_MFD2_PWR, _V(-0.383, 1.922, 15.023)+orbiter_ofs, powerButtonRadius);  
    oapiVCSetAreaClickmode_Spherical(AID_MFD3_PWR, _V(-0.114, 2.037, 15.058)+orbiter_ofs, powerButtonRadius);  
    oapiVCSetAreaClickmode_Spherical(AID_MFD4_PWR, _V( 0.155, 2.153, 15.090)+orbiter_ofs, powerButtonRadius);  
    oapiVCSetAreaClickmode_Spherical(AID_MFD5_PWR, _V( 0.155, 1.922, 15.023)+orbiter_ofs, powerButtonRadius);  

	// register+activate MFD brightness buttons
	oapiVCRegisterArea (AID_MFD1_BRT, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea (AID_MFD2_BRT, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea (AID_MFD3_BRT, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea (AID_MFD4_BRT, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea (AID_MFD5_BRT, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY);
	oapiVCSetAreaClickmode_Quadrilateral (AID_MFD1_BRT, _V(-0.162,2.1605,15.090)+orbiter_ofs, _V(-0.147,2.1605,15.090)+orbiter_ofs, _V(-0.162,2.1455,15.090)+orbiter_ofs, _V(-0.147,2.1455,15.090)+orbiter_ofs);
	oapiVCSetAreaClickmode_Quadrilateral (AID_MFD2_BRT, _V(-0.162,1.9295,15.023)+orbiter_ofs, _V(-0.147,1.9295,15.023)+orbiter_ofs, _V(-0.162,1.9145,15.023)+orbiter_ofs, _V(-0.147,1.9145,15.023)+orbiter_ofs);
	oapiVCSetAreaClickmode_Quadrilateral (AID_MFD3_BRT, _V(0.107,2.0445,15.058)+orbiter_ofs, _V(0.122,2.0445,15.058)+orbiter_ofs, _V(0.107,2.0295,15.058)+orbiter_ofs, _V(0.122,2.0295,15.058)+orbiter_ofs);
	oapiVCSetAreaClickmode_Quadrilateral (AID_MFD4_BRT, _V(0.376,2.1605,15.090)+orbiter_ofs, _V(0.391,2.1605,15.090)+orbiter_ofs, _V(0.376,2.1455,15.090)+orbiter_ofs, _V(0.391,2.1455,15.090)+orbiter_ofs);
	oapiVCSetAreaClickmode_Quadrilateral (AID_MFD5_BRT, _V(0.376,1.9295,15.023)+orbiter_ofs, _V(0.391,1.9295,15.023)+orbiter_ofs, _V(0.376,1.9145,15.023)+orbiter_ofs, _V(0.391,1.9145,15.023)+orbiter_ofs);
}

// --------------------------------------------------------------
// register VC buttons for the aft MFD at the starbord panel
// (accessible from payload control position only)
// --------------------------------------------------------------
void Atlantis::RegisterVC_AftMFD ()
{
	// register+activate aft MFD function buttons
	SURFHANDLE tex1 = oapiGetTextureHandle (hOrbiterVCMesh, 7);
	oapiVCRegisterArea (AID_MFDA_BUTTONS, _R(0,127,255,140), PANEL_REDRAW_USER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBUP|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY, PANEL_MAP_BACKGROUND, tex1);
	oapiVCSetAreaClickmode_Quadrilateral (AID_MFDA_BUTTONS, _V(1.3862,2.2570,13.8686)+orbiter_ofs, _V(1.3862,2.2570,13.6894)+orbiter_ofs, _V(1.3678,2.2452,13.8686)+orbiter_ofs, _V(1.3678,2.2452,13.6894)+orbiter_ofs);

	// register+activate MFD power button
    const double powerButtonRadius = 0.0075; // radius of power button on each MFD
	oapiVCRegisterArea (AID_MFDA_PWR, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_ONREPLAY);
    oapiVCSetAreaClickmode_Spherical(AID_MFDA_PWR, _V(1.3929,2.2632,13.8947)+orbiter_ofs, powerButtonRadius);

	// register+activate MFD brightness buttons
	oapiVCRegisterArea (AID_MFDA_BRT, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY);
	oapiVCSetAreaClickmode_Quadrilateral (AID_MFDA_BRT, _V(1.4024,2.2675,13.6736)+orbiter_ofs, _V(1.4024,2.2675,13.6586)+orbiter_ofs, _V(1.3893,2.2590,13.6736)+orbiter_ofs, _V(1.3893,2.2590,13.6586)+orbiter_ofs);
}

// --------------------------------------------------------------
// Load virtual cockpit mode
// --------------------------------------------------------------
bool Atlantis::clbkLoadVC (int id)
{
	bool ok = false;

	// register MFD function buttons
	// this needs to be done globally, so that the labels are correctly updated from all VC positions
	SURFHANDLE tex1 = oapiGetTextureHandle (hOrbiterVCMesh, 7);

	// commander MFD function buttons
	oapiVCRegisterArea (AID_CDR1_BUTTONS, _R(0,1,255,14), PANEL_REDRAW_USER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBUP|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY, PANEL_MAP_BACKGROUND, tex1);
	oapiVCRegisterArea (AID_CDR2_BUTTONS, _R(0,15,255,28), PANEL_REDRAW_USER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBUP|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY, PANEL_MAP_BACKGROUND, tex1);
	// pilot MFD function buttons
	oapiVCRegisterArea (AID_PLT1_BUTTONS, _R(0,29,255,42), PANEL_REDRAW_USER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBUP|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY, PANEL_MAP_BACKGROUND, tex1);
	oapiVCRegisterArea (AID_PLT2_BUTTONS, _R(0,43,255,56), PANEL_REDRAW_USER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBUP|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY, PANEL_MAP_BACKGROUND, tex1);
	// central console MFD function buttons
	oapiVCRegisterArea (AID_MFD1_BUTTONS, _R(0, 57,255, 70), PANEL_REDRAW_USER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBUP|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY, PANEL_MAP_BACKGROUND, tex1);
	oapiVCRegisterArea (AID_MFD2_BUTTONS, _R(0, 71,255, 84), PANEL_REDRAW_USER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBUP|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY, PANEL_MAP_BACKGROUND, tex1);
	oapiVCRegisterArea (AID_MFD3_BUTTONS, _R(0, 85,255, 98), PANEL_REDRAW_USER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBUP|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY, PANEL_MAP_BACKGROUND, tex1);
	oapiVCRegisterArea (AID_MFD4_BUTTONS, _R(0, 99,255,112), PANEL_REDRAW_USER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBUP|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY, PANEL_MAP_BACKGROUND, tex1);
	oapiVCRegisterArea (AID_MFD5_BUTTONS, _R(0,113,255,126), PANEL_REDRAW_USER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_LBUP|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_ONREPLAY, PANEL_MAP_BACKGROUND, tex1);

	switch (id) {
	case 0: // commander position
		SetCameraOffset (_V(orbiter_ofs.x-0.67,orbiter_ofs.y+2.55,orbiter_ofs.z+14.4));
		SetCameraDefaultDirection (_V(0,0,1));
		SetCameraMovement (_V(0,0,0.3), 0, 0, _V(-0.3,0,0), 75*RAD, -5*RAD, _V(0.3,0,0), -20*RAD, -27*RAD);
		huds.hudcnt = _V(orbiter_ofs.x-0.671257, orbiter_ofs.y+2.523535, orbiter_ofs.z+14.969);
		oapiVCSetNeighbours (-1, 1, -1, 2);

		RegisterVC_CdrMFD (); // activate commander MFD controls
		RegisterVC_CntMFD (); // activate central panel MFD controls

		ok = true;
		break;
	case 1: // pilot position
		SetCameraOffset (_V(orbiter_ofs.x+0.67,orbiter_ofs.y+2.55,orbiter_ofs.z+14.4));
		SetCameraDefaultDirection (_V(0,0,1));
		SetCameraMovement (_V(0,0,0.3), 0, 0, _V(-0.3,0,0), 20*RAD, -27*RAD, _V(0.3,0,0), -75*RAD, -5*RAD);
		huds.hudcnt = _V(orbiter_ofs.x+0.671257, orbiter_ofs.y+2.523535, orbiter_ofs.z+14.969);
		oapiVCSetNeighbours (0, -1, -1, 2);

		RegisterVC_PltMFD (); // activate pilot MFD controls
		RegisterVC_CntMFD (); // activate central panel MFD controls

		ok = true;
		break;
	case 2: // payload view position
		SetCameraOffset (_V(orbiter_ofs.x+0.4,orbiter_ofs.y+3.15,orbiter_ofs.z+12.8));
		SetCameraDefaultDirection (_V(0,0,-1));
		SetCameraMovement (_V(0,0.20,0.20), 0, 40.0*RAD, _V(0.3,-0.3,0.15), 60.0*RAD, -50.0*RAD, _V(-0.8,0,0), 0, 0);
		oapiVCSetNeighbours (1, 0, -1, 0);

		RegisterVC_AftMFD (); // activate aft MFD controls
		plop->RegisterVC ();  // register panel R13L interface
		ok = true;
		break;
	}

	if (ok) {
		// register the HUDs (synced)
		oapiVCRegisterHUD (&huds);
		// register all MFD displays
		for (int i = 0; i < 10; i++)
			oapiRegisterMFD (MFD_LEFT+i, mfds+i);
		// update panel R13L
		plop->UpdateVC();
	}
	return ok;
}

// --------------------------------------------------------------
// Respond to virtual cockpit mouse event
// --------------------------------------------------------------
bool Atlantis::clbkVCMouseEvent (int id, int event, VECTOR3 &p)
{
	static bool counting = false;
	static double t0 = 0.0;

	switch (id) {
	// handle MFD selection buttons
	case AID_CDR1_BUTTONS:
	case AID_CDR2_BUTTONS:
	case AID_PLT1_BUTTONS:
	case AID_PLT2_BUTTONS:
	case AID_MFD1_BUTTONS:
	case AID_MFD2_BUTTONS:
	case AID_MFD3_BUTTONS:
	case AID_MFD4_BUTTONS: 
	case AID_MFD5_BUTTONS:
	case AID_MFDA_BUTTONS: {
		int mfd = id-AID_CDR1_BUTTONS+MFD_LEFT;
		int bt = (int)(p.x*5.99);
		if (bt < 5) oapiProcessMFDButton (mfd, bt, event);
		else {
			if (event & PANEL_MOUSE_LBDOWN) {
				t0 = oapiGetSysTime();
				counting = true;
			} else if ((event & PANEL_MOUSE_LBUP) && counting) {
				oapiSendMFDKey (mfd, OAPI_KEY_F2);
				counting = false;
			} else if ((event & PANEL_MOUSE_LBPRESSED) && counting && (oapiGetSysTime()-t0 >= 1.0)) {
				oapiSendMFDKey (mfd, OAPI_KEY_F1);
				counting = false;
			}
		}
		} return true;

    // D. Beachy: handle power buttons
    case AID_CDR1_PWR:
    case AID_CDR2_PWR:
	case AID_PLT1_PWR:
	case AID_PLT2_PWR:
    case AID_MFD1_PWR:
    case AID_MFD2_PWR:
    case AID_MFD3_PWR:
	case AID_MFD4_PWR: 
	case AID_MFD5_PWR:
	case AID_MFDA_PWR: {
        int mfd = id - AID_CDR1_PWR+MFD_LEFT;
        oapiSendMFDKey(mfd, OAPI_KEY_ESCAPE);
        } return true;
              
	// handle MFD brightness buttons
	case AID_CDR1_BRT:
	case AID_CDR2_BRT:
	case AID_PLT1_BRT:
	case AID_PLT2_BRT:
	case AID_MFD1_BRT:
	case AID_MFD2_BRT:
	case AID_MFD3_BRT: 
	case AID_MFD4_BRT: 
	case AID_MFD5_BRT:
	case AID_MFDA_BRT: {
		static double t0, brt0;
		static bool up;
		int mfd = id-AID_CDR1_BRT;
		if (event & PANEL_MOUSE_LBDOWN) {
			up = (p.x >= 0.5);
			t0 = oapiGetSysTime();
			brt0 = mfdbright[mfd];
		} else if (event & PANEL_MOUSE_LBPRESSED) {
			double dt = oapiGetSysTime()-t0;
			double brt, dbrt = dt * 0.2;
			if (up) brt = min (1.0, brt0 + dbrt);
			else    brt = max (0.25, brt0 - dbrt);
			mfdbright[mfd] = brt;
			if (vis) {
				MESHHANDLE hMesh = GetMesh (vis, mesh_vc);
				MATERIAL *mat = oapiMeshMaterial (hMesh, 10+mfd);
				mat->emissive.r = mat->emissive.g = mat->emissive.b = (float)brt;
			}
		}
		} return false;

	// handle panel R13L events (payload bay operations)
	case AID_R13L:
		return plop->VCMouseEvent (id, event, p);
	}
	return false;
}

// --------------------------------------------------------------
// Respond to virtual cockpit area redraw request
// --------------------------------------------------------------
bool Atlantis::clbkVCRedrawEvent (int id, int event, SURFHANDLE surf)
{
	switch (id) {
	case AID_CDR1_BUTTONS:
	case AID_CDR2_BUTTONS:
	case AID_PLT1_BUTTONS:
	case AID_PLT2_BUTTONS:
	case AID_MFD1_BUTTONS:
	case AID_MFD2_BUTTONS:
	case AID_MFD3_BUTTONS:
	case AID_MFD4_BUTTONS: 
	case AID_MFD5_BUTTONS:
	case AID_MFDA_BUTTONS: {
		int mfd = id-AID_CDR1_BUTTONS+MFD_LEFT;
		RedrawPanel_MFDButton (surf, mfd);
		} return true;
	default:
		if (id >= AID_R13L_MIN && id <= AID_R13L_MAX)
			return plop->VCRedrawEvent (id, event, surf);
		break;
	}
	return false;
}

// --------------------------------------------------------------
// Respond to a HUD redraw request
// --------------------------------------------------------------
bool Atlantis::clbkDrawHUD (int mode, const HUDPAINTSPEC *hps, oapi::Sketchpad *skp)
{
	// draw the default HUD
	VESSEL3::clbkDrawHUD (mode, hps, skp);
	int cx = hps->CX, cy = hps->CY;

	// show OMS thrust marker
	if (status >= 3) {
		int omsy = cy + (int)(15.0*hps->Scale);
		int dx = (int)(1.0*hps->Scale);
		skp->Line (cx-2*dx, omsy, cx+2*dx, omsy);
		skp->Line (cx, omsy-dx, cx, omsy+dx);
	}

	// show RCS mode
	if (status >= 3 && oapiCockpitMode() == COCKPIT_VIRTUAL) {
		switch (GetAttitudeMode()) {
		case RCS_ROT:
			skp->Text (0, hps->H-13, "RCS ROT", 7);
			break;
		case RCS_LIN:
			skp->Text (0, hps->H-13, "RCS_LIN", 7);
			break;
		}
	}
	return true;
}

// --------------------------------------------------------------
// Keyboard interface handler (buffered key events)
// --------------------------------------------------------------
int Atlantis::clbkConsumeBufferedKey (DWORD key, bool down, char *kstate)
{
	if (!down) return 0; // only process keydown events

	if (KEYMOD_SHIFT (kstate)) {

		switch (key) {
		case OAPI_KEY_E:
			if (status != 3) return 1; // Allow MMU only after orbiter has detached from MT
			return 1;
		}	
	} else if (KEYMOD_CONTROL (kstate)) {
		switch (key) {
		case OAPI_KEY_SPACE: // open RMS control dialog
			oapiOpenDialogEx (g_Param.hDLL, IDD_CTRL, Atlantis_DlgProc, DLG_CAPTIONCLOSE, this);
			return 1;
		case OAPI_KEY_B: // deploy/retract speedbrake
			if (!Playback()) RevertSpeedbrake ();
			return 1;
		case OAPI_KEY_U: // deploy/store Ku-band antenna
			if (!Playback()) plop->RevertKuAntennaAction ();
			return 1;
		}
	} else { // unmodified keys
		switch (key) {
		case OAPI_KEY_G:  // "Landing gear"
			if (!Playback()) RevertLandingGear ();
			return 1;
		case OAPI_KEY_J:  // "Jettison"
			if (!Playback()) bManualSeparate = true;
			return 1;
		case OAPI_KEY_K:  // "Cargo bay doors"
			if (!Playback()) plop->RevertDoorAction ();
			return 1;
		case OAPI_KEY_8:
			ToggleGrapple();
			return 1;
		case OAPI_KEY_9: 
			center_arm = true;
			return 1;
		case OAPI_KEY_E:
			do_eva = true;
			return 1;
		}
	}
	return 0;
}

// ==============================================================
// API callback interface
// ==============================================================

// --------------------------------------------------------------
// Module initialisation
// --------------------------------------------------------------
DLLCLBK void InitModule (HINSTANCE hModule)
{
	g_Param.hDLL = hModule;
	oapiRegisterCustomControls (hModule);
	g_Param.tkbk_label = oapiCreateSurface (LOADBMP (IDB_TKBKLABEL));

	// allocate GDI resources
	g_Param.font[0] = CreateFont (-11, 0, 0, 0, 400, 0, 0, 0, 0, 0, 0, 0, 0, "Arial");
}

DLLCLBK void ExitModule (HINSTANCE hModule)
{
	oapiUnregisterCustomControls (hModule);
	oapiDestroySurface (g_Param.tkbk_label);

	// deallocate GDI resources
	DeleteObject (g_Param.font[0]);
}

// --------------------------------------------------------------
// Vessel initialisation
// --------------------------------------------------------------
DLLCLBK VESSEL *ovcInit (OBJHANDLE hvessel, int flightmodel)
{
	return new Atlantis (hvessel, flightmodel);
}

// --------------------------------------------------------------
// Vessel cleanup
// --------------------------------------------------------------
DLLCLBK void ovcExit (VESSEL *vessel)
{
	if (vessel) delete (Atlantis*)vessel;
}

// ==============================================================
// Message callback function for Atlantis control dialog box
// ==============================================================

BOOL CALLBACK Atlantis_DlgProc (HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	Atlantis *sts = (uMsg == WM_INITDIALOG ? (Atlantis*)lParam : (Atlantis*)oapiGetDialogContext (hWnd));
	// pointer to vessel instance was passed as dialog context

	switch (uMsg) {
	case WM_COMMAND:
		switch (LOWORD(wParam)) {
		case IDCANCEL:
			oapiCloseDialog (hWnd);
			return TRUE;
		case IDC_PLBAYOP:
			sts->plop->OpenDialog ();
			break;
		case IDC_RMSOP:
			oapiOpenDialogEx (g_Param.hDLL, IDD_RMS, RMS_DlgProc, DLG_CAPTIONCLOSE, sts);
			break;
		}
		break;
	}
	return oapiDefDialogProc (hWnd, uMsg, wParam, lParam);
}

// ==============================================================
// Message callback function for RMS control dialog box
// ==============================================================

BOOL CALLBACK RMS_DlgProc (HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	Atlantis *sts = (uMsg == WM_INITDIALOG ? (Atlantis*)lParam : (Atlantis*)oapiGetDialogContext (hWnd));
	// pointer to vessel instance was passed as dialog context

	const double step = 0.05*RAD;
	static double t0;
	double t1;
	HICON hIcon;

	switch (uMsg) {
	case WM_INITDIALOG:
		hIcon = LoadIcon (g_Param.hDLL, MAKEINTRESOURCE(IDI_UP));
		SendDlgItemMessage (hWnd, IDC_WRIST_PITCHUP, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
		SendDlgItemMessage (hWnd, IDC_ELBOW_PITCHUP, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
		SendDlgItemMessage (hWnd, IDC_SHOULDER_PITCHUP, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
		hIcon = LoadIcon (g_Param.hDLL, MAKEINTRESOURCE(IDI_DOWN));
		SendDlgItemMessage (hWnd, IDC_WRIST_PITCHDOWN, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
		SendDlgItemMessage (hWnd, IDC_ELBOW_PITCHDOWN, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
		SendDlgItemMessage (hWnd, IDC_SHOULDER_PITCHDOWN, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
		hIcon = LoadIcon (g_Param.hDLL, MAKEINTRESOURCE(IDI_LEFT));
		SendDlgItemMessage (hWnd, IDC_WRIST_YAWLEFT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
		SendDlgItemMessage (hWnd, IDC_SHOULDER_YAWLEFT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
		hIcon = LoadIcon (g_Param.hDLL, MAKEINTRESOURCE(IDI_RIGHT));
		SendDlgItemMessage (hWnd, IDC_WRIST_YAWRIGHT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
		SendDlgItemMessage (hWnd, IDC_SHOULDER_YAWRIGHT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
		hIcon = LoadIcon (g_Param.hDLL, MAKEINTRESOURCE(IDI_RRIGHT));
		SendDlgItemMessage (hWnd, IDC_WRIST_ROLLRIGHT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
		hIcon = LoadIcon (g_Param.hDLL, MAKEINTRESOURCE(IDI_RLEFT));
		SendDlgItemMessage (hWnd, IDC_WRIST_ROLLLEFT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
		SendDlgItemMessage (hWnd, IDC_SHOWGRAPPLE, BM_SETCHECK, oapiGetShowGrapplePoints() ? BST_CHECKED:BST_UNCHECKED, 0);
		SetWindowText (GetDlgItem (hWnd, IDC_GRAPPLE), sts->SatGrappled() ? "Release" : "Grapple");
		EnableWindow (GetDlgItem (hWnd, IDC_STOW), sts->SatGrappled() ? FALSE : TRUE);
		SetWindowText (GetDlgItem (hWnd, IDC_PAYLOAD), sts->SatStowed() ? "Purge" : "Arrest");
		EnableWindow (GetDlgItem (hWnd, IDC_PAYLOAD), sts->SatStowed() || sts->CanArrest() ? TRUE:FALSE);
		SetTimer (hWnd, 1, 50, NULL);
		t0 = oapiGetSimTime();
		return FALSE;
	case WM_DESTROY:
		KillTimer (hWnd, 1);
		return 0;
	case WM_TIMER:
		if (wParam == 1) {
			t1 = oapiGetSimTime();
			if (SendDlgItemMessage (hWnd, IDC_SHOULDER_YAWLEFT, BM_GETSTATE, 0, 0) & BST_PUSHED) {
				sts->arm_sy = min (1.0, sts->arm_sy + (t1-t0)*ARM_OPERATING_SPEED);
				sts->SetAnimationArm (sts->anim_arm_sy, sts->arm_sy);
			} else if (SendDlgItemMessage (hWnd, IDC_SHOULDER_YAWRIGHT, BM_GETSTATE, 0, 0) & BST_PUSHED) {
				sts->arm_sy = max (0.0, sts->arm_sy - (t1-t0)*ARM_OPERATING_SPEED);
				sts->SetAnimationArm (sts->anim_arm_sy, sts->arm_sy);
			} else if (SendDlgItemMessage (hWnd, IDC_SHOULDER_PITCHUP, BM_GETSTATE, 0, 0) & BST_PUSHED) {
				sts->arm_sp = min (1.0, sts->arm_sp + (t1-t0)*ARM_OPERATING_SPEED);
				sts->SetAnimationArm (sts->anim_arm_sp, sts->arm_sp);
			} else if (SendDlgItemMessage (hWnd, IDC_SHOULDER_PITCHDOWN, BM_GETSTATE, 0, 0) & BST_PUSHED) {
				sts->arm_sp = max (0.0, sts->arm_sp - (t1-t0)*ARM_OPERATING_SPEED);
				sts->SetAnimationArm (sts->anim_arm_sp, sts->arm_sp);
			} else if (SendDlgItemMessage (hWnd, IDC_ELBOW_PITCHUP, BM_GETSTATE, 0, 0) & BST_PUSHED) {
				sts->arm_ep = max (0.0, sts->arm_ep - (t1-t0)*ARM_OPERATING_SPEED);
				sts->SetAnimationArm (sts->anim_arm_ep, sts->arm_ep);
			} else if (SendDlgItemMessage (hWnd, IDC_ELBOW_PITCHDOWN, BM_GETSTATE, 0, 0) & BST_PUSHED) {
				sts->arm_ep = min (1.0, sts->arm_ep + (t1-t0)*ARM_OPERATING_SPEED);
				sts->SetAnimationArm (sts->anim_arm_ep, sts->arm_ep);
			} else if (SendDlgItemMessage (hWnd, IDC_WRIST_PITCHUP, BM_GETSTATE, 0, 0) & BST_PUSHED) {
				sts->arm_wp = min (1.0, sts->arm_wp + (t1-t0)*ARM_OPERATING_SPEED);
				sts->SetAnimationArm (sts->anim_arm_wp, sts->arm_wp);
			} else if (SendDlgItemMessage (hWnd, IDC_WRIST_PITCHDOWN, BM_GETSTATE, 0, 0) & BST_PUSHED) {
				sts->arm_wp = max (0.0, sts->arm_wp - (t1-t0)*ARM_OPERATING_SPEED);
				sts->SetAnimationArm (sts->anim_arm_wp, sts->arm_wp);
			} else if (SendDlgItemMessage (hWnd, IDC_WRIST_YAWLEFT, BM_GETSTATE, 0, 0) & BST_PUSHED) {
				sts->arm_wy = min (1.0, sts->arm_wy + (t1-t0)*ARM_OPERATING_SPEED);
				sts->SetAnimationArm (sts->anim_arm_wy, sts->arm_wy);
			} else if (SendDlgItemMessage (hWnd, IDC_WRIST_YAWRIGHT, BM_GETSTATE, 0, 0) & BST_PUSHED) {
				sts->arm_wy = max (0.0, sts->arm_wy - (t1-t0)*ARM_OPERATING_SPEED);
				sts->SetAnimationArm (sts->anim_arm_wy, sts->arm_wy);
			} else if (SendDlgItemMessage (hWnd, IDC_WRIST_ROLLLEFT, BM_GETSTATE, 0, 0) & BST_PUSHED) {
				sts->arm_wr = max (0.0, sts->arm_wr - (t1-t0)*ARM_OPERATING_SPEED);
				sts->SetAnimationArm (sts->anim_arm_wr, sts->arm_wr);
			} else if (SendDlgItemMessage (hWnd, IDC_WRIST_ROLLRIGHT, BM_GETSTATE, 0, 0) & BST_PUSHED) {
				sts->arm_wr = min (1.0, sts->arm_wr + (t1-t0)*ARM_OPERATING_SPEED);
				sts->SetAnimationArm (sts->anim_arm_wr, sts->arm_wr);
			}
			t0 = t1;
		}
		if (!sts->center_arm) EnableWindow (GetDlgItem (hWnd, IDC_GRAPPLE), TRUE);
		break;
	case WM_COMMAND:
		switch (LOWORD(wParam)) {
		case IDCANCEL:
			oapiCloseDialog (hWnd);
			return TRUE;
		case IDC_STOW:
			if (sts->center_arm = !sts->center_arm) {
				sts->center_arm_t = oapiGetSimTime();
				EnableWindow (GetDlgItem (hWnd, IDC_GRAPPLE), FALSE);
			}
			return 0;
		case IDC_GRAPPLE:
			sts->ToggleGrapple();
			return 0;
		case IDC_PAYLOAD:
			sts->ToggleArrest();
			return 0;
		case IDC_SHOWGRAPPLE:
			oapiSetShowGrapplePoints (SendDlgItemMessage (hWnd, IDC_SHOWGRAPPLE, BM_GETCHECK, 0, 0) == BST_CHECKED ? true : false);
			return 0;
		}
		break;
	}
	return oapiDefDialogProc (hWnd, uMsg, wParam, lParam);
}
