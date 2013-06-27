// ==============================================================
//                ORBITER MODULE: DeltaGlider
//                  Part of the ORBITER SDK
//          Copyright (C) 2001-2008 Martin Schweiger
//                   All rights reserved
//
// ThrottleScram.cpp
// Scram throttle sliders
// ==============================================================

#define STRICT 1
#include "ThrottleScram.h"
#include "DeltaGlider.h"

// ==============================================================

// constants for texture coordinates
static const float texw = (float)PANEL2D_TEXW; // texture width
static const float texh = (float)PANEL2D_TEXH; // texture height
static const float tx_x0 = 1135.0;
static const float tx_y0 = texh-599.0f;
static const float tx_dx = 24.0f;
static const float tx_dy = 18.0f;
// constants for panel coordinates
static const float bb_x0 = 108.5f;
static const float bb_x1 = bb_x0+26.0f;
static const float bb_y0 = 471.5f;


// ==============================================================

ThrottleScram::ThrottleScram (VESSEL3 *v): PanelElement (v)
{
	dg = (DeltaGlider*)v;
	Reset2D();
}

// ==============================================================

void ThrottleScram::AddMeshData2D (MESHHANDLE hMesh, DWORD grpidx)
{
	static const DWORD NVTX = 8;
	static const DWORD NIDX = 12;
	static const NTVERTEX VTX[NVTX] = {
		// left slider
		{bb_x0,      bb_y0,      0,  0,0,0,  tx_x0/texw,        tx_y0/texh},
		{bb_x0+tx_dx,bb_y0,      0,  0,0,0,  (tx_x0+tx_dx)/texw,tx_y0/texh},
		{bb_x0,      bb_y0+tx_dy,0,  0,0,0,  tx_x0/texw,        (tx_y0+tx_dy)/texh},
		{bb_x0+tx_dx,bb_y0+tx_dy,0,  0,0,0,  (tx_x0+tx_dx)/texw,(tx_y0+tx_dy)/texh},
		// right slider
		{bb_x1,      bb_y0,      0,  0,0,0,  tx_x0/texw,        tx_y0/texh},
		{bb_x1+tx_dx,bb_y0,      0,  0,0,0,  (tx_x0+tx_dx)/texw,tx_y0/texh},
		{bb_x1,      bb_y0+tx_dy,0,  0,0,0,  tx_x0/texw,        (tx_y0+tx_dy)/texh},
		{bb_x1+tx_dx,bb_y0+tx_dy,0,  0,0,0,  (tx_x0+tx_dx)/texw,(tx_y0+tx_dy)/texh}
	};
	static const WORD IDX[NIDX] = {
		// left slider
		0,1,2, 3,2,1,
		// right slider
		4,5,6, 7,6,5
	};

	AddGeometry (hMesh, grpidx, VTX, NVTX, IDX, NIDX);
}

// ==============================================================

void ThrottleScram::Reset2D ()
{
	for (int i = 0; i < 2; i++) ppos[i] = 0.0f;
}

// ==============================================================

bool ThrottleScram::Redraw2D (SURFHANDLE surf)
{
	int i, j, vofs;
	float pos;
	static const float sy[4] = {bb_y0,bb_y0,bb_y0+tx_dy,bb_y0+tx_dy};

	for (i = 0; i < 2; i++) {
		double level = dg->GetThrusterLevel (dg->th_scram[i]);
		pos = (float)(-level*84.0);
		if (pos != ppos[i]) {
			vofs = vtxofs+i*4;
			for (j = 0; j < 4; j++) grp->Vtx[vofs+j].y = sy[j]+pos;
			ppos[i] = pos;
		}
	}
	return false;
}

// ==============================================================

bool ThrottleScram::ProcessMouse2D (int event, int mx, int my)
{
	static int ctrl = 0;
	if (event & PANEL_MOUSE_LBDOWN) { // record which slider to operate
		if      (mx <  12) ctrl = 0; // left engine
		else if (mx >= 37) ctrl = 1; // right engine
		else               ctrl = 2; // both
	}
	dg->SetScramLevel (ctrl, max (0.0, min (1.0, 1.0-my/84.0)));
	return true;
}