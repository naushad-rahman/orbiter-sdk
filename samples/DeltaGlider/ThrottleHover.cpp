// ==============================================================
//                ORBITER MODULE: DeltaGlider
//                  Part of the ORBITER SDK
//          Copyright (C) 2001-2008 Martin Schweiger
//                   All rights reserved
//
// ThrottleHover.cpp
// Hover throttle sliders
// ==============================================================

#define STRICT 1
#include "ThrottleHover.h"
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
static const float bb_dx =  50.0f;
static const float bb_y0 = 358.5f;


// ==============================================================

ThrottleHover::ThrottleHover (VESSEL3 *v): PanelElement (v)
{
	dg = (DeltaGlider*)v;
	Reset2D();
}

// ==============================================================

void ThrottleHover::AddMeshData2D (MESHHANDLE hMesh, DWORD grpidx)
{
	static const DWORD NVTX = 4;
	static const DWORD NIDX = 6;
	static const NTVERTEX VTX[NVTX] = {
		// hover slider
		{bb_x0,      bb_y0,      0,  0,0,0,  tx_x0/texw,        tx_y0/texh},
		{bb_x0+bb_dx,bb_y0,      0,  0,0,0,  (tx_x0+tx_dx)/texw,tx_y0/texh},
		{bb_x0,      bb_y0+tx_dy,0,  0,0,0,  tx_x0/texw,        (tx_y0+tx_dy)/texh},
		{bb_x0+bb_dx,bb_y0+tx_dy,0,  0,0,0,  (tx_x0+tx_dx)/texw,(tx_y0+tx_dy)/texh}
	};
	static const WORD IDX[NIDX] = {
		0,1,2, 3,2,1
	};

	AddGeometry (hMesh, grpidx, VTX, NVTX, IDX, NIDX);
}

// ==============================================================

void ThrottleHover::Reset2D ()
{
	ppos = 0.0f;
}

// ==============================================================

bool ThrottleHover::Redraw2D (SURFHANDLE surf)
{
	int j;
	float pos;
	static const float sy[4] = {bb_y0,bb_y0,bb_y0+tx_dy,bb_y0+tx_dy};

	double level = dg->GetThrusterLevel (dg->th_hover[0]);
	pos = (float)(-level*116.0);
	if (pos != ppos) {
		for (j = 0; j < 4; j++) grp->Vtx[vtxofs+j].y = sy[j]+pos;
		ppos = pos;
	}
	return false;
}

// ==============================================================

bool ThrottleHover::ProcessMouse2D (int event, int mx, int my)
{
	my = max (0, min (116, my-9));
	dg->SetThrusterGroupLevel (dg->thg_hover, 1.0-my/116.0);
	return true;
}
