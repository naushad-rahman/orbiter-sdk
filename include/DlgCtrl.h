#ifndef __DLGCTRL_H
#define __DLGCTRL_H

#define STRICT 1
#include "windows.h"

void oapiRegisterCustomControls (HINSTANCE hInst);
void oapiUnregisterCustomControls (HINSTANCE hInst);

long FAR PASCAL MsgProc_Gauge (HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
long FAR PASCAL MsgProc_Switch (HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

struct GAUGEPARAM {
	int rangemin, rangemax;
	enum GAUGEBASE { LEFT, RIGHT, TOP, BOTTOM } base;
	enum GAUGECOLOR { BLACK, RED } color;
};

void oapiSetGaugeParams (HWND hCtrl, GAUGEPARAM *gp, bool redraw = true);
void oapiSetGaugeRange (HWND hCtrl, int rmin, int rmax, bool redraw = true);
int  oapiSetGaugePos (HWND hCtrl, int pos, bool redraw = true);
int  oapiIncGaugePos (HWND hCtrl, int dpos, bool redraw = true);
int  oapiGetGaugePos (HWND hCtrl);

struct SWITCHPARAM {
	enum SWITCHMODE { TWOSTATE, THREESTATE } mode;
	enum ORIENTATION { HORIZONTAL, VERTICAL } align;
};

void oapiSetSwitchParams (HWND hCtrl, SWITCHPARAM *sp, bool redraw);
int oapiSetSwitchState (HWND hCtrl, int state, bool redraw);
int oapiGetSwitchState (HWND hCtrl);

#endif // !__DLGCTRL_H
