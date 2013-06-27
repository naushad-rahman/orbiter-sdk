// ==============================================================
//                ORBITER MODULE: DeltaGlider
//                  Part of the ORBITER SDK
//          Copyright (C) 2001-2003 Martin Schweiger
//                   All rights reserved
//
// Ramjet.cpp
// Implementation for the delta glider ramjet engine
// ==============================================================

#include "Ramjet.h"

// constructor
Ramjet::Ramjet (VESSEL *_vessel): vessel(_vessel)
{
	nthdef = 0;    // no thrusters associated yet
}

// destructor
Ramjet::~Ramjet ()
{
	if (nthdef) {  // delete list of thruster definitions
		for (UINT i = 0; i < nthdef; i++)
			delete thdef[i];
		delete []thdef;
	}
}

// add new thruster definition to list
void Ramjet::AddThrusterDefinition (THRUSTER_HANDLE th,
	double Qr, double Ai, double Tb_max, double dmf_max)
{
	THDEF *thd   = new THDEF;
	thd->th      = th;
	thd->Qr      = Qr;
	thd->Ai      = Ai;
	thd->Tb_max  = Tb_max;
	thd->dmf_max = dmf_max;
	thd->dmf     = 0.0;
	thd->F       = 0.0;
	for (int i = 0; i < 3; i++) thd->T[i] = 0.0;

	THDEF **tmp = new THDEF*[nthdef+1];
	if (nthdef) {
		memcpy (tmp, thdef, nthdef*sizeof (THDEF*));
		delete []thdef;
	}
	thdef = tmp;
	thdef[nthdef++] = thd;
}

// calculate current thrust force for all engines
void Ramjet::Thrust (double *F) const
{
	const OBJHANDLE hBody = vessel->GetAtmRef();
	const ATMCONST *atm = (hBody ? oapiGetPlanetAtmConstants (hBody) : 0);

	if (atm) { // atmospheric parameters available
		
		double M, Fs, T0, Td, Tb, Tb0, Te, p0, pd, D, rho, cp, v0, ve, tr, lvl, dma, dmf, precov, dmafac;
		const double eps = 1e-4;
		const double dma_scale = 2.7e-4;

		M   = vessel->GetMachNumber();                     // Mach number
		T0  = vessel->GetAtmTemperature();                 // freestream temperature
		p0  = vessel->GetAtmPressure();                    // freestream pressure
		rho = vessel->GetAtmDensity();                     // freestream density
		cp  = atm->gamma * atm->R / (atm->gamma-1.0);      // specific heat (pressure)
		v0  = M * sqrt (atm->gamma * atm->R * T0);         // freestream velocity
		tr  = (1.0 + 0.5*(atm->gamma-1.0) * M*M);          // temperature ratio
		Td  = T0 * tr;                                     // diffuser temperature
		pd  = p0 * pow (Td/T0, atm->gamma/(atm->gamma-1.0)); // diffuser pressure
		precov = max (0.0, 1.0-0.075*pow (max(M,1.0)-1.0, 1.35)); // pressure recovery
		dmafac = dma_scale*precov*pd;

		for (UINT i = 0; i < nthdef; i++) {
			Tb0 = thdef[i]->Tb_max;                        // max burner temperature
			if (Tb0 > Td) {                                // we are within operational range
				lvl  = vessel->GetThrusterLevel (thdef[i]->th); // throttle level
				D    = (Tb0-Td) / (thdef[i]->Qr/cp - Tb0); // max fuel-to-air ratio (what if negative?)
				D   *= lvl;                                // actual fuel-to-air ratio

				dma = dmafac * thdef[i]->Ai;               // air mass flow rate [kg/s]
				//dma  = rho * v0 * thdef[i]->Ai;            // air mass flow rate
				dmf  = D * dma;                            // fuel mass flow rate
				if (dmf > thdef[i]->dmf_max) {             // max fuel rate exceeded
					dmf = thdef[i]->dmf_max;
					D = dmf/dma;
				}
				Tb   = (D*thdef[i]->Qr/cp + Td) / (1.0+D); // actual burner temperature
				Te   = Tb * pow (p0/pd, (atm->gamma-1.0)/atm->gamma); // exhaust temperature
				ve   = sqrt (2.0*cp*(Tb-Te));              // exhaust velocity
			    Fs  = (1.0+D)*ve - v0;                     // specific thrust
				thdef[i]->F = F[i] = max (0.0, Fs*dma);    // thrust force
				thdef[i]->dmf = dmf;
				thdef[i]->T[1] = Tb;
				thdef[i]->T[2] = Te;

			} else {                                       // overheating!

				thdef[i]->F = F[i] = 0.0;
				thdef[i]->dmf = 0.0;
				thdef[i]->T[1] = thdef[i]->T[2] = Td;

			}
			thdef[i]->T[0] = Td;
		}

	} else {   // no atmospheric parameters

		for (UINT i = 0; i < nthdef; i++) {
			thdef[i]->F = F[i] = 0.0;
			thdef[i]->dmf = 0.0;
		}

	}
}

double Ramjet::TSFC (UINT idx) const
{
	const double eps = 1e-5;
	return thdef[idx]->dmf/(thdef[idx]->F+eps);
}
