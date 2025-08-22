// ==============================================================
//                 ORBITER MODULE TEMPLATE
//                  Part of the ORBITER SDK
//           Copyright (C) 2002-2003 Martin Schweiger
//                   All rights reserved
//
// ==============================================================

#include <filesystem>
#include <string>
#define STRICT
#define ORBITER_MODULE
#include <windows.h>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <direct.h>
#include "..//..//include//Orbitersdk.h"
#include "..//..//include//MFDlib.h"
#include "FlightDataRecMFD.h"

// ==============================================================
// Global variables

const int ndata = 600;      // max samples

int paused = 1;
int auto_inc = 1;
double M = 0;
double R = 0;
float sample_dt;     // sample interval
float prev_v_mag = 0.0;
char delim_char = ' ';
std::string tgt_base;
std::filesystem::path logdir;
std::filesystem::path logfile;
std::filesystem::path logpath;
std::filesystem::path cfgpath;
std::filesystem::path curpath;

std::filesystem::path flightdatafolder("FlightData");
std::filesystem::path configfolder("Config");
std::filesystem::path configfilename("FDRMFD.cfg");

OBJHANDLE hbase = 0;
VECTOR3 b_pos;

static struct {  // "FlightDataRec MFD" parameters
	int mode;      // identifier for new MFD mode
} g_FlightDataRecMFD;

struct {  // global data storage
	double tnext;  // time of next sample
	int   sample;  // current sample index
	float *sim_time;   // sim time
	float *ves_alt;    // altitude
	float *ves_pitch;  // pitch
	float *ves_roll;   // roll 
	float *ves_yaw;    // yaw
	float *ves_v_rad;   // radial velocity
	float *ves_v_tan;   // tangential velocity (airspeed)
	float *ves_a_rad;   // radial acceleration (Vacc)
	float *ves_a_tan;   // tangential acceleration (Tacc)
	float *ves_a_g;     // net acceleration in gees
    float *ves_surf_lon;  // surface longitude
	float *ves_surf_lat;   // surface latitude
	float *ves_surf_hdg;    // surface heading
	float *ves_dist;   // distance to/from surface base
	float *ves_aoa;     // angle of attack
    float *ves_mach;    // Mach number
    float *ves_lift;    // Lift
    float *ves_drag;    // Drag
    float *atm_t;  // atmospheric temperature
	float *atm_stp;  // atmospheric pressure
	float *atm_dynp; // atmospheric dynamic pressure
	float *atm_d;  // atmospheric density
	float *eng_fuel_mass; // fuel mass for vessel's default propellant source (kg)
	float *eng_fuel_rate; // fuel mass flow rate for default propellant source (kg/s)
	float *eng_main_t;  // main engine thrust (%)
	float *eng_hover_t; // hover engine thrust (%)
} g_Data;

// Thanks Chris Knestrick! :)
// Thanks www.askdrmath.com! :-)
static inline double CalcSphericalDistance(VECTOR3 Pos1, VECTOR3 Pos2)
{
	double DeltaLat = Pos2.LAT - Pos1.LAT;
	double DeltaLong = Pos2.LONG - Pos1.LONG;

	double A = pow(sin(DeltaLat/2), 2) + cos(Pos1.LAT) * cos (Pos2.LAT) * pow(sin(DeltaLong/2), 2);
	double C = 2 * atan2(sqrt(A), sqrt(1 - A));
	
	return (R * C);
}


// ==============================================================
// API interface

void PurgeDataPoints(void);
void ReadConfig(void);
void WriteConfig(void);

DLLCLBK void opcDLLInit (HINSTANCE hDLL){

	static char *name = const_cast<char *>("Flight Data Recorder");
	MFDMODESPEC spec;

	spec.name    = name;
	spec.key     = OAPI_KEY_F;
	spec.msgproc = FlightDataRecMFD::MsgProc;

	sample_dt = 1.0; // default to 1 sample per second.
	g_Data.tnext  = 0.0;
	g_Data.sample = 0;
	g_Data.sim_time   = new float[ndata];   memset (g_Data.sim_time,  0, ndata*sizeof(float));
	g_Data.ves_alt    = new float[ndata];   memset (g_Data.ves_alt,   0, ndata*sizeof(float));
	g_Data.ves_pitch  = new float[ndata];   memset (g_Data.ves_pitch, 0, ndata*sizeof(float));
	g_Data.ves_roll  = new float[ndata];   memset (g_Data.ves_roll, 0, ndata*sizeof(float));
	g_Data.ves_yaw  = new float[ndata];   memset (g_Data.ves_yaw, 0, ndata*sizeof(float));
	g_Data.ves_v_rad   = new float[ndata];   memset (g_Data.ves_v_rad,  0, ndata*sizeof(float));
	g_Data.ves_v_tan   = new float[ndata];   memset (g_Data.ves_v_tan,  0, ndata*sizeof(float));
    g_Data.ves_a_rad  = new float[ndata];   memset (g_Data.ves_a_rad,  0, ndata*sizeof(float));
	g_Data.ves_a_tan  = new float[ndata];   memset (g_Data.ves_a_tan,  0, ndata*sizeof(float));
	g_Data.ves_a_g  = new float[ndata];   memset (g_Data.ves_a_tan,  0, ndata*sizeof(float));
	g_Data.ves_surf_lon   = new float[ndata];   memset (g_Data.ves_surf_lon,  0, ndata*sizeof(float));
	g_Data.ves_surf_lat   = new float[ndata];   memset (g_Data.ves_surf_lat,  0, ndata*sizeof(float));
	g_Data.ves_surf_hdg   = new float[ndata];   memset (g_Data.ves_surf_hdg,  0, ndata*sizeof(float));
	g_Data.ves_dist   = new float[ndata];   memset (g_Data.ves_dist,  0, ndata*sizeof(float));
	g_Data.ves_aoa   = new float[ndata];   memset (g_Data.ves_aoa,  0, ndata*sizeof(float));
	g_Data.ves_mach   = new float[ndata];   memset (g_Data.ves_mach,  0, ndata*sizeof(float));
	g_Data.ves_lift   = new float[ndata];   memset (g_Data.ves_lift,  0, ndata*sizeof(float));
	g_Data.ves_drag   = new float[ndata];   memset (g_Data.ves_drag,  0, ndata*sizeof(float));
	g_Data.atm_t   = new float[ndata];   memset (g_Data.atm_t,  0, ndata*sizeof(float));
	g_Data.atm_stp   = new float[ndata];   memset (g_Data.atm_stp,  0, ndata*sizeof(float));
	g_Data.atm_dynp   = new float[ndata];   memset (g_Data.atm_dynp,  0, ndata*sizeof(float));
	g_Data.atm_d   = new float[ndata];   memset (g_Data.atm_d,  0, ndata*sizeof(float));
	g_Data.eng_fuel_mass   = new float[ndata];   memset (g_Data.eng_fuel_mass,  0, ndata*sizeof(float));
	g_Data.eng_fuel_rate   = new float[ndata];   memset (g_Data.eng_fuel_rate,  0, ndata*sizeof(float));
	g_Data.eng_main_t   = new float[ndata];   memset (g_Data.eng_main_t,  0, ndata*sizeof(float));
	g_Data.eng_hover_t   = new float[ndata];   memset (g_Data.eng_hover_t,  0, ndata*sizeof(float));

	g_FlightDataRecMFD.mode = oapiRegisterMFDMode (spec);
	PurgeDataPoints();
	tgt_base[0] = '\0';

	try {
		// Carpeta raíz de Orbiter
		std::filesystem::path curpath = std::filesystem::current_path();

		// Carpeta "FlightData" dentro del directorio actual
		flightdatafolder = curpath / "FlightData";

		// Crear la carpeta si no existe
		std::filesystem::create_directories(flightdatafolder);

	} catch (const std::exception& e) {
		// Si algo falla, escribir al log de Orbiter
		oapiWriteLog(const_cast<char *>("Error creating FlightData folder"));
	}

	logdir = curpath / flightdatafolder;

	cfgpath = curpath / configfolder / configfilename;

	logfile = "flight-log-0000.dat";

	logpath = curpath / logdir / logfile;

	ReadConfig();
}

DLLCLBK void opcDLLExit (HINSTANCE hDLL)
{
	paused = 1;
	WriteConfig();
	oapiUnregisterMFDMode (g_FlightDataRecMFD.mode);
	delete []g_Data.sim_time;
	delete []g_Data.ves_alt;
	delete []g_Data.ves_pitch;
	delete []g_Data.ves_roll;
	delete []g_Data.ves_yaw;
	delete []g_Data.ves_v_rad;
	delete []g_Data.ves_v_tan;
	delete []g_Data.ves_a_rad;
	delete []g_Data.ves_a_tan;
	delete []g_Data.ves_surf_lon;
	delete []g_Data.ves_surf_lat;
	delete []g_Data.ves_surf_hdg;
	delete []g_Data.ves_dist;
	delete []g_Data.ves_aoa;
	delete []g_Data.ves_mach;
	delete []g_Data.ves_lift;
	delete []g_Data.ves_drag;
	delete []g_Data.atm_t;
	delete []g_Data.atm_stp;
	delete []g_Data.atm_dynp;
	delete []g_Data.atm_d;
	delete []g_Data.eng_fuel_mass;
	delete []g_Data.eng_fuel_rate;
	delete []g_Data.eng_main_t;
	delete []g_Data.eng_hover_t;

}

// We record vessel parameters outside the MFD to keep tracking
// even if the MFD mode doesn't exist

void log_data(void);

DLLCLBK void opcPreStep (double simt, double simdt, double mjd){
	
  if (!paused) {
	if (simt >= g_Data.tnext) {
		VESSEL *v = oapiGetFocusInterface();
		VESSELSTATUS v_stat;	
		VECTOR3 vel, pos, v_pos;
		ATMPARAM atm;
		OBJHANDLE ref;
		double a, r2, v2, vr2, vt2;
		double alt = v->GetAltitude();

		ref = v->GetSurfaceRef();
		M = oapiGetMass (ref);
		R = oapiGetSize (ref);
		v->GetStatus(v_stat);

		g_Data.sim_time[g_Data.sample]  = (float)simt;
			
		// grab vessel attitude samples
		g_Data.ves_alt[g_Data.sample]   = (float)(alt*1e-3);
		g_Data.ves_pitch[g_Data.sample] = (float)(v->GetPitch()*DEG);
		g_Data.ves_roll[g_Data.sample] = (float)(v->GetBank()*DEG);
		g_Data.ves_yaw[g_Data.sample] = (float)(v->GetSlipAngle()*DEG);

		// grab vessel velocity samples and calculate acceleration 
		v->GetRelativeVel (v->GetSurfaceRef(), vel);
		v->GetRelativePos (v->GetSurfaceRef(), pos);
		r2 = pos.x*pos.x + pos.y*pos.y + pos.z*pos.z;
		v2 = vel.x*vel.x + vel.y*vel.y + vel.z*vel.z;
		a  = (vel.x*pos.x + vel.y*pos.y + vel.z*pos.z) / r2;
		vr2 = a*a * r2;
		vt2 = v2 - vr2;
		g_Data.ves_v_rad[g_Data.sample] = (vr2 >= 0.0 ? a >= 0.0 ? (float)sqrt(vr2) : -(float)sqrt(vr2) : 0.0f);
		g_Data.ves_v_tan[g_Data.sample] = (vt2 >= 0.0 ? (float)sqrt(vt2) : 0.0f);
		if (g_Data.sample > 0) {
			g_Data.ves_a_rad[g_Data.sample] = (((g_Data.ves_v_rad[g_Data.sample]) - (g_Data.ves_v_rad[g_Data.sample-1]))/sample_dt);
			g_Data.ves_a_tan[g_Data.sample] = (((g_Data.ves_v_tan[g_Data.sample]) - (g_Data.ves_v_tan[g_Data.sample-1]))/sample_dt);
		}
		// ---------- G meter, somewhat agrees with Dan Polli's DG3 G meter -----------
// reentry use
		float v_mag, dp = 0.0;
			v_mag = (float) sqrt((vel.x*vel.x) + (vel.y*vel.y) + (vel.z*vel.z));
		if (prev_v_mag > 0) {
			dp = (float) (v_mag-prev_v_mag)/(float) G;
			dp = dp < 0 ? -dp : dp;
			g_Data.ves_a_g[g_Data.sample] = dp;
		}
		else g_Data.ves_a_g[g_Data.sample] = 0.0;
		prev_v_mag = v_mag;


		// grab vessel position samples
		if (hbase) {
			v->GetEquPos(v_pos.LONG, v_pos.LAT, v_pos.RADIUS);
			g_Data.ves_surf_lon[g_Data.sample] = (float)(v_pos.LONG*DEG);
			g_Data.ves_surf_lat[g_Data.sample] = (float)(v_pos.LAT*DEG);
			oapiGetFocusHeading(&a);
			g_Data.ves_surf_hdg[g_Data.sample] = (float)(a*DEG);
			g_Data.ves_dist[g_Data.sample] = (float)(CalcSphericalDistance(b_pos, v_pos)*1e-3); // distance in km
		}

		// angle of attack
		g_Data.ves_aoa[g_Data.sample] = (float)(v->GetAOA()*DEG);

		// mach
		g_Data.ves_mach[g_Data.sample] = (float)v->GetMachNumber();

		g_Data.ves_lift[g_Data.sample] = (float)v->GetLift();
		g_Data.ves_drag[g_Data.sample] = (float)v->GetDrag();


		// grab atmospheric samples
		oapiGetPlanetAtmParams(v_stat.rbody, R+alt, &atm);
		g_Data.atm_t[g_Data.sample] = (float)atm.T;
		g_Data.atm_stp[g_Data.sample] = (float)atm.p;
		g_Data.atm_dynp[g_Data.sample] = (float)v->GetDynPressure();
		g_Data.atm_d[g_Data.sample] = (float)atm.rho;
		
		//grab engine samples
		g_Data.eng_fuel_mass[g_Data.sample] = (float)v->GetTotalPropellantMass();
		g_Data.eng_fuel_rate[g_Data.sample] = (float)v->GetTotalPropellantFlowrate();
		g_Data.eng_main_t[g_Data.sample] = (float)(v->GetThrusterGroupLevel(THGROUP_MAIN))*100;
		g_Data.eng_hover_t[g_Data.sample] = (float)(v->GetThrusterGroupLevel(THGROUP_HOVER))*100;


		//  log data to file
		log_data();

		//sprintf(oapiDebugString(), "Tacc: %f   Vacc: %f", g_Data.ves_a_tan[g_Data.sample], g_Data.ves_a_rad[g_Data.sample]);

		// get ready for next sample period
		if (((g_Data.sample+1) % ndata) == 0) g_Data.sample = 0;
		else g_Data.sample = g_Data.sample+1;
		g_Data.tnext = simt + sample_dt;
	}

  }
}

DLLCLBK void opcOpenRenderViewport (HWND renderWnd, DWORD width, DWORD height, BOOL fullscreen)
{
	PurgeDataPoints();
	if (tgt_base[0] != '\0') { 
		VESSELSTATUS v_stat;	
		VESSEL *v = oapiGetFocusInterface();

		v->GetStatus(v_stat);
		hbase = oapiGetBaseByName(v_stat.rbody, const_cast<char *>(tgt_base.c_str()));
		if (hbase != NULL) {
			oapiGetBaseEquPos(hbase, &b_pos.LONG, &b_pos.LAT, &b_pos.RADIUS);
		} 
	}
}


// ==============================================================
// FlightDataRec MFD implementation

FlightDataRecMFD::FlightDataRecMFD (DWORD w, DWORD h, VESSEL *vessel)
: GraphMFD (w, h, vessel)
{
	int g;

	ref_alt = new float[ndata];
	ref_tvel = new float[ndata];
	ref = vessel->GetSurfaceRef();

	g = AddGraph ();
	SetAxisTitle (g, 0, const_cast<char *>("Vtan: m/s"));
	SetAxisTitle (g, 1, const_cast<char *>("Alt: km"));
	AddPlot (g, g_Data.ves_v_tan, g_Data.ves_alt, ndata, 1, &g_Data.sample);

	g = AddGraph ();
	SetAxisTitle (g, 0, const_cast<char *>("Vrad: m/s"));
	SetAxisTitle (g, 1, const_cast<char *>("Alt: km"));
	AddPlot (g, g_Data.ves_v_rad, g_Data.ves_alt, ndata, 1, &g_Data.sample);

	g = AddGraph ();
	SetAxisTitle (g, 0, const_cast<char *>("Time: s"));
	SetAxisTitle (g, 1, const_cast<char *>("Vacc: m/s^2"));
	AddPlot (g, g_Data.sim_time, g_Data.ves_a_rad, ndata, 1, &g_Data.sample);


	g = AddGraph ();
	SetAxisTitle (g, 0, const_cast<char *>("RTT: km"));
	SetAxisTitle (g, 1, const_cast<char *>("Alt: km"));
	AddPlot (g, g_Data.ves_dist, g_Data.ves_alt, ndata, 1, &g_Data.sample);

	g = AddGraph ();
	SetAxisTitle (g, 0, const_cast<char *>("RTT: km"));
	SetAxisTitle (g, 1, const_cast<char *>("Vtan: m/s"));
    AddPlot (g, g_Data.ves_dist, g_Data.ves_v_tan, ndata, 1, &g_Data.sample);

	g = AddGraph ();	
	SetAxisTitle (g, 0, const_cast<char *>("Time: s"));
	SetAxisTitle (g, 1, const_cast<char *>("Tacc: m/s^2"));
	AddPlot (g, g_Data.sim_time, g_Data.ves_a_tan, ndata, 1, &g_Data.sample);

	page = 0;
}

FlightDataRecMFD::~FlightDataRecMFD ()
{
	WriteConfig();
	delete []ref_alt;
	delete []ref_tvel;
}

void FlightDataRecMFD::InitReferences (void)
{
	const double G = 6.67259e-11;
	M = oapiGetMass (ref);
	R = oapiGetSize (ref);
	double f0 = graph[0].data_min;
	double f1 = (graph[0].data_max - graph[0].data_min)/(double)(ndata-1);
	double f2 = sqrt (G*M)*1e-3;
	int i;
	for (i = 0; i < ndata; i++) {
		ref_alt[i]  = (float)(f0 + i * f1);
		ref_tvel[i] = (float)(f2/sqrt(ref_alt[i]*1e3+R));
	}	
}

// message parser
OAPI_MSGTYPE FlightDataRecMFD::MsgProc (UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam)
{
	switch (msg) {
	case OAPI_MSG_MFD_OPENED:
		return (OAPI_MSGTYPE)(new FlightDataRecMFD (LOWORD(wparam), HIWORD(wparam), (VESSEL*)lparam));
	}
	return 0;
}

bool FlightDataRecMFD::ConsumeKeyBuffered (DWORD key)
{
	bool DelimInput (void *id, char *str, void *data);
	bool BaseInput (void *id, char *str, void *data);
	bool RateInput (void *id, char *str, void *data);
	bool PathInput (void *id, char *str, void *data);
	bool FileInput (void *id, char *str, void *data);
	void IncrementFileCounter(void);

	switch (key) {
	case OAPI_KEY_A:
		if (paused) paused = 0;
		else { paused = 1; if (auto_inc) IncrementFileCounter(); }
		return true;
	case OAPI_KEY_P:
		page = (page+1) % 2;
		return true;
    case OAPI_KEY_T:
		oapiOpenInputBox (const_cast<char *>("Target Base:"), BaseInput, 0, 20, (void*)this);
		return true;
	case OAPI_KEY_D:
		oapiOpenInputBox (const_cast<char *>("Delimiter character:"), DelimInput, 0, 20, (void*)this);
		return true;
	case OAPI_KEY_U:
		PurgeDataPoints();
	case OAPI_KEY_R:
		oapiOpenInputBox (const_cast<char *>("Samples per second:"), RateInput, 0, 20, (void*)this);
		return true;
	case OAPI_KEY_H:
		oapiOpenInputBox (const_cast<char *>("Data Path:"), PathInput, 0, 20, (void*)this);
		return true;
	case OAPI_KEY_F:
		oapiOpenInputBox (const_cast<char *>("Data file name:"), FileInput, 0, 20, (void*)this);
		return true;
	case OAPI_KEY_I:
		auto_inc = (auto_inc+1) % 2;
		return true;
	}
	return false;
}

bool FlightDataRecMFD::ConsumeButton (int bt, int event)
{
	if (!(event & PANEL_MOUSE_LBDOWN)) return false;
	static const DWORD btkey[9] = { OAPI_KEY_T, OAPI_KEY_D, OAPI_KEY_A, OAPI_KEY_P, OAPI_KEY_U,
									OAPI_KEY_R, OAPI_KEY_H, OAPI_KEY_F, OAPI_KEY_I};
	if (bt < 9) return ConsumeKeyBuffered (btkey[bt]);
	else return false;
}

char *FlightDataRecMFD::ButtonLabel (int bt)
{
	static const char *label[12] = {"TGT", "DLM", "DA", "DIS", "PUR", "RAT", "PTH", "FLE", "", "", "", "INC"};
	return (bt < 12 ? const_cast<char *>(label[bt]) : 0);
}

int FlightDataRecMFD::ButtonMenu (const MFDBUTTONMENU **menu) const
{
	static const MFDBUTTONMENU mnu[12] = {
		{"select Target base", 0, 'T'},
		{"specify Delimiter", 0, 'D'},
		{"data Acquisition toggle", 0, 'A'},
		{"select display Page", 0, 'P'},
		{"pUrge plot data", 0, 'U'},
		{"sample Rate", 0, 'R'},
		{"data patH", 0, 'H'},
		{"data File name", 0, 'F'},
		{"", 0, '\0'},
		{"", 0, '\0'},
		{"", 0, '\0'},
		{"auto Increment toggle", 0, 'I'},
	};
	if (menu) *menu = mnu;
	return 12;
}

void FlightDataRecMFD::Update (HDC hDC)
{
	char title[255];
	char rng_target[255];
	float altmin, altmax, tmp;

	MFDUpdate(hDC, cw, ch);
	title[0] = '\0';
    strcpy(title, "Flight Data Recorder");
	Title (hDC, title);

  if (!paused) {
	FindRange (g_Data.ves_alt, ndata, altmin, altmax);
	if (altmin > altmax)
		tmp = altmin, altmin = altmax, altmax = tmp;

	if (altmin == altmax)
		altmin -= 0.5, altmax += 0.5;
		SetRange (0, 1, altmin, altmax);
		SetRange (1, 1, altmin, altmax);
		SetRange (3, 1, altmin, altmax);

		InitReferences();

		SetAutoRange (0, 0, 0); // Vel
		SetAutoRange (1, 0, 0); // Vel
		SetAutoRange (2, 0, 0); // Time
		SetAutoRange (2, 1, 0); // Vacc
		SetAutoRange (3, 0); // RTT
		SetAutoRange (4, 0); // RTT
		SetAutoRange (4, 1, 0); // Vel
		SetAutoRange (5, 0, 0); // Time
		SetAutoRange (5, 1, 0); // Vacc

		switch (page) {
			case 0:
				TextXY(hDC, 30, 0, WHITE, BLACK, "PG0");
				Plot (hDC, 0, ch, (H+ch)/3, "Vtan/Alt");
				Plot (hDC, 1, ((H+ch)/3), ((H+ch)/3)*2, "Vrad/Alt");
				Plot (hDC, 2, ((H+ch)/3)*2, H, "Vert acc");
				break;
			case 1:
				TextXY(hDC, 30, 0, WHITE, BLACK, "PG1");
				Plot (hDC, 3, ch, (H+ch)/3, "Alt/Range");
				Plot (hDC, 4, (H+ch)/3, ((H+ch)/3)*2, "Vtan/Range");
				Plot (hDC, 5, ((H+ch)/3)*2, H, "Tan acc");
				break;
		}
	} else {
		strcpy(rng_target, "TGT BASE: ");
		if (tgt_base[0] == '\0') strcat(rng_target, " !!  NONE  !!");
		else strcat(rng_target, tgt_base.c_str());
		TextXY(hDC, 0, 16, YELLOW, BLACK, "Rate: %.3f", (1/sample_dt));
		TextXY(hDC, 13, 16, YELLOW, BLACK, "samples/sec");
		
		TextXY(hDC, 7, 12, RED, BLACK, "DATA ACQUISITION PAUSED");
		
		TextXY(hDC, 0, 8, YELLOW, BLACK, rng_target);

		TextXY(hDC, 0, 5, YELLOW, BLACK, "Delimiter: '");
		TextXY(hDC, 12, 5, YELLOW, BLACK, "%c'", delim_char);

		TextXY(hDC, 0, 6, YELLOW, BLACK, "Auto Inc. Filename:");
		switch (auto_inc) {
			case 0:
				TextXY(hDC, 20, 6, YELLOW, BLACK, "OFF");
				break;
			case 1:
				TextXY(hDC, 20, 6, YELLOW, BLACK, "ON");
				break;
		}
		TextXY(hDC, 0, 3, YELLOW, BLACK, "Log File:");
		TextXY(hDC, 10, 3, YELLOW, BLACK, logfile.string().c_str());
		
		TextXY(hDC, 0, 2, YELLOW, BLACK, "Log Dir:");
		TextXY(hDC, 10, 2, YELLOW, BLACK, logdir.string().c_str());

	}

}

bool FlightDataRecMFD::SetAltRange (char *rstr)
{
	float altmin, altmax;

	if (rstr[0] == 'a' || rstr[0] == 'A') {
		alt_auto = true;
		return true;
	} else if (sscanf (rstr, "%f%f", &altmin, &altmax) == 2 && altmin < altmax) {
		alt_auto = false;
		SetRange (0, 1, altmin, altmax);
		SetRange (1, 1, altmin, altmax);
		SetRange (3, 1, altmin, altmax);
		InitReferences();
		return true;
	}
	return false;
}

bool FlightDataRecMFD::SetVradRange (char *rstr)
{
	float rmin, rmax;
	if (rstr[0] == 'a' || rstr[0] == 'A') {
		vrad_auto = true;
		return true;
	} else if (sscanf (rstr, "%f%f", &rmin, &rmax) == 2 && rmin < rmax) {
		vrad_auto = false;
		SetRange (2, 0, rmin, rmax);
		SetRange (1, 0, rmin, rmax);
		return true;
	}
	return false;
}

bool FlightDataRecMFD::SetBase (const std::string rstr)
{
	VESSELSTATUS v_stat;	

	VESSEL *v = oapiGetFocusInterface();

	v->GetStatus(v_stat);
	
	hbase = oapiGetBaseByName(v_stat.rbody, const_cast<char *>(rstr.c_str()));

    if(hbase != NULL){
		oapiGetBaseEquPos(hbase, &b_pos.LONG, &b_pos.LAT, &b_pos.RADIUS);
		return true;
	} else {
		return false;
	}
}

bool FlightDataRecMFD::SetVtanRange (char *rstr)
{
	float rmin, rmax;
	if (rstr[0] == 'a' || rstr[0] == 'A') {
		vtan_auto = true;
		return true;
	} else if (sscanf (rstr, "%f%f", &rmin, &rmax) == 2 && rmin < rmax) {
		vtan_auto = false;
		SetRange (0, 0, rmin, rmax);
		SetRange (2, 1, rmin, rmax);
		return true;
	}
	return false;
}


void IncrementFileCounter(void){
    // obtener el nombre de archivo (ejemplo: "log-0001.txt")
    std::string filename = logfile.filename().string();

    // encontrar el punto (extensión)
    auto dotPos = filename.find_last_of('.');
    if (dotPos == std::string::npos) {
        dotPos = filename.size(); // no tiene extensión
    }

    // buscar la parte numérica antes del punto
    int pos = static_cast<int>(dotPos) - 1;
    while (pos >= 0 && std::isdigit(static_cast<unsigned char>(filename[pos]))) {
        --pos;
    }

    pos++; // ahora apunta al primer dígito
    if (pos < dotPos) {
        std::string numberPart = filename.substr(pos, dotPos - pos);
        int number = std::stoi(numberPart);

        // incrementar el número
        number++;

        // mantener el mismo ancho con ceros a la izquierda
        std::string newNumber = std::to_string(number);
        if (newNumber.size() < numberPart.size()) {
            newNumber.insert(0, numberPart.size() - newNumber.size(), '0');
        }

        // reconstruir el nuevo nombre
        filename.replace(pos, dotPos - pos, newNumber);
    }

    // asignar el nuevo nombre al path
    logfile = filename;
    logpath = logdir / logfile; // logdir + logfile
}

void PurgeDataPoints(void)
{
	int remain_paused = 1;

	if (paused == 1){
		remain_paused = 1;
	} else { 
		paused == 1; remain_paused = 0;
	}

	g_Data.tnext  = 0.0;
	g_Data.sample = 0;
	memset (g_Data.ves_alt,   0, ndata*sizeof(float));
	memset (g_Data.ves_pitch, 0, ndata*sizeof(float));
	memset (g_Data.ves_roll, 0, ndata*sizeof(float));
	memset (g_Data.ves_yaw, 0, ndata*sizeof(float));
	memset (g_Data.ves_v_rad,  0, ndata*sizeof(float));
	memset (g_Data.ves_v_tan,  0, ndata*sizeof(float));
	memset (g_Data.ves_a_rad,  0, ndata*sizeof(float));
	memset (g_Data.ves_a_tan,  0, ndata*sizeof(float));
	memset (g_Data.ves_surf_lon,  0, ndata*sizeof(float));
	memset (g_Data.ves_surf_lat,  0, ndata*sizeof(float));
	memset (g_Data.ves_surf_hdg,  0, ndata*sizeof(float));
	memset (g_Data.ves_dist,  0, ndata*sizeof(float));
	memset (g_Data.ves_aoa,  0, ndata*sizeof(float));
	memset (g_Data.ves_mach,  0, ndata*sizeof(float));
	memset (g_Data.ves_lift,  0, ndata*sizeof(float));
	memset (g_Data.ves_drag,  0, ndata*sizeof(float));
	memset (g_Data.atm_t,  0, ndata*sizeof(float));
	memset (g_Data.atm_stp,  0, ndata*sizeof(float));
	memset (g_Data.atm_dynp,  0, ndata*sizeof(float));
	memset (g_Data.atm_d,  0, ndata*sizeof(float));
	memset (g_Data.eng_fuel_mass,  0, ndata*sizeof(float));
	memset (g_Data.eng_fuel_rate,  0, ndata*sizeof(float));
	memset (g_Data.eng_main_t,  0, ndata*sizeof(float));
	memset (g_Data.eng_hover_t,  0, ndata*sizeof(float));
    
	paused = remain_paused;
}

void log_data(void){

	std::ofstream out_file;
	
	out_file.open(logpath, std::ios::app);

	if (out_file.is_open()){

		out_file << g_Data.sample << delim_char;
		out_file << g_Data.sim_time[g_Data.sample] << delim_char;
		out_file << g_Data.ves_alt[g_Data.sample] << delim_char;
		out_file << g_Data.ves_pitch[g_Data.sample] << delim_char;
		out_file << g_Data.ves_roll[g_Data.sample] << delim_char;
		out_file << g_Data.ves_yaw[g_Data.sample] << delim_char;
		out_file << g_Data.ves_v_rad[g_Data.sample] << delim_char;
		out_file << g_Data.ves_v_tan[g_Data.sample] << delim_char;
		out_file << g_Data.ves_a_rad[g_Data.sample] << delim_char;
		out_file << g_Data.ves_a_tan[g_Data.sample] << delim_char;
		out_file << g_Data.ves_a_g[g_Data.sample] << delim_char;
		out_file << g_Data.ves_surf_lon[g_Data.sample] << delim_char;
		out_file << g_Data.ves_surf_lat[g_Data.sample] << delim_char;
		out_file << g_Data.ves_surf_hdg[g_Data.sample] << delim_char;
		out_file << g_Data.ves_dist[g_Data.sample] << delim_char;
		out_file << g_Data.ves_aoa[g_Data.sample] << delim_char;
		out_file << g_Data.ves_mach[g_Data.sample] << delim_char;
		out_file << g_Data.ves_lift[g_Data.sample] << delim_char;
		out_file << g_Data.ves_drag[g_Data.sample] << delim_char;
		out_file << g_Data.atm_t[g_Data.sample] << delim_char;
		out_file << g_Data.atm_stp[g_Data.sample] << delim_char;
		out_file << g_Data.atm_dynp[g_Data.sample] << delim_char;
		out_file << g_Data.atm_d[g_Data.sample] << delim_char;
		out_file << g_Data.eng_fuel_mass[g_Data.sample] << delim_char;
		out_file << g_Data.eng_fuel_rate[g_Data.sample] << delim_char;
		out_file << g_Data.eng_main_t[g_Data.sample] << delim_char;
		out_file << g_Data.eng_hover_t[g_Data.sample];
		out_file << std::endl;
	}

}


bool DelimInput (void *id, char *str, void *data)
{
	delim_char = str[0];
	return true;
}

bool BaseInput (void *id, char *str, void *data){

	tgt_base = str;

	if(!((FlightDataRecMFD*)data)->SetBase (str)){
		tgt_base.clear();
		return false;
	}

	return true;
}

bool RateInput (void *id, char *str, void *data){
	
	float rate;

	rate = std::stof(str);

	if(rate == 1){
		sample_dt = (1/rate);
	}
	
	return true;
}

std::string strip_quotes(const std::string& s) {
    if (s.size() >= 2 && 
        ((s.front() == '"' && s.back() == '"') || 
         (s.front() == '\'' && s.back() == '\''))) {
        return s.substr(1, s.size() - 2);
    }
    return s;
}

bool PathInput(void* id, char *str, void* data) {
    namespace fs = std::filesystem;

    // 1. Copiar y limpiar comillas
    std::string raw = str;
    std::string clean = strip_quotes(raw);

    // 2. Construir ruta final usando filesystem
    fs::path base(clean);
    fs::path full = base / logfile;   // aquí no importa si "base" termina en \ o no

    // 3. Guardar en globales
    ::logdir  = base.string();
    ::logpath = full.string();

    return true;
}

bool FileInput(void *id, char *str, void *data) {
    namespace fs = std::filesystem;

    std::string clean = str;

    // eliminar comillas sobrantes
    if (!clean.empty() && clean.front() == '"' && clean.back() == '"') {
        clean = clean.substr(1, clean.size() - 2);
    }

    logfile = clean;                // guardamos limpio
    logpath = fs::path(logdir) / logfile;
    auto_inc = 0;

    return true;
}

void WriteConfig() {

    std::ofstream out_file(cfgpath);
    if (!out_file) {
        throw std::runtime_error("No se pudo abrir el archivo de configuración: " + cfgpath.string());
    }

    out_file << "AUTOINC "  << auto_inc   << '\n'
             << "SAMPLEDT " << sample_dt  << '\n';

    if (delim_char != ' ') {
        out_file << "DELIM " << std::string(1, delim_char) << '\n';
    }

    out_file << "TGTBASE " << tgt_base  << '\n'
             << "LOGDIR "  << logdir.string()    << '\n'
             << "LOGFILE " << logfile.string()   << '\n'
             << "PAUSED "  << paused    << '\n';
}

void ReadConfig() {
    namespace fs = std::filesystem;

    std::ifstream in_file(cfgpath);
    if (!in_file) {
        return; // no existe config → usar valores por defecto
    }

    std::string key;
    std::string value;
    std::string line;

    while (std::getline(in_file, line)) {
        if (line.empty() || line[0] == '#') continue; // ignorar vacías o comentarios

        std::istringstream iss(line);
        if (!(iss >> key)) continue;
        std::getline(iss, value);
        if (!value.empty() && value.front() == ' ') value.erase(0, 1);

        if (key == "PAUSED") {
            try { paused = std::stoi(value); } catch (...) {}
        } else if (key == "AUTOINC") {
            try { auto_inc = std::stoi(value); } catch (...) {}
        } else if (key == "SAMPLEDT") {
            try { sample_dt = std::stof(value); } catch (...) {}
        } else if (key == "DELIM") {
            if (!value.empty()) delim_char = value[0];
        } else if (key == "TGTBASE") {
            tgt_base = value;
        } else if (key == "LOGDIR") {
            logdir = fs::path(value);
        } else if (key == "LOGFILE") {
            logfile = fs::path(value);
        }
    }

    logpath = logdir / logfile;
}


FlightDataRecMFD::SavePrm FlightDataRecMFD::saveprm = {0, 0, 0.0, '\0', '\0', '\0', '\0'};
