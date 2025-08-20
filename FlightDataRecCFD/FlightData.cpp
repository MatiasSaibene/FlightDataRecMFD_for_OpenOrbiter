// ==============================================================
//                 ORBITER MODULE: FlightData
//                  Part of the ORBITER SDK
//            Copyright (C) 2003 Martin Schweiger
//                   All rights reserved
//
// FlightData.cpp
// Dialog box for displaying vessel flight data during the
// simulation.
// ==============================================================

#define STRICT
#define ORBITER_MODULE
#include <stdio.h>
#include <windows.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <direct.h>
#include <commdlg.h>
#include "..//..//include//Orbitersdk.h"
#include "resource.h"
#include "FDGraph.h"

#define NGRAPH 22
#define NRATE 4

#define LONG x
#define LAT y
#define RADIUS z

#define HOR x
#define VERT y
#define LATERAL y
#define CONFIGFILE "FDRCFD.cfg"

// ==============================================================
// Global variables

HINSTANCE g_hInst;          // module instance handle
HWND g_hDlg;                // dialog handle
DWORD g_dwCmd;              // custom function identifier
DWORD g_nGraph = 0;         // number of displayed graphs
FlightDataGraph **g_Graph;  // list of graphs currently displayed
VESSEL *g_VESSEL = 0;       // current reference vessel
OBJHANDLE g_BASE = 0;       // current reference base 
double g_T = 0.0;           // sample time
bool g_bRecording;          // recorder on/off

double M = 0;
double R = 0;
float g_DT;     // sample interval
char delim_char = ' ';
int tgt_base = 0;
char range_target[255];
char logdir[_MAX_DIR+_MAX_DRIVE];
char logfile[_MAX_FNAME];
char logpath[_MAX_PATH];
char orbiterpath[_MAX_PATH];

char curdrive = 0;
char curpath[_MAX_PATH];

OBJHANDLE hbase = 0;
VECTOR3 b_pos;

static char *desc = const_cast<char *>("Open a window to track and record flight parameters of a spacecraft.");

struct {  // global data storage
	double tnext;  // time of next sample
	int   sample;  // current sample index
	float sim_time;   // sim time
	float ves_alt;    // altitude
	float ves_pitch;  // pitch
	float ves_roll;   // roll 
	float ves_yaw;    // yaw
	float ves_v_rad;   // radial velocity
	float ves_v_tan;   // tangential velocity (airspeed)
	float prev_ves_v_rad; // previous radial velocity sample
	float prev_ves_v_tan; // previous tangential velocity sample
	float ves_a_rad;   // radial acceleration (Vacc)
	float ves_a_tan;   // tangential acceleration (Tacc)
	float ves_a_g;   // net acceleration (G)
    float ves_surf_lon;  // surface longitude
	float ves_surf_lat;   // surface latitude
	float ves_surf_hdg;    // surface heading
	float ves_dist;   // distance to/from surface base
	float ves_aoa;     // angle of attack
    float ves_mach;    // Mach number
	float ves_lift;	// lift
	float ves_drag; // drag
    float atm_t;  // atmospheric temperature
	float atm_stp;  // atmospheric pressure
	float atm_dynp; // atmospheric dynamic pressure
	float atm_d;  // atmospheric density
	float eng_fuel_mass; // fuel mass for vessel's default propellant source (kg)
	float eng_fuel_rate; // fuel mass flow rate for default propellant source (kg/s)
	float eng_main_t;  // main engine thrust (%)
	float eng_hover_t; // hover engine thrust (%)
} g_Data;


// ==============================================================
// Local prototypes

void OpenDlgClbk (void *context);
OAPI_MSGTYPE CALLBACK MsgProc (HWND, UINT, WPARAM, LPARAM);
LRESULT CALLBACK Graph_WndProc (HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
void ReadConfig(void);
void WriteConfig(void);
void LogData(void);
void GetSamples(double simt);

// Thanks Chris Knestrick! ;)
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

DLLCLBK void opcDLLInit (HINSTANCE hDLL)
{
	g_hInst = hDLL;
	g_hDlg = 0;
	g_dwCmd = oapiRegisterCustomCmd (const_cast<char *>("Flight Data Recorder"), desc, OpenDlgClbk, NULL);
	// adds a new command into Orbiter's custom function list

	// register the window class for the data graph
	WNDCLASS wndClass;
	wndClass.style       = CS_HREDRAW | CS_VREDRAW;
	wndClass.lpfnWndProc = Graph_WndProc;
	wndClass.cbClsExtra    = 0;
	wndClass.cbWndExtra    = 0;
	wndClass.hInstance     = hDLL;
	wndClass.hIcon         = NULL;
	wndClass.hCursor       = LoadCursor (NULL, IDC_CROSS);
	wndClass.hbrBackground = (HBRUSH)GetStockObject (WHITE_BRUSH);
	wndClass.lpszMenuName  = NULL;
	wndClass.lpszClassName = "GraphWindow";
	RegisterClass (&wndClass);

	Graph::InitGDI();
	g_DT = 1.0; // default to 1 sample per second.
	g_Data.sample = 0;
	g_Data.sim_time   = 0;
	g_Data.ves_alt    = 0;
	g_Data.ves_pitch  = 0;
	g_Data.ves_roll  = 0;
	g_Data.ves_yaw  = 0;
	g_Data.ves_v_rad   = 0;
	g_Data.ves_v_tan   = 0;
	g_Data.prev_ves_v_rad  = 0;
	g_Data.prev_ves_v_tan   = 0;
    g_Data.ves_a_rad  = 0;
    g_Data.ves_a_tan  = 0;
	g_Data.ves_a_g = 0;
	g_Data.ves_surf_lon   = 0;
	g_Data.ves_surf_lat   = 0;
	g_Data.ves_surf_hdg   = 0;
	g_Data.ves_dist   = 0;
	g_Data.ves_aoa   = 0;
	g_Data.ves_mach   = 0;
	g_Data.atm_t   = 0;
	g_Data.atm_stp   = 0;
	g_Data.atm_dynp   = 0;
	g_Data.atm_d   = 0;
	g_Data.eng_fuel_mass   = 0;
	g_Data.eng_fuel_rate   = 0;
	g_Data.eng_main_t   = 0;
	g_Data.eng_hover_t   = 0;

	range_target[0] = '\0';
	curdrive = _getdrive();
	if( _getdcwd(curdrive, curpath, _MAX_PATH ) != NULL ) {
		strcpy(logdir, curpath);
		strcat(logdir, "\\");
		strcpy(orbiterpath, logdir);
	}
	else strcpy(logdir, "C:\\orbiter\\FlightData\\");
	strcpy(logfile, "flight-log-0000.dat");
	strcpy(logpath, logdir);
	strcat(logpath, logfile);
	ReadConfig();
}

DLLCLBK void opcDLLExit (HINSTANCE hDLL)
{
	UnregisterClass ("GraphWindow", g_hInst);
	oapiUnregisterCustomCmd (g_dwCmd);

	Graph::FreeGDI();
}

DLLCLBK void opcTimestep (double simt, double simdt, double mjd)
{
	if (!g_hDlg) return; // flight data dialog not open
	if (!g_bRecording) return; // recorder turned off


	double syst = oapiGetSysTime(); // ignore time acceleration for graph updates
	if (syst >= g_T+g_DT) {
		GetSamples(simt);
		//  update graphs
		for (DWORD i = 0; i < g_nGraph; i++)
			g_Graph[i]->AppendDataPoint();
		g_T = syst;
		InvalidateRect (GetDlgItem (g_hDlg, IDC_GRAPH), NULL, TRUE);
	}
}

void OpenDlgClbk (void *context)
{
	HWND hDlg = oapiOpenDialog (g_hInst, IDD_FLIGHTDATA, MsgProc);
	if (hDlg) g_hDlg = hDlg; // otherwise open already
}

void ResetVesselList (HWND hDlg)
{
	SendDlgItemMessage (hDlg, IDC_VESSELLIST, CB_RESETCONTENT, 0, 0);
	DWORD i, n = oapiGetVesselCount();
	if (!n) return; // this should not happen
	char name[256];
	for (i = 0; i < n; i++) {
		oapiGetObjectName (oapiGetVesselByIndex (i), name, 256);
		SendDlgItemMessage (hDlg, IDC_VESSELLIST, CB_ADDSTRING, 0, (LPARAM)name);
	}
	g_VESSEL = oapiGetVesselInterface (oapiGetFocusObject());
	i = SendDlgItemMessage (hDlg, IDC_VESSELLIST, CB_FINDSTRINGEXACT, -1, (LPARAM)g_VESSEL->GetName());
	SendDlgItemMessage (hDlg, IDC_VESSELLIST, CB_SETCURSEL, (i != CB_ERR ? i:0), 0);

}

void ResetBaseList (HWND hDlg)
{
	SendDlgItemMessage (hDlg, IDC_BASELIST, CB_RESETCONTENT, 0, 0);
	g_VESSEL = oapiGetVesselInterface (oapiGetFocusObject());
	OBJHANDLE surf_ref = g_VESSEL->GetSurfaceRef();
	DWORD i, n = oapiGetBaseCount(surf_ref);
	char name[256]="\0";
	if (!n) return; // this should not happen
	for (i = 0; i < n; i++) {
		oapiGetObjectName(oapiGetBaseByIndex(surf_ref, i), name, 256);
		SendDlgItemMessage (hDlg, IDC_BASELIST, CB_ADDSTRING, 0, (LPARAM)name);
	}
	n = oapiGetVesselCount();
	if (!n) return; // this should not happen
	for (i = 0; i < n; i++) {
		oapiGetObjectName (oapiGetVesselByIndex (i), name, 256);
		SendDlgItemMessage (hDlg, IDC_BASELIST, CB_ADDSTRING, 0, (LPARAM)name);
	}
	n = oapiGetStationCount();
	if (!n) return; // this should not happen
	SendDlgItemMessage (hDlg, IDC_BASELIST, CB_ADDSTRING, 0, (LPARAM)name);
	for (i = 0; i < n; i++) {
		oapiGetObjectName(oapiGetStationByIndex (i), name, 256);
		SendDlgItemMessage (hDlg, IDC_BASELIST, CB_ADDSTRING, 0, (LPARAM)name);
	}
	i = SendDlgItemMessage (hDlg, IDC_BASELIST, CB_FINDSTRINGEXACT, -1, (LPARAM)g_VESSEL->GetName());
	SendDlgItemMessage (hDlg, IDC_BASELIST, CB_SETCURSEL, (i != CB_ERR ? i:0), 0);
}

void ResetVessel (VESSEL *vessel)
{
	g_VESSEL = vessel;
	for (DWORD i = 0; i < g_nGraph; i++)
		g_Graph[i]->ResetData();
}

void AddGraph (HWND hDlg, int which)
{
	RECT rw, rc;
	int dh;
	DWORD i;
	char cbuf[256];

	for (i = 0; i < g_nGraph; i++)
		if (g_Graph[i]->DType() == which) return; // already present

	// stretch dialog window to accomodate the new graph
	HWND hCanvas = GetDlgItem (hDlg, IDC_GRAPH);
	GetWindowRect (hDlg, &rw);
	GetWindowRect (hCanvas, &rc);
	dh = (g_nGraph ? (rc.bottom-rc.top)/g_nGraph : (rc.right-rc.left)/2);

	SetWindowPos (hDlg, 0, 0, 0, rw.right-rw.left, rw.bottom-rw.top+dh, SWP_NOMOVE | SWP_NOREPOSITION | SWP_NOZORDER);
	SetWindowPos (hCanvas, 0, 0, 0, rc.right-rc.left, rc.bottom-rc.top+dh, SWP_NOMOVE | SWP_NOREPOSITION | SWP_NOZORDER);

	// add new graph to the list
	FlightDataGraph **tmp = new FlightDataGraph*[g_nGraph+1];
	if (g_nGraph) {
		memcpy (tmp, g_Graph, g_nGraph*sizeof(FlightDataGraph*));
		delete []g_Graph;
	}
	g_Graph = tmp;
	switch (which) {
	case 6:
	case 7:
	case 8:
	case 11:
	case 14:
	case 18:
		g_Graph[g_nGraph] = new FlightDataGraph (which, 2); break;
	default:
		g_Graph[g_nGraph] = new FlightDataGraph (which); break;
	}
	LoadString (g_hInst, IDS_DATA+which, cbuf, 256);
	g_Graph[g_nGraph]->SetTitle (cbuf);
	LoadString (g_hInst, IDS_YLABEL+which, cbuf, 256);
	g_Graph[g_nGraph]->SetYLabel (cbuf);
	LoadString (g_hInst, IDS_LEGEND+which, cbuf, 256);
	g_Graph[g_nGraph]->SetLegend (cbuf);
	g_nGraph++;
}

void DelGraph (HWND hDlg, int which)
{
	RECT rw, rc;
	DWORD i, j;
	int dh;

	for (i = 0; i < g_nGraph; i++)
		if (g_Graph[i]->DType() == which) break;
	if (i == g_nGraph) return; // not found

	// shrink dialog window
	HWND hCanvas = GetDlgItem (hDlg, IDC_GRAPH);
	GetWindowRect (hDlg, &rw);
	GetWindowRect (hCanvas, &rc);
	dh = (rc.bottom-rc.top)/g_nGraph;

	SetWindowPos (hDlg, 0, 0, 0, rw.right-rw.left, rw.bottom-rw.top-dh, SWP_NOMOVE | SWP_NOREPOSITION | SWP_NOZORDER);
	SetWindowPos (hCanvas, 0, 0, 0, rc.right-rc.left, rc.bottom-rc.top-dh, SWP_NOMOVE | SWP_NOREPOSITION | SWP_NOZORDER);

	// remove graph from the list
	FlightDataGraph **tmp = (g_nGraph > 1 ? new FlightDataGraph*[g_nGraph-1] : 0);
	if (tmp) {
		for (i = j = 0; i < g_nGraph; i++)
			if (g_Graph[i]->DType() == which) delete g_Graph[i];
			else tmp[j++] = g_Graph[i];
		delete []g_Graph;
	}
	g_Graph = tmp;
	g_nGraph--;
}

// =================================================================================
// FlightData dialog message handler
// =================================================================================

OAPI_MSGTYPE CALLBACK MsgProc (HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	switch (uMsg) {
	case WM_INITDIALOG: {
		char cbuf[256];
		int i;
		for (i = 0; i < NGRAPH; i++) {
			LoadString (g_hInst, IDS_DATA+i, cbuf, 256);
			SendDlgItemMessage (hDlg, IDC_DATALIST, LB_ADDSTRING, 0, (LPARAM)cbuf);
		}
		for (i = 0; i < NRATE; i++) {
			static const char *ratestr[NRATE] = {"0.01","0.1","1","10"};
			SendDlgItemMessage (hDlg, IDC_RATE, CB_ADDSTRING, 0, (LPARAM)ratestr[i]);
		}
		SendDlgItemMessage (hDlg, IDC_RATE, CB_SETCURSEL, 2, 0);
		SendDlgItemMessage(hDlg, IDC_LOGFILE, WM_SETTEXT, 0, (long long) &logpath[0]);
		g_DT = 1.0;
		g_T  = 0.0;
		ResetVesselList(hDlg);
		ResetBaseList(hDlg);
		g_bRecording = false;
		} return TRUE;
	case WM_DESTROY:
		WriteConfig();
		if (g_nGraph) {
			for (DWORD i = 0; i < g_nGraph; i++) delete g_Graph[i];
			delete []g_Graph;
			g_nGraph = 0;
		}
		g_hDlg = 0;
		return 0;
	case WM_COMMAND:
		switch (LOWORD (wParam)) {
		case IDCANCEL:
			oapiCloseDialog (g_hDlg);
			return TRUE;
		case IDC_VESSELLIST:
			if (HIWORD (wParam) == CBN_SELCHANGE) {
				int item = SendDlgItemMessage (hDlg, IDC_VESSELLIST, CB_GETCURSEL, 0, 0);
				if (item == CB_ERR) return TRUE;
				char cbuf[256];
				SendDlgItemMessage (hDlg, IDC_VESSELLIST, CB_GETLBTEXT, item, (LPARAM)cbuf);
				OBJHANDLE hv = oapiGetVesselByName (cbuf);
				if (hv) ResetVessel (oapiGetVesselInterface(hv));
				return TRUE;
			}
			break;
		case IDC_BASELIST:
			if (HIWORD (wParam) == CBN_SELCHANGE) {
				int item = SendDlgItemMessage (hDlg, IDC_BASELIST, CB_GETCURSEL, 0, 0);
				if (item == CB_ERR) return TRUE;
				char cbuf[256];
				SendDlgItemMessage (hDlg, IDC_BASELIST, CB_GETLBTEXT, item, (LPARAM)cbuf);
				strcpy(range_target, cbuf);
				tgt_base = 0;
				hbase = oapiGetBaseByName(g_VESSEL->GetSurfaceRef(), range_target);
				if (hbase != NULL) {
					// Rage target IS a surface BASE.
					tgt_base = 1;
					oapiGetBaseEquPos(hbase, &b_pos.LONG, &b_pos.LAT, &b_pos.RADIUS);
				}
				else {
					// Range target is NOT surface BASE, try VESSEL.
					hbase = oapiGetVesselByName(range_target);
					if (hbase != NULL) {
						if (oapiGetEquPos(hbase, &b_pos.LONG, &b_pos.LAT, &b_pos.RADIUS) == FALSE) {
						//  FAILED to get VESSEL's EQU coordinates.
							hbase = NULL;
							tgt_base = 0;
						}
					}
					else {
						// Range target is NOT VESSEL, try STATION.
						hbase = oapiGetStationByName(range_target);
						if (hbase != NULL) {
							if (oapiGetEquPos(hbase, &b_pos.LONG, &b_pos.LAT, &b_pos.RADIUS) == FALSE) {
							//  FAILED to get STATION's EQU coordinates.
								hbase = NULL;
								tgt_base = 0;
							}
						}
						else {
							// Range target is NOT STATION.  UNKNOWN TARGET.
							MessageBox(g_hDlg, "Unknown range target.", range_target, MB_OK);
							hbase = NULL;
							tgt_base = 0;
						}

					}
				}
				return TRUE;
			}
			break;
		case IDC_RATE:
			if (HIWORD (wParam) == CBN_SELCHANGE) {
				int i, item = SendDlgItemMessage (hDlg, IDC_RATE, CB_GETCURSEL, 0, 0);
				for (i = 0, g_DT = (float)100.0; i < item; i++) g_DT *= (float)0.1;
				return TRUE;
			}
			break;
		case IDC_DATALIST:
			if (HIWORD (wParam) == LBN_SELCHANGE) {
				for (DWORD i = 0; i < NGRAPH; i++) {
					if (SendDlgItemMessage (hDlg, IDC_DATALIST, LB_GETSEL, i, 0))
						AddGraph (hDlg, i);
					else
						DelGraph (hDlg, i);
				}
			}
			break;
		case IDC_STARTSTOP:
			SendDlgItemMessage(hDlg, IDC_LOGFILE, WM_GETTEXT, _MAX_PATH, (long long) &logpath[0]);
			g_bRecording = !g_bRecording;
			SetWindowText (GetDlgItem (hDlg, IDC_STARTSTOP), g_bRecording ? "Stop":"Start");
			return TRUE;
		case IDC_BROWSE: {
			int i = 0;
			char str_filter[] = "Comma Separated Values (*.csv)\0*.csv\0Text Files (*.dat)\0*.dat\0\0";
			OPENFILENAME ofn;
			memset(&ofn, 0, sizeof(OPENFILENAME));
			ofn.lStructSize = sizeof(OPENFILENAME);
			ofn.hwndOwner = g_hDlg;
			ofn.lpstrFilter = str_filter;
			ofn.lpstrFile = logpath;
			ofn.nMaxFile = _MAX_PATH;
			ofn.lpstrInitialDir = logdir;
			ofn.Flags = OFN_NONETWORKBUTTON|OFN_SHAREAWARE;
			ofn.lpstrDefExt = "dat";
			if (GetSaveFileName(&ofn) == 0) return TRUE;
			_chdir(orbiterpath);
			strncpy(logdir, logpath, ofn.nFileOffset);
			strcpy(logfile, logpath+ofn.nFileOffset);
			SendDlgItemMessage(hDlg, IDC_LOGFILE, WM_SETTEXT, 0, (long long) &logpath[0]);
			return TRUE; }
		case IDC_RESET:
			for (DWORD i = 0; i < g_nGraph; i++)
				g_Graph[i]->ResetData();
			return TRUE;
		}
		break;
	}
	return oapiDefDialogProc (hDlg, uMsg, wParam, lParam);
}

DLLCLBK void opcOpenRenderViewport (HWND renderWnd, DWORD width, DWORD height, BOOL fullscreen)
{
	if (range_target[0] != '\0') { 
		VESSELSTATUS v_stat;	

		if (g_VESSEL) {
			g_VESSEL->GetStatus(v_stat);
			hbase = oapiGetBaseByName(v_stat.rbody, range_target);
			if (hbase != NULL) oapiGetBaseEquPos(hbase, &b_pos.LONG, &b_pos.LAT, &b_pos.RADIUS);
			hbase = oapiGetStationByName(range_target);
			if (hbase != NULL) oapiGetEquPos(hbase, &b_pos.LONG, &b_pos.LAT, &b_pos.RADIUS);

		}
	}
}

// =================================================================================
// Graph canvas message handler
// =================================================================================

LRESULT CALLBACK Graph_WndProc (HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	switch (uMsg) {
	case WM_PAINT:
		if (g_nGraph) {
			RECT r;
			PAINTSTRUCT ps;
			HDC hDC;
			DWORD i;
			int gw, gh;
			GetClientRect (hWnd, &r);
			gw = r.right-r.left;
			gh = (r.bottom-r.top)/g_nGraph;
			hDC = BeginPaint (hWnd, &ps);
			for (i = 0; i < g_nGraph; i++) {
				SetViewportOrgEx (hDC, 0, i*gh, NULL);
				g_Graph[i]->Refresh (hDC, gw, gh);
			}
			EndPaint (hWnd, &ps);
		}
		break;
	}
	return DefWindowProc (hWnd, uMsg, wParam, lParam);
}

// =================================================================================
//  Flight Data Recorder Utility Functions
// =================================================================================

void LogData(void) {
	std::ofstream out_file(logpath,std::ios::app);
	if (out_file)
    {
		out_file << g_Data.sample << delim_char;
		out_file << g_Data.sim_time << delim_char;
		out_file << g_Data.ves_alt << delim_char;
		out_file << g_Data.ves_pitch << delim_char;
		out_file << g_Data.ves_roll << delim_char;
		out_file << g_Data.ves_yaw << delim_char;
		out_file << g_Data.ves_v_rad << delim_char;
		out_file << g_Data.ves_v_tan << delim_char;
		out_file << g_Data.ves_a_rad << delim_char;
		out_file << g_Data.ves_a_tan << delim_char;
		out_file << g_Data.ves_a_g << delim_char;
		out_file << g_Data.ves_surf_lon << delim_char;
		out_file << g_Data.ves_surf_lat << delim_char;
		out_file << g_Data.ves_surf_hdg << delim_char;
		out_file << g_Data.ves_dist << delim_char;
		out_file << g_Data.ves_aoa << delim_char;
		out_file << g_Data.ves_mach << delim_char;
		out_file << g_Data.ves_lift << delim_char;
		out_file << g_Data.ves_drag << delim_char;
		out_file << g_Data.atm_t << delim_char;
		out_file << g_Data.atm_stp << delim_char;
		out_file << g_Data.atm_dynp << delim_char;
		out_file << g_Data.atm_d << delim_char;
		out_file << g_Data.eng_fuel_mass << delim_char;
		out_file << g_Data.eng_fuel_rate << delim_char;
		out_file << g_Data.eng_main_t << delim_char;
		out_file << g_Data.eng_hover_t;
		out_file << std::endl;
	}

}

void GetSamples(double simt) {
	VESSELSTATUS v_stat;	
	VECTOR3 vel, pos, v_pos;
	ATMPARAM atm;
	OBJHANDLE ref;
	double a, r2, v2, vr2, vt2;
	double alt = g_VESSEL->GetAltitude();

	ref = g_VESSEL->GetSurfaceRef();
	M = oapiGetMass (ref);
	R = oapiGetSize (ref);
	g_VESSEL->GetStatus(v_stat);

	g_Data.sim_time = (float)simt;
		
	// grab vessel attitude samples
	g_Data.ves_alt = (float)(alt*1e-3);
	g_Data.ves_pitch = (float)(g_VESSEL->GetPitch()*DEG);
	g_Data.ves_roll = (float)(g_VESSEL->GetBank()*DEG);
	g_Data.ves_yaw = (float)(g_VESSEL->GetSlipAngle()*DEG);

	// grab vessel velocity samples and calculate acceleration 
	g_VESSEL->GetRelativeVel (g_VESSEL->GetSurfaceRef(), vel);
	g_VESSEL->GetRelativePos (g_VESSEL->GetSurfaceRef(), pos);
	r2 = pos.x*pos.x + pos.y*pos.y + pos.z*pos.z;
	v2 = ((vel.x*vel.x) + (vel.y*vel.y) + (vel.z*vel.z));
	a  = (vel.x*pos.x + vel.y*pos.y + vel.z*pos.z) / r2;
	vr2 = a*a * r2;
	vt2 = v2 - vr2;
	g_Data.ves_v_rad = (vr2 >= 0.0 ? a >= 0.0 ? (float)sqrt(vr2) : -(float)sqrt(vr2) : 0.0f);
	g_Data.ves_v_tan = (vt2 >= 0.0 ? (float)sqrt(vt2) : 0.0f);
	if ((g_Data.sample > 0) && (g_Data.ves_alt > 0.003)) {
		g_Data.ves_a_rad = (((g_Data.ves_v_rad) - (g_Data.prev_ves_v_rad))/g_DT);
		g_Data.ves_a_tan = (((g_Data.ves_v_tan) - (g_Data.prev_ves_v_tan))/g_DT);
	}
	g_Data.prev_ves_v_rad = g_Data.ves_v_rad;
	g_Data.prev_ves_v_tan = g_Data.ves_v_tan;

	// ---------- G meter -----------
	// 
		g_Data.ves_a_g = g_Data.ves_a_rad + g_Data.ves_a_tan;
		g_Data.ves_a_g = g_Data.ves_a_g < 0 ? -(g_Data.ves_a_g/(float) G) : (g_Data.ves_a_g/(float) G);
	
	// grab vessel position samples
	g_VESSEL->GetEquPos(v_pos.LONG, v_pos.LAT, v_pos.RADIUS);
	g_Data.ves_surf_lon = (float)(v_pos.LONG*DEG);
	g_Data.ves_surf_lat = (float)(v_pos.LAT*DEG);
	oapiGetFocusHeading(&a);
	g_Data.ves_surf_hdg = (float)(a*DEG);
	if (hbase) {
		if (!tgt_base) oapiGetEquPos(hbase, &b_pos.LONG, &b_pos.LAT, &b_pos.RADIUS); // if we are tracking a station, update position vector.
		g_Data.ves_dist = (float)(CalcSphericalDistance(b_pos, v_pos)*1e-3); // distance in km, disregarding altitude.
	}

	// angle of attack
	g_Data.ves_aoa = (float)(g_VESSEL->GetAOA()*DEG);

	// mach
	g_Data.ves_mach = (float)g_VESSEL->GetMachNumber();
	
	//  lift & drag
	g_Data.ves_lift = (float)(g_VESSEL->GetLift() * 0.001);
	g_Data.ves_drag = (float)(g_VESSEL->GetDrag() * 0.001);

	// grab atmospheric samples
	oapiGetPlanetAtmParams(v_stat.rbody, R+alt, &atm);
	g_Data.atm_t = (float)atm.T;
	g_Data.atm_stp = (float)atm.p;
	g_Data.atm_dynp = (float)g_VESSEL->GetDynPressure();
	g_Data.atm_d = (float)atm.rho;
		
	//grab engine samples
	g_Data.eng_fuel_mass = (float)g_VESSEL->GetTotalPropellantMass();
	g_Data.eng_fuel_rate = (float)g_VESSEL->GetTotalPropellantFlowrate();
	g_Data.eng_main_t = (float)(g_VESSEL->GetThrusterGroupLevel(THGROUP_MAIN))*100;
	g_Data.eng_hover_t = (float)(g_VESSEL->GetThrusterGroupLevel(THGROUP_HOVER))*100;


	//  log data to file
	LogData();

	//sprintf(oapiDebugString(), "Tacc: %f   Vacc: %f", g_Data.ves_a_tan[g_Data.sample], g_Data.ves_a_rad[g_Data.sample]);

	// get ready for next sample period
	g_Data.sample++;
}

void WriteConfig(void) {

	char str[3];
	char cfgpath[_MAX_PATH];

	strcpy(cfgpath, orbiterpath);
	strcat(cfgpath, "config\\");
	strcat(cfgpath, CONFIGFILE);

	std::ofstream out_file(cfgpath,std::ios::trunc);
	
	str[0] = delim_char;
	str[1] = '\0';

	if (out_file)
    {
		out_file << "SAMPLEDT " << g_DT << std::endl;
		if (delim_char != ' ') { out_file << "DELIM " << str << std::endl; }
		out_file << "TGTBASE " << range_target << std::endl;
		out_file << "LOGDIR " << logdir << std::endl;
		out_file << "LOGFILE " << logfile << std::endl;
	}
}

void ReadConfig(void) {
	char line[255];
	char str[3];
	char cfgpath[_MAX_PATH];

	strcpy(cfgpath, orbiterpath);
	strcat(cfgpath, "\\config\\");
	strcat(cfgpath, CONFIGFILE);

	std::ifstream in_file(cfgpath,std::ios::in);
   
	if (in_file.is_open()) {
		while (!in_file.eof()) {
			in_file.getline(line, 254);
			if (!strnicmp (line, "SAMPLEDT", 8))
				sscanf (line+8, "%f", &g_DT);
			else if (!strnicmp (line, "DELIM", 5)) {
				strcpy(str, line+6);
				delim_char = str[0];
			}
			else if (!strnicmp (line, "TGTBASE", 7)) {
				strcpy(range_target, line+8);
			}
			else if (!strnicmp (line, "LOGDIR", 6))
				strcpy(logdir, line+7);
			else if (!strnicmp (line, "LOGFILE", 7))
				strcpy(logfile, line+8);
		}
		strcpy(logpath, logdir);
		strcat(logpath, logfile);
	}
}
