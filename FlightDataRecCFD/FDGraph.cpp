// ==============================================================
//                 ORBITER MODULE: FlightData
//                  Part of the ORBITER SDK
//            Copyright (C) 2003 Martin Schweiger
//                   All rights reserved
//
// FDGraph.cpp
// Flight data graph class implementation.
// ==============================================================

#include "FDGraph.h"
#include "..//..//include//Orbitersdk.h"
#include "resource.h"

#define KNOTS 1.94384f
#define FEET 3.28084f

extern VESSEL *g_VESSEL;
extern float g_DT;
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

float prev_v_mag, prev_v_x, prev_v_y, prev_v_z = 0.0;


void FlightDataGraph::AppendDataPoint ()
{
	float dp;
	float dp2[2];

	switch (dtype) {
	case 0: // sample index
		dp = (float)g_Data.sample;
		Graph::AppendDataPoint (dp);
		return;
	case 1: // sim elapsed time
		dp = g_Data.sim_time;
		Graph::AppendDataPoint (dp);
		return;
	case 2: // Altitude
		dp = g_Data.ves_alt;
		Graph::AppendDataPoint (dp);
		return;
	case 3: // Pitch
		dp = g_Data.ves_pitch;
		Graph::AppendDataPoint (dp);
		return;
	case 4: // Roll
		dp = g_Data.ves_roll;
		Graph::AppendDataPoint (dp);
		return;
	case 5: // Yaw
		dp = g_Data.ves_yaw;
		Graph::AppendDataPoint (dp);
		return;
	case 6: // Velocity (Radial/Tangential)
		dp2[0] = g_Data.ves_v_rad;
		dp2[1] = g_Data.ves_v_tan;
		Graph::AppendDataPoints (dp2);
		return;
	case 7: // Acceleration (Radial/Tangential)
		dp2[0] = g_Data.ves_a_rad;
		dp2[1] = g_Data.ves_a_tan;
		Graph::AppendDataPoints (dp2);
		return;
	case 8: // Longitude/Lattitude
		dp2[0] = g_Data.ves_surf_lon;
		dp2[1] = g_Data.ves_surf_lat;
		Graph::AppendDataPoints (dp2);
		return;
	case 9: // Heading
		dp = g_Data.ves_surf_hdg;
		Graph::AppendDataPoint (dp);
		return;
	case 10: // Range-to-Target Base
		dp = g_Data.ves_dist;
		Graph::AppendDataPoint (dp);
		return;
	case 11: // AOA
		dp2[0] = g_Data.ves_aoa;
		dp2[1] = g_Data.ves_yaw;
		Graph::AppendDataPoints (dp2);
		return;
	case 12: // Mach
		dp = g_Data.ves_mach;
		Graph::AppendDataPoint (dp);
		return;
	case 13: // Temperature
		dp = g_Data.atm_t;
		Graph::AppendDataPoint (dp);
		return;
	case 14: // Pressure
		dp2[0] = g_Data.atm_stp;
		dp2[1] = g_Data.atm_dynp;
		Graph::AppendDataPoints (dp2);
		return;
	case 15: // Density
		dp = g_Data.atm_d;
		Graph::AppendDataPoint (dp);
		return;
	case 16: // Engine Fuel Mass
		dp = g_Data.eng_fuel_mass;
		Graph::AppendDataPoint (dp);
		return;
	case 17: // Engine Fuel Rate
		dp = g_Data.eng_fuel_rate;
		Graph::AppendDataPoint (dp);
		return;
	case 18: // lift and drag
		dp2[0] = g_Data.ves_lift;
		dp2[1] = g_Data.ves_drag;
		Graph::AppendDataPoints (dp2);
		return;
	case 19: // L/D
		dp = (g_Data.ves_drag ? g_Data.ves_lift/g_Data.ves_drag : 0);
		Graph::AppendDataPoint (dp);
		return;
	case 20: // Mass
		dp = (float)g_VESSEL->GetMass();
		Graph::AppendDataPoint (dp);
		return;
	case 21: // acceleration (G)	
		dp = g_Data.ves_a_g;
		Graph::AppendDataPoint (dp);
	}
}
