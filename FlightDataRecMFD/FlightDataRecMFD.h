#ifndef __FlightDataRecMFD_H
#define __FlightDataRecMFD_H

#include "..//..//include//Orbitersdk.h"
#include "..//..//include//MFDAPI.h"

#define LONG x
#define LAT y
#define RADIUS z

#define HOR x
#define VERT y
#define LATERAL y

class FlightDataRecMFD: public GraphMFD {
public:
	FlightDataRecMFD (DWORD w, DWORD h, VESSEL *vessel);
	~FlightDataRecMFD ();
	bool ConsumeKeyBuffered (DWORD key);
	bool ConsumeButton (int bt, int event);
	char *ButtonLabel (int bt);
	int  ButtonMenu (const MFDBUTTONMENU **menu) const;
	void Update (HDC hDC);
	bool SetAltRange (char *rstr);
	bool SetVradRange (char *rstr);
	bool SetVtanRange (char *rstr);
	bool SetBase (const std::string rstr);
	static OAPI_MSGTYPE MsgProc (UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam);

private:
	void InitReferences (void);
	OBJHANDLE ref;
	double tgt_alt;
	bool  alt_auto;
	bool  vrad_auto, vtan_auto;
	float minpitch, maxpitch;
	int page;
	float *ref_alt;
	float *ref_tvel;

	// transient parameter storage
	static struct SavePrm {
		int paused;
		int auto_inc;
		float sample_dt;
		char delim_char;
		char tgt_base[255];
		char logdir[255];
		char logfile[25];
		
		int page;
		float altmin, altmax;
		float vradmin, vradmax;
		float vtanmin, vtanmax;
	} saveprm;
};

#endif //!__FlightDataRecMFD_H