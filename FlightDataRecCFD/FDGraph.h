// ==============================================================
//                 ORBITER MODULE: FlightData
//                  Part of the ORBITER SDK
//            Copyright (C) 2003 Martin Schweiger
//                   All rights reserved
//
// FDGraph.h
// Flight data graph class interface.
// ==============================================================

#ifndef __FDGRAPH_H
#define __FDGRAPH_H

#include "Graph.h"

class FlightDataGraph: public Graph {
public:
	FlightDataGraph (int _dtype, int _nplot = 1): Graph (_nplot), dtype(_dtype) {}
	int DType() const { return dtype; }
	void AppendDataPoint ();

private:
	int dtype;
};

#endif // !__FDGRAPH_H