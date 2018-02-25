/*
   drvPLOTME.cpp : This file is part of pstoedit
   Skeleton for the implementation of new backends

   Copyright (C) 1993 - 2014 Wolfgang Glunz, wglunz35_AT_pstoedit.net

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

*/
#include "drvplotme.h"
#include I_fstream
#include I_stdio
#include I_stdlib

// #include "version.h"



drvPLOTME::derivedConstructor(drvPLOTME):
//(const char * driveroptions_p,ostream & theoutStream,ostream & theerrStream): // Constructor
constructBase
{
// driver specific initializations
// and writing of header to output file
	outf << "Sample header \n";
//  float           scale;
//  float           x_offset;
//  float           y_offset;
}

drvPLOTME::~drvPLOTME()
{
// driver specific deallocations
// and writing of trailer to output file
	outf << "Sample trailer \n";
	options=0;
}

void drvPLOTME::print_coords()
{
	for (unsigned int n = 0; n < numberOfElementsInPath(); n++) {
		const basedrawingelement & elem = pathElement(n);
		switch (elem.getType()) {
		case moveto:{
				const Point & p = elem.getPoint(0);
				outf << "\t\tmoveto ";
				outf << p.x_ + x_offset << " " << /*   currentDeviceHeight -  */ p.y_ +
					y_offset << " ";
			}
			break;
		case lineto:{
				const Point & p = elem.getPoint(0);
				outf << "\t\tlineto ";
				outf << p.x_ + x_offset << " " << /*   currentDeviceHeight -  */ p.y_ +
					y_offset << " ";
			}
			break;
		case closepath:
			outf << "\t\tclosepath ";
			break;
		case curveto:{
				outf << "\t\tcurveto ";
				for (unsigned int cp = 0; cp < 3; cp++) {
					const Point & p = elem.getPoint(cp);
					outf << (p.x_ + x_offset) << " " << /*   currentDeviceHeight -  */ (p.y_ +
																						y_offset) <<
						" ";
				}
			}
			break;
		default:
			errf << "\t\tFatal: unexpected case in drvplotme " << endl;
			abort();
			break;
		}
		outf << endl;
	}
}


void drvPLOTME::open_page()
{
	outf << "Opening page: " << currentPageNumber << endl;
}

void drvPLOTME::close_page()
{
	outf << "Closing page: " << (currentPageNumber) << endl;
}

void drvPLOTME::show_path()
{
	outf << "Path # " << currentNr();
	if (isPolygon())
		outf << " (polygon): " << endl;
	else
		outf << " (polyline): " << endl;
	outf << "\tcurrentShowType: ";
	switch (currentShowType()) {
	case drvbase::stroke:
		outf << "stroked";
		break;
	case drvbase::fill:
		outf << "filled";
		break;
	case drvbase::eofill:
		outf << "eofilled";
		break;
	default:
		// cannot happen
		outf << "unexpected ShowType " << (int) currentShowType();
		break;
	}
	outf << endl;
	outf << "\tcurrentLineWidth: " << currentLineWidth() << endl;
	outf << "\tcurrentR: " << currentR() << endl;
	outf << "\tcurrentG: " << currentG() << endl;
	outf << "\tcurrentB: " << currentB() << endl;
	outf << "\tedgeR:    " << edgeR() << endl;
	outf << "\tedgeG:    " << edgeG() << endl;
	outf << "\tedgeB:    " << edgeB() << endl;
	outf << "\tfillR:    " << fillR() << endl;
	outf << "\tfillG:    " << fillG() << endl;
	outf << "\tfillB:    " << fillB() << endl;
	outf << "\tcurrentLineCap: " << currentLineCap() << endl;
	outf << "\tdashPattern: " << dashPattern() << endl;
	outf << "\tPath Elements 0 to " << numberOfElementsInPath() - 1 << endl;
	print_coords();
}

static DriverDescriptionT < drvPLOTME > D_plotme("plotme", "plotme driver", "this is a long description for the plotme driver","pltme", false,	// backend does not support subpaths
											   // if subpathes are supported, the backend must deal with
											   // sequences of the following form
											   // moveto (start of subpath)
											   // lineto (a line segment)
											   // lineto
											   // moveto (start of a new subpath)
											   // lineto (a line segment)
											   // lineto
											   //
											   // If this argument is set to false each subpath is drawn
											   // individually which might not necessarily represent
											   // the original drawing.
											   true,	// backend does not support curves
											   false,	// backend does not support elements which are filled and have edges
											   false,	// backend does not support text
											   DriverDescription::noimage,	// no support for PNG file images
											   DriverDescription::normalopen, false,	// if format supports multiple pages in one file
											   false  /*clipping */ 
											   );
