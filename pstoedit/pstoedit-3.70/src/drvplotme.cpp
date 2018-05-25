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

#include <algorithm>

// #include "version.h"

const unsigned min_fitpoints = 5;
const unsigned max_fitpoints = 1000;
const double fitpoints_scale_factor = 0.5;

drvPLOTME::derivedConstructor(drvPLOTME)
    : //(const char * driveroptions_p,ostream & theoutStream,ostream &
      //theerrStream): // Constructor
      constructBase {
    // outf << "Sample header \n";
}

drvPLOTME::~drvPLOTME() {
    // driver specific deallocations
    // and writing of trailer to output file
    // outf << "Sample trailer \n";
    options = 0;
}

void drvPLOTME::open_page() {
    // outf << "Opening page: " << currentPageNumber << endl;
}

void drvPLOTME::close_page() {
    // outf << "Closing page: " << (currentPageNumber) << endl;
}

void drvPLOTME::show_path() {

    if (numberOfElementsInPath() == 0) {
        return;
    }

    switch (currentShowType()) {
    default:
    case stroke: {
        outf << "STARTPATH;STROKE;";
        break;
    }
    case fill: {
        outf << "STARTPATH;FILL;";
        break;
    }
    case eofill: {
        outf << "STARTPATH;EOFILL;";
        break;
    }
    }

    outf << this->currentR() << ";" << this->currentG() << ";" << this->currentB() << std::endl;

    Point currentPoint(0.0f, 0.0f);
    const Point firstPoint = pathElement(0).getPoint(0);

    for (unsigned int n = 0; n < numberOfElementsInPath(); n++) {
        const basedrawingelement &elem = pathElement(n);

        switch (elem.getType()) {
        case moveto: {
            const Point &p = elem.getPoint(0);
            outf << "MOVETO;" << p.x_ << ";" << p.y_ << "\n";
            currentPoint = p;
        } break;
        case lineto: {
            const Point &p = elem.getPoint(0);
            outf << "LINETO;" << p.x_ << ";" << p.y_ << "\n";
            currentPoint = p;
        } break;
        case closepath:
            outf << "LINETO;" << firstPoint.x_ << ";" << firstPoint.y_ << "\n";
            break;

        case curveto: {
            const Point &cp1 = elem.getPoint(0);
            const Point &cp2 = elem.getPoint(1);
            const Point &ep = elem.getPoint(2);
            // curve is approximated with a variable number or linear segments.
            // fitpoints should be somewhere between 5 and 50 for reasonable
            // page size plots we compute distance between current point and
            // endpoint and use that to help pick the number of segments to use.
            const float dist = pythagoras((float)(ep.x_ - currentPoint.x_),
                                          (float)(ep.y_ - currentPoint.y_));
            // errf << "dist:" << dist << std::endl;
            unsigned int fitpoints = dist / fitpoints_scale_factor;
            // errf << "fitpoints:" << fitpoints << std::endl;
            fitpoints = std::clamp(fitpoints, min_fitpoints, max_fitpoints);

            for (unsigned int s = 1; s < fitpoints; s++) {
                const float t = 1.0f * s / (fitpoints - 1);
                const Point pt = PointOnBezier(t, currentPoint, cp1, cp2, ep);
                outf << "LINETO;" << pt.x_ << ";" << pt.y_ << "\n";
            }
            currentPoint = ep;

        } break;
        default:
            errf << "\t\tFatal: unexpected case in drvgcode " << endl;
            abort();
            break;
        }
    }
}

static DriverDescriptionT<drvPLOTME> D_plotme(
    "plotme", "plotme driver",
    "this is a long description for the plotme driver", "pltme",
    true, // backend not support subpaths
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
    true,  // backend does support curves
    true,  // backend does support elements which are filled and have edges
    false, // backend does not support text
    DriverDescription::noimage, // no support for PNG file images
    DriverDescription::normalopen,
    false, // if format supports multiple pages in one file
    false  /*clipping */
);
