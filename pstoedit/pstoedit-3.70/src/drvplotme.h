#ifndef __drvPLOTME_h
#define __drvPLOTME_h

/* 
   drvplotme.h : This file is part of pstoedit
   Class declaration for a sample output driver with no additional attributes
   and methods (minimal interface)

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

#include "drvbase.h"

class drvPLOTME : public drvbase {

public:

	derivedConstructor(drvPLOTME);
	~drvPLOTME();
	class DriverOptions : public ProgramOptions {
	public:
		DriverOptions() {}
	}*options;

#include "drvfuncs.h"

private:
};

#endif
 
 
