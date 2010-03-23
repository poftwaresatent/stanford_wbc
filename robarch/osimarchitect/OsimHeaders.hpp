/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2009 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

//=========================================================================
/*!
 \author     Samir Menon
 \file       OsimHeaders.hpp (headers for the osim file parser)
 */
//=========================================================================

#ifndef OSIMHEADERS_HPP_
#define OSIMHEADERS_HPP_

#include <assert.h>

#define TESTING_FUNCTIONS_ON 1

#ifdef ROBARCH_IS_WINDOWS_
	#include "PrVector3.h"
	#include "PrMatrix3.h"
	#include "wbc_tinyxml/wbc_tinyxml.h"
#endif

#ifndef ROBARCH_IS_WINDOWS_
	#include <saimatrix/SAIVector3.h>
	#include <saimatrix/SAIMatrix3.h>
	#include <wbc_tinyxml/wbc_tinyxml.h>
#endif //#ifdef ROBARCH_IS_WINDOWS_

namespace osimparser {

#ifndef ROBARCH_IS_WINDOWS_
	#define PrVector3 SAIVector3
	#define PrMatrix3 SAIMatrix3
#endif //#ifdef ROBARCH_IS_WINDOWS_

#ifdef ROBARCH_IS_WINDOWS_
	#define SAIVector3 PrVector3 
	#define SAIMatrix3 PrMatrix3 
#endif

//typedef assert SAIAssertSz

const int _max_path=200;

}

#endif /*OSIMHEADERS_HPP_*/
