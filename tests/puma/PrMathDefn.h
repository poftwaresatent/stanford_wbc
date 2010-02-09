/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2010 Stanford University. All rights reserved.
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

/***************************************************************
 * PrMathDefn.h
 *
 * This files contains math constants and typedef's.
 *
 ****************************************************************/

/*
 * modification history
 *----------------------
 *
 * 10/26/97: K.C. Chang: created.
 */

#ifndef _PrMathDefn_h
#define _PrMathDefn_h

#include <cmath>

#ifndef M_PI
#define M_PI        3.14159265358979323846  // pi
#endif
#ifndef M_PI_2
#define M_PI_2      1.57079632679489661923  // pi/2
#endif
#ifndef M_PI_4
#define M_PI_4      0.78539816339744830962  // pi/4
#endif
 
#define PR_EPSILON         0.00001
#define PR_COS_THRESHHOLD ( 1.0 - PR_EPSILON )

#define PR_THOUSAND        1000
#define PR_MILLION         ( PR_THOUSAND*PR_THOUSAND )
#define PR_BILLION         ( PR_MILLION*PR_THOUSAND )

#define PR_LB_TO_KG        ( 0.45359237 ) //=( 1.0 / 2.2046226 )
#define PR_INCH_TO_METER   ( 0.0254 )

#define PR_GRAVITY_CONSTANT ( 9.81 )

#define PR_BITS_PER_BYTE  8
#define PR_BYTES_PER_WORD ( sizeof( unsigned int ) )

#endif // _PrMathDefn_h
