/*
 * saimatrix -- utility library for stanford-wbc.sourceforge.net
 *
 * Copyright (c) 1997-2008 Stanford University. All rights reserved.
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
 * SAIGlobalDefn.h
 *
 * This files contains global constants and typedef's.
 *
 ****************************************************************/

/*
 * modification history
 *----------------------
 *
 * 10/26/97: K.C. Chang: created.
 */

#ifndef _SAIGlobalDefn_h
#define _SAIGlobalDefn_h

#include <stdio.h>

#ifndef NULL
#define NULL 0
#endif

#define SAI_DOUBLE_PRECISION // double precision must be synchronized with tao/matrix/TaoDeTypes.h
#ifdef SAI_DOUBLE_PRECISION
typedef double Float;
#else // SAI_DOUBLE_PRECISION
typedef float Float;
#endif // SAI_DOUBLE_PRECISION

typedef void (*VoidFuncPtr )( void* arg );
typedef void (*VoidNoArgFuncPtr )();

#endif // _SAIGlobalDefn_h
