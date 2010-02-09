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
 * PrGlobalDefn.h
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

#ifndef _PrGlobalDefn_h
#define _PrGlobalDefn_h


#include <stdio.h>
#ifndef NULL
#define NULL 0
#endif

//#define PR_DOUBLE_PRECISION // double precision
#ifdef PR_DOUBLE_PRECISION
typedef double Float;
#else // PR_DOUBLE_PRECISION
typedef float Float;
#endif // PR_DOUBLE_PRECISION

#define PR_MISSING_NEWMAT10_LIB

typedef void (*VoidFuncPtr )( void* arg );
typedef void (*VoidNoArgFuncPtr )();

#define PR_EMPTY_ID      (-1)
#define PR_PUMA260_ID    0
#define PR_PUMA560_ID    1
#define PR_TRACK_ID      2
#define PR_SAMMBASE_ID   3
#define PR_OTTO_BOCK_ID  4
#define PR_SAMMARM0_ID   5
#define PR_SAMMARM1_ID   6
#define PR_XRBASE_ID     7
#define PR_XRARM0_ID     8
#define PR_XRARM1_ID     9

#define PR_ZEBRA_SENSOR_0 0
#define PR_ZEBRA_SENSOR_1 1
#define PR_ZEBRA_SENSOR_2 2

#define PR_ZEBRA_FS_ID    PR_ZEBRA_SENSOR_0
#define PR_ATI_FS_ID      3
#define PR_JR3_ISA_FS_ID  4
#define PR_JR3_PCI_FS_ID  5

#define PR_JR3_PCI_ACCEL_ID 0

// use with setprio() and getprio()
#define PR_PRIORITY_MIN   1
#define PR_PRIORITY_LOW  10  // default
#define PR_PRIORITY_MID  15
#define PR_PRIORITY_HIGH 19  // highest for non-super user
#define PR_PRIORITY_MAX  29  // highest for super user

#endif // _PrGlobalDefn_h
