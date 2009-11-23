/* Copyright (c) 2005 Arachi, Inc. and Stanford University. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef _deMath_h
#define _deMath_h

#include <tao/matrix/TaoDeTypes.h>

#include <math.h>

/*!
 *	\ingroup deMath
 *	\name Trigonometric functions
 */
//	@{
#define	deSqrt(x)		((deFloat)sqrt(x))
#define deCos(x)		((deFloat)cos(x))
#define deSin(x)		((deFloat)sin(x))
#define deAcos(x)		((deFloat)acos(x))
#define deAsin(x)		((deFloat)asin(x))
#define deAtan2(x,y)	((deFloat)atan2(x,y))
#define deFabs(x)		((deFloat)fabs(x))
//	@}

/*!
 *	\ingroup deMath
 *	\name	Constants
 */
//	@{
#ifndef M_PI
#define DE_M_PI			((deFloat)3.14159265358979323846)  // pi
#else
#define DE_M_PI			((deFloat)M_PI)
#endif
#ifndef M_PI_2
#define DE_M_PI_2		((deFloat)1.57079632679489661923)  // pi/2
#else
#define DE_M_PI_2		((deFloat)M_PI_2)
#endif
#define DE_QUATERNION_EPSILON			((deFloat)0.000001)
#define DE_QUATERNION_COS_THRESHHOLD	(1 - DE_QUATERNION_EPSILON)
//	@}

/*!
 *	\ingroup deMath
 *	\name	API
 */
//	@{
#ifdef WIN32
#define DE_MATH_API		__inline
#else
#define DE_MATH_API		inline
#endif
//	@}

/*!
 *	\ingroup deMath
 *	\name	Matrix size
 */
//	@{

#ifdef DE_PS2_VU
#define DE_VECTOR3_SIZE		4
#define DE_MATRIX3_COL		4
#else
#define DE_VECTOR3_SIZE		3
#define DE_MATRIX3_COL		3
#endif

#define DE_QUATERNION_SIZE	4
#define DE_MATRIX3_ROW		3
#define DE_MATRIX3_SIZE		(DE_MATRIX3_ROW * DE_MATRIX3_COL)	
#define DE_FRAME_SIZE		(DE_QUATERNION_SIZE + DE_VECTOR3_SIZE)
#define DE_TRANSFORM_SIZE	(DE_MATRIX3_SIZE + DE_VECTOR3_SIZE)
//	@}

#include "TaoDeVector3f.h"
#include "TaoDeQuaternionf.h"
#include "TaoDeMatrix3f.h"

#ifdef __cplusplus

class deVector3;
class deQuaternion;
class deMatrix3;
class deFrame;
class deTransform;
class deVector6;
class deMatrix6;

#include "TaoDeVector3.h"
#include "TaoDeQuaternion.h"
#include "TaoDeMatrix3.h"
#include "TaoDeFrame.h"
#include "TaoDeTransform.h"
#include "TaoDeVector6.h"
#include "TaoDeMatrix6.h"

#include "TaoDeVector3.inl"
#include "TaoDeQuaternion.inl"
#include "TaoDeMatrix3.inl"
#include "TaoDeFrame.inl"
#include "TaoDeTransform.inl"
#include "TaoDeVector6.inl"
#include "TaoDeMatrix6.inl"

#endif //__cplusplus

#endif // _deMath_h
