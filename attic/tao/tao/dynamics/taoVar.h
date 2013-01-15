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

#ifndef _taoVar_h
#define _taoVar_h

#ifdef OSX
// that's why variables should never start with an underscore...
# undef _Q
#endif // OSX

#include <tao/matrix/TaoDeMath.h>
#include "taoDVar.h"

/*!
 *	\brief 1-DOF joint variable class for articulated body
 *	\ingroup taoDynamics
 *
 *	This provides joint variables necessary for articulated body dynamics.	
 */
class taoVarDOF1 : public taoDVar
{
public:
	deFloat _Q;		//!<	joint position
	deFloat _dQ;	//!<	joint velocity
	deFloat _ddQ;	//!<	joint acceleration
	deFloat _Tau;	//!<	joint force (torque)
};

/*!
 *	\brief spherical joint variable class for articulated body
 *	\ingroup taoDynamics
 *
 *	This provides joint variables necessary for articulated body dynamics.	
 */
class taoVarSpherical : public taoDVar
{
public:
	deQuaternion _Q;		//!<	joint position
	deVector3 _dQ;			//!<	joint velocity in local frame
	deVector3 _ddQ;			//!<	joint acceleration
	deVector3 _Tau;			//!<	joint force (torque)
	deVector3 _dQrotated;	//!<	joint velocity in reference (parent) frame
};

#endif // _taoVar_h
