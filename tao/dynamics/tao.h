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

#ifndef _tao_h
#define _tao_h

/*!
 *	\defgroup taoControl Control
 */

/*!
 *	\defgroup taoDynamics Dynamics
 */

/*!
 *	\defgroup deMath Matrix
 */

/*!
 *	\defgroup deUtility Utility
 */

#include <tao/matrix/TaoDeMath.h>
#include <tao/utility/TaoDeMassProp.h>
#include <tao/utility/TaoDeLogger.h>

#include <tao/dynamics/taoTypes.h>

#include <tao/dynamics/taoWorld.h>
#include <tao/dynamics/taoGroup.h>
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoJoint.h>
#include <tao/dynamics/taoVar.h>

#ifdef TAO_CONTROL
#include "taoControlJt.h"
#endif

#include <tao/dynamics/taoDynamics.h>

#endif // _tao_h
