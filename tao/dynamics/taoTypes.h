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

#ifndef _taoTypes_h
#define _taoTypes_h

#include <tao/matrix/TaoDeTypes.h>

#undef TAO_CONTROL
//#define TAO_CONTROL

typedef enum {TAO_AXIS_X = 0, TAO_AXIS_Y = 1, TAO_AXIS_Z = 2, TAO_AXIS_S = 3, TAO_AXIS_USER = 4} taoAxis;
typedef enum {TAO_JOINT_PRISMATIC, TAO_JOINT_REVOLUTE, TAO_JOINT_SPHERICAL, TAO_JOINT_USER} taoJointType;
#ifdef TAO_CONTROL
typedef enum {TAO_CONTROL_ZERO, TAO_CONTROL_FLOAT, TAO_CONTROL_PD, TAO_CONTROL_GOALPOSITION} taoControlType;
#endif
#endif // _taoTypes_h
