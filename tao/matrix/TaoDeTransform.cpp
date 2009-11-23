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


#include <tao/matrix/TaoDeMath.h>

// Denavit-Hartenberg
void deTransform::set(const deFloat alpha, const deFloat a, const deFloat d, const deFloat theta)
{
	deFloat ca, sa, ct, st;

	ca = deCos(alpha);
	sa = deSin(alpha);
	ct = deCos(theta);
	st = deSin(theta);

	_m.set(	ct,			-st,		0, 
			st * ca,	ct * ca,	-sa, 
			st * sa,	ct * sa,	ca);
	_v.set(	a,			-sa * d,	ca * d);
}

// Screw
void deTransform::set(const deVector3& axis, const deFloat pitch, const deFloat angle)
{
	deFloat d, c, s, v, x, y, z;

	d = angle * (1 - pitch);
	c = deCos(angle * pitch);
	s = deSin(angle * pitch);
	v = 1 - c;
	x = axis[0];
	y = axis[1];
	z = axis[2];

	_m.set(	x * x * v + c,     x * y * v - z * s, x * z * v + y * s,
			x * y * v + z * s, y * y * v + c,     y * z * v - x * s,
			x * z * v - y * s, y * z * v + x * s, z * z * v + c);  
	_v.multiply(axis, d);
}

