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

#include <tao/utility/TaoDeMassProp.h>

// assuming the dimensions stay the same and the body consists of only one
void deMassProp::scale(const deFloat m)
{
	_inertia *= m / _m;
	_m = m;
}

void deMassProp::mass(const deFloat m, const deFrame* f)
{
	deVector3 pos;

	if (f == NULL)
	{
		pos.zero();
	}
	else 
	{
		pos = f->translation();

		// find inertial term
		deFloat Ixx= m * (pos[1] * pos[1] + pos[2] * pos[2]);
		deFloat Iyy= m * (pos[0] * pos[0] + pos[2] * pos[2]);
		deFloat Izz= m * (pos[0] * pos[0] + pos[1] * pos[1]);
		deFloat Ixy= m * (pos[0] * pos[1]);
		deFloat Ixz= m * (pos[0] * pos[2]);
		deFloat Iyz= m * (pos[1] * pos[2]);
		deMatrix3 I;
		I.set(Ixx,	-Ixy,	-Ixz,
			 -Ixy,	Iyy,	-Iyz,
			 -Ixz,	-Iyz,	Izz);

		_inertia += I;
	}

	// Update center of mass
	deVector3 sum;
	sum.multiply(_center, _m);
	deVector3 tmpV;
	tmpV.multiply(pos, m);
	sum += tmpV;

	_m += m;
	_center.multiply(sum, 1 / _m);
}

void deMassProp::inertia(const deMatrix3* inertia, const deFrame* f)
{
	if (f == NULL) 
	{
		_inertia += *inertia;
	}
	else
	{
		// R = R I Rt;
		deMatrix3 R, Rt, tmpM;
		R.set(f->rotation());
		Rt.transpose(R);
		tmpM.multiply(R, *inertia);
		R.multiply(tmpM, Rt);
		_inertia += R;
	}
}

void deMassProp::inertia(const deVector3* diag, const deFrame* f)
{
	// NOTE: This can probably be made more efficient but not critical to do so
	deMatrix3 I;
	I.diagonal(*diag);
	inertia(&I, f);
}

void deMassProp::inertia(const deFloat Ixx, const deFloat Iyy, const deFloat Izz, const deFrame* f)
{
	 deVector3 I;
	 I.set(Ixx, Iyy, Izz);
	 inertia(&I, f);
}

// V = DE_M_PI*r*r*h
void deMassProp::cylinder(const deFloat mp, const deFloat h, const deFloat r, const deFrame* f)
{
	deFloat m;
	if (isDensity(mp))
		m = -mp * (DE_M_PI) * r * r * h;
	else
		m = mp;
	mass(m, f);
	deFloat Ixx= (m / 12) * (3 * r * r + h * h);
	deFloat Izz= (deFloat)0.5 * m * r * r;
	inertia(Ixx, Ixx, Izz, f);
}

// h in z-axis
// center of mass h/4 from base in z
// V = (1.0f/3.0f)*(DE_M_PI)*r*r*h
void deMassProp::cone(const deFloat mp, const deFloat h, const deFloat r, const deFrame* f)
{
	deFloat m;
	if (isDensity(mp))
		m = -mp * (DE_M_PI) * r * r * h / 3;
	else
		m = mp;
	mass(m, f);
	deFloat Ixx = (3 * m / 80) * (4 * r * r + h * h);
	deFloat Izz = (3 * m / 10) * r * r;
	inertia(Ixx, Ixx, Izz, f);
}

// a in x-axis
// b in y-axis
// h in z-axis
// center of mass h/4 from base in z
// V = a*b*h/3.0f
void deMassProp::pyramid(const deFloat mp, const deFloat a, const deFloat b, const deFloat h, const deFrame* f)
{
	deFloat m;
	if (isDensity(mp))
		m = -mp * a * b * h / 3;
	else
		m=mp;
	mass(m, f);
	deFloat Ixx = (m / 80) * (4 * b * b + 3 * h * h);
	deFloat Iyy = (m / 80) * (4 * a * a + 3 * h * h);
	deFloat Izz = (m / 20) * (a * a + b * b);
	inertia(Ixx, Iyy, Izz, f);
}

// a in x-axis
// b in y-axis
// c in z-axis
// V = a*b*c
void deMassProp::block(const deFloat mp, const deFloat a, const deFloat b, const deFloat c, const deFrame* f)
{
	deFloat m;
	if (isDensity(mp))
		m = -mp * a * b * c;
	else
		m = mp;
	mass(m, f);
	deFloat m12 = m / 12;
	deFloat Ixx = m12 * (b * b + c * c);
	deFloat Iyy = m12 * (a * a + c * c);
	deFloat Izz = m12 * (a * a + b * b);
	inertia(Ixx, Iyy, Izz, f);
}

// V = (4.0f/3.0f)*(DE_M_PI)*r*r*r
void deMassProp::sphere(const deFloat mp, const deFloat r, const deFrame* f)
{
	deFloat m;
	if (isDensity(mp))
		m = -mp * 4 * (DE_M_PI) * r * r * r / 3;
	else
		m=mp;
	mass(m, f);
	deFloat I = 2 * m * r * r / 5;
	inertia(I, I, I);
}

// center of mass (3/8)*r from base in z
// V = (2.0f/3.0f)*(DE_M_PI)*r*r*r
void deMassProp::hemisphere(const deFloat mp, const deFloat r, const deFrame* f)
{
	deFloat m;
	if (isDensity(mp))
		m = -mp * 2 * (DE_M_PI) * r * r * r / 3;
	else
		m=mp;
	mass(m, f);
	deFloat Ixx = 83 * m * r * r / 320;
	deFloat Izz = 2 * m * r * r / 5;
	inertia(Ixx, Ixx, Izz, f);
}

// a in x-axis
// b in y-axis
// c in z-axis
// V = (4.0f/3.0f)*(DE_M_PI)*a*b*c
void deMassProp::ellipsoid(const deFloat mp, const deFloat a, const deFloat b, const deFloat c, const deFrame* f)
{
	deFloat m;
	if (isDensity(mp))
		m = -mp * 4 * (DE_M_PI) * a * b * c / 3;
	else
		m = mp;
	mass(m, f);
	deFloat m5 = m / 5;
	deFloat Ixx = m5 * (b * b + c * c);
	deFloat Iyy = m5 * (a * a + c * c);
	deFloat Izz = m5 * (a * a + b * b);
	inertia(Ixx, Iyy, Izz, f);
}

// l in z-axis
void deMassProp::rod(const deFloat mp, const deFloat l, const deFrame* f)
{
	deFloat m;
	if (isDensity(mp))
		m = -mp * l;
	else
		m = mp;
	mass(m, f);
	deFloat I = m * l * l / 12;
	inertia(I, I, 0);
}

// A = DE_M_PI*r*r
void deMassProp::disk(const deFloat mp, const deFloat r, const deFrame* f)
{
	deFloat m;
	if (isDensity(mp))
		m = -mp * (DE_M_PI) * r * r;
	else
		m=mp;
	mass(m, f);
	deFloat Ixx = (deFloat)0.25 * m * r * r;
	deFloat Izz = (deFloat)0.5 * m * r * r;
	inertia(Ixx, Ixx, Izz, f);
}

// a in x-axis
// b in y-axis
// A = a*b
void deMassProp::plate(const deFloat mp, const deFloat a, const deFloat b, const deFrame* f)
{
	deFloat m;
	if (isDensity(mp))
		m = -mp * a * b;
	else
		m = mp;
	mass(m, f);
	deFloat m12 = m / 12;
	deFloat Ixx = m12 * b * b;
	deFloat Iyy = m12 * a * a;
	deFloat Izz = m12 * (a * a + b * b);
	inertia(Ixx, Iyy, Izz, f);
}

// h in z-axis
// A = 2.0f*(DE_M_PI)*r*h
void deMassProp::cylinderShell(const deFloat mp, const deFloat h, const deFloat r, const deFrame* f)
{
	deFloat m;
	if (isDensity(mp))
		m = -mp * 2 * (DE_M_PI) * r * h;
	else
		m = mp;
	mass(m, f);
	deFloat Ixx = (m / 12) * (6 * r * r + h * h);
	deFloat Izz = m * r * r;
 	inertia(Ixx, Ixx, Izz, f);
}

// h in z-axis
// center of mass (1/3)*h from base in z
// A = (DE_M_PI)*r*deSqrt(r*r + h*h, const deFrame* f)
void deMassProp::coneShell(const deFloat mp, const deFloat h, const deFloat r, const deFrame* f)
{
	deFloat m;
	if (isDensity(mp))
		m = -mp * (DE_M_PI) * r * deSqrt(r * r + h * h);
	else
		m = mp;
	mass(m, f);
	deFloat Ixx = (m / 18) * ((deFloat)4.5 * r * r + h * h);
	deFloat Izz = (deFloat)0.5 * m * r * r;
	inertia(Ixx, Ixx, Izz, f);
}

// A = 4.0f*(DE_M_PI)*r*r
void deMassProp::sphereShell(const deFloat mp, const deFloat r, const deFrame* f)
{
	deFloat m;
	if (isDensity(mp))
		m = -mp * 4 * (DE_M_PI) * r * r;
	else
		m = mp;
	mass(m, f);
	deFloat I = (2 * m / 3) * r * r;
	inertia(I, I, I);
}

// center of mass r/2.0f from base in z
// A = 2.0f*(DE_M_PI)*r*r
void deMassProp::hemisphereShell(const deFloat mp, const deFloat r, const deFrame* f)
{
	deFloat m;
	if (isDensity(mp))
		m = -mp * 2 * (DE_M_PI) * r * r;
	else
		m = mp;
	mass(m, f);
	deFloat Ixx = (5 * m / 12) * r * r;
	deFloat Izz = (2 * m / 3) * r * r;
	inertia(Ixx, Ixx, Izz, f);
}

