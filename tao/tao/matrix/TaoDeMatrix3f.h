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

#ifndef _deMatrix3f_h
#define _deMatrix3f_h

#ifdef __cplusplus
extern "C" {
#endif

extern void deSetQ4M3(deFloat* resq, const deFloat (*m1)[DE_MATRIX3_COL]);
/* Euler XYZ */
extern void deSetV3M3xyz(deFloat* resv, const deFloat (*m1)[DE_MATRIX3_COL]);
/* Euler ZYX */
extern void deSetV3M3zyx(deFloat* resv, const deFloat (*m1)[DE_MATRIX3_COL]);
extern void deSetV3M3zyxV3(deFloat* resv, const deFloat (*m1)[DE_MATRIX3_COL], const deFloat* last);

extern void deLUdecomposeM3M3(deFloat (*lu)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL]);
extern void deBackSubstituteV3M3V3(deFloat* x, const deFloat (*lu)[DE_MATRIX3_COL], const deFloat* y);
extern void deSetM3S2(deFloat (*res)[DE_MATRIX3_COL], const int axis, const deFloat angle);

/* inlines */

DE_MATH_API void deSetM3S9(deFloat (*res)[DE_MATRIX3_COL],
						   const deFloat a0, const deFloat a1, const deFloat a2,
						   const deFloat a3, const deFloat a4, const deFloat a5,
						   const deFloat a6, const deFloat a7, const deFloat a8)
{
	res[0][0] = a0;	res[0][1] = a1;	res[0][2] = a2;
	res[1][0] = a3;	res[1][1] = a4;	res[1][2] = a5;
	res[2][0] = a6;	res[2][1] = a7;	res[2][2] = a8;
}

DE_MATH_API void deZeroM3(deFloat (*res)[DE_MATRIX3_COL])
{
	res[0][0] = 0;	res[0][1] = 0;	res[0][2] = 0;
	res[1][0] = 0;	res[1][1] = 0;	res[1][2] = 0;
	res[2][0] = 0;	res[2][1] = 0;	res[2][2] = 0;
}

DE_MATH_API void deIdentityM3(deFloat (*res)[DE_MATRIX3_COL])
{
	res[0][0] = 1;	res[0][1] = 0;	res[0][2] = 0;
	res[1][0] = 0;	res[1][1] = 1;	res[1][2] = 0;
	res[2][0] = 0;	res[2][1] = 0;	res[2][2] = 1;
}

DE_MATH_API void deMulM3S1(deFloat (*res)[DE_MATRIX3_COL], const deFloat s)
{
	res[0][0] *= s;	res[0][1] *= s;	res[0][2] *= s;
	res[1][0] *= s;	res[1][1] *= s;	res[1][2] *= s;
	res[2][0] *= s;	res[2][1] *= s;	res[2][2] *= s;
}

DE_MATH_API void deNegateM3M3(deFloat (*res)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL])
{
	res[0][0] = -m1[0][0];	res[0][1] = -m1[0][1];	res[0][2] = -m1[0][2];
	res[1][0] = -m1[1][0];	res[1][1] = -m1[1][1];	res[1][2] = -m1[1][2];
	res[2][0] = -m1[2][0];	res[2][1] = -m1[2][1];	res[2][2] = -m1[2][2];
}

DE_MATH_API void deSetM3M3(deFloat (*res)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL])
{
	res[0][0] = m1[0][0];	res[0][1] = m1[0][1];	res[0][2] = m1[0][2];
	res[1][0] = m1[1][0];	res[1][1] = m1[1][1];	res[1][2] = m1[1][2];
	res[2][0] = m1[2][0];	res[2][1] = m1[2][1];	res[2][2] = m1[2][2];
}

DE_MATH_API void deAddM3M3(deFloat (*res)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL])
{
	res[0][0] += m1[0][0];	res[0][1] += m1[0][1];	res[0][2] += m1[0][2];
	res[1][0] += m1[1][0];	res[1][1] += m1[1][1];	res[1][2] += m1[1][2];
	res[2][0] += m1[2][0];	res[2][1] += m1[2][1];	res[2][2] += m1[2][2];
}

DE_MATH_API void deSubM3M3(deFloat (*res)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL])
{
	res[0][0] -= m1[0][0];	res[0][1] -= m1[0][1];	res[0][2] -= m1[0][2];
	res[1][0] -= m1[1][0];	res[1][1] -= m1[1][1];	res[1][2] -= m1[1][2];
	res[2][0] -= m1[2][0];	res[2][1] -= m1[2][1];	res[2][2] -= m1[2][2];
}


DE_MATH_API void deAddM3M3M3(deFloat (*res)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL], const deFloat (*m2)[DE_MATRIX3_COL])
{
	res[0][0] = m1[0][0] + m2[0][0];	res[0][1] = m1[0][1] + m2[0][1];	res[0][2] = m1[0][2] + m2[0][2];
	res[1][0] = m1[1][0] + m2[1][0];	res[1][1] = m1[1][1] + m2[1][1];	res[1][2] = m1[1][2] + m2[1][2];
	res[2][0] = m1[2][0] + m2[2][0];	res[2][1] = m1[2][1] + m2[2][1];	res[2][2] = m1[2][2] + m2[2][2];
}

DE_MATH_API void deSubM3M3M3(deFloat (*res)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL], const deFloat (*m2)[DE_MATRIX3_COL])
{
	res[0][0] = m1[0][0] - m2[0][0];	res[0][1] = m1[0][1] - m2[0][1];	res[0][2] = m1[0][2] - m2[0][2];
	res[1][0] = m1[1][0] - m2[1][0];	res[1][1] = m1[1][1] - m2[1][1];	res[1][2] = m1[1][2] - m2[1][2];
	res[2][0] = m1[2][0] - m2[2][0];	res[2][1] = m1[2][1] - m2[2][1];	res[2][2] = m1[2][2] - m2[2][2];
}

DE_MATH_API void deMulM3M3S1(deFloat (*res)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL], const deFloat s)
{
	res[0][0] = m1[0][0] * s;	res[0][1] = m1[0][1] * s;	res[0][2] = m1[0][2] * s;
	res[1][0] = m1[1][0] * s;	res[1][1] = m1[1][1] * s;	res[1][2] = m1[1][2] * s;
	res[2][0] = m1[2][0] * s;	res[2][1] = m1[2][1] * s;	res[2][2] = m1[2][2] * s;
}

DE_MATH_API void deMulM3M3M3(deFloat (*res)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL], const deFloat (*m2)[DE_MATRIX3_COL])
{
    res[0][0] = m1[0][0] * m2[0][0] + m1[0][1] * m2[1][0] + m1[0][2] * m2[2][0];
    res[0][1] = m1[0][0] * m2[0][1] + m1[0][1] * m2[1][1] + m1[0][2] * m2[2][1];
    res[0][2] = m1[0][0] * m2[0][2] + m1[0][1] * m2[1][2] + m1[0][2] * m2[2][2];
    res[1][0] = m1[1][0] * m2[0][0] + m1[1][1] * m2[1][0] + m1[1][2] * m2[2][0];
    res[1][1] = m1[1][0] * m2[0][1] + m1[1][1] * m2[1][1] + m1[1][2] * m2[2][1];
    res[1][2] = m1[1][0] * m2[0][2] + m1[1][1] * m2[1][2] + m1[1][2] * m2[2][2];
    res[2][0] = m1[2][0] * m2[0][0] + m1[2][1] * m2[1][0] + m1[2][2] * m2[2][0];
    res[2][1] = m1[2][0] * m2[0][1] + m1[2][1] * m2[1][1] + m1[2][2] * m2[2][1];
    res[2][2] = m1[2][0] * m2[0][2] + m1[2][1] * m2[1][2] + m1[2][2] * m2[2][2];
}

/* res = m1^T * m2 */
DE_MATH_API void deMulM3M3tM3(deFloat (*res)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL], const deFloat (*m2)[DE_MATRIX3_COL])
{
    res[0][0] = m1[0][0] * m2[0][0] + m1[1][0] * m2[1][0] + m1[2][0] * m2[2][0];
    res[0][1] = m1[0][0] * m2[0][1] + m1[1][0] * m2[1][1] + m1[2][0] * m2[2][1];
    res[0][2] = m1[0][0] * m2[0][2] + m1[1][0] * m2[1][2] + m1[2][0] * m2[2][2];
    res[1][0] = m1[0][1] * m2[0][0] + m1[1][1] * m2[1][0] + m1[2][1] * m2[2][0];
    res[1][1] = m1[0][1] * m2[0][1] + m1[1][1] * m2[1][1] + m1[2][1] * m2[2][1];
    res[1][2] = m1[0][1] * m2[0][2] + m1[1][1] * m2[1][2] + m1[2][1] * m2[2][2];
    res[2][0] = m1[0][2] * m2[0][0] + m1[1][2] * m2[1][0] + m1[2][2] * m2[2][0];
    res[2][1] = m1[0][2] * m2[0][1] + m1[1][2] * m2[1][1] + m1[2][2] * m2[2][1];
    res[2][2] = m1[0][2] * m2[0][2] + m1[1][2] * m2[1][2] + m1[2][2] * m2[2][2];
}

/* res = m1 * m2^T */
DE_MATH_API void deMulM3M3M3t(deFloat (*res)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL], const deFloat (*m2)[DE_MATRIX3_COL])
{
    res[0][0] = m1[0][0] * m2[0][0] + m1[0][1] * m2[0][1] + m1[0][2] * m2[0][2];
    res[0][1] = m1[0][0] * m2[1][0] + m1[0][1] * m2[1][1] + m1[0][2] * m2[1][2];
    res[0][2] = m1[0][0] * m2[2][0] + m1[0][1] * m2[2][1] + m1[0][2] * m2[2][2];
    res[1][0] = m1[1][0] * m2[0][0] + m1[1][1] * m2[0][1] + m1[1][2] * m2[0][2];
    res[1][1] = m1[1][0] * m2[1][0] + m1[1][1] * m2[1][1] + m1[1][2] * m2[1][2];
    res[1][2] = m1[1][0] * m2[2][0] + m1[1][1] * m2[2][1] + m1[1][2] * m2[2][2];
    res[2][0] = m1[2][0] * m2[0][0] + m1[2][1] * m2[0][1] + m1[2][2] * m2[0][2];
    res[2][1] = m1[2][0] * m2[1][0] + m1[2][1] * m2[1][1] + m1[2][2] * m2[1][2];
    res[2][2] = m1[2][0] * m2[2][0] + m1[2][1] * m2[2][1] + m1[2][2] * m2[2][2];
}

DE_MATH_API void deMulV3M3V3(deFloat* resv, const deFloat (*m1)[DE_MATRIX3_COL], const deFloat* v2)
{
    resv[0] = m1[0][0] * v2[0] + m1[0][1] * v2[1] + m1[0][2] * v2[2];
    resv[1] = m1[1][0] * v2[0] + m1[1][1] * v2[1] + m1[1][2] * v2[2];
    resv[2] = m1[2][0] * v2[0] + m1[2][1] * v2[1] + m1[2][2] * v2[2];
}

/* resv = m2^T * v2 */
DE_MATH_API void deMulV3M3tV3(deFloat* resv, const deFloat (*m1)[DE_MATRIX3_COL], const deFloat* v2)
{
    resv[0] = m1[0][0] * v2[0] + m1[1][0] * v2[1] + m1[2][0] * v2[2];
    resv[1] = m1[0][1] * v2[0] + m1[1][1] * v2[1] + m1[2][1] * v2[2];
    resv[2] = m1[0][2] * v2[0] + m1[1][2] * v2[1] + m1[2][2] * v2[2];
}

/* res = m1 * (v2 x) */
DE_MATH_API void deMulM3M3V3x(deFloat (*res)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL], const deFloat* v2)
{
    res[0][0] =				  m1[0][1] * v2[2] - m1[0][2] * v2[1];
    res[0][1] = -m1[0][0] * v2[2]                 + m1[0][2] * v2[0];
    res[0][2] =  m1[0][0] * v2[1]	- m1[0][1] * v2[0];
    res[1][0] =                  m1[1][1] * v2[2] - m1[1][2] * v2[1];
    res[1][1] = -m1[1][0] * v2[2]                 + m1[1][2] * v2[0];
    res[1][2] =  m1[1][0] * v2[1]	- m1[1][1] * v2[0];
    res[2][0] =                  m1[2][1] * v2[2] - m1[2][2] * v2[1];
    res[2][1] = -m1[2][0] * v2[2]                 + m1[2][2] * v2[0];
    res[2][2] =  m1[2][0] * v2[1]	- m1[2][1] * v2[0];
}

/* [0 1 2;3 4 5;6 7 8]^T = [0 3 6;1 4 7;2 5 8]  */
DE_MATH_API void deTransposeM3M3(deFloat (*res)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL])
{
    res[0][0] = m1[0][0];    res[0][1] = m1[1][0];    res[0][2] = m1[2][0];
    res[1][0] = m1[0][1];    res[1][1] = m1[1][1];    res[1][2] = m1[2][1];
    res[2][0] = m1[0][2];    res[2][1] = m1[1][2];    res[2][2] = m1[2][2];
}

DE_MATH_API void deColumnV3M3S1(deFloat* resv, const deFloat (*m1)[DE_MATRIX3_COL], const int col)
{
	resv[0] = m1[0][col];
	resv[1] = m1[1][col];
	resv[2] = m1[2][col];
}

DE_MATH_API void deDiagonalM3V3(deFloat (*res)[DE_MATRIX3_COL], const deFloat* v1)
{
	res[0][0] = v1[0];	res[0][1] = 0;		res[0][2] = 0;
	res[1][0] = 0;		res[1][1] = v1[1];	res[1][2] = 0;
	res[2][0] = 0;		res[2][1] = 0;		res[2][2] = v1[2];
}

DE_MATH_API void deDiagonalM3S3(deFloat (*res)[DE_MATRIX3_COL], const deFloat x, const deFloat y, const deFloat z)
{
	res[0][0] = x;		res[0][1] = 0;		res[0][2] = 0;
	res[1][0] = 0;		res[1][1] = y;		res[1][2] = 0;
	res[2][0] = 0;		res[2][1] = 0;		res[2][2] = z;
}

DE_MATH_API void deDiagonalV3M3(deFloat* resv, const deFloat (*m1)[DE_MATRIX3_COL])
{
    resv[0] = m1[0][0];
    resv[1] = m1[1][1];
    resv[2] = m1[2][2];
}

DE_MATH_API deFloat dedetM3(const deFloat (*m1)[DE_MATRIX3_COL])
{
	return (  m1[0][0] * (m1[1][1] * m1[2][2] - m1[1][2] * m1[2][1])
			- m1[0][1] * (m1[1][0] * m1[2][2] - m1[1][2] * m1[2][0])
			+ m1[0][2] * (m1[1][0] * m1[2][1] - m1[1][1] * m1[2][0]));
}

DE_MATH_API void deInvertDetSPDM3M3(deFloat (*res)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL])
{
     deFloat ood = 1 / dedetM3(m1);
	
     res[0][0] =  ood * (m1[1][1] * m1[2][2] - m1[1][2] * m1[2][1]);
     res[0][1] = -ood * (m1[0][1] * m1[2][2] - m1[0][2] * m1[2][1]);
     res[0][2] =  ood * (m1[0][1] * m1[1][2] - m1[0][2] * m1[1][1]);
     res[1][0] =  res[0][1];
     res[1][1] =  ood * (m1[0][0] * m1[2][2] - m1[0][2] * m1[2][0]);
     res[1][2] = -ood * (m1[0][0] * m1[1][2] - m1[0][2] * m1[1][0]);
     res[2][0] =  res[0][2];
     res[2][1] =  res[1][2];
     res[2][2] =  ood * (m1[0][0] * m1[1][1] - m1[0][1] * m1[1][0]);
}

DE_MATH_API void deInvertDetM3M3(deFloat (*res)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL])
{
     deFloat ood = 1 / dedetM3(m1);
	
     res[0][0] =  ood * (m1[1][1] * m1[2][2] - m1[1][2] * m1[2][1]);
     res[0][1] = -ood * (m1[0][1] * m1[2][2] - m1[0][2] * m1[2][1]);
     res[0][2] =  ood * (m1[0][1] * m1[1][2] - m1[0][2] * m1[1][1]);
     res[1][0] = -ood * (m1[1][0] * m1[2][2] - m1[1][2] * m1[2][0]);
     res[1][1] =  ood * (m1[0][0] * m1[2][2] - m1[0][2] * m1[2][0]);
     res[1][2] = -ood * (m1[0][0] * m1[1][2] - m1[0][2] * m1[1][0]);
     res[2][0] =  ood * (m1[1][0] * m1[2][1] - m1[1][1] * m1[2][0]);
     res[2][1] = -ood * (m1[0][0] * m1[2][1] - m1[0][1] * m1[2][0]);
     res[2][2] =  ood * (m1[0][0] * m1[1][1] - m1[0][1] * m1[1][0]);
}

/* resv = m1 - m2 */
DE_MATH_API void deAngularErrorV3M3M3(deFloat* resv, const deFloat (*m1)[DE_MATRIX3_COL], const deFloat (*m2)[DE_MATRIX3_COL])
{
	resv[0] = -(deFloat)0.5*((m1[1][0] * m2[2][0] + m1[1][1] * m2[2][1] + m1[1][2] * m2[2][2]) 
				 - (m1[2][0] * m2[1][0] + m1[2][1] * m2[1][1] + m1[2][2] * m2[1][2]));
	resv[1] = -(deFloat)0.5*((m1[2][0] * m2[0][0] + m1[2][1] * m2[0][1] + m1[2][2] * m2[0][2])
		         - (m1[0][0] * m2[2][0] + m1[0][1] * m2[2][1] + m1[0][2] * m2[2][2]));
	resv[2] = -(deFloat)0.5*((m1[0][0] * m2[1][0] + m1[0][1] * m2[1][1] + m1[0][2] * m2[1][2])
		         - (m1[1][0] * m2[0][0] + m1[1][1] * m2[0][1] + m1[1][2] * m2[0][2]));
}

/*
 * 27 mult + 12 add
 * Intro. to Robotics by J. Craig: p. 55
 */
DE_MATH_API void deSetM3Q4(deFloat (*res)[DE_MATRIX3_COL], const deFloat* q1)
{
    res[0][0] = 1 - 2 * (q1[1] * q1[1] + q1[2] * q1[2]);
    res[0][1] =     2 * (q1[0] * q1[1] - q1[3] * q1[2]);
    res[0][2] =     2 * (q1[0] * q1[2] + q1[3] * q1[1]);
    res[1][0] =     2 * (q1[0] * q1[1] + q1[3] * q1[2]);
    res[1][1] = 1 - 2 * (q1[0] * q1[0] + q1[2] * q1[2]);
    res[1][2] =     2 * (q1[1] * q1[2] - q1[3] * q1[0]);
    res[2][0] =     2 * (q1[0] * q1[2] - q1[3] * q1[1]);
    res[2][1] =     2 * (q1[1] * q1[2] + q1[3] * q1[0]);
    res[2][2] = 1 - 2 * (q1[0] * q1[0] + q1[1] * q1[1]);
}

/* res = v1 * v2^T */
DE_MATH_API void deMulM3V3V3t(deFloat (*res)[DE_MATRIX3_COL], const deFloat* v1, const deFloat* v2)
{
    res[0][0] = v1[0] * v2[0];
    res[0][1] = v1[0] * v2[1];
    res[0][2] = v1[0] * v2[2];
    res[1][0] = v1[1] * v2[0];
    res[1][1] = v1[1] * v2[1];
    res[1][2] = v1[1] * v2[2];
    res[2][0] = v1[2] * v2[0];
    res[2][1] = v1[2] * v2[1];
    res[2][2] = v1[2] * v2[2];
}

/* ([x y z] x) = [0 -z y; z 0 -x; -y x 0] */
DE_MATH_API void deSetM3V3x(deFloat (*res)[DE_MATRIX3_COL], const deFloat* v1)
{
    res[0][0]=      0;	res[0][1]= -v1[2]; res[0][2]=  v1[1];
    res[1][0]=  v1[2]; res[1][1]=      0; res[1][2]= -v1[0];
    res[2][0]= -v1[1]; res[2][1]=  v1[0]; res[2][2]=      0;
}

/* res = (v1 x)*m2 */
DE_MATH_API void deMulM3V3xM3(deFloat (*res)[DE_MATRIX3_COL], const deFloat* v1, const deFloat (*m2)[DE_MATRIX3_COL])
{
    res[0][0]= -v1[2] * m2[1][0] + v1[1] * m2[2][0];
    res[0][1]= -v1[2] * m2[1][1] + v1[1] * m2[2][1];
    res[0][2]= -v1[2] * m2[1][2] + v1[1] * m2[2][2];
    res[1][0]=  v1[2] * m2[0][0] - v1[0] * m2[2][0];
    res[1][1]=  v1[2] * m2[0][1] - v1[0] * m2[2][1];
    res[1][2]=  v1[2] * m2[0][2] - v1[0] * m2[2][2];
    res[2][0]= -v1[1] * m2[0][0] + v1[0] * m2[1][0];
    res[2][1]= -v1[1] * m2[0][1] + v1[0] * m2[1][1];
    res[2][2]= -v1[1] * m2[0][2] + v1[0] * m2[1][2];
}

/* setting rotation matrices */

/* axis angle */
DE_MATH_API void deSetM3V3S1(deFloat (*res)[DE_MATRIX3_COL], const deFloat* axis, const deFloat angle)
{
	/* also, see Intro. to Robotics by J. Craig: p. 52 */

	deFloat c = deCos(angle);
	deFloat v = 1 - c;
	deFloat s = deSin(angle);

	res[0][0] = axis[0] * axis[0] * v + c;
	res[0][1] = axis[0] * axis[1] * v - axis[2] * s;
	res[0][2] = axis[0] * axis[2] * v + axis[1] * s;
	res[1][0] = axis[1] * axis[0] * v + axis[2] * s;
	res[1][1] = axis[1] * axis[1] * v + c;
	res[1][2] = axis[1] * axis[2] * v - axis[0] * s;
	res[2][0] = axis[2] * axis[0] * v - axis[1] * s;
	res[2][1] = axis[2] * axis[1] * v + axis[0] * s;
	res[2][2] = axis[2] * axis[2] * v + c;
}

DE_MATH_API void deSetM3xyzS3(deFloat (*res)[DE_MATRIX3_COL], const deFloat x, const deFloat y, const deFloat z)
{
	deFloat cx = deCos(x);
	deFloat sx = deSin(x);
	deFloat cy = deCos(y);
	deFloat sy = deSin(y);
	deFloat cz = deCos(z);
	deFloat sz = deSin(z);

	res[0][0] =  cy * cz;
	res[0][1] = -cy * sz;
	res[0][2] =  sy;

	res[1][0] =  sx * sy * cz + cx * sz;
	res[1][1] = -sx * sy * sz + cx * cz;
	res[1][2] = -sx * cy;

	res[2][0] = -cx * sy * cz + sx * sz;
	res[2][1] =  cx * sy * sz + sx * cz;
	res[2][2] =  cx * cy;
}

DE_MATH_API void deSetM3zyxS3(deFloat (*res)[DE_MATRIX3_COL], const deFloat x, const deFloat y, const deFloat z)
{
	deFloat cx = deCos(x);
	deFloat sx = deSin(x);
	deFloat cy = deCos(y);
	deFloat sy = deSin(y);
	deFloat cz = deCos(z);
	deFloat sz = deSin(z);

	res[0][0] =  cy * cz;
	res[1][0] =  sx * sy * cz - cx * sz;
	res[2][0] =  cx * sy * cz + sx * sz;

	res[0][1] =  cy * sz;
	res[1][1] =  sx * sy * sz + cx * cz;
	res[2][1] =  cx * sy * sz - sx * cz;

	res[2][0] = -sy;
	res[2][1] =  sx * cy;
	res[2][2] =  cx * cy;
}

#ifdef __cplusplus
}
#endif

#endif /* _deMatrix3f_h */
