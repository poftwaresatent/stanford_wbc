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

#ifndef _deVector3f_h
#define _deVector3f_h

#ifdef __cplusplus
extern "C" {
#endif

/* inlines */

DE_MATH_API void deSetV3S3(deFloat* res, const deFloat x, const deFloat y, const deFloat z)
{
	res[0] = x;
	res[1] = y;
	res[2] = z;
}

DE_MATH_API void deSetV3V3(deFloat* res, const deFloat* v1)
{
	res[0] = v1[0];
	res[1] = v1[1];
	res[2] = v1[2];
}

DE_MATH_API void deNegateV3V3(deFloat* res, const deFloat* v1)
{
	res[0] = -v1[0];
	res[1] = -v1[1];
	res[2] = -v1[2];
}

DE_MATH_API void deAddV3S1(deFloat* res, const deFloat s)
{
	res[0] += s;
	res[1] += s;
	res[2] += s;
}

DE_MATH_API void deMulV3S1(deFloat* res, const deFloat s)
{
	res[0] *= s;
	res[1] *= s;
	res[2] *= s;
}

DE_MATH_API int deIsEqualV3V3(const deFloat* v1, const deFloat* v2)
{
	return (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] == v2[2]);
}

DE_MATH_API deFloat deDotV3V3(const deFloat* v1, const deFloat* v2)
{
	return (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]);
}

DE_MATH_API void deZeroV3(deFloat* res)
{
	res[0] = 0;
	res[1] = 0;
	res[2] = 0;
}

DE_MATH_API void deNegV3V3(deFloat* res, const deFloat* v1)
{
	res[0] = -v1[0];
	res[1] = -v1[1];
	res[2] = -v1[2];
}

DE_MATH_API void deAddV3V3(deFloat* res, const deFloat* v1)
{
	res[0] += v1[0];
	res[1] += v1[1];
	res[2] += v1[2];
}

DE_MATH_API void deSubV3V3(deFloat* res, const deFloat* v1)
{
	res[0] -= v1[0];
	res[1] -= v1[1];
	res[2] -= v1[2];
}

DE_MATH_API void deMulV3V3(deFloat* res, const deFloat* v1)
{
	res[0] *= v1[0];
	res[1] *= v1[1];
	res[2] *= v1[2];
}

DE_MATH_API void deAddV3V3S1(deFloat* res, const deFloat* v1, const deFloat s)
{
	res[0] = v1[0] + s;
	res[1] = v1[1] + s;
	res[2] = v1[2] + s;
}

DE_MATH_API void deMulV3V3S1(deFloat* res, const deFloat* v1, const deFloat s)
{
	res[0] = v1[0] * s;
	res[1] = v1[1] * s;
	res[2] = v1[2] * s;
}


DE_MATH_API void deAddV3V3V3(deFloat* res, const deFloat* v1, const deFloat* v2)
{
	res[0] = v1[0] + v2[0];
	res[1] = v1[1] + v2[1];
	res[2] = v1[2] + v2[2];
}

DE_MATH_API void deSubV3V3V3(deFloat* res, const deFloat* v1, const deFloat* v2)
{
    res[0] = v1[0] - v2[0];
    res[1] = v1[1] - v2[1];
    res[2] = v1[2] - v2[2];
}

DE_MATH_API void deMulV3V3V3(deFloat* res, const deFloat* v1, const deFloat* v2)
{
	res[0] = v1[0] * v2[0];
	res[1] = v1[1] * v2[1];
	res[2] = v1[2] * v2[2];
}

DE_MATH_API void deCrossV3V3V3(deFloat* res, const deFloat* v1, const deFloat* v2)
{
	res[0] = v1[1] * v2[2] - v1[2] * v2[1];
	res[1] = v1[2] * v2[0] - v1[0] * v2[2];
	res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

DE_MATH_API deFloat deMagnitudeV3(const deFloat* v1)
{
	return deSqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);
}

DE_MATH_API void deNormalizeV3(deFloat* res)
{
	deFloat mag = 1 / deSqrt(res[0] * res[0] + res[1] * res[1] + res[2] * res[2]);
	res[0] *= mag;
	res[1] *= mag;
	res[2] *= mag;
}

DE_MATH_API void deMinV3V3(deFloat* res, const deFloat* v1)
{
	if (v1[0] < res[0]) res[0] = v1[0];
	if (v1[1] < res[1]) res[1] = v1[1];
	if (v1[2] < res[2]) res[2] = v1[2];
}

DE_MATH_API void deMaxV3V3(deFloat* res, const deFloat* v1)
{
	if (v1[0] > res[0]) res[0] = v1[0];
	if (v1[1] > res[1]) res[1] = v1[1];
	if (v1[2] > res[2]) res[2] = v1[2];
}

/* 
 * res = v1 + t * (v2 - v1)
 */
DE_MATH_API void deLerpV3V3V3S1(deFloat* res, const deFloat* v1, const deFloat* v2, const deFloat t)
{
  res[0] = v1[0] + t * (v2[0] - v1[0]);
  res[1] = v1[1] + t * (v2[1] - v1[1]);
  res[2] = v1[2] + t * (v2[2] - v1[2]);
}

#ifdef __cplusplus
}
#endif

#endif /* _deVector3f_h */

