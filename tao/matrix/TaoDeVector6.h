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

#ifndef _deVector6_h
#define _deVector6_h
/*!
 *	\brief		6x1 vector class 
 *	\ingroup	deMath
 *
 *	This class consists of two 3x1 vectors.
 *	\remarks	spatial vector representing translational part and rotational part
 *	\sa deVector3
 */
class deVector6
{
public:
	//!	\return	3x1 vector
	DE_MATH_API deVector3& operator[](deInt row);
	//!	\return	3x1 vector
	DE_MATH_API const deVector3& operator[](deInt row) const;
	//!	\return	element at i
	DE_MATH_API deFloat& elementAt(deInt i);
	//!	\return	element at i
	DE_MATH_API const deFloat& elementAt(deInt i) const;
	//!	this = v
	DE_MATH_API void operator=(const deVector6& v);
	//!	this = zero vector
	DE_MATH_API void zero();
	//! this = -v
	DE_MATH_API void negate(const deVector6& v);
	//! this = v1 + v2
	DE_MATH_API void add(const deVector6& v1, const deVector6& v2);
	//! this = v1 - v2
	DE_MATH_API void subtract(const deVector6& v1, const deVector6& v2);
	//! this[i] = v[i] * s
	DE_MATH_API void multiply(const deVector6& v, const deFloat s);
	//! this = m * v
	DE_MATH_API void multiply(const deMatrix6& m, const deVector6& v);
	//! this = m^T * v
	DE_MATH_API void transposedMultiply(const deMatrix6& m, const deVector6& v);
	//! this = T - Td
	DE_MATH_API void error(const deTransform& T, const deTransform& Td);
	//! this = T - Td
	DE_MATH_API void error(const deFrame &F, const deFrame &Fd);
	//!	this += v
	DE_MATH_API void operator+=(const deVector6& v);
	//!	this -= v
	DE_MATH_API void operator-=(const deVector6& v);
	//!	this *= s
	DE_MATH_API void operator*=(const deFloat s);
	//!	\return	sum(this[i]*v[i])
	DE_MATH_API deFloat dot(const deVector6& v) const;
	//! this = x where y = m x
	DE_MATH_API void solve(const deMatrix6& m, const deVector6& y);
	//! this = x where y = m x and m is SPD
	DE_MATH_API void solveSPD(const deMatrix6& m, const deVector6& y);

	/*!
	 *	\name	Spatial transformation only
	 */
	//	@{
	//! this = X v
	// X = [ R 0; dxR R ]
	// Fh = X Fi
	//    = [ R fi ; dxR fi + R ni ]
	DE_MATH_API void xform(const deTransform& t, const deVector6& v);
	//! this = X^T v
	// Xt = [ Rt -Rtdx; 0 Rt ]
	// Vi = Xt Vh
	//    = [ Rtvh - Rtdxwh; Rtwh ]
	//    = [ Rt(vh - dxwh); Rtwh ]
	DE_MATH_API void xformT(const deTransform& t, const deVector6& v);
	//! this = X^-T v
	// Xtinv = [ R dxR; 0 R ]
	// Vi = Xtinv Vh
	//    = [ R vh + dxR wh;  R wh ]
	DE_MATH_API void xformInvT(const deTransform& t, const deVector6& v);
	//! this = X^-1 v
	// Xinv = [ Rt 0; -Rtdx Rt ]
	// Vh = Xinv Vi
	//    = [ Rtvi ; -Rtdxvi + Rtwi ]
	//    = [ Rtvi ; Rt(wi - dxvi) ]
	DE_MATH_API void xformInv(const deTransform& t, const deVector6& v);
	//! this = [v0;v1]X[tmp0;tmp1] = [ v1x , v0x ; 0 , v1x ] [ tmp0 ; tmp1 ]
	//                            = [ v1 x tmp0 + v0 x tmp1; v1 x tmp1 ]
	DE_MATH_API void crossMultiply(const deVector6& v1, const deVector6& v2);
	//	@}

	//! this = x where y = LU x
	void backSub(const deMatrix6& lu, const deVector6& y);
	//! this = x where y = LU x and L*U is SPD
	void backSubSPD(const deMatrix6& lu, const deVector6& y);

private:
	deVector3 _vec3[2];
};

#endif // _deVector6_h
