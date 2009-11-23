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

#ifndef _deMatrix6_h
#define _deMatrix6_h
/*!
 *	\brief		6x6 matrix class 
 *	\ingroup	deMath
 *
 *	This class consists of four 3x3 matrices.
 *	\remarks	spatial matrix representing translational part and rotational part
 *	\sa deMatrix3
 */
class deMatrix6
{
public:
	//!	\return	row
	deMatrix3* operator[](const deInt row) { return (_mat3 + row * 2); }
	//!	\return	row
	const deMatrix3* operator[](const deInt row) const { return (_mat3 + row * 2); }
	//!	\return	element at (i,j)	
	DE_MATH_API deFloat& elementAt(const deInt i, const deInt j);
	//!	\return	element at (i,j)	
	DE_MATH_API const deFloat& elementAt(const deInt i, const deInt j) const;
	//!	this = m
	DE_MATH_API void operator=(const deMatrix6& m);
	//!	this = zero matrix
	DE_MATH_API void zero();
	//!	this = identity matrix
	DE_MATH_API void identity();
	//! this = -m
	DE_MATH_API void negate(const deMatrix6& m);
	//! this = m1 + m2
	DE_MATH_API void add(const deMatrix6& m1, const deMatrix6& m2);
	//! this = m1 - m2
	DE_MATH_API void subtract(const deMatrix6& m1, const deMatrix6& m2);
	//! this[i] = m[i] * s
	DE_MATH_API void multiply(const deMatrix6& m, const deFloat s);
	//! this += m
	DE_MATH_API void operator+=(const deMatrix6& m);
	//! this -= m
	DE_MATH_API void operator-=(const deMatrix6& m);
	//! this[i] -= s
	DE_MATH_API void operator*=(const deFloat s);
	//! this = m^T
	DE_MATH_API void transpose(const deMatrix6& m);
	//! this = m^-1
	void inverse(const deMatrix6& m);
	//! this = m^-1 where m is SPD
	void inverseSPD(const deMatrix6& m);
	//! this = LU decomposition of m
	void ludecomp(const deMatrix6& m);
	//! this = LU decomposition of m where m is SPD
	void ludecompSPD(const deMatrix6& m);
	//! this = m1 * m2
	void multiply(const deMatrix6& m1, const deMatrix6& m2);
	//! this = m1^T * m2
	void transposedMultiply(const deMatrix6& m1, const deMatrix6& m2);
	//! this = m1 * m2^T
	void multiplyTransposed(const deMatrix6& m1, const deMatrix6& m2);
	//! this = v1 * v2^T
	void multiplyTransposed(const deVector6& v1, const deVector6& v2);
	//! this = L * I * L^T : I is symmetric
	void similarityXform(const deMatrix6& L, const deMatrix6& I);
	//! this = L^T * I * L : I is symmetric
	void similarityXformT(const deMatrix6& L, const deMatrix6& I);
	/*!
	 *	\name	Spatial transformation only
	 */
	//	@{
	//! this = X where X = [R 0; dxR R]
	DE_MATH_API void set(const deTransform& t);
	//! this = X * m
	void xform(const deTransform& t, const deMatrix6& m);
	//! this = Li = R Li+1 Rt : L symmetric
	void similarityRform(const deTransform& t, const deMatrix6& L);
	//! this =  Ii = X Ii+1 Xt : I symmetric
	void similarityXform(const deTransform& t, const deMatrix6& I);
	//! this = Ii+1 = Xt Ii X : I symmetric
	void similarityXformT(const deTransform& t, const deMatrix6& I);
	//! this =  Li+1 = Xinv Li Xinvt : L symmetric
	void similarityXformInv(const deTransform& t, const deMatrix6& I);
	//	@}

private:
	deMatrix3 _mat3[4];
};

#endif // _deMatrix6_h
