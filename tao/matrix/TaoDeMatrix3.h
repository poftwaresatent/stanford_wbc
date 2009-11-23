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

#ifndef _deMatrix3_h
#define _deMatrix3_h
/*!
 *	\brief		3x3 matrix class
 *	\ingroup	deMath
 *
 *	This is a C++ wrapper class of deMatrix3f.
 *	\remarks	this matrix is also used as a rotation matrix in deTransformation.
 *	\sa deMatrix3f.h, deQuaternion, deTransform
 */
class deMatrix3
{
public:
  /** default ctor zeros all elements */
  inline deMatrix3() { zero(); }
  
  /** \note BEWARE that the underlying data is not necessarily 3x3,
      the actual dimensions depend on the precompiler switches in
      TaoDeMath.h! Thus, it is more portable to use elementAt() for
      accessing the matrix elements. */
 	operator deFloat*() { return _data[0]; }

  /** \note BEWARE that the underlying data is not necessarily 3x3,
      the actual dimensions depend on the precompiler switches in
      TaoDeMath.h! Thus, it is more portable to use elementAt() for
      accessing the matrix elements. */
	operator const deFloat*() const { return _data[0]; }

	//! \return	row
	deFloat* operator[](const deInt row) { return _data[row]; }
	//! \return	row
	const deFloat* operator[](const deInt row) const { return _data[row]; }
	//! \return	this[i][j]
	deFloat& elementAt(const deInt i, const deInt j) { return _data[i][j]; }
	//! \return	this[i][j]
	const deFloat& elementAt(const deInt i, const deInt j) const { return _data[i][j]; }
	//!	this = m
	DE_MATH_API void operator=(const deMatrix3& m);
	//!	this = zero matrix
	DE_MATH_API void zero();
	//!	this = identity matrix
	DE_MATH_API void identity();
	//! this = -m
	DE_MATH_API void negate(const deMatrix3& m);
	//! this = m1 + m2
	DE_MATH_API void add(const deMatrix3& m1, const deMatrix3& m2);
	//! this = m1 - m2
	DE_MATH_API void subtract(const deMatrix3& m1, const deMatrix3& m2);
	//! this = m1 * m2
	DE_MATH_API void multiply(const deMatrix3& m1, const deMatrix3& m2);
	//! this = m1^T * m2
	DE_MATH_API void transposedMultiply(const deMatrix3& m1, const deMatrix3& m2);
	//!	this = m1 * m2^T
	DE_MATH_API void multiplyTransposed(const deMatrix3& m1, const deMatrix3& m2);
	//!	this = m1 * m2^T
	DE_MATH_API void multiply(const deMatrix3& m, const deFloat s);
	//!	this += m
	DE_MATH_API void operator+=(const deMatrix3& m);
	//!	this -= m
	DE_MATH_API void operator-=(const deMatrix3& m);
	//!	this *= s
	DE_MATH_API void operator*=(const deFloat s);
	//! diag(this) = (x, y, z), offdiag(this) = 0
	DE_MATH_API void diagonal(const deFloat x, const deFloat y, const deFloat z);
	//! diag(this) = v, offdiag(this) = 0
	DE_MATH_API void diagonal(const deVector3& v);

	/*!	
	 *	\name Rotation matrix only
	 */
	// @{
	//! this = X-Y-Z Euler angles
	DE_MATH_API void eulerXYZ(const deFloat x, const deFloat y, const deFloat z);
	//! this = Z-Y-X Euler angles
	DE_MATH_API void eulerZYX(const deFloat x, const deFloat y, const deFloat z);
	//! this = q
	DE_MATH_API void set(const deQuaternion& q);
	//! this = rotation matrix of axis-angle representation
	DE_MATH_API void set(const deInt axis, const deFloat angle); 
	//! this = rotation matrix of axis-angle representation
	DE_MATH_API void set(const deVector3& axis, const deFloat angle);
	//! this = [a0 ... a8]
	DE_MATH_API void set(const deFloat a0, const deFloat a1, const deFloat a2,
			const deFloat a3, const deFloat a4, const deFloat a5,
			const deFloat a6, const deFloat a7, const deFloat a8);
	// @}

	//! \return det(this)
	DE_MATH_API deFloat det() const;
	//! this = m^-1 using determinent
	DE_MATH_API void inverseDet(const deMatrix3& m);
	//! this = m^-1 using determinent where m is SPD
	DE_MATH_API void inverseDetSPD(const deMatrix3& m);
	//! this = LU decomposition of m
	DE_MATH_API void ludecomp(const deMatrix3& m);
	//! this = m^T
	DE_MATH_API void transpose(const deMatrix3& m);
	//! this = v1 * v2^T
	DE_MATH_API void multiplyTransposed(const deVector3& v1, const deVector3& v2);
	//! this = (v x) * m
	DE_MATH_API void crossMultiply(const deVector3& v, const deMatrix3& m);
	//! this = (v x)
	DE_MATH_API void cross(const deVector3& v);
	//! this = m * (v x)
	DE_MATH_API void multiplyCross(const deMatrix3& m, const deVector3& v);

	friend class deVector3;
	friend class deQuaternion;

private:
	deFloat _data[DE_MATRIX3_ROW][DE_MATRIX3_COL];
};

#endif // _deMatrix3_h
