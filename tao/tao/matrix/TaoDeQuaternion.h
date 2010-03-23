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

#ifndef _deQuaternion_h
#define _deQuaternion_h

/*!
 *	\brief		Quaternion class
 *	\ingroup	deMath
 *
 *	This is a C++ wrapper class of deQuaternionf.
 *	\remarks	q = [vx, vy, vz, w] = [axis*sin(theta/2), cos(theta/2)] <--- axisAngle(axis,theta).
 *			quaternion is used to represent rotation in deFrame.
 *	\sa	deQuaternionf.h, deFrame
 */
class deQuaternion
{
public:
  /** Default ctor calls identity() on itself. */
  inline deQuaternion() { identity(); }
  
  /** Initialization from the four quaternion values. */
  inline deQuaternion(deFloat qx, deFloat qy, deFloat qz, deFloat qw) { set(qx, qy, qz, qw); }
  
  inline deQuaternion(deQuaternion const & orig) {
    for (size_t ii(0); ii < DE_QUATERNION_SIZE; ++ii)
      _data[ii] = orig._data[ii];
  }
  
	//! cast operator
	operator deFloat*() { return _data; }
	//! cast operator
	operator const deFloat*() const { return _data; }
	//! \return q[i]
	deFloat& operator[](const deInt i) { return _data[i]; }
	//! \return const q[i]
	const deFloat& operator[](const deInt i) const { return _data[i]; }
	//! this = q
	DE_MATH_API void operator=(const deQuaternion& q);
	//! \return (this == q )
	DE_MATH_API deInt operator==(const deQuaternion& q);
	//! this[i] += q[i]
	DE_MATH_API void operator+=(const deQuaternion& q);
	//! this[i] -= q[i]
	DE_MATH_API void operator-=(const deQuaternion& q);
	//! this[i] *= s
	DE_MATH_API void operator*=(const deFloat s);
	//! this = (0, 0, 0, 1)
	DE_MATH_API void identity();
	//! this = (0, 0, 0, 0)
	DE_MATH_API void zero();
	//! return this^T * q
	DE_MATH_API deFloat dot(const deQuaternion& q);
	//! this[i] *= (1 / sqrt(this^T * this))
	DE_MATH_API void normalize();
	//! convert to axis-angle notation
	DE_MATH_API void get(deVector3& axis, deFloat &angle) const;
	//! this = m
	DE_MATH_API void set(const deMatrix3& m);
	//! this = (axis, angle)
	DE_MATH_API void set(const deInt axis, const deFloat angle);
	//! this = (axis, angle)
	DE_MATH_API void set(const deVector3& axis, const deFloat angle);
	//! this = (x, y, z, q)
	DE_MATH_API void set(const deFloat x, const deFloat y, const deFloat z, const deFloat w);
	//! this = [x y z q]
	DE_MATH_API void set(const deFloat* q);
	//! this = [x, y, z] = ZYX Euler angles
	DE_MATH_API void eulerZYX(const deVector3& v);
	//! this = -q
	DE_MATH_API void negate(const deQuaternion& q);
	//! this = q^-1
	DE_MATH_API void inverse(const deQuaternion& q);
	//! this = q1 + q2
	DE_MATH_API void add(const deQuaternion& q1, const deQuaternion& q2);
	//! this = q1 - q2
	DE_MATH_API void subtract(const deQuaternion& q1, const deQuaternion& q2);
	//! this = q1 * q2
	DE_MATH_API void multiply(const deQuaternion& q1, const deQuaternion& q2);
	//! this = q1^-1 * q2
	DE_MATH_API void inversedMultiply(const deQuaternion& q1, const deQuaternion& q2);
	//! this = q1 * q2^-1
	DE_MATH_API void multiplyInversed(const deQuaternion& q1, const deQuaternion& q2);
	//! this = dq
	// dq = E omega
	// dq = 0.5 q_tilde omega
	DE_MATH_API void velocity(const deQuaternion& q, const deVector3& omega);
	//! this = converted qg sign consitent to q
	/*! 
	 *	\remarks this funtion compares q to qg and put sign consistent qg to this (= qg or -qg).
	 *	\remarks (q is equivalent to -q) for all angles (theta) where q = [cos(theta/2), v*sin(theta/2)]
	 */
	DE_MATH_API void consistentSign(const deQuaternion& q, const deQuaternion& qg);
	//! this = SLERP (spherical linear interpolation with extra spins)
	DE_MATH_API void slerp(const deQuaternion& q, const deQuaternion& qg, const deFloat t, const deFloat addedSpins);
	//!	this = q + t * (qg - q)
	/*!	\remarks this = LERP (linear interpolation)
	 */
	DE_MATH_API void lerp(const deQuaternion& q, const deQuaternion& qg, const deFloat t);

private:
	deFloat _data[DE_QUATERNION_SIZE];
};

#endif // _deQuaternion_h
