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

#ifndef _deVector3_h
#define _deVector3_h
/*!
 *	\brief		3x1 vector class
 *	\ingroup	deMath
 *
 *	This is a C++ wrapper class of deVector3f.
 *	\sa	deVector3f.h
 */
class deVector3
{
public:
  /** default ctor zeros all elements */
  inline deVector3() { zero(); }
  
  /** init elements from three ctor args */
  inline deVector3(deFloat v0, deFloat v1, deFloat v2)
  { _data[0] = v0; _data[1] = v1; _data[2] = v2; }
  
  inline deVector3(deVector3 const & orig) {
    for (size_t ii(0); ii < DE_VECTOR3_SIZE; ++ii)
      _data[ii] = orig._data[ii];
  }
  
	//! return this[i]
	deFloat& operator[](const deInt i) { return _data[i]; }
	//! return const this[i]
	const deFloat& operator[](const deInt i) const { return _data[i]; }
	//! (deVector3& )this
	operator deFloat*() { return _data; }
	//! (const deVector3& )this
	operator const deFloat*() const { return _data; }
	//! return this[i]
	deFloat& elementAt(const deInt i) { return _data[i]; }
	//! return const this[i]
	const deFloat& elementAt(const deInt i) const { return _data[i]; }

	//! this = v;
	DE_MATH_API void operator=(const deVector3& v);
	//! return (this == v)
	DE_MATH_API deInt operator==(const deVector3& v) const;
	//! this += v
	DE_MATH_API void operator+=(const deVector3& v);
	//! this -= v
	DE_MATH_API void operator-=(const deVector3& v);
	//! this[i] *= v[i]
	DE_MATH_API void operator*=(const deVector3& v);
	//! this[i] *= s
	DE_MATH_API void operator*=(const deFloat s);
	//! this[i] += s
	DE_MATH_API void operator+=(const deFloat s);

	//! this = 0
	DE_MATH_API void zero();

	//! return this^T * v
	DE_MATH_API deFloat dot(const deVector3& v) const;
	//! return sqrt(this^T * this)
	DE_MATH_API deFloat magnitude() const;
	//! this[i] *= (1 / magnitude())
	DE_MATH_API void normalize();
	//! this = (x, y, z)
	DE_MATH_API void set(const deFloat x, const deFloat y, const deFloat z);
	//! this = [x y z]
	DE_MATH_API void set(const deFloat* v);
	//! [x y z] = this
	DE_MATH_API void get(deFloat* v) const;

	//! this[i] = min(this[i], v[i])
	DE_MATH_API void minimum(const deVector3& v);
	//! this[i] = max(this[i], v[i])
	DE_MATH_API void maximum(const deVector3& v);

	//! this = -v
	DE_MATH_API void negate(const deVector3& v);
	//! this = v1 + v2
	DE_MATH_API void add(const deVector3& v1, const deVector3& v2);
	//! this = v1 - v2
	DE_MATH_API void subtract(const deVector3& v1, const deVector3& v2);
	//! this[i] = v1[i] * v2[i]
	DE_MATH_API void multiply(const deVector3& v1, const deVector3& v2);
	//! this[i] = v[i] * s
	DE_MATH_API void multiply(const deVector3& v, const deFloat s);
	//! this[i] = v[i] + s
	DE_MATH_API void add(const deVector3& v, const deFloat s);
	//! this = v1 x v2
	DE_MATH_API void crossMultiply(const deVector3& v1, const deVector3& v2);
	//! this = m * v
	DE_MATH_API void multiply(const deMatrix3& m, const deVector3& v);
	//! this = m^T * v
	DE_MATH_API void transposedMultiply(const deMatrix3& m, const deVector3& v);
	//! this = [r,p]v = r*v + p
	DE_MATH_API void multiply(const deTransform& t, const deVector3& v);
	//! this = ~[r,p]*v = [~r, -(~r*p)]*v = ~r*v -~r*p = ~r*(v-p)
	DE_MATH_API void inversedMultiply(const deTransform& t, const deVector3& v);
	//! this = diag(m)
	DE_MATH_API void diagonal(const deMatrix3& m);
	//! this = col of m
	DE_MATH_API void column(const deMatrix3& m, const deInt col);
	//! this = dPhi = R - Rd
	DE_MATH_API void angularError(const deMatrix3& R, const deMatrix3& Rd);
	//! this = XYZ Euler angles of m
	DE_MATH_API void eulerXYZ(const deMatrix3& m);
	//! this = ZYX Euler angles of m
	DE_MATH_API void eulerZYX(const deMatrix3& m);
	//! this = ZYX Euler angles of m using v for singularity
	DE_MATH_API void eulerZYX(const deMatrix3& m, const deVector3& v);
	//! this = ZYX Euler angles of q
	DE_MATH_API void eulerZYX(const deQuaternion& q);
	//! this = x where y = LU x
	DE_MATH_API void backSub(const deMatrix3& LU, const deVector3& y);
	//! this = q * v
	DE_MATH_API void multiply(const deQuaternion& q, const deVector3& v);
	//! this = q^-1 * v
	DE_MATH_API void inversedMultiply(const deQuaternion& q, const deVector3& v);
	//! this = col of q
	DE_MATH_API void column(const deQuaternion& q, const deInt col);
	// dPhi = Einv ( q - qd) = -2*q_tilde^T qd <-- since (q_tilde^T q) = 0
	// E = 0.5*q_tilde
	// Einv = (EtE)inv Et = 4Et = 2*qtilde^T <-- since (q_tilde^T q_tilde) = I
	// q_tilde = [ -x -y -z; w -z  y; z  w -x; -y  x  w ] 
	// q_tilde^T = [ -x  w  z -y; -y -z  w  x; -z  y -x  w ] 	       
	DE_MATH_API void angularError(const deQuaternion& q, const deQuaternion& qd);
	//! this = [r,p]v = r*v + p
	DE_MATH_API void multiply(const deFrame& f, const deVector3& v);
	//! this = ~[r,p]*v = [~r, -(~r*p)]*v = ~r*v -~r*p = ~r*(v-p)
	DE_MATH_API void inversedMultiply(const deFrame& f, const deVector3& v);

	//! this = LERP
	// this = v + t * (vg - v)
	DE_MATH_API void lerp(const deVector3& v, const deVector3& vg, const deFloat t);

private:
	deFloat _data[DE_VECTOR3_SIZE];
};

#endif // _deVector3_h
