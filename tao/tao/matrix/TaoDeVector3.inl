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

#ifndef _deVector3_inl
#define _deVector3_inl

//! This is a C++ inline class of deVector3.

DE_MATH_API deInt deVector3::operator==(const deVector3& v) const { return deIsEqualV3V3(_data, v); }
DE_MATH_API deFloat deVector3::dot(const deVector3& v) const { return deDotV3V3(_data, v); }
DE_MATH_API deFloat deVector3::magnitude() const { return deMagnitudeV3(_data); }

DE_MATH_API void deVector3::operator=(const deVector3& v) { deSetV3V3(_data, v); }
DE_MATH_API void deVector3::operator+=(const deVector3& v) { deAddV3V3(_data, v); }
DE_MATH_API void deVector3::operator-=(const deVector3& v) { deSubV3V3(_data, v); }
DE_MATH_API void deVector3::operator*=(const deVector3& v) { deMulV3V3(_data, v); }
DE_MATH_API void deVector3::operator*=(const deFloat s) { deMulV3S1(_data, s); }
DE_MATH_API void deVector3::operator+=(const deFloat s) { deAddV3S1(_data, s); }
DE_MATH_API void deVector3::zero() { deZeroV3(_data); }
DE_MATH_API void deVector3::normalize() { deNormalizeV3(_data); }
DE_MATH_API void deVector3::set(const deFloat x, const deFloat y, const deFloat z) { deSetV3S3(_data, x, y, z); }
DE_MATH_API void deVector3::set(const deFloat* v) { deSetV3V3(_data, v); }
DE_MATH_API void deVector3::get(deFloat* v) const { deSetV3V3(v, _data); }
DE_MATH_API void deVector3::minimum(const deVector3& v) { deMinV3V3(_data, v); }
DE_MATH_API void deVector3::maximum(const deVector3& v) { deMaxV3V3(_data, v); }
DE_MATH_API void deVector3::negate(const deVector3& v) { deNegateV3V3(_data, v); }
DE_MATH_API void deVector3::add(const deVector3& v1, const deVector3& v2) { deAddV3V3V3(_data, v1, v2); }
DE_MATH_API void deVector3::subtract(const deVector3& v1, const deVector3& v2) { deSubV3V3V3(_data, v1, v2); }
DE_MATH_API void deVector3::multiply(const deVector3& v1, const deVector3& v2) { deMulV3V3V3(_data, v1, v2); }
DE_MATH_API void deVector3::multiply(const deVector3& v, const deFloat s) { deMulV3V3S1(_data, v, s); }
DE_MATH_API void deVector3::add(const deVector3& v, const deFloat s) { deAddV3V3S1(_data, v, s); }	
DE_MATH_API void deVector3::crossMultiply(const deVector3& v1, const deVector3& v2) { deCrossV3V3V3(_data, v1, v2); }
DE_MATH_API void deVector3::multiply(const deMatrix3& m, const deVector3& v) { deMulV3M3V3(_data, m._data, v); }
DE_MATH_API void deVector3::transposedMultiply(const deMatrix3& m, const deVector3& v) { deMulV3M3tV3(_data, m._data, v); }
DE_MATH_API void deVector3::multiply(const deTransform& t, const deVector3& v) {
	deMulV3M3V3(_data, t.rotation()._data, v);
	deAddV3V3(_data, t.translation());
}
DE_MATH_API void deVector3::inversedMultiply(const deTransform& t, const deVector3& v) {
	deVector3 p;
	deSubV3V3V3(p, v, t.translation());
	deMulV3M3tV3(_data, t.rotation()._data, (const deVector3&)p); // YYY
}
DE_MATH_API void deVector3::diagonal(const deMatrix3& m) { deDiagonalV3M3(_data, m._data); }
DE_MATH_API void deVector3::column(const deMatrix3& m, const deInt col) { deColumnV3M3S1(_data, m._data, col); }
DE_MATH_API void deVector3::angularError(const deMatrix3& R, const deMatrix3& Rd) { deAngularErrorV3M3M3(_data, R._data, Rd._data); }
DE_MATH_API void deVector3::eulerXYZ(const deMatrix3& m) { deSetV3M3xyz(_data, m._data); }
DE_MATH_API void deVector3::eulerZYX(const deMatrix3& m) { deSetV3M3zyx(_data, m._data); }
DE_MATH_API void deVector3::eulerZYX(const deMatrix3& m, const deVector3& v) { deSetV3M3zyxV3(_data, m._data, v); }
DE_MATH_API void deVector3::eulerZYX(const deQuaternion& q) { deSetV3Q4zyx(_data, q); }
DE_MATH_API void deVector3::backSub(const deMatrix3& LU, const deVector3& y) { deBackSubstituteV3M3V3(_data, LU._data, y); }
DE_MATH_API void deVector3::multiply(const deQuaternion& q, const deVector3& v) { deMulV3Q4V3(_data, q, v); }
DE_MATH_API void deVector3::inversedMultiply(const deQuaternion& q, const deVector3& v) { deMulV3Q4iV3(_data, q, v); }
DE_MATH_API void deVector3::column(const deQuaternion& q, const deInt col) { deColumnV3Q4S1(_data, q, col); }
DE_MATH_API void deVector3::angularError(const deQuaternion& q, const deQuaternion& qd) { deAngularErrorV3Q4Q4(_data, q, qd); }
//! this = [r,p]v = r*v + p
DE_MATH_API void deVector3::multiply(const deFrame& f, const deVector3& v) { 
	deMulV3Q4V3(_data, f.rotation(), v);
	deAddV3V3(_data, f.translation());
}
//! this = ~[r,p]*v = [~r, -(~r*p)]*v = ~r*v -~r*p = ~r*(v-p)
DE_MATH_API void deVector3::inversedMultiply(const deFrame& f, const deVector3& v) {
	deVector3 p;
	deSubV3V3V3(p, v, f.translation());
	deMulV3Q4iV3(_data, f.rotation(), (const deVector3&)p); // YYY
}
DE_MATH_API void deVector3::lerp(const deVector3& v, const deVector3& vg, const deFloat t) { deLerpV3V3V3S1(_data, v, vg, t); }

#endif // _deVector3_inl
