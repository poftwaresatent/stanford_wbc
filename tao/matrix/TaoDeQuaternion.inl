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

#ifndef _deQuaternion_inl
#define _deQuaternion_inl

DE_MATH_API deInt deQuaternion::operator==(const deQuaternion& q) { return deIsEqualQ4Q4(_data, q); }
DE_MATH_API deFloat deQuaternion::dot(const deQuaternion& q) { return deDotQ4Q4(_data, q); }

DE_MATH_API void deQuaternion::operator=(const deQuaternion& q) { deSetQ4Q4(_data, q); }
DE_MATH_API void deQuaternion::operator+=(const deQuaternion& q) { deAddQ4Q4(_data, q); }
DE_MATH_API void deQuaternion::operator-=(const deQuaternion& q) { deSubQ4Q4(_data, q); }
DE_MATH_API void deQuaternion::operator*=(const deFloat s) { deMulQ4S1(_data, s); }
DE_MATH_API void deQuaternion::identity() { deIdentityQ4(_data); }
DE_MATH_API void deQuaternion::zero() { deZeroQ4(_data); }
DE_MATH_API void deQuaternion::normalize() { deNormalizeQ4(_data); }
DE_MATH_API void deQuaternion::get(deVector3& axis, deFloat &angle) const { deAxisAngleV3S1Q4(axis, &angle, _data); }
DE_MATH_API void deQuaternion::set(const deMatrix3& m) { deSetQ4M3(_data, m._data); }
DE_MATH_API void deQuaternion::set(const deInt axis, const deFloat angle) { deSetQ4S2(_data, axis, angle); }
DE_MATH_API void deQuaternion::set(const deVector3& axis, const deFloat angle) { deSetQ4V3S1(_data, axis, angle); }
DE_MATH_API void deQuaternion::set(const deFloat x, const deFloat y, const deFloat z, const deFloat w) { deSetQ4S4(_data, x, y, z, w); }
DE_MATH_API void deQuaternion::set(const deFloat* q) { deSetQ4Q4(_data, q); }
DE_MATH_API void deQuaternion::eulerZYX(const deVector3& v) { deSetQ4zyxV3(_data, v); }
DE_MATH_API void deQuaternion::negate(const deQuaternion& q) { deNegateQ4Q4(_data, q); }
DE_MATH_API void deQuaternion::inverse(const deQuaternion& q) { deInvertQ4Q4(_data, q); }
DE_MATH_API void deQuaternion::add(const deQuaternion& q1, const deQuaternion& q2) { deAddQ4Q4Q4(_data, q1, q2); }
DE_MATH_API void deQuaternion::subtract(const deQuaternion& q1, const deQuaternion& q2) { deSubQ4Q4Q4(_data, q1, q2); }
DE_MATH_API void deQuaternion::multiply(const deQuaternion& q1, const deQuaternion& q2) { deMulQ4Q4Q4(_data, q1, q2); }
DE_MATH_API void deQuaternion::inversedMultiply(const deQuaternion& q1, const deQuaternion& q2) { deMulQ4Q4iQ4(_data, q1, q2); }
DE_MATH_API void deQuaternion::multiplyInversed(const deQuaternion& q1, const deQuaternion& q2) { deMulQ4Q4Q4i(_data, q1, q2); }
DE_MATH_API void deQuaternion::velocity(const deQuaternion& q, const deVector3& omega) { deVelocityQ4Q4V3(_data, q, omega); }
DE_MATH_API void deQuaternion::consistentSign(const deQuaternion& q, const deQuaternion& qg) { deConsistentSignQ4Q4Q4(_data, q, qg); }
DE_MATH_API void deQuaternion::slerp(const deQuaternion& q, const deQuaternion& qg, const deFloat t, const deFloat addedSpins) { deSlerpQ4Q4Q4S2(_data, q, qg, t, addedSpins); }
DE_MATH_API void deQuaternion::lerp(const deQuaternion& q, const deQuaternion& qg, const deFloat t) { deLerpQ4Q4Q4S1(_data, q, qg, t); }

#endif // _deQuaternion_inl
