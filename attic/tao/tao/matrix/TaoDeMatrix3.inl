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

#ifndef _deMatrix3_inl
#define _deMatrix3_inl

DE_MATH_API deFloat deMatrix3::det() const { return dedetM3(_data); }

DE_MATH_API void deMatrix3::operator=(const deMatrix3& m) { deSetM3M3(_data, m._data); }
DE_MATH_API void deMatrix3::zero() { deZeroM3(_data); }
DE_MATH_API void deMatrix3::identity() { deIdentityM3(_data); }
DE_MATH_API void deMatrix3::negate(const deMatrix3& m) { deNegateM3M3(_data, m._data); }
DE_MATH_API void deMatrix3::add(const deMatrix3& m1, const deMatrix3& m2) { deAddM3M3M3(_data, m1._data, m2._data); }
DE_MATH_API void deMatrix3::subtract(const deMatrix3& m1, const deMatrix3& m2) { deSubM3M3M3(_data, m1._data, m2._data); }
DE_MATH_API void deMatrix3::multiply(const deMatrix3& m1, const deMatrix3& m2) { deMulM3M3M3(_data, m1._data, m2._data); }
DE_MATH_API void deMatrix3::transposedMultiply(const deMatrix3& m1, const deMatrix3& m2) { deMulM3M3tM3(_data, m1._data, m2._data); }
DE_MATH_API void deMatrix3::multiplyTransposed(const deMatrix3& m1, const deMatrix3& m2) { deMulM3M3M3t(_data, m1._data, m2._data); }
DE_MATH_API void deMatrix3::multiply(const deMatrix3& m, const deFloat s) { deMulM3M3S1(_data, m._data, s); }
DE_MATH_API void deMatrix3::operator+=(const deMatrix3& m) { deAddM3M3(_data, m._data); }
DE_MATH_API void deMatrix3::operator-=(const deMatrix3& m) { deSubM3M3(_data, m._data); }
DE_MATH_API void deMatrix3::operator*=(const deFloat s) { deMulM3S1(_data, s); }
DE_MATH_API void deMatrix3::diagonal(const deFloat x, const deFloat y, const deFloat z) { deDiagonalM3S3(_data, x, y, z); }
DE_MATH_API void deMatrix3::diagonal(const deVector3& v)  { deDiagonalM3V3(_data, v); }
DE_MATH_API void deMatrix3::eulerXYZ(const deFloat x, const deFloat y, const deFloat z) { deSetM3xyzS3(_data, x, y, z); }
DE_MATH_API void deMatrix3::eulerZYX(const deFloat x, const deFloat y, const deFloat z) { deSetM3zyxS3(_data, x, y, z); }
DE_MATH_API void deMatrix3::set(const deInt axis, const deFloat angle) { deSetM3S2(_data, axis, angle); }
DE_MATH_API void deMatrix3::set(const deVector3& axis, const deFloat angle) { deSetM3V3S1(_data, axis, angle); }
DE_MATH_API void deMatrix3::inverseDet(const deMatrix3& m) { deInvertDetM3M3(_data, m._data); }
DE_MATH_API void deMatrix3::inverseDetSPD(const deMatrix3& m) { deInvertDetSPDM3M3(_data, m._data); }
DE_MATH_API void deMatrix3::ludecomp(const deMatrix3& m) { deLUdecomposeM3M3(_data, m._data); }
DE_MATH_API void deMatrix3::transpose(const deMatrix3& m) { deTransposeM3M3(_data, m._data); }
DE_MATH_API void deMatrix3::multiplyTransposed(const deVector3& v1, const deVector3& v2) { deMulM3V3V3t(_data, v1, v2); }
DE_MATH_API void deMatrix3::crossMultiply(const deVector3& v, const deMatrix3& m) { deMulM3V3xM3(_data, v, m._data); }
DE_MATH_API void deMatrix3::cross(const deVector3& v) { deSetM3V3x(_data, v); }
DE_MATH_API void deMatrix3::multiplyCross(const deMatrix3& m, const deVector3& v) { deMulM3M3V3x(_data, m._data, v); }
DE_MATH_API void deMatrix3::set(const deQuaternion& q) { deSetM3Q4(_data, q); }
DE_MATH_API void deMatrix3::set(const deFloat a0, const deFloat a1, const deFloat a2,
								const deFloat a3, const deFloat a4, const deFloat a5,
								const deFloat a6, const deFloat a7, const deFloat a8)
							{ deSetM3S9(_data, a0, a1, a2, a3, a4, a5, a6, a7, a8); }

#endif // _deMatrix3_inl
