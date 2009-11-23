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

#ifndef _deVector6_inl
#define _deVector6_inl

DE_MATH_API deVector3& deVector6::operator[](deInt row) { return _vec3[row]; }
DE_MATH_API const deVector3& deVector6::operator[](deInt row) const { return _vec3[row]; }

DE_MATH_API deFloat& deVector6::elementAt(deInt i) { return (i < 3) ? _vec3[0][i] : _vec3[1][i - 3]; }
DE_MATH_API const deFloat& deVector6::elementAt(deInt i) const { return (i < 3) ? _vec3[0][i] : _vec3[1][i - 3]; }


DE_MATH_API deFloat deVector6::dot(const deVector6& v) const { return (_vec3[0].dot(v._vec3[0]) + _vec3[1].dot(v._vec3[1])); }

DE_MATH_API void deVector6::operator=(const deVector6& v) { _vec3[0] = v._vec3[0]; _vec3[1] = v._vec3[1]; }
DE_MATH_API void deVector6::zero() { _vec3[0].zero(); _vec3[1].zero(); }
DE_MATH_API void deVector6::negate(const deVector6& v) { _vec3[0].negate(v._vec3[0]); _vec3[1].negate(v._vec3[1]); }
DE_MATH_API void deVector6::add(const deVector6& v1,const deVector6& v2) { _vec3[0].add(v1._vec3[0], v2._vec3[0]); _vec3[1].add(v1._vec3[1], v2._vec3[1]); }
DE_MATH_API void deVector6::subtract(const deVector6& v1,const deVector6& v2) { _vec3[0].subtract(v1._vec3[0], v2._vec3[0]); _vec3[1].subtract(v1._vec3[1], v2._vec3[1]); }
DE_MATH_API void deVector6::multiply(const deVector6& v, const deFloat s) { _vec3[0].multiply(v._vec3[0], s); _vec3[1].multiply(v._vec3[1], s); }
DE_MATH_API void deVector6::multiply(const deMatrix6& m, const deVector6& v) {
	deVector3 tmpV3;
	_vec3[0].multiply(m[0][0], v._vec3[0]); tmpV3.multiply(m[0][1], v._vec3[1]); _vec3[0] += tmpV3;
	_vec3[1].multiply(m[1][0], v._vec3[0]); tmpV3.multiply(m[1][1], v._vec3[1]); _vec3[1] += tmpV3;
}
DE_MATH_API void deVector6::transposedMultiply(const deMatrix6& m, const deVector6& v) {
	deVector3 tmpV3;
	_vec3[0].transposedMultiply(m[0][0], v._vec3[0]); tmpV3.transposedMultiply(m[1][0], v._vec3[1]); _vec3[0] += tmpV3;
	_vec3[1].transposedMultiply(m[0][1], v._vec3[0]); tmpV3.transposedMultiply(m[1][1], v._vec3[1]); _vec3[1] += tmpV3;
}
DE_MATH_API void deVector6::error(const deTransform& T, const deTransform& Td) { _vec3[0].subtract(T.translation(), Td.translation()); _vec3[1].angularError(T.rotation(), Td.rotation()); }
DE_MATH_API void deVector6::error(const deFrame &F, const deFrame& Fd)  { _vec3[0].subtract(F.translation(), Fd.translation()); _vec3[1].angularError(F.rotation(), Fd.rotation()); }
DE_MATH_API void deVector6::operator+=(const deVector6& v) { _vec3[0] += v._vec3[0]; _vec3[1] += v._vec3[1]; }
DE_MATH_API void deVector6::operator-=(const deVector6& v) { _vec3[0] -= v._vec3[0]; _vec3[1] -= v._vec3[1]; }
DE_MATH_API void deVector6::operator*=(const deFloat s) { _vec3[0] *= s; _vec3[1] *= s; }
DE_MATH_API void deVector6::solve(const deMatrix6& m, const deVector6& y) { deMatrix6 lu; lu.ludecomp(m); backSub(lu,y); }
DE_MATH_API void deVector6::solveSPD(const deMatrix6& m, const deVector6& y) { deMatrix6 lu; lu.ludecompSPD(m); backSubSPD(lu,y); }
DE_MATH_API void deVector6::xform(const deTransform& t, const deVector6& v) {
	_vec3[0].multiply(t.rotation(), v._vec3[0]);
	_vec3[1].multiply(t.rotation(), v._vec3[1]);
	deVector3 tmpV;
	tmpV.crossMultiply(t.translation(), _vec3[0]); 
	_vec3[1] += tmpV;
}
DE_MATH_API void deVector6::xformT(const deTransform& t, const deVector6& v) {
	_vec3[0].crossMultiply(t.translation(), v._vec3[1]);
	_vec3[1].subtract(v._vec3[0], _vec3[0]);
	_vec3[0].transposedMultiply(t.rotation(), _vec3[1]);
	_vec3[1].transposedMultiply(t.rotation(), v._vec3[1]);
}
DE_MATH_API void deVector6::xformInvT(const deTransform& t, const deVector6& v) {
	_vec3[0].multiply(t.rotation(), v._vec3[0]);
	_vec3[1].multiply(t.rotation(), v._vec3[1]);
	deVector3 tmpV;
	tmpV.crossMultiply(t.translation(), _vec3[1]); 
	_vec3[0] += tmpV;
}
DE_MATH_API void deVector6::xformInv(const deTransform& t, const deVector6& v) {
	_vec3[1].crossMultiply(t.translation(), v._vec3[0]);
	_vec3[0].subtract(v._vec3[1], _vec3[1]);
	_vec3[1].transposedMultiply(t.rotation(), _vec3[0]);
	_vec3[0].transposedMultiply(t.rotation(), v._vec3[0]);
}
DE_MATH_API void deVector6::crossMultiply(const deVector6& v1, const deVector6& v2) {
	_vec3[1].crossMultiply(v1._vec3[0], v2._vec3[1]);
	_vec3[0].crossMultiply(v1._vec3[1], v2._vec3[0]);
	_vec3[0] += _vec3[1];
	_vec3[1].crossMultiply(v1._vec3[1], v2._vec3[1]);
}

#endif // _deVector6_inl
