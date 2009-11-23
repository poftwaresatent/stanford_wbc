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

#ifndef _deMatrix6_inl
#define _deMatrix6_inl

DE_MATH_API deFloat& deMatrix6::elementAt(const deInt i, const deInt j) { 
	return _mat3[(i / 3) * 2 + (j / 3)].elementAt(i % 3, j % 3); 
}
DE_MATH_API const deFloat& deMatrix6::elementAt(const deInt i, const deInt j) const {
	return _mat3[(i / 3) * 2 + (j / 3)].elementAt(i % 3, j % 3); 
}
DE_MATH_API void deMatrix6::operator=(const deMatrix6& m) {
	_mat3[0] = m._mat3[0];	_mat3[1] = m._mat3[1];
	_mat3[2] = m._mat3[2];	_mat3[3] = m._mat3[3];
}
DE_MATH_API void deMatrix6::zero() {
	_mat3[0].zero();	_mat3[1].zero();
	_mat3[2].zero();	_mat3[3].zero();
}
DE_MATH_API void deMatrix6::identity() {
	_mat3[0].identity();	_mat3[1].zero();
	_mat3[2].zero();		_mat3[3].identity();
}
DE_MATH_API void deMatrix6::negate(const deMatrix6& m) {
	_mat3[0].negate(m._mat3[0]);	_mat3[1].negate(m._mat3[1]);
	_mat3[2].negate(m._mat3[2]);	_mat3[3].negate(m._mat3[3]);
}
DE_MATH_API void deMatrix6::add(const deMatrix6& m1, const deMatrix6& m2) {
	_mat3[0].add(m1._mat3[0], m2._mat3[0]);	_mat3[1].add(m1._mat3[1], m2._mat3[1]);
	_mat3[2].add(m1._mat3[2], m2._mat3[2]);	_mat3[3].add(m1._mat3[3], m2._mat3[3]);
}
DE_MATH_API void deMatrix6::subtract(const deMatrix6& m1, const deMatrix6& m2) {
	_mat3[0].subtract(m1._mat3[0], m2._mat3[0]);	_mat3[1].subtract(m1._mat3[1], m2._mat3[1]);
	_mat3[2].subtract(m1._mat3[2], m2._mat3[2]);	_mat3[3].subtract(m1._mat3[3], m2._mat3[3]);
}
DE_MATH_API void deMatrix6::multiply(const deMatrix6& m, const deFloat s) {
	_mat3[0].multiply(m._mat3[0], s);	_mat3[1].multiply(m._mat3[1], s);
	_mat3[2].multiply(m._mat3[2], s);	_mat3[3].multiply(m._mat3[3], s);
}
DE_MATH_API void deMatrix6::operator+=(const deMatrix6& m) {
	_mat3[0] += m._mat3[0];	_mat3[1] += m._mat3[1];
	_mat3[2] += m._mat3[2];	_mat3[3] += m._mat3[3];
}
DE_MATH_API void deMatrix6::operator-=(const deMatrix6& m) {
	_mat3[0] -= m._mat3[0];	_mat3[1] -= m._mat3[1];
	_mat3[2] -= m._mat3[2];	_mat3[3] -= m._mat3[3];
}
DE_MATH_API void deMatrix6::operator*=(const deFloat s) {
	_mat3[0] *= s;	_mat3[1] *= s;
	_mat3[2] *= s;	_mat3[3] *= s;
}
DE_MATH_API void deMatrix6::transpose(const deMatrix6& m) {
	_mat3[0].transpose(m._mat3[0]);	_mat3[1].transpose(m._mat3[2]);
	_mat3[2].transpose(m._mat3[1]);	_mat3[3].transpose(m._mat3[3]);
}
DE_MATH_API void deMatrix6::set(const deTransform& t) {
	_mat3[0] = t.rotation();
	_mat3[1].zero();
	_mat3[2].crossMultiply(t.translation(), _mat3[0]);
	_mat3[3] = _mat3[0];
}

#endif // _deMatrix6_inl
