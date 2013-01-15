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

#ifndef _deTransform_inl
#define _deTransform_inl

DE_MATH_API deMatrix3& deTransform::rotation() { return _m; }
DE_MATH_API const deMatrix3& deTransform::rotation() const { return _m; }
DE_MATH_API deVector3& deTransform::translation() { return _v; }
DE_MATH_API const deVector3& deTransform::translation() const { return _v; }

DE_MATH_API void deTransform::identity() { _m.identity(); _v.zero(); }
DE_MATH_API void deTransform::operator=(const deTransform& t) { _m = t._m; _v = t._v; }
//! this = [r1,p1][r2,p2] = [r1*r2, r1*p2 + p1]
DE_MATH_API void deTransform::multiply(const deTransform& t1, const deTransform& t2) {
	_m.multiply(t1.rotation(), t2.rotation());
	_v.multiply(t1.rotation(), t2.translation());
	_v += t1.translation();
}
//! this =  ~[r,p] = [~r, -(~r*p)]
DE_MATH_API void deTransform::inverse(const deTransform& t) {
	_m.transpose(t.rotation());
	_v.multiply(_m, t.translation());
	_v.negate(_v);
}
//! this = ~[r1,p1][r2,p2] = [~r1, -(~r1*p1)][r2,p2] = [~r1*r2, ~r1*(p2-p1)]
DE_MATH_API void deTransform::inversedMultiply(const deTransform& t1, const deTransform& t2) {
	deVector3 p;
	p.subtract(t2.translation(), t1.translation());
	_v.transposedMultiply(t1.rotation(), p);
	_m.transposedMultiply(t1.rotation(), t2.rotation());
}
//! this = [r1,p1]~[r2,p2] = [r1,p1][~r2, -(~r2*p2)] = [(r1*~r2), p1-(r1*~r2)*p2]
DE_MATH_API void deTransform::multiplyInversed(const deTransform& t1, const deTransform& t2)
{
	_m.multiplyTransposed(t1.rotation(), t2.rotation());
	_v.multiply(_m, t2.translation());
	_v.subtract(t1.translation(), _v);
}
DE_MATH_API void deTransform::set(const deFrame& f) { _m.set(f.rotation()); _v = f.translation(); }
DE_MATH_API void deTransform::set(const deMatrix3& m, const deVector3& v) { _m = m; _v = v; }

#endif // _deTransform_inl

