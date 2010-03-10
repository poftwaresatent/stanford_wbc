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

#ifndef _deFrame_inl
#define _deFrame_inl
	
DE_MATH_API void deFrame::identity() { _q.identity(); _v.zero(); }
DE_MATH_API void deFrame::operator=(const deFrame & f) { _q = f._q; _v = f._v; } 
//! this = f1 * f2 = [r1,p1][r2,p2] = [r1*r2, r1*p2 + p1]
DE_MATH_API void deFrame::multiply(const deFrame& f1, const deFrame& f2) {
	_q.multiply(f1.rotation(), f2.rotation());
	_v.multiply(f1.rotation(), f2.translation());
	_v += f1.translation();
}
//! this = f1^-1 * f2 
//       = ~[r1,p1][r2,p2] = [~r1, -(~r1*p1)][r2,p2] = [~r1*r2, ~r1*p2 - (~r1*p1)]
//       = [~r1*r2, ~r1*(p2-p1)]
DE_MATH_API void deFrame::inversedMultiply(const deFrame& f1, const deFrame& f2) {
	deVector3 p;
	p.subtract(f2.translation(), f1.translation());
	_v.inversedMultiply(f1.rotation(), p);
	_q.inversedMultiply(f1.rotation(), f2.rotation());
}
//! this = f1 * f2^-1 
//       = [r1,p1]~[r2,p2] = [r1,p1][~r2, -(~r2*p2)] = [r1*~r2, -r1*(~r2*p2) + p1]
//       = [r1*~r2, -r1*~r2*p2 + p1]
DE_MATH_API void deFrame::multiplyInversed(const deFrame& f1, const deFrame& f2) {
	_q.multiplyInversed(f1.rotation(), f2.rotation());
	_v.multiply(_q, f2.translation());
	_v.subtract(f1.translation(), _v);
}
//! this = f^-1 =  ~[r,p] = [~r, -(~r*p)]
DE_MATH_API void deFrame::inverse(const deFrame& f) {
	_q.inverse(f.rotation());
	_v.multiply(_q, f.translation());
	_v.negate(_v);
}
DE_MATH_API void deFrame::set(const deTransform& t) { _q.set(t.rotation()); _v = t.translation(); }
DE_MATH_API void deFrame::set(const deQuaternion& q, const deVector3& v) { _q = q; _v = v; }

#endif // _deFrame_inl

