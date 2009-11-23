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

#ifndef _deTransform_h
#define _deTransform_h
/*!
 *	\brief		Transformation class using rotational matrix 
 *	\ingroup	deMath
 *
 *	This class consists of a matrix for rotation and a vector for translation.
 *	\sa deVector3, deMatrix3, deFrame
 */
class deTransform
{
public:
	//!	\return	rotation part
	DE_MATH_API deMatrix3& rotation();
	//!	\return	rotation part
	DE_MATH_API const deMatrix3& rotation() const;
	//!	\return	translation part
	DE_MATH_API deVector3& translation();
	//!	\return	translation part
	DE_MATH_API const deVector3& translation() const ;
	//!	this = identity matrix
	DE_MATH_API void identity();
	//!	this = t
	DE_MATH_API void operator=(const deTransform& t);
	//! this = [r1,p1][r2,p2] = [r1*r2, r1*p2 + p1]
	DE_MATH_API void multiply(const deTransform& t1, const deTransform& t2);
	//! this =  ~[r,p] = [~r, -(~r*p)]
	DE_MATH_API void inverse(const deTransform& t);
	//! this = ~[r1,p1][r2,p2] = [~r1, -(~r1*p1)][r2,p2] = [~r1*r2, ~r1*(p2-p1)]
	DE_MATH_API void inversedMultiply(const deTransform& t1, const deTransform& t2);
	//! this = [r1,p1]~[r2,p2] = [r1,p1][~r2, -(~r2*p2)] = [(r1*~r2), p1-(r1*~r2)*p2]
	DE_MATH_API void multiplyInversed(const deTransform& t1, const deTransform& t2);
	//! this = f
	DE_MATH_API void set(const deFrame& f);
	//! this = [m, v]
	DE_MATH_API void set(const deMatrix3& m, const deVector3& v);
	//! this = Denavit-Hartenberg parameter
	void set(const deFloat alpha, const deFloat a, const deFloat d, const deFloat theta);
	//! this = Screw coordinates
	void set(const deVector3& axis, const deFloat pitch, const deFloat angle);

private:
	deMatrix3 _m;
	deVector3 _v;
};

#endif // _deTransform_h

