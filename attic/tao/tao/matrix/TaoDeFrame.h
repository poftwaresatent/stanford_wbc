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

#ifndef _deFrame_h
#define _deFrame_h
/*!
 *	\brief		Transformation class using quaternion 
 *	\ingroup	deMath
 *
 *	This class consists of a quaternion for rotation and a vector for translation.
 *	\sa deVector3, deQuaternion, deTransform
 */
class deFrame
{
public:
  /** Default ctor inits to identity. */
  inline deFrame() { identity(); }
  
  /** Construction from three floats ends up as pure translation. */
  inline deFrame(deFloat tx, deFloat ty, deFloat tz)
  { set(deQuaternion(), deVector3(tx, ty, tz)); }
  
  inline deFrame(deFrame const & orig): _q(orig._q), _v(orig._v) {}
  
	//! \return rotation part
	deQuaternion& rotation() { return _q; }
	//! \return rotation part
	const deQuaternion& rotation() const { return _q; }
	//! \return translation part
	deVector3& translation() { return _v;; }
	//! \return translation part
	const deVector3& translation() const { return _v; }
	//! this = identity matrix
	DE_MATH_API void identity();
	//! this = f
	DE_MATH_API void operator=(const deFrame & f);

  /**
     Combine two frame transformations: \c f2 gets rotated and
     translated by \c f1. For example, if you know the COM frame of a
     link relative to its local frame, and you want to know the global
     COM position given the global position of the local frame, you
     do:
     \code
     deFrame com_wrt_link, link_wrt_global;
     deFrame com;
     com.multiply(link_wrt_global, com_wrt_link);
     \endcode
     
     \note Do NOT invoke this method on one of the parameters, because
     the operation is not atomic. If you want to replace a frame by
     the result of this multiplication, use a temporary object.
     
     \code
     // old pseudo-code documentation for this method:
     this = f1 * f2 = [r1,p1][r2,p2] = [r1*r2, r1*p2 + p1]
     \endcode
     
     \code
     // BAD example: this will produce garbage
     deFrame foo, bar;
     foo.multiply(foo, bar); // foo gets partially modified half way through the operation
     \endcode

     \code
     // GOOD example: use a temporary object if you want to replace a frame
     deFrame foo, bar;
     foo.multiply(deFrame(foo), bar);
     \endcode
  */
	DE_MATH_API void multiply(const deFrame& f1, const deFrame& f2);

	//! this = f1^-1 * f2 
	//       = ~[r1,p1][r2,p2] = [~r1, -(~r1*p1)][r2,p2] = [~r1*r2, ~r1*p2 - (~r1*p1)]
	//       = [~r1*r2, ~r1*(p2-p1)]
	DE_MATH_API void inversedMultiply(const deFrame& f1, const deFrame& f2);
	//! this = f1 * f2^-1 
	//       = [r1,p1]~[r2,p2] = [r1,p1][~r2, -(~r2*p2)] = [r1*~r2, -r1*(~r2*p2) + p1]
	//       = [r1*~r2, -r1*~r2*p2 + p1]
	DE_MATH_API void multiplyInversed(const deFrame& f1, const deFrame& f2);
	//! this = f^-1
	//		 =  ~[r,p] = [~r, -(~r*p)]
	DE_MATH_API void inverse(const deFrame& f);
	//!	this = t
	DE_MATH_API void set(const deTransform& t);
	//!	this = [q, v]
	DE_MATH_API void set(const deQuaternion& q, const deVector3& v);

private:
	deQuaternion _q;
	deVector3 _v;
};

#endif // _deFrame_h

