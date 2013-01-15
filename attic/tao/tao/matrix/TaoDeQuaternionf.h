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

#ifndef _deQuaternionf_h
#define _deQuaternionf_h

#ifdef __cplusplus
extern "C" {
#endif

/* 
 * q = [w,v] = [cos(theta/2);, axis*sin(theta/2);] <--- axisAngle(axis,theta);
 * q = [vx,vy,vz,w] --> float[4];
 */

extern void deColumnV3Q4S1(deFloat* res, const deFloat* q1, const int col);
extern void deConsistentSignQ4Q4Q4(deFloat* res, const deFloat* q1, const deFloat* q2);
extern void deSlerpQ4Q4Q4S2(deFloat* res, const deFloat* q1, const deFloat* q2, const deFloat t, const deFloat addedSpins);
extern void deAxisAngleV3S1Q4(deFloat* axis, deFloat *angle, const deFloat* q1);
extern void deSetQ4zyxV3(deFloat* res, const deFloat *v);

/* inlines */

DE_MATH_API void deSetQ4S4(deFloat* res, const deFloat x, const deFloat y, const deFloat z, const deFloat w)
{
    res[0] = x;	res[1] = y;	res[2] = z;	res[3] = w;
}

DE_MATH_API void deIdentityQ4(deFloat* res)
{
	res[0] = res[1] = res[2] = 0;	res[3] = 1;
}

DE_MATH_API void deZeroQ4(deFloat* res)
{
	res[0] = res[1] = res[2] = res[3] = 0;
}

DE_MATH_API void deNegateQ4Q4(deFloat* res, const deFloat* q1)
{
	res[0] = -q1[0];
	res[1] = -q1[1];
	res[2] = -q1[2];
	res[3] = -q1[3];
}

DE_MATH_API void deInvertQ4Q4(deFloat* res, const deFloat* q1)
{
	res[0] = -q1[0];
	res[1] = -q1[1];
	res[2] = -q1[2];
	res[3] =  q1[3];
}

DE_MATH_API void deSetQ4Q4(deFloat* res, const deFloat* q1)
{
	res[0] = q1[0];
	res[1] = q1[1];
	res[2] = q1[2];
	res[3] = q1[3];
}

DE_MATH_API void deAddQ4Q4Q4(deFloat* res, const deFloat* q1, const deFloat* q2)
{
	res[0] = q1[0] + q2[0];
	res[1] = q1[1] + q2[1];
	res[2] = q1[2] + q2[2];
	res[3] = q1[3] + q2[3];
}

DE_MATH_API void deSubQ4Q4Q4(deFloat* res, const deFloat* q1, const deFloat* q2)
{
	res[0] = q1[0] - q2[0];
	res[1] = q1[1] - q2[1];
	res[2] = q1[2] - q2[2];
	res[3] = q1[3] - q2[3];
}

DE_MATH_API deFloat deDotQ4Q4(const deFloat* q1, const deFloat* q2)
{
	return (q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]);
}

/* res = (w1*w2 - v1.v2, v1*w2 + v2*w1 + v1xv2) */
DE_MATH_API void deMulQ4Q4Q4(deFloat* res, const deFloat* q1, const deFloat* q2)
{
	res[0] = q1[0] * q2[3] + q2[0] * q1[3] + (q1[1] * q2[2] - q1[2] * q2[1]);
	res[1] = q1[1] * q2[3] + q2[1] * q1[3] + (q1[2] * q2[0] - q1[0] * q2[2]);
	res[2] = q1[2] * q2[3] + q2[2] * q1[3] + (q1[0] * q2[1] - q1[1] * q2[0]);
	res[3] = q1[3] * q2[3] - (q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2]);
}

DE_MATH_API void deMulQ4Q4iQ4(deFloat* res, const deFloat* q1, const deFloat* q2)
{
	/* w_ = w_, v_ = - v_ */
	res[0] = -q1[0] * q2[3] + q2[0] * q1[3] - (q1[1] * q2[2] - q1[2] * q2[1]);
	res[1] = -q1[1] * q2[3] + q2[1] * q1[3] - (q1[2] * q2[0] - q1[0] * q2[2]);
	res[2] = -q1[2] * q2[3] + q2[2] * q1[3] - (q1[0] * q2[1] - q1[1] * q2[0]);
	res[3] =  q1[3] * q2[3] + (q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2]);
}

DE_MATH_API void deMulQ4Q4Q4i(deFloat* res, const deFloat* q1, const deFloat* q2)
{
	/* w_ = w_, v_ = - v_ */
	res[0] = q1[0] * q2[3] - q2[0] * q1[3] - (q1[1] * q2[2] - q1[2] * q2[1]);
	res[1] = q1[1] * q2[3] - q2[1] * q1[3] - (q1[2] * q2[0] - q1[0] * q2[2]);
	res[2] = q1[2] * q2[3] - q2[2] * q1[3] - (q1[0] * q2[1] - q1[1] * q2[0]);
	res[3] = q1[3] * q2[3] + (q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2]);
}

DE_MATH_API void deMulQ4S1(deFloat* res, const deFloat s)
{
	res[0] *= s;
	res[1] *= s;
	res[2] *= s;
	res[3] *= s;
}

DE_MATH_API void deAddQ4Q4(deFloat* res, const deFloat* q1)
{
	res[0] += q1[0];
	res[1] += q1[1];
	res[2] += q1[2];
	res[3] += q1[3];
}

DE_MATH_API void deSubQ4Q4(deFloat* res, const deFloat* q1)
{
	res[0] -= q1[0];
	res[1] -= q1[1];
	res[2] -= q1[2];
	res[3] -= q1[3];
}

/*
 * [w,v'] = q * [0,v] * ~q  <==> v' = R v
 * = [w1,v1]*[0,v2]*[w1,-v1] = [ -v1.v2, w1v2+ v1 x v2]*[w1,-v1]
 * = [(-v1.v2)(w1)+(w1v2+ v1 x v2).v1,
 *    (v1.v2)v1 + w1(w1v2+ v1 x v2) + v1 x (w1v2+ v1 x v2)]
 * since A x (B + C) = A x B + A x C
 * = [w, (w1*w1)v2 + 2*w1(v1 x v2) + (v1.v2)v1 + v1 x (v1 x v2)]
 * since A x (B x C) = (A dot C)B - (A dot B)C
 * = [w, (w1*w1)v2 + 2*w1(v1 x v2) + (v1.v2)v1 + (v1.v2)v1 - (v1.v1)v2]
 * = [w, (w1*w1 - v1.v1)v2 + 2*w1(v1 x v2) + 2*(v1.v2)v1]
 * v' = [(w1*w1 - v1.v1)v2 + 2*w1(v1 x v2) + 2*(v1.v2)v1]
 */
DE_MATH_API void deMulV3Q4V3(deFloat* res, const deFloat* q1, const deFloat* v2)
{
	deFloat m11 = q1[3] * q1[3] - (q1[0] * q1[0] + q1[1] * q1[1] + q1[2] * q1[2]);
	deFloat m12 = 2 * (q1[0] * v2[0] + q1[1] * v2[1] + q1[2] * v2[2]);
	deFloat w2 = q1[3] + q1[3];

	res[0] = q1[0] * m12 + v2[0] * m11 + (q1[1] * v2[2] - q1[2] * v2[1]) * w2;
	res[1] = q1[1] * m12 + v2[1] * m11 + (q1[2] * v2[0] - q1[0] * v2[2]) * w2;
	res[2] = q1[2] * m12 + v2[2] * m11 + (q1[0] * v2[1] - q1[1] * v2[0]) * w2;
}

/* [w,v'] = ~q * [0,v] * q  <==> v' = R v
 * = [w1,-v1]*[0,v2]*[w1,v1] = [ v1.v2, w1v2 - v1 x v2]*[w1,v1]
 * v' = [(w1*w1 - v1.v1)v2 - 2*w1(v1 x v2) + 2*(v1.v2)v1]
 */
DE_MATH_API void deMulV3Q4iV3(deFloat* res, const deFloat* q1, const deFloat* v2)
{
  // w_ = w_, v_ = - v_
	deFloat m11 = q1[3] * q1[3] - (q1[0] * q1[0] + q1[1] * q1[1] + q1[2] * q1[2]);
	deFloat m12 = 2 * (q1[0] * v2[0] + q1[1] * v2[1] + q1[2] * v2[2]);
	deFloat w2 = -(q1[3] + q1[3]);

	res[0] = q1[0] * m12 + v2[0] * m11 + (q1[1] * v2[2] - q1[2] * v2[1]) * w2;
	res[1] = q1[1] * m12 + v2[1] * m11 + (q1[2] * v2[0] - q1[0] * v2[2]) * w2;
	res[2] = q1[2] * m12 + v2[2] * m11 + (q1[0] * v2[1] - q1[1] * v2[0]) * w2;
}

DE_MATH_API int deIsEqualQ4Q4(const deFloat* q1, const deFloat* q2)
{
	deFloat mag2 = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
    return (mag2 > DE_QUATERNION_COS_THRESHHOLD || mag2 < -DE_QUATERNION_COS_THRESHHOLD);
}

/* [w1,v1]*[w2,v2] = [(w1*w2 - v1 dot v2), (w1*v2+w2*v1+ v1 cross v2)] */
DE_MATH_API void deNormalizeQ4(deFloat* res)
{
	deFloat mag = 1 / deSqrt(res[0] * res[0] + res[1] * res[1] + res[2] * res[2] + res[3] * res[3]);
    res[0] *= mag;
    res[1] *= mag;
    res[2] *= mag;
    res[3] *= mag;
}

/* axis angle */
DE_MATH_API void deSetQ4V3S1(deFloat* res, const deFloat* axis, const deFloat angle)
{
	deFloat s = deSin(0.5 * angle);
	res[0] = axis[0] * s;
	res[1] = axis[1] * s;
	res[2] = axis[2] * s;
	res[3] = deCos(0.5 * angle);
}

/* axis angle */
DE_MATH_API void deSetQ4S2(deFloat* res, const int axis, const deFloat angle)
{
	/* also, see Intro. to Robotics by J. Craig: p. 51 */
	res[0] = res[1] = res[2] = 0;
	res[axis] = deSin(0.5 * angle);
	res[3] = deCos(0.5 * angle);
}	

/*
 * dPhi = Einv ( q - qd) = -2*q_tilde^T qd <-- since (q_tilde^T q) = 0
 * E = 0.5*q_tilde
 * Einv = (EtE)inv Et = 4Et = 2*qtilde^T <-- since (q_tilde^T q_tilde) = I
 *
 * q = [w x y z]
 * Khatib
 * q_tilde = [ -x -y -z
 *              w  z -y
 *             -z  w  x
 *               y -x  w ] 
 * q_tilde^T = [ -x  w -z  y
 *               -y  z  w -x
 *               -z -y  x  w ] 	       
 */
DE_MATH_API void deAngularErrorV3Q4Q4(deFloat* res, const deFloat* q1, const deFloat* q2)
{
  res[0] = -2 * ( q1[3] * q2[0] - q1[2] * q2[1] + q1[1] * q2[2] - q1[0] * q2[3]);
  res[1] = -2 * ( q1[2] * q2[0] + q1[3] * q2[1] - q1[0] * q2[2] - q1[1] * q2[3]);
  res[2] = -2 * (-q1[1] * q2[0] + q1[0] * q2[1] + q1[3] * q2[2] - q1[2] * q2[3]);
}

/* 
 * dq = E v2
 * dq = 0.5 q_tilde v2
 */
DE_MATH_API void deVelocityQ4Q4V3(deFloat* res, const deFloat* q1, const deFloat* v2)
{
  res[0] = (deFloat)0.5 * ( q1[3] * v2[0] + q1[2] * v2[1] - q1[1] * v2[2]);
  res[1] = (deFloat)0.5 * (-q1[2] * v2[0] + q1[3] * v2[1] + q1[0] * v2[2]);
  res[2] = (deFloat)0.5 * ( q1[1] * v2[0] - q1[0] * v2[1] + q1[3] * v2[2]);
  res[3] = (deFloat)0.5 * (-q1[0] * v2[0] - q1[1] * v2[1] - q1[2] * v2[2]);
}

/* 
 * res = q1 + t * (q2 - q1)
 */
DE_MATH_API void deLerpQ4Q4Q4S1(deFloat* res, const deFloat* q1, const deFloat* q2, const deFloat t)
{
  res[0] = q1[0] + t * (q2[0] - q1[0]);
  res[1] = q1[1] + t * (q2[1] - q1[1]);
  res[2] = q1[2] + t * (q2[2] - q1[2]);
  res[3] = q1[3] + t * (q2[3] - q1[3]);
}

/* 
 * res = [x, y, z] = Z-Y-X Euler angles
 */
DE_MATH_API	void deSetV3Q4zyx(deFloat* res, const deFloat* q)
{
    deFloat sqx = q[0] * q[0];
    deFloat sqy = q[1] * q[1];
    deFloat sqz = q[2] * q[2];
    deFloat sqw = q[3] * q[3];

	res[0] = deAtan2(2 * (q[1] * q[2] + q[0] * q[3]), (-sqx - sqy + sqz + sqw));
	res[1] = deAsin(-2 * (q[0] * q[2] - q[1] * q[3]));
	res[2] = deAtan2(2 * (q[0] * q[1] + q[2] * q[3]), (sqx - sqy - sqz + sqw));
}

#ifdef __cplusplus
}
#endif

#endif /* _deQuaternion_h */
