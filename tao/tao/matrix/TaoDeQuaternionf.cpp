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


#include <tao/matrix/TaoDeMath.h>
#include <assert.h>

void deColumnV3Q4S1(deFloat* res, const deFloat* q1, const int col)
{
	switch (col)
	{
	case 0:
      res[0] = 1 - 2 * (q1[1] * q1[1] + q1[2] * q1[2]);
      res[1] =     2 * (q1[0] * q1[1] + q1[3] * q1[2]);
      res[2] =     2 * (q1[0] * q1[2] - q1[3] * q1[1]);
	  break;
	case 1:
      res[0] =     2 * (q1[0] * q1[1] - q1[3] * q1[2]);
      res[1] = 1 - 2 * (q1[0] * q1[0] + q1[2] * q1[2]);
      res[2] =     2 * (q1[1] * q1[2] + q1[3] * q1[0]);
	  break;
	case 2:
      res[0] =     2 * (q1[0] * q1[2] + q1[3] * q1[1]);
      res[1] =     2 * (q1[1] * q1[2] - q1[3] * q1[0]);
      /* res[2] = q1[3] * q1[3] - q1[0] * q1[0] - q1[1] * q1[1] + q1[2] * q1[2]; */
      res[2] = 1 - 2 * (q1[0] * q1[0] + q1[1] * q1[1]);
	  break;
	}
}

/* q = [w,v] = [cos(theta/2), axis*sin(theta/2)] <--- axisAngle(axis,theta) */
void deAxisAngleV3S1Q4(deFloat* axis, deFloat *angle, const deFloat* q1)
{
	deFloat mag;
	if (q1[3] > DE_QUATERNION_COS_THRESHHOLD || q1[3] < -DE_QUATERNION_COS_THRESHHOLD)
	{	
		/* no rotation  --> w = 1, v = 0; */
		axis[0] = 0;
		axis[1] = 0;
		axis[2] = 1;
		*angle = 0;
    }
	else
    {
		mag = 1 / deSqrt(q1[0] * q1[0] + q1[1] * q1[1] + q1[2] * q1[2]); // = |sin(theta/2)|
		axis[0] = q1[0] * mag;
		axis[1] = q1[1] * mag;
		axis[2] = q1[2] * mag;
		*angle = 2 * deAcos(q1[3]);
    }
}

/*
 * vFrom and vTo are unit vectors
 * from v1 to v2
 */
/* res = from v1 to v2 */
DE_MATH_API void deSetQ4V3V3(deFloat* res, const deFloat* v1, const deFloat* v2)
{
	deFloat v[DE_QUATERNION_SIZE];
	deFloat cosTheta = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];

	if (cosTheta > DE_QUATERNION_COS_THRESHHOLD) 
	{
		/*
		 * the vectors are the same
		 * w=1, v=0
		 */
		res[0] = 0;
		res[1] = 0;
		res[2] = 0;
		res[3] = 1;
	}
	else if (cosTheta < -DE_QUATERNION_COS_THRESHHOLD) 
	{
	  /*
       * the vectors are opposing
       * w=1, v=0
	   */
		res[0] = 0;
		res[1] = 0;
		res[2] = 0;
		res[3] = 1;
	}
	else
	{
		deCrossV3V3V3(v, v1, v2);
		deNormalizeV3(v);
		deSetQ4V3S1(res, v, deAcos(cosTheta)); /* axis angle */
	}
}

/*
 * by Khatib: Advanced Robotics CS327A
 * For all rotations, at least one of the Euler parameters has a magnitude larger than 0.5.
 * Between two steps of computation, the sign of the largest Euler parameter is maintained constant.
 * This assumption is valid as long as the computation servo-rate is not slower than half of the rotation rate of change.
 * e.g., for a servo-rate of 50Hz, the magnitude of angular velocity must not exceed 100 rad/sec!
 * (q is equivalent to -q) for all angles (theta) where q = [cos(theta/2), v*sin(theta/2)]
 * this funtion compares q1 to q2 and put sign consistent q2 to res.
 */
void deConsistentSignQ4Q4Q4(deFloat* res, const deFloat* q1, const deFloat* q2)
{
	deFloat max = deFabs(q1[0]);
	int imax=0;
	int i;

	for(i = 1; i < 4; i++)
	{
		deFloat value = deFabs(q1[i]);
		if (value > max)
		{
			imax = i;
			max = value;
		}
	}
	assert(max > 0.5);
	res[0] = q2[0];
	res[1] = q2[1];
	res[2] = q2[2];
	res[3] = q2[3];
	if (q1[imax] > 0)
	{
		if (q2[imax] < 0)
		{
			res[0] = -q2[0];
			res[1] = -q2[1];
			res[2] = -q2[2];
			res[3] = -q2[3];
		}
	}
	else if (q1[imax] < 0)
	{
		if (q2[imax] > 0)
		{
			res[0] = -q2[0];
			res[1] = -q2[1];
			res[2] = -q2[2];
			res[3] = -q2[3];
		}
	}
}

/*
 * slerp : spherical linear interpolation
 * slerp(q1,q2,u) = (sin((1-u)*th)/sin(th))*q1 + (sin(u*th)/sin(th))*q2
 *                   where cos(th) = q1.q2
 * slerp(q1,q2,u,s) = (sin(th-u*th2)/sin(th))*q1 + (sin(u*th2)/sin(th))*q2
 *                    where th2 = th + s*pi
 */
void deSlerpQ4Q4Q4S2(deFloat* res, const deFloat* q1, const deFloat* q2, const deFloat t, const deFloat addedSpins)
{
	deFloat scale1, scale2;
	deFloat theta, sinTheta, theta2;
	/* compute dot product, aka cos(theta): */
	deFloat cosTheta = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
	int flip = 0;

	/* flip end quaternion */
	if (cosTheta < 0)
	{
		cosTheta = -cosTheta;
		flip = 1;
	}

	/* if the quaternions are close, use linear interploation */
	if (cosTheta > DE_QUATERNION_COS_THRESHHOLD)
	{
		scale1 = 1 - t;
		scale2 = t;
	}
	else /* otherwise, do spherical interpolation */
	{
		theta = deAcos(cosTheta);
		sinTheta = deSin(theta);
		theta2 = theta + (addedSpins * DE_M_PI);
		
		scale1 = deSin(theta - (t * theta2)) / sinTheta;
		scale2 = deSin(theta2 * t) / sinTheta;
	}

	if (flip)
		scale2 = -scale2;

	res[0] = q1[0] * scale1 + q2[0] * scale2;
	res[1] = q1[1] * scale1 + q2[1] * scale2;
	res[2] = q1[2] * scale1 + q2[2] * scale2;
	res[3] = q1[3] * scale1 + q2[3] * scale2;
}

/* 
 * res = q = Z-Y-X Euler angles
 */
void deSetQ4zyxV3(deFloat* res, const deFloat *v) 
{
	deFloat th, svx, svy, svz, cvx, cvy, cvz;

	th = v[0] * (deFloat)0.5;
	svx = deSin(th);
	cvx = deCos(th);
	th = v[1] * (deFloat)0.5;
	svy = deSin(th);
	cvy = deCos(th);
	th = v[2] * (deFloat)0.5;
	svz = deSin(th);
	cvz = deCos(th);

	res[0] = svx * cvy * cvz - cvx * svy * svz;
	res[1] = cvx * svy * cvz + svx * cvy * svz;
	res[2] = cvx * cvy * svz - svx * svy * cvz;
	res[3] = cvx * cvy * cvz + svx * svy * svz;
}

