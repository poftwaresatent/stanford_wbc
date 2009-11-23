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

/* resv = [x,y,z] */
void deSetV3M3xyz(deFloat* resv, const deFloat (*m1)[DE_MATRIX3_COL])
{
	/* -1 < sinY < 1 */
	if (m1[0][2] > -DE_QUATERNION_COS_THRESHHOLD && m1[0][2] < DE_QUATERNION_COS_THRESHHOLD)	/* cosY != 0 */
	{
		resv[0] = deAtan2(-m1[1][2], m1[2][2]);  /* x */
		resv[2] = deAtan2(-m1[0][1], m1[0][0]);  /* z */

		/* sinZ < 0 || sinZ > 0 */
		if (m1[0][1] < -DE_QUATERNION_EPSILON || m1[0][1] > DE_QUATERNION_EPSILON)  /* sinZ != 0 */
			resv[1] = deAtan2(m1[0][2], m1[0][1] / -deSin(resv[2]));	/* y */
		else	/* cosZ != 0 */
			resv[1] = deAtan2(m1[0][2], m1[0][0] / deCos(resv[2]));		/* y */
	}
	else	/* if (theta[1] == DE_M_PI_2 || theta[1] == -DE_M_PI_2) */
	{
		if (m1[0][2] > 0)
		{
			resv[1] = DE_M_PI_2;			/* y */
			resv[0] = 0;						
			resv[2] = deAtan2(m1[1][0], m1[1][1]);		/* z + x */	// XXX
		}
		else
		{
			resv[1] = -DE_M_PI_2;				/* y */
			resv[0] = 0;
			resv[2] = deAtan2(m1[1][0], m1[1][1]);		/* z - x */	// XXX
		}
	}

	/*
	 * 0 <= x <= 2 * PI
	 * if (resv[0] < 0.0) resv[0] += DE_M_PI + DE_M_PI;
	 * if (resv[1] < 0.0) resv[1] += DE_M_PI + DE_M_PI;
	 * if (resv[2] < 0.0) resv[2] += DE_M_PI + DE_M_PI;
	 */
}

/* resv = [x,y,z] */
void deSetV3M3zyx(deFloat* resv, const deFloat (*m1)[DE_MATRIX3_COL])
{
	/* -1 < sinY < 1 */
	if (m1[2][0] > -DE_QUATERNION_COS_THRESHHOLD && m1[2][0] < DE_QUATERNION_COS_THRESHHOLD)	/* cosY != 0 */
	{
		resv[2] = deAtan2(m1[1][0], m1[0][0]);  /* z */
		resv[0] = deAtan2(m1[2][1], m1[2][2]);  /* x */

#if 1
		/* sinX < 0 || sinX > 0 */
		if (deFabs(m1[2][1]) - deFabs(m1[2][2]) >= 0)  /* sinX != 0 */
			resv[1] = deAtan2(-m1[2][0], m1[2][1] / deSin(resv[0]));	/* y */
		else	/* cosX != 0 */
			resv[1] = deAtan2(-m1[2][0], m1[2][2] / deCos(resv[0]));	/* y */
#else
		/* sinX < 0 || sinX > 0 */
		if (m1[2][1] < -DE_QUATERNION_EPSILON || m1[2][1] > DE_QUATERNION_EPSILON)  /* sinX != 0 */
			resv[1] = deAtan2(-m1[2][0], m1[2][1] / deSin(resv[0]));	/* y */
		else	/* cosX != 0 */
			resv[1] = deAtan2(-m1[2][0], m1[2][2] / deCos(resv[0]));	/* y */
#endif
	}
	else	/* if (theta[1] == DE_M_PI_2 || theta[1] == -DE_M_PI_2) */
	{
		resv[1] = deAsin(-m1[2][0]);	/* y */
		resv[0] = 0;
		if (m1[2][0] > 0)
		{
		//	assert(resv[1] < 0);
			resv[2] = deAtan2(-m1[1][2], m1[1][1]);	/* z + x */
			resv[2] -= resv[0];
		}
		else
		{
		//	assert(resv[1] > 0);
			resv[2] = deAtan2(m1[1][2], m1[1][1]);	/* z - x */
			resv[2] += resv[0];
		}
	}

	/*
	 * 0 <= x <= 2 * PI
	 * if (resv[0] < 0.0) resv[0] += DE_M_PI + DE_M_PI;
	 * if (resv[1] < 0.0) resv[1] += DE_M_PI + DE_M_PI;
	 * if (resv[2] < 0.0) resv[2] += DE_M_PI + DE_M_PI;
	 */
}

/* resv = [x,y,z] */
void deSetV3M3zyxV3(deFloat* resv, const deFloat (*m1)[DE_MATRIX3_COL], const deFloat* last)
{
	/* -1 < sinY < 1 */
	if (m1[2][0] > -DE_QUATERNION_COS_THRESHHOLD  && m1[2][0] < DE_QUATERNION_COS_THRESHHOLD)	/* cosY != 0 */
	{
		resv[2] = deAtan2(m1[1][0], m1[0][0]);  /* z */
		resv[0] = deAtan2(m1[2][1], m1[2][2]);  /* x */

		/* sinX < 0 || sinX > 0 */
		if (deFabs(m1[2][1]) - deFabs(m1[2][2]) >= 0)  /* sinX != 0 */
			resv[1] = deAtan2(-m1[2][0], m1[2][1] / deSin(resv[0]));	/* y */
		else	/* cosX != 0 */
			resv[1] = deAtan2(-m1[2][0], m1[2][2] / deCos(resv[0]));	/* y */
	}
	else	/* if (theta[1] == DE_M_PI_2 || theta[1] == -DE_M_PI_2) */
	{
		resv[1] = deAsin(-m1[2][0]);	/* y */
		resv[0] = last[0];
		if (m1[2][0] > 0)
		{
		//	assert(resv[1] < 0);
			resv[2] = deAtan2(-m1[1][2], m1[1][1]);	/* z + x */
			resv[2] -= resv[0];
		}
		else
		{
		//	assert(resv[1] > 0);
			resv[2] = deAtan2(m1[1][2], m1[1][1]);	/* z - x */
			resv[2] += resv[0];
		}
	}

	/*
	 * 0 <= x <= 2 * PI
	 * if (resv[0] < 0.0) resv[0] += DE_M_PI + DE_M_PI;
	 * if (resv[1] < 0.0) resv[1] += DE_M_PI + DE_M_PI;
	 * if (resv[2] < 0.0) resv[2] += DE_M_PI + DE_M_PI;
	 */
}

/* axis angle */
void deSetM3S2(deFloat (*res)[DE_MATRIX3_COL], const deInt axis, const deFloat angle)
{
	/* also, see deIntro. to Robotics by J. Craig: p. 51 */

	deFloat c = deCos(angle);
	deFloat s = deSin(angle);

	switch (axis)
    {
    case 0:  /* DE_AXIS_X */
		res[0][0] = 1;		res[0][1] = 0;		res[0][2] = 0;
		res[1][0] = 0;		res[1][1] = c;		res[1][2] = -s;
		res[2][0] = 0;		res[2][1] = s;		res[2][2] = c;
		break;
    case 1:  /* DE_AXIS_Y */
		res[0][0] = c;		res[0][1] = 0;		res[0][2] = s;
		res[1][0] = 0;		res[1][1] = 1;		res[1][2] = 0;
		res[2][0] = -s;		res[2][1] = 0;		res[2][2] = c;
		break;
    case 2:  /* DE_AXIS_Z */
		res[0][0] = c;		res[0][1] = -s;		res[0][2] = 0;
		res[1][0] = s;		res[1][1] = c;		res[1][2] = 0;
		res[2][0] = 0;		res[2][1] = 0;		res[2][2] = 1;
		break;
    }
}

void deSetQ4M3(deFloat* resq, const deFloat (*m1)[DE_MATRIX3_COL])
{
	/* also, see deIntro. to Robotics by J. Craig: p. 56 */
	/*
	 * w = 0.5*sqrt(1+r11+r22+r33) = cos(angle/2);
	 * v1= (r32-r23)/4w            = Kx*sin(angle/2);
	 * v2= (r13-r31)/4w            = Ky*sin(angle/2);
	 * v3= (r21-r12)/4w            = Kz*sin(angle/2);
	 */
	/* assumption: w>=0  --> 0<= theata <= pi */

	deFloat c, xx, yy;
	deFloat ww = (deFloat)0.25 * (1 + m1[0][0] + m1[1][1] + m1[2][2]);  /* = w^2 */
	if (ww > DE_QUATERNION_EPSILON)
    {
		resq[3] = deSqrt(ww);
		c = 1 / (4 * resq[3]);
		resq[0] = (m1[2][1] - m1[1][2]) * c;
		resq[1] = (m1[0][2] - m1[2][0]) * c;
		resq[2] = (m1[1][0] - m1[0][1]) * c;
    }
	else
    {
		/* 180 deg rotation  --> w = 0, v = 0 */
		resq[3] = 0;

		/* let's find v */

		/* Kx^2 = -0.5(r22+r33) */
		xx = -(deFloat)0.5 * (m1[1][1] + m1[2][2]);

		if (xx > DE_QUATERNION_EPSILON)
		{
			resq[0] = deSqrt(xx);
			resq[1] = m1[0][1] / (resq[0] + resq[0]);
			resq[2] = m1[0][2] / (resq[0] + resq[0]);
		}
		else /* Kx^2=0 */
		{
			resq[0] = 0;

			/*
			 * Ky^2 = -0.5(r11+r33)   * r11 = -1
			 *      = -0.5(r33-1)
			 *      = 0.5(1 - r33)
			 */
			yy = (deFloat)0.5 * (1 - m1[2][2]);
			
			if (yy > DE_QUATERNION_EPSILON)
			{
				resq[1] = deSqrt(yy);
				resq[2] = m1[1][2] / (resq[1] + resq[1]);
			}
			else /* Ky^2 = 0 */
			{
				resq[1] = 0;
				resq[2] = 1;
			}
		}
    }
}

/*
 * y = LU x
 * find x given LU and y 
 * diag(U) = [1 1 1 ... 1]
 */
void deBackSubstituteV3M3V3(deFloat* x, const deFloat (*lu)[DE_MATRIX3_COL], const deFloat* y)
{
	/* LU x = y ----> x */
	/* L d  = y --> fw sub */
	/* U x = d ---> bw sub */
	deInt i, j;
	deFloat d[DE_VECTOR3_SIZE];

	/* forward */
	for (i = 0; i < 3; i++)
	{
		d[i] = y[i];
		for (j = 0; j < i; j++)
			d[i] -= lu[i][j] * d[j];
	}	
	/* backward */
	for (i = 2; i >= 0; i--)
	{
		x[i] = d[i];
		for (j = i + 1; j < 3; j++)
			x[i] -= lu[i][j] * x[j];
		x[i] /= lu[i][i];
	}
}

/* output: lu */
/* diag(U) = [1 1 1 ... 1] */
void deLUdecomposeM3M3(deFloat (*lu)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL])
{
	deInt i, j, k;

	for (k = 0; k < 3; k++)
    {
		lu[k][k] = m1[k][k];
		for (i = 0; i < k; i++)
		{
			lu[i][k] = m1[i][k];
			lu[k][i] = m1[k][i];
			for (j = 0; j < i; j++)
			{
				lu[i][k] -= lu[i][j] * lu[j][k];
				lu[k][i] -= lu[k][j] * lu[j][i];
			}
			lu[k][i] /= lu[i][i];
			lu[k][k] -= lu[k][i] * lu[i][k];
		}
    }
}

/*
 * y = LU x
 * y = LL' x
 * find x given LU and y 
 *(symmetric and positive definite)
 */
DE_MATH_API void deBackSubstituteSPDV3M3V3(deFloat* x, const deFloat (*LU)[DE_MATRIX3_COL], const deFloat* y)
{
	/* U = L' */
	/* LU = LL' */
	/* LU x = y ----> x */
	/* L d  = y --> fw sub */
	/* U x = d ---> bw sub */
	deInt i, j, k;
	deFloat d[DE_VECTOR3_SIZE];

	/* forward */
	for (k = 0; k < 3; k++)
	{
		for (i = 0; i < 3; i++)
		{
			d[i] = y[i];
			for (j = 0; j < i; j++)
				d[i] -= LU[i][j] * d[j];
			d[i] /= LU[i][i];
		}	
	}

	/* backward */
	for (k = 0; k < 3; k++)
	{
		for (i = 2; i >= 0; i--)
		{	
			x[i] = d[i];
			for (j = i + 1; j < 3; j++)
				x[i] -= LU[j][i] * x[j];
			x[i] /= LU[i][i];
		}
	}
}

/* (symmetric and positive definite) */
DE_MATH_API void deLUdecomposeSPDM3M3(deFloat (*lu)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL])
{
	/* U = L' */
	deInt i, j, k;

	for (k = 0; k < 3; k++)
    {
		lu[k][k] = m1[k][k];
		for (j = 0; j < k; j++) 
		{
			lu[k][j] = m1[j][k];
			for (i = 0; i < j; i++)
				lu[k][j] -= lu[k][i] * lu[j][i];
			lu[k][j] /= lu[j][j];
			lu[k][k] -= lu[k][j] * lu[k][j];
		}
		lu[k][k] = deSqrt(lu[k][k]);
    }
}

/*
 * y = m1 x
 * solve for x given y
 */
DE_MATH_API void deSolveV3M3V3(deFloat* x, const deFloat (*m1)[DE_MATRIX3_COL], const deFloat* y)
{
	deFloat lu[DE_MATRIX3_ROW][DE_MATRIX3_COL];

	deLUdecomposeM3M3(lu, m1);
	deBackSubstituteV3M3V3(x, lu, y);
}

/*
 * y = m1 x
 * solve for x given y
 *(symmetric and positive definite)
 */
DE_MATH_API void deSolveSPDV3M3V3(deFloat* x, const deFloat (*m1)[DE_MATRIX3_COL], const deFloat* y)
{
	deFloat lu[DE_MATRIX3_ROW][DE_MATRIX3_COL];

	deLUdecomposeSPDM3M3(lu, m1);
	deBackSubstituteSPDV3M3V3(x, lu, y);
}

/* Matrix Inverse by Crout's LU decomposition */
/* p275-285, Numerical Methods for Engineers by Chadea and Canale */

/* output: ainv */
DE_MATH_API void deInvertM3M3(deFloat (*ainv)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL])
{
	deInt k;
	deFloat x[DE_VECTOR3_SIZE], y[DE_VECTOR3_SIZE];
	deFloat lu[DE_MATRIX3_ROW][DE_MATRIX3_COL];
	
	/*
	 * LU ainv = I
	 * a x = y
	 * LU x = y
	 */
	deLUdecomposeM3M3(lu, m1);
	
	for (k = 0; k < 3; k++)
	{
		y[0] = 0;
		y[1] = 0;
		y[2] = 0;
		y[k] = 1;
		
		deBackSubstituteV3M3V3(x, lu, y);
		ainv[0][k] = x[0];
		ainv[1][k] = x[1];
		ainv[2][k] = x[2];
	}
}

/* Matrix Inverse by Cholesky's LU decomposition */
/* p288-290, Numerical Methods for Engineers by Chadea and Canale */

/* output: ainv */
/* (symmetric and positive definite) */
DE_MATH_API void deInvertSPDM3M3(deFloat (*ainv)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL])
{
	deInt k;
	deFloat x[DE_VECTOR3_SIZE], y[DE_VECTOR3_SIZE];
	deFloat lu[DE_MATRIX3_ROW][DE_MATRIX3_COL];

	/*
	 * LU ainv = I
	 * a x = y
	 * LU x = y
	 */
	deLUdecomposeSPDM3M3(lu, m1);
	
	for(k = 0; k < 3; k++)
    {
		y[0] = 0;
		y[1] = 0;
		y[2] = 0;
		y[k] = 1;
		
		deBackSubstituteSPDV3M3V3(x, lu, y);
		ainv[0][k] = x[0];
		ainv[1][k] = x[1];
		ainv[2][k] = x[2];
    }
}

DE_MATH_API void deInvertGaussSPDM3M3(deFloat (*ainv)[DE_MATRIX3_COL], const deFloat (*m1)[DE_MATRIX3_COL])
{
	deFloat m01, m02, m12, m12prime;

	ainv[0][0] = 1 / m1[0][0];
	m01 = m1[0][1] * ainv[0][0];
	m02 = m1[0][2] * ainv[0][0];
	
	ainv[1][1] = 1 / (m1[1][1] - m01 * m1[0][1]);
	ainv[1][0] = -m01 * ainv[1][1];                      
	m12 = m1[1][2] - m02 * m1[0][1];
	m12prime = m12 * ainv[1][1];              
	
	ainv[2][2] = 1 / (m1[2][2] - m02 * m1[0][2] - m12prime * m12);
	ainv[2][1] = - m12prime * ainv[2][2];
	ainv[2][0] = (m12prime * m01 - m02) * ainv[2][2];
	
	ainv[1][2] = ainv[2][1];
	ainv[1][1] -= ainv[2][1] * m12prime;
	ainv[1][0] -= ainv[2][0] * m12prime;
	
	ainv[0][2] = ainv[2][0];
	ainv[0][1] = ainv[1][0];
	ainv[0][0] -= (ainv[1][0] * m01 + ainv[2][0] * m02);
}

