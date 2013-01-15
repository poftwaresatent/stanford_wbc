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

void deMatrix6::multiplyTransposed(const deVector6& v1, const deVector6& v2) 
{
	_mat3[0].multiplyTransposed(v1[0], v2[0]);
	_mat3[1].multiplyTransposed(v1[0], v2[1]);
	_mat3[2].multiplyTransposed(v1[1], v2[0]);
	_mat3[3].multiplyTransposed(v1[1], v2[1]);
}

void deMatrix6::similarityXform(const deMatrix6& L, const deMatrix6& I)
{
	deMatrix3 tmpM0,tmpM1,tmpM2;

	tmpM0.multiply(L._mat3[0], I._mat3[0]);
	tmpM1.multiply(L._mat3[1], I._mat3[2]);
	tmpM2.add(tmpM0, tmpM1);
	_mat3[0].multiplyTransposed(tmpM2, L._mat3[0]);
	_mat3[1].multiplyTransposed(tmpM2, L._mat3[2]);

	tmpM0.multiply(L._mat3[0], I._mat3[1]);
	tmpM1.multiply(L._mat3[1], I._mat3[3]);
	tmpM2.add(tmpM0, tmpM1);
	tmpM0.multiplyTransposed(tmpM2, L._mat3[1]);
	tmpM1.multiplyTransposed(tmpM2, L._mat3[3]);
	_mat3[0] += tmpM0;
	_mat3[1] += tmpM1;

	_mat3[2].transpose(_mat3[1]);

	tmpM0.multiply(L._mat3[2], I._mat3[0]);
	tmpM1.multiply(L._mat3[3], I._mat3[2]);
	tmpM2.add(tmpM0, tmpM1);
	_mat3[3].multiplyTransposed(tmpM2, L._mat3[2]);

	tmpM0.multiply(L._mat3[2], I._mat3[1]);
	tmpM1.multiply(L._mat3[3], I._mat3[3]);
	tmpM2.add(tmpM0, tmpM1);
	tmpM0.multiplyTransposed(tmpM2, L._mat3[3]);
	_mat3[3] += tmpM0;
}

void deMatrix6::similarityXformT(const deMatrix6& L, const deMatrix6& I)
{
	deMatrix3 tmpM0,tmpM1,tmpM2;

	tmpM0.transposedMultiply(L._mat3[0], I._mat3[0]);
	tmpM1.transposedMultiply(L._mat3[2], I._mat3[2]);
	tmpM2.add(tmpM0, tmpM1);
	_mat3[0].multiply(tmpM2, L._mat3[0]);
	_mat3[1].multiply(tmpM2, L._mat3[1]);

	tmpM0.transposedMultiply(L._mat3[0], I._mat3[1]);
	tmpM1.transposedMultiply(L._mat3[2], I._mat3[3]);
	tmpM2.add(tmpM0, tmpM1);
	tmpM0.multiply(tmpM2, L._mat3[2]);
	tmpM1.multiply(tmpM2, L._mat3[3]);
	_mat3[0] += tmpM0;
	_mat3[1] += tmpM1;

	_mat3[2].transpose(_mat3[1]);

	tmpM0.transposedMultiply(L._mat3[1], I._mat3[0]);
	tmpM1.transposedMultiply(L._mat3[3], I._mat3[2]);
	tmpM2.add(tmpM0, tmpM1);
	_mat3[3].multiply(tmpM2, L._mat3[1]);

	tmpM0.transposedMultiply(L._mat3[1], I._mat3[1]);
	tmpM1.transposedMultiply(L._mat3[3], I._mat3[3]);
	tmpM2.add(tmpM0, tmpM1);
	tmpM0.multiply(tmpM2, L._mat3[3]);
	_mat3[3] += tmpM0;
}

// this = m1 * m2
// [A0 A1;A2 A3]*[B0 B1;B2 B3] = [A0*B0+A1*B2 A0*B1+A1*B3;
//                                A2*B0+A3*B2 A2*B1+A3*B3]
void deMatrix6::multiply(const deMatrix6& m1, const deMatrix6& m2)
{
    deMatrix3 tmpM;

    _mat3[0].multiply(m1._mat3[0], m2._mat3[0]);
    tmpM.multiply(m1._mat3[1], m2._mat3[2]);
    _mat3[0] += tmpM;

	_mat3[1].multiply(m1._mat3[0], m2._mat3[1]);
    tmpM.multiply(m1._mat3[1], m2._mat3[3]);
    _mat3[1] += tmpM;

	_mat3[2].multiply(m1._mat3[2], m2._mat3[0]);
    tmpM.multiply(m1._mat3[3], m2._mat3[2]);
    _mat3[2] += tmpM;

	_mat3[3].multiply(m1._mat3[2], m2._mat3[1]);
    tmpM.multiply(m1._mat3[3], m2._mat3[3]);
    _mat3[3] += tmpM;
}

// this = m1^T * m2
// [A0 A1;A2 A3]^T*[B0 B1;B2 B3] = [A0^T A2^T;A1^T A3^T]*[B0 B1;B2 B3] 
//								 = [A0^T*B0+A2^T*B2 A0^T*B1+A2^T*B3;
//                                  A1^T*B0+A3^T*B2 A1^T*B1+A3^T*B3]
void deMatrix6::transposedMultiply(const deMatrix6& m1, const deMatrix6& m2)
{
    deMatrix3 tmpM;

    _mat3[0].transposedMultiply(m1._mat3[0], m2._mat3[0]);
    tmpM.transposedMultiply(m1._mat3[2], m2._mat3[2]);
    _mat3[0] += tmpM;

    _mat3[1].transposedMultiply(m1._mat3[0], m2._mat3[1]);
    tmpM.transposedMultiply(m1._mat3[2], m2._mat3[3]);
    _mat3[1] += tmpM;

    _mat3[2].transposedMultiply(m1._mat3[1], m2._mat3[0]);
    tmpM.transposedMultiply(m1._mat3[3], m2._mat3[2]);
    _mat3[2] += tmpM;

    _mat3[3].transposedMultiply(m1._mat3[1], m2._mat3[1]);
    tmpM.transposedMultiply(m1._mat3[3], m2._mat3[3]);
    _mat3[3] += tmpM;
}

// this = m1 * m2^T
// [A0 A1;A2 A3]*[B0 B1;B2 B3]^T = [A0 A1;A2 A3]*[B0^T B2^T;B1^T B3^T]
//                               = [A0*B0^T+A1*B1^T A0*B2^T+A1*B3^T;
//                                  A2*B0^T+A3*B1^T A2*B2^T+A3*B3^T]
void deMatrix6::multiplyTransposed(const deMatrix6& m1, const deMatrix6& m2)
{
    deMatrix3 tmpM;

    _mat3[0].multiplyTransposed(m1._mat3[0], m2._mat3[0]);
    tmpM.multiplyTransposed(m1._mat3[1], m2._mat3[1]);
    _mat3[0] += tmpM;

    _mat3[1].multiplyTransposed(m1._mat3[0], m2._mat3[2]);
    tmpM.multiplyTransposed(m1._mat3[1], m2._mat3[3]);
    _mat3[1] += tmpM;

    _mat3[2].multiplyTransposed(m1._mat3[2], m2._mat3[0]);
    tmpM.multiplyTransposed(m1._mat3[3], m2._mat3[1]);
    _mat3[2] += tmpM;

    _mat3[3].multiplyTransposed(m1._mat3[2], m2._mat3[2]);
    tmpM.multiplyTransposed(m1._mat3[3], m2._mat3[3]);
    _mat3[3] += tmpM;
}

// X = [ R 0; dxR R ]
// L = X M = X [M00 M01; M10 M11]
//   = [ R M00  R M01; dx R M00 + R M10   dx R M01 + R M11]
// L00 = R M00
// L01 = R M01
// L10 = R M10 + dx L00
// L11 = R M11 + dx L01
void deMatrix6::xform(const deTransform& t, const deMatrix6& m)
{
  deMatrix3 tmpM;

  _mat3[0].multiply(t.rotation(), m._mat3[0]);

  _mat3[1].multiply(t.rotation(), m._mat3[1]);

  _mat3[2].multiply(t.rotation(), m._mat3[2]);
  tmpM.crossMultiply(t.translation(), _mat3[0]);
  _mat3[2] += tmpM;

  _mat3[3].multiply(t.rotation(), m._mat3[3]);
  tmpM.crossMultiply(t.translation(), _mat3[1]);
  _mat3[3] += tmpM;
}

// i-1Xi I (i-1Xi)^T = [R 0; (Px)R R][I00 I01; I10 I11][Rt -Rt(Px); 0 Rt]
// = [R I00  R I01; ((Px)R I00 + R I10) ((Px)R I01 + R I11)][Rt -Rt(Px); 0 Rt]
// = [X00 X01; X10 X11]
// X00 = R I00 Rt
// X01 = -R I00 Rt(Px) + R I01 Rt
//     = -X00(Px) + R I01 Rt
// X10 = (Px)R I00 Rt + R I10 Rt
//     = (Px)X00 + R I10 Rt
// X11 = -(Px)R I00 Rt(Px) - R I10 Rt(Px) + (Px)R I01 Rt + R I11 Rt
//     = (Px) X01 - R I10 Rt(Px) + R I11 Rt
// if I is symmetric, (TRUE!)
// X00 = R I00 Rt
// X10 = (Px)X00 + R I10 Rt 
// X01 = X10^T
// X11 = (Px) X01 - R I10 Rt(Px) + R I11 Rt
void deMatrix6::similarityXform(const deTransform& t, const deMatrix6& I)
{
    deMatrix3 tmpM, RIRt;

    // X00 = R I00 Rt
    tmpM.multiply(t.rotation(), I._mat3[0]);
    _mat3[0].multiplyTransposed(tmpM, t.rotation());

    // X10 = (Px)X00 + R I10 Rt 
    _mat3[2].crossMultiply(t.translation(), _mat3[0]);
    tmpM.multiply(t.rotation(), I._mat3[2]);
    RIRt.multiplyTransposed(tmpM, t.rotation());
    _mat3[2] += RIRt;

	// X01 = X10^T
	_mat3[1].transpose(_mat3[2]);

    // X11 = (Px) X01 - R I10 Rt(Px) + R I11 Rt
    _mat3[3].crossMultiply(t.translation(), _mat3[1]);
    
    tmpM.multiplyCross(RIRt, t.translation());
    _mat3[3] -= tmpM;

    tmpM.multiply(t.rotation(), I._mat3[3]);
    RIRt.multiplyTransposed(tmpM, t.rotation());
    _mat3[3] += RIRt;
}

// i-1Xi^T I (i-1Xi) = [Rt -Rt(Px); [I00 I01; [R    0;
//                       0      Rt]  I10 I11] (Px)R R]
// = [(Rt I00 - Rt (Px) I10)  (Rt I01 - Rt (Px) I11);   [R    0;
//                  (Rt I10)                (Rt I11)]   (Px)R R]
// = [(Rt I00 R - Rt (Px) I10 R + Rt I01 (Px)R - Rt (Px) I11 (Px)R)  (Rt I01 R - Rt (Px) I11 R);
//                    (Rt I10 R +                     Rt I11 (Px)R)                  (Rt I11 R)]
// = [X00 X01; X10 X11]
// X00 = Rt I00 R - Rt (Px) I10 R + Rt I01 (Px)R - Rt (Px) I11 (Px)R
// X01 = Rt I01 R - Rt (Px) I11 R
// X10 = Rt I10 R + Rt I11 (Px)R
// X11 = Rt I11 R
// if I is symmetric, (TRUE!)
// X11 = Rt I11 R
// X10 = Rt I10 R + Rt I11 (Px) R
// X01 = X10^T
// X00 = Rt I00 R - Rt (Px) I10 R + Rt I01 (Px) R - Rt (Px) I11 (Px) R
void deMatrix6::similarityXformT(const deTransform& t, const deMatrix6& I)
{
	deMatrix3 tmpM, PxR;

	// X11 = Rt I11 R
	tmpM.transposedMultiply(t.rotation(), I._mat3[3]);
	_mat3[3].multiply(tmpM, t.rotation());

	// X10 = Rt I10 R + Rt I11 (Px) R
	PxR.crossMultiply(t.translation(), t.rotation());
	_mat3[2].multiply(tmpM, PxR);
	tmpM.multiply(I._mat3[2], t.rotation());
	_mat3[1].transposedMultiply(t.rotation(), tmpM);
	_mat3[2] += _mat3[1];

	// X00 = Rt I00 R - Rt (Px) I10 R + Rt I01 (Px) R - Rt (Px) I11 (Px) R
	_mat3[0].transposedMultiply(tmpM, PxR);
	tmpM.transpose(_mat3[0]);
	_mat3[0] += tmpM;
	_mat3[1].transposedMultiply(PxR, I._mat3[3]);
	tmpM.multiply(_mat3[1], PxR);
	_mat3[0] += tmpM;
	_mat3[1].transposedMultiply(t.rotation(), I._mat3[0]);
	tmpM.multiply(_mat3[1], t.rotation());
	_mat3[0] += tmpM;

	// X01 = X10^T
	_mat3[1].transpose(_mat3[2]);
}

// iXh L (iXh)^T = [Rt 0; -Rt(Px) Rt][L00 L01; L10 L11][R (Px)R; 0 R]
// = [Rt L00  Rt L01;(-Rt(Px)L00 + Rt L10) (-Rt(Px)L01 + Rt L11)][R (Px)R;0 R]
// = [X00 X01; X10 X11]
// X00 = Rt L00 R
// X01 = Rt L00 (Px)R + Rt L01 R
//     = Rt [L00 (Px) + L01] R
// X10 = -Rt(Px) L00 R + Rt L10 R
//     = Rt [L10 - (Px) L00] R
//     = Rt K R
//  K  = [L10 - (Px) L00]
// X11 = -Rt(Px) L00 (Px)R + Rt L10 (Px)R - Rt(Px) L01 R + Rt L11 R
//     = Rt [-(Px) L00 (Px) + L10 (Px) - (Px) L01 +  L11 ] R
//     = Rt [ [L10 - (Px) L00] (Px) + L11 - (Px) L01 ] R
//     = Rt [ K (Px) + L11 - (Px) L01 ] R
// if L is symmetric, (TRUE!)
// X00 = Rt L00 R
//  K  = [L10 - (Px) L00]
// X10 = Rt K R
// X01 = X10^T
// X11 = Rt [ K (Px) + L11 - (Px) L01 ] R

void deMatrix6::similarityXformInv(const deTransform& t, const deMatrix6& L)
{
    deMatrix3 K;

    // X00 = Rt L00 R
    _mat3[1].transposedMultiply(t.rotation(), L._mat3[0]);
    _mat3[0].multiply(_mat3[1], t.rotation());
	
    // K  = [L10 - (Px) L00]
    // X10 = Rt K R
    K = L._mat3[2];
    _mat3[1].crossMultiply(t.translation(), L._mat3[0]);
    K -= _mat3[1];
    _mat3[1].transposedMultiply(t.rotation(), K);
    _mat3[2].multiply(_mat3[1], t.rotation());
	
    // X11 = Rt [ K (Px) + L11 - (Px) L01 ] R
    _mat3[3].multiplyCross(K, t.translation());
    _mat3[3] += L._mat3[3];
    _mat3[1].crossMultiply(t.translation(), L._mat3[1]);
    _mat3[3] -= _mat3[1];
    _mat3[1].transposedMultiply(t.rotation(), _mat3[3]);
    _mat3[3].multiply(_mat3[1], t.rotation());

	// X01 = X10^T
	_mat3[1].transpose(_mat3[2]);
}

// ^0L_e = [R 0; 0 R][L00 L01; L10 L11][Rt 0; 0 Rt]
// = [R L00  R L01; R L10  R L11][Rt 0; 0 Rt]
// = [R L00 Rt  R L01 Rt; R L10 Rt  R L11 Rt]
// = [X00 X01; X10 X11]
// X00 = R L00 Rt
// X10 = R L10 Rt
// X01 = X10^T  if I01 = I10^T
// X11 = R L11 Rt
void deMatrix6::similarityRform(const deTransform& t, const deMatrix6& L)
{
	// X00 = R L00 Rt
	_mat3[1].multiply(t.rotation(), L._mat3[0]);
	_mat3[0].multiplyTransposed(_mat3[1], t.rotation());

	// X10 = R L10 Rt
	_mat3[1].multiply(t.rotation(), L._mat3[2]);
	_mat3[2].multiplyTransposed(_mat3[1], t.rotation());

	// X11 = R L11 Rt
	_mat3[1].multiply(t.rotation(), L._mat3[3]);
	_mat3[3].multiplyTransposed(_mat3[1], t.rotation());

	// X01 = X10^T
	_mat3[1].transpose(_mat3[2]);
}

/* Matrix Inverse by Crout's LU decomposition */
/* p275-285, Numerical Methods for Engineers by Chadea and Canale */
void deMatrix6::inverse(const deMatrix6& m)
{
	deInt i, k;
	deMatrix6 lu;
	deVector6 x, y;

	/*
	 * LU ainv = I
	 * a x = y
	 * LU x = y
	 */

	lu.ludecomp(m);

	for(k = 0; k < 6; k++)
	{
		y.zero();
		y.elementAt(k) = 1;

		x.backSub(lu, y);
		for(i = 0; i < 6; i++)
			elementAt(i, k) = x.elementAt(i);
	}
}

void deMatrix6::ludecomp(const deMatrix6& m)
{
	deInt i, j, k;

	for (k = 0; k < 6; k++)
	{
		elementAt(k, k) = m.elementAt(k, k);
		for (i = 0; i < k; i++)
		{
			elementAt(i, k) = m.elementAt(i, k);
			elementAt(k, i) = m.elementAt(k, i);
			for (j = 0; j < i; j++)
			{
				elementAt(i, k) -= elementAt(i, j) * elementAt(j, k);
				elementAt(k, i) -= elementAt(k, j) * elementAt(j, i);
			}
			elementAt(i, k) /= elementAt(i, i);
			elementAt(k, k) -= elementAt(k, i) * elementAt(i, k);
		}
	}
}

/* Matrix Inverse by Cholesky's LU decomposition */
/* p288-290, Numerical Methods for Engineers by Chadea and Canale */
/* (symmetric and positive definite) */
void deMatrix6::inverseSPD(const deMatrix6& m)
{
	deInt i, k;
	deMatrix6 lu;
	deVector6 x, y;

	/*
	 * LU ainv = I
	 * a x = y
	 * LU x = y
	 */

	lu.ludecompSPD(m);

	for(k = 0; k < 6; k++)
	{
		y.zero();
		y.elementAt(k) = 1;

		x.backSubSPD(lu, y);
		for(i = 0; i < 6; i++)
			elementAt(i, k) = x.elementAt(i);
	}
}

/* (symmetric and positive definite) */
void deMatrix6::ludecompSPD(const deMatrix6& m)
{
	deInt i, j, k;

	/* U = L' */
	for (k = 0; k < 6; k++)
	{
		elementAt(k, k) = m.elementAt(k, k);
		for (j = 0; j < k; j++) 
		{
			elementAt(k, j) = m.elementAt(j, k);
			for (i = 0; i < j; i++)
				elementAt(k, j) -= elementAt(k, i) * elementAt(j, i);
			elementAt(k, j) /= elementAt(j, j);
			elementAt(k, k) -= elementAt(k, j) * elementAt(k, j);
		}
		elementAt(k, k) = deSqrt(elementAt(k, k));
	}
}

