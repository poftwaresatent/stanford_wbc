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


#include "taoCNode.h"
#include <tao/matrix/TaoDeMath.h>

// v1m = -u1 dot (V1[0] - p1x V1[1])
// v2m = -u2 dot (V2[0] - p2x V2[1])
// conservation of momentum: m1*v1p + m2*v2p = m1*v1m + m2*v2m
// definition of coef. of restitution: dvp = -e*dvm where dv = v1 - v2
// combine to solve for v1p and v2p
// v1p = (m1*v1m + m2*v2m - m2*e*dvm)/(m1 + m2) = (m1*v1m + m2*(v2m - e*dvm))/(m1 + m2)
// v2p = v1p + e*dvm
// y1 = m1*(v1p - v1m)*u1
// y2 = m2*(v2p - v2m)*u2
// for pc 2 pc --> Ui === Uj
deInt taoCNode::impact2(taoCNode* ni, const deVector3* Pie, const deVector3* Ui,
						taoCNode* nj, const deVector3* Pje, const deVector3* Uj)
{
	deInt donei, donej;
	deFloat mi, vpi, vmi, mj, vpj, vmj, edvm;
	deVector3 vi, vj;

	if (ni->getIsFixed())
	{
		vj.negate(*Uj);
		return nj->impact1(Pje, &vj, ni->_cor, ni->_cofg);
	}
	else if (nj->getIsFixed())
		return ni->impact1(Pie, Ui, nj->_cor, nj->_cofg);

	donei = donej = 1;

	mi = ni->effectiveMass(Pie, Ui);
	if (mi == 0)
		return nj->impact1(Pje, Uj, ni->_cor, ni->_cofg);

	mj = nj->effectiveMass(Pje, Uj);
	// YYY : little bit slower; recompute effectiveMass for ni!
	if (mj == 0)
		return ni->impact1(Pie, Ui, nj->_cor, nj->_cofg);

	ni->linearVelocity(&vi, Pie);  // v = Vie
	vmi = Ui->dot(vi);

	nj->linearVelocity(&vj, Pje);  // v = Vje
	vmj = Uj->dot(vj);

	edvm = ((deFloat)0.5) * (ni->_cor + nj->_cor) * (vmi - vmj);

	// v1p = (m1*v1m + m2*v2m - m2*e*dvm)/(m1 + m2) = (m1*v1m + m2*(v2m - e*dvm))/(m1 + m2)
	vpi = (mi * vmi + mj * (vmj - edvm)) / (mi + mj);
	// v2p = v1p + e*dvm
	vpj = vpi + edvm;

	donei = ni->_Impact1(Pie, Ui, &vi, mi, vpi, vmi, nj->_cofg);

	vi.negate(*Uj);
	donej = nj->_Impact1(Pje, &vi, &vj, mj, -vpj, -vmj, ni->_cofg);

	return (donei && donej);
}

deInt taoCNode::impact1(const deVector3* Pie, const deVector3* Ui, const deFloat cor2, const deFloat cofg2)
{
	deFloat m, vp, vm;
	deVector3 v;

	if (_isFixed)
		return 1;

	m = effectiveMass(Pie, Ui);

	if (m == 0)
		return 1;

	linearVelocity(&v, Pie);	// v = Vce1
	vm = Ui->dot(v);

	vp = -((deFloat)0.5) * (_cor + cor2) * vm;

	return _Impact1(Pie, Ui, &v, m, vp, vm, cofg2);
}

deInt taoCNode::_Impact1(const deVector3* Pie, const deVector3* Ui, const deVector3* V, const deFloat m, const deFloat vp, const deFloat vm, const deFloat cofg2)
{
	deFloat vtmag;
	deVector3 v, ut;

	ut.multiply(*Ui, vm);
	ut.subtract(*V, ut);
	vtmag = ut.dot(ut);

	if (vtmag > 0)	// test if there's tangential velocity component
		ut *= 1 / deSqrt(vtmag);

	if (vtmag >= 0) // fricion not related to the normal impulse.
		_Friction(Pie, Ui, &ut, vtmag, m);

	if (vp < vm)	// test if it is colliding
	{
		v.multiply(*Ui, m * (vp - vm));				// Y = Yn + 0
		if (vtmag > 0)	// test if there's tangential velocity component
		{
			ut *= m * (vp - vm) * (deFloat)0.5 * (_cofg + cofg2);	// Yt
			v += ut;								// Y = Yn + Yt
		}
		impulse(Pie, &v);
		// check if Vt+ is opposit to Vt- then set Vt+ = 0
		return 0;
	}

	return 1;
}

void taoCNode::_Friction(const deVector3* Pie, const deVector3* Ui, const deVector3* Ut, const deFloat vtmag, const deFloat m)
{
	deFloat an, atmag;
	deVector3 a, ft;

	// at the contact point in local frame
	// find acceleration, Ac = Xt Ai
	linearAcceleration(&a, Pie);
	// find normal acceleration, An = Ui dot Ac
	an = Ui->dot(a);
	if (an > 0)	// test if colliding nomal acceleration component
	{
		if (vtmag > 0)  // test if there's tangential velocity component
		{
			// find normal force, Fn = m * An
			ft.multiply(*Ut, - m * an * _cofd - _cofv * vtmag);
			// find tangential force Ft = cof * Fn * Ut
			// apply Ft at {i}  Fti = X Ft
			force(Pie, &ft);	// friction due to tangential vel and normal force
		}
		else // no tangential velocity component
		{
			ft.multiply(*Ui, an);
			ft.subtract(a, ft);		// At
			atmag = ft.dot(ft);
			if (atmag > 0) // test if there's tangential acceleration component
			{
				ft *=  - m * an * _cofs / deSqrt(atmag);
				force(Pie, &ft);	// friction due to tangential acceleration and normal force
			}
		}
	}
}

deInt taoCNode::penetration1(const deVector3* Pie, const deVector3* Ui, const deVector3* pdist, const deFloat dt)
{
	deFloat m, vm;
	deVector3 v, y;

	m = effectiveMass(Pie, Ui);

	if (m == 0)
		return 1;

	linearVelocity(&v, Pie);	// v = Vce1
	vm = Ui->dot(*pdist) - Ui->dot(v) * dt;

	if (vm < 0)	// test if it is penetrating
	{
		y.multiply(*Ui, m * vm);				// Y = Yn + 0
		impulseDist(Pie, &y);
		return 0;
	}

	return 1;
}
