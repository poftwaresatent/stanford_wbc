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


#include "taoABNode.h"
#include "taoABJoint.h"

#include <assert.h>

void taoABNodeNOJ::abInertiaInit(deMatrix6& Ia)
{ 
	Ia = *I();
}

void taoABNodeNOJ::impulseInit(const deVector3& point, const deVector3& impulse)
{
	deVector6& Ya = *Pa();

	// Ya = -Xc Yc
	//    = -[R , 0; dxR , R] [ y, 0]
	//    = [-Ry ; -dxRy]
	Ya[0].negate(impulse);
	Ya[1].crossMultiply(point, Ya[0]);
}


// Pa -= Fext
void taoABNodeNOJ::externalForce(deVector6& Pa, const deVector6& G, const deVector6& Fext)
{
	Pa -= G;
	Pa -= Fext;
}

// Iah	= Ih + sum [ Li Iai Lti ] --> does not work with joint inertia
// since D != S^t Ia S but D == S^t Ia S + joint inertia
// Solution:
// Iah	= Ih + sum [ Li Iai Xti ] --> does work with joint inertia
void taoABNodeNOJ::_abInertia(deMatrix6& Iah, const deMatrix6& L, const deMatrix6& Ia, const deTransform& X)
{
	deMatrix6 tmpM6, m1 ,m2;
	//tmpM6.similarityXform(L, Ia);
	m1.multiply(L, Ia);
	m2.set(X);
	tmpM6.multiplyTransposed(m1, m2); // YYY: optimize this
	Iah += tmpM6;
}
// Pah = Ph - Fexth + sum [ Li (Iai Ci + Pai) + X SbarTi taui ]
void taoABNodeNOJ::_abBiasForce(deVector6& Pah, const deMatrix6& L, const deMatrix6& Ia, const deVector6& C, const deVector6& Pa)
{
	deVector6 tmpV61, tmpV62;

	tmpV61.multiply(Ia, C);
	tmpV61 += Pa;
	tmpV62.multiply(L, tmpV61);
	Pah += tmpV62;
}


// Ii = Xc Ic Xtc
//    = [ RMRt, -RMRt rx; rx RMRt, RIRt - rx RMRt rx]
//    = [ M, -Mrx; rxM, RIRt - m rx rx]
void taoABNodeNOJ::inertia(const deFloat* mass, const deVector3* centerOfMass, const deMatrix3* inertiaTensor)
{
	//  inertia = RIRt - mass()*rx*rx
	//  Ic = RIRt = inertia + mass()*rx*rx // change inertia

	deVector3 mr;
	mr.multiply(*centerOfMass, *mass);
	_I[0][1].cross(mr);  // mass()*rx
	_Ic.multiplyCross(_I[0][1], *centerOfMass); // mass()*rx*rx
	_Ic += *inertiaTensor; // Ic = RIRt

	_I[0][0].zero();
	_I[0][0][0][0] = _I[0][0][1][1] = _I[0][0][2][2] = *mass;
	_I[0][1].negate(_I[0][1]);        // -mass()*rx
	_I[1][0].transpose(_I[0][1]);
	_I[1][1] = *inertiaTensor;
}

// FIX:  assuming Pi for deJoint = 0
//
// Pi = Xc (Wc X Ic Vc) - Ii (Wi X Vi)  
//    = Xc (Wc X [mi vc; Ic wc]) -Ii (Wi X Vi)
void taoABNodeNOJ::biasForce(deVector6& P, const deVector6& V, const deVector3& WxV)
{
	// R = I_3;
	// Vc = Xtc Vi
	//    = Xt * V = [ Rt -Rtdx; 0 Rt ] [ v ; w ] = [ v-dxw ; w ]
	deVector6 tmpV6;
	deFloat mass = _I[0][0][0][0];

	// mi vc = m ( v - dx w) = m v - m dx w = m v - I[1][0] w 
	// Ic wc = Ic w
	tmpV6[0].multiply(V[0], mass);
	tmpV6[1].multiply(_I[1][0], V[1]);

	tmpV6[0] -= tmpV6[1];			// mi vc
	tmpV6[1].multiply(_Ic, V[1]);	// Ic wc

	// Wc X tmpV61 = [ 0; w] X tmpV61 = [wx, 0 ; 0, wx] tmpV61
	P[0].crossMultiply(V[1], tmpV6[0]);
	P[1].crossMultiply(V[1], tmpV6[1]);

	// P = Xc (Wc X Ic Vc) = [ 1 , 0; rx , 1] [ p0 ; p1] = [p0; rx p0 + p1]
	tmpV6[0].multiply(_I[1][0], P[0]);
	
	if (mass > 1e-9) { // otherwise these terms are (pretty darn near) zero anyway
	  tmpV6[0] *= 1/mass;
	  P[1] += tmpV6[0];
	}
	
	//  I (W X V) = [I00,I01;I10,I11]*[WxV;0] = [I00*WxV;I10*WxV]
	tmpV6[0].multiply(_I[0][0], WxV);
	tmpV6[1].multiply(_I[1][0], WxV);
	P -= tmpV6;
}

deFloat taoABNodeNOJ::kineticEnergy(deVector6& V, const deVector6& Vh)
{
	deVector3 WxV, WhxVh;
	deVector6 IV;

	WhxVh.zero();

	velocity(V, WxV, Vh, WhxVh);
	
	IV.multiply(*I(), V);

	return V.dot(IV) / 2;
}

deFloat taoABNodeNOJ::potentialEnergy(const deVector3& gh, const deFrame& globalFrame, const deFloat mass, const deVector3& centerOfMass)
{
	deVector3 h;

	h.multiply(globalFrame.rotation(), centerOfMass);
	h += globalFrame.translation();
			
	return -mass * gh.dot(h);	
}

void taoABNodeNOJ1::updateLocalX(const deFrame& homeFrame, const deFrame& localFrame)
{
	deTransform homeX;
	homeX.set(homeFrame);
	_joint->update_localX(homeX, localFrame);
}

void taoABNodeNOJ1::getFrameLocal(deFrame& localFrame)
{
	localFrame.set(_joint->localX());
}

void taoABNodeNOJn::updateLocalX(const deFrame& homeFrame, const deFrame& localFrame)
{
	deTransform homeX;
	deFrame f;
	f.identity();

	homeX.set(homeFrame);
	_joint[0]->update_localX(homeX, f);

	homeX.identity();
	for (deInt i = 1; i < getNOJ(); i++)
		_joint[i]->update_localX(homeX, f);

}

void taoABNodeNOJn::getFrameLocal(deFrame& localFrame)
{
	deTransform localX, localX2;
	localX = _joint[0]->localX();

	for (deInt i = 1; i < getNOJ(); i++)
	{
		localX2 = localX;
		localX.multiply(localX2, _joint[i]->localX());
	}

	localFrame.set(localX);
}

// 0Je = Je = iXe^T Si = 0Xi^(-T) Si
// where 0Xi^(-T) = [ R rxR; 0 R ]
// since 0Re = identity
// and  iXe^T = [(inv 0Xi) 0Xe]^T = (0Xe)^T * (0Xi)^(-T) = 0Xi ^(-T)
// since  0Xe = identity matrix    <--  {e} = {0}
void taoABNodeNOJ1::globalJacobian(const deFrame& globalFrame)
{
	deTransform Xg;

	Xg.set(globalFrame);

	_joint->compute_Jg(Xg);
}

// A += J * ddQ
void taoABNodeNOJ1::plusEq_Jg_ddQ(deVector6& Ag)
{
	_joint->plusEq_Jg_ddQ(Ag);
}

// Tau += Jt * F
void taoABNodeNOJ1::add2Tau_JgT_F(const deVector6& Fg)
{
	_joint->add2Tau_JgT_F(Fg);
}

// 0Xi = 0Xh hXi
// 0Xh = 0Xi hXi^-1
void taoABNodeNOJn::globalJacobian(const deFrame& globalFrame)
{
	deInt i;
	deTransform Xg, Xgh;

	Xg.set(globalFrame);

	for (i = _noj - 1; i > 0; i--)
	{
		_joint[i]->compute_Jg(Xg);

		Xgh.multiplyInversed(Xg, _joint[i]->localX());
		Xg = Xgh;
	}
	_joint[i]->compute_Jg(Xg);
}

// A += J * ddQ
void taoABNodeNOJn::plusEq_Jg_ddQ(deVector6& Ag)
{
	deInt i;
	for (i = _noj - 1; i >= 0; i--)
		_joint[i]->plusEq_Jg_ddQ(Ag);
}

// Tau += Jt * F
void taoABNodeNOJn::add2Tau_JgT_F(const deVector6& Fg)
{
	deInt i;
	for (i = _noj - 1; i >= 0; i--)
		_joint[i]->add2Tau_JgT_F(Fg);
}

// Gi = Xc Gc
// Gc = [ mi Rtci gi ; 0]
// gi = Rt gh
void taoABNodeNOJ1::gravityForce(deVector6& G, deVector3& g, const deVector3& gh)
{
	g.transposedMultiply(_joint->localX().rotation(), gh);	

	G[0].multiply(g, (*I())[0][0][0][0]);
	// [1,0;rx,1][g;0] = [g;rxg]
	G[1].multiply((*I())[1][0], g);
}

void taoABNodeNOJn::gravityForce(deVector6& G, deVector3& g, const deVector3& gh)
{
	deVector3 gp = gh;

	for (deInt i = 0; i < _noj; i++)
	{
		g.transposedMultiply(_joint[i]->localX().rotation(), gp);
		gp = g;
	}


	G[0].multiply(g, (*I())[0][0][0][0]);
	// [1,0;rx,1][g;0] = [g;rxg]
	G[1].multiply((*I())[1][0], g);
}

void taoABNodeNOJ1::velocityOnly(deVector6& V, const deVector6& Vh)
{
	V.xformT(_joint->localX(), Vh);
	_joint->plusEq_SdQ(V);
}

void taoABNodeNOJn::velocityOnly(deVector6& V, const deVector6& Vh)
{
	deVector6 Vp = Vh;

	for (deInt i = 0; i < _noj; i++)
	{
		V.xformT(_joint[i]->localX(), Vp);
		_joint[i]->plusEq_SdQ(V);

		Vp = V;
	}
}

// Vi = hXi^T Vh + Si dqi;
// xform = [R 0; dxR R]
// xformT = [ Rt -Rtdx; 0 Rt ]

// Ci = Wi X Vi - Xt (Wh X Vh) + Vi X Si dqi
// V X = [v0 ; v1] X = [ v1x , v0x ; 0 , v1x]
// WxV = [ 0 ; v1 ] x [ v0 ; v1 ]
//     = [ v1x , 0 ; 0 , v1x ] [ v0 ; v1 ] = [v1 x v0 ;v1 x v1] = [ v1 x v0 ; 0 ]
//     = [ wxv ; 0 ]
// Xt * WxV = [ Rt -Rtdx; 0 Rt ] [ wxv ; 0 ] = [ Rt WxV ; 0 ]

void taoABNodeNOJ1::velocity(deVector6& V, deVector3& WxV, const deVector6& Vh, const deVector3& WhxVh)
{
	V.xformT(_joint->localX(), Vh);
	_joint->plusEq_SdQ(V);

	WxV.crossMultiply(V[1], V[0]);
	_joint->C()[0].transposedMultiply(_joint->localX().rotation(), WhxVh);
	_joint->C()[0].subtract(WxV, _joint->C()[0]);
	_joint->C()[1].zero();
	_joint->plusEq_V_X_SdQ(_joint->C(), V);
}

void taoABNodeNOJ1::biasAcceleration(deVector6& H, const deVector6& Hh)
{
	H.xformT(_joint->localX(), Hh);
	H += _joint->C();
}

void taoABNodeNOJn::velocity(deVector6& V, deVector3& WxV, const deVector6& Vh, const deVector3& WhxVh)
{
	deVector6 Vp = Vh;
	deVector3 WpxVp = WhxVh;

	for (deInt i = 0; i < _noj; i++)
	{
		V.xformT(_joint[i]->localX(), Vp);
		_joint[i]->plusEq_SdQ(V);

		WxV.crossMultiply(V[1], V[0]);

		_joint[i]->C()[0].transposedMultiply(_joint[i]->localX().rotation(), WhxVh);
		_joint[i]->C()[0].subtract(WxV, _joint[i]->C()[0]);
		_joint[i]->C()[1].zero();
		_joint[i]->plusEq_V_X_SdQ(_joint[i]->C(), V);
	
		Vp = V;
		WpxVp = WxV;
	}
}

void taoABNodeNOJn::biasAcceleration(deVector6& H, const deVector6& Hh)
{
	deVector6 Hp = Hh;

	for (deInt i = 0; i < _noj; i++)
	{
		H.xformT(_joint[i]->localX(), Hp);
		H += _joint[i]->C();
		Hp = H;
	}
}

// Dinv = inv(St Ia S)
// SbarT = Ia S Dinv
// hLi = hXi [ 1 - Si Sbari ]^T = hXi [1 - Sbari^T Si^T] = X - X SbarT St
// Lt = [1 - S Sbar] Xt

void taoABNodeNOJ1::abInertiaDepend(deMatrix6& Iah, deVector6& Pah, deMatrix6& Ia, deInt propagate)
{
	_joint->minusEq_SdQ_damping(_joint->Pa(), Ia);

	_joint->compute_Dinv_and_SbarT(Ia);

	if (propagate)
	{
		_joint->L().set(_joint->localX());

		_joint->minusEq_X_SbarT_St(_joint->L(), _joint->localX());

		_abInertia(Iah, _joint->L(), Ia, _joint->localX());
		_abBiasForce(Pah, _joint->L(), Ia, _joint->C(), _joint->Pa());
		_joint->plusEq_X_SbarT_Tau(Pah, _joint->localX());
	}
}

void taoABNodeNOJn::abInertiaDepend(deMatrix6& Iah, deVector6& Pah, deMatrix6& Ia, deInt propagate)
{
	deMatrix6 Iap;

	for (deInt i = _noj - 1; i > 0; i--)
	{
		_joint[i]->minusEq_SdQ_damping(_joint[i]->Pa(), Ia);
		_joint[i]->compute_Dinv_and_SbarT(Ia);

		_joint[i]->L().set(_joint[i]->localX());
		_joint[i]->minusEq_X_SbarT_St(_joint[i]->L(), _joint[i]->localX());

		_joint[i - 1]->Pa().zero();
		_abBiasForce(_joint[i - 1]->Pa(), _joint[i]->L(), Ia, _joint[i]->C(), _joint[i]->Pa());
		_joint[i]->plusEq_X_SbarT_Tau(_joint[i - 1]->Pa(), _joint[i]->localX());

		Iap.zero();
		_abInertia(Iap, _joint[i]->L(), Ia, _joint[i]->localX());

		Ia = Iap;
	}

	_joint[0]->minusEq_SdQ_damping(_joint[0]->Pa(), Ia);

	_joint[0]->compute_Dinv_and_SbarT(Ia);

	if (propagate)
	{
		_joint[0]->L().set(_joint[0]->localX());

		_joint[0]->minusEq_X_SbarT_St(_joint[0]->L(), _joint[0]->localX());

		_abInertia(Iah, _joint[0]->L(), Ia, _joint[0]->localX());
		_abBiasForce(Pah, _joint[0]->L(), Ia, _joint[0]->C(), _joint[0]->Pa());
		_joint[0]->plusEq_X_SbarT_Tau(Pah, _joint[0]->localX());
	}
}

// O = S Dinv St + Lt Oh L
void taoABNodeNOJ1::osInertiaInv(deMatrix6& Oa, const deMatrix6& Oah)
{
	Oa.similarityXformT(_joint->L(), Oah);
	_joint->plusEq_S_Dinv_St(Oa);
}

// O = S Dinv St + Lt Oh L
void taoABNodeNOJn::osInertiaInv(deMatrix6& Oa, const deMatrix6& Oah)
{
	deMatrix6 tmpM6;

	Oa = Oah;
	for (deInt i = 0; i < _noj; i++)
	{
		tmpM6 = Oa;
		Oa.similarityXformT(_joint[i]->L(), tmpM6);
		_joint[i]->plusEq_S_Dinv_St(Oa);
	}
}

void taoABNodeNOJ1::abImpulse(deVector6& Yah, deInt propagate)
{
	if (propagate)
		Yah.multiply(_joint->L(), _joint->Pa());
}

void taoABNodeNOJn::abImpulse(deVector6& Yah, deInt propagate)
{
	for (deInt i = _noj - 1; i > 0; i--)
		_joint[i - 1]->Pa().multiply(_joint[i]->L(), _joint[i]->Pa());

	if (propagate)
		Yah.multiply(_joint[0]->L(), _joint[0]->Pa());
}

// C = Ph = 0
// Pah = - Fexth + sum [ Li (Pai) + X SbarTi taui ]
void taoABNodeNOJ1::abBiasForceConfig(deVector6& Pah, deInt propagate)
{
	// store fwdIn() in dq_ to be used by JointAcceleration(1)
	//_joint[0]->read_dQ(fwdIn, fwdInSphere);

	if (propagate)
	{
		Pah.multiply(_joint->L(), _joint->Pa());
		_joint->plusEq_X_SbarT_Tau(Pah, _joint->localX());
	}
}

void taoABNodeNOJn::abBiasForceConfig(deVector6& Pah, deInt propagate)
{
	// store fwdIn() in dq_ to be used by JointAcceleration(1)
	//_joint[0]->read_dQ(fwdIn, fwdInSphere);

	for (deInt i = _noj - 1; i > 0; i--)
	{
		_joint[i - 1]->Pa().multiply(_joint[i]->L(), _joint[i]->Pa());
		_joint[i]->plusEq_X_SbarT_Tau(_joint[i - 1]->Pa(), _joint[i]->localX());
	}
	if (propagate)
	{
		Pah.multiply(_joint[0]->L(), _joint[0]->Pa());
		_joint[0]->plusEq_X_SbarT_Tau(Pah, _joint[0]->localX());
	}
}

// ddQ = Dinv*(tau - St*Pa) - Sbar*(X Ah + Ci)
// Ai = (X Ah + Ci) + Si ddQi;
void taoABNodeNOJ1::acceleration(deVector6& A, const deVector6& Ah)
{
	A.xformT(_joint->localX(), Ah);
	A += _joint->C();

	_joint->compute_ddQ(_joint->Pa(), A);
	_joint->plusEq_SddQ(A);
}

void taoABNodeNOJn::acceleration(deVector6& A, const deVector6& Ah)
{
	deVector6 Ap = Ah;

	for (deInt i = 0; i < _noj; i++)
	{
		A.xformT(_joint[i]->localX(), Ap);
		A += _joint[i]->C();
		_joint[i]->compute_ddQ(_joint[i]->Pa(), A);
		_joint[i]->plusEq_SddQ(A);
		Ap = A;
	}
}

void taoABNodeNOJ1::accelerationOnly(deVector6& A, const deVector6& Ah)
{
	A.xformT(_joint->localX(), Ah);
	A += _joint->C();
	_joint->plusEq_SddQ(A);
}

void taoABNodeNOJn::accelerationOnly(deVector6& A, const deVector6& Ah)
{
	deVector6 Ap = Ah;

	for (deInt i = 0; i < _noj; i++)
	{
		A.xformT(_joint[i]->localX(), Ap);
		A += _joint[i]->C();
		_joint[i]->plusEq_SddQ(A);
		Ap = A;
	}
}

void taoABNodeNOJ1::netForce(deVector6& F, const deVector6& A, const deVector6& P)
{
	F.multiply(*I(), A);
	F += P;
	// see taoABJoint::compute_Tau()
}

void taoABNodeNOJn::netForce(deVector6& F, const deVector6& A, const deVector6& P)
{
	F.multiply(*I(), A);
	F += P;
	// see taoABJoint::compute_Tau()
}

void taoABNodeNOJ1::force(deVector6& Fh, deInt propagate)
{
// no damping
	if (propagate)
	{
		deVector6 tmpV;
		tmpV.xform(_joint->localX(), _joint->Pa());
		Fh += tmpV;
	}
	_joint->compute_Tau(_joint->Pa());
}

void taoABNodeNOJn::force(deVector6& Fh, deInt propagate)
{
	for (deInt i = _noj - 1; i > 0; i--)
	{
		_joint[i - 1]->Pa().xform(_joint[i]->localX(), _joint[i]->Pa());
		_joint[i]->compute_Tau(_joint[i]->Pa());
	}

	if (propagate)
	{
		deVector6 tmpV;
		tmpV.xform(_joint[0]->localX(), _joint[0]->Pa());
		Fh += tmpV;
	}
	_joint[0]->compute_Tau(_joint[0]->Pa());
}

// d dQ = Dinv*(- St*Pa) - Sbar*(X dVh)
// dVi = (X dVh) + Si d dQi;

// ddQ = Dinv*(tau - St*Pa) - Sbar*(X Ah)
// Ai = (X Ah) + Si ddQi;
void taoABNodeNOJ1::velocityDelta(deVector6& dV, const deVector6& dVh, const deInt dist)
{
	dV.xformT(_joint->localX(), dVh);
	if (getFlag())
		_joint->compute_ddQ_zeroTau(_joint->Pa(), dV);
	else
		_joint->compute_ddQ_zeroTauPa(dV);

	if (dist)
		_joint->addQdelta();
	else
		_joint->addDQdelta();

	_joint->plusEq_SddQ(dV);

	setFlag(0);
}

void taoABNodeNOJn::velocityDelta(deVector6& dV, const deVector6& dVh, const deInt dist)
{
	deVector6 dVp = dVh;

	for (deInt i = 0; i < _noj; i++)
	{
		dV.xformT(_joint[i]->localX(), dVp);
		if (getFlag())
			_joint[i]->compute_ddQ_zeroTau(_joint[i]->Pa(), dV);		
		else
			_joint[i]->compute_ddQ_zeroTauPa(dV);

		if (dist)
			_joint[i]->addQdelta();
		else
			_joint[i]->addDQdelta();

		_joint[i]->plusEq_SddQ(dV);
		dVp = dV;
	}

	setFlag(0);
}

