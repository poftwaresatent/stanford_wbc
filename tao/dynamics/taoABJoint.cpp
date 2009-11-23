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


#include "taoABJoint.h"
#include "taoVar.h"

void taoABJointDOF1::update_localX(const deTransform& home, const deFrame& localFrame)
{
	deTransform local;

	if (_S[1].dot(_S[1]) > DE_QUATERNION_EPSILON)
		local.rotation().set(_S[1], getVarDOF1()->_Q);
	else
		local.identity();

	local.translation().multiply(_S[0], getVarDOF1()->_Q);
	
	localX().multiply(home, local);
}

// Vi = hXi^T Vh + Si dqi;
// xform = [R 0; dxR R]
// xformT = [ Rt -Rtdx; 0 Rt ]
void taoABJointDOF1::plusEq_SdQ(deVector6& V)
{
	deVector6 tmpV6;
	tmpV6.multiply(_S, getVarDOF1()->_dQ);
	V += tmpV6;
}

// Ci = Wi X Vi - Xt (Wh X Vh) + Vi X Si dqi
// V X = [v0 ; v1] X = [ v1x , v0x ; 0 , v1x]
// WxV = [ 0 ; v1 ] x [ v0 ; v1 ]
//     = [ v1x , 0 ; 0 , v1x ] [ v0 ; v1 ] = [v1 x v0 ;v1 x v1] = [ v1 x v0 ; 0 ]
//     = [ wxv ; 0 ]
// Xt * WxV = [ Rt -Rtdx; 0 Rt ] [ wxv ; 0 ] = [ Rt WxV ; 0 ]
void taoABJointDOF1::plusEq_V_X_SdQ(deVector6& C, const deVector6& V)
{
	deVector6 tmpV6;
	tmpV6.crossMultiply(V, _S);
	tmpV6 *= getVarDOF1()->_dQ;
	C += tmpV6;
}

// Dinv = inv(St Ia S)
// SbarT = Ia S Dinv
// hLi = hXi [ 1 - Si Sbari ]^T = hXi [1 - Sbari^T Si^T] = X - X SbarT St
// Lt = [1 - S Sbar] Xt
void taoABJointDOF1::compute_Dinv_and_SbarT(const deMatrix6& Ia)
{
	deVector6 IaS;
	IaS.multiply(Ia, _S);
	_Dinv = _S.dot(IaS) + getInertia();
	_Dinv = 1/_Dinv;

	_SbarT.multiply(IaS, _Dinv);
}

void taoABJointDOF1::minusEq_X_SbarT_St(deMatrix6& L, const deTransform& localX)
{
	deVector6 tmpV6;
	tmpV6.xform(localX, _SbarT);
	deMatrix6 tmpM6;
	tmpM6.multiplyTransposed(tmpV6, _S);
	L -= tmpM6;
}

void taoABJointDOF1::plusEq_S_Dinv_St(deMatrix6& Omega)
{
	deMatrix6 SDiSt;
	SDiSt.multiplyTransposed(_S, _S);
	SDiSt *= _Dinv;
	Omega += SDiSt;
}

/*
void taoABJointRevolute::plusEq_S_Dinv_St(deMatrix6& Omega)
{
	Omega[1][1][_axis][_axis] += _Dinv;
}

void taoABJointPrismatic::plusEq_S_Dinv_St(deMatrix6& Omega)
{
	Omega[0][0][_axis][_axis] += _Dinv;
}
*/

// Pah = Ph - Fexth + sum [ Li (Iai Ci + Pai) + X SbarTi taui ]
void taoABJointDOF1::plusEq_X_SbarT_Tau(deVector6& Pah, const deTransform& localX)
{
	deVector6 tmpV6;
	tmpV6.xform(localX, _SbarT);
	tmpV6 *= getVarDOF1()->_Tau;
	Pah += tmpV6;
}

void taoABJointDOF1::compute_Tau(const deVector6& F)
{
// see taoABNode::netForce()
	getVarDOF1()->_Tau = _S.dot(F) + getVarDOF1()->_ddQ * getInertia();
}

// ddQ = Dinv*(tau - St*Pa) - Sbar*(X Ah + Ci)
// Ai = (hXi^T Ah + Ci) + Si ddqi;
void taoABJointDOF1::compute_ddQ(const deVector6& Pa, const deVector6& XAh_C)
{
	getVarDOF1()->_ddQ = _Dinv * (getVarDOF1()->_Tau - _S.dot(Pa)) - _SbarT.dot(XAh_C);
}
void taoABJointDOF1::compute_ddQ_zeroTau(const deVector6& Pa, const deVector6& XAh_C)
{
	getVarDOF1()->_ddQ = -_Dinv * _S.dot(Pa) - _SbarT.dot(XAh_C);
}
void taoABJointDOF1::compute_ddQ_zeroTauPa(const deVector6& XAh_C)
{
	getVarDOF1()->_ddQ = -_SbarT.dot(XAh_C);
}

void taoABJointDOF1::plusEq_SddQ(deVector6& A)
{
	deVector6 tmpV6;
	tmpV6.multiply(_S, getVarDOF1()->_ddQ);
	A += tmpV6;
}

void taoABJointDOF1::minusEq_SdQ_damping(deVector6& B, const deMatrix6& Ia)
{
	deVector6 tmpV;
	tmpV.multiply(Ia, _S);
	tmpV *= getVarDOF1()->_dQ * (- getDamping());
	B -= tmpV;
}

// 0Jn = Jn = iXn^T Si = 0Xi^(-T) Si
// where 0Xi^(-T) = [ R rxR; 0 R ]
void taoABJointDOF1::compute_Jg(const deTransform &globalX)
{ 
	_Jg.xformInvT(globalX, _S);
}

// Ag += Jg * ddQ
void taoABJointDOF1::plusEq_Jg_ddQ(deVector6& Ag)
{
	deVector6 JddQ;
	JddQ.multiply(_Jg, getVarDOF1()->_dQ);
	Ag += JddQ;
}

// Tau += JgT * F
void taoABJointDOF1::add2Tau_JgT_F(const deVector6& Fg)
{
	getVarDOF1()->_Tau += _Jg.dot(Fg);
}

// F += S * inertia * ddQ
void taoABJointDOF1::plusEq_S_inertia_ddQ(deVector6& F, const deVector6& A)
{
	deVector6 tau;
	tau[0].multiply(_S[0], A[0]);
	tau[1].multiply(_S[1], A[1]);
	tau *= getInertia();
	F += tau;
}

void taoABJointSpherical::update_localX(const deTransform& home, const deFrame& localFrame)
{
	deMatrix3 r;
	r.set(getVarSpherical()->_Q);
	localX().rotation().multiply(home.rotation(), r);
	localX().translation() = localFrame.translation();
}

// Vi = hXi^T Vh + Si dqi;
// xform = [R 0; dxR R]
// xformT = [ Rt -Rtdx; 0 Rt ]
void taoABJointSpherical::plusEq_SdQ(deVector6& V)
{
	V[1] += getVarSpherical()->_dQ;
}

// Ci = Wi X Vi - Xt (Wh X Vh) + Vi X Si dqi
// V X = [v0 ; v1] X = [ v1x , v0x ; 0 , v1x]
// WxV = [ 0 ; v1 ] x [ v0 ; v1 ]
//     = [ v1x , 0 ; 0 , v1x ] [ v0 ; v1 ] = [v1 x v0 ;v1 x v1] = [ v1 x v0 ; 0 ]
//     = [ wxv ; 0 ]
// Xt * WxV = [ Rt -Rtdx; 0 Rt ] [ wxv ; 0 ] = [ Rt WxV ; 0 ]
// V X [0 ; dq] = [ v1x , v0x ; 0 , v1x] [0 ; dq] = [ v0 x dq ; v1 x dq]
void taoABJointSpherical::plusEq_V_X_SdQ(deVector6& C, const deVector6& V)
{
	deVector3 tmpV3;
	tmpV3.crossMultiply(V[0], getVarSpherical()->_dQ);
	C[0] += tmpV3;
	tmpV3.crossMultiply(V[1], getVarSpherical()->_dQ);
	C[1] += tmpV3;
}

// Dinv = inv(St Ia S)
// SbarT = Ia S Dinv
// hLi = hXi [ 1 - Si Sbari ]^T = hXi [1 - Sbari^T Si^T] = X - X SbarT St
// Lt = [1 - S Sbar] Xt
void taoABJointSpherical::compute_Dinv_and_SbarT(const deMatrix6& Ia)
{
	deMatrix3 I = Ia[1][1];
	I[0][0] += getInertia();
	I[1][1] += getInertia();
	I[2][2] += getInertia();
	_Dinv.inverseDetSPD(I);

	_SbarT[0].multiply(Ia[0][1], _Dinv);
	_SbarT[1].multiply(Ia[1][1], _Dinv);
}

// Xform = [R 0; dxR R]
// L_ = X[1 - SbarT St] = X - X SbarT St
// SbarT St = [0, SbarT0; 0 SbarT1]
// X SbarT St = [R ,0; dxR, R][0, SbarT0; 0 SbarT1]=[0,R SbarT0;0, dxR SbarT0 + R SbarT1]
// L = X - X SbarT St = [ R , -R SbarT0; dxR , R - (dxR SbarT0 + R SbarT1) ]
//                                            = R (1 - SbarT1) - dxR SbarT0
void taoABJointSpherical::minusEq_X_SbarT_St(deMatrix6& L, const deTransform& localX)
{
	deMatrix3 tmpM31, tmpM32;
	tmpM31.multiply(localX.rotation(), _SbarT[0]);
	L[0][1] -= tmpM31;
	tmpM32.multiply(localX.rotation(), _SbarT[1]);
	L[1][1] -= tmpM32;
	tmpM32.crossMultiply(localX.translation(), tmpM31);
	L[1][1] -= tmpM32;
}

// Pah = Ph - Fexth + sum [ Li (Iai Ci + Pai) + X SbarTi taui ]
void taoABJointSpherical::plusEq_X_SbarT_Tau(deVector6& Pah, const deTransform& localX)
{
	deVector6 tmpV61, tmpV62;
	tmpV61[0].multiply(_SbarT[0], getVarSpherical()->_Tau);
	tmpV61[1].multiply(_SbarT[1], getVarSpherical()->_Tau);
	tmpV62.xform(localX, tmpV61);
	Pah += tmpV62;
}

void taoABJointSpherical::compute_Tau(const deVector6& F)
{
// see taoABNode::netForce()
	getVarSpherical()->_Tau.multiply(getVarSpherical()->_ddQ, getInertia());
	getVarSpherical()->_Tau += F[1];
}

// ddQ = Dinv*(tau - St*Pa) - Sbar*(X Ah + Ci)
// Ai = (X Ah + Ci) + Si ddQi;
void taoABJointSpherical::compute_ddQ(const deVector6& Pa, const deVector6& XAh_C)
{
	// ddQ = - Sbar*(X Ah + Ci) = (SbarT)^t * (X Ah + Ci)
	compute_ddQ_zeroTauPa(XAh_C);

	deVector3 tmpV, tmpV1;
	// ddQ += Dinv * (tau - Pa)
	// St Pa = Pa[1]
	tmpV1.subtract(getVarSpherical()->_Tau, Pa[1]);
	tmpV.multiply(_Dinv, tmpV1);
	getVarSpherical()->_ddQ += tmpV;
}

// ddQ = Dinv*(- St*Pa) - Sbar*(X Ah + Ci)
// Ai = (X Ah + Ci) + Si ddQi;
void taoABJointSpherical::compute_ddQ_zeroTau(const deVector6& Pa, const deVector6& XAh_C)
{
	deVector3 tmpV;

	// ddQ = - Sbar*(X Ah + Ci) = (SbarT)^t * (X Ah + Ci)
	compute_ddQ_zeroTauPa(XAh_C);

	// ddQ += Dinv * (- Pa)
	// St Pa = Pa[1]
	tmpV.multiply(_Dinv, Pa[1]);  
	getVarSpherical()->_ddQ -= tmpV;
}

// ddQ = - Sbar*(X Ah + Ci) = (SbarT)^t * (X Ah + Ci)
void taoABJointSpherical::compute_ddQ_zeroTauPa(const deVector6& XAh_C)
{
	deVector3 tmpV;
	tmpV.transposedMultiply(_SbarT[0], XAh_C[0]);
	getVarSpherical()->_ddQ.negate(tmpV);
	tmpV.transposedMultiply(_SbarT[1], XAh_C[1]);
	getVarSpherical()->_ddQ -= tmpV;
}

void taoABJointSpherical::plusEq_SddQ(deVector6& A)
{
	A[1] += getVarSpherical()->_ddQ;
}

void taoABJointSpherical::minusEq_SdQ_damping(deVector6& B, const deMatrix6& Ia)
{
	deVector3 tmpV;
	tmpV.multiply(Ia[1][1], getVarSpherical()->_dQ);
	tmpV *= (- getDamping());
	B[1] -= tmpV;
}

void taoABJointSpherical::plusEq_S_Dinv_St(deMatrix6& Omega)
{
	Omega[1][1] += _Dinv;
}

// 0Jn = Jn = iXn^T Si = 0Xi^(-T) Si
// where 0Xi^(-T) = [ R rxR; 0 R ]
void taoABJointSpherical::compute_Jg(const deTransform &globalX) 
{ 
	_Jg[0].crossMultiply(globalX.translation(), globalX.rotation());
	_Jg[1] = globalX.rotation(); 
}

// Ag += Jg * ddQ
void taoABJointSpherical::plusEq_Jg_ddQ(deVector6& Ag)
{
	deVector6 JddQ;
	JddQ[0].multiply(_Jg[0], getVarSpherical()->_dQ);
	JddQ[1].multiply(_Jg[1], getVarSpherical()->_dQ);
	Ag += JddQ;
}

// Tau += JgT * F
void taoABJointSpherical::add2Tau_JgT_F(const deVector6& Fg)
{
	deVector3 JtF, JtF1;
	JtF.transposedMultiply(_Jg[0], Fg[0]);
	JtF1.transposedMultiply(_Jg[1], Fg[1]);
	JtF += JtF1;
	getVarSpherical()->_Tau += JtF;
}

// F += S * inertia * ddQ
void taoABJointSpherical::plusEq_S_inertia_ddQ(deVector6& F, const deVector6& A)
{
	deVector3 tau;
	tau.multiply(A[1], getInertia());
	F[1] += tau;
}
