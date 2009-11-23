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

#ifndef _taoABJoint_h
#define _taoABJoint_h

#include "taoTypes.h"
#include "taoDJoint.h"
#include <tao/matrix/TaoDeMath.h>

class taoDVar;
class taoVarDOF1;
class taoVarSpherical;

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
 *	\brief		Articulated body joint class
 *	\ingroup	taoDynamics
 *
 *	This class provides joint for articulated body.
 */
class taoABJoint
{
public:
	taoABJoint(taoDJoint* joint = NULL) 
	{ 
		_joint = joint;
		_C.zero();
		_Pa.zero();
		_localX.identity(); 
		_L.zero();
	}

	virtual ~taoABJoint() {}

	virtual deVector6& C() { return _C; }
	virtual deVector6& Pa() { return _Pa; }
	virtual deTransform& localX() { return _localX; }
	virtual deMatrix6& L() { return _L; }

	virtual void update_localX(const deTransform& home, const deFrame& localFrame) = 0;
	// Vi = hXi^T Vh + Si dqi;
	// xform = [R 0; dxR R]
	// xformT = [ Rt -Rtdx; 0 Rt ]
	virtual void plusEq_SdQ(deVector6& V) = 0;
	// Ci = Wi X Vi - Xt (Wh X Vh) + Vi X Si dqi
	// V X = [v0 ; v1] X = [ v1x , v0x ; 0 , v1x]
	// WxV = [ 0 ; v1 ] x [ v0 ; v1 ]
	//     = [ v1x , 0 ; 0 , v1x ] [ v0 ; v1 ] = [v1 x v0 ;v1 x v1] = [ v1 x v0 ; 0 ]
	//     = [ wxv ; 0 ]
	// Xt * WxV = [ Rt -Rtdx; 0 Rt ] [ wxv ; 0 ] = [ Rt WxV ; 0 ]
	virtual void plusEq_V_X_SdQ(deVector6& C, const deVector6& V) = 0;
	// Dinv = inv(St Ia S)
	// SbarT = Ia S Dinv
	// hLi = hXi [ 1 - Si Sbari ]^T = hXi [1 - Sbari^T Si^T] = X - X SbarT St
	// Lt = [1 - S Sbar] Xt
	virtual void compute_Dinv_and_SbarT(const deMatrix6& Ia) = 0;
	virtual void minusEq_X_SbarT_St(deMatrix6& L, const deTransform& localX) = 0;
	// Pah = Ph - Fexth + sum [ Li (Iai Ci + Pai) + X SbarTi taui ]
	virtual void plusEq_X_SbarT_Tau(deVector6& Pah, const deTransform& localX) = 0;
	// ddQ = Dinv*(tau - St*Pa) - Sbar*(X Ah + Ci)
	// Ai = (hXi^T Ah + Ci) + Si ddqi;
	virtual void compute_ddQ(const deVector6& Pa, const deVector6& XAh_C) = 0;
	virtual void compute_ddQ_zeroTau(const deVector6& Pa, const deVector6& XAh_C) = 0;
	// d dQ = Dinv*(- St*Ya) - Sbar*(X dVh)
	// dVi = (hXi^T dVh) + Si d dqi;
	virtual void compute_ddQ_zeroTauPa(const deVector6& XAh_C) = 0;
	virtual void compute_Tau(const deVector6& F) = 0;

	virtual void plusEq_SddQ(deVector6& A) = 0;

	virtual void minusEq_SdQ_damping(deVector6& B, const deMatrix6& Ia) = 0;
	virtual void plusEq_S_Dinv_St(deMatrix6& Omega) = 0;

	// 0Jn = Jn = iXn^T Si = 0Xi^(-T) Si
	// where 0Xi^(-T) = [ R rxR; 0 R ]
	virtual void compute_Jg(const deTransform &globalX) = 0;
	// Ag += Jg * ddQ
	virtual void plusEq_Jg_ddQ(deVector6& Ag) = 0;
	// Tau += JgT * F
	virtual void add2Tau_JgT_F(const deVector6& Fg) = 0;
	// F += S * inertia * A
	virtual void plusEq_S_inertia_ddQ(deVector6& F, const deVector6& A) = 0;

	virtual deFloat getDamping() { return _joint->getDamping(); }
	virtual deFloat getInertia() { return _joint->getInertia(); }

	virtual taoDVar* getDVar() { return _joint->getDVar(); }

	virtual void zeroTau() { _joint->zeroTau(); }
	virtual void addDQdelta() { _joint->addDQdelta(); }
	virtual void addQdelta() { _joint->addQdelta(); }

private:
	taoDJoint* _joint;

	deVector6 _C;
	deVector6 _Pa;
	deTransform _localX;
	deMatrix6 _L;		
};

class taoABJointFixed : public taoABJoint
{
public:
	virtual void update_localX(const deTransform& home, const deFrame& localFrame) { localX().set(localFrame); }
	virtual void plusEq_SdQ(deVector6& V) {}
	virtual void plusEq_V_X_SdQ(deVector6& C, const deVector6& V) {}
	virtual void compute_Dinv_and_SbarT(const deMatrix6& Ia) {}
	virtual void minusEq_X_SbarT_St(deMatrix6& L, const deTransform& localX) {}
	virtual void plusEq_X_SbarT_Tau(deVector6& Pah, const deTransform& localX) {}
	virtual void compute_ddQ(const deVector6& Pa, const deVector6& XAh_C) {}
	virtual void compute_ddQ_zeroTau(const deVector6& Pa, const deVector6& XAh_C) {}
	virtual void compute_ddQ_zeroTauPa(const deVector6& XAh_C) {}
	virtual void compute_Tau(const deVector6& F) {}

	virtual void plusEq_SddQ(deVector6& A) {}
	virtual void minusEq_SdQ_damping(deVector6& B, const deMatrix6& Ia) {}
	virtual void plusEq_S_Dinv_St(deMatrix6& Omega) {}

	virtual void compute_Jg(const deTransform &globalX) {}
	virtual void plusEq_Jg_ddQ(deVector6& Ag) {}
	virtual void add2Tau_JgT_F(const deVector6& Fg) {}

	virtual void plusEq_S_inertia_ddQ(deVector6& F, const deVector6& A) {}

	virtual void zeroTau() {}
	virtual void addDQdelta() {}
	virtual void addQdelta() {}
};

class taoABJointSpherical : public taoABJoint
{
public:
	taoABJointSpherical(taoDJoint* joint) : taoABJoint(joint)
	{
		_SbarT[0].zero();
		_SbarT[1].zero();
		_Dinv.zero();
		_Jg[0].zero();
		_Jg[1].zero();
	}
	virtual void update_localX(const deTransform& home, const deFrame& localFrame);
	virtual void plusEq_SdQ(deVector6& V);
	virtual void plusEq_V_X_SdQ(deVector6& C, const deVector6& V);
	virtual void compute_Dinv_and_SbarT(const deMatrix6& Ia);
	virtual void minusEq_X_SbarT_St(deMatrix6& L, const deTransform& localX);
	virtual void plusEq_X_SbarT_Tau(deVector6& Pah, const deTransform& localX);
	virtual void compute_ddQ(const deVector6& Pa, const deVector6& XAh_C);
	virtual void compute_ddQ_zeroTau(const deVector6& Pa, const deVector6& XAh_C);
	virtual void compute_ddQ_zeroTauPa(const deVector6& XAh_C);
	virtual void compute_Tau(const deVector6& F);

	virtual void plusEq_SddQ(deVector6& A);
	virtual void minusEq_SdQ_damping(deVector6& B, const deMatrix6& Ia);
	virtual void plusEq_S_Dinv_St(deMatrix6& Omega);

	virtual void compute_Jg(const deTransform &globalX);
	virtual void plusEq_Jg_ddQ(deVector6& Ag);
	virtual void add2Tau_JgT_F(const deVector6& Fg);

	virtual void plusEq_S_inertia_ddQ(deVector6& F, const deVector6& A);

	virtual taoVarSpherical* getVarSpherical() { return (taoVarSpherical*)getDVar(); }

	virtual deMatrix3* Jg() { return _Jg; }

private:
	deMatrix3 _SbarT[2];
	deMatrix3 _Dinv;
	deMatrix3 _Jg[2];
};

class taoABJointDOF1 : public taoABJoint
{
public:
	taoABJointDOF1(taoDJoint* joint) : taoABJoint(joint)
	{
		_S.zero();
		_SbarT.zero();
		_Dinv = 0;
		_Jg.zero();
	}
	virtual void update_localX(const deTransform& home, const deFrame& localFrame);
	virtual void plusEq_SdQ(deVector6& V);
	virtual void plusEq_V_X_SdQ(deVector6& C, const deVector6& V);
	virtual void compute_Dinv_and_SbarT(const deMatrix6& Ia);
	virtual void minusEq_X_SbarT_St(deMatrix6& L, const deTransform& localX);
	virtual void plusEq_X_SbarT_Tau(deVector6& Pah, const deTransform& localX);
	virtual void compute_ddQ(const deVector6& Pa, const deVector6& XAh_C);
	virtual void compute_ddQ_zeroTau(const deVector6& Pa, const deVector6& XAh_C);
	virtual void compute_ddQ_zeroTauPa(const deVector6& XAh_C);
	virtual void compute_Tau(const deVector6& F);

	virtual void plusEq_SddQ(deVector6& A);
	virtual void minusEq_SdQ_damping(deVector6& B, const deMatrix6& Ia);
	virtual void plusEq_S_Dinv_St(deMatrix6& Omega);

	virtual void compute_Jg(const deTransform &globalX);
	virtual void plusEq_Jg_ddQ(deVector6& Ag);
	virtual void add2Tau_JgT_F(const deVector6& Fg);

	virtual void plusEq_S_inertia_ddQ(deVector6& F, const deVector6& A);

	virtual taoVarDOF1* getVarDOF1() { return (taoVarDOF1*)getDVar(); }

	virtual deVector6& S() { return _S; }

	virtual deVector6& Jg() { return _Jg; }

private:
	deVector6 _S;
	deVector6 _SbarT;
	deFloat _Dinv;
	deVector6 _Jg;
};

class taoABJointPrismatic : public taoABJointDOF1
{
public:
	taoABJointPrismatic(taoAxis axis, taoDJoint* joint) : taoABJointDOF1(joint)
	{
		deVector6 unit;
		unit.zero();
		unit[0][axis] = 1;
		S() = unit;
	}

	taoABJointPrismatic(const deVector3& axis, taoDJoint* joint) : taoABJointDOF1(joint)
	{
		deVector6 unit;
		unit.zero();
		unit[0] = axis;
		unit[0].normalize();
		S() = unit;
	}
};

class taoABJointRevolute : public taoABJointDOF1
{
public:
	taoABJointRevolute(taoAxis axis, taoDJoint* joint) : taoABJointDOF1(joint)
	{
		deVector6 unit;
		unit.zero();
		unit[1][axis] = 1;
		S() = unit;
	}

	taoABJointRevolute(const deVector3& axis, taoDJoint* joint) : taoABJointDOF1(joint)
	{
		deVector6 unit;
		unit.zero();
		unit[1] = axis;
		unit[1].normalize();
		S() = unit;
	}
};

#endif // DOXYGEN_SHOULD_SKIP_THIS

#endif // _taoABJoint_h

