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

#ifndef _taoABNode_h
#define _taoABNode_h

#include "taoABJoint.h"
#include <tao/matrix/TaoDeMath.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
 *	\brief		Articulated body node class
 *	\ingroup	taoDynamics
 *
 *	This class provides node for articulated body.
 */
class taoABNode
{
public:
	taoABNode() { _V.zero(); _A.zero(); _H.zero(), _Omega.zero(); }

	virtual ~taoABNode() {}

	virtual void setFlag(deInt v) = 0;
	virtual const deInt getFlag() const = 0;

	virtual deVector6* Pa() = 0;

	virtual const deMatrix6* I() const = 0;

	virtual deMatrix6* Omega() { return &_Omega; }
	virtual deVector6* H() { return &_H; }
	virtual deVector6* V() { return &_V; }
	virtual deVector6* A() { return &_A; }

	// Ia = I
	virtual void abInertiaInit(deMatrix6& Ia) = 0;

	// Ya = -Xc Yc
	//    = -[R , 0; dxR , R] [ y, 0]
	//    = [-Ry ; -dxRy]
	virtual void impulseInit(const deVector3& point, const deVector3& impulse) = 0;


	// Ii = Xc Ic Xtc
	//    = [ RMRt, -RMRt rx; rx RMRt, RIRt - rx RMRt rx]
	//    = [ M, -Mrx; rxM, RIRt - m rx rx]
	virtual void inertia(const deFloat* mass, const deVector3* centerOfMass, const deMatrix3* inertiaTensor) = 0;

	// Pi = Xc (Wc X Ic Vc) - Ii (Wi X Vi)  
	//    = Xc (Wc X [mi vc; Ic wc]) -Ii (Wi X Vi)
	virtual void biasForce(deVector6& P, const deVector6& V, const deVector3& WxV) = 0;

	// Dinv = inv(St Ia S)
	// SbarT = Ia S Dinv
	// hLi = hXi [ 1 - Si Sbari ]^T = hXi [1 - Sbari^T Si^T] = X - X SbarT St
	// Lt = [1 - S Sbar] Xt
	// Iah = Ih + sum [ Li Iai Lti ]
	virtual void _abInertia(deMatrix6& Iah, const deMatrix6& L, const deMatrix6& Ia, const deTransform& X) = 0;

	// Pah = Ph - Fexth + sum [ Li (Iai Ci + Pai) + X SbarTi taui ]
	virtual void _abBiasForce(deVector6& Pah, const deMatrix6& L, const deMatrix6& Ia, const deVector6& C, const deVector6& Pa) = 0;

	virtual void netForce(deVector6& F, const deVector6& A, const deVector6& P) = 0;
	// Pa -= Fext
	virtual void externalForce(deVector6& Pa, const deVector6& G, const deVector6& Fext) = 0;

	virtual void updateLocalX(const deFrame& homeFrame, const deFrame& localFrame) = 0;

	virtual void getFrameLocal(deFrame& localFrame) = 0;

	virtual void abImpulse(deVector6& Yah, deInt propagate) = 0;
	// 0Je = Je = iXe^T Si = 0Xi^(-T) Si
	// where 0Xi^(-T) = [ R rxR; 0 R ]
	// since 0Re = identity
	// and  iXe^T = [(inv 0Xi) 0Xe]^T = (0Xe)^T * (0Xi)^(-T) = 0Xi ^(-T)
	// since  0Xe = identity matrix    <--  {e} = {0}
	virtual void globalJacobian(const deFrame& globalFrame) = 0;
	// A += J * ddQ
	virtual void plusEq_Jg_ddQ(deVector6& Ag) = 0;
	// Tau += Jt * F
	virtual void add2Tau_JgT_F(const deVector6& Fg) = 0;
	// Gi = Xc Gc
	// Gc = [ mi Rtci gi ; 0]
	// gi = Rt gh
	virtual void gravityForce(deVector6& Fext, deVector3& g, const deVector3& gh) = 0;

	// Vi = hXi^T Vh + Si dqi;
	// xform = [R 0; dxR R]
	// xformT = [ Rt -Rtdx; 0 Rt ]
	// Ci = Wi X Vi - Xt (Wh X Vh) + Vi X Si dqi
	// V X = [v0 ; v1] X = [ v1x , v0x ; 0 , v1x]
	// WxV = [ 0 ; v1 ] x [ v0 ; v1 ]
	//     = [ v1x , 0 ; 0 , v1x ] [ v0 ; v1 ] = [v1 x v0 ;v1 x v1] = [ v1 x v0 ; 0 ]
	//     = [ wxv ; 0 ]
	// Xt * WxV = [ Rt -Rtdx; 0 Rt ] [ wxv ; 0 ] = [ Rt WxV ; 0 ]
	virtual void velocity(deVector6& V, deVector3& WxV, const deVector6& Vh, const deVector3& WhxVh) = 0;
	virtual void velocityOnly(deVector6& V, const deVector6& Vh) = 0;
	// Hi = hXi^T Hh + Ci
	virtual void biasAcceleration(deVector6& H, const deVector6& Hh) = 0;
	// Vi = hXi^T Vh + Si dqi;
	// E = 0.5 Vt I V
	virtual deFloat kineticEnergy(deVector6& V, const deVector6& Vh) = 0;
	// E = mgh
	virtual deFloat potentialEnergy(const deVector3& gh, const deFrame& globalFrame, const deFloat mass, const deVector3& centerOfMass) = 0;


	// ddQ = Dinv*(tau - St*Pa) - Sbar*(X Ah + Ci)
	// Ai = (hXi^T Ah + Ci) + Si ddqi;
	virtual void acceleration(deVector6& A, const deVector6& Ah) = 0;
	virtual void accelerationOnly(deVector6& A, const deVector6& Ah) = 0;
	// ddQ = Dinv*(tau - St*Pa) - Sbar*(X Ah)
	// Ai = (hXi^T Ah) + Si ddqi;
	// d dQ = Dinv*(- St*Pa) - Sbar*(X dVh)
	// dVi = (hXi^T dVh) + Si d dqi;
	virtual void velocityDelta(deVector6& dV, const deVector6& dVh, const deInt dist) = 0;

	virtual void force(deVector6& Fh, deInt propagate) = 0;

	// C = Ph = 0
	// Pah = - Fexth + sum [ Li (Pai) + X SbarTi taui ]
	virtual void abBiasForceConfig(deVector6& Pah, deInt propagate) = 0;
	virtual void abInertiaDepend(deMatrix6& Iah, deVector6& Pah, deMatrix6& Ia, deInt propagate) = 0;
	// O = S Dinv St + Lt Oh L
	virtual void osInertiaInv(deMatrix6& Oa, const deMatrix6& Oah) = 0;

	virtual void setABJoint(taoABJoint* joint, deInt i = 0) = 0;
	virtual taoABJoint* getABJoint(deInt i = 0) = 0;

	virtual void setNOJ(deInt n) = 0;
	virtual const deInt getNOJ() const = 0;

private:
	deVector6 _V;
	deVector6 _A;
	deVector6 _H;		
	deMatrix6 _Omega;	// Op Sp Matrix inverse : diagonal terms
};

class taoABNodeRoot : public taoABNode
{
public:
	virtual const deInt getFlag() const { return 0; }
	virtual void setFlag(deInt v) {}

	virtual deVector6* Pa() { return NULL; }
	virtual const deMatrix6* I() const { return NULL; }

	virtual void abInertiaInit(deMatrix6& Ia) {}
	virtual void impulseInit(const deVector3& point, const deVector3& impulse) {}

	virtual void inertia(const deFloat* mass, const deVector3* centerOfMass, const deMatrix3* inertiaTensor) {}
	virtual void biasForce(deVector6& P, const deVector6& V, const deVector3& WxV) {}
	virtual void _abInertia(deMatrix6& Iah, const deMatrix6& L, const deMatrix6& Ia, const deTransform& X) {}
	virtual void _abBiasForce(deVector6& Pah, const deMatrix6& L, const deMatrix6& Ia, const deVector6& C, const deVector6& Pa) {}
	virtual void netForce(deVector6& F, const deVector6& A, const deVector6& P) {}
	virtual void externalForce(deVector6& Pa, const deVector6& G, const deVector6& Fext) {}

	virtual void updateLocalX(const deFrame& homeFrame, const deFrame& localFrame) {}
	virtual void getFrameLocal(deFrame& localFrame) {}

	virtual void abImpulse(deVector6& Yah, deInt propagate) {}
	virtual void globalJacobian(const deFrame& globalFrame) {}
	virtual void plusEq_Jg_ddQ(deVector6& Ag) {}
	virtual void add2Tau_JgT_F(const deVector6& Fg) {}

	virtual void gravityForce(deVector6& Fext, deVector3& g, const deVector3& gh) { g = gh; }
	virtual void velocity(deVector6& V, deVector3& WxV, const deVector6& Vh, const deVector3& WhxVh) { WxV.crossMultiply(V[1], V[0]); }
	virtual void velocityOnly(deVector6& V, const deVector6& Vh) {}
	virtual void biasAcceleration(deVector6& H, const deVector6& Hh) { H.zero(); }
	virtual deFloat kineticEnergy(deVector6& V, const deVector6& Vh) { return 0; }
	virtual deFloat potentialEnergy(const deVector3& gh, const deFrame& globalFrame, const deFloat mass, const deVector3& centerOfMass) { return 0; }
	virtual void acceleration(deVector6& A, const deVector6& Ah) {}
	virtual void accelerationOnly(deVector6& A, const deVector6& Ah) {}
	virtual void velocityDelta(deVector6& dV, const deVector6& dVh, const deInt dist) { dV.zero(); }

	virtual void force(deVector6& Fh, deInt propagate) {}

	virtual void abBiasForceConfig(deVector6& Pah, deInt propagate) {}
	virtual void abInertiaDepend(deMatrix6& Iah, deVector6& Pah, deMatrix6& Ia, deInt propagate) {}
	virtual void osInertiaInv(deMatrix6& Oa, const deMatrix6& Oah) {}

	virtual void setABJoint(taoABJoint* joint, deInt i = 0) {}
	virtual taoABJoint* getABJoint(deInt i = 0) { return NULL; }

	virtual void setNOJ(deInt n) {}
	virtual const deInt getNOJ() const { return 0; }
};

class taoABNodeNOJ : public taoABNode
{
public:
	taoABNodeNOJ()
	{
		_flag = 0;
		_Ic.zero();
		_I.zero();
	}
	virtual const deInt getFlag() const { return _flag; }
	virtual void setFlag(deInt v) { _flag = v; }

	virtual const deMatrix3* Ic() const { return &_Ic; }
	virtual const deMatrix6* I() const { return &_I;}

	virtual deVector6* Pa() { return &(getABJoint(getNOJ() - 1)->Pa()); }

	virtual void abInertiaInit(deMatrix6& Ia);
	virtual void impulseInit(const deVector3& point, const deVector3& impulse);

	virtual void inertia(const deFloat* mass, const deVector3* centerOfMass, const deMatrix3* inertiaTensor);
	virtual void biasForce(deVector6& P, const deVector6& V, const deVector3& WxV);
	virtual void _abInertia(deMatrix6& Iah, const deMatrix6& L, const deMatrix6& Ia, const deTransform& X);
	virtual void _abBiasForce(deVector6& Pah, const deMatrix6& L, const deMatrix6& Ia, const deVector6& C, const deVector6& Pa);
	virtual void externalForce(deVector6& Pa, const deVector6& G, const deVector6& Fext);
	virtual deFloat kineticEnergy(deVector6& V, const deVector6& Vh);
	virtual deFloat potentialEnergy(const deVector3& gh, const deFrame& globalFrame, const deFloat mass, const deVector3& centerOfMass);

	virtual void setABJoint(taoABJoint* joint, deInt i = 0) = 0;
	virtual taoABJoint* getABJoint(deInt i = 0) = 0;

	virtual void setNOJ(deInt n) = 0;
	virtual const deInt getNOJ() const = 0;

private:
	deInt _flag;
	deMatrix3 _Ic;
	deMatrix6 _I;
};

class taoABNodeNOJ1 : public taoABNodeNOJ
{
public:
	taoABNodeNOJ1() : _joint(NULL) {}

	virtual void updateLocalX(const deFrame& homeFrame, const deFrame& localFrame);
	virtual void getFrameLocal(deFrame& localFrame);
	virtual void abImpulse(deVector6& Yah, deInt propagate);
	virtual void globalJacobian(const deFrame& globalFrame);
	virtual void plusEq_Jg_ddQ(deVector6& Ag);
	virtual void add2Tau_JgT_F(const deVector6& Fg);

	virtual void gravityForce(deVector6& Fext, deVector3& g, const deVector3& gh);
	virtual void velocity(deVector6& V, deVector3& WxV, const deVector6& Vh, const deVector3& WhxVh);
	virtual void velocityOnly(deVector6& V, const deVector6& Vh);

	virtual void biasAcceleration(deVector6& H, const deVector6& Hh);
	virtual void acceleration(deVector6& A, const deVector6& Ah);
	virtual void accelerationOnly(deVector6& A, const deVector6& Ah);
	virtual void velocityDelta(deVector6& dV, const deVector6& dVh, const deInt dist);

	virtual void netForce(deVector6& F, const deVector6& A, const deVector6& P);
	virtual void force(deVector6& Fh, deInt propagate);

	virtual void abBiasForceConfig(deVector6& Pah, deInt propagate);
	virtual void abInertiaDepend(deMatrix6& Iah, deVector6& Pah, deMatrix6& Ia, deInt propagate);
	virtual void osInertiaInv(deMatrix6& Oa, const deMatrix6& Oah);

	virtual void setABJoint(taoABJoint* joint, deInt i = 0) { _joint = joint; }
	virtual taoABJoint* getABJoint(deInt i = 0) { return _joint; }

	virtual void setNOJ(deInt n) {}
	virtual const deInt getNOJ() const { return 1; }

	virtual ~taoABNodeNOJ1() { delete _joint; }

private:
	taoABJoint* _joint;
};

class taoABNodeNOJn : public taoABNodeNOJ
{
public:
	taoABNodeNOJn() : _noj(0), _joint(NULL) {}

	virtual void updateLocalX(const deFrame& homeFrame, const deFrame& localFrame);
	virtual void getFrameLocal(deFrame& localFrame);
	virtual void abImpulse(deVector6& Yah, deInt propagate);
	virtual void globalJacobian(const deFrame& globalFrame);
	virtual void plusEq_Jg_ddQ(deVector6& Ag);
	virtual void add2Tau_JgT_F(const deVector6& Fg);

	virtual void gravityForce(deVector6& Fext, deVector3& g, const deVector3& gh);
	virtual void velocity(deVector6& V, deVector3& WxV, const deVector6& Vh, const deVector3& WhxVh);
	virtual void velocityOnly(deVector6& V, const deVector6& Vh);

	virtual void biasAcceleration(deVector6& H, const deVector6& Hh);
	virtual void acceleration(deVector6& A, const deVector6& Ah);
	virtual void accelerationOnly(deVector6& A, const deVector6& Ah);
	virtual void velocityDelta(deVector6& dV, const deVector6& dVh, const deInt dist);

	virtual void netForce(deVector6& F, const deVector6& A, const deVector6& P);
	virtual void force(deVector6& Fh, deInt propagate);

	virtual void abBiasForceConfig(deVector6& Pah, deInt propagate);
	virtual void abInertiaDepend(deMatrix6& Iah, deVector6& Pah, deMatrix6& Ia, deInt propagate);
	virtual void osInertiaInv(deMatrix6& Oa, const deMatrix6& Oah);

	virtual void setABJoint(taoABJoint* joint, deInt i = 0) { _joint[i] = joint; }
	virtual taoABJoint* getABJoint(deInt i = 0) { return _joint[i]; }

        virtual void setNOJ(deInt n) { _noj = n; _joint = new taoABJoint*[_noj]; }
	virtual const deInt getNOJ() const { return _noj; }

	virtual ~taoABNodeNOJn() 
	{ 
		for (int i = 0; i < _noj; i++)
			delete _joint[i];
		delete[] _joint; 
	}

private:
	deInt _noj;
	taoABJoint** _joint;
};

#endif // DOXYGEN_SHOULD_SKIP_THIS

#endif // _taoABNode_h

