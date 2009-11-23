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

#ifndef _taoJoint_h
#define _taoJoint_h

#include "taoDJoint.h"
#include "taoVar.h"
#include "taoTypes.h"
#include <tao/matrix/TaoDeMath.h>

class taoABJoint;
class taoJCParam;

/*!
 *	\brief		Base joint class for articulated body
 *	\ingroup	taoDynamics
 *
 *	This provides a joint for articulated body dynamics
 *	\sa	taoDJoint
 */
class taoJoint : public taoDJoint
{
public:
	taoJoint() : _type(TAO_JOINT_USER), _abJoint(NULL), _jcp(NULL), _clamp_dQ(0), _max_dQ(5), _damping(0), _inertia(0), _var(NULL), _next(NULL) {}
	virtual ~taoJoint();

	virtual deInt getDOF() = 0;
	virtual void reset() = 0;

	virtual void setDVar(taoDVar* var) { _var = var; }
	virtual taoDVar* getDVar() { return _var; }

	void setType(taoJointType t) { _type = t; }
	taoJointType getType() { return _type; }

	void setABJoint(taoABJoint* joint) { _abJoint = joint; }
	taoABJoint* getABJoint() { return _abJoint; }

	void setJCParam(taoJCParam* jcp) { _jcp = jcp; }
	taoJCParam* getJCParam() { return _jcp; }

	void setDQclamp(deInt b) { _clamp_dQ = b; }
	deInt getDQclamp() { return _clamp_dQ; }
	
	void setNext(taoJoint* joint) { _next = joint; }
	taoJoint* getNext() { return _next; }

	virtual void setDQmax(deFloat dq) { _max_dQ = dq; }
	virtual deFloat getDQmax() { return _max_dQ; }

	virtual void setDamping(deFloat d) { _damping = d; }
	virtual deFloat getDamping() { return _damping; }

	virtual void setInertia(deFloat i) { _inertia = i; }
	virtual deFloat getInertia() { return _inertia; }

	virtual void clampDQ() = 0;
	virtual void integrate(const deFloat dt) = 0;
	virtual void updateFrameLocal(deFrame* local) = 0;

	virtual void setTau(const deFloat* v) = 0;
	virtual void setDDQ(const deFloat* v) = 0;
	virtual void setDQ(const deFloat* v) = 0;
	virtual void setQ(const deFloat* v) = 0;
	virtual void getTau(deFloat* v) = 0;
	virtual void getDDQ(deFloat* v) = 0;
	virtual void getDQ(deFloat* v) = 0;
	virtual void getQ(deFloat* v) = 0;

private:
	taoJointType _type;
	taoABJoint* _abJoint;
	taoJCParam* _jcp;

	deInt _clamp_dQ;
	deFloat _max_dQ;

	deFloat _damping;
	deFloat _inertia;

	taoDVar* _var;
	taoJoint* _next;
};

/*!
 *	\brief		Spherical joint class for articulated body
 *	\ingroup	taoDynamics
 *
 *	This provides a spherical joint for articulated body dynamics
 *	\sa	taoJoint
 */
class taoJointSpherical : public taoJoint
{
public:
	taoJointSpherical();

	virtual deInt getDOF() { return 3; }
	virtual void reset();

	virtual taoVarSpherical* getVarSpherical() { return (taoVarSpherical*)getDVar(); }

	virtual void addQdelta();
	virtual void addDQdelta();
	virtual void zeroTau() { getVarSpherical()->_Tau.zero(); }
	virtual void setTau(const deFloat* v) { getVarSpherical()->_Tau.set(v); }
	virtual void setDDQ(const deFloat* v) { getVarSpherical()->_ddQ.set(v); }
	virtual void setDQ(const deFloat* v) { getVarSpherical()->_dQ.set(v); }
	virtual void setQ(const deFloat* v) { getVarSpherical()->_Q.set(v); }
	virtual void getTau(deFloat* v) { getVarSpherical()->_Tau.get(v); }
	virtual void getDDQ(deFloat* v) { getVarSpherical()->_ddQ.get(v); }
	virtual void getDQ(deFloat* v) { getVarSpherical()->_dQ.get(v); }
	virtual void getQ(deFloat* v) { deQuaternion& q = getVarSpherical()->_Q; v[0] = q[0]; v[1] = q[1]; v[2] = q[2]; v[3] = q[3]; }

	virtual void clampDQ();
	virtual void integrate(const deFloat dt);
	virtual void updateFrameLocal(deFrame* local) { local->rotation() = getVarSpherical()->_Q; }

	// notice that in spherical joints the position components of the Jacobian are zero
	// therefore, to build a 6x3 matrix the upper 3x3 needs to be set to zero and the lower
	// needs to be set the return of this function
	virtual deMatrix3* getJg();
};

/*!
 *	\brief		1 DOF joint class for articulated body
 *	\ingroup	taoDynamics
 *
 *	This provides a 1-DOF joint for articulated body dynamics
 *	\sa	taoJoint, taoJointPrismatic, taoJointRevolute
 */
class taoJointDOF1 : public taoJoint
{
public:
	taoJointDOF1(taoAxis axis) : _axis(axis) {}

	taoAxis getAxis() const { return _axis; }
	virtual deInt getDOF() { return 1; }

	virtual deVector6& getS();

	virtual void reset() 
	{
		getVarDOF1()->_Q = 0;
		getVarDOF1()->_dQ = 0;
		getVarDOF1()->_ddQ = 0;
		getVarDOF1()->_Tau = 0;
	}

	virtual taoVarDOF1* getVarDOF1() { return (taoVarDOF1*)getDVar(); }

	virtual void addQdelta() { getVarDOF1()->_Q += getVarDOF1()->_ddQ; }
	virtual void addDQdelta() { getVarDOF1()->_dQ += getVarDOF1()->_ddQ; if (getDQclamp()) clampDQ(); }
	virtual void zeroTau() { getVarDOF1()->_Tau = 0; }
	virtual void setTau(const deFloat* v) { getVarDOF1()->_Tau = *v; }
	virtual void setDDQ(const deFloat* v) { getVarDOF1()->_ddQ = *v; }
	virtual void setDQ(const deFloat* v) { getVarDOF1()->_dQ = *v; }
	virtual void setQ(const deFloat* v) { getVarDOF1()->_Q = *v; }
	virtual void getTau(deFloat* v) { *v = getVarDOF1()->_Tau; }
	virtual void getDDQ(deFloat* v) { *v = getVarDOF1()->_ddQ; }
	virtual void getDQ(deFloat* v) { *v = getVarDOF1()->_dQ; }
	virtual void getQ(deFloat* v) { *v = getVarDOF1()->_Q; }

	virtual void clampDQ();
	virtual void integrate(const deFloat dt);

	virtual deVector6& getJg();

private:
	taoAxis _axis;
};

/*!
 *	\brief		Prismatic joint class for articulated body
 *	\ingroup	taoDynamics
 *
 *	This provides a prismatic joint for articulated body dynamics
 *	\sa	taoJointDOF1
 */
class taoJointPrismatic : public taoJointDOF1
{
public:
	taoJointPrismatic(taoAxis axis);
	virtual void updateFrameLocal(deFrame* local) 
	{ local->translation()[getAxis()] = getVarDOF1()->_Q; }
};

/*!
 *	\brief		Revolute joint class for articulated body
 *	\ingroup	taoDynamics
 *
 *	This provides a revolute joint for articulated body dynamics
 *	\sa	taoJointDOF1
 */
class taoJointRevolute : public taoJointDOF1
{
public:
	taoJointRevolute(taoAxis axis);
	virtual void updateFrameLocal(deFrame* local)
	{
		local->rotation().set(getAxis(), getVarDOF1()->_Q);
	}
};

/*!
 *	\brief		User defined joint class for articulated body
 *	\ingroup	taoDynamics
 *
 *	This provides a user defined joint for articulated body dynamics
 *	\sa	taoJointDOF1
 */
class taoJointUser : public taoJointDOF1
{
public:
	taoJointUser();
	//virtual void updateFrameLocal(deFrame* local) = 0;
};

#endif // _taoJoint_h
