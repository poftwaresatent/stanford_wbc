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
	virtual taoDVar const * getDVar() const { return _var; }

	void setType(taoJointType t) { _type = t; }
	taoJointType getType() const { return _type; }

	void setABJoint(taoABJoint* joint) { _abJoint = joint; }
	taoABJoint* getABJoint() { return _abJoint; }
	taoABJoint const * getABJoint() const { return _abJoint; }

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
  
  /** \note pointer semantics to support more than one DOF */
	virtual void setTau(const deFloat* v) = 0;
        virtual void zeroTau() = 0;
  
  /** \note pointer semantics to support more than one DOF */
	virtual void setDDQ(const deFloat* v) = 0;
	virtual void zeroDDQ() = 0;
  
  /** \note pointer semantics to support more than one DOF */
	virtual void setDQ(const deFloat* v) = 0;
	virtual void zeroDQ() = 0;
  
  /** \note pointer semantics to support more than one DOF */
	virtual void setQ(const deFloat* v) = 0;
	virtual void zeroQ() = 0;
  
  /** \note pointer semantics to support more than one DOF */
	virtual void getTau(deFloat* v) const = 0;
  
  /** \note pointer semantics to support more than one DOF */
	virtual void getDDQ(deFloat* v) const = 0;
  
  /** \note pointer semantics to support more than one DOF */
	virtual void getDQ(deFloat* v) const = 0;
  
  /** \note pointer semantics to support more than one DOF */
	virtual void getQ(deFloat* v) const = 0;
  
  /** Retrieve the column(s) of the global Jacobian due to this
      joint. You have to pass in an array of deVector6 instances, the
      number of instances must be at least getDOF(), and each of them
      will be filled with the values of Jg_p (upper three entries) and
      Jg_w (lower three entries).
      
      \note This method was retrofitted because TAO did not provide a
      generic method of retrieving the global Jacobian. User were
      forced to jump through hoops and use dynamic casts, which is
      error prone and hard to maintain.
  */
  virtual void getJgColumns(deVector6 * Jg_columns) const = 0;
  
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
	virtual taoVarSpherical const * getVarSpherical() const { return (taoVarSpherical*)getDVar(); }

	virtual void addQdelta();
	virtual void addDQdelta();
	virtual void zeroTau() { getVarSpherical()->_Tau.zero(); }
	virtual void zeroDDQ() { getVarSpherical()->_ddQ.zero(); }
	virtual void zeroDQ() { getVarSpherical()->_dQ.zero(); getVarSpherical()->_dQrotated.zero(); }
	virtual void zeroQ() { getVarSpherical()->_Q.identity(); }
	virtual void setTau(const deFloat* v) { getVarSpherical()->_Tau.set(v); }
	virtual void setDDQ(const deFloat* v) { getVarSpherical()->_ddQ.set(v); }
	virtual void setDQ(const deFloat* v) { getVarSpherical()->_dQ.set(v); }
	virtual void setQ(const deFloat* v) { getVarSpherical()->_Q.set(v); }
	virtual void getTau(deFloat* v) const { getVarSpherical()->_Tau.get(v); }
	virtual void getDDQ(deFloat* v) const { getVarSpherical()->_ddQ.get(v); }
	virtual void getDQ(deFloat* v) const { getVarSpherical()->_dQ.get(v); }
	virtual void getQ(deFloat* v) const { deQuaternion const & q = getVarSpherical()->_Q; v[0] = q[0]; v[1] = q[1]; v[2] = q[2]; v[3] = q[3]; }

	virtual void clampDQ();
	virtual void integrate(const deFloat dt);
	virtual void updateFrameLocal(deFrame* local) { local->rotation() = getVarSpherical()->_Q; }

	/** The "documentation" used to say this (whatever that
	    means exactly): "notice that in spherical joints the
	    position components of the Jacobian are zero therefore, to
	    build a 6x3 matrix the upper 3x3 needs to be set to zero
	    and the lower needs to be set the return of this function".
	    
	    One thing the documentation did not say is that the
	    returned pointer is an array with two elements. getJg()[0]
	    contains something like (I'm still figuring it out from
	    the code...) the contribution of the joint to the
	    translational velocity, and getJg()[0] contains the
	    rotational part, both dependent on whatever frame got
	    passed to taoABJointSpherical::compute_Jg()
	    previously. taoABJointSpherical::compute_Jg() gets called
	    by taoABNodeNOJ1::globalJacobian() or
	    taoABNodeNOJn::globalJacobian(), which get called by
	    taoABDynamics::globalJacobianOut() and
	    taoABDynamics::compute_Jg_Omega_H(), both of which pass
	    taoDNode::frameGlobal() into it (it can get expressed in
	    successively retracted frames though during the iteration
	    over all the joints of a
	    node). taoDynamics::globalJacobian() seems to be the
	    top-level entry point for that chain of calls, so whatever
	    node (and its global frame) you pass to it will be used
	    throughout the recursion.
	    
	    \note This method is retained mainly for backwards
	    compatibility, you should use getJgColumns() instead,
	    which does the necessary operations to inject the
	    positional components. Looking at the taoJointDOF1
	    version, by analogy this method probably returns the
	    Jacobian contribution of this joint, expressed in the
	    global frame. Someone chose to return it as a 3x3 matrix
	    because they thought it was neat from the point of view of
	    this class, but it actually makes it hard to
	    polymorphically mix joint types, making it completely
	    pointless to declare this method virtual.
	*/
	virtual deMatrix3* getJg();
  
  /** You need to pass in an array of three deVector6 instances,
      because spherical joints have three degrees of (velocity)
      freedom. Note that getDOF() returns three, so you can use the
      polymorphic form. */
  virtual void getJgColumns(deVector6 * Jg_columns) const;
  
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
	virtual taoVarDOF1 const * getVarDOF1() const { return (taoVarDOF1*)getDVar(); }

	virtual void addQdelta() { getVarDOF1()->_Q += getVarDOF1()->_ddQ; }
	virtual void addDQdelta() { getVarDOF1()->_dQ += getVarDOF1()->_ddQ; if (getDQclamp()) clampDQ(); }
	virtual void zeroTau() { getVarDOF1()->_Tau = 0; }
	virtual void zeroDDQ() { getVarDOF1()->_ddQ = 0; }
	virtual void zeroDQ() { getVarDOF1()->_dQ = 0; }
	virtual void zeroQ() { getVarDOF1()->_Q = 0; }

	virtual void setTau(const deFloat* v) { getVarDOF1()->_Tau = *v; }
	virtual void setDDQ(const deFloat* v) { getVarDOF1()->_ddQ = *v; }
	virtual void setDQ(const deFloat* v) { getVarDOF1()->_dQ = *v; }
	virtual void setQ(const deFloat* v) { getVarDOF1()->_Q = *v; }
	virtual void getTau(deFloat* v) const { *v = getVarDOF1()->_Tau; }
	virtual void getDDQ(deFloat* v) const { *v = getVarDOF1()->_ddQ; }
	virtual void getDQ(deFloat* v) const { *v = getVarDOF1()->_dQ; }
	virtual void getQ(deFloat* v) const { *v = getVarDOF1()->_Q; }

	virtual void clampDQ();
	virtual void integrate(const deFloat dt);

	/** It looks like this method returns the Jacobian
	    contribution of this joint, expressed in the global frame,
	    and with rotations expressed as components around the
	    global coordinate axes.
	    
	    \note This method looks like a failed attempt at
	    polymorphism -- see also taoJointSpherical::getJg() -- and
	    is retained for backwards compatibility. You should use
	    getJgColumns() instead.
	*/
	virtual deVector6& getJg() const;
  
  /** You need to pass in one deVector6 instance, because DOF1 joints
      have one degree of (velocity) freedom. Note that getDOF()
      returns one, so you can use the polymorphic form. */
  virtual void getJgColumns(deVector6 * Jg_columns) const;
  
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
