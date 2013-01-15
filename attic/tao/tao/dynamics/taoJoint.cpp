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

#include <tao/dynamics/taoJoint.h>
#include "taoABJoint.h"

taoJoint::~taoJoint() 
{ 
	delete _var;
	//delete _abJoint;  // done in ~taoABNode()
}

taoJointSpherical::taoJointSpherical() 
{ 
	setABJoint(new taoABJointSpherical(this)); 
	setType(TAO_JOINT_SPHERICAL);
}

void taoJointSpherical::reset()
{
	getVarSpherical()->_Q.identity();
	getVarSpherical()->_dQ.zero();
	getVarSpherical()->_ddQ.zero();
	getVarSpherical()->_Tau.zero();
	getVarSpherical()->_dQrotated.zero();
}

void taoJointSpherical::addQdelta()
{
	deVector3 dq0;
	deQuaternion dq;

	dq0.multiply(getVarSpherical()->_Q, getVarSpherical()->_ddQ);
	dq.velocity(getVarSpherical()->_Q, dq0);
	getVarSpherical()->_Q += dq;		// YYY: consistentSign(); ?
	getVarSpherical()->_Q.normalize();
}

void taoJointSpherical::addDQdelta()
{
	deVector3 ddq;
	ddq.multiply(getVarSpherical()->_Q, getVarSpherical()->_ddQ);
	getVarSpherical()->_dQrotated += ddq;
	getVarSpherical()->_dQ.inversedMultiply(getVarSpherical()->_Q, getVarSpherical()->_dQrotated);

	if (getDQclamp()) 
		clampDQ();
}

void taoJointSpherical::clampDQ()
{
	deInt changed = 0;
	for (deInt i = 0; i < 3; i++)
		if (getVarSpherical()->_dQ[i] > getDQmax())
		{
			getVarSpherical()->_dQ[i] = getDQmax();
			changed = 1;
		}
		else if (getVarSpherical()->_dQ[i] < -getDQmax())
		{
			getVarSpherical()->_dQ[i] = -getDQmax();
			changed = 1;
		}
	if (changed)
		getVarSpherical()->_dQrotated.multiply(getVarSpherical()->_Q, getVarSpherical()->_dQ);
}

void taoJointSpherical::integrate(const deFloat dt)
{
	deQuaternion dq;
	dq.velocity(getVarSpherical()->_Q, getVarSpherical()->_dQrotated);
	dq *= dt;
	getVarSpherical()->_Q += dq;		// YYY: consistentSign(); ?
	getVarSpherical()->_Q.normalize();

	deVector3 ddq;
	ddq.multiply(getVarSpherical()->_Q, getVarSpherical()->_ddQ);
	deVector3 tmp;	
	tmp.multiply(ddq, dt);
	getVarSpherical()->_dQrotated += tmp;

	getVarSpherical()->_dQ.inversedMultiply(getVarSpherical()->_Q, getVarSpherical()->_dQrotated);

	if (getDQclamp()) 
		clampDQ();
}

deMatrix3* taoJointSpherical::getJg()
{
	return ((taoABJointSpherical*)getABJoint())->Jg();
}

void taoJointSpherical::getJgColumns(deVector6 * Jg_columns) const
{
  deMatrix3 const * Jg_p(((taoABJointSpherical*)getABJoint())->Jg());
  deMatrix3 const * Jg_w(Jg_p + 1);
  for (size_t icol(0); icol < 3; ++icol) {
    deVector3 & col_p(Jg_columns[icol][0]);
    deVector3 & col_w(Jg_columns[icol][1]);
    for (size_t isubrow(0); isubrow < 3; ++isubrow) {
      col_p[isubrow] = Jg_p->elementAt(isubrow, icol);
      col_w[isubrow] = Jg_w->elementAt(isubrow, icol);
    }
  }
}

deVector6& taoJointDOF1::getJg() const
{
	return ((taoABJointDOF1*)getABJoint())->Jg();
}

void taoJointDOF1::getJgColumns(deVector6 * Jg_columns) const
{
  deVector6 const & Jg(((taoABJointDOF1*)getABJoint())->Jg());
  *Jg_columns = Jg;
}

deVector6& taoJointDOF1::getS()
{
	return ((taoABJointDOF1*)getABJoint())->S();
}

void taoJointDOF1::integrate(const deFloat dt)
{
	getVarDOF1()->_Q += getVarDOF1()->_dQ * dt;
	getVarDOF1()->_dQ += getVarDOF1()->_ddQ * dt;

	if (getDQclamp()) 
		clampDQ();
}

void taoJointDOF1::clampDQ()
{
	if (getVarDOF1()->_dQ > getDQmax())
		getVarDOF1()->_dQ = getDQmax();
	else if (getVarDOF1()->_dQ < -getDQmax())
		getVarDOF1()->_dQ = -getDQmax();
}

taoJointPrismatic::taoJointPrismatic(taoAxis axis) : taoJointDOF1(axis) 
{ 
	setABJoint(new taoABJointPrismatic(axis, this)); 
	setType(TAO_JOINT_PRISMATIC);
}

taoJointRevolute::taoJointRevolute(taoAxis axis) : taoJointDOF1(axis)
{ 
	setABJoint(new taoABJointRevolute(axis, this));
	setType(TAO_JOINT_REVOLUTE);
}

taoJointUser::taoJointUser() : taoJointDOF1(TAO_AXIS_USER)
{ 
	setABJoint(new taoABJointDOF1(this));
	setType(TAO_JOINT_USER);
}
