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

#ifndef _taoDNode_h
#define _taoDNode_h

#include "taoTypes.h"
#include "taoCNode.h"

class taoABNode;
class taoJoint;
class deVector3;
class deVector6;
class deFrame;
class deMatrix3;

/*!
 *	\brief abstract node class for articulated body
 *	\ingroup taoDynamics
 *
 *	This class should be used as a base class and implemented accordingly.	
 */
class taoDNode : public taoCNode
{
public:
	taoDNode() : _abNode(NULL), _propagate(1) {}
	virtual ~taoDNode() {}
	virtual void sync(deFrame* local) = 0;
	//!	indicates if this node is root
	/*!	\retval	1	this node is root
	 *	\retval	0	this node in not root
	 */
	virtual deInt isRoot() { return (getDParent() == NULL); }
	//!	indicates if parent of this node is root
	/*!	\retval	1	parent this node is root
	 *	\retval	0	parent this node in not root
	 */
	virtual deInt isParentRoot() { return (isRoot() || getDParent()->isRoot()); }

	virtual taoJoint* getJointList() = 0;
	virtual taoJoint const * getJointList() const = 0;

	//!	gets _abNode
	virtual taoABNode* getABNode() { return _abNode; }
	//!	sets _abNode
	virtual void setABNode(taoABNode* node) { _abNode = node; }

	//!	indicates if dynamics of node should be included for control
	/*!	\retval	1	to propagate dynamics of node for control 
	 *	\retval	0	not to propagate dynamics of node for control
	 */
	virtual deInt getPropagate() { return _propagate; }
	virtual void setPropagate(deInt p) { _propagate = p; } // YYY

	virtual deVector6* velocity() = 0;
	virtual deVector6* acceleration() = 0;

	//! \return	home frame
	virtual deFrame* frameHome() = 0;
	//! \return	local frame
	virtual deFrame* frameLocal() = 0;
	//! \return	global frame
	//virtual deFrame* frameGlobal() = 0;
	//! \return	mass
	virtual deFloat* mass() = 0;
	//! \return	center of gravity in local frame
	virtual deVector3* center() = 0;
	virtual deVector3 const * center() const = 0;
	//!	\return	inertia tensor at the origin of local frame
	virtual deMatrix3* inertia() = 0;
	//!	\retval	fext	accumulate all external forces
	virtual deVector6* force() = 0;
	virtual void zeroForce() = 0;
	virtual void addForce(const deVector6* f) = 0;

	virtual void updateFrame() = 0;
	virtual void integrate(deFloat dt) = 0;

	//!	\return	parent node
	virtual taoDNode* getDParent() = 0;
	virtual taoDNode const * getDParent() const = 0;
	virtual void setDChild(taoDNode* n) = 0;
	//!	\return	first child node
	virtual taoDNode* getDChild() = 0;
	virtual taoDNode const* getDChild() const = 0;
	//!	\return	next sibling node
	virtual taoDNode* getDSibling() = 0;
	virtual taoDNode const* getDSibling() const = 0;

private:
	taoABNode* _abNode;
	deInt _propagate;
};

#endif // _taoDNode_h
