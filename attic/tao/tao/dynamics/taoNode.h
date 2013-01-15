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

#ifndef _taoNode_h
#define _taoNode_h

#include "taoDNode.h"
#include <tao/matrix/TaoDeMath.h>

class taoGroup;
class taoControl;

/*!
 *	\brief		node class for articulated body dynamics
 *	\ingroup	taoDynamics
 *
 *	This implements taoDNode for articulated body dynamics.
 *	\sa	taoCNode, taoDNode, taoDynamics
 */
class taoNode : public taoDNode
{
public:
	taoNode();
	taoNode(taoDNode* parent, deFrame const * home);
	virtual ~taoNode();

	virtual void sync(deFrame* local);

	virtual taoJoint* getJointList() { return _jointList; }
	virtual taoJoint const * getJointList() const { return _jointList; }

	/*!
	 *	\remarks	expressed in local frame
	 */
	virtual deVector6* velocity();
	/*!
	 *	\remarks	expressed in local frame
	 */
	virtual deVector6* acceleration();
	virtual void getFrameGraphics(deFrame* Tog) { *Tog = *frameGlobal(); }

	virtual deFrame* frameHome() { return &_frameHome; }
	virtual deFrame* frameLocal() { return &_frameLocal; }
	virtual deFrame* frameGlobal() { return &_frameGlobal; }
	virtual deFrame const * frameGlobal() const { return &_frameGlobal; }
	virtual deFloat* mass() { return &_mass; }
	virtual deVector3* center() { return &_center; }
	virtual deVector3 const * center() const { return &_center; }
	virtual deMatrix3* inertia() { return &_inertia; }
	/*!
	 *	\remarks	expressed in local frame
	 */
	virtual deVector6* force() { return &_Fext; }
	virtual void zeroForce() { _Fext.zero(); }
	/*!
	 *	\remarks	expressed in local frame
	 */
	virtual void addForce(const deVector6* f) { _Fext += *f; }

	virtual void updateFrame();
	virtual void integrate(deFloat dt);
	
	virtual void setDParent(taoDNode* n) { _parent = n; }
	virtual taoDNode* getDParent() { return _parent; }
	virtual taoDNode const * getDParent() const { return _parent; }
	virtual void setDChild(taoDNode* n) { _child = (taoNode*)n; }
	virtual taoDNode* getDChild() { return _child; }
	virtual taoDNode const* getDChild() const { return _child; }
	virtual void setDSibling(taoDNode* n) { _sibling = (taoNode*)n; }
	virtual taoDNode* getDSibling() { return _sibling; }
	virtual taoDNode const* getDSibling() const { return _sibling; }

	virtual void addJoint(taoJoint* joint);
	virtual void addABNode();

	virtual void unlink();
	virtual void link(taoDNode* parent, deFrame const * home);

	virtual void deleteJointABNode();

	/*!
	 *	\return		Vie
	 *	\remarks	expressed in local frame
	 */
	virtual void linearVelocity(deVector3* Vie, const deVector3* Pie);
	/*!
	 *	\return		Aie
	 *	\remarks	expressed in local frame
	 */
	virtual void linearAcceleration(deVector3* Aie, const deVector3* Pie);
	// XXX
	//virtual deFloat effectiveMass(const deVector3* Pie, const deVector3* Ui);
	virtual void impulse(const deVector3* Pie, const deVector3* Yie);
	virtual void impulseDist(const deVector3* Pie, const deVector3* Yie);
	virtual void force(const deVector3* Pie, const deVector3* Fie);

private:
	deFrame _frameHome;
	deFrame _frameLocal;
	deFrame _frameGlobal;

	deVector3 _center;
	deFloat _mass;
	deMatrix3 _inertia;

	deVector6 _Fext;

	taoDNode* _parent;
	taoNode* _child;
	taoNode* _sibling;

	taoJoint* _jointList;

	void _Initialize();
};

/*!
 *	\brief		root node class for articulated body dynamics
 *	\ingroup	taoDynamics
 *
 *	This implements taoDNode for root node of a articulated body tree structure.
 *	\sa	taoCNode, taoDNode, taoDynamics, taoControl
 */
class taoNodeRoot : public taoDNode
{
public:
	taoNodeRoot(deFrame const & global);

	virtual ~taoNodeRoot();

	virtual void sync(deFrame* local) { _frameGlobal = *local; } 

	virtual taoJoint* getJointList() { return NULL; }
	virtual taoJoint const * getJointList() const { return NULL; }

	virtual deVector6* velocity();
	virtual deVector6* acceleration();
	virtual void getFrameGraphics(deFrame* Tog) { *Tog = *frameGlobal(); }

	virtual deFrame* frameHome() { return &_frameGlobal; }
	virtual deFrame* frameLocal() { return &_frameGlobal; }
	virtual deFrame* frameGlobal() { return &_frameGlobal; }
	virtual deFrame const * frameGlobal() const { return &_frameGlobal; }
	virtual deFloat* mass() { return &_zero; } // YYY
	virtual deVector3* center() { return NULL; }
	virtual deVector3 const * center() const { return NULL; }
	virtual deMatrix3* inertia() { return NULL; }
	virtual deVector6* force() { return NULL; }
	virtual void zeroForce() {}
	virtual void addForce(const deVector6* f) {}

	virtual void updateFrame() {}
	virtual void integrate(deFloat dt) {}

	virtual taoDNode* getDParent() { return NULL; }
	virtual taoDNode const * getDParent() const { return NULL; }
	virtual void setDChild(taoDNode* n) { _child = (taoNode*)n; }
	virtual taoDNode* getDChild() { return _child; }
	virtual taoDNode const* getDChild() const { return _child; }
	virtual taoDNode* getDSibling() { return NULL; }
	virtual taoDNode const* getDSibling() const { return NULL; }

	void setNext(taoNodeRoot* r) { _next = r; }
	taoNodeRoot* getNext() { return _next; }

	void setGroup(taoGroup* g) { _group = g; }
	taoGroup* getGroup() { return _group; }

	void setController(taoControl* c) { _controller = c; }
	taoControl* getController() { return _controller; }

	/*!
	 *	\remarks	node->home = node->local;  joint var = zero;
	 */
	virtual void sync();

private:
	deFloat _zero; // YYY
	deFrame _frameGlobal;

	taoGroup* _group;
	taoControl* _controller;

	taoNode* _child;

	taoNodeRoot* _next;

	void _DeleteNodeTree(taoDNode* r);
	void _SyncNodeTree(taoDNode* r);
};

#endif // _taoNode_h
