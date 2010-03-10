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

#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoJoint.h>
#include "taoABNode.h"
#include <tao/dynamics/taoDynamics.h>

#ifdef TAO_CONTROL
#include "taoControl.h"
#endif

deVector6* taoNode::velocity()
{
	return getABNode()->V();
}

deVector6* taoNode::acceleration()
{
	return getABNode()->A();
}

taoNode::taoNode()
{
	_Initialize();
}

taoNode::taoNode(taoDNode* parent, deFrame const * home)
{
	_Initialize();
	link(parent, home);
}

taoNode::~taoNode()
{
	deleteJointABNode();
}

void taoNode::_Initialize()
{
	_frameHome.identity();
	_frameLocal.identity();
	_frameGlobal.identity();

	_mass = 0;
	_center.zero();
	_inertia.zero();

	_Fext.zero();

	_parent = NULL;
	_child = NULL;
	_sibling = NULL;

	_jointList = NULL;
}

void taoNode::sync(deFrame* local)
{
	_frameHome = *local;
	_frameLocal = _frameHome;
	_frameGlobal.multiply(*_parent->frameGlobal(), _frameLocal);

	taoJoint* j = _jointList;
	while (j)
	{	
		j->reset();
		j = j->getNext();
	}
}

void taoNode::deleteJointABNode()
{
	taoJoint* j;
	while (_jointList)
	{	
		j = _jointList;
		_jointList = _jointList->getNext();
		delete j;
	}
	delete getABNode();
}

void taoNode::updateFrame()
{
#if 0
	getABNode()->getFrameLocal(_frameLocal);
#else
	deFrame fl;
	_frameLocal = _frameHome;
	for (taoJoint* j = _jointList; j != NULL; j = j->getNext())
	{
		fl.identity();
		j->updateFrameLocal(&fl);
		_frameGlobal = _frameLocal;
		_frameLocal.multiply(_frameGlobal, fl);
	}
#endif
	_frameGlobal.multiply(*_parent->frameGlobal(), _frameLocal);
}

void taoNode::integrate(deFloat dt)
{
	taoJoint* j = _jointList;
	while (j)
	{	
		j->integrate(dt);
		j = j->getNext();
	}
}

void taoNode::addJoint(taoJoint* joint)
{
	taoJoint* j = _jointList;

	if (!j)
	{
		_jointList = joint;
	}
	else
	{
		while (j->getNext())
			j = j->getNext();
		j->setNext(joint);
	}
	joint->setNext(NULL);
}

void taoNode::addABNode()
{
	deInt i = 0;
	taoJoint* j = _jointList;
	while (j)
	{
		i++;
		j = j->getNext();
	}
	if (i > 1)
		setABNode(new taoABNodeNOJn);
	else
		setABNode(new taoABNodeNOJ1);

	getABNode()->setNOJ(i);

	i = 0;
	j = _jointList;
	while (j)
	{
		getABNode()->setABJoint(j->getABJoint(), i);

		i++;
		j = j->getNext();
	}
	if (i == 0)
		getABNode()->setABJoint(new taoABJointFixed);
}

void taoNode::link(taoDNode* parent, deFrame const * home)
{
	_parent = parent;
	_sibling = (taoNode*)_parent->getDChild();
	_parent->setDChild(this);
	_frameHome = *home;
	_frameLocal = _frameHome;
	_frameGlobal = _frameHome;
}

void taoNode::unlink()
{
	taoNode* c = (taoNode*)_parent->getDChild();
	if (c == this)
	{
		_parent->setDChild(_sibling);
	}
	else
	{
		while (c->_sibling != this)
			c = c->_sibling;
				
		c->_sibling = _sibling;
	}
	_sibling = NULL;
	_parent = NULL;
	deleteJointABNode();
}

void taoNode::linearVelocity(deVector3* Vie, const deVector3* Pie)
{
	Vie->crossMultiply(*Pie, (*getABNode()->V())[1]);
	Vie->subtract((*getABNode()->V())[0], *Vie);
}

void taoNode::linearAcceleration(deVector3* Aie, const deVector3* Pie)
{
	Aie->crossMultiply(*Pie, (*getABNode()->A())[1]);
	Aie->subtract((*getABNode()->A())[0], *Aie);
}

/* XXX
deFloat taoNode::effectiveMass(const deVector3* Pie, const deVector3* Ui)
{
	deFloat mi = taoDynamics::effectiveMassInv(this, Pie, Ui);
	if (mi == 0)
		return 0;
	return 1 / mi;
}
*/

void taoNode::impulse(const deVector3* Pie, const deVector3* Yie)
{
	taoDynamics::impulse(this, Pie, Yie);
}

void taoNode::impulseDist(const deVector3* Pie, const deVector3* Yie)
{
	taoDynamics::impulseDist(this, Pie, Yie);
}

void taoNode::force(const deVector3* Pie, const deVector3* Fie)
{
	taoDynamics::force(this, Pie, Fie);
}

deVector6* taoNodeRoot::velocity()
{
	return getABNode()->V();
}

deVector6* taoNodeRoot::acceleration()
{
	return getABNode()->A();
}

taoNodeRoot::taoNodeRoot(deFrame const & global)
{ 
	_zero = 0;
	_frameGlobal = global;

	_group = NULL;
	_controller = NULL;

	_child = NULL;

	_next = NULL;

	setABNode(new taoABNodeRoot);
}

taoNodeRoot::~taoNodeRoot()
{
#ifdef TAO_CONTROL
	delete getController();
#endif
	delete getABNode();
	if (_child)
		_DeleteNodeTree(_child);
}

void taoNodeRoot::_DeleteNodeTree(taoDNode* r) 
{ 
	taoDNode* c = r->getDChild();
	taoDNode* s = r->getDSibling();

	if (c)
		_DeleteNodeTree(c);
	if (s)
		_DeleteNodeTree(s);
	delete r;
}

void taoNodeRoot::sync()
{
	_SyncNodeTree(this);
}

void taoNodeRoot::_SyncNodeTree(taoDNode* r)
{
	r->sync(r->frameLocal());

	for (taoDNode* n = r->getDChild(); n != NULL; n = n->getDSibling())
		_SyncNodeTree(n);
}

