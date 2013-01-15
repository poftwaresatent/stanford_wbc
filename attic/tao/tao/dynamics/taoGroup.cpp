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


#include "taoGroup.h"
#include <tao/dynamics/taoJoint.h>
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoDynamics.h>

#ifdef TAO_CONTROL
#include "taoControlJt.h"
#endif

taoGroup::~taoGroup()
{
	taoNodeRoot* r;
	while (_rootList)
	{	
		r = _rootList;
		_rootList = _rootList->getNext();
		delete r;
	}
}

void taoGroup::update(const deFloat time, const deFloat dt, const deInt n)
{
	deInt i = 0;
	while (i < n)
	{
		taoNodeRoot* r = _rootList;
		while (r)
		{
			if (!r->getIsFixed())
			{
#ifdef TAO_CONTROL
				if (r->getController())
					r->getController()->control(time);
#endif
				taoDynamics::fwdDynamics(r, &_gravity);
				taoDynamics::integrate(r, dt);
				taoDynamics::updateTransformation(r);
			}
			r = r->getNext();
		}
		i++;
	}
}

void taoGroup::control(const deFloat time)
{
	taoNodeRoot* r = _rootList;
	while (r)
	{
		if (!r->getIsFixed())
		{
#ifdef TAO_CONTROL
			if (r->getController())
				r->getController()->control(time);
#endif
		}
		r = r->getNext();
	}
}

void taoGroup::simulate(const deFloat dt)
{
	taoNodeRoot* r = _rootList;
	while (r)
	{
		if (!r->getIsFixed())
		{
			taoDynamics::fwdDynamics(r, &_gravity);
			taoDynamics::integrate(r, dt);
		}
		r = r->getNext();
	}
}

void taoGroup::updateTransformation()
{
	taoNodeRoot* r = _rootList;
	while (r)
	{
		if (!r->getIsFixed())
		{
			taoDynamics::updateTransformation(r);
		}
		r = r->getNext();
	}
}

void taoGroup::addRoot(taoNodeRoot* r, const deInt id)
{
	r->setNext(_rootList);
	_rootList = r;

	r->setID(id);
	r->setGroup(this);
}

taoNodeRoot* taoGroup::removeRoot(const deInt id)
{
	taoNodeRoot* n, *g = _rootList;
	if (g && g->getID() == id)
	{
		_rootList = g->getNext();
		g->setNext(NULL);
		return g;
	}
	else
	{
		while (g->getNext())
		{
			if (g->getNext()->getID() == id)
			{
				n = g->getNext();
				g->setNext(n->getNext());
				n->setNext(NULL);
				return n;
			}
			g = g->getNext();
		}
	}
	return NULL;
}

taoNodeRoot* taoGroup::findRoot(const deInt id)
{
	taoNodeRoot* r = _rootList;
	while (r)
	{
		if (r->getID() == id)
			return r;
		r = r->getNext();
	}
	return NULL;
}

taoNodeRoot* taoGroup::unlinkFixed(taoNodeRoot* root, taoNode* node)
{
#ifdef TAO_CONTROL
	if (root->getController())
		root->getController()->deleteParamTree(node);
#endif

	taoNodeRoot* r = new taoNodeRoot(*node->frameGlobal());

	node->unlink();

	if (!root->getDChild())
		delete removeRoot(root->getID());

	deFrame home;
	home.identity();

	node->link(r, &home);

	node->addABNode();

	taoDynamics::initialize(r);

	addRoot(r, -999);

	return r;
}

taoNodeRoot* taoGroup::unlinkFree(taoNodeRoot* root, taoNode* node, deFloat inertia, deFloat damping)
{
#ifdef TAO_CONTROL
	if (root->getController())
		root->getController()->deleteParamTree(node);
#endif

	deVector6 v = *node->velocity();
	taoNodeRoot* r = new taoNodeRoot(*node->frameGlobal());

	node->unlink();

	if (!root->getDChild())
		delete removeRoot(root->getID());

	deFrame home;
	home.identity();

	node->link(r, &home);

	taoJointDOF1* joint;

	joint = new taoJointPrismatic(TAO_AXIS_X);
	joint->setDamping(damping);
	joint->setInertia(inertia);
	joint->setDVar(new taoVarDOF1);
	joint->reset();
	joint->getVarDOF1()->_dQ = v[0][TAO_AXIS_X];
	node->addJoint(joint);

	joint = new taoJointPrismatic(TAO_AXIS_Y);
	joint->setDamping(damping);
	joint->setInertia(inertia);
	joint->setDVar(new taoVarDOF1);
	joint->reset();
	joint->getVarDOF1()->_dQ = v[0][TAO_AXIS_Y];
	node->addJoint(joint);

	joint = new taoJointPrismatic(TAO_AXIS_Z);
	joint->setDamping(damping);
	joint->setInertia(inertia);
	joint->setDVar(new taoVarDOF1);
	joint->reset();
	joint->getVarDOF1()->_dQ = v[0][TAO_AXIS_Z];
	node->addJoint(joint);

	taoJointSpherical* joint2 = new taoJointSpherical;
	joint2->setDamping(damping);
	joint2->setInertia(inertia);
	joint2->setDVar(new taoVarSpherical);
	joint2->reset();
	joint2->getVarSpherical()->_dQ = v[1];
	node->addJoint(joint2);

	node->addABNode();
	taoDynamics::initialize(r);

	addRoot(r, -999);

	return r;
}

void taoGroup::sync(taoNodeRoot* root, deFloat time)
{
#ifdef TAO_CONTROL
	if (root->getController())
		root->getController()->reset(time);
#endif

	root->sync();

	taoDynamics::initialize(root);
}
