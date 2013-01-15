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

#include "taoWorld.h"
#include "taoGroup.h"

taoWorld::~taoWorld()
{
	taoGroup* g;
	while (_groupList)
	{	
		g = _groupList;
		_groupList = _groupList->getNext();
		delete g;
	}
}

taoGroup* taoWorld::removeGroup(const deInt id)
{
	taoGroup* n, *g = _groupList;
	if (g && g->getID() == id)
	{
		_groupList = g->getNext();
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

void taoWorld::addGroup(taoGroup* g, const deInt id)
{
	g->setNext(_groupList);
	_groupList = g;

	g->setID(id);
}

taoGroup* taoWorld::findGroup(const deInt id)
{
	taoGroup* g = _groupList;
	while (g)
	{
		if (g->getID() == id)
			return g;
		g = g->getNext();
	}
	return NULL;
}

void taoWorld::update(const deFloat time, const deFloat dt, const deInt n)
{
	taoGroup* g = _groupList;
	while (g)
	{	
		if (!g->getIsFixed())
			g->update(time, dt, n);
		g = g->getNext();
	}
}

void taoWorld::control(const deFloat time)
{
	taoGroup* g = _groupList;
	while (g)
	{	
		if (!g->getIsFixed())
			g->control(time);
		g = g->getNext();
	}
}

void taoWorld::simulate(const deFloat dt)
{
	taoGroup* g = _groupList;
	while (g)
	{	
		if (!g->getIsFixed())
			g->simulate(dt);
		g = g->getNext();
	}
}

void taoWorld::updateTransformation()
{
	taoGroup* g = _groupList;
	while (g)
	{	
		if (!g->getIsFixed())
			g->updateTransformation();
		g = g->getNext();
	}
}
