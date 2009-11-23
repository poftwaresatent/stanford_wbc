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

#ifndef _taoGroup_h
#define _taoGroup_h

#include "taoTypes.h"
#include <tao/matrix/TaoDeMath.h>

class taoDNode;
class taoNode;
class taoNodeRoot;

/*!
 *	\brief container class to hold dynamics characters
 *	\ingroup taoDynamics
 *
 *	A group is a container object for dynamics characters, particle systems, 
 *	rigid bodies, and articulated bodies. Characters in different groups can 
 *	not interact, e.g., no collision between characters from two different groups. 
 *	All characters in a group share common parameters such as integration time step, gravity, etc.
 */
class taoGroup
{
public:
	taoGroup() : _id(-1), _isFixed(0), _rootList(NULL), _next(NULL) { _gravity.zero(); }
	~taoGroup();

	void setID(deInt i) { _id = i; }
	const deInt getID() const { return _id; }

	void setIsFixed(int f) { _isFixed = f; }
	deInt getIsFixed() { return _isFixed; }

	deVector3* gravity() { return &_gravity; }

	void setNext(taoGroup* g) { _next = g; }
	taoGroup* getNext() { return _next; }

	taoNodeRoot* getRootList() { return _rootList; }
	void addRoot(taoNodeRoot* r, const deInt id);
	taoNodeRoot* removeRoot(const deInt id);
	taoNodeRoot* findRoot(const deInt id);

	void update(const deFloat time, const deFloat dt, const deInt n);
	void control(const deFloat time);
	void simulate(const deFloat dt);
	void updateTransformation();

	taoNodeRoot* unlinkFixed(taoNodeRoot* root, taoNode* node);
	taoNodeRoot* unlinkFree(taoNodeRoot* root, taoNode* node, deFloat inertia, deFloat damping);

	void sync(taoNodeRoot* root, deFloat time);

private:
	deInt _id;
	deInt _isFixed;
	deVector3 _gravity;
	taoNodeRoot* _rootList;
	taoGroup* _next;
};

#endif // _taoGroup_h
