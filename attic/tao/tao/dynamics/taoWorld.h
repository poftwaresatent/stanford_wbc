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

#ifndef _taoWorld_h
#define _taoWorld_h

#include "taoTypes.h"

class taoGroup;

/*!
 *	\brief container class to hold dynamics groups.
 *	\ingroup taoDynamics
 *
 *	A world is a container object for dynamics groups that, in turn, are container 
 *	objects themselves holding dynamics characters, particle systems, 
 *	\remarks	There is only one world.
 */
class taoWorld
{
public:
	taoWorld() : _groupList(NULL) {}
	~taoWorld();

	taoGroup* getGroupList() { return _groupList; }

	taoGroup* removeGroup(const deInt id);

	void addGroup(taoGroup* g, const deInt id);

	taoGroup* findGroup(const deInt id);

	/*!
	 *	\remarks	this can be replaced by following 3 individual call.
	 *	\remarks	control(), simulate(), updateTransformation()
	 *
	 *	\arg	time	control desired goal achieving time. this value 
	 *					is used to compute the goal frames.
	 *					Also, this value should be greater than the last 
	 *					control time, taoControl::time() and less than equal 
	 *					to the current goal time set by taoControl::setGoalPosition().
	 *	\arg	dt		integration time step.  notice that this value is independent to \a time.
	 *	\arg	n		number of iteration of the loop if necessary
	 */
	void update(const deFloat time, const deFloat dt, const deInt n);

	void control(const deFloat time);
	void simulate(const deFloat dt);
	void updateTransformation();

private:
	taoGroup* _groupList;
};

#endif // _taoWorld_h
