/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2009 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
 * \file TaskSet.cpp
 * \author Luis Sentis
 * \note Based on the implementation by Irena Pashchenko
 */

#include <wbc/core/TaskSet.hpp>
#include <assert.h>

namespace wbc {
 
  TaskSet::TaskSet()
  : numTasks_( 0 ) {}


  void TaskSet::swap( TaskSet& ts )
  {
    taskSet_.swap( ts.taskSet_ );
    numTasks_ = ts.numTasks_;
  }


  /** \note level can be number starting with 0, two special negative
      numbers are reserved: APPEND_LASTLEVEL - appends into the last
      priority set APPEND - appends as a new priority level at the
      end. */
  void TaskSet::addTask( TaskDescription* pTask, int level )
  {
    if( !pTask )
      return;

    if( level < APPEND_LASTLEVEL )
      {
	assert( false );
	return;
      }
  
    if( level == APPEND || level >= numLevels() )
      { 
	taskSet_.push_back( TaskList() );
	taskSet_.back().push_back( TaskItem( pTask ) );
      }
    else if( level == APPEND_LASTLEVEL )
      { 
	taskSet_.back().push_back( TaskItem( pTask ) );
      }
    else
      {
	TaskList2D::iterator iSet; // insert before this set
	int iCounter = 1;
	for( iSet = taskSet_.begin(); iSet != taskSet_.end(); iSet++, iCounter++ )
	  if( iCounter > level )
	    break;

	iSet--;
	if( iSet != taskSet_.end() )
	  (*iSet).push_back( TaskItem( pTask ) );
	else
	  {
	    taskSet_.push_front( TaskList() );
	    taskSet_.front().push_back( TaskItem( pTask ) );
	  }
      }

    ++numTasks_;
  }


  bool TaskSet::removeTask( TaskDescription* pTask )
  {
    if( !pTask )
      return false;

    TaskList2D::iterator iList;
    TaskList::iterator iListItem;
    if( !findTask( pTask, iList, iListItem ) )
      return false;

    (*iList).erase( iListItem );
    taskSet_.erase(iList);

    --numTasks_;
    return true;
  }


  void TaskSet::removeEmptyLevels()
  {
    TaskList2D::iterator iList;
    for( iList = taskSet_.begin(); iList != taskSet_.end(); )
      {
	TaskList2D::iterator iNext = iList; 
	iNext++;
    
	if( (*iList).empty() )
	  taskSet_.erase( iList );
    
	iList = iNext;
      }
  }


  void TaskSet::removeAll()
  {
    numTasks_ = 0;
  
    TaskList2D::iterator iList;
    TaskList::iterator iListItem;

    taskSet_.clear();
  }


  void TaskSet::moveTask( TaskDescription* pTask, int iNumOfLevels )
  {
    TaskList2D::iterator iList;
    TaskList::iterator iListItem;
    if( iNumOfLevels == 0 )
      return;

    if( !findTask( pTask, iList, iListItem ) )
      {
	assert( false );
	return;
      }

    TaskItem piTask( pTask ); // store the item, as it is ref-counted
    (*iList).erase( iListItem );

    int iCounter = 0;
    while( iCounter != iNumOfLevels || iList != taskSet_.end() )
      {
	if( iNumOfLevels > 0 )
	  {
	    ++iCounter;
	    iList++;
	  }
	else
	  {
	    --iCounter;
	    iList--;
	  }
      }

    if( iList != taskSet_.end() )
      (*iList).push_back( piTask );
    else
      {
	if( iNumOfLevels > 0 )
	  {
	    taskSet_.push_back( TaskList() );
	    taskSet_.back().push_back( piTask );
	  }
	else
	  {
	    taskSet_.push_front( TaskList() );
	    taskSet_.front().push_back( piTask );
	  }
      }
  }


  /** \note Modified by Luis Sentis. */
  bool TaskSet::findTask( TaskDescription* pTask, 
                          TaskList2D::iterator& ioList, 
                          TaskList::iterator& ioListItem )
  {
    TaskList2D::iterator iList;
    TaskList::iterator iListItem;
    bool fFound = false;

  
    for( iList = taskSet_.begin(); iList != taskSet_.end(); ++iList) {
      for( iListItem = (*iList).begin(); iListItem != (*iList).end(); ++iListItem ) {
	if( (*iListItem) == pTask ) { 
	  fFound = true;
	  ioList = iList; // Luis
	  ioListItem = iListItem; // Luis
	  break;
	}
      }
    }
  
    // Removed by Luis
    //ioList = iList;
    //ioListItem = iListItem;
    return fFound;
  }
  
  
  void TaskSet::
  appendServos(TaskList2D::const_iterator const & itMap,
	       SAIVector & acceleration)
  {
    acceleration.setSize(0);
    TaskList::const_iterator const iend(itMap->end());
    for (TaskList::const_iterator ii(itMap->begin()); ii != iend; ++ii) {
      if (TaskDescription::Contact_Task != (*ii)->taskType()) {
	acceleration.append((*ii)->commandAccel());
      }
    }
  }
  
}
