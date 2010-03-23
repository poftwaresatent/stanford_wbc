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
 * \file TaskSet.hpp
 * \author Luis Sentis
 * \note Based on the implementation by Irena Pashchenko
 */

#ifndef WBC_TASK_SET_HPP
#define WBC_TASK_SET_HPP

#include <wbc/core/TaskDescription.hpp>
#include <list>
#include <string>

namespace wbc {

  /**
     This class encompasses a set of tasks that are either in execution
     or part of an inactive segment of the task sequence.  The set can
     be populated with tasks by specifying their prioritization, and
     priority of each task can be dynamically changed.
  */

  class TaskSet {
  public:

    TaskSet();
    ~TaskSet() {}

    typedef TaskDescription* TaskItem;
    typedef std::list< TaskItem > TaskList;
    typedef std::list< TaskList> TaskList2D;

    enum { APPEND_LASTLEVEL=-2, APPEND=-1 };

    void addTask( TaskDescription* pTask, int level = APPEND );

    void swap( TaskSet& ts );

    bool removeTask( TaskDescription* task );

    void removeEmptyLevels();

    void removeAll();

    /** moves task from its present location by specified
	positive or negative number of levels. */
    void moveTask( TaskDescription* pTask, int iNumOfLevels );

    int numLevels() const { return (int) taskSet_.size(); }

    int numTasks() const { return numTasks_; }

    TaskList2D::iterator       begin()       { return taskSet_.begin(); }
    TaskList2D::const_iterator begin() const { return taskSet_.begin(); }
    TaskList2D::iterator       end()         { return taskSet_.end(); }
    TaskList2D::const_iterator end()   const { return taskSet_.end(); }
    
    static void appendServos(/** in  */ TaskList2D::const_iterator const & itMap,
			     /** out */ SAIVector & acceleration);
    
  private:

    TaskList2D taskSet_;
    int numTasks_;
  
    bool findTask( TaskDescription* ptask, TaskList2D::iterator& iList,
		   TaskList::iterator& iListItem );
    
    // do not allow copy constructor & assignment
    TaskSet(const TaskSet&);
    TaskSet& operator=(const TaskSet&);
  };

}

#endif
