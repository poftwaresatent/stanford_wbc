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
   \file TaskModelBase.hpp Utility for implementing serializable task models.
   \author Roland Philippsen
*/

#ifndef WBC_TASK_MODEL_BASE_HPP
#define WBC_TASK_MODEL_BASE_HPP

#include <wbcrun/TaskModelAPI.hpp>
#include <wbc/core/TaskSet.hpp>
#include <wbc/core/SAIMatrixAPI.hpp>

namespace wbc {
  
  class BehaviorDescription;
  class RobotControlModel;
  
  
  class TaskModelView
  {
  public:
    virtual ~TaskModelView() {}
    virtual SAIMatrix const * massInertia() const = 0;
    virtual SAIMatrix const * invMassInertia() const = 0;
    virtual SAIMatrix const * gravityForce() const = 0;
    virtual SAIMatrix const * coriolisCentrifugalForce() const = 0;
    virtual SAIMatrix const * LambdaStar(int taskID) const = 0;
    virtual SAIMatrix const * JStar(int taskID) const = 0;
  };
  
  
  /**
     Specialization of wbcrun::TaskModel for using SAIMatrix and
     providing some functionality useful for all specific task model
     implementations.
   */
  class TaskModelBase
    : public wbcrun::TaskModel<SAIMatrixAPI>
  {
  public:
    explicit TaskModelBase(wbcnet::endian_mode_t endian_mode);
    
    virtual TaskModelView const * GetView() const = 0;
    
    virtual bool Update(BehaviorDescription const & behavior,
			RobotControlModel const & robmodel) = 0;
    
    static void appendJacobians(SAIMatrix & Jlevel, TaskSet::TaskList2D::const_iterator ilevel);
    
    bool Reset(/** identifier of this transition request */
	       int requestID,
	       /** description of the behavior we're transiting to */
	       BehaviorDescription const & behavior);
    
    int ComputeTaskID(int taskset_idx, int level_idx) const;
    
    void SetAcquisitionTime(timeval const & tt);
    
    void prettyPrint (std::ostream & os, std::string const & title, std::string const & prefix) const;
    
  protected:
    typedef std::vector<int> level_offset_t;
    level_offset_t m_level_offset;
  };
  
}

#endif // WBC_TASK_MODEL_BASE_HPP
