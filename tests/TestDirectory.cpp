/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2009 Stanford University. All rights reserved.
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
   \file TestDirectory.cpp
   \author Roland Philippsen
   \note Originally Copyright (c) 2009 Roland Philippsen, released under a BSD license.
*/

#include "TestDirectory.hpp"

using namespace wbcnet;

namespace wbc {
  
  srv_result_t TestDirectory::
  ListBehaviors(listing_t & behaviors) const
  {
    behaviors.clear();
    behaviors.push_back("FloatBehavior");
    behaviors.push_back("PostureBehavior");
    behaviors.push_back("EndEffectorBehavior");
    return SRV_SUCCESS;
  }
  
  
  srv_result_t TestDirectory::
  ListBehaviorCmds(int behaviorID, command_list_t & commands) const
  {
    commands.clear();
    
    // what everyone understands:
    commands.push_back(SRV_GET_DIMENSION);
    
    if (0 == behaviorID) {
      // FloatBehavior
      // no further commands...
      return SRV_SUCCESS;
    }
    
    // suppose every other understands every thing
    commands.push_back(SRV_SET_GOAL);
    commands.push_back(SRV_GET_GOAL);
    commands.push_back(SRV_GET_POSITIONS);
    
    if (1 == behaviorID) {
      // PostureBehavior
      return SRV_SUCCESS;
    }
    
    if (2 == behaviorID) {
      // EndEffectorBehavior
      return SRV_SUCCESS;
    }
    
    return SRV_INVALID_BEHAVIOR_ID;
  }
  
  
  srv_result_t TestDirectory::
  HandleServoCmd(int commandID,
		 srv_code_t const * code_in,
		 srv_matrix_t const * data_in,
		 listing_t const & str_in,
		 srv_code_t * code_out,
		 srv_matrix_t * data_out,
		 listing_t & str_out)
  {
    return SRV_NOT_IMPLEMENTED;
  }
  
  
  srv_result_t TestDirectory::
  HandleBehaviorCmd(int behaviorID,
		    int commandID,
		    srv_code_t const * code_in,
		    srv_matrix_t const * data_in,
		    srv_code_t * code_out,
		    srv_matrix_t * data_out)
  {
    if ((0 > behaviorID) || (2 < behaviorID))
      return SRV_INVALID_BEHAVIOR_ID;
    
    if (0 == behaviorID) {
      // FloatBehavior
      if (SRV_GET_DIMENSION != commandID)
	return SRV_INVALID_COMMAND;
      if (code_out && code_out->SetNElements(1))
	(*code_out)[0] = 7;
      return SRV_SUCCESS;
    }
    
    int dim;
    if (1 == behaviorID) // PostureBehavior
      dim = 7;
    else // EndEffectorBehavior
      dim = 3;
    
    if (SRV_GET_DIMENSION == commandID) {
      if (code_out && code_out->SetNElements(1))
	(*code_out)[0] = dim;
      return SRV_SUCCESS;
    }
    
    if (SRV_SET_GOAL == commandID) {
      if (( ! data_in) || (data_in->NRows() != dim) || (data_in->NColumns() != 1))
	return SRV_INVALID_DATA;
      // could also check e.g. workspace limits and return SRV_OUT_OF_RANGE or whatever
      return SRV_SUCCESS;
    }
    
    if (SRV_GET_GOAL == commandID) {
      if (( ! data_out) || ( ! data_out->SetSize(dim, 1)))
	return SRV_INVALID_DATA;
      for (int ii(0); ii < dim; ++ii)
	data_out->GetElement(ii, 0) = ii;
      return SRV_SUCCESS;
    }
    
    if (SRV_GET_POSITIONS == commandID) {
      if (( ! data_out) || ( ! data_out->SetSize(dim, 1)))
	return SRV_INVALID_DATA;
      for (int ii(0); ii < dim; ++ii)
	data_out->GetElement(ii, 0) = ii - 0.1;
      return SRV_SUCCESS;
    }
    
    return SRV_INVALID_COMMAND;
  }
  
  
  srv_result_t TestDirectory::
  ListTasks(int behaviorID, listing_t & tasks) const
  {
    if (0 == behaviorID) {
      // FloatBehavior
      tasks.push_back("GravityCompensationTask");
      return SRV_SUCCESS;
    }
    
    if (1 == behaviorID) {
      // PostureBehavior
      tasks.push_back("PostureTask");
      tasks.push_back("FrictionTask");
      return SRV_SUCCESS;
    }
    
    if (2 == behaviorID) {
      // EndEffectorBehavior
      tasks.push_back("JointLimitTask");
      tasks.push_back("ObstacleAvoidanceTask");
      tasks.push_back("EndEffectorPositionTask");
      tasks.push_back("PostureTask");
      return SRV_SUCCESS;
    }
    
    return SRV_INVALID_BEHAVIOR_ID;
  }
  
  
  srv_result_t TestDirectory::
  ListTaskCmds(int behaviorID, int taskID, command_list_t & commands) const
  {
    commands.clear();
    commands.push_back(SRV_GET_TASK_TYPE);
    commands.push_back(SRV_GET_DIMENSION);
    
    if (0 == behaviorID) {
      // FloatBehavior
      if (0 == taskID) {
	// GravityCompensationTask
	// ... but nothing more to add
	return SRV_SUCCESS;
      }
      return SRV_INVALID_TASK_ID;
    }
    
    if (1 == behaviorID) {
      // PostureBehavior
      if (0 == taskID) {
	// PostureTask
	commands.push_back(SRV_SET_PROP_GAIN);
	commands.push_back(SRV_GET_PROP_GAIN);
	commands.push_back(SRV_SET_DIFF_GAIN);
	commands.push_back(SRV_GET_DIFF_GAIN);
	commands.push_back(SRV_SET_MAX_VEL);
	commands.push_back(SRV_GET_MAX_VEL);
	commands.push_back(SRV_SET_MAX_ACCEL);
	commands.push_back(SRV_GET_MAX_ACCEL);
	return SRV_SUCCESS;
      }
      if (1 == taskID) {
	// FrictionTask
	commands.push_back(SRV_SET_DIFF_GAIN);
	commands.push_back(SRV_GET_DIFF_GAIN);
	return SRV_SUCCESS;
      }
      return SRV_INVALID_TASK_ID;
    }
    
    if (2 == behaviorID) {
      // EndEffectorBehavior
      if (0 == taskID) {
	// JointLimitTask
	return SRV_SUCCESS;
      }
      if (1 == taskID) {
	// ObstacleAvoidanceTask
	commands.push_back(SRV_SET_PROP_GAIN);
	commands.push_back(SRV_GET_PROP_GAIN);
	commands.push_back(SRV_SET_DIFF_GAIN);
	commands.push_back(SRV_GET_DIFF_GAIN);
	commands.push_back(SRV_SET_MAX_VEL);
	commands.push_back(SRV_GET_MAX_VEL);
	commands.push_back(SRV_SET_MAX_ACCEL);
	commands.push_back(SRV_GET_MAX_ACCEL);
	return SRV_SUCCESS;
      }
      if (2 == taskID) {
	// EndEffectorPositionTask
	commands.push_back(SRV_SET_PROP_GAIN);
	commands.push_back(SRV_GET_PROP_GAIN);
	commands.push_back(SRV_SET_DIFF_GAIN);
	commands.push_back(SRV_GET_DIFF_GAIN);
	commands.push_back(SRV_SET_MAX_VEL);
	commands.push_back(SRV_GET_MAX_VEL);
	commands.push_back(SRV_SET_MAX_ACCEL);
	commands.push_back(SRV_GET_MAX_ACCEL);
	return SRV_SUCCESS;
      }
      if (3 == taskID) {
	// PostureTask
	commands.push_back(SRV_SET_PROP_GAIN);
	commands.push_back(SRV_GET_PROP_GAIN);
	commands.push_back(SRV_SET_DIFF_GAIN);
	commands.push_back(SRV_GET_DIFF_GAIN);
	commands.push_back(SRV_SET_MAX_VEL);
	commands.push_back(SRV_GET_MAX_VEL);
	commands.push_back(SRV_SET_MAX_ACCEL);
	commands.push_back(SRV_GET_MAX_ACCEL);
	return SRV_SUCCESS;
      }
      return SRV_INVALID_TASK_ID;
    }
    
    return SRV_INVALID_BEHAVIOR_ID;
  }
  
  
  srv_result_t TestDirectory::
  HandleTaskCmd(int behaviorID,
		int taskID,
		int commandID,
		srv_code_t const * code_in,
		srv_matrix_t const * data_in,
		srv_code_t * code_out,
		srv_matrix_t * data_out)
  {
    int dim;
    if (0 == behaviorID) {
      // FloatBehavior
      if (0 == taskID) {
	// GravityCompensationTask
	dim = 7;
      }
      else
	return SRV_INVALID_TASK_ID;
    }
    else if (1 == behaviorID) {
      // PostureBehavior
      if (0 == taskID) {
	// PostureTask
	dim = 7;
      }
      else if (1 == taskID) {
	// FrictionTask
	dim = 7;
      }
      else
	return SRV_INVALID_TASK_ID;
    }
    else if (2 == behaviorID) {
      // EndEffectorBehavior
      if (0 == taskID) {
	// JointLimitTask
	dim = 7;
      }
      else if (1 == taskID) {
	// ObstacleAvoidanceTask
	dim = 7;
      }
      else if (2 == taskID) {
	// EndEffectorPositionTask
	dim = 3;
      }
      else if (3 == taskID) {
	// PostureTask
	dim = 7;
      }
      else
	return SRV_INVALID_TASK_ID;
    }
    else
      return SRV_INVALID_BEHAVIOR_ID;
    
    if (SRV_GET_TASK_TYPE == commandID) {
      if (( ! code_out) || ( ! code_out->SetNElements(1)))
	return SRV_INVALID_DATA;
      (*code_out)[0] = 42;
      return SRV_SUCCESS;
    }
    
    if (SRV_GET_DIMENSION == commandID) {
      if (( ! code_out) || ( ! code_out->SetNElements(1)))
	return SRV_INVALID_DATA;
      (*code_out)[0] = dim;
      return SRV_SUCCESS;
    }
    
    if (SRV_SET_PROP_GAIN == commandID) {
      // also permit 1x1 data
      if (( ! data_in)
	  || ((data_in->NRows() != dim) && (data_in->NRows() != 1))
	  || (data_in->NColumns() != 1))
	return SRV_INVALID_DATA;
      // could also check limits and return SRV_OUT_OF_RANGE or whatever
      return SRV_SUCCESS;
    }
    
    if (SRV_GET_PROP_GAIN == commandID) {
      if (( ! data_out) || ( ! data_out->SetSize(dim, 1)))
	return SRV_INVALID_DATA;
      for (int ii(0); ii < dim; ++ii)
	data_out->GetElement(ii, 0) = 200 + 3.0 * ii;
      return SRV_SUCCESS;
    }

    if (SRV_SET_DIFF_GAIN == commandID) {
      // also permit 1x1 data
      if (( ! data_in)
	  || ((data_in->NRows() != dim) && (data_in->NRows() != 1))
	  || (data_in->NColumns() != 1))
	return SRV_INVALID_DATA;
      // could also check limits and return SRV_OUT_OF_RANGE or whatever
      return SRV_SUCCESS;
    }
    
    if (SRV_GET_DIFF_GAIN == commandID) {
      if (( ! data_out) || ( ! data_out->SetSize(dim, 1)))
	return SRV_INVALID_DATA;
      for (int ii(0); ii < dim; ++ii)
	data_out->GetElement(ii, 0) = 50 - 2.0 * ii;
      return SRV_SUCCESS;
    }

    if (SRV_SET_MAX_VEL == commandID) {
      // also permit 1x1 data
      if (( ! data_in)
	  || ((data_in->NRows() != dim) && (data_in->NRows() != 1))
	  || (data_in->NColumns() != 1))
	return SRV_INVALID_DATA;
      // could also check limits and return SRV_OUT_OF_RANGE or whatever
      return SRV_SUCCESS;
    }
    
    if (SRV_GET_MAX_VEL == commandID) {
      if (( ! data_out) || ( ! data_out->SetSize(dim, 1)))
	return SRV_INVALID_DATA;
      for (int ii(0); ii < dim; ++ii)
	data_out->GetElement(ii, 0) = 5 + 0.7 * ii;
      return SRV_SUCCESS;
    }

    if (SRV_SET_MAX_ACCEL == commandID) {
      // also permit 1x1 data
      if (( ! data_in)
	  || ((data_in->NRows() != dim) && (data_in->NRows() != 1))
	  || (data_in->NColumns() != 1))
	return SRV_INVALID_DATA;
      // could also check limits and return SRV_OUT_OF_RANGE or whatever
      return SRV_SUCCESS;
    }
    
    if (SRV_GET_MAX_ACCEL == commandID) {
      if (( ! data_out) || ( ! data_out->SetSize(dim, 1)))
	return SRV_INVALID_DATA;
      for (int ii(0); ii < dim; ++ii)
	data_out->GetElement(ii, 0) = 17 - 0.42 * ii;
      return SRV_SUCCESS;
    }
    
    return SRV_INVALID_COMMAND;
  }
  
}
