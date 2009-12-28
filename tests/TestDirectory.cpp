/*
 * Copyright (c) 2009 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "TestDirectory.hpp"

namespace wbcrun {
  
  wbcnet::srv_result_t TestDirectory::
  ListBehaviors(listing_t & behaviors) const
  {
    behaviors.clear();
    behaviors.push_back("FloatBehavior");
    behaviors.push_back("PostureBehavior");
    behaviors.push_back("EndEffectorBehavior");
    return SRV_SUCCESS;
  }
  
  
  wbcnet::srv_result_t TestDirectory::
  ListBehaviorCmds(int behaviorID, request_list_t & requests) const
  {
    requests.clear();
    
    // what everyone understands:
    requests.push_back(SRV_GET_DIMENSION);
    
    if (0 == behaviorID) {
      // FloatBehavior
      // no further commands...
      return SRV_SUCCESS;
    }
    
    // suppose every other understands every thing
    requests.push_back(SRV_SET_GOAL);
    requests.push_back(SRV_GET_GOAL);
    requests.push_back(SRV_GET_ACTUAL);
    
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
  
  
  wbcnet::srv_result_t TestDirectory::
  HandleServoCmd(int requestID,
		 srv::vector_t const * code_in,
		 srv::matrix_t const * data_in,
		 srv::vector_t * code_out,
		 srv::matrix_t * data_out)
  {
    return SRV_NOT_IMPLEMENTED;
  }
  
  
  wbcnet::srv_result_t TestDirectory::
  HandleBehaviorCmd(int behaviorID,
		    int requestID,
		    srv::vector_t const * code_in,
		    srv::matrix_t const * data_in,
		    srv::vector_t * code_out,
		    srv::matrix_t * data_out)
  {
    if ((0 > behaviorID) || (2 < behaviorID))
      return SRV_INVALID_BEHAVIOR_ID;
    
    if (0 == behaviorID) {
      // FloatBehavior
      if (SRV_GET_DIMENSION != requestID)
	return srv::INVALID_REQUEST;
      if (code_out && code_out->SetNElements(1))
	(*code_out)[0] = 7;
      return SRV_SUCCESS;
    }
    
    int dim;
    if (1 == behaviorID) // PostureBehavior
      dim = 7;
    else // EndEffectorBehavior
      dim = 3;
    
    if (SRV_GET_DIMENSION == requestID) {
      if (code_out && code_out->SetNElements(1))
	(*code_out)[0] = dim;
      return SRV_SUCCESS;
    }
    
    if (SRV_SET_GOAL == requestID) {
      if (( ! data_in) || (data_in->NRows() != dim) || (data_in->NColumns() != 1))
	return SRV_INVALID_DATA;
      // could also check e.g. workspace limits and return SRV_OUT_OF_RANGE or whatever
      return SRV_SUCCESS;
    }
    
    if (SRV_GET_GOAL == requestID) {
      if (( ! data_out) || ( ! data_out->SetSize(dim, 1)))
	return SRV_INVALID_DATA;
      for (int ii(0); ii < dim; ++ii)
	data_out->GetElement(ii, 0) = ii;
      return SRV_SUCCESS;
    }
    
    if (SRV_GET_ACTUAL == requestID) {
      if (( ! data_out) || ( ! data_out->SetSize(dim, 1)))
	return SRV_INVALID_DATA;
      for (int ii(0); ii < dim; ++ii)
	data_out->GetElement(ii, 0) = ii - 0.1;
      return SRV_SUCCESS;
    }
    
    return srv::INVALID_REQUEST;
  }
  
  
  wbcnet::srv_result_t TestDirectory::
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
  
  
  wbcnet::srv_result_t TestDirectory::
  ListTaskCmds(int behaviorID, int taskID, request_list_t & requests) const
  {
    requests.clear();
    requests.push_back(SRV_GET_TASK_TYPE);
    requests.push_back(SRV_GET_DIMENSION);
    
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
	requests.push_back(SRV_SET_PROP_GAIN);
	requests.push_back(SRV_GET_PROP_GAIN);
	requests.push_back(SRV_SET_DIFF_GAIN);
	requests.push_back(SRV_GET_DIFF_GAIN);
	requests.push_back(SRV_SET_MAX_VEL);
	requests.push_back(SRV_GET_MAX_VEL);
	requests.push_back(SRV_SET_MAX_ACCEL);
	requests.push_back(SRV_GET_MAX_ACCEL);
	return SRV_SUCCESS;
      }
      if (1 == taskID) {
	// FrictionTask
	requests.push_back(SRV_SET_DIFF_GAIN);
	requests.push_back(SRV_GET_DIFF_GAIN);
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
	requests.push_back(SRV_SET_PROP_GAIN);
	requests.push_back(SRV_GET_PROP_GAIN);
	requests.push_back(SRV_SET_DIFF_GAIN);
	requests.push_back(SRV_GET_DIFF_GAIN);
	requests.push_back(SRV_SET_MAX_VEL);
	requests.push_back(SRV_GET_MAX_VEL);
	requests.push_back(SRV_SET_MAX_ACCEL);
	requests.push_back(SRV_GET_MAX_ACCEL);
	return SRV_SUCCESS;
      }
      if (2 == taskID) {
	// EndEffectorPositionTask
	requests.push_back(SRV_SET_PROP_GAIN);
	requests.push_back(SRV_GET_PROP_GAIN);
	requests.push_back(SRV_SET_DIFF_GAIN);
	requests.push_back(SRV_GET_DIFF_GAIN);
	requests.push_back(SRV_SET_MAX_VEL);
	requests.push_back(SRV_GET_MAX_VEL);
	requests.push_back(SRV_SET_MAX_ACCEL);
	requests.push_back(SRV_GET_MAX_ACCEL);
	return SRV_SUCCESS;
      }
      if (3 == taskID) {
	// PostureTask
	requests.push_back(SRV_SET_PROP_GAIN);
	requests.push_back(SRV_GET_PROP_GAIN);
	requests.push_back(SRV_SET_DIFF_GAIN);
	requests.push_back(SRV_GET_DIFF_GAIN);
	requests.push_back(SRV_SET_MAX_VEL);
	requests.push_back(SRV_GET_MAX_VEL);
	requests.push_back(SRV_SET_MAX_ACCEL);
	requests.push_back(SRV_GET_MAX_ACCEL);
	return SRV_SUCCESS;
      }
      return SRV_INVALID_TASK_ID;
    }
    
    return SRV_INVALID_BEHAVIOR_ID;
  }
  
  
  wbcnet::srv_result_t TestDirectory::
  HandleTaskCmd(int behaviorID,
		int taskID,
		int requestID,
		srv::vector_t const * code_in,
		srv::matrix_t const * data_in,
		srv::vector_t * code_out,
		srv::matrix_t * data_out)
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
    
    if (SRV_GET_TASK_TYPE == requestID) {
      if (( ! code_out) || ( ! code_out->SetNElements(1)))
	return SRV_INVALID_DATA;
      (*code_out)[0] = 42;
      return SRV_SUCCESS;
    }
    
    if (SRV_GET_DIMENSION == requestID) {
      if (( ! code_out) || ( ! code_out->SetNElements(1)))
	return SRV_INVALID_DATA;
      (*code_out)[0] = dim;
      return SRV_SUCCESS;
    }
    
    if (SRV_SET_PROP_GAIN == requestID) {
      // also permit 1x1 data
      if (( ! data_in)
	  || ((data_in->NRows() != dim) && (data_in->NRows() != 1))
	  || (data_in->NColumns() != 1))
	return SRV_INVALID_DATA;
      // could also check limits and return SRV_OUT_OF_RANGE or whatever
      return SRV_SUCCESS;
    }
    
    if (SRV_GET_PROP_GAIN == requestID) {
      if (( ! data_out) || ( ! data_out->SetSize(dim, 1)))
	return SRV_INVALID_DATA;
      for (int ii(0); ii < dim; ++ii)
	data_out->GetElement(ii, 0) = 200 + 3.0 * ii;
      return SRV_SUCCESS;
    }

    if (SRV_SET_DIFF_GAIN == requestID) {
      // also permit 1x1 data
      if (( ! data_in)
	  || ((data_in->NRows() != dim) && (data_in->NRows() != 1))
	  || (data_in->NColumns() != 1))
	return SRV_INVALID_DATA;
      // could also check limits and return SRV_OUT_OF_RANGE or whatever
      return SRV_SUCCESS;
    }
    
    if (SRV_GET_DIFF_GAIN == requestID) {
      if (( ! data_out) || ( ! data_out->SetSize(dim, 1)))
	return SRV_INVALID_DATA;
      for (int ii(0); ii < dim; ++ii)
	data_out->GetElement(ii, 0) = 50 - 2.0 * ii;
      return SRV_SUCCESS;
    }

    if (SRV_SET_MAX_VEL == requestID) {
      // also permit 1x1 data
      if (( ! data_in)
	  || ((data_in->NRows() != dim) && (data_in->NRows() != 1))
	  || (data_in->NColumns() != 1))
	return SRV_INVALID_DATA;
      // could also check limits and return SRV_OUT_OF_RANGE or whatever
      return SRV_SUCCESS;
    }
    
    if (SRV_GET_MAX_VEL == requestID) {
      if (( ! data_out) || ( ! data_out->SetSize(dim, 1)))
	return SRV_INVALID_DATA;
      for (int ii(0); ii < dim; ++ii)
	data_out->GetElement(ii, 0) = 5 + 0.7 * ii;
      return SRV_SUCCESS;
    }

    if (SRV_SET_MAX_ACCEL == requestID) {
      // also permit 1x1 data
      if (( ! data_in)
	  || ((data_in->NRows() != dim) && (data_in->NRows() != 1))
	  || (data_in->NColumns() != 1))
	return SRV_INVALID_DATA;
      // could also check limits and return SRV_OUT_OF_RANGE or whatever
      return SRV_SUCCESS;
    }
    
    if (SRV_GET_MAX_ACCEL == requestID) {
      if (( ! data_out) || ( ! data_out->SetSize(dim, 1)))
	return SRV_INVALID_DATA;
      for (int ii(0); ii < dim; ++ii)
	data_out->GetElement(ii, 0) = 17 - 0.42 * ii;
      return SRV_SUCCESS;
    }
    
    return srv::INVALID_REQUEST;
  }
  
}
