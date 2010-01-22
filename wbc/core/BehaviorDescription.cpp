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
 * \file       BehaviorDescription.cpp
 * \author     Luis Sentis and Roland Philippsen
 */

#include <wbc/core/BehaviorDescription.hpp>
#include <wbc/core/SAIMatrixAPI.hpp>
#include <wbcnet/msg/Service.hpp>
#include <wbc/util/RecorderImpl.hpp>
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));

namespace wbc {

  BehaviorDescription::
  BehaviorDescription(std::string const & _name)
    : name(_name), recorder_(0)
  {
  }


  BehaviorDescription::
  ~BehaviorDescription()
  {
    delete recorder_;
  }


  void BehaviorDescription::
  robotControlModel( RobotControlModel * robmodel )
    throw(std::runtime_error)
  {
    m_robModel = robmodel;
    loadMovementPrimitives(robmodel);
  }


  void BehaviorDescription::
  registerTaskSet(TaskSet * ts)
  {
    m_lookup[ts] = m_task_sets.size();
    m_task_sets.push_back(ts);
  }


  int BehaviorDescription::
  lookupTaskSetIndex(TaskSet const * taskSet) const
  {
    reverse_lookup_t::const_iterator foo(m_lookup.find(taskSet));
    if (m_lookup.end() == foo)
      return -1;
    return foo->second;  
  }
  
  
  int BehaviorDescription::
  handleCommand(int commandID,
		wbcnet::srv_code_t const * code_in,
		wbcnet::srv_matrix_t const * data_in,
		wbcnet::srv_code_t * code_out,
		wbcnet::srv_matrix_t * data_out)
  {
    switch (commandID) {
      
    case wbcnet::SRV_KEY_PRESS:
      if (code_in->NElements() < 1) {
	LOG_WARN (logger, "wbc::BehaviorDescription::handleCommand(): missing keycode in SRV_KEY_PRESS");
	return wbcnet::SRV_MISSING_CODE;
      }
      LOG_INFO (logger, "wbc::BehaviorDescription::handleCommand(): SRV_KEY_PRESS with keycode " << (*code_in)[0]);
      return handleKey((*code_in)[0]);
      
    case wbcnet::SRV_SET_GOAL:
      {
	// NOTE: legacy code had the goal stored as a row-vector,
	// which is why we do it "the transposed" way here.
	SAIVector goal(0);
	if (data_in->NRows() > 0) {
	  goal.setSize(data_in->NColumns());
	  for (int ii(0); ii < data_in->NColumns(); ++ii) {
	    goal[ii] = data_in->GetElement(0, ii);
	  }
	}
	LOG_INFO (logger,
		  "wbc::BehaviorDescription::handleCommand(): SRV_SET_GOAL with "
		  << goal.size() << " elements");
	return handleSetGoal(goal);
      }
      
    case wbcnet::SRV_GET_JACOBIAN:
      {
	LOG_INFO (logger, "wbc::BehaviorDescription::handleCommand(): SRV_GET_JACOBIAN");
	SAIMatrixAPI jacobian;
	int const result(handleGetJacobian(jacobian));
	if (wbcnet::SRV_SUCCESS != result) {
	  LOG_WARN (logger,
		    "wbc::BehaviorDescription::handleCommand(): handleGetJacobian() failed with "
		    << wbcnet::srv_result_to_string(result));
	  return result;
	}
	if ( ! data_out->Copy(jacobian)) {
	  LOG_ERROR (logger, "wbc::BehaviorDescription::handleCommand(): could not copy Jacobian to data_out");
	  return wbcnet::SRV_OTHER_ERROR;
	}
	return wbcnet::SRV_SUCCESS;
      }
    }
    
    LOG_WARN (logger,
	      "wbc::BehaviorDescription::handleCommand(): unrecognized commandID = "
	      << commandID << " = " << wbcnet::srv_command_to_string(commandID));
    
    return wbcnet::SRV_NOT_IMPLEMENTED;
  }
  
  
  int BehaviorDescription::
  handleKey(int keycode)
  {
    LOG_INFO (logger, "wbc::BehaviorDescription::handleKey(): ABSTRACT in BehaviorDescription");
    return wbcnet::SRV_ABSTRACT_METHOD;
  }
  
  
  int BehaviorDescription::
  handleSetGoal(SAIVector const & goal)
  {
    LOG_INFO (logger, "wbc::BehaviorDescription::handleSetGoal(): ABSTRACT in BehaviorDescription");
    return wbcnet::SRV_ABSTRACT_METHOD;
  }
  
  
  int BehaviorDescription::
  handleGetJacobian(SAIMatrix & jacobian)
  {
    LOG_INFO (logger, "wbc::BehaviorDescription::handleGetJacobian(): ABSTRACT in BehaviorDescription");
    return wbcnet::SRV_ABSTRACT_METHOD;
  }
  
  
  Recorder * BehaviorDescription::
  createRecorder(char const * header, char const * filename, Recorder::Mode mode)
  {
    if (recorder_)
      delete recorder_;
    recorder_ = new RecorderImpl(header, filename, mode);
    return recorder_;
  }


  Recorder * BehaviorDescription::
  getRecorder()
  {
    return recorder_;
  }
  
  
  bool BehaviorDescription::
  handleInit(std::string const & key, std::string const & value) throw(std::runtime_error)
  {
    LOG_DEBUG (logger, "wbc::BehaviorDescription::handleInit(" << key << ", " << value << ")");
    return false;
  }
  
}
