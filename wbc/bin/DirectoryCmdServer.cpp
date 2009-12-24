/*
 * Copyright (c) 2009 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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

#include "DirectoryCmdServer.hpp"
#include <wbc/core/BehaviorDescription.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/Kinematics.hpp>
#include <wbc/bin/ServoProcess.hpp>
#include <wbcnet/log.hpp>
#include <sstream>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));


namespace wbc {

  DirectoryCmdServer::
  DirectoryCmdServer(std::vector<BehaviorDescription*> const & behavior,
		     ServoProcessAPI * servo)
    : m_behavior(behavior),
      m_servo(servo)
  {
  }


  wbcnet::srv_result_t DirectoryCmdServer::
  ListBehaviors(wbcrun::listing_t & behaviors) const
  {
    for (size_t ii(0); ii < m_behavior.size(); ++ii)
      behaviors.push_back(m_behavior[ii]->name);
    return wbcnet::SRV_SUCCESS;
  }


  wbcnet::srv_result_t DirectoryCmdServer::
  ListBehaviorCmds(int behaviorID,
		   wbcrun::command_list_t & commands) const
  {
    if ((0 > behaviorID) || (static_cast<int>(m_behavior.size()) <= behaviorID))
      return wbcnet::SRV_INVALID_BEHAVIOR_ID;
    return wbcnet::SRV_SUCCESS;
  }


  wbcnet::srv_result_t DirectoryCmdServer::
  HandleServoCmd(int commandID,
		 wbcnet::msg::Service::vector_type const * code_in,
		 wbcnet::msg::Service::matrix_type const * data_in,
		 wbcnet::msg::Service::vector_type * code_out,
		 wbcnet::msg::Service::matrix_type * data_out)
  {
    if (logger->isDebugEnabled()) {
      ostringstream msg;
      msg << "DirectoryCmdServer::HandleServoCmd(" << commandID << ", ...)\n"
	  << "  code_in:\n";
      code_in->Display(msg, "    ");
      msg << "  data_in:\n";
      data_in->Display(msg, "    ");
      LOG_DEBUG (logger, msg.str());
    }
    
    if ( ! m_servo)
      return (wbcnet::srv_result_t) (wbcnet::SRV_OTHER_ERROR + 1);

    switch (commandID) {
      
    case wbcnet::SRV_SET_BEHAVIOR:
      if (code_in->NElements() < 1) {
	return wbcnet::SRV_MISSING_CODE;
      }
      return m_servo->BeginBehaviorTransition((*code_in)[0]);
      
    case wbcnet::SRV_GET_POSITIONS:
      {
	SAIVector const & jpos(m_servo->GetKinematics()->jointPositions());
	if ( ! data_out->SetSize(jpos.size(), 1))
	  return wbcnet::SRV_OTHER_ERROR;
	for (int ii(0); ii < jpos.size(); ++ii)
	  data_out->GetElement(ii, 0) = jpos[ii];
      }
      return wbcnet::SRV_SUCCESS;
      
    case wbcnet::SRV_GET_VELOCITIES:
      {
	SAIVector const & jvel(m_servo->GetKinematics()->jointVelocities());
	if ( ! data_out->SetSize(jvel.size(), 1))
	  return wbcnet::SRV_OTHER_ERROR;
	for (int ii(0); ii < jvel.size(); ++ii)
	  data_out->GetElement(ii, 0) = jvel[ii];
      }
      return wbcnet::SRV_SUCCESS;
      
    case wbcnet::SRV_GET_TORQUES:
      {
	SAIVector const & tau(m_servo->GetCommandTorques());
	if ( ! data_out->SetSize(tau.size(), 1))
	  return wbcnet::SRV_OTHER_ERROR;
	for (int ii(0); ii < tau.size(); ++ii)
	  data_out->GetElement(ii, 0) = tau[ii];
      }
      return wbcnet::SRV_SUCCESS;
      
    case wbcnet::SRV_GET_LINK_TRANSFORM:
      if (code_in->NElements() < 1)
 	return wbcnet::SRV_MISSING_CODE;
      {
    	taoDNode * node(m_servo->GetBranching()->node((*code_in)[0]));
    	if ( ! node)
    	  return wbcnet::SRV_INVALID_CODE;
    	SAIVector zero(3);
    	SAITransform const transform(m_servo->GetKinematics()->globalFrame(node, zero));
    	SAIVectorAPI reply(transform.rotation().vecForm());
    	reply.append(transform.translation());
    	data_out->Copy(reply);
      }
      return wbcnet::SRV_SUCCESS;
      
    }
    
    return wbcnet::SRV_NOT_IMPLEMENTED;
  }


  wbcnet::srv_result_t DirectoryCmdServer::
  HandleBehaviorCmd(int behaviorID,
		    int commandID,
		    wbcnet::msg::Service::vector_type const * code_in,
		    wbcnet::msg::Service::matrix_type const * data_in,
		    wbcnet::msg::Service::vector_type * code_out,
		    wbcnet::msg::Service::matrix_type * data_out)
  {
    if ((0 > behaviorID) || (static_cast<int>(m_behavior.size()) <= behaviorID))
      return wbcnet::SRV_INVALID_BEHAVIOR_ID;
    return wbcnet::SRV_NOT_IMPLEMENTED;
  }


  wbcnet::srv_result_t DirectoryCmdServer::
  ListTasks(int behaviorID,
	    wbcrun::listing_t & tasks) const
  {
    if ((0 > behaviorID) || (static_cast<int>(m_behavior.size()) <= behaviorID))
      return wbcnet::SRV_INVALID_BEHAVIOR_ID;
    BehaviorDescription::task_set_vector const & task_set(m_behavior[behaviorID]->allTaskSets());
    for (size_t ii(0); ii < task_set.size(); ++ii)
      for (TaskSet::TaskList2D::const_iterator jt(task_set[ii]->begin());
	   jt != task_set[ii]->end(); ++jt)
	for (TaskSet::TaskList::const_iterator kt(jt->begin());
	     kt != jt->end(); ++kt)
	  tasks.push_back((*kt)->name);
    return wbcnet::SRV_SUCCESS;
  }


  wbcnet::srv_result_t DirectoryCmdServer::
  ListTaskCmds(int behaviorID,
	       int taskID,
	       wbcrun::command_list_t & commands) const
  {
    if ((0 > behaviorID) || (static_cast<int>(m_behavior.size()) <= behaviorID))
      return wbcnet::SRV_INVALID_BEHAVIOR_ID;
    BehaviorDescription::task_set_vector const & task_set(m_behavior[behaviorID]->allTaskSets());
    int count(0);
    TaskDescription * task(0);
    for (size_t ii(0); ii < task_set.size(); ++ii)
      for (TaskSet::TaskList2D::const_iterator jt(task_set[ii]->begin());
	   jt != task_set[ii]->end(); ++jt)
	for (TaskSet::TaskList::const_iterator kt(jt->begin());
	     kt != jt->end(); ++kt) {
	  if (count == taskID) {
	    task = *kt;
	    break;
	  }
	  ++count;
	}
    if ( ! task)
      return wbcnet::SRV_INVALID_TASK_ID;
    commands.push_back(wbcnet::SRV_GET_TASK_TYPE);
    commands.push_back(wbcnet::SRV_GET_DIMENSION);
    commands.push_back(wbcnet::SRV_SET_GOAL);
    commands.push_back(wbcnet::SRV_GET_GOAL);
    commands.push_back(wbcnet::SRV_GET_POSITIONS);
    commands.push_back(wbcnet::SRV_GET_VELOCITIES);
    commands.push_back(wbcnet::SRV_SET_PROP_GAIN);
    commands.push_back(wbcnet::SRV_GET_PROP_GAIN);
    commands.push_back(wbcnet::SRV_SET_DIFF_GAIN);
    commands.push_back(wbcnet::SRV_GET_DIFF_GAIN);
    commands.push_back(wbcnet::SRV_SET_MAX_VEL);
    commands.push_back(wbcnet::SRV_GET_MAX_VEL);
    commands.push_back(wbcnet::SRV_SET_MAX_ACCEL);
    commands.push_back(wbcnet::SRV_GET_MAX_ACCEL);
    return wbcnet::SRV_SUCCESS;
  }


  wbcnet::srv_result_t DirectoryCmdServer::
  HandleTaskCmd(int behaviorID,
		int taskID,
		int commandID,
		wbcnet::msg::Service::vector_type const * code_in,
		wbcnet::msg::Service::matrix_type const * data_in,
		wbcnet::msg::Service::vector_type * code_out,
		wbcnet::msg::Service::matrix_type * data_out)
  {
    LOG_DEBUG (logger,
	       "DirectoryCmdServer::HandleTaskCmd(" << behaviorID << ", " << taskID << ", " << commandID << ", ...)");
  
    if ((0 > behaviorID) || (static_cast<int>(m_behavior.size()) <= behaviorID))
      return wbcnet::SRV_INVALID_BEHAVIOR_ID;

    LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): match behavior = " << m_behavior[behaviorID]->name);
  
    BehaviorDescription::task_set_vector const & task_set(m_behavior[behaviorID]->allTaskSets());
    int count(0);
    TaskDescription * task(0);
    for (size_t ii(0); ii < task_set.size(); ++ii)
      for (TaskSet::TaskList2D::const_iterator jt(task_set[ii]->begin());
	   jt != task_set[ii]->end(); ++jt)
	for (TaskSet::TaskList::const_iterator kt(jt->begin());
	     kt != jt->end(); ++kt) {
	  if (count == taskID) {
	    task = *kt;
	    break;
	  }
	  ++count;
	}
    if ( ! task)
      return wbcnet::SRV_INVALID_TASK_ID;
  
    LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): match task = " << task->name);
  
    switch (commandID) {
    
    case wbcnet::SRV_GET_TASK_TYPE:
      LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): GET_TASK_TYPE");
      if (code_out->SetNElements(1))
	(*code_out)[0] = static_cast<int32_t>(task->taskType());
      else
	return wbcnet::SRV_OTHER_ERROR;
      break;
    
    case wbcnet::SRV_GET_DIMENSION:
      LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): GET_DIMENSION NOT_IMPLEMENTED");
      // XXXX should unify TaskDescription state definition
      return wbcnet::SRV_NOT_IMPLEMENTED;
      break;
  
    case wbcnet::SRV_SET_GOAL:
      LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): SET_GOAL NOT_IMPLEMENTED");
      // XXXX should unify TaskDescription state definition
      return wbcnet::SRV_NOT_IMPLEMENTED;
      break;
  
    case wbcnet::SRV_GET_GOAL:
      LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): GET_GOAL NOT_IMPLEMENTED");
      // XXXX should unify TaskDescription state definition
      return wbcnet::SRV_NOT_IMPLEMENTED;
      break;
    
    case wbcnet::SRV_GET_POSITIONS:
      LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): GET_POSITIONS NOT_IMPLEMENTED");
      // XXXX should unify TaskDescription state definition
      return wbcnet::SRV_NOT_IMPLEMENTED;
      break;
    
    case wbcnet::SRV_GET_VELOCITIES:
      LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): GET_VELOCITIES NOT_IMPLEMENTED");
      // XXXX should unify TaskDescription state definition
      return wbcnet::SRV_NOT_IMPLEMENTED;
      break;
    
    case wbcnet::SRV_SET_PROP_GAIN:
      LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): SET_PROP_GAIN");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ((data_in->NRows() < 1) || (data_in->NColumns() < 1))
	return wbcnet::SRV_INVALID_DATA;
      task->propGain(data_in->GetElement(0));
      LOG_DEBUG (logger,  "  new prop gain: " << task->propGain());
      break;
    
    case wbcnet::SRV_GET_PROP_GAIN:
      LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): GET_PROP_GAIN");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ( ! data_out->SetSize(1, 1))
	return wbcnet::SRV_OTHER_ERROR;
      data_out->GetElement(0) = task->propGain();
      LOG_DEBUG (logger,  "  prop gain: " << data_out->GetElement(0));
      break;
    
    case wbcnet::SRV_SET_DIFF_GAIN:
      LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): SET_DIFF_GAIN");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ((data_in->NRows() < 1) || (data_in->NColumns() < 1))
	return wbcnet::SRV_INVALID_DATA;
      task->diffGain(data_in->GetElement(0));
      LOG_DEBUG (logger,  "  new diff gain: " << task->diffGain());
      break;
    
    case wbcnet::SRV_GET_DIFF_GAIN:
      LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): GET_DIFF_GAIN");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ( ! data_out->SetSize(1, 1))
	return wbcnet::SRV_OTHER_ERROR;
      data_out->GetElement(0) = task->diffGain();
      LOG_DEBUG (logger,  "  diff gain: " << data_out->GetElement(0));
      break;

    case wbcnet::SRV_SET_MAX_VEL:
      LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): SET_MAX_VEL");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ((data_in->NRows() < 1) || (data_in->NColumns() < 1))
	return wbcnet::SRV_INVALID_DATA;
      task->maxVel(data_in->GetElement(0));
      LOG_DEBUG (logger,  "  new max vel: " << task->maxVel());
      break;
    
    case wbcnet::SRV_GET_MAX_VEL:
      LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): GET_MAX_VEL");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ( ! data_out->SetSize(1, 1))
	return wbcnet::SRV_OTHER_ERROR;
      data_out->GetElement(0) = task->maxVel();
      LOG_DEBUG (logger,  "  max vel: " << data_out->GetElement(0));
      break;

    case wbcnet::SRV_SET_MAX_ACCEL:
      LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): SET_MAX_ACCEL");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ((data_in->NRows() < 1) || (data_in->NColumns() < 1))
	return wbcnet::SRV_INVALID_DATA;
      task->maxAccel(data_in->GetElement(0));
      LOG_DEBUG (logger,  "  new max accel: " << task->maxAccel());
      break;
    
    case wbcnet::SRV_GET_MAX_ACCEL:
      LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): GET_MAX_ACCEL");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ( ! data_out->SetSize(1, 1))
	return wbcnet::SRV_OTHER_ERROR;
      data_out->GetElement(0) = task->maxAccel();
      LOG_DEBUG (logger,  "  max accel: " << data_out->GetElement(0));
      break;
    
    default:
      LOG_DEBUG (logger,
		 "DirectoryCmdServer::HandleTaskCmd(): unhandled command\n"
		 << "  commandID = " << commandID
		 << " \"" << wbcnet::srv_command_to_string(commandID) << "\"");
      return wbcnet::SRV_INVALID_COMMAND;
    }
  
    LOG_DEBUG (logger,  "DirectoryCmdServer::HandleTaskCmd(): SUCCESS");
  
    return wbcnet::SRV_SUCCESS;
  }

}
