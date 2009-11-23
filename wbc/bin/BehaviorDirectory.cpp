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

#include "BehaviorDirectory.hpp"
#include <wbc/core/BehaviorDescription.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/Kinematics.hpp>
#include <wbc/bin/ServoProcess.hpp>
#include <wbcnet/log.hpp>
#include <sstream>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));


namespace wbc {

  BehaviorDirectory::
  BehaviorDirectory(std::vector<BehaviorDescription*> const & behavior,
		    ServoImplementation * servo)
    : m_behavior(behavior),
      m_servo(servo)
  {
  }


  wbcrun::srv::result_t BehaviorDirectory::
  ListBehaviors(wbcrun::listing_t & behaviors) const
  {
    for (size_t ii(0); ii < m_behavior.size(); ++ii)
      behaviors.push_back(m_behavior[ii]->name);
    return wbcrun::srv::SUCCESS;
  }


  wbcrun::srv::result_t BehaviorDirectory::
  ListBehaviorCmds(int behaviorID,
		   wbcrun::request_list_t & requests) const
  {
    if ((0 > behaviorID) || (static_cast<int>(m_behavior.size()) <= behaviorID))
      return wbcrun::srv::INVALID_BEHAVIOR_ID;
    return wbcrun::srv::SUCCESS;
  }


  wbcrun::srv::result_t BehaviorDirectory::
  HandleServoCmd(int requestID,
		 wbcrun::ServiceMessage::vector_type const * code_in,
		 wbcrun::ServiceMessage::matrix_type const * data_in,
		 wbcrun::ServiceMessage::vector_type * code_out,
		 wbcrun::ServiceMessage::matrix_type * data_out)
  {
    if (logger->isDebugEnabled()) {
      ostringstream msg;
      msg << "BehaviorDirectory::HandleServoCmd(" << requestID << ", ...)\n"
	  << "  code_in:\n";
      code_in->Display(msg, "    ");
      msg << "  data_in:\n";
      data_in->Display(msg, "    ");
      LOG_DEBUG (logger, msg.str());
    }
  
    if ( ! m_servo)
      return wbcrun::srv::NOT_IMPLEMENTED;

//#warning 'XXXX to do: implement servo command handling (after the relevant msgs have been cleaned up)'
    // switch (requestID) {

    // case wbcrun::cmd::SET_BEHAVIOR:
    //   return wbcrun::srv::NOT_IMPLEMENTED;

    // // case wbcrun::cmd::GET_LINK_TRANSFORM:
    //   // if (code_in->NElements() < 1)
    //   // 	return wbcrun::srv::MISSING_CODE;
    //   // {
    //   // 	int const nodeID((*code_in)[0]);
    //   // 	taoDNode * node(m_servo->m_robmodel->branching()->node(nodeID));
    //   // 	if ( ! node)
    //   // 	  return wbcrun::srv::INVALID_CODE;
    //   // 	SAIVector zero(3);
    //   // 	SAITransform const
    //   // 	  transform(m_servo->m_kinematics->globalFrame(node, zero));
    //   // 	SAIVectorAPI reply(transform.rotation().vecForm());
    //   // 	reply.append(transform.translation());
    //   // 	code_out->SetNElements(0);
    //   // 	data_out->Copy(reply);
    //   // }
    //   // return wbcrun::srv::SUCCESS;
  
    
    // // case wbcrun::cmd::GET_LINK_ANGLE:
    //   // if (code_in->NElements() < 1)
    //   // 	return wbcrun::srv::MISSING_CODE;
    //   // {
    //   // 	int size = code_in->NElements() ;
    //   // 	SAIVectorAPI alpha(size);
    //   // 	for (int ii(0);ii<size;++ii){
    //   // 	  int const nodeID((*code_in)[ii]);
    //   // 	  LOG_DEBUG (logger, "BehaviorDirectory::HandleServoCmd()\n" << "  GET_LINK_ANGLE, nodeID = " << nodeID);
    //   // 	  if (nodeID >  m_servo->m_kinematics->jointPositions().size()){
    //   // 	    LOG_WARN (logger,
    //   // 		      "BehaviorDirectory::HandleServoCmd(): GET_LINK_ANGLE: only "
    //   // 		      << m_servo->m_kinematics->jointPositions().size() << " joints available");
    //   // 	    return wbcrun::srv::INVALID_CODE;
    //   // 	  }
    //   // 	  else
    //   // 	    alpha.elementAt(ii) = m_servo->m_kinematics->jointPositions()[nodeID];
    //   // 	}
    //   // 	code_out->SetNElements(0);
    //   // 	data_out->Copy(alpha);
    //   // }
    //   // return wbcrun::srv::SUCCESS;  
    // }
  
    return wbcrun::srv::NOT_IMPLEMENTED;
  }


  wbcrun::srv::result_t BehaviorDirectory::
  HandleBehaviorCmd(int behaviorID,
		    int requestID,
		    wbcrun::ServiceMessage::vector_type const * code_in,
		    wbcrun::ServiceMessage::matrix_type const * data_in,
		    wbcrun::ServiceMessage::vector_type * code_out,
		    wbcrun::ServiceMessage::matrix_type * data_out)
  {
    if ((0 > behaviorID) || (static_cast<int>(m_behavior.size()) <= behaviorID))
      return wbcrun::srv::INVALID_BEHAVIOR_ID;
    return wbcrun::srv::NOT_IMPLEMENTED;
  }


  wbcrun::srv::result_t BehaviorDirectory::
  ListTasks(int behaviorID,
	    wbcrun::listing_t & tasks) const
  {
    if ((0 > behaviorID) || (static_cast<int>(m_behavior.size()) <= behaviorID))
      return wbcrun::srv::INVALID_BEHAVIOR_ID;
    BehaviorDescription::task_set_vector const & task_set(m_behavior[behaviorID]->allTaskSets());
    for (size_t ii(0); ii < task_set.size(); ++ii)
      for (TaskSet::TaskList2D::const_iterator jt(task_set[ii]->begin());
	   jt != task_set[ii]->end(); ++jt)
	for (TaskSet::TaskList::const_iterator kt(jt->begin());
	     kt != jt->end(); ++kt)
	  tasks.push_back((*kt)->name);
    return wbcrun::srv::SUCCESS;
  }


  wbcrun::srv::result_t BehaviorDirectory::
  ListTaskCmds(int behaviorID,
	       int taskID,
	       wbcrun::request_list_t & requests) const
  {
    if ((0 > behaviorID) || (static_cast<int>(m_behavior.size()) <= behaviorID))
      return wbcrun::srv::INVALID_BEHAVIOR_ID;
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
      return wbcrun::srv::INVALID_TASK_ID;
    requests.push_back(wbcrun::cmd::GET_TASK_TYPE);
    requests.push_back(wbcrun::cmd::GET_DIMENSION);
    requests.push_back(wbcrun::cmd::SET_GOAL);
    requests.push_back(wbcrun::cmd::GET_GOAL);
    requests.push_back(wbcrun::cmd::GET_ACTUAL);
    requests.push_back(wbcrun::cmd::SET_PROP_GAIN);
    requests.push_back(wbcrun::cmd::GET_PROP_GAIN);
    requests.push_back(wbcrun::cmd::SET_DIFF_GAIN);
    requests.push_back(wbcrun::cmd::GET_DIFF_GAIN);
    requests.push_back(wbcrun::cmd::SET_MAX_VEL);
    requests.push_back(wbcrun::cmd::GET_MAX_VEL);
    requests.push_back(wbcrun::cmd::SET_MAX_ACCEL);
    requests.push_back(wbcrun::cmd::GET_MAX_ACCEL);
    return wbcrun::srv::SUCCESS;
  }


  wbcrun::srv::result_t BehaviorDirectory::
  HandleTaskCmd(int behaviorID,
		int taskID,
		int requestID,
		wbcrun::ServiceMessage::vector_type const * code_in,
		wbcrun::ServiceMessage::matrix_type const * data_in,
		wbcrun::ServiceMessage::vector_type * code_out,
		wbcrun::ServiceMessage::matrix_type * data_out)
  {
    LOG_DEBUG (logger,
	       "BehaviorDirectory::HandleTaskCmd(" << behaviorID << ", " << taskID << ", " << requestID << ", ...)");
  
    if ((0 > behaviorID) || (static_cast<int>(m_behavior.size()) <= behaviorID))
      return wbcrun::srv::INVALID_BEHAVIOR_ID;

    LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): match behavior = " << m_behavior[behaviorID]->name);
  
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
      return wbcrun::srv::INVALID_TASK_ID;
  
    LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): match task = " << task->name);
  
    switch (requestID) {
    
    case wbcrun::cmd::GET_TASK_TYPE:
      LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): GET_TASK_TYPE");
      if (code_out->SetNElements(1))
	(*code_out)[0] = static_cast<int32_t>(task->taskType());
      else
	return wbcrun::srv::OTHER_ERROR;
      break;
    
    case wbcrun::cmd::GET_DIMENSION:
      LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): GET_DIMENSION NOT_IMPLEMENTED");
      // XXXX should unify TaskDescription state definition
      return wbcrun::srv::NOT_IMPLEMENTED;
      break;
  
    case wbcrun::cmd::SET_GOAL:
      LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): SET_GOAL NOT_IMPLEMENTED");
      // XXXX should unify TaskDescription state definition
      return wbcrun::srv::NOT_IMPLEMENTED;
      break;
  
    case wbcrun::cmd::GET_GOAL:
      LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): GET_GOAL NOT_IMPLEMENTED");
      // XXXX should unify TaskDescription state definition
      return wbcrun::srv::NOT_IMPLEMENTED;
      break;
    
    case wbcrun::cmd::GET_ACTUAL:
      LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): GET_ACTUAL NOT_IMPLEMENTED");
      // XXXX should unify TaskDescription state definition
      return wbcrun::srv::NOT_IMPLEMENTED;
      break;
    
    case wbcrun::cmd::SET_PROP_GAIN:
      LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): SET_PROP_GAIN");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ((data_in->NRows() < 1) || (data_in->NColumns() < 1))
	return wbcrun::srv::INVALID_DATA;
      task->propGain(data_in->GetElement(0));
      LOG_DEBUG (logger,  "  new prop gain: " << task->propGain());
      break;
    
    case wbcrun::cmd::GET_PROP_GAIN:
      LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): GET_PROP_GAIN");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ( ! data_out->SetSize(1, 1))
	return wbcrun::srv::OTHER_ERROR;
      data_out->GetElement(0) = task->propGain();
      LOG_DEBUG (logger,  "  prop gain: " << data_out->GetElement(0));
      break;
    
    case wbcrun::cmd::SET_DIFF_GAIN:
      LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): SET_DIFF_GAIN");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ((data_in->NRows() < 1) || (data_in->NColumns() < 1))
	return wbcrun::srv::INVALID_DATA;
      task->diffGain(data_in->GetElement(0));
      LOG_DEBUG (logger,  "  new diff gain: " << task->diffGain());
      break;
    
    case wbcrun::cmd::GET_DIFF_GAIN:
      LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): GET_DIFF_GAIN");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ( ! data_out->SetSize(1, 1))
	return wbcrun::srv::OTHER_ERROR;
      data_out->GetElement(0) = task->diffGain();
      LOG_DEBUG (logger,  "  diff gain: " << data_out->GetElement(0));
      break;

    case wbcrun::cmd::SET_MAX_VEL:
      LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): SET_MAX_VEL");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ((data_in->NRows() < 1) || (data_in->NColumns() < 1))
	return wbcrun::srv::INVALID_DATA;
      task->maxVel(data_in->GetElement(0));
      LOG_DEBUG (logger,  "  new max vel: " << task->maxVel());
      break;
    
    case wbcrun::cmd::GET_MAX_VEL:
      LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): GET_MAX_VEL");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ( ! data_out->SetSize(1, 1))
	return wbcrun::srv::OTHER_ERROR;
      data_out->GetElement(0) = task->maxVel();
      LOG_DEBUG (logger,  "  max vel: " << data_out->GetElement(0));
      break;

    case wbcrun::cmd::SET_MAX_ACCEL:
      LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): SET_MAX_ACCEL");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ((data_in->NRows() < 1) || (data_in->NColumns() < 1))
	return wbcrun::srv::INVALID_DATA;
      task->maxAccel(data_in->GetElement(0));
      LOG_DEBUG (logger,  "  new max accel: " << task->maxAccel());
      break;
    
    case wbcrun::cmd::GET_MAX_ACCEL:
      LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): GET_MAX_ACCEL");
      // XXXX one day we can treat vectors of gains (one per joint or other dimension)
      if ( ! data_out->SetSize(1, 1))
	return wbcrun::srv::OTHER_ERROR;
      data_out->GetElement(0) = task->maxAccel();
      LOG_DEBUG (logger,  "  max accel: " << data_out->GetElement(0));
      break;
    
    default:
      LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): unhandled request\n"
		      << "  requestID = " << requestID
		      << " \"" << wbcrun::cmd::request_to_string(requestID) << "\"");
      return wbcrun::srv::INVALID_REQUEST;
    }
  
    LOG_DEBUG (logger,  "BehaviorDirectory::HandleTaskCmd(): SUCCESS");
  
    return wbcrun::srv::SUCCESS;
  }

}
