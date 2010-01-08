/*
 * Copyright (c) 2010 Stanford University
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
   \file directory.cpp
   \author Roland Philippsen
   \note Originally Copyright (c) 2009 Roland Philippsen, released under a BSD license.
*/

#include "directory.hpp"

using namespace std;
using namespace wbcnet;

namespace wbc {
  
  
  bool BaseDirectoryCmdServer::
  Dispatch(wbcnet::msg::Service const & request,
	   wbcnet::msg::Service & reply)
  {
    reply.InitReply(request);
    
    if (1 > request.code.NElements()) {
      reply.code[0] = SRV_MISSING_CODE;
      return false;
    }
    
    int const domain(request.code[0]);
    
    // servo domain //////////////////////////////////////////////////
    
    if (SRV_SERVO_DOMAIN == domain) {
      
      if (2 > request.code.NElements()) {
	reply.code[0] = SRV_MISSING_CODE;
	return false;
      }
      
      if (SRV_GET_BEHAVIOR_LIST == request.code[1]) {
	listing_t behaviors;
	reply.code[0] = ListBehaviors(behaviors);
	if (SRV_SUCCESS == reply.code[0]) {
	  if ( ! reply.append(behaviors.begin(), behaviors.end())) {
	    reply.code[0] = SRV_OTHER_ERROR;
	  }
	}
	return true;
      }
      
      std::list<std::string> str_in;
      std::list<std::string> str_out;
      if ( ! request.extract(str_in)) {
	reply.code[0] = SRV_OTHER_ERROR + 1;
      }
      reply.code[0] = HandleServoCmd(request.code[1],
				     &request.code.CreateSplice(2),
				     &request.matrix,
				     str_in,
				     &reply.code,
				     &reply.matrix,
				     str_out);
      if ( ! reply.append(str_out.begin(), str_out.end())) {
	reply.code[0] = SRV_OTHER_ERROR + 2;
      }
      return true;
      
    }
    
    // behavior domain //////////////////////////////////////////////////
    
    if (SRV_BEHAVIOR_DOMAIN == domain) {
      
      if (3 > request.code.NElements()) {
	reply.code[0] = SRV_MISSING_CODE;
	return false;
      }
      
      int const behaviorID(request.code[1]);
      int const commandID(request.code[2]);
      
      if (SRV_GET_COMMAND_LIST == commandID) {
	command_list_t commands;
	reply.code[0] = ListBehaviorCmds(behaviorID, commands);
	if (SRV_SUCCESS == reply.code[0]) {
	  reply.code.SetNElements(commands.size() + 1);
	  size_t ii(1);
	  for (command_list_t::const_iterator rr(commands.begin()); rr != commands.end();
	       ++rr, ++ii)
	    reply.code[ii] = *rr;
	}
	return true;
      }
      
      if (SRV_GET_TASK_LIST == commandID) {
	listing_t tasks;
	reply.code[0] = ListTasks(behaviorID, tasks);
	if (SRV_SUCCESS == reply.code[0]) {
	  if ( ! reply.append(tasks.begin(), tasks.end())) {
	    reply.code[0] = SRV_OTHER_ERROR;
	  }
	}
	return true;
      }
      
      reply.code[0] = HandleBehaviorCmd(behaviorID, commandID,
					&request.code.CreateSplice(2), &request.matrix,
					&reply.code, &reply.matrix);
      return true;
      
    }
    
    // task domain //////////////////////////////////////////////////
    
    if (SRV_TASK_DOMAIN == domain) {
      
      if (4 > request.code.NElements()) {
	reply.code[0] = SRV_MISSING_CODE;
	return false;
      }
      
      int const behaviorID(request.code[1]);
      int const taskID(request.code[2]);
      int const commandID(request.code[3]);
      
      if (SRV_GET_COMMAND_LIST == commandID) {
	command_list_t commands;
	reply.code[0] = ListTaskCmds(behaviorID, taskID, commands);
	if (SRV_SUCCESS == reply.code[0]) {
	  reply.code.SetNElements(commands.size() + 1);
	  size_t ii(1);
	  for (command_list_t::const_iterator rr(commands.begin()); rr != commands.end();
	       ++rr, ++ii)
	    reply.code[ii] = *rr;
	  return true;
	}
      }
      
      reply.code[0] = HandleTaskCmd(behaviorID, taskID, commandID,
				    &request.code.CreateSplice(3), &request.matrix,
				    &reply.code, &reply.matrix);
      return true;
      
    }
    
    // other domains: currently not implemented
    
    reply.code[0] = SRV_NOT_IMPLEMENTED; // redundant but cheap
    
    return false;
  }
  
  
  DirectoryCmdClient::
  DirectoryCmdClient(ServiceTransaction * transaction,
		     bool own_transaction)
    : m_transaction(transaction),
      m_own_transaction(own_transaction)
  {
  }
  
  
  DirectoryCmdClient::
  ~DirectoryCmdClient()
  {
    if (m_own_transaction)
      delete m_transaction;
  }
  
  
  wbcnet::srv_result_t DirectoryCmdClient::
  ListBehaviors(listing_t & behaviors) const
  {
    m_transaction->GetRequest()->InitListBehaviors();
    m_transaction->SendWaitReceive();
    behaviors.clear();
    if (m_transaction->GetReply()->extract(behaviors)
	&& (m_transaction->GetReply()->code.NElements() > 0))
      return (wbcnet::srv_result_t) m_transaction->GetReply()->code[0];
    return SRV_OTHER_ERROR;
  }
  
  
  wbcnet::srv_result_t DirectoryCmdClient::
  ListBehaviorCmds(int behaviorID, command_list_t & commands) const
  {
    m_transaction->GetRequest()->InitListBehaviorCmds(behaviorID);
    m_transaction->SendWaitReceive();
    commands.clear();
    wbcnet::srv_code_t const & code(m_transaction->GetReply()->code);
    int const ncodes(code.NElements());
    if (ncodes < 1)
      return SRV_OTHER_ERROR;
    if (code[0] == SRV_SUCCESS)
      for (int ii(1); ii < ncodes; ++ii)
	commands.push_back((srv_command_t) code[ii]);
    return (wbcnet::srv_result_t) code[0];
  }
  
  
  wbcnet::srv_result_t DirectoryCmdClient::
  ListBehaviorCmds(int behaviorID, command_list_t & commands, listing_t & command_names) const
  {
    wbcnet::srv_result_t const rr(ListBehaviorCmds(behaviorID, commands));
    command_names.clear();
    if (SRV_SUCCESS == rr) {
      for (command_list_t::const_iterator ii(commands.begin()); ii != commands.end(); ++ii) {
	command_names.push_back(srv_command_to_string(*ii));
      }
    }
    return rr;
  }
  
  
  wbcnet::srv_result_t DirectoryCmdClient::
  HandleServoCmd(int commandID,
		 wbcnet::srv_code_t const * code_in,
		 wbcnet::srv_matrix_t const * data_in,
		 listing_t const & str_in,
		 wbcnet::srv_code_t * code_out,
		 wbcnet::srv_matrix_t * data_out,
		 listing_t & str_out)
  {
    m_transaction->GetRequest()->InitServoCmd(commandID, code_in, data_in, str_in);
    m_transaction->SendWaitReceive();
    code_out->Copy(m_transaction->GetReply()->code);
    data_out->Copy(m_transaction->GetReply()->matrix);
    str_out.clear();
    m_transaction->GetReply()->extract(str_out);
    if (code_out->NElements() > 0)
      return wbcnet::srv_result_t((*code_out)[0]);
    return SRV_OTHER_ERROR;
  }
  
  
  wbcnet::srv_result_t DirectoryCmdClient::
  HandleBehaviorCmd(int behaviorID,
		    int commandID,
		    wbcnet::srv_code_t const * code_in,
		    wbcnet::srv_matrix_t const * data_in,
		    wbcnet::srv_code_t * code_out,
		    wbcnet::srv_matrix_t * data_out)
  {
    m_transaction->GetRequest()->InitBehaviorCmd(behaviorID, commandID, code_in, data_in);
    m_transaction->SendWaitReceive();
    code_out->Copy(m_transaction->GetReply()->code);
    data_out->Copy(m_transaction->GetReply()->matrix);
    if (code_out->NElements() > 0)
      return wbcnet::srv_result_t((*code_out)[0]);
    return SRV_OTHER_ERROR;
  }
    
  
  wbcnet::srv_result_t DirectoryCmdClient::
  ListTasks(int behaviorID, listing_t & tasks) const
  {
    m_transaction->GetRequest()->InitListTasks(behaviorID);
    m_transaction->SendWaitReceive();
    tasks.clear();
    if (m_transaction->GetReply()->extract(tasks)
	&& (m_transaction->GetReply()->code.NElements() > 0))
      return (wbcnet::srv_result_t) m_transaction->GetReply()->code[0];
    return SRV_OTHER_ERROR;
  }
  
  
  wbcnet::srv_result_t DirectoryCmdClient::
  ListTaskCmds(int behaviorID, int taskID, command_list_t & commands) const
  {
    m_transaction->GetRequest()->InitListTaskCmds(behaviorID, taskID);
    m_transaction->SendWaitReceive();
    commands.clear();
    wbcnet::srv_code_t const & code(m_transaction->GetReply()->code);
    int const ncodes(code.NElements());
    if (ncodes < 1)
      return SRV_OTHER_ERROR;
    if (code[0] == SRV_SUCCESS)
      for (int ii(1); ii < ncodes; ++ii)
	commands.push_back((srv_command_t) code[ii]);
    return (wbcnet::srv_result_t) code[0];
  }
  
  
  wbcnet::srv_result_t DirectoryCmdClient::
  ListTaskCmds(int behaviorID, int taskID, command_list_t & commands,
	       listing_t & command_names) const
  {
    wbcnet::srv_result_t const rr(ListTaskCmds(behaviorID, taskID, commands));
    command_names.clear();
    if (SRV_SUCCESS == rr) {
      for (command_list_t::const_iterator ii(commands.begin()); ii != commands.end(); ++ii) {
	command_names.push_back(srv_command_to_string(*ii));
      }
    }
    return rr;
  }
  
  
  wbcnet::srv_result_t DirectoryCmdClient::
  HandleTaskCmd(int behaviorID,
		int taskID,
		int commandID,
		wbcnet::srv_code_t const * code_in,
		wbcnet::srv_matrix_t const * data_in,
		wbcnet::srv_code_t * code_out,
		wbcnet::srv_matrix_t * data_out)
  {
    m_transaction->GetRequest()->InitTaskCmd(behaviorID, taskID, commandID, code_in, data_in);
    m_transaction->SendWaitReceive();
    code_out->Copy(m_transaction->GetReply()->code);
    data_out->Copy(m_transaction->GetReply()->matrix);
    if (code_out->NElements() > 0)
      return wbcnet::srv_result_t((*code_out)[0]);
    return SRV_OTHER_ERROR;
  }
  
}
