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

#include "directory.hpp"

using namespace std;
using namespace wbcnet;

namespace wbcrun {
  
  
  bool DirectoryDispatcher::Handle(Directory & directory,
				   wbcnet::msg::Service const & request,
				   wbcnet::msg::Service & reply)
  {
    if (1 > request.code.NElements()) {
      reply.code.SetNElements(1);
      reply.code[0] = SRV_MISSING_CODE;
      reply.matrix.SetSize(0, 0);
      return true;
    }
    
    switch (request.code[0]) {
      
    case SRV_SERVO_DOMAIN:
      if (2 > request.code.NElements()) {
	reply.code.SetNElements(1);
	reply.code[0] = SRV_MISSING_CODE;
	reply.matrix.SetSize(0, 0);
      }
      else {
	int const commandID(request.code[1]);
	reply.clear();
	int32_t result;
	if (SRV_GET_BEHAVIOR_LIST == commandID) {
	  listing_t behaviors;
	  result = directory.ListBehaviors(behaviors);
	  if (SRV_SUCCESS == result)
	    if ( ! reply.append(behaviors.begin(), behaviors.end()))
	      result = SRV_OTHER_ERROR;
	  reply.code.SetNElements(1);
	  reply.matrix.SetSize(0, 0);
	}
	else {
	  result = directory.HandleServoCmd(commandID,
					    &request.code.CreateSplice(2), &request.matrix,
					    &reply.code, &reply.matrix);
	}
	// make sure reply.code[0] is set to the result code... should
	// be done inside directory, but you never know.
	if (reply.code.NElements() < 1)
	  reply.code.SetNElements(1);
	reply.code[0] = result;
      }
      return true;
      
    case SRV_BEHAVIOR_DOMAIN:
      if (3 > request.code.NElements()) {
	reply.code.SetNElements(1);
	reply.code[0] = SRV_MISSING_CODE;
	reply.matrix.SetSize(0, 0);
      }
      else {
	int const behaviorID(request.code[1]);
	int const commandID(request.code[2]);
	int32_t result;
	if (SRV_GET_COMMAND_LIST == commandID) {
	  command_list_t commands;
	  result = directory.ListBehaviorCmds(behaviorID, commands);
	  if (SRV_SUCCESS != result)
	    reply.code.SetNElements(1);
	  else {
	    reply.code.SetNElements(commands.size() + 1);
	    size_t ii(1);
	    for (command_list_t::const_iterator rr(commands.begin()); rr != commands.end();
		 ++rr, ++ii)
	      reply.code[ii] = *rr;
	  }
	}
	else if (SRV_GET_TASK_LIST == commandID) {
	  listing_t tasks;
	  result = directory.ListTasks(behaviorID, tasks);
	  reply.code.SetNElements(1);
	  reply.clear();	// XXXX need to do this more often all over this code
	  reply.matrix.SetSize(0, 0); // XXXX need to do this more often all over this code
	  if (SRV_SUCCESS == result) {
	    if ( ! reply.append(tasks.begin(), tasks.end()))
	      result = SRV_OTHER_ERROR;
	  }
	}
	else {
	  result = directory.HandleBehaviorCmd(behaviorID, commandID,
					       &request.code.CreateSplice(2), &request.matrix,
					       &reply.code, &reply.matrix);
	}
	// make sure reply.code[0] is set to the result code... should
	// be done inside directory, but you never know.
	if (reply.code.NElements() < 1)
	  reply.code.SetNElements(1);
	reply.code[0] = result;
      }
      return true;
      
    case SRV_TASK_DOMAIN:
      if (4 > request.code.NElements()) {
	reply.code.SetNElements(1);
	reply.code[0] = SRV_MISSING_CODE;
	reply.matrix.SetSize(0, 0);
      }
      else {
	int const behaviorID(request.code[1]);
	int const taskID(request.code[2]);
	int const commandID(request.code[3]);
	int32_t result;
	if (SRV_GET_COMMAND_LIST == commandID) {
	  command_list_t commands;
	  result = directory.ListTaskCmds(behaviorID, taskID, commands);
	  if (SRV_SUCCESS != result)
	    reply.code.SetNElements(1);
	  else {
	    reply.code.SetNElements(commands.size() + 1);
	    size_t ii(1);
	    for (command_list_t::const_iterator rr(commands.begin()); rr != commands.end();
		 ++rr, ++ii)
	      reply.code[ii] = *rr;
	  }
	}
	else {
	  result = directory.HandleTaskCmd(behaviorID, taskID, commandID,
					   &request.code.CreateSplice(3), &request.matrix,
					   &reply.code, &reply.matrix);
	}
	// make sure reply.code[0] is set to the result code... should
	// be done inside directory, but you never know.
	if (reply.code.NElements() < 1)
	  reply.code.SetNElements(1);
	reply.code[0] = result;
      }
      return true;
      
    }
    
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
		 wbcnet::srv_code_t * code_out,
		 wbcnet::srv_matrix_t * data_out)
  {
    m_transaction->GetRequest()->InitServoCmd(commandID, code_in, data_in);
    m_transaction->SendWaitReceive();
    code_out->Copy(m_transaction->GetReply()->code);
    data_out->Copy(m_transaction->GetReply()->matrix);
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
