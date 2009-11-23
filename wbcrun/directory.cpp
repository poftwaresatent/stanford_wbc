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
#include <cstring>

using namespace std;

namespace wbcrun {

  namespace cmd {
    
    static char const * request_name[] = {
      "GET_TASK_TYPE",
      "GET_DIMENSION",
      "GET_LINK_ANGLE",
      "GET_LINK_TRANSFORM",
      "SET_BEHAVIOR",
      "SET_GOAL",
      "GET_GOAL",
      "GET_ACTUAL",
      "SET_PROP_GAIN",
      "GET_PROP_GAIN",
      "SET_DIFF_GAIN",
      "GET_DIFF_GAIN",
      "SET_MAX_VEL",
      "GET_MAX_VEL",
      "SET_MAX_ACCEL",
      "GET_MAX_ACCEL",
      "GET_BEHAVIOR_LIST",
      "GET_COMMAND_LIST",
      "GET_TASK_LIST"
      // When adding stuff at the end here, you have to update ALL
      // locations down below which use the last entry of the enum in
      // order to check for the size.
    };
    
    
    char const * request_to_string(int request)
    {
      if ((0 > request) || (GET_TASK_LIST < request))
	return 0;
      return request_name[request];
    }
    
    
    int string_to_request(std::string const & name)
    {
      static map<string, int> nm;
      if (nm.empty())
	for (int ii(0); ii <= GET_TASK_LIST; ++ii)
	  nm[request_name[ii]] = ii;
      map<string, int>::const_iterator inm(nm.find(name));
      if (nm.end() == inm)
	return -1;
      return inm->second;
    }
    
  }
  
  
  bool DirectoryDispatcher::Handle(Directory & directory,
				   ServiceMessage const & request,
				   ServiceMessage & reply)
  {
    if (1 > request.code.NElements()) {
      reply.code.SetNElements(1);
      reply.code[0] = srv::MISSING_CODE;
      reply.matrix.SetSize(0, 0);
      return true;
    }
    
    switch (request.code[0]) {
      
    case srv::SERVO_DIR:
      if (2 > request.code.NElements()) {
	reply.code.SetNElements(1);
	reply.code[0] = srv::MISSING_CODE;
	reply.matrix.SetSize(0, 0);
      }
      else {
	int const requestID(request.code[1]);
	reply.clear();
	int32_t result;
	if (cmd::GET_BEHAVIOR_LIST == requestID) {
	  listing_t behaviors;
	  result = directory.ListBehaviors(behaviors);
	  if (srv::SUCCESS == result)
	    if ( ! reply.append(behaviors.begin(), behaviors.end()))
	      result = srv::OTHER_ERROR;
	  reply.code.SetNElements(1);
	  reply.matrix.SetSize(0, 0);
	}
	else {
	  result = directory.HandleServoCmd(requestID,
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
      
    case srv::BEHAVIOR_DIR:
      if (3 > request.code.NElements()) {
	reply.code.SetNElements(1);
	reply.code[0] = srv::MISSING_CODE;
	reply.matrix.SetSize(0, 0);
      }
      else {
	int const behaviorID(request.code[1]);
	int const requestID(request.code[2]);
	int32_t result;
	if (cmd::GET_COMMAND_LIST == requestID) {
	  request_list_t requests;
	  result = directory.ListBehaviorCmds(behaviorID, requests);
	  if (srv::SUCCESS != result)
	    reply.code.SetNElements(1);
	  else {
	    reply.code.SetNElements(requests.size() + 1);
	    size_t ii(1);
	    for (request_list_t::const_iterator rr(requests.begin()); rr != requests.end();
		 ++rr, ++ii)
	      reply.code[ii] = *rr;
	  }
	}
	else if (cmd::GET_TASK_LIST == requestID) {
	  listing_t tasks;
	  result = directory.ListTasks(behaviorID, tasks);
	  reply.code.SetNElements(1);
	  reply.clear();	// XXXX need to do this more often all over this code
	  reply.matrix.SetSize(0, 0); // XXXX need to do this more often all over this code
	  if (srv::SUCCESS == result) {
	    if ( ! reply.append(tasks.begin(), tasks.end()))
	      result = srv::OTHER_ERROR;
	  }
	}
	else {
	  result = directory.HandleBehaviorCmd(behaviorID, requestID,
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
      
    case srv::TASK_DIR:
      if (4 > request.code.NElements()) {
	reply.code.SetNElements(1);
	reply.code[0] = srv::MISSING_CODE;
	reply.matrix.SetSize(0, 0);
      }
      else {
	int const behaviorID(request.code[1]);
	int const taskID(request.code[2]);
	int const requestID(request.code[3]);
	int32_t result;
	if (cmd::GET_COMMAND_LIST == requestID) {
	  request_list_t requests;
	  result = directory.ListTaskCmds(behaviorID, taskID, requests);
	  if (srv::SUCCESS != result)
	    reply.code.SetNElements(1);
	  else {
	    reply.code.SetNElements(requests.size() + 1);
	    size_t ii(1);
	    for (request_list_t::const_iterator rr(requests.begin()); rr != requests.end();
		 ++rr, ++ii)
	      reply.code[ii] = *rr;
	  }
	}
	else {
	  result = directory.HandleTaskCmd(behaviorID, taskID, requestID,
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
  
  
  srv::result_t DirectoryCmdClient::
  ListBehaviors(listing_t & behaviors) const
  {
    CreateListBehaviorsRequest(m_transaction->GetRequest());
    m_transaction->SendWaitReceive();
    behaviors.clear();
    if (m_transaction->GetReply()->extract(behaviors)
	&& (m_transaction->GetReply()->code.NElements() > 0))
      return (srv::result_t) m_transaction->GetReply()->code[0];
    return srv::OTHER_ERROR;
  }
  
  
  srv::result_t DirectoryCmdClient::
  ListBehaviorCmds(int behaviorID, request_list_t & requests) const
  {
    CreateListBehaviorCmdsRequest(behaviorID, m_transaction->GetRequest());
    m_transaction->SendWaitReceive();
    requests.clear();
    ServiceMessage::vector_type const & code(m_transaction->GetReply()->code);
    int const ncodes(code.NElements());
    if (ncodes < 1)
      return srv::OTHER_ERROR;
    if (code[0] == srv::SUCCESS)
      for (int ii(1); ii < ncodes; ++ii)
	requests.push_back((cmd::request_t) code[ii]);
    return (srv::result_t) code[0];
  }
  
  
  srv::result_t DirectoryCmdClient::
  ListBehaviorCmds(int behaviorID, request_list_t & requests, listing_t & request_names) const
  {
    srv::result_t const rr(ListBehaviorCmds(behaviorID, requests));
    request_names.clear();
    if (srv::SUCCESS == rr) {
      for (request_list_t::const_iterator ii(requests.begin()); ii != requests.end(); ++ii) {
	char const * raw(request_to_string(*ii));
	if (raw)
	  request_names.push_back(raw);
	else
	  request_names.push_back("(invalid ID)");
      }
    }
    return rr;
  }
  
  
  srv::result_t DirectoryCmdClient::
  HandleServoCmd(int requestID,
		 ServiceMessage::vector_type const * code_in,
		 ServiceMessage::matrix_type const * data_in,
		 ServiceMessage::vector_type * code_out,
		 ServiceMessage::matrix_type * data_out)
  {
    CreateServoCmdRequest(requestID, code_in, data_in, m_transaction->GetRequest());
    m_transaction->SendWaitReceive();
    code_out->Copy(m_transaction->GetReply()->code);
    data_out->Copy(m_transaction->GetReply()->matrix);
    if (code_out->NElements() > 0)
      return srv::result_t((*code_out)[0]);
    return srv::OTHER_ERROR;
  }
  
  
  srv::result_t DirectoryCmdClient::
  HandleBehaviorCmd(int behaviorID,
		    int requestID,
		    ServiceMessage::vector_type const * code_in,
		    ServiceMessage::matrix_type const * data_in,
		    ServiceMessage::vector_type * code_out,
		    ServiceMessage::matrix_type * data_out)
  {
    CreateBehaviorCmdRequest(behaviorID, requestID, code_in, data_in, m_transaction->GetRequest());
    m_transaction->SendWaitReceive();
    code_out->Copy(m_transaction->GetReply()->code);
    data_out->Copy(m_transaction->GetReply()->matrix);
    if (code_out->NElements() > 0)
      return srv::result_t((*code_out)[0]);
    return srv::OTHER_ERROR;
  }
    
  
  srv::result_t DirectoryCmdClient::
  ListTasks(int behaviorID, listing_t & tasks) const
  {
    CreateListTasksRequest(behaviorID, m_transaction->GetRequest());
    m_transaction->SendWaitReceive();
    tasks.clear();
    if (m_transaction->GetReply()->extract(tasks)
	&& (m_transaction->GetReply()->code.NElements() > 0))
      return (srv::result_t) m_transaction->GetReply()->code[0];
    return srv::OTHER_ERROR;
  }
  
  
  srv::result_t DirectoryCmdClient::
  ListTaskCmds(int behaviorID, int taskID, request_list_t & requests) const
  {
    CreateListTaskCmdsRequest(behaviorID, taskID, m_transaction->GetRequest());
    m_transaction->SendWaitReceive();
    requests.clear();
    ServiceMessage::vector_type const & code(m_transaction->GetReply()->code);
    int const ncodes(code.NElements());
    if (ncodes < 1)
      return srv::OTHER_ERROR;
    if (code[0] == srv::SUCCESS)
      for (int ii(1); ii < ncodes; ++ii)
	requests.push_back((cmd::request_t) code[ii]);
    return (srv::result_t) code[0];
  }
  
  
  srv::result_t DirectoryCmdClient::
  ListTaskCmds(int behaviorID, int taskID, request_list_t & requests,
	       listing_t & request_names) const
  {
    srv::result_t const rr(ListTaskCmds(behaviorID, taskID, requests));
    request_names.clear();
    if (srv::SUCCESS == rr) {
      for (request_list_t::const_iterator ii(requests.begin()); ii != requests.end(); ++ii) {
	char const * raw(request_to_string(*ii));
	if (raw)
	  request_names.push_back(raw);
	else
	  request_names.push_back("(invalid ID)");
      }
    }
    return rr;
  }
  
  
  srv::result_t DirectoryCmdClient::
  HandleTaskCmd(int behaviorID,
		int taskID,
		int requestID,
		ServiceMessage::vector_type const * code_in,
		ServiceMessage::matrix_type const * data_in,
		ServiceMessage::vector_type * code_out,
		ServiceMessage::matrix_type * data_out)
  {
    CreateTaskCmdRequest(behaviorID, taskID, requestID, code_in, data_in,
			 m_transaction->GetRequest());
    m_transaction->SendWaitReceive();
    code_out->Copy(m_transaction->GetReply()->code);
    data_out->Copy(m_transaction->GetReply()->matrix);
    if (code_out->NElements() > 0)
      return srv::result_t((*code_out)[0]);
    return srv::OTHER_ERROR;
  }
  
  
  void DirectoryCmdClient::
  CreateListBehaviorsRequest(ServiceMessage * msg)
  {
    msg->code.SetNElements(2);
    msg->code[0] = srv::SERVO_DIR;
    msg->code[1] = cmd::GET_BEHAVIOR_LIST;
    msg->matrix.SetSize(0, 0);
    msg->clear();		// clears the string list
  }
  
  
  void DirectoryCmdClient::
  CreateListBehaviorCmdsRequest(int behaviorID,
				ServiceMessage * msg)
  {
    msg->code.SetNElements(3);
    msg->code[0] = srv::BEHAVIOR_DIR;
    msg->code[1] = behaviorID;
    msg->code[2] = cmd::GET_COMMAND_LIST;
    msg->matrix.SetSize(0, 0);
    msg->clear();		// clears the string list
  }
  
  
  void DirectoryCmdClient::
  CreateServoCmdRequest(int requestID,
			ServiceMessage::vector_type const * code_in,
			ServiceMessage::matrix_type const * data_in,
			ServiceMessage * request)
  {
    request->code.SetNElements(code_in->NElements() + 2);
    request->code[0] = srv::SERVO_DIR;
    request->code[1] = requestID;
    if (code_in->NElements() > 0)
      memmove(request->code.ElementPointer() + 2, code_in->ElementPointer(),
	      code_in->NElements() * code_in->ElementStorageSize());
    request->matrix.Copy(*data_in);
  }
  
  
  void DirectoryCmdClient::
  CreateBehaviorCmdRequest(int behaviorID,
			   int requestID,
			   ServiceMessage::vector_type const * code_in,
			   ServiceMessage::matrix_type const * data_in,
			   ServiceMessage * request)
  {
    request->code.SetNElements(code_in->NElements() + 3);
    request->code[0] = srv::BEHAVIOR_DIR;
    request->code[1] = behaviorID;
    request->code[2] = requestID;
    if (code_in->NElements() > 0)
      memmove(request->code.ElementPointer() + 3, code_in->ElementPointer(),
	      code_in->NElements() * code_in->ElementStorageSize());
    request->matrix.Copy(*data_in);
  }
  
  
  void DirectoryCmdClient::
  CreateListTasksRequest(int behaviorID,
			 ServiceMessage * msg)
  {
    msg->code.SetNElements(3);
    msg->code[0] = srv::BEHAVIOR_DIR;
    msg->code[1] = behaviorID;
    msg->code[2] = cmd::GET_TASK_LIST;
    msg->matrix.SetSize(0, 0);
    msg->clear();		// clears the string list
  }
  
  
  void DirectoryCmdClient::
  CreateListTaskCmdsRequest(int behaviorID,
			    int taskID,
			    ServiceMessage * msg)
  {
    msg->code.SetNElements(4);
    msg->code[0] = srv::TASK_DIR;
    msg->code[1] = behaviorID;
    msg->code[2] = taskID;
    msg->code[3] = cmd::GET_COMMAND_LIST;
    msg->matrix.SetSize(0, 0);
    msg->clear();		// clears the string list
  }
  
  
  void DirectoryCmdClient::
  CreateTaskCmdRequest(int behaviorID,
		       int taskID,
		       int requestID,
		       ServiceMessage::vector_type const * code_in,
		       ServiceMessage::matrix_type const * data_in,
		       ServiceMessage * request)
  {
    request->code.SetNElements(code_in->NElements() + 4);
    request->code[0] = srv::TASK_DIR;
    request->code[1] = behaviorID;
    request->code[2] = taskID;
    request->code[3] = requestID;
    if (code_in->NElements() > 0)
      memmove(request->code.ElementPointer() + 4, code_in->ElementPointer(),
	      code_in->NElements() * code_in->ElementStorageSize());
    request->matrix.Copy(*data_in);
  }
  
}
