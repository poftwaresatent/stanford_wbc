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

#ifndef WBCRUN_DIRECTORY_HPP
#define WBCRUN_DIRECTORY_HPP

#include <wbcnet/msg/Service.hpp>
#include <list>
#include <string>

namespace wbcrun {
  
  
  typedef std::list<std::string> listing_t;
  typedef std::list<wbcnet::srv_command_t> command_list_t;
  
  
  class Directory
  {
  public:
    virtual ~Directory() {}
    
    virtual wbcnet::srv_result_t HandleServoCmd(int commandID,
					 wbcnet::srv_code_t const * code_in,
					 wbcnet::srv_matrix_t const * data_in,
					 wbcnet::srv_code_t * code_out,
					 wbcnet::srv_matrix_t * data_out) = 0;
    
    virtual wbcnet::srv_result_t ListBehaviors(listing_t & behaviors) const = 0;
    
    virtual wbcnet::srv_result_t ListBehaviorCmds(int behaviorID,
					   command_list_t & commands) const = 0;
    
    virtual wbcnet::srv_result_t HandleBehaviorCmd(int behaviorID,
					    int commandID,
					    wbcnet::srv_code_t const * code_in,
					    wbcnet::srv_matrix_t const * data_in,
					    wbcnet::srv_code_t * code_out,
					    wbcnet::srv_matrix_t * data_out) = 0;
    
    virtual wbcnet::srv_result_t ListTasks(int behaviorID,
				    listing_t & tasks) const = 0;
    
    virtual wbcnet::srv_result_t ListTaskCmds(int behaviorID,
				       int taskID,
				       command_list_t & commands) const = 0;
    
    virtual wbcnet::srv_result_t HandleTaskCmd(int behaviorID,
					int taskID,
					int commandID,
					wbcnet::srv_code_t const * code_in,
					wbcnet::srv_matrix_t const * data_in,
					wbcnet::srv_code_t * code_out,
					wbcnet::srv_matrix_t * data_out) = 0;
  };
  
  
  class ServiceTransaction {
  public:
    virtual ~ServiceTransaction() {}
    virtual wbcnet::msg::Service * GetRequest() = 0;
    virtual wbcnet::msg::Service * GetReply() = 0;
    virtual void SendWaitReceive() = 0;
  };
  
  
  class DirectoryCmdClient
    : public Directory
  {
  public:
    DirectoryCmdClient(ServiceTransaction * transaction,
		       bool own_transaction);
    virtual ~DirectoryCmdClient();
    
    virtual wbcnet::srv_result_t HandleServoCmd(int commandID,
					 wbcnet::srv_code_t const * code_in,
					 wbcnet::srv_matrix_t const * data_in,
					 wbcnet::srv_code_t * code_out,
					 wbcnet::srv_matrix_t * data_out);
    
    virtual wbcnet::srv_result_t ListBehaviors(listing_t & behaviors) const;
    
    virtual wbcnet::srv_result_t ListBehaviorCmds(int behaviorID,
					   command_list_t & commands) const;
    
    wbcnet::srv_result_t ListBehaviorCmds(int behaviorID,
				   command_list_t & commands,
				   listing_t & command_names) const;
    
    virtual wbcnet::srv_result_t HandleBehaviorCmd(int behaviorID,
					    int commandID,
					    wbcnet::srv_code_t const * code_in,
					    wbcnet::srv_matrix_t const * data_in,
					    wbcnet::srv_code_t * code_out,
					    wbcnet::srv_matrix_t * data_out);
    
    virtual wbcnet::srv_result_t ListTasks(int behaviorID,
				    listing_t & tasks) const;
    
    virtual wbcnet::srv_result_t ListTaskCmds(int behaviorID,
				       int taskID,
				       command_list_t & commands) const;
    
    wbcnet::srv_result_t ListTaskCmds(int behaviorID,
			       int taskID,
			       command_list_t & commands,
			       listing_t & command_names) const;
    
    virtual wbcnet::srv_result_t HandleTaskCmd(int behaviorID,
					int taskID,
					int commandID,
					wbcnet::srv_code_t const * code_in,
					wbcnet::srv_matrix_t const * data_in,
					wbcnet::srv_code_t * code_out,
					wbcnet::srv_matrix_t * data_out);
    
  protected:
    ServiceTransaction * m_transaction;
    bool m_own_transaction;
  };
  
  
  /**
     This subclass only adds some minimal dispatching
     functionality. In order to implement a user command server, you
     should subclass this and provide implemetations for the methods
     that have been left pure virtual.
   */
  class DirectoryCmdServer
    : public Directory
  {
  public:
    /**
       Unpack the request along the domain, command, and other code
       elements, and call the corresponding more specific methods that
       need to be implemented by subclasses, extracting the relevant
       parts of the request code vector in the process. Protocol
       errors and similar glitches are also directly caught here, so
       subclasses need not worry about them.
       
       - SRV_SERVO_DOMAIN: if the command is SRV_GET_BEHAVIOR_LIST,
         the request gets forwarded to ListBehaviors(), otherwise to
         HandleServoCmd().
	 
       - SRV_BEHAVIOR_DOMAIN: if the command is SRV_GET_COMMAND_LIST,
         the request gets forwarded to ListBehaviorCmds(). If the
         command is SRV_GET_TASK_LIST, then ListTasks() gets
         called. Otherwise, HandleBehaviorCmd() is the method that
         catches it.
	 
       - SRV_TASK_DOMAIN: if the command is SRV_GET_COMMAND_LIST, the
         request gets forwarded to ListTaskCmds(). Otherwise,
         HandleTaskCmd() is called.
	 
       - SRV_OTHER_DOMAIN and upwards, as well as negative domain
         codes, are currently treated like SRV_NOT_IMPLEMENTED.
	 
       \todo Implement SRV_OTHER_DOMAIN handling (probably needs some
       tweaks to the super class).
       
       \return true if the request got dispatched, false otherwise. In
       any case, you can just go ahead and send the reply over the
       wire (unless the subclass code that it has been dispatched to
       bungled it).
    */
    bool Dispatch(wbcnet::msg::Service const & request,
		  wbcnet::msg::Service & reply);
  };
  
}

#endif // WBCRUN_DIRECTORY_HPP
