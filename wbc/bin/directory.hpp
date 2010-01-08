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
   \file directory.hpp
   \author Roland Philippsen
   \note Originally Copyright (c) 2009 Roland Philippsen, released under a BSD license.
*/

#ifndef WBC_DIRECTORY_HPP
#define WBC_DIRECTORY_HPP

#include <wbcnet/msg/Service.hpp>
#include <list>
#include <string>

namespace wbc {
  
  
  typedef std::list<std::string> listing_t;
  typedef std::list<wbcnet::srv_command_t> command_list_t;
  
  
  class Directory
  {
  public:
    virtual ~Directory() {}
    
    /** \todo Maybe add str_in and str_out parameters to all other
	abstract methods as well. */
    virtual wbcnet::srv_result_t HandleServoCmd(int commandID,
						wbcnet::srv_code_t const * code_in,
						wbcnet::srv_matrix_t const * data_in,
						listing_t const & str_in,
						wbcnet::srv_code_t * code_out,
						wbcnet::srv_matrix_t * data_out,
						listing_t & str_out) = 0;
    
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
						listing_t const & str_in,
						wbcnet::srv_code_t * code_out,
						wbcnet::srv_matrix_t * data_out,
						listing_t & str_out);
    
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
  class BaseDirectoryCmdServer
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

#endif // WBC_DIRECTORY_HPP
