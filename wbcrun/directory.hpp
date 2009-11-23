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

#include <wbcrun/service.hpp>
#include <list>
#include <string>

namespace wbcrun {
  
  namespace cmd {
    
    /**       
       \note Partially redundant with wbcrun::srv::id_t, but only used
       embedded within a DIRECTORY request.
    */
    typedef enum {
      // also update request_to_string() and string_to_request() when changing this enum!
      GET_TASK_TYPE,
      GET_DIMENSION,
      GET_LINK_ANGLE,
      GET_LINK_TRANSFORM,
      SET_BEHAVIOR,
      SET_GOAL,
      GET_GOAL,
      GET_ACTUAL,
      SET_PROP_GAIN,
      GET_PROP_GAIN,
      SET_DIFF_GAIN,
      GET_DIFF_GAIN,
      SET_MAX_VEL,
      GET_MAX_VEL,
      SET_MAX_ACCEL,
      GET_MAX_ACCEL,
      GET_BEHAVIOR_LIST,
      GET_COMMAND_LIST,
      GET_TASK_LIST
      // also update request_to_string() and string_to_request() when changing this enum!
    } request_t;
    
    /** \return NULL for invalid request */
    char const * request_to_string(int request);
    
    /** \return -1 for invalid name */
    int string_to_request(std::string const & name);
    
  }
  
  typedef std::list<std::string> listing_t;
  typedef std::list<cmd::request_t> request_list_t;
  
  
  class Directory
  {
  public:
    virtual ~Directory() {}
    
    virtual srv::result_t HandleServoCmd(int requestID,
					 ServiceMessage::vector_type const * code_in,
					 ServiceMessage::matrix_type const * data_in,
					 ServiceMessage::vector_type * code_out,
					 ServiceMessage::matrix_type * data_out) = 0;
    
    virtual srv::result_t ListBehaviors(listing_t & behaviors) const = 0;
    
    virtual srv::result_t ListBehaviorCmds(int behaviorID,
					   request_list_t & requests) const = 0;
    
    virtual srv::result_t HandleBehaviorCmd(int behaviorID,
					    int requestID,
					    ServiceMessage::vector_type const * code_in,
					    ServiceMessage::matrix_type const * data_in,
					    ServiceMessage::vector_type * code_out,
					    ServiceMessage::matrix_type * data_out) = 0;
    
    virtual srv::result_t ListTasks(int behaviorID,
				    listing_t & tasks) const = 0;
    
    virtual srv::result_t ListTaskCmds(int behaviorID,
				       int taskID,
				       request_list_t & requests) const = 0;
    
    virtual srv::result_t HandleTaskCmd(int behaviorID,
					int taskID,
					int requestID,
					ServiceMessage::vector_type const * code_in,
					ServiceMessage::matrix_type const * data_in,
					ServiceMessage::vector_type * code_out,
					ServiceMessage::matrix_type * data_out) = 0;
  };
  
  
  class ServiceTransaction {
  public:
    virtual ~ServiceTransaction() {}
    virtual ServiceMessage * GetRequest() = 0;
    virtual ServiceMessage * GetReply() = 0;
    virtual void SendWaitReceive() = 0;
  };
  
  
  class DirectoryCmdClient
    : public Directory
  {
  public:
    DirectoryCmdClient(ServiceTransaction * transaction,
		       bool own_transaction);
    virtual ~DirectoryCmdClient();
    
    virtual srv::result_t HandleServoCmd(int requestID,
					 ServiceMessage::vector_type const * code_in,
					 ServiceMessage::matrix_type const * data_in,
					 ServiceMessage::vector_type * code_out,
					 ServiceMessage::matrix_type * data_out);
    
    virtual srv::result_t ListBehaviors(listing_t & behaviors) const;
    
    virtual srv::result_t ListBehaviorCmds(int behaviorID,
					   request_list_t & requests) const;
    
    srv::result_t ListBehaviorCmds(int behaviorID,
				   request_list_t & requests,
				   listing_t & request_names) const;
    
    virtual srv::result_t HandleBehaviorCmd(int behaviorID,
					    int requestID,
					    ServiceMessage::vector_type const * code_in,
					    ServiceMessage::matrix_type const * data_in,
					    ServiceMessage::vector_type * code_out,
					    ServiceMessage::matrix_type * data_out);
    
    virtual srv::result_t ListTasks(int behaviorID,
				    listing_t & tasks) const;
    
    virtual srv::result_t ListTaskCmds(int behaviorID,
				       int taskID,
				       request_list_t & requests) const;
    
    srv::result_t ListTaskCmds(int behaviorID,
			       int taskID,
			       request_list_t & requests,
			       listing_t & request_names) const;
    
    virtual srv::result_t HandleTaskCmd(int behaviorID,
					int taskID,
					int requestID,
					ServiceMessage::vector_type const * code_in,
					ServiceMessage::matrix_type const * data_in,
					ServiceMessage::vector_type * code_out,
					ServiceMessage::matrix_type * data_out);
    
    static void CreateListBehaviorsRequest(ServiceMessage * msg);
    
    static void CreateListBehaviorCmdsRequest(int behaviorID,
					      ServiceMessage * msg);
    
    static void CreateServoCmdRequest(int requestID,
				      ServiceMessage::vector_type const * code_in,
				      ServiceMessage::matrix_type const * data_in,
				      ServiceMessage * request);
    
    static void CreateBehaviorCmdRequest(int behaviorID,
					 int requestID,
					 ServiceMessage::vector_type const * code_in,
					 ServiceMessage::matrix_type const * data_in,
					 ServiceMessage * request);
    
    static void CreateListTasksRequest(int behaviorID,
				       ServiceMessage * msg);
    
    static void CreateListTaskCmdsRequest(int behaviorID,
					  int taskID,
					  ServiceMessage * msg);
    
    static void CreateTaskCmdRequest(int behaviorID,
				     int taskID,
				     int requestID,
				     ServiceMessage::vector_type const * code_in,
				     ServiceMessage::matrix_type const * data_in,
				     ServiceMessage * request);
    
  protected:
    ServiceTransaction * m_transaction;
    bool m_own_transaction;
  };
  

  /** \todo XXXX make the next logical step: turn it into
      DirectoryCmdServer! ...or, actually, into a static function,
      because it just does case-based marshalling of requests and
      replies */
  class DirectoryDispatcher
  {
  public:
    virtual ~DirectoryDispatcher() {}
    
    /**
       The default implementation tries to re-route some "standard"
       commands and returns true to indicate that it has handled the
       command (or detected a protocol error). In case of re-routing,
       some processing is done first:

       - BEHAVIOR_DIR eats up two elements of the request.code vector,
         code[0] being the BEHAVIOR_DIR tag and code[1] being the
         behavior ID passed to HandleBehaviorCmd().

       - TASK_DIR eats up three elements of request.code, code[0]
         being the TASK_DIR tag, code[1] being the behavior ID, and
         code[2] being the task ID passed to HandleTaskCmd().

       \note This list is undergoing current development, and the doc
       can be out of sync. Look at the code in order to get up-to-date
       information.
       
       The idea is to make it easier for subclasses to implement
       additional service calls by first checking whether the
       superclass can handle it. For example in a theoretical
       MyDispatcher subclass:
       \code
       bool MyDispatcher::Handle(...) {
         if (DirectoryDispatcher::Handle(...))
	   return true;
	 switch (request.code[0]) {
	   case 42:
	     life_the_universe_and_everything();
	     // should fill in reply accordingly...
	     return true;
	 }
	 return false;
       }
       \endcode
       This way, further subclassing of MyDispatcher allows to chain
       handlers from general to specific.
       
       \return false if the request.code[0] did not match any of the
       known tags, or true if it did or some other condition was
       handled by this implementation.
       
       \todo Remove redundancy between return value and the reply
       out-parameter, while keeping it easy for subclass implementers.
    */
    virtual bool Handle(Directory & directory,
			/** the request sent by the user */
			ServiceMessage const & request,
			/** the reply to be filled in by the implementation */
			ServiceMessage & reply);
  };
  
}

#endif // WBCRUN_DIRECTORY_HPP
