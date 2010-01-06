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

#ifndef WBCNET_MSG_SERVICE_HPP
#define WBCNET_MSG_SERVICE_HPP

#include <wbcnet/msg/UserCommand.hpp>

namespace wbcnet {
  
  
  /**
     List of domains for service requests and replies. The domain
     specifies which part of the controller the command is directed
     at. You can specify custom domains by using the values of
     SRV_OTHER_DOMAIN and upwards.
   */
  typedef enum {
    SRV_SERVO_DOMAIN,
    SRV_BEHAVIOR_DOMAIN,
    SRV_TASK_DOMAIN,
    SRV_OTHER_DOMAIN
  } srv_domain_t;
  
  
  /**
     \return A string representation according to the srv_domain_t
     enum. If the domain is SRV_OTHER_DOMAIN or above, the value of
     domain-SRV_OTHER_DOMAIN will be included in the string. If the
     domain is negative, the returned string will simply include that
     value.
  */
  std::string srv_domain_to_string(int domain);
  
  
  /**
     \return The inverse of srv_domain_to_string(), or -1 for an
     invalid name. No attempt is made to invert the SRV_OTHER_DOMAIN
     mechanism (except for an exact match). */
  int string_to_srv_domain(std::string const & name);
  
  
  /**
     List of command codes for service requests and replies. After the
     domain, the command specifies what exactly is asked of that
     particular domain. You can specify custom commands by using the
     values of SRV_OTHER_COMMAND and upwards.
   */
  typedef enum {
    SRV_KEY_PRESS,
    SRV_TOGGLE_RECORDER,
    SRV_GET_TASK_TYPE,
    SRV_GET_DIMENSION,
    SRV_GET_LINK_ANGLE,
    SRV_GET_LINK_TRANSFORM,
    SRV_SET_BEHAVIOR,
    SRV_GET_BEHAVIOR,
    SRV_SET_GOAL,
    SRV_GET_GOAL,
    SRV_GET_POSITIONS,
    SRV_GET_VELOCITIES,
    SRV_GET_TORQUES,
    SRV_SET_PROP_GAIN,
    SRV_GET_PROP_GAIN,
    SRV_SET_DIFF_GAIN,
    SRV_GET_DIFF_GAIN,
    SRV_SET_GAINS,
    SRV_GET_GAINS,
    SRV_SET_MAX_VEL,
    SRV_GET_MAX_VEL,
    SRV_SET_MAX_ACCEL,
    SRV_GET_MAX_ACCEL,
    SRV_GET_BEHAVIOR_LIST,
    SRV_GET_COMMAND_LIST,
    SRV_GET_TASK_LIST,
    SRV_ACTIVATE,
    SRV_FLOAT,
    SRV_OTHER_COMMAND
  } srv_command_t;
  
  
  /**
     \return A string representation according to the srv_command_t
     enum. If the command is SRV_OTHER_COMMAND or above, the value of
     command-SRV_OTHER_COMMAND will be included in the string. If the
     command is negative, the returned string will simply include that
     value.
  */
  std::string srv_command_to_string(int command);
  
  
  /**
     \return The inverse of srv_command_to_string(), or -1 for an
     invalid name. No attempt is made to invert the SRV_OTHER_COMMAND
     mechanism (except for an exact match). */
  int string_to_srv_command(std::string const & name);
  
  
  /**
     Application specific service result codes. Zero stands for
     success, and the small values are reserved for some standard
     error codes. You can send arbitrary codes, of course, but you can
     (and should) add the value of SRV_OTHER_ERROR to it, then
     srv_result_to_string() will create a custom error message.
  */
  typedef enum {
    SRV_SUCCESS,
    SRV_NOT_IMPLEMENTED,
    SRV_TRY_AGAIN,
    SRV_INVALID_DIMENSION,
    SRV_INVALID_BEHAVIOR_ID,
    SRV_INVALID_TASK_ID,
    SRV_INVALID_COMMAND,
    SRV_INVALID_CODE,
    SRV_MISSING_CODE,
    SRV_INVALID_DATA,
    SRV_OUT_OF_RANGE,
    SRV_OTHER_ERROR
  } srv_result_t;
  
  
  /**
     \return A string representation according to the srv_result_t
     enum. If the result is SRV_OTHER_ERROR or above, the value of
     result-SRV_OTHER_ERROR will be included in the string. If the
     result is negative, the returned string will simply include that
     value.
  */
  std::string srv_result_to_string(int result);
  
  
  /**
     \return The inverse of srv_result_to_string(), or -1 for an
     invalid name. No attempt is made to invert the SRV_OTHER_ERROR
     mechanism (except for an exact match). */
  int string_to_srv_result(std::string const & name);
  
  
  typedef Vector<int32_t> srv_code_t;
  typedef Matrix<double> srv_matrix_t;
  
  
  namespace msg {
    
    
    /**
       A "service" message contains a request from a user to the
       controller, or a reply from the controller to the user. It
       provides some standard result codes and uses the default
       (storage-only) implementation for the vector and matrix types.
    */
    class Service
      : public UserCommand< srv_code_t, srv_matrix_t >
    {
    public:
      explicit Service(/** You will probably want to use
			   msg::USER_REQUEST or msg::USER_REPLY
			   depending on whether this is a request or a
			   reply message. */
		       unique_id_t id);
      
      /** Copies the requestID from the given request (because replies
	  should have the same request ID as the message that they are
	  answering to), initializes the code size to one and sets it
	  to NOT_IMPLEMENTED, clears the matrix and string list. */
      void InitReply(Service const & request);
      
      /** Increments the requestID, initializes the sizes of the code
	  vector and matrix, and if non-empty fills them with
	  zeros. The string list is cleared out. */
      void InitRequest(uint8_t nCodes, uint8_t nRows, uint8_t nColumns);
      
      void InitListBehaviors();
      
      void InitListBehaviorCmds(int behaviorID);
      
      void InitServoCmd(int commandID,
			srv_code_t const * code_in,
			srv_matrix_t const * data_in);
      
      void InitBehaviorCmd(int behaviorID,
			   int commandID,
			   srv_code_t const * code_in,
			   srv_matrix_t const * data_in);
      
      void InitListTasks(int behaviorID);
      
      void InitListTaskCmds(int behaviorID,
			    int taskID);
      
      void InitTaskCmd(int behaviorID,
		       int taskID,
		       int commandID,
		       srv_code_t const * code_in,
		       srv_matrix_t const * data_in);
      
      void InitSetBehavior(int behaviorID);
      
      void InitSetGoal(double const * goal_coordinates, size_t n_coordinates);
      void InitSetGains(double const * gains, size_t n_gains);
  
      void InitGetPositions();
      void InitGetVelocities();
      void InitGetTorques();
      void InitFloat();
      void InitActivate();
      
      void InitGetLinkTransform(int linkID);
      void InitGetLinkTransform(std::string const & linkName);
      
      void InitToggleRecorder();
  
      void InitKeyPress(/** Key press commands have been moved to the
			    SRV_BEHAVIOR_DOMAIN. They used to be
			    domain-less and apply to the current
			    behavior, so you did not need to specify a
			    behavior ID. In order to support that old
			    behavior, you can specify -1 as behavior
			    ID, which will (should) end up retrieveing
			    the current behavior and sending it that
			    key code. */
			int behaviorID,
			int32_t keycode);
    };
    
  }
  
}

#endif // WBCNET_MSG_SERVICE_HPP
