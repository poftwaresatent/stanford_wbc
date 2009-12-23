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
    SRV_INVALID_REQUEST,
    SRV_INVALID_CODE,
    SRV_MISSING_CODE,
    SRV_INVALID_DATA,
    SRV_OUT_OF_RANGE,
    SRV_OTHER_ERROR
    // If you add something to this enum, please *immediately* also
    // update result_to_string() and string_to_result().
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
  
  
  namespace msg {
    
    /**
       A "service" message contains a request from a user to the
       controller, or a reply from the controller to the user. It
       provides some standard result codes and uses the default
       (storage-only) implementation for the vector and matrix types.
    */
    class Service
      : public UserCommand< wbcnet::Vector<int32_t>, wbcnet::Matrix<double> >
    {
    public:
      explicit Service(/** You will probably want to use
			   msg::USER_REQUEST or msg::USER_REPLY
			   depending on whether this is a request or a
			   reply message. */
		       unique_id_t id);
      
      /** Clears out everything and sets the code vector to one
	  element with the value SRV_NOT_IMPLEMENTED (see srv_result_t
	  below). */
      void Reset();
    };
    
  }
  
}

#endif // WBCNET_MSG_SERVICE_HPP
