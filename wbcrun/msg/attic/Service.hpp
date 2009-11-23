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

#ifndef WBCRUN_MSG_SERVICE_HPP
#define WBCRUN_MSG_SERVICE_HPP

#include <wbcnet/msg/UserCommand.hpp>
#include <wbcnet/data.hpp>

namespace wbcrun {

  namespace srv {
    
    typedef wbcnet::Vector<int32_t> code_t;
    typedef wbcnet::Matrix<double> matrix_t;
    
    /**
       \note If you add a status, do not forget to update the
       implementation of result_to_string() and string_to_result().
    */
    typedef enum {
      SUCCESS,
      NOT_IMPLEMENTED,
      TRY_AGAIN,
      INVALID_DIMENSION,
      INVALID_BEHAVIOR_ID,
      INVALID_TASK_ID,
      INVALID_REQUEST,
      INVALID_CODE,
      MISSING_CODE,
      INVALID_DATA,
      OUT_OF_RANGE,
      OTHER_ERROR
      // If you add something to this enum, please *immediately* also
      // update result_to_string() and string_to_result().
    } result_t;
    
    /** \return The string representation according to the result_t
	enum, or "(invalid result)" if it lies outside that range. */
    char const * result_to_string(int result);
    
    /** \return -1 for invalid name */
    int string_to_result(std::string const & name);

  }
  
  namespace msg {
    
    class Service
      : public wbcnet::msg::UserCommand<srv::code_t, srv::matrix_t>
    {
    public:
      explicit Service(/** You will probably want to use
			   msg::USER_REQUEST or msg::USER_REPLY
			   depending on whether this is a request or a
			   reply message. */
		       wbcnet::unique_id_t id);
      
      /** Increments the requestID, initializes the sizes of the code
	  vector and matrix, and if non-empty fills them with
	  zeros. The string list is cleared out. */
      void InitRequest(uint8_t nCodes, uint8_t nRows, uint8_t nColumns);
      
      /** Copies the requestID from the given request, initializes the
	  code size to one and sets it to NOT_IMPLEMENTED, clears the
	  matrix and string list. */
      void InitReply(Service const & request);
    };
    
  }
  
}

#endif // WBCRUN_MSG_SERVICE_HPP
