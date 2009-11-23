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

#ifndef WBCRUN_TEST_DIRECTORY_HPP
#define WBCRUN_TEST_DIRECTORY_HPP

#include <wbcrun/directory.hpp>

namespace wbcrun {
  
  class TestDirectory
    : public Directory
  {
  public:
    virtual srv::result_t HandleServoCmd(int requestID,
					 srv::vector_t const * code_in,
					 srv::matrix_t const * data_in,
					 srv::vector_t * code_out,
					 srv::matrix_t * data_out);
    
    virtual srv::result_t ListBehaviors(listing_t & behaviors) const;
    virtual srv::result_t ListBehaviorCmds(int behaviorID, request_list_t & requests) const;
    
    virtual srv::result_t HandleBehaviorCmd(int behaviorID,
					    int requestID,
					    srv::vector_t const * code_in,
					    srv::matrix_t const * data_in,
					    srv::vector_t * code_out,
					    srv::matrix_t * data_out);
    
    virtual srv::result_t ListTasks(int behaviorID, listing_t & tasks) const;
    virtual srv::result_t ListTaskCmds(int behaviorID, int taskID, request_list_t & requests) const;
    
    virtual srv::result_t HandleTaskCmd(int behaviorID,
					int taskID,
					int requestID,
					srv::vector_t const * code_in,
					srv::matrix_t const * data_in,
					srv::vector_t * code_out,
					srv::matrix_t * data_out);
  };
  
}

#endif // WBCRUN_TEST_DIRECTORY_HPP
