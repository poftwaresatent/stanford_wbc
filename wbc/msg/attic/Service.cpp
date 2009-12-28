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

#include "Service.hpp"

static uint8_t const msz(std::numeric_limits<uint8_t>::max());

namespace {
  
  static char const * result_name[] = {
    "SUCCESS",
    "NOT_IMPLEMENTED",
    "TRY_AGAIN",
    "INVALID_DIMENSION",
    "INVALID_BEHAVIOR_ID",
    "INVALID_TASK_ID",
    "INVALID_REQUEST",
    "INVALID_CODE",
    "MISSING_CODE",
    "INVALID_DATA",
    "OUT_OF_RANGE",
    "OTHER_ERROR"
  };
  
}

namespace wbcrun {
  
  namespace srv {
    
    char const * srv_result_to_string(int result)
    {
      if ((0 > result) || (OTHER_ERROR < result))
	return "(invalid result)";
      return result_name[result];
    }
    
    
    int string_to_result(std::string const & name)
    {
      static std::map<std::string, int> nm;
      if (nm.empty())
	for (int ii(0); ii <= OTHER_ERROR; ++ii)
	  nm[result_name[ii]] = ii;
      std::map<std::string, int>::const_iterator inm(nm.find(name));
      if (nm.end() == inm)
	return -1;
      return inm->second;
    }
    
  }
  
  namespace msg {
    
    Service::
    Service(wbcnet::unique_id_t id)
      : wbcnet::msg::UserCommand<srv::code_t, srv::matrix_t>(id,
							     0, 0, 0, // initial sizes
							     msz, msz, msz // max sizes
							     )
    {
    }
    
    
    void Service::
    InitRequest(uint8_t _nCodes, uint8_t _nRows, uint8_t _nColumns)
    {
      ++requestID;
      clear();
      nCodes = _nCodes;
      nRows = _nRows;
      nColumns = _nColumns;
      code.SetNElements(_nCodes);
      code.SetZero();
      matrix.SetSize(_nRows, _nColumns);
      matrix.SetZero();
    }
    
    
    void Service::
    InitReply(Service const & request)
    {
      requestID = request.requestID;
      clear();
      nCodes = 1;
      nRows = 0;
      nColumns = 0;
      code.SetNElements(1);
      code[0] = SRV_NOT_IMPLEMENTED;
      matrix.SetSize(0, 0);
      matrix.SetZero();
    }
    
  }
  
}
