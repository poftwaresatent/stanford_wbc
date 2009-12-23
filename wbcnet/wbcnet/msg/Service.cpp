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
#include <limits>
#include <sstream>

using namespace std;


static uint8_t const msz(std::numeric_limits<uint8_t>::max());

static char const * srv_result_name[] = {
  "SRV_SUCCESS",
  "SRV_NOT_IMPLEMENTED",
  "SRV_TRY_AGAIN",
  "SRV_INVALID_DIMENSION",
  "SRV_INVALID_BEHAVIOR_ID",
  "SRV_INVALID_TASK_ID",
  "SRV_INVALID_REQUEST",
  "SRV_INVALID_CODE",
  "SRV_MISSING_CODE",
  "SRV_INVALID_DATA",
  "SRV_OUT_OF_RANGE",
  "SRV_OTHER_ERROR"
};


namespace wbcnet {
  
  
  string srv_result_to_string(int result)
  {
    if (0 > result) {
      ostringstream os;
      os << "(negative result: " << result << ")";
      return os.str();
    }
    // note: SRV_OTHER_ERROR==result gets handled using the srv_result_name table
    if (SRV_OTHER_ERROR < result) {
      ostringstream os;
      os << "SRV_OTHER_ERROR #" << result - SRV_OTHER_ERROR;
      return os.str();
    }
    return srv_result_name[result];
  }
  
  
  int string_to_srv_result(std::string const & name)
  {
    static map<string, int> nm;
    if (nm.empty())
      for (int ii(0); ii <= SRV_OTHER_ERROR; ++ii)
	nm[srv_result_name[ii]] = ii;
    map<string, int>::const_iterator inm(nm.find(name));
    if (nm.end() == inm)
      return -1;
    return inm->second;
  }
  
  
  namespace msg {
    
    
    Service::
    Service(unique_id_t id)
      : UserCommand< Vector<int32_t>, Matrix<double> >(id,
						       0, 0, 0, // initial sizes
						       msz, msz, msz // max sizes
						       )
    {
    }
    
    
    void Service::
    Reset()
    {
      // somewhat redundant in most cases
      requestID = 0;
      nCodes = 1;
      nRows = 0;
      nColumns = 0;
      code.SetNElements(1);
      if (code.NElements() > 0)
	code[0] = SRV_NOT_IMPLEMENTED;
      matrixptr->SetSize(0, 0);
    }
    
  }
  
}
