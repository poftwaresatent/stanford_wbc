/*
 * Copyright (c) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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

#include "ServoCommand.hpp"
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));

namespace wbcnet {
  
  namespace msg {

    ServoCommandWrap::
    ServoCommandWrap(unique_id_t id,
		     uint8_t _ncommands,
		     VectorStorageAPI * _commandPtr)
      : Proxy(id),
	check_ncommands(_ncommands),
	ncommands(_ncommands),
	commandPtr(_commandPtr)
    {
      m_header.AddField(new ValuePack<uint8_t, fixed_count>
			("Command::ncommands", true, &ncommands, 1));
      m_header.AddField(new TimestampPack("Command::acquisitionTime", true, &acquisitionTime));
      // we have to use variable-sized semantics because
      //  a) we don't know what comes in over the net
      //  b) we might get uninitialized fooPtr and VectorPack, on
      //     fixed-size vectors, tries to access it at construction time.
      m_payload.AddField(new VectorPack("Command::command", true, false, commandPtr));
    }
  
  
    proxy_status ServoCommandWrap::
    CheckHeader() const
    {
      if (check_ncommands == ncommands)
	return PROXY_OK;
      LOG_ERROR (logger,
		     "wbcnet::ServoCommandWrap::CheckHeader(): ncommands is "
		     << static_cast<unsigned int>(ncommands)
		     << " but should be " << static_cast<unsigned int>(check_ncommands));
      return PROXY_DIMENSION_MISMATCH;
    }
    
  }
  
}
