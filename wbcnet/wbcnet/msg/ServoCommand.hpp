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

#ifndef WBCNET_MSG_SERVO_COMMAND_HPP
#define WBCNET_MSG_SERVO_COMMAND_HPP

#include <wbcnet/proxy.hpp>

namespace wbcnet {
  
  namespace msg {
    
    /**
       A Proxy for sending and receiving command vectors (e.g. generalized torques).
    */
    class ServoCommandWrap
      : public Proxy
    {
    public:
      ServoCommandWrap(unique_id_t id,
		       uint8_t ncommands,
		       VectorStorageAPI * commandPtr);
    
      /** Yell if check_ncommands != ncommands. hipCoordinates are always
	  supposed to be 6 DOF, but this is not verified. */
      virtual proxy_status CheckHeader() const;
    
      // for CheckHeader()
      uint8_t const check_ncommands;
    
      // header
      uint8_t ncommands;
      timestamp acquisitionTime;
    
      // wrapped payload
      VectorStorageAPI * commandPtr;
    };
  
  
    /**
       Semi-generic serializable command vector container.
    */
    template<typename vector_t>
    class ServoCommand
      : public ServoCommandWrap
    {
    public:
      typedef vector_t vector_type;
      
      ServoCommand(unique_id_t id,
		   uint8_t ncommands)
	: ServoCommandWrap(id, ncommands, &command),
	  command(ncommands)
      {
      }
      
      
      bool operator == (ServoCommand const & rhs) const
      {
	return (acquisitionTime == rhs.acquisitionTime)
	  &&   (command == rhs.command);
      }
      
      bool operator != (ServoCommand const & rhs) const
      { return ! (*this == rhs); }
      
      template<typename ostream_t>
      void display(ostream_t & os, char const * prefix) const
      {
	os << prefix << "ncommands: " << ncommands << "\n"
	   << prefix << "time: " << acquisitionTime.tv_sec << "s "
	   << acquisitionTime.tv_usec << "usec\n";
	for (int ii(0); ii < ncommands; ++ii)
	  os << prefix << "command[" << ii << "]: " << command[ii] << "\n";
      }
      
      // real payload
      vector_t command;
    };
    
  }
  
}

#endif // WBCNET_MSG_SERVO_COMMAND_HPP
