/* 
 * Copyright (C) 2008 Roland Philippsen <roland dot philippsen at gmx dot net>
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

#ifndef WBCNET_SPQ_NET_CONFIG_HPP
#define WBCNET_SPQ_NET_CONFIG_HPP

#include <wbcnet/NetConfig.hpp>

namespace wbcnet {
  
  class SPQueue;
  
  class SPQNetConfig
    : public NetConfig
  {
  public:
    typedef NetConfig::process_t process_t;
    
    SPQNetConfig();
    
    virtual wbcnet::Channel * CreateChannel(process_t from_process,
					    process_t to_process) const
      throw(std::runtime_error);
    
    wbcnet::SPQueue * CreateSPQueue(process_t from_process,
				    process_t to_process) const;
    
    wbcnet::SPQueue * GetSPQueue(process_t from_process,
				 process_t to_process) const;
    
  protected:
    typedef std::map<process_t, wbcnet::SPQueue*> to_t;
    typedef std::map<process_t, to_t> from_to_t;
    
    mutable from_to_t m_from_to;
  };
  
}

#endif // WBCNET_SPQ_NET_CONFIG_HPP
