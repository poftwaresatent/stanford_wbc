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

#ifndef WBCRUN_FACTORY_HPP
#define WBCRUN_FACTORY_HPP

#include <wbcrun/Registry.hpp>

namespace wbcrun {
  
  template<typename root_t>
  class FactoryAPI {
  public:
    virtual ~FactoryAPI() {}
    virtual root_t * Create() = 0;
  };
  
  
  template<typename leaf_t, typename root_t>
  class Factory : public FactoryAPI<root_t> {
  public:
    virtual leaf_t * Create() { return new leaf_t(); }
  };
  
  
  template<typename value_t>
  class FactoryRegistry
    : public Registry<FactoryAPI<value_t>*,
		      registry_trait_delete<FactoryAPI<value_t>*> >
  {
  public:
    virtual ~FactoryRegistry() {}
    
    value_t * Create(std::string const & name) const throw(std::runtime_error)
    { return this->Get(name)->Create(); }
  };
  
}

#endif // WBCRUN_FACTORY_HPP
