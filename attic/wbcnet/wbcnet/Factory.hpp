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

#ifndef WBCNET_FACTORY_HPP
#define WBCNET_FACTORY_HPP

#include <wbcnet/Registry.hpp>

namespace wbcnet {
  
  
  /**
     A factory is something that has a Create() method. The type of
     thing returned (or at least a superclass of it) must be known at
     compile time.
  */
  template<typename root_t>
  class FactoryAPI {
  public:
    virtual ~FactoryAPI() {}
    virtual root_t * Create() = 0;
  };
  
  
  /**
     Specialization of FactoryAPI for any default-constructible
     type. In order to support runtime polymorphism at compile time,
     you have to pass two types to this template: the type of the
     superclass in \c root_t and the type of the subclass in \c
     leaf_t.
  */
  template<typename leaf_t, typename root_t>
  class Factory : public FactoryAPI<root_t> {
  public:
    virtual leaf_t * Create() { return new leaf_t(); }
  };
  
  
  /**
     A factory registry allows you to create subclass instances by
     name. It is implemented here as a collection of named
     factories. Use the Registry::Add() method to register
     factories. In this instantiation, the pointers passed to
     Registry::Add() get deleted for you when the registry is
     destroyed.
  */
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

#endif // WBCNET_FACTORY_HPP
