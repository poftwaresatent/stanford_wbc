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

#ifndef WBCNET_REGISTRY_HPP
#define WBCNET_REGISTRY_HPP

#include <string>
#include <map>
#include <stdexcept>

namespace wbcnet {
  
  /** Traits for Registry instances that should not delete their
      contents, e.g. they contain values or references, or pointers
      that get deleted elsewhere. */
  template<typename value_t>
  struct registry_trait_nodelete {
    static void deref(value_t & value) {}
  };
  
  /** Traits for Registry instances that should delete their contents,
      i.e. they own the things that they contain. */
  template<typename value_t>
  struct registry_trait_delete {
    static void deref(value_t & value) { delete value; }
  };
  
  
  /** Name-based registry of values or objects, where each name can
      only be assigned once. Use the second template argument to
      specify whether the Registry should delete the contents for you
      (this obviously only compiles if value_type is a pointer
      type). By default, it does NOT delete them. */
  template<typename value_type,
	   typename deref_trait_type = registry_trait_nodelete<value_type> >
  class Registry {
  public:
    typedef value_type value_t;
    typedef deref_trait_type deref_trait_t;
    
    virtual ~Registry() {
      for (iterator_t ii(m_registry.begin()); ii != m_registry.end(); ++ii)
	deref_trait_t::deref(ii->second);
    }
    
    bool Have(std::string const & name) const
    { return m_registry.find(name) != m_registry.end(); }
    
    void Add(std::string const & name, value_t value) throw(std::runtime_error) {
      if (m_registry.find(name) != m_registry.end())
	throw std::runtime_error("wbcnet::Registry::Add(): name \"" + name + "\" already taken");
      m_registry.insert(make_pair(name, value));
    }
    
    value_t Get(std::string const & name) const throw(std::runtime_error) {
      const_iterator_t ir(m_registry.find(name));
      if (m_registry.end() == ir)
	throw std::runtime_error("wbcnet::Registry::Get(): name \"" + name + "\" not registered");
      return ir->second;
    }
    
    template<typename ostream_t>
    void DumpNames(ostream_t & os, std::string const prefix) {
      for (iterator_t ii(m_registry.begin()); ii != m_registry.end(); ++ii)
	os << prefix << ii->first << "\n";
    }
    
  protected:
    typedef typename std::map<std::string, value_t> registry_t;
    typedef typename registry_t::iterator iterator_t;
    typedef typename registry_t::const_iterator const_iterator_t;
    
    registry_t m_registry;
  };
  
}

#endif // WBCNET_REGISTRY_HPP
