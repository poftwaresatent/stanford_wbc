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

/** \file id.hpp Simple name-to-ID and back dictionary (manage proxy and command ID). */

#ifndef WBCNET_ID_HPP
#define WBCNET_ID_HPP

#include <stdexcept>
#include <map>
#include <string>
#ifdef WIN32
#include "wbcnet/win32_compat.hpp"
#else
#include <stdint.h>
#endif
namespace wbcnet {
  
  
  typedef uint16_t unique_id_t;
  
  
  class IDLookup
  {
  public:
    struct name_error: public std::runtime_error { name_error(std::string const & name); };
    struct id_error: public std::runtime_error { id_error(unique_id_t id); };
    
    explicit IDLookup(bool auto_assign);
    
    unique_id_t Assign(std::string const & name);
    
    void Add(std::string const & name, unique_id_t id) throw(name_error, id_error);
    
    unique_id_t GetID(std::string const & name) const throw(name_error);
    
    std::string GetName(unique_id_t id) const throw(id_error);
    
    void DumpByName(std::ostream & os, std::string const & prefix) const;
    
    void DumpByID(std::ostream & os, std::string const & prefix) const;

    bool const auto_assign;
    
  protected:
    typedef std::map<unique_id_t, std::string> i2s_t;
    typedef std::map<std::string, unique_id_t> s2i_t;
    
    unique_id_t DoAssign(std::string const & name) const;
    
    mutable i2s_t m_i2s;
    mutable s2i_t m_s2i;
  };
  
  
  namespace idl {
    
    struct not_initialized: public std::runtime_error { not_initialized(); };
    struct init_conflict: public std::runtime_error { init_conflict(); };
    
    void CreateSingleton() throw(init_conflict);
    
    void CreateAutoAssignSingleton() throw(init_conflict);
    
    void DestroySingleton();
    
    unique_id_t Assign(std::string const & name) throw(not_initialized);
    
    void Add(std::string const & name, unique_id_t id)
      throw(not_initialized, IDLookup::name_error, IDLookup::id_error);
    
    unique_id_t GetID(std::string const & name) throw(not_initialized, IDLookup::name_error);
    
    std::string GetName(unique_id_t id) throw(not_initialized, IDLookup::id_error);

    void DumpByName(std::ostream & os, std::string const & prefix) throw(not_initialized);
    
    void DumpByID(std::ostream & os, std::string const & prefix) throw(not_initialized);
    
  }
  
}

#endif // WBCNET_ID_HPP
