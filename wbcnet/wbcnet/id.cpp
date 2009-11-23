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

/** \file id.cpp Implementation of IDLookup. */

#include "id.hpp"
#include "strutil.hpp"

namespace wbcnet {
  
  
  IDLookup::name_error::
  name_error(std::string const & name)
    : std::runtime_error("unknown or duplicate name \"" + name + "\"")
  {
  }
  
  
  IDLookup::id_error::
  id_error(unique_id_t id)
    : std::runtime_error("unknown or duplicate ID " + sfl::to_string<int>(id))
  {
  }
  
  
  IDLookup::
  IDLookup(bool _auto_assign)
    : auto_assign(_auto_assign)
  {
  }
  
  
  unique_id_t IDLookup::
  DoAssign(std::string const & name) const
  {
    unique_id_t id(0);
    if ( ! m_i2s.empty())
      id = m_i2s.rbegin()->first + 1;
// #warning 'Well, if someone ever assigns numeric_limits<unique_id_t>::max() somewhere, we will overflow here and probably overwrite an existing ID.'
    m_i2s.insert(make_pair(id, name));
    m_s2i.insert(make_pair(name, id));
    return id;
  }
  
  
  unique_id_t IDLookup::
  Assign(std::string const & name)
  {
    s2i_t::const_iterator ii(m_s2i.find(name));
    if (ii != m_s2i.end())
      return ii->second;
    return DoAssign(name);
  }
  
  
  void IDLookup::
  Add(std::string const & name, unique_id_t id) throw(name_error, id_error)
  {
    i2s_t::const_iterator ii(m_i2s.find(id));
    if ((ii != m_i2s.end()) && (ii->second != name))
      throw id_error(id);
    s2i_t::const_iterator jj(m_s2i.find(name));
    if ((jj != m_s2i.end()) && (jj->second != id))
      throw name_error(name);
    m_i2s.insert(make_pair(id, name));
    m_s2i.insert(make_pair(name, id));
  }
  
  
  unique_id_t IDLookup::
  GetID(std::string const & name) const throw(name_error)
  {
    s2i_t::const_iterator ii(m_s2i.find(name));
    if (ii != m_s2i.end())
      return ii->second;
    if ( ! auto_assign)
      throw name_error(name);
    return DoAssign(name);
  }
  
  
  std::string IDLookup::
  GetName(unique_id_t id) const throw(id_error)
  {
    i2s_t::const_iterator ii(m_i2s.find(id));
    if (ii == m_i2s.end())
      throw id_error(id);
    return ii->second;
  }
  
  
  void IDLookup::
  DumpByName(std::ostream & os, std::string const & prefix) const
  {
    for (s2i_t::const_iterator ii(m_s2i.begin()); ii != m_s2i.end(); ++ii)
      os << prefix << ii->first << "\t" << (int) ii->second << "\n";
  }
  
  
  void IDLookup::
  DumpByID(std::ostream & os, std::string const & prefix) const
  {
    for (i2s_t::const_iterator ii(m_i2s.begin()); ii != m_i2s.end(); ++ii)
      os << prefix << (int) ii->first << "\t" << ii->second << "\n";
  }
  
  
  namespace idl {
    
    static IDLookup * idl(0);
    
    
    not_initialized::
    not_initialized()
      : std::runtime_error("please call wbcnet::idl::CreateSingleton() or CreateAutoAssignSingleton() before using it")
    {
    }
    
    
    init_conflict::
    init_conflict()
      : std::runtime_error("cannot wbcnet::idl::CreateSingleton() and CreateAutoAssignSingleton()")
    {
    }
    
    
    void CreateSingleton() throw(init_conflict)
    {
      if ((0 != idl) && idl->auto_assign)
	throw init_conflict();
      if (0 == idl)
	idl = new IDLookup(false);
    }
    
    
    void CreateAutoAssignSingleton() throw(init_conflict)
    {
      if ((0 != idl) && ( ! idl->auto_assign))
	throw init_conflict();
      if (0 == idl)
	idl = new IDLookup(true);
    }
    
    
    void DestroySingleton()
    {
      delete idl;
      idl = 0;
    }
    
    
    unique_id_t Assign(std::string const & name) throw(not_initialized)
    {
      if (0 == idl)
	throw not_initialized();
      return idl->Assign(name);
    }
    
    
    void Add(std::string const & name, unique_id_t id)
      throw(not_initialized, IDLookup::name_error, IDLookup::id_error)
    {
      if (0 == idl)
	throw not_initialized();
      idl->Add(name, id);
    }
    
    
    unique_id_t GetID(std::string const & name) throw(not_initialized, IDLookup::name_error)
    {
      if (0 == idl)
	throw not_initialized();
      return idl->GetID(name);
    }
    
    
    std::string GetName(unique_id_t id) throw(not_initialized, IDLookup::id_error)
    {
      if (0 == idl)
	throw not_initialized();
      return idl->GetName(id);
    }
    
    
    void DumpByName(std::ostream & os, std::string const & prefix) throw(not_initialized)
    {
      if (0 == idl)
	throw not_initialized();
      idl->DumpByName(os, prefix);
    }
    
    
    void DumpByID(std::ostream & os, std::string const & prefix) throw(not_initialized)
    {
      if (0 == idl)
	throw not_initialized();
      idl->DumpByID(os, prefix);
    }
    
  }
  
}
