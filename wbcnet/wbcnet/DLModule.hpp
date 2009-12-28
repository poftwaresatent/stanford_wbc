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

#ifndef WBCRUN_DL_MODULE_HPP
#define WBCRUN_DL_MODULE_HPP

#include <wbcrun/Registry.hpp>
#include <string>
#include <iosfwd>

namespace wbcrun {
  
  class Module {
  public:
    virtual ~Module() {}
  };
  
}

#ifdef DISABLE_PLUGINS
//# warning 'Plugins are DISABLED, skipping all dynamic loader support.'
#else // DISABLE_PLUGINS

// ...rest of the file...

extern "C" {
  typedef wbcrun::Module * (*wbcrun_create_module_t)();
  wbcrun::Module * wbcrun_create_module();
}


namespace wbcrun {
  
  class DLModule
  {
  public:
    DLModule();
    virtual ~DLModule();
    
    /** The returned module will be deleted by the destructor of DLModule. */
    Module * Load(std::string const & filename,
		  bool immediate) throw(std::runtime_error);
    
    template<class ModuleSubclass>
    ModuleSubclass * Get() const throw(std::runtime_error) {
      if ( ! m_module)
	throw std::runtime_error("wbcrun::DLModule::Get(): no module loaded");
      ModuleSubclass * ms(dynamic_cast<ModuleSubclass *>(m_module));
      if ( ! ms)
	throw std::runtime_error("wbcrun::DLModule::Get(): invalid module subclass");
      return ms;
    }
    
  private:
    void * m_dl;
    Module * m_module;
  };
  
  
  typedef wbcrun::Registry<DLModule *, registry_trait_delete<DLModule *> > ModuleRegistrySuper;
  
  class ModuleRegistry
    : protected ModuleRegistrySuper {
  public:
    DLModule * LoadModule(std::string const & name,
			  std::string const & path) throw(std::runtime_error);
    
    ModuleRegistrySuper::Have;
    
    template<class ModuleSubclass>
    ModuleSubclass * Get(std::string const & name) const throw(std::runtime_error) {
      return ModuleRegistrySuper::Get(name)->Get<ModuleSubclass>();
      registry_t::const_iterator ir(m_registry.find(name));
      if (m_registry.end() == ir)
	throw std::runtime_error("wbcrun::FactoryRegistry::Create(): name \"" + name
				 + "\" is not in registry");
      return ir->second->Get<ModuleSubclass>();
    }
  };
  
}

#endif // DISABLE_PLUGINS
#endif // WBCRUN_DL_MODULE_HPP
