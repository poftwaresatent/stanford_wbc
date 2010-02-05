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

#ifndef WBCNET_DL_MODULE_HPP
#define WBCNET_DL_MODULE_HPP

#include <wbcnet/Registry.hpp>
#include <string>
#include <iosfwd>

namespace wbcnet {
  
  /**
     A module is just am empty shell... you can subclass it however
     you like. We just need some sort of base class for the DLModule
     and ModuleRegistry mechanisms.
  */
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
  
  /**
     Plugin implementers have to provide this function. It allows the
     wbcnet::DLModule loading mechanism to create an instance of your
     plugin. This function has C language binding in order to avoid
     issues due to different C++ name mangling schemes.
  */
  wbcnet::Module * wbcnet_create_module();
  
}


namespace wbcnet {
  
  
  /**
     A dynamically loadable module, implementing wbcnet's plugin
     support infrastructure. By writing a shared library that provides
     a wbcnet_create_module() function (which has to have C language
     binding), you can load your Module subclass using the Load()
     method.
     
     \note The DLModule class wraps a single Module subclass. In order
     to get a whole collection of loadable modules, use a
     ModuleRegistry.
  */
  class DLModule
  {
  public:
    DLModule();
    
    /**
       Deletes the loaded Module for you.
    */
    virtual ~DLModule();
    
    /**
       Attempts to load a plugin file and instantiate the Module
       therein.  The returned module will be deleted by the destructor
       of DLModule. Errors are handled by throwing an exception
       (e.g. no such file, or no wbcnet_create_module() in the shared
       library).
    */
    Module * Load(/** The path to the plugin. */
		  std::string const & filename,
		  /** Whether to use immediate or deferred symbol
		      lookup. If in doubt, say "false" here. */
		  bool immediate) throw(std::runtime_error);
    
    /**
       Dynamically casts the loaded Module to the given
       subclass. Throws an exception if either there is no module, or
       it has the wrong type.
    */
    template<class ModuleSubclass>
    ModuleSubclass * Get() const throw(std::runtime_error) {
      if ( ! m_module)
	throw std::runtime_error("wbcnet::DLModule::Get(): no module loaded");
      ModuleSubclass * ms(dynamic_cast<ModuleSubclass *>(m_module));
      if ( ! ms)
	throw std::runtime_error("wbcnet::DLModule::Get(): invalid module subclass");
      return ms;
    }
    
  private:
    void * m_dl;
    Module * m_module;
  };
  
  
  /**
     Typedef to tidy up the declaration of ModuleRegistry. We want
     our ModuleRegistry to delete its pointers when it gets
     destroyed.
  */
  typedef wbcnet::Registry<DLModule *, registry_trait_delete<DLModule *> > ModuleRegistrySuper;
  
  
  /**
     A collection of DLModule (and thus Module) instances which can be
     accessed via name-based lookup. Example usage can be found in
     wbc::Plugin, which implements a search of various filesystem
     locations in order to instantiate plugins.
  */
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
	throw std::runtime_error("wbcnet::FactoryRegistry::Create(): name \"" + name
				 + "\" is not in registry");
      return ir->second->Get<ModuleSubclass>();
    }
  };
  
}

#endif // DISABLE_PLUGINS
#endif // WBCNET_DL_MODULE_HPP
