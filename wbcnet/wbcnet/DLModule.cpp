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

#ifdef DISABLE_PLUGINS
# error 'Plugins are DISABLED, refusing to even attempt to build dynamic loader support.'
#endif // DISABLE_PLUGINS

#include "DLModule.hpp"
#include <wbcnet/log.hpp>
#include <sstream>
#include <dlfcn.h>

static wbcnet::logger_t logger(wbcnet::get_logger("wbcrun"));

using namespace std;

namespace wbcrun {
  
  
  DLModule::
  DLModule()
    : m_dl(0),
      m_module(0)
  {
  }
  
  
  DLModule::
  ~DLModule()
  {
    delete m_module;
    if (m_dl)
      dlclose(m_dl);
  }
  
  
  Module * DLModule::
  Load(std::string const & filename,
       bool immediate) throw(std::runtime_error)
  {
    if (m_dl) {
      ostringstream os;
      os << "wbcrun::DLModule::Load(" << filename << "): already loaded another file";
      throw runtime_error(os.str());
    }

    int flag;
    if (immediate)
      flag = RTLD_NOW;
    else
      flag = RTLD_LAZY;
    
    m_dl = dlopen(filename.c_str(), flag);
    if ( ! m_dl) {
      ostringstream os;
      os << "wbcrun::DLModule::Load(" << filename << "): dlopen() failed: " << dlerror();
      throw runtime_error(os.str());
    }
    
    LOG_TRACE (logger, "wbcrun::DLModule::Load(" << filename << "): dlopen() OK");
    
    dlerror();
    wbcrun_create_module_t create_module;
    create_module = (wbcrun_create_module_t) dlsym(m_dl, "wbcrun_create_module");
    char const * dlsym_error(dlerror());
    if (dlsym_error) {
      ostringstream os;
      os << "wbcrun::DLModule::Load(" << filename << "): dlsym() failed: " << dlsym_error;
      throw runtime_error(os.str());
    }
    
    LOG_TRACE (logger, "wbcrun::DLModule::Load(" << filename << "): dlsym() OK");
    
    m_module = create_module();
    if ( ! m_module) {
      dlclose(m_dl);
      m_dl = 0;
      ostringstream os;
      os << "wbcrun::DLModule::Load(" << filename << "): create_module() failed";
      throw runtime_error(os.str());
    }
    
    LOG_TRACE (logger, "wbcrun::DLModule::Load(" << filename << "): create_module() OK");
    
    return m_module;
  }
  
  
  DLModule * ModuleRegistry::
  LoadModule(std::string const & name,
	     std::string const & path) throw(std::runtime_error)
  {
    if (Have(name)) {
      ostringstream os;
      os << "wbcrun::ModuleRegistry::LoadModule(): name \"" << name << "\" already taken";
      throw runtime_error(os.str());
    }
    
    DLModule * dl(new DLModule());
    try {
      dl->Load(path, false);
      Add(name, dl);
      LOG_TRACE (logger,
		     "wbcrun::ModuleRegistry::LoadModule(" << name << ", " << path << ") OK");
      return dl;
    }
    catch (runtime_error const & ee) {
      delete dl;
      throw ee;
    }
    catch (exception const & ee) {
      delete dl;
      throw runtime_error("wbcrun::ModuleRegistry::LoadModule(" + name
			  + "): unexpected exception: " + ee.what());
    }
  }
  
}
