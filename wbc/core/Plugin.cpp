/*
 * Stanford_WBC_Extension -- plugin library for stanford-wbc.sourceforge.net
 *
 * Copyright (C) 2008, 2009 Stanford University
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#include "Plugin.hpp"
#include "RobotFactory.hpp"
#include "TaskModelFactory.hpp"
#include "RawControllerAPI.hpp"
#include <wbc/core/BehaviorFactory.hpp>
#include <wbc/core/ServoBehaviorsAPI.hpp>
#include <wbcnet/strutil.hpp>
#include <wbcnet/log.hpp>

#ifndef DISABLE_PLUGINS
#include <stdlib.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <string.h>
#endif // DISABLE_PLUGINS

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));

using namespace std;

namespace wbc {

#ifdef DISABLE_PLUGINS
//# warning 'Plugins are DISABLED, skipping PluginRegistry support.'
#else // DISABLE_PLUGINS
  
  // ...all PluginRegistry method definitions...
  
  PluginRegistry::
  PluginRegistry(Extensions & extensions)
    : m_extensions(extensions)
  {
#ifdef WBC_PLUGIN_PATH_STR
    m_search_directories.push_back(WBC_PLUGIN_PATH_STR);
#endif // WBC_PLUGIN_PATH_STR
    
    if (getenv("WBC_PLUGIN_PATH")) {
      listing_t env_tok;
      sfl::tokenize(getenv("WBC_PLUGIN_PATH"), ':', env_tok);
      for (listing_t::const_iterator it(env_tok.begin()); it != env_tok.end(); ++it)
	if ( ! it->empty())
	  m_search_directories.push_back(*it);
    }
    
    if (getenv("HOME"))
      m_search_directories.push_back(string(getenv("HOME")) + "/.wbc");
    
    if (logger->isDebugEnabled()) {
      ostringstream msg;
      msg << "wbc::PluginRegistry: search directories:";
      for (listing_t::const_iterator it(m_search_directories.begin());
	   it != m_search_directories.end(); ++it)
	msg << "\n  " << *it;
      LOG_DEBUG (logger, msg.str());
    }
  }
  
  
  void PluginRegistry::
  LoadPluginFile(std::string const & name, std::string const & path) throw(std::runtime_error)
  {
    wbcrun::DLModule * dl(LoadModule(name, path));
    // in case this fails we're stuck with a bad entry in the module
    // registry, but well... no support for unloading them at the
    // moment
    dl->Get<Plugin>()->Init(m_extensions);
  }
  
  
  void PluginRegistry::
  LoadPlugin(std::string const & name) throw(std::runtime_error)
  {
    if (Have(name))
      throw runtime_error("wbc::PluginRegistry::LoadPlugin(" + name + "): name already taken");
    
    // accumulate a nice message in case we fail
    ostringstream excos;
    excos << "wbc::PluginRegistry::LoadPlugin(" << name << "): no matching plugin found";
    
    for (listing_t::const_iterator dir(m_search_directories.begin());
	 dir != m_search_directories.end(); ++dir) {
      listing_t path;
      path.push_back(*dir + "/" + name);
      path.push_back(*dir + "/" + name + ".so");
      path.push_back(*dir + "/lib" + name + ".so");
      for (listing_t::const_iterator pp(path.begin()); pp != path.end(); ++pp) {
	try {
	  LoadPluginFile(name, *pp);
	  return;
	}
	catch (runtime_error const & ee) {
	  excos << "\n  tried " << *pp << ": " << ee.what();
	}
      }
    }
    
    // if we arrive here it means we did not succeed...
    throw runtime_error(excos.str());
  }
  
  
  size_t PluginRegistry::
  SearchAndLoadPlugins()
  {
    LOG_DEBUG (logger, "wbc::PluginRegistry::SearchAndLoadPlugins()");
    
    size_t count(0);
    for (listing_t::const_iterator dir(m_search_directories.begin());
	 dir != m_search_directories.end(); ++dir) {
      
      LOG_DEBUG (logger, "  directory: " << *dir);
      
      DIR * dirp(opendir(dir->c_str()));
      if ( ! dirp) {
	LOG_WARN (logger,
		       "wbc::PluginRegistry::SearchAndLoadPlugins(): opendir("  << *dir
		       << "): " << strerror(errno));
	continue;
      }
      
      for (struct dirent * dent(readdir(dirp)); 0 != dent; dent = readdir(dirp)) {
	std::string const name(dent->d_name);
	LOG_DEBUG (logger, "  entry: \"" << name << "\"");
	// XXXX simplistic matching: has to start with "lib"
	if (0 != strncmp("lib", name.c_str(), 3)) {
	  LOG_DEBUG (logger, "  entry does not start with \"lib\"");
	  continue;
	}
	try {
	  LoadPluginFile(name, *dir + "/" + name);
	  ++count;
	}
	catch (runtime_error const & ee) {
	  LOG_ERROR (logger,
			 "wbc::PluginRegistry::SearchAndLoadPlugins(): EXCEPTION\n"
			 << "  LoadPluginFile(" << name << ", " << *dir << "/" << name
			 << ") failed with\n" << "  " << ee.what());
	}
      }
      
      closedir(dirp);
    }
    
    return count;
  }
  
#endif // DISABLE_PLUGINS
  
  
  Extensions::
  Extensions(RobotControlModel * robmodel)
    : robot_registry(new RobotFactoryRegistry()),
      behavior_registry(new BehaviorFactoryRegistry(robmodel)),
      task_model_registry(new TaskModelFactoryRegistry()),
      servo_behaviors_registry(new ServoBehaviorsFactoryRegistry()),
      raw_controller_registry(new RawControllerFactoryRegistry())
  {
  }
  
  
  Extensions::
  ~Extensions()
  {
    delete robot_registry;
    delete behavior_registry;
    delete task_model_registry;
    delete servo_behaviors_registry;
  }
  
  
  void Extensions::
  AddRobot(std::string const & name,
	   RobotFactory * factory) throw(std::runtime_error)
  {
    LOG_DEBUG (logger, "wbc::Extensions::AddRobot(): " << name);
    robot_registry->Add(name, factory);
  }


  void Extensions::
  AddBehavior(std::string const & name,
	      BehaviorFactoryAPI * factory) throw(std::runtime_error)
  {
    LOG_DEBUG (logger, "wbc::Extensions::AddBehavior(): " << name);
    behavior_registry->Add(name, factory);
  }
  
  
  void Extensions::
  AddTaskModel(std::string const & name,
	       TaskModelFactoryAPI * factory)
    throw(std::runtime_error)
  {
    LOG_DEBUG (logger, "wbc::Extensions::AddTaskmodel(): " << name);
    task_model_registry->Add(name, factory);
  }
  
  
  void Extensions::
  AddServoBehaviors(std::string const & name,
		    wbcrun::FactoryAPI<ServoBehaviorsAPI> * factory)
    throw(std::runtime_error)
  {
    LOG_DEBUG (logger, "wbc::Extensions::AddServoBehaviors(): " << name);
    servo_behaviors_registry->Add(name, factory);
  }
  
  
  void Extensions::
  AddRawController(std::string const & name,
		   wbcrun::FactoryAPI<RawControllerAPI> * factory)
    throw(std::runtime_error)
  {
    LOG_DEBUG (logger, "wbc::Extensions::AddRawController(): " << name);
    raw_controller_registry->Add(name, factory);
  }
  
}
