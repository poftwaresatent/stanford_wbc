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

#ifndef WBC_PLUGIN_HPP
#define WBC_PLUGIN_HPP

#include <wbcnet/DLModule.hpp>
#include <wbcnet/Factory.hpp>
#include <list>

namespace wbc {
  
  class RobotFactory;
  class BehaviorFactoryAPI;
  class TaskModelFactoryAPI;
  class ServoBehaviorsAPI;
  class RawControllerAPI;
  
  class RobotFactoryRegistry;
  class BehaviorFactoryRegistry;
  class TaskModelFactoryRegistry;
  class ServoBehaviorsFactoryRegistry;
  class RawControllerFactoryRegistry;
  
  class RobotControlModel;
  
  
  /**
     Collection of the various entities that people can hook into the
     whole-body controller. The typical approach is to register a
     bunch of factory objects, that can then be retrieved by string
     lookup. Having one collection makes it easier to maintain all
     existing plugins, at least when adding methods here.

     \note You should REALLY think twice before removing methods, or
     changing their signatures: you can make a lot of people annoyed
     at the change.
   */
  class Extensions
  {
  public:
    explicit Extensions(RobotControlModel * robmodel);
    virtual ~Extensions();
    
    /** Register a robot factory. Transfers ownership of the factory:
	the caller must NOT delete the pointer. */
    void AddRobot(std::string const & name,
		  RobotFactory * factory) throw(std::runtime_error);

    /** Register a behavior factory. Transfers ownership of the factory:
	the caller must NOT delete the pointer.
	
	You can directly use a BehaviorFactory<> if your behavior
	subclass takes no constructor arguments.
    */
    void AddBehavior(std::string const & name,
		     BehaviorFactoryAPI * factory) throw(std::runtime_error);

    /** Register a task model factory. Transfers ownership of the
	factory: the caller must NOT delete the pointer.
	
	In order to use this, create a subclass of TaskModelFactoryAPI
	which returns a new instance of your subclass of TaskModelAPI.
    */
    void AddTaskModel(std::string const & name,
		      TaskModelFactoryAPI * factory)
      throw(std::runtime_error);

    /** Register a servo behaviors factory. Transfers ownership of the
	factory: the caller must NOT delete the pointer.
	
	You can directly use a ServoBehaviorsFactory<> if your
	subclass takes no constructor arguments.
    */
    void AddServoBehaviors(std::string const & name,
			   wbcnet::FactoryAPI<ServoBehaviorsAPI> * factory)
      throw(std::runtime_error);
    
    void AddRawController(std::string const & name,
			  wbcnet::FactoryAPI<RawControllerAPI> * factory)
      throw(std::runtime_error);
    
    RobotFactoryRegistry * robot_registry;
    BehaviorFactoryRegistry * behavior_registry;
    TaskModelFactoryRegistry * task_model_registry;
    ServoBehaviorsFactoryRegistry * servo_behaviors_registry;
    RawControllerFactoryRegistry * raw_controller_registry;
  };
  
  
  /**
     The class you have to specialize when writing a whole-body
     control plugin. This is a bit historically grown, a simple
     function with C binding would probably do the trick just as well,
     but anyway. Just call the various methods of the Extensions
     object passed to your plugin.
   */
  class Plugin : public wbcnet::Module {
  public:
    virtual void Init(Extensions & extensions) throw(std::runtime_error) = 0;
  };
  

#ifdef DISABLE_PLUGINS
//# warning 'Plugins are DISABLED, skipping PluginRegistry support.'
#else // DISABLE_PLUGINS
  
  /**
     Utility for loading plugin files. This is also where the plugin
     path search "intelligence" (haha!) lives.
   */
  class PluginRegistry
    : protected wbcnet::ModuleRegistry
  {
  public:
    explicit PluginRegistry(Extensions & extensions);
    
    void LoadPluginFile(std::string const & name, std::string const & path)
      throw(std::runtime_error);
    
    void LoadPlugin(std::string const & name) throw(std::runtime_error);
    
    /** \return the number of successfully loaded plugins */
    size_t SearchAndLoadPlugins();
    
  protected:
    typedef std::list<std::string> listing_t;
    
    listing_t m_search_directories;
    Extensions & m_extensions;
  };

#endif // DISABLE_PLUGINS
  
}

#endif // WBC_PLUGIN_HPP
