/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2009 Stanford University. All rights reserved.
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

/**
   \file ServoBehaviorsAPI.hpp
   \author Roland Philippsen
*/

#ifndef WBC_SERVO_BEHAVIORS_API_HPP
#define WBC_SERVO_BEHAVIORS_API_HPP

#include <wbcnet/Factory.hpp>

class SAIVector;

namespace wbcnet {
  class TaskModelAPI;
}

namespace wbc {
  
  class ServoBehaviorsAPI;
  class BehaviorDescription;
  class RobotControlModel;
  class Time;  
  
  
  /**
     Utility for creating ServoBehaviorsAPI subclass-factories on the
     fly. If your ServoBehaviorsAPI subclass takes no constructor
     arguments, you can instantiate a factory for it just by declaring
     ServoBehaviorsFactory<YourServoBehaviorsAPISubclass>.
     
     If you need a ServoBehaviorsAPI subclass that takes constructor
     arguments, you can create a corresponding factory as follows:
     \code
     class SB: public wbc::ServoBehaviorsAPI {
     public:
       explicit SB(int bar);
       ...
     };
     
     class SBFactory: public ServoBehaviorsFactory<SB> {
     public:
       explicit SBFactory(int _bar): bar(_bar) {}
       virtual SB * Create() { return new SB(bar); }
       int bar;
     };
     
     ...
     wbc::ServoBehaviorsFactoryRegistry reg;
     reg.Add("SB42", new SBFactory(42));
     \endcode
     
     \see ServoBehaviorsFactoryRegistry documentation for further examples.
  */
  template<class ServoBehaviorSubclass>
  class ServoBehaviorsFactory
    : public wbcnet::Factory<ServoBehaviorSubclass, ServoBehaviorsAPI>
  {};
  
  
  /**
     How to register and use factories for your ServoBehaviorsAPI subclasses:
     \code
     class SB1: public wbc::ServoBehaviorsAPI { ... };
     class SB2: public wbc::ServoBehaviorsAPI { ... };
     ...
     wbc::ServoBehaviorsFactoryRegistry reg;
     reg.Add("1", new wbc::ServoBehaviorsFactory<SB1>());
     reg.Add("2", new wbc::ServoBehaviorsFactory<SB2>());
     ...
     wbc::ServoBehaviorsAPI * sb(reg.Create("1"));
     ...
     delete sb;
     \endcode
  */
  class ServoBehaviorsFactoryRegistry
    : public wbcnet::FactoryRegistry<ServoBehaviorsAPI>
  {};
  
  
  class ServoBehaviorsAPI
  {
  public:
    virtual ~ServoBehaviorsAPI() {}
    
    /** \todo \c behavior and \c robmodel should be const pointers,
	but that requires some wider refactorings first. */
    virtual bool updateTorques( /** in  */ BehaviorDescription * behavior,
				/** in  */ RobotControlModel * robmodel,
				/** in  */ wbcnet::TaskModelAPI const * taskModel,
				/** out */ SAIVector & generalizedTorques) = 0;
    
    virtual void record( Time const & now ) = 0;
  };
  
}

#endif // WBC_SERVO_BEHAVIORS_API_HPP
