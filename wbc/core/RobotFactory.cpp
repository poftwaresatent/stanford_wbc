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

#include "RobotFactory.hpp"
#include "RobotAPI.hpp"
#include <wbcnet/strutil.hpp>
#include <wbcnet/log.hpp>
#include <iostream>
#include <map>
#include <typeinfo>
#include <stdlib.h>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));

using sfl::splitstring;
using namespace std;

namespace wbc {
  
  
  RobotFactory::
  ~RobotFactory()
  {
  }


  RobotAPI * RobotFactoryRegistry::
  parseCreate(string const & api_spec, ServoInspector * servo_inspector) const
    throw(std::runtime_error)
  {
    string name, spec;
    splitstring(api_spec, ':', name, spec);
    LOG_DEBUG (logger,
	       "wbc::RobotFactoryRegistry::parseCreate(" << api_spec
	       << "): name='" << name << "'  spec='" << spec << "'");
    RobotFactory * factory;
    try {
      factory = Get(name);
    }
    catch (std::runtime_error const & ee) {
      ostringstream msg;
      msg << "wbc::RobotFactoryRegistry::parseCreate(`" << api_spec << "', ...): EXCEPTION\n"
	  << "  available robots:\n";
      dumpAll("    ", msg);
      msg << "  error: " << ee.what();
      throw runtime_error(msg.str());
    }
    RobotAPI * robot(factory->parse(spec, servo_inspector));
    return robot;
  }
  
  
  BidirectionalRobotAPI * RobotFactoryRegistry::
  parseCreateBidirectional(std::string const & api_spec,
			   ServoInspector * servo_inspector) const
    throw(std::runtime_error)
  {
    RobotAPI * robot(parseCreate(api_spec, servo_inspector));
    if ( ! robot) {
      throw runtime_error("wbc::RobotFactoryRegistry::parseCreateBidirectional(`" + api_spec 
			  + "', ...): could not create robot");
    }
    BidirectionalRobotAPI * birobot(dynamic_cast<BidirectionalRobotAPI*>(robot));
    if ( ! birobot) {
      delete robot;
      throw runtime_error("wbc::RobotFactoryRegistry::parseCreateBidirectional(`" + api_spec 
			  + "', ...): robot is not bidirectional");
    }
    return birobot;
  }
  
  
  void RobotFactoryRegistry::
  dumpAll(string const & prefix, ostream & os) const
  {
    os << prefix << "help on available RobotAPI factories:";
    for (const_iterator_t ii(m_registry.begin()); ii != m_registry.end(); ++ii) {
      os << "\n" << prefix << "  " << ii->first << "\n";
      ii->second->dumpHelp(prefix + "  ", os);
    }
  }

}
