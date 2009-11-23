/*
 * Copyright (c) 2009 Stanford University
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
   \file attributes.cpp
   \author Roland Philippsen
*/

#include "attributes.hpp"
#include <wbc/core/Plugin.hpp>
#include <wbc/parse/BehaviorParser.hpp>
#include <wbc/core/RobotFactory.hpp>
#include <wbc/core/BehaviorFactory.hpp>
#include <wbc/core/BehaviorDescription.hpp>
#include <wbc/parse/TiXmlBRParser.hpp>
#include <wbc/parse/OsimBRParser.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/MobileManipulatorTaskModel.hpp>
#include <wbc/core/MobileManipulatorServoBehaviors.hpp>
#include <wbc/bin/builtin.hpp>
#include <wbc/bin/options.hpp>
#include <wbcnet/TaskAtomizer.hpp>
#include <wbcnet/log.hpp>
#include <wbcnet/NetConfig.hpp>

#include <sstream>
#include <limits>

#include <err.h>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>


static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));


namespace wbc {


  attributes::
  attributes()
    : extensions(0),
      netcfg(0),
      robmodel(0),
      behavior(0)
  {
  }
  
  
  attributes::
  ~attributes()
  {
    for (size_t ii(0); ii < behavior.size(); ++ii)
      delete behavior[ii];
    delete robmodel;
    delete netcfg;
    delete extensions;
  }
  
  
  attributes * attributes::
  create(options const & opt) throw(std::runtime_error)
  {
    attributes * attr(new attributes());
    try {
#ifdef DISABLE_NETWORKING
# warning 'Networking is DISABLED, skipping over networking support code.'
      attr->netcfg = new wbcrun::SPQueueNetConfig();
#else // DISABLE_NETWORKING
      attr->netcfg = wbcnet::NetConfig::Create(opt.communication);
#endif // DISABLE_NETWORKING
      attr->robmodel = new RobotControlModel(BRParser::parse(opt.brparser_type, opt.xml_filename));
      LOG_INFO (logger, "loading extensions...");
      attr->extensions = load_extensions(attr->robmodel);
      LOG_INFO (logger, "adding humanoid stuff...");
      BehaviorParser::StdParse(opt.xml_filename, attr->behavior, *attr->extensions->behavior_registry);
      if (attr->behavior.empty()) {
	throw runtime_error("no (valid) behavior declarations found in `" + opt.xml_filename + "'");
      }
    }
    catch (std::runtime_error const & ee) {
      delete attr;
      throw ee;
    }
    
    return attr;
  }
  
}
