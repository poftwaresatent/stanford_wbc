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

#ifndef WBC_ROBOT_FACTORY_HPP
#define WBC_ROBOT_FACTORY_HPP

#include <wbcrun/Registry.hpp>
#include <string>
#include <iosfwd>

namespace wbc {
  
  class RobotAPI;
  class BidirectionalRobotAPI;
  class ServoInspector;
  
  /**
     This allows us to more easily parse command-line arguments into
     RobotAPI instances without the need for a lot of <code>ifdef</code>
     directives or so.
  */
  class RobotFactory
  {
  public:
    virtual ~RobotFactory();
    
    /**
       \return A newly constructed subclass of RobotAPI, based on the
       spec string. You can return NULL to signal failure, for example
       if the spec is invalid.
    */
    virtual RobotAPI * parse(std::string const & spec,
			     ServoInspector * servo_inspector) = 0;
    
    /**
       Write some help about your subclass of RobotAPI, especially how
       the spec string gets interpreted by your implementation of
       RobotFactory::parse(). The output of 'servo -H' is essentially
       a concatenation of help messages from all registered
       RobotFactory subclasses, so please stick to the layout of the
       others. For example, the SAI2 robot API implements this as
       follows:
       \code
       void FactorySAI2::
       dumpHelp(std::string const & prefix, std::ostream & os) const
       {
       os << prefix << "spec = [ handshake [ : endian [ : port [ : address ]]]]\n"
       << prefix << "  default = s:x:10401:127.0.0.1\n"
       << prefix << "  handshake is s (server) or c (client)\n"
       << prefix << "  endian    is a (auto) or x (skip all swaps)\n"
       << prefix << "  port      is the TCP port number to use\n"
       << prefix << "  address   is the IP address (in dot-notation) of the server\n";
       }
       \endcode
    */
    virtual void dumpHelp(std::string const & prefix,
			  std::ostream & os) const = 0;
  };
  
  
  typedef wbcrun::Registry<RobotFactory *,
			   wbcrun::registry_trait_delete<RobotFactory *> >
  RobotFactoryRegistrySuper;
  
  /**
     \note Use the superclass' Add() to register RobotFactory
     instances with this registry. The default constructor adds a
     couple of builtin factories.
  */
  class RobotFactoryRegistry
    : public RobotFactoryRegistrySuper
  {
  public:
    /**
       Subdivides api_spec into "name:rest", uses "name" to find a
       Factory, and calls Factory::parse("rest"). If there is no ':' in
       the api_spec, it is taken as the name.
       
       \note Throws an exception if something goes wrong.
       
       \return A new RobotAPI instance if the factory resolution
       worked and the Factory::parse() call succeeded. Otherwise, an
       exception is thrown.
    */
    RobotAPI * parseCreate(std::string const & api_spec,
			   ServoInspector * servo_inspector) const
      throw(std::runtime_error);
  
    /**
       Like parseCreate(), but also attempts to dynamic_cast to
       BidirectionalRobotAPI. If that fails, the created robot is
       deleted and the method returns zero.
    */
    BidirectionalRobotAPI *
    parseCreateBidirectional(std::string const & api_spec, 
			     ServoInspector * servo_inspector) const
      throw(std::runtime_error);
    
    /**
       Calls RobotFactory::dumpHelp() on all registered instances.
    */
    void dumpAll(std::string const & prefix, std::ostream & os) const;
  };
  
}

#endif // WBC_ROBOT_FACTORY_HPP
