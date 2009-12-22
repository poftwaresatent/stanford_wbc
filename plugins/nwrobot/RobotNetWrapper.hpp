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
   \author Roland Philippsen
*/

#ifndef HAVE_NETWRAP
# error Cannot compile this unless you have libnetwrapper.
#endif // HAVE_NETWRAP

#ifndef ROBOT_NET_WRAPPER_HPP
#define ROBOT_NET_WRAPPER_HPP

#include <wbc/core/RobotAPI.hpp>
#include <wbc/core/RobotFactory.hpp>
#include <wbcrun/msg/RobotState.hpp>
#include <wbcrun/msg/ServoCommand.hpp>
#include <wbcnet/Muldex.hpp>
#include <wbcnet/com.hpp>
#include <wbcnet/NetWrapperWrap.hpp>

class RobotNetWrapper
  : public wbc::BidirectionalRobotAPI
{
public:
  static uint16_t defaultPort();
  
  RobotNetWrapper(bool server_mode,
		  bool nonblocking,
		  uint16_t port,
		  /** only used for client mode */
		  std::string const & address);
  
  virtual ~RobotNetWrapper();
  
  virtual bool readSensors(SAIVector & jointAngles, SAIVector & jointVelocities,
			   timeval & acquisition_time, SAIMatrix * opt_force);
  virtual bool writeCommand(SAIVector const & command);
  virtual void shutdown() const;
  
  virtual bool writeSensors(SAIVector const & jointAngles, SAIVector const & jointVelocities,
			    SAIMatrix const * opt_force);
  virtual bool readCommand(SAIVector & command);

  // We don't need protected or private, because it's used through the
  // abstract superclass only unless people know exactly what they're
  // doing anyway. Thus, keep it simple and allow anonymous helper
  // classes direct access to our fields.
  
  bool lazyCreateChannel();
  
  bool m_server_mode;
  bool m_nonblocking;
  uint16_t m_port;
  std::string m_address;
  wbcrun::msg::RobotState * m_robot_state;
  wbcrun::msg::ServoCommand * m_servo_command;
  wbcnet::MdxDispatcher m_muldex;
  wbcnet::TCPNetWrapper * m_channel;
};


struct FactoryNetWrapper
  : public wbc::RobotFactory
{
  virtual RobotNetWrapper * parse(std::string const & spec, wbc::ServoInspector * servo_inspector);
  virtual void dumpHelp(std::string const & prefix, std::ostream & os) const;
};

#endif // ROBOT_NET_WRAPPER_HPP
