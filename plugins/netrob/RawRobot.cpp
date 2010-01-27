/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
   \file plugins/netrob/RawRobot.cpp
   \author Roland Philippsen
*/

#include "RawRobot.hpp"
#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <wbcnet/log.hpp>
#include <wbcnet/strutil.hpp>
#include <sys/time.h>

using namespace sfl;
using namespace std;

static wbcnet::logger_t logger(wbcnet::get_logger("netrob"));


namespace netrob {
  
  
  RawRobot::
  RawRobot(size_t npos, size_t nvel, size_t ncom, 
	   wbcnet::SockWrap * sock_wrap)
    : m_state(npos, nvel),
      m_command(ncom),
      m_sock_wrap(sock_wrap)
  {
    m_tick.tv_sec = 0;
    m_tick.tv_usec = 0;
  }
  
  
  RawRobot::
  ~RawRobot()
  {
    delete m_sock_wrap;
  }
  
  
  bool RawRobot::
  readSensors(SAIVector & jointPositions, SAIVector & jointVelocities, timeval & acquisition_time,
	      SAIMatrix * opt_force)
  {
    jointPositions.setSize(m_state.npos);
    jointVelocities.setSize(m_state.nvel);
    if (opt_force) {
      opt_force->setSize(0, 0);
    }
    
    if ( ! m_sock_wrap->ResizeSourceBuffer(m_state.buffer.GetSize())) {
      LOG_ERROR (logger,
		 "netrob::RawRobot::readSensors(): m_sock_wrap->ResizeSourceBuffer("
		 << m_state.buffer.GetSize() << ") failed");
      return false;
    }
    
    wbcnet::com_status cs(m_sock_wrap->Receive(m_state.buffer));
    for (/* nop */; wbcnet::COM_TRY_AGAIN == cs; cs = m_sock_wrap->Receive(m_state.buffer)) {
      usleep(10000);
    }
    if (wbcnet::COM_OK != cs) {
      LOG_ERROR (logger,
		 "netrob::RawRobot::readSensors(): m_sock_wrap->Receive() failed with "
		 << wbcnet::com_status_str(cs));
      return false;
    }
    
    for (size_t ii(0); ii < m_state.npos; ++ii) {
      jointPositions[ii] = m_state.joint_positions[ii];
    }
    
    for (size_t ii(0); ii < m_state.nvel; ++ii) {
      jointVelocities[ii] = m_state.joint_velocities[ii];
    }
    
    acquisition_time = m_tick;
    m_tick.tv_usec += 1000000 / 60; // hardcode 60Hz for now...
    if (m_tick.tv_usec >= 1000000) {
      m_tick.tv_usec = 0;
      ++ m_tick.tv_sec;
    }
    
    return true;
  }
  
  
  bool RawRobot::
  writeCommand(SAIVector const & command)
  {
    if (command.size() != m_command.ncom) {
      LOG_ERROR (logger,
		 "netrob::RawRobot::writeCommand(): size mismatch, should be " << m_command.ncom
		 << " but is " << command.size());
      return false;
    }
    
    for (size_t ii(0); ii < m_command.ncom; ++ii) {
      m_command.joint_torques[ii] = command[ii];
    }
    
    wbcnet::com_status cs(m_sock_wrap->Send(m_command.buffer));
    for (/* nop */; wbcnet::COM_TRY_AGAIN == cs; cs = m_sock_wrap->Send(m_command.buffer)) {
      usleep(10000);
    }
    if (wbcnet::COM_OK != cs) {
      LOG_ERROR (logger,
		 "netrob::RawRobot::writeCommand(): m_sock_wrap->Send() failed with "
		 << wbcnet::com_status_str(cs));
      return false;
    }
    
    return true;
  }
  
  
  bool RawRobot::
  writeSensors(SAIVector const & jointAngles, SAIVector const & jointVelocities,
	       SAIMatrix const * opt_force)
  {
    if ((jointAngles.size() != m_state.npos) || (jointVelocities.size() != m_state.nvel)) {
      LOG_ERROR (logger,
		 "netrob::RawRobot::writeSensors(): size mismatch, should be [" << m_state.npos
		 << ", " << m_state.nvel << " but is [" << jointAngles.size() << ", " << jointVelocities.size()
		 << "]");
      return false;
    }
    
    
    for (size_t ii(0); ii < m_state.npos; ++ii) {
      m_state.joint_positions[ii] = jointAngles[ii];
    }
    
    for (size_t ii(0); ii < m_state.nvel; ++ii) {
      m_state.joint_velocities[ii] = jointVelocities[ii];
    }
    
    wbcnet::com_status cs(m_sock_wrap->Send(m_state.buffer));
    for (/* nop */; wbcnet::COM_TRY_AGAIN == cs; cs = m_sock_wrap->Send(m_state.buffer)) {
      usleep(10000);
    }
    if (wbcnet::COM_OK != cs) {
      LOG_ERROR (logger,
		 "netrob::RawRobot::writeSensors(): m_sock_wrap->Send() failed with "
		 << wbcnet::com_status_str(cs));
      return false;
    }
    
    return true;
  }
  
  
  bool RawRobot::
  readCommand(SAIVector & command)
  {
    command.setSize(m_command.ncom);
    
    if ( ! m_sock_wrap->ResizeSourceBuffer(m_command.buffer.GetSize())) {
      LOG_ERROR (logger,
		 "netrob::RawRobot::readCommand(): m_sock_wrap->ResizeSourceBuffer("
		 << m_command.buffer.GetSize() << ") failed");
      return false;
    }
    
    wbcnet::com_status cs(m_sock_wrap->Receive(m_command.buffer));
    for (/* nop */; wbcnet::COM_TRY_AGAIN == cs; cs = m_sock_wrap->Receive(m_command.buffer)) {
      usleep(10000);
    }
    if (wbcnet::COM_OK != cs) {
      LOG_ERROR (logger,
		 "netrob::RawRobot::readCommand(): m_sock_wrap->Receive() failed with "
		 << wbcnet::com_status_str(cs));
      return false;
    }
    
    for (size_t ii(0); ii < m_command.ncom; ++ii) {
      command[ii] = m_command.joint_torques[ii];
    }
    
    return true;
  }
  
  
  void RawRobot::
  shutdown() const
  {
  }
  
  
  RawRobot * RawFactory::
  parse(std::string const & spec, wbc::ServoInspector * servo_inspector)
  {
    vector<string> token;
    sfl::tokenize(spec, ':', token);
    string mode("s");
    sfl::token_to(token, 0, mode);
    in_port_t port(9955);
    sfl::token_to(token, 1, port);
    string address("*");
    if ("c" == mode) {
      address = "127.0.0.1";
    }
    sfl::token_to(token, 2, address);
    static bool const is_nonblocking(false);
    
    wbcnet::SockWrap * sock_wrap(0);
    
    if ("s" == mode) {
      wbcnet::SoServer * sos(new wbcnet::SoServer(0, -1, true));
      if ( ! sos->Open(port, is_nonblocking, address)) {
	delete sos;
	LOG_ERROR (logger,
		   "netrob::RawFactory::parse(" << spec << ", ...): sos->Open(" << port
		   << ", " << sfl::to_string(is_nonblocking) << ", " << address << ") failed");
	return 0;
      }
      if ( ! sos->BindListen(1)) {
	delete sos; 
	LOG_ERROR (logger,
		   "netrob::RawFactory::parse(" << spec << ", ...): sos->BindListen(1) failed");
	return 0;
      }
      cout << "netrob::RawFactory::parse(" << spec << ", ...): accepting " << flush;
      wbcnet::com_status cs(sos->Accept());
      while (wbcnet::COM_TRY_AGAIN == cs) {
	cout << "." << flush;
	usleep(250000);
	cs = sos->Accept();
      }
      if (wbcnet::COM_OK != cs) {
	delete sos;
	LOG_ERROR (logger,
		   "netrob::RawFactory::parse(" << spec << ", ...): sos->Accept() failed with "
		   << wbcnet::com_status_str(cs));
	return 0;
      }
      cout << "OK\n";
      sock_wrap = sos;
    }
    
    else if ("c" == mode) {
      wbcnet::SoClient * soc(new wbcnet::SoClient(0, -1, true));
      if ( ! soc->Open(port, is_nonblocking, address)) {
	delete soc;
	LOG_ERROR (logger,
		   "netrob::RawFactory::parse(" << spec << ", ...): soc->Open(" << port
		   << ", " << sfl::to_string(is_nonblocking) << ", " << address << ") failed");
	return 0;
      }
      cout << "netrob::RawFactory::parse(" << spec << ", ...): connecting " << flush;
      wbcnet::com_status cs(soc->Connect());
      while ((wbcnet::COM_TRY_AGAIN == cs) || (wbcnet::COM_OTHER_ERROR == cs)) {
	if (wbcnet::COM_TRY_AGAIN == cs)
	  cout << "." << flush;
	else
	  cout << "x" << flush;      
	usleep(250000);
	cs = soc->Connect();
      }
      if (wbcnet::COM_OK != cs) {
	delete soc;
	LOG_ERROR (logger,
		   "netrob::RawFactory::parse(" << spec << ", ...): soc->Connect() failed with "
		   << wbcnet::com_status_str(cs));
	return 0;
      }
      cout << "OK\n";
      sock_wrap = soc;
    }
    
    if ( ! sock_wrap) {
	LOG_ERROR (logger,
		   "netrob::RawFactory::parse(" << spec << ", ...): neither server nor client mode selected");
	return 0;
    }
    
    return new RawRobot(6, 6, 6, sock_wrap); // XXXX make the sizes configurable one day...
  }
  
  
  void RawFactory::
  dumpHelp(std::string const & prefix, std::ostream & os) const
  {
    os << prefix << "spec = [ mode [ : port [ : address ] ] ]\n"
       << prefix << "  default = s:9955:*\n"
       << prefix << "  (for client mode, the default address is 127.0.0.1)\n";
  }
  
}
