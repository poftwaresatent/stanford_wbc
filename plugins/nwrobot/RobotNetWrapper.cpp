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

#include "RobotNetWrapper.hpp"
#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <wbcnet/log.hpp>
#include <wbcnet/strutil.hpp>
#include <sys/time.h>

using namespace sfl;
using namespace std;

static wbcnet::logger_t logger(wbcnet::get_logger("nwrobot"));


namespace {
  
  using namespace wbcnet;
  
  class MessageHandler
    : public wbcnet::MdxHandler
  {
  public:
    MessageHandler(RobotNetWrapper * robot);
    
    virtual int DoHandleMessageHeader(unique_id_t msg_id, BufferAPI const & buf,
				      endian_mode_t endian_mode);
    virtual int DoHandleMessagePayload(unique_id_t msg_id, BufferAPI const & buf,
				       endian_mode_t endian_mode);
    
  private:
    RobotNetWrapper * m_robot;
  };
  
}


uint16_t RobotNetWrapper::
defaultPort()
{
  return 2365;
}


RobotNetWrapper::
RobotNetWrapper(bool server_mode,
		bool nonblocking,
		uint16_t port,
		std::string const & address)
  : m_server_mode(server_mode),
    m_nonblocking(nonblocking),
    m_port(port),
    m_address(address),
    m_robot_state(0),
    m_servo_command(0),
    m_muldex(-1, -1, wbcnet::ENDIAN_DETECT),
    m_channel(0)
{
  m_muldex.SetHandler(wbcnet::msg::ROBOT_STATE, new MessageHandler(this));
  m_muldex.SetHandler(wbcnet::msg::SERVO_COMMAND, new MessageHandler(this));
}


RobotNetWrapper::
~RobotNetWrapper()
{
  delete m_robot_state;
  delete m_servo_command;
  delete m_channel;
}


bool RobotNetWrapper::
lazyCreateChannel()
{
  if (m_channel) {
    return true;
  }
  
  m_channel = new wbcnet::TCPNetWrapper();
  if ( ! m_channel->Open(m_port, m_address, m_server_mode, m_nonblocking, 250000)) {
    LOG_ERROR (logger,
	       "RobotNetWrapper::lazyCreateChannel(): TCPNetWrapper::Open() failed.\n"
	       << "  port: " << (int) m_port << "\n"
	       << "  address: " << m_address << "\n"
	       << "  server_mode: " << (m_server_mode ? "true\n" : "false\n")
	       << "  nonblocking: " << (m_nonblocking ? "true\n" : "false\n"));
    delete m_channel;
    m_channel = 0;
    return false;
  }
  
  return true;
}


bool RobotNetWrapper::
readSensors(SAIVector & jointPositions, SAIVector & jointVelocities, timeval & acquisition_time,
	    SAIMatrix * opt_force)
{
  if ( ! lazyCreateChannel()) {
    return false;
  }
  
  wbcnet::muldex_status const ms(m_muldex.DemuxOne(m_channel));
  //// ...hardcoded to blocking communication for now...
  // if ((ms.muldex != wbcnet::muldex_status::SUCCESS)
  //     && (ms.muldex != wbcnet::muldex_status::TRY_AGAIN)) {
  if (ms.muldex != wbcnet::muldex_status::SUCCESS) {
    LOG_ERROR (logger,
	       "RobotNetWrapper::readSensors(): m_muldex.DemuxOne() said "
	       << wbcnet::muldex_status_str(ms));
    return false;
  }
  if ( ! m_robot_state) {
    LOG_ERROR (logger,
	       "RobotNetWrapper::readSensors(): no robot state after m_muldex.DemuxOne()\n"
	       "  ...are you sending more than one message type on this channel?");
    return false;
  }
  
  jointPositions = m_robot_state->jointAngles;
  jointVelocities = m_robot_state->jointVelocities;
  acquisition_time = m_robot_state->acquisitionTime;
  if (opt_force) {
    *opt_force = m_robot_state->forces;
  }
  
  return true;
}


bool RobotNetWrapper::
writeCommand(SAIVector const & command)
{
  if ( ! lazyCreateChannel()) {
    return false;
  }
  
  if ( ! m_servo_command) {
    if (std::numeric_limits<uint8_t>::max() < command.size()) {
      LOG_ERROR (logger,
		 "RobotNetWrapper::writeCommand(): command size " << command.size()
		 << " exceeds maximum of " << (int) std::numeric_limits<uint8_t>::max());
      return false;
    }
    m_servo_command = new wbc::msg::ServoCommand(true, static_cast<uint8_t>(command.size()));
  }
  m_servo_command->command = command;
  
  wbcnet::muldex_status const ms(m_muldex.Mux(m_channel, *m_servo_command));
  //// Just do blocking communication for now...
  // if ((ms.muldex != wbcnet::muldex_status::SUCCESS)
  //     && (ms.com != wbcnet::COM_TRY_AGAIN)) {
  if (ms.muldex != wbcnet::muldex_status::SUCCESS) {
    LOG_ERROR (logger,
	       "RobotNetWrapper::writeCommand(): muldex.Mux() said " << wbcnet::muldex_status_str(ms));
    return false;
  }
  
  return true;
}


bool RobotNetWrapper::
writeSensors(SAIVector const & jointAngles, SAIVector const & jointVelocities,
	     SAIMatrix const * opt_force)
{
  if ( ! lazyCreateChannel()) {
    return false;
  }
  
  if ( ! m_robot_state) {
    if ((std::numeric_limits<uint8_t>::max() < jointAngles.size())
	|| (std::numeric_limits<uint8_t>::max() < jointVelocities.size())
	|| (opt_force && (std::numeric_limits<uint8_t>::max() < opt_force->row()))
	|| (opt_force && (std::numeric_limits<uint8_t>::max() < opt_force->column()))) {
      LOG_ERROR (logger,
		 "RobotNetWrapper::writeSensors(): maximum size exceeded\n"
		 << "  maximum = " << (int) std::numeric_limits<uint8_t>::max() << "\n"
		 << "  jointAngles.size() = " << jointAngles.size() << "\n"
		 << "  jointVelocities.size() = " << jointVelocities.size() << "\n"
		 << "  opt_force->row() = " << (opt_force ? to_string(opt_force->row()) : string("<NULL>")) << "\n"
		 << "  opt_force->column() = " << (opt_force ? to_string(opt_force->column()) : string("<NULL>")));
      return false;
    }
    m_robot_state = new wbc::msg::RobotState(true,
					     static_cast<uint8_t>(jointAngles.size()),
					     static_cast<uint8_t>(jointVelocities.size()),
					     static_cast<uint8_t>(opt_force ? opt_force->row() : 0),
					     static_cast<uint8_t>(opt_force ? opt_force->column() : 0));
  }
  m_robot_state->jointAngles = jointAngles;
  m_robot_state->jointVelocities = jointVelocities;
  struct timeval now;
  if (0 != gettimeofday(&now, NULL)) {
    LOG_ERROR (logger, "RobotNetWrapper::writeSensors(): gettimeofday() failed");
    return false;
  }
  m_robot_state->acquisitionTime = now;
  if (opt_force) {
    m_robot_state->forces = *opt_force;
  }
  
  wbcnet::muldex_status const ms(m_muldex.Mux(m_channel, *m_robot_state));
  //// Just do blocking communication for now...
  // if ((ms.muldex != wbcnet::muldex_status::SUCCESS)
  //     && (ms.com != wbcnet::COM_TRY_AGAIN)) {
  if (ms.muldex != wbcnet::muldex_status::SUCCESS) {
    LOG_ERROR (logger,
	       "RobotNetWrapper::writeSensors(): muldex.Mux() said " << wbcnet::muldex_status_str(ms));
    return false;
  }
  
  return true;
}


bool RobotNetWrapper::
readCommand(SAIVector & command)
{
  if ( ! lazyCreateChannel()) {
    return false;
  }
  
  wbcnet::muldex_status const ms(m_muldex.DemuxOne(m_channel));
  //// ...hardcoded to blocking communication for now...
  // if ((ms.muldex != wbcnet::muldex_status::SUCCESS)
  //     && (ms.muldex != wbcnet::muldex_status::TRY_AGAIN)) {
  if (ms.muldex != wbcnet::muldex_status::SUCCESS) {
    LOG_ERROR (logger,
	       "RobotNetWrapper::readCommand(): m_muldex.DemuxOne() said "
	       << wbcnet::muldex_status_str(ms));
    return false;
  }
  if ( ! m_servo_command) {
    LOG_ERROR (logger,
	       "RobotNetWrapper::readCommand(): no servo command after m_muldex.DemuxOne()\n"
	       "  ...are you sending more than one message type on this channel?");
    return false;
  }
  
  command = m_servo_command->command;
  
  return true;
}


void RobotNetWrapper::
shutdown() const
{
}


RobotNetWrapper * FactoryNetWrapper::
parse(std::string const & spec, wbc::ServoInspector * servo_inspector)
{
  vector<string> token;
  sfl::tokenize(spec, ':', token);
  
  string mode_str("s");
  std::string address;
  uint16_t port(RobotNetWrapper::defaultPort());
  
  sfl::token_to(token, 0, mode_str);
  sfl::token_to(token, 1, address);
  sfl::token_to(token, 2, port);
  
  bool server_mode;
  bool nonblocking;
  if (("s" == mode_str) || ("S" == mode_str)) {
    server_mode = true;
    nonblocking = true;
  }
  else if (("bs" == mode_str) || ("BS" == mode_str)) {
    server_mode = true;
    nonblocking = false;
  }
  else if (("c" == mode_str) || ("C" == mode_str)) {
    server_mode = false;
    nonblocking = true;
  }
  else if (("bc" == mode_str) || ("BC" == mode_str)) {
    server_mode = false;
    nonblocking = false;
  }
  else {
    LOG_ERROR (logger, "FactoryNetWrapper::parse(): invalid mode string `" << mode_str << "' in spec `" << spec << "'");
    return 0;
  }
  
  LOG_INFO (logger,
	    "FactoryNetWrapper::parse(): creating RobotNetWrapper\n"
	    << "  server_mode: " << (server_mode ? "true\n" : "false\n")
	    << "  port: " << (int) port << "\n"
	    << "  address: " << address);
  return new RobotNetWrapper(server_mode, nonblocking, port, address);
}

  
void FactoryNetWrapper::
dumpHelp(std::string const & prefix, std::ostream & os) const
{
  os << prefix << "spec = [ mode [ : address [ : port ] ] ]\n"
     << prefix << "  default = s::" << (int) RobotNetWrapper::defaultPort() << "\n"
     << prefix << "  address is only relevant in client mode `c'\n"
     << prefix << "  you can say `bs' or `bc' for blocking mode\n";
}


namespace {
  
  
  MessageHandler::
  MessageHandler(RobotNetWrapper * robot)
    : MdxHandler(true, 0),
      m_robot(robot)
  {
  }
  
  int MessageHandler::
  DoHandleMessageHeader(unique_id_t msg_id, BufferAPI const & buf,
			endian_mode_t endian_mode)
  {
    Proxy * proxy(0);
    
    switch (msg_id) {
    case wbcnet::msg::ROBOT_STATE:
      if ( ! m_robot->m_robot_state)
	m_robot->m_robot_state = new wbc::msg::RobotState(true, 0, 0, 0, 0);
      proxy = m_robot->m_robot_state;
      break;
    case wbcnet::msg::SERVO_COMMAND:
      if ( ! m_robot->m_servo_command)
	m_robot->m_servo_command = new wbc::msg::ServoCommand(true, 0);
      proxy = m_robot->m_servo_command;
      break;
    default:
      LOG_ERROR (logger, "MessageHandler::DoHandleMessageHeader(): unexpected message id " << (int) msg_id);
      return 17;
    }
    
    if ( ! proxy) {
      LOG_ERROR (logger, "MessageHandler::DoHandleMessageHeader(): BUG! no proxy after switch");
      return 18;
    }
    
    proxy_status const ps(proxy->UnpackHeader(buf, endian_mode));
    if (PROXY_OK != ps) {
      LOG_ERROR (logger, "MessageHandler::DoHandleMessageHeader(): UnpackHeader() failed: " << proxy_status_str(ps));
      return 19;
    }
    
    return 0;
  }
  
  
  int MessageHandler::
  DoHandleMessagePayload(unique_id_t msg_id, BufferAPI const & buf, endian_mode_t endian_mode)
  {
    Proxy * proxy(0);
    switch (msg_id) {
    case wbcnet::msg::ROBOT_STATE:
      proxy = m_robot->m_robot_state;
      break;
    case wbcnet::msg::SERVO_COMMAND:
      proxy = m_robot->m_servo_command;
      break;
    default:
      LOG_ERROR (logger, "MessageHandler::DoHandleMessagePayload(): unexpected message id " << (int) msg_id);
      return 17;
    }
    
    if ( ! proxy) {
      LOG_ERROR (logger, "MessageHandler::DoHandleMessagePayload(): BUG! no proxy after switch");
      return 18;
    }
    
    proxy_status const ps(proxy->UnpackPayload(buf, endian_mode));
    if (PROXY_OK != ps) {
      LOG_ERROR (logger, "MessageHandler::DoHandleMessagePayload(): UnpackPayload() failed: " << proxy_status_str(ps));
      return 19;
    }
    
    return 0;
  }
  
}
