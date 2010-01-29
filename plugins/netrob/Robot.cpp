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
   \file plugins/netrob/Robot.cpp
   \author Roland Philippsen
*/

#include "Robot.hpp"
#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <wbcnet/log.hpp>
#include <wbcnet/strutil.hpp>
#include <sys/time.h>

using namespace sfl;
using namespace std;

static wbcnet::logger_t logger(wbcnet::get_logger("netrob"));


namespace netrob {
  
  using namespace wbcnet;
  
  class MessageHandler
    : public wbcnet::MdxHandler
  {
  public:
    MessageHandler(Robot * robot);
    
    virtual int DoHandleMessageHeader(unique_id_t msg_id, BufferAPI const & buf,
				      endian_mode_t endian_mode);
    virtual int DoHandleMessagePayload(unique_id_t msg_id, BufferAPI const & buf,
				       endian_mode_t endian_mode);
    
  private:
    Robot * m_robot;
  };
  
  
  Robot::
  Robot(wbcnet::NetConfig * net_config,
	std::string const & channel_spec,
	bool blocking)
    : m_blocking(blocking),
      m_robot_state(0),
      m_servo_command(0),
      m_muldex(-1, -1, wbcnet::ENDIAN_DETECT),
      m_net_config(net_config),
      m_channel_spec(channel_spec),
      m_channel(0),
      m_send_ok(false),
      m_receive_ok(false)
  {
    m_muldex.SetHandler(wbcnet::msg::ROBOT_STATE, new MessageHandler(this));
    m_muldex.SetHandler(wbcnet::msg::SERVO_COMMAND, new MessageHandler(this));
  }
  
  
  Robot::
  ~Robot()
  {
    delete m_robot_state;
    delete m_servo_command;
    delete m_net_config;
    delete m_channel;
  }
  
  
  bool Robot::
  lazyCreateChannel()
  {
    if (m_channel) {
      return true;
    }
    
    try {
      m_channel = m_net_config->CreateChannel(m_channel_spec);
    }
    catch (std::runtime_error const & ee) {
      LOG_ERROR (logger,
		 "netrob::Robot::lazyCreateChannel(): CreateChannel(" << m_channel_spec
		 << ") EXCEPTION " << ee.what());
      return false;
    }
    
    return true;
  }
  
  
  bool Robot::
  readSensors(SAIVector & jointPositions, SAIVector & jointVelocities, timeval & acquisition_time,
	      SAIMatrix * opt_force)
  {
    if ( ! lazyCreateChannel()) {
      return false;
    }
    
    wbcnet::muldex_status const ms(m_muldex.DemuxOne(m_channel));
    
    if (ms.muldex == wbcnet::muldex_status::SUCCESS) {
      m_receive_ok = true;
    }
    else {
      m_receive_ok = false;
      if (m_blocking) {
	LOG_ERROR (logger,
		   "netrob::Robot::readSensors(): m_muldex.DemuxOne() said "
		   << wbcnet::muldex_status_str(ms) << " (in BLOCKING mode)");
	return false;
      }
      if (ms.com == wbcnet::COM_TRY_AGAIN) {
	LOG_DEBUG (logger,
		  "netrob::Robot::readSensors(): m_muldex.DemuxOne() said "
		  << wbcnet::muldex_status_str(ms) << " (which is OK because we are in NON-blocking mode)");
	// keep working after the else block
      }
      else {
	LOG_ERROR (logger,
		   "netrob::Robot::readSensors(): m_muldex.DemuxOne() said "
		   << wbcnet::muldex_status_str(ms) << " (in NON-blocking mode)");
	return false;
      }
    }
    
    if ( ! m_robot_state) {
      if (m_blocking) {
	LOG_ERROR (logger, "netrob::Robot::readSensors(): no robot state (in BLOCKING mode)");
	return false;
      }
      // This'll only work if the params already have the correct
      // size... which they might or might not, depending on whether
      // the caller has used the robot model to correctly initialize
      // the sizes.
      LOG_DEBUG (logger, "netrob::Robot::readSensors(): no robot state yet (in NON-blocking mode)");
      jointPositions.zero();
      jointVelocities.zero();
      acquisition_time.tv_sec = 0;
      acquisition_time.tv_usec = 0;
      if (opt_force) {
	opt_force->zero();
      }
      return true;
    }
    
    jointPositions = m_robot_state->jointAngles;
    jointVelocities = m_robot_state->jointVelocities;
    acquisition_time = m_robot_state->acquisitionTime;
    if (opt_force) {
      *opt_force = m_robot_state->forces;
    }
    
    return true;
  }
  
  
  bool Robot::
  writeCommand(SAIVector const & command)
  {
    if (std::numeric_limits<uint8_t>::max() < command.size()) {
      LOG_ERROR (logger,
		 "netrob::Robot::writeCommand(): command size " << command.size()
		 << " exceeds maximum of " << (int) std::numeric_limits<uint8_t>::max());
      return false;
    }
    
    if ( ! lazyCreateChannel()) {
      return false;
    }
    
    if ( ! m_servo_command) {
      m_servo_command = new wbc::msg::ServoCommand(true, static_cast<uint8_t>(command.size()));
    }
    m_servo_command->command = command;
    
    wbcnet::muldex_status const ms(m_muldex.Mux(m_channel, *m_servo_command));
    
    if (ms.muldex == wbcnet::muldex_status::SUCCESS) {
      m_send_ok = true;
    }
    else {
      m_send_ok = false;
      if (m_blocking) {
	LOG_ERROR (logger,
		   "netrob::Robot::writeCommand(): muldex.Mux() said "
		   << wbcnet::muldex_status_str(ms) << " (in BLOCKING mode)");
	return false;
      }
      if (ms.com == wbcnet::COM_TRY_AGAIN) {
	LOG_DEBUG (logger,
		  "netrob::Robot::writeCommand(): muldex.Mux() said "
		  << wbcnet::muldex_status_str(ms) << " (which is OK because we are in NON-blocking mode)");
	return true;
      }
      else {
	LOG_ERROR (logger,
		   "netrob::Robot::writeCommand(): muldex.Mux() said "
		   << wbcnet::muldex_status_str(ms) << " (in NON-blocking mode)");
	return false;
      }
    }
    
    return true;
  }
  
  
  bool Robot::
  writeSensors(SAIVector const & jointAngles, SAIVector const & jointVelocities,
	       SAIMatrix const * opt_force)
  {
    if ((std::numeric_limits<uint8_t>::max() < jointAngles.size())
	|| (std::numeric_limits<uint8_t>::max() < jointVelocities.size())
	|| (opt_force && (std::numeric_limits<uint8_t>::max() < opt_force->row()))
	|| (opt_force && (std::numeric_limits<uint8_t>::max() < opt_force->column()))) {
      LOG_ERROR (logger,
		 "netrob::Robot::writeSensors(): maximum size exceeded\n"
		 << "  maximum = " << (int) std::numeric_limits<uint8_t>::max() << "\n"
		 << "  jointAngles.size() = " << jointAngles.size() << "\n"
		 << "  jointVelocities.size() = " << jointVelocities.size() << "\n"
		 << "  opt_force->row() = " << (opt_force ? to_string(opt_force->row()) : string("<NULL>")) << "\n"
		 << "  opt_force->column() = " << (opt_force ? to_string(opt_force->column()) : string("<NULL>")));
      return false;
    }
    
    if ( ! lazyCreateChannel()) {
      return false;
    }
    
    if ( ! m_robot_state) {
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
      LOG_ERROR (logger, "netrob::Robot::writeSensors(): gettimeofday() failed");
      return false;
    }
    m_robot_state->acquisitionTime = now;
    if (opt_force) {
      m_robot_state->forces = *opt_force;
    }
    
    wbcnet::muldex_status const ms(m_muldex.Mux(m_channel, *m_robot_state));
    
    if (ms.muldex == wbcnet::muldex_status::SUCCESS) {
      m_send_ok = true;
    }
    else {
      m_send_ok = false;
      if (m_blocking) {
	LOG_ERROR (logger,
		   "netrob::Robot::writeSensors(): muldex.Mux() said "
		   << wbcnet::muldex_status_str(ms) << " (in BLOCKING mode)");
	return false;
      }
      if (ms.com == wbcnet::COM_TRY_AGAIN) {
	LOG_DEBUG (logger,
		  "netrob::Robot::writeSensors(): muldex.Mux() said "
		  << wbcnet::muldex_status_str(ms) << " (which is OK because we are in NON-blocking mode)");
	return true;
      }
      else {
	LOG_ERROR (logger,
		   "netrob::Robot::writeSensors(): muldex.Mux() said "
		   << wbcnet::muldex_status_str(ms) << " (in NON-blocking mode)");
	return false;
      }
    }
    
    return true;
  }
  
  
  bool Robot::
  readCommand(SAIVector & command)
  {
    if ( ! lazyCreateChannel()) {
      return false;
    }
    
    wbcnet::muldex_status const ms(m_muldex.DemuxOne(m_channel));
    
    if (ms.muldex == wbcnet::muldex_status::SUCCESS) {
      m_receive_ok = true;
    }
    else {
      m_receive_ok = false;
      if (m_blocking) {
	LOG_ERROR (logger,
		   "netrob::Robot::readCommand(): m_muldex.DemuxOne() said "
		   << wbcnet::muldex_status_str(ms) << " (in BLOCKING mode)");
	return false;
      }
      if (ms.com == wbcnet::COM_TRY_AGAIN) {
	LOG_DEBUG (logger,
		  "netrob::Robot::readCommand(): m_muldex.DemuxOne() said "
		  << wbcnet::muldex_status_str(ms) << " (which is OK because we are in NON-blocking mode)");
	// keep working after the else block
      }
      else {
	LOG_ERROR (logger,
		   "netrob::Robot::readCommand(): m_muldex.DemuxOne() said "
		   << wbcnet::muldex_status_str(ms) << " (in NON-blocking mode)");
	return false;
      }
    }
    
    if ( ! m_servo_command) {
      if (m_blocking) {
	LOG_ERROR (logger, "netrob::Robot::readCommand(): no servo command (in BLOCKING mode)");
	return false;
      }
      // This'll only work if the params already have the correct
      // size... which they might or might not, depending on whether
      // the caller has used the robot model to correctly initialize
      // the sizes.
      LOG_DEBUG (logger, "netrob::Robot::readCommand(): no servo command yet (in NON-blocking mode)");
      command.zero();
      return true;
    }
    
    command = m_servo_command->command;
    
    return true;
  }
  
  
  void Robot::
  shutdown() const
  {
  }
  
  
  Robot * Factory::
  parse(std::string const & spec, wbc::ServoInspector * servo_inspector)
  {
    vector<string> token;
    sfl::tokenize(spec, '+', token);
    
    string mode_spec("b");
    string net_spec("mq:b");
    string channel_spec("netrob");
    sfl::token_to(token, 0, mode_spec);
    sfl::token_to(token, 1, net_spec);
    sfl::token_to(token, 2, channel_spec);
    
    bool blocking;
    if (("b" == mode_spec) || ("" == mode_spec)) {
      blocking = true;
    }
    else if ("n" == mode_spec) {
      blocking = false;
    }
    else {
      LOG_ERROR (logger,
		 "netrob::Factory::parse(" << spec << "): invalid mode_spec \"" << mode_spec
		 << "\" (should be 'b', 'n', or '')");
      return 0;
    }
    
    wbcnet::NetConfig * netconf;
    try {
      netconf = wbcnet::NetConfig::Create(net_spec);
      LOG_INFO (logger,
		"netrob::Factory::parse(): creating Robot with net_spec \"" << net_spec
		<< "\" channel_spec \"" << channel_spec << "\" blocking " << sfl::to_string(blocking) << "\n");
      return new Robot(netconf, channel_spec, blocking);
    }
    catch (std::runtime_error const & ee) {
      LOG_ERROR (logger,
		 "netrob::Factory::parse(): wbcnet::NetConfig::Create(" << net_spec
		 << ") EXCEPTION " << ee.what());
    }
    return 0;
  }
  
  
  void Factory::
  dumpHelp(std::string const & prefix, std::ostream & os) const
  {
    os << prefix << "spec = mode_spec '+' net_spec '+' channel_spec\n"
       << prefix << "  default = b+mq:b+netrob\n"
       << prefix << "  mode_spec is 'b' for blocking and 'n' for non-blocking\n"
       << prefix << "  net_spec is passed to wbcnet::NetConfig::Create()\n"
       << prefix << "  channel_spec is passed to wbcnet::NetConfig::CreateChannel()\n";
  }
  
  
  MessageHandler::
  MessageHandler(Robot * robot)
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
