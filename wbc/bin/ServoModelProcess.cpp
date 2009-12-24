/*
 * Copyright (c) 2009 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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

#include "ServoModelProcess.hpp"
#include "DirectoryCmdServer.hpp"
#include <wbcnet/NetConfig.hpp>
#include <wbcrun/msg/RobotState.hpp>
#include <wbcnet/log.hpp>
#include <iostream>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));

using namespace std; 


namespace wbc {
  
  
  ServoModelProcess::
  ServoModelProcess()
    : Process("servo-model", 0, -1, wbcnet::ENDIAN_DETECT),
      m_directory_cmd_server(0),
      m_servo_imp(0),
      m_own_servo_imp(false),
      m_model_imp(0),
      m_own_model_imp(false),
      m_state(READY_STATE),
      m_have_behaviorID(false),
      m_robot_state(0),
      m_user_channel(0),
      m_user_task_spec(),
      m_user_request(wbcrun::msg::USER_REQUEST),
      m_user_reply(wbcrun::msg::USER_REPLY)
  {
  }
  
  
  bool ServoModelProcess::
  Step() throw(std::exception)
  {
    LOG_TRACE (logger, "wbc::ServoModelProcess::Step(): receiving...");
    
    Receive();
    
    if ( ! m_servo_imp->UpdateRobotState(*m_robot_state)) {
      LOG_TRACE (logger, "wbc::ServoModelProcess::Step(): UpdateRobotState() failed");
      m_state = ERROR_STATE;
      return false;
    }
    
    if (ERROR_STATE == m_state) {
      LOG_ERROR (logger, "wbc::ServoModelProcess::Step(): ERROR state");
      return false;
    }
    
    if (READY_STATE == m_state) {
      if (m_have_behaviorID) {
	LOG_TRACE (logger, "wbc::ServoModelProcess::Step(): READY --> RUNNING");
	m_state = RUNNING_STATE;
      }
      else
	LOG_TRACE (logger, "wbc::ServoModelProcess::Step(): still waiting for behavior ID");
    }
    
    if (RUNNING_STATE != m_state) {
      LOG_TRACE (logger, "wbc::ServoModelProcess::Step(): not running --> use NULL torque");
      if ( ! m_servo_imp->NullTorqueCommand()) {
	LOG_ERROR (logger, "wbc::ServoModelProcess::Step(): NullTorqueCommand() failed");
	m_state = ERROR_STATE;
	return false;
      }
    }
    else {
      LOG_TRACE (logger, "wbc::ServoModelProcess::Step(): RUNNING");
      
      LOG_TRACE (logger, "  update model");
      // This is a bit obscure: the model implementation gets called
      // first, so we tell it to go ahead and update the current
      // behavior for us. Further down, when we update the servo
      // implementation, we tell it to skip that update. Otherwise it
      // would get called twice.
      if ( ! m_model_imp->ComputeModel(*m_robot_state, m_user_task_spec, false)) {
	LOG_ERROR (logger, "wbc::ServoModelProcess::Step(): ComputeModel() FAILED");
	m_state = ERROR_STATE;
	return false;
      }
      
      LOG_TRACE (logger, "wbc::ServoModelProcess::Step(): RUNNING: update torque");
      if ( ! m_servo_imp->UpdateTorqueCommand(m_model_imp->GetTaskModel(), m_user_task_spec.behaviorID, true)) {
	LOG_ERROR (logger,
		       "wbc::ServoModelProcess::Step(): UpdateTorqueCommand(..., "
		       << (int) m_user_task_spec.behaviorID << ") failed");
	m_state = ERROR_STATE;
	return false;
      }
    }
    
    LOG_TRACE (logger, "wbc::ServoModelProcess::Step(): sending...");
    
    Send();
    
    return true;
  }
  
  
  ServoModelProcess::
  ~ServoModelProcess()
  {
    delete m_robot_state;
    delete m_user_channel;
    if (m_own_servo_imp)
      delete m_servo_imp;
    if (m_own_model_imp)
      delete m_model_imp;
    delete m_directory_cmd_server;
  }
  
  
  /**
     \todo hardcoded max_n_snd and max_n_rcv.
  */
  void ServoModelProcess::
  Init(ServoImplementation * servo_imp,
       bool own_servo_imp,
       ModelImplementation * model_imp,
       bool own_model_imp,
       wbcnet::NetConfig const & netconf,
       uint8_t npos, uint8_t nvel,
       uint8_t force_nrows, uint8_t force_ncols) throw(std::exception)
  {
    if (m_servo_imp) {
      // We could also just return, but maybe people think they can
      // re-configure us by calling Init() with a different NetConfig,
      // but that's trickier than it might seem because of the
      // incoming and outgoing message queues.
      throw runtime_error("wbc::ServoModelProcess::Init(): already initialized");
    }
    
    m_servo_imp = servo_imp;
    m_own_servo_imp = own_servo_imp;
    m_model_imp = model_imp;
    m_own_model_imp = own_model_imp;
    
    m_user_channel = netconf.CreateChannel(wbcnet::NetConfig::SERVO, wbcnet::NetConfig::USER);
    if ( ! m_user_channel)
      throw runtime_error("wbc::ServoModelProcess::Init(): could not create user channel");
    AddSink(m_user_channel, 100);
    AddSource(m_user_channel, 1); // limit the max rate of user requests to one per cycle
    
    m_robot_state = new wbcrun::msg::RobotState(false, npos, nvel, force_nrows, force_ncols);
    
    CreateHandler(wbcrun::msg::TASK_SPEC, "user_task_spec", & m_user_task_spec);
    CreateHandler(wbcrun::msg::USER_REQUEST, "user_request", & m_user_request);
  }
  
  
  int ServoModelProcess::
  HandleMessagePayload(wbcnet::unique_id_t msg_id)
  {
    if (wbcrun::msg::TASK_SPEC == msg_id) {
      LOG_TRACE (logger, "wbc::ServoModelProcess::HandleMessagePayload(): got TASK_SPEC");
      m_have_behaviorID = true;
    }
    
    else if (wbcrun::msg::USER_REQUEST == msg_id) {
      LOG_TRACE (logger, "wbc::ServoModelProcess::HandleMessagePayload(): got USER_REQUEST");
      if ( ! m_directory_cmd_server) {
	m_directory_cmd_server = new DirectoryCmdServer(m_servo_imp->GetBehaviorLibrary(), this);
      }
      if ( ! m_directory_cmd_server->Dispatch(m_user_request, m_user_reply)) {
	if (logger->isWarnEnabled()) {
	  ostringstream msg;
	  msg << "wbc::ServoModelProcess::HandleMessagePayload(): DirectoryCmdServer::Dispatch() failed\n"
	      << "  It did not seem like the user request:\n";
	  m_user_request.Dump(msg, "    ");
	  LOG_WARN (logger, msg.str());
	}
      }
      EnqueueMessage(m_user_channel, &m_user_reply, true, false);
    }
    
    else {
      LOG_TRACE (logger,
		     "wbc::ServoModelProcess::HandleMessagePayload()\n"
		     << "  unknown message ID " << (int) msg_id
		     << " [" << wbcrun::msg::get_id_str(msg_id) << "]");
      return 999;
    }
    
    return 0;
  }
  
}
