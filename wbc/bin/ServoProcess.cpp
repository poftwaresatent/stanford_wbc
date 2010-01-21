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
   \file ServoProcess.cpp
   \author Roland Philippsen (execution control) and Luis Sentis (servo algorithm)
*/

#include "ServoProcess.hpp"
#include "TaskModelListener.hpp"
#include "DirectoryCmdServer.hpp"
#include <wbcnet/message_id.hpp>
#include <wbcnet/NetConfig.hpp>
#include <wbcnet/log.hpp>
#include <wbc/core/TaskModelBase.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/Kinematics.hpp>
#include <wbc/core/Dynamics.hpp>
#include <wbc/core/Contact.hpp>
#include <wbc/core/BehaviorDescription.hpp>
#include <wbc/core/RobotAPI.hpp>
#include <wbc/core/ServoBehaviorsAPI.hpp>
#include <wbc/msg/RobotState.hpp>
#include <wbcnet/msg/TaskSpec.hpp>
#include <wbcnet/msg/TaskMatrix.hpp>
#include <wbcnet/DelayHistogram.hpp>
#include <sstream>


static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));

using namespace std; 

namespace {
  static SAIVector const nullVector;
}


namespace wbc {
  
  
  ServoImplementation::
  ServoImplementation(size_t ndof,
		      size_t ndof_actuated,
		      size_t nvel,
		      size_t contact_nrows,
		      size_t contact_ncols,
		      RobotControlModel * robmodel,
		      ServoBehaviorsAPI * servoBehaviors,
		      std::vector<BehaviorDescription*> const & behavior,
		      /** \todo depends on multirate or not */
		      std::vector<TaskModelBase*> const & task_model_pool,
		      RobotAPI * robotAPI,
		      wbcnet::DelayHistogram * dhist,
		      int pskip)
    : m_joint_angles(ndof),
      m_joint_velocities(nvel),
      m_contact_forces(contact_nrows, contact_ncols),
      m_command_torques(0),	// init to zero to detect the second iteration
      m_dynamics(robmodel->dynamics()),
      m_kinematics(robmodel->kinematics()),
      m_contact(robmodel->contact()),
      m_current_behavior(0),
      m_servoBehaviors(servoBehaviors),
      m_behavior(behavior),
      m_task_model_listener(new TaskModelListener(task_model_pool[0],
						  task_model_pool[1])),
      m_robotAPI(robotAPI),
      m_ndof_actuated(ndof_actuated),
      m_dhist(dhist),
      m_pskip(pskip),
      m_robot_state(false, ndof, nvel, contact_nrows, contact_ncols),
      m_robmodel(robmodel),
      m_end_effector(robmodel->branching()->findLink("End_Effector"))
  {
  }
  
  
  ServoImplementation::
  ~ServoImplementation()
  {
    delete m_task_model_listener;
  }
  
  
  TaskModelListener * ServoImplementation::
  GetTaskModelListener()
  {
    return m_task_model_listener;
  }
  
  
  bool ServoImplementation::
  UpdateRobotState(msg::RobotState & robot_state)
  {
    LOG_DEBUG (logger, "wbc::ServoImplementation::UpdateRobotState()");
    
    timeval tstamp;
    if ( ! m_robotAPI->readSensors(m_joint_angles,
				   m_joint_velocities,
				   tstamp,
				   &m_contact_forces)) {
      LOG_ERROR (logger, "wbc::ServoImplementation::UpdateRobotState(): RobotAPI::readSensors() failed");
      return false;
    }

    if (logger->isDebugEnabled()) {
      ostringstream msg;
      m_contact_forces.prettyPrint(msg, "contact forces", "");
      LOG_DEBUG (logger, msg.str());
    }
    
    if ((size_t) m_command_torques.size() != m_ndof_actuated) // skip timing measure in the first iteration
      m_command_torques.setSize(m_ndof_actuated, true);
    else if (m_dhist)
      m_dhist->StartStop(0, &m_robot_state_tstamp, &tstamp);
    m_robot_state_tstamp = tstamp;
    
    if ( ! robot_state.Copy(wbcnet::timestamp(tstamp),
			    m_joint_angles,
			    m_joint_velocities,
			    m_contact_forces)) {
      LOG_ERROR (logger, "wbc::ServoImplementation::UpdateRobotState(): robot_state.Copy() failed");
      return false;
    }
    
    if (m_pskip > 0) {
      static int count(0);
      if (0 == (count % m_pskip)) {
	cout << "wbc::ServoImplementation::UpdateRobotState(): robot_state\n";
	robot_state.display(cout, "  ");
      }
      ++count;
    }
    
    // XXXX to do: remove duplicates... pass robot_state into the
    // other methods!  ...only needed in case we are running the model
    // and servo in one process, in the multirate setup the
    // robot_state parameter actually already is our local copy, but
    // in uni-process setup it is the model implementation's copy of
    // the state.
    m_robot_state.Copy(robot_state.acquisitionTime,
		       *robot_state.jointAnglesPtr,
		       *robot_state.jointVelocitiesPtr,
		       *robot_state.forcesPtr);
    return true;
  }
  
  
  bool ServoImplementation::
  NullTorqueCommand()
  {
    LOG_DEBUG (logger, "wbc::ServoImplementation::NullTorqueCommand()");
    
    m_command_torques.zero();
    
    return m_robotAPI->writeCommand(m_command_torques);
  }
  
  
  bool ServoImplementation::
  UpdateTorqueCommand(TaskModelBase const * model,
		      int current_behaviorID,
		      bool skip_behavior_update)
  {
    LOG_DEBUG (logger, "wbc::ServoImplementation::UpdateTorqueCommand()");
    
    if ((0 > current_behaviorID) || (static_cast<size_t>(current_behaviorID) >= m_behavior.size())) {
      LOG_ERROR (logger,
		 "wbc::ServoImplementation::UpdateTorqueCommand(): invalid behaviorID "
		 << current_behaviorID << " (only " << m_behavior.size() << " available)");
      return false;
    }
    m_current_behavior = m_behavior[current_behaviorID];
    
    // update the models: only dynamics needs to sync from
    // model (which might actually not have changed since
    // last time -- waste of computing cycles?), all the others get
    // updated from current state
    
    if (logger->isDebugEnabled()) {
      ostringstream msg;
      model->prettyPrint(msg,
			 "wbc::ServoImplementation::UpdateTorqueCommand(): model",
			 "  ");
      LOG_DEBUG (logger, msg.str());
    }
    
    m_kinematics->onUpdate(m_robot_state.jointAngles, m_robot_state.jointVelocities);
    ostringstream msg;
    if ( ! m_kinematics->checkDisplayJointLimits(logger->isDebugEnabled() ? &msg : 0,
						 "wbc::ServoImplementation::UpdateTorqueCommand()",
						 "  ")) {
      LOG_DEBUG (logger, msg.str() << "wbc::ServoImplementation::UpdateTorqueCommand(): joint limits hit (ignored...)");
      // XXXX just ignore joint limits for the time being
    }
    
    m_contact->onUpdate(m_contact_forces);
    

    // a bit of a hack to deal with time-dilation due to simulation
    m_robmodel->time(m_robot_state.acquisitionTime);
    
    SAIVector gravity, coriolis;
    TaskModelView const * model_view(model->GetView());
    model_view->gravityForce()->getColumn(0, gravity);
    model_view->coriolisCentrifugalForce()->getColumn(0, coriolis);
    if ((1 == coriolis.size()) || (1 == gravity.size()))
      LOG_WARN (logger,
		"ServoImplementation::UpdateTorqueCommand(): coriolis and/or gravity have size one");
    
    // XXXX to do: uniform interface that takes matrices for all
    // parameters, and no need to synch in uni-process update
    m_dynamics->synchronize(*model_view->massInertia(),
			    *model_view->invMassInertia(),
			    gravity,
			    coriolis);
    
    if ( ! skip_behavior_update) {
      // Update the currently running behavior, which will take into
      // account all the models we updated above.
      m_current_behavior->onUpdate();
    }
    
    bool const emergency( ! m_servoBehaviors->updateTorques(m_current_behavior,
							    m_robmodel,
							    model,
							    m_command_torques));
    
    if (emergency) {
      // hardcoded high-friction emergency behavior 
      m_command_torques
	= m_dynamics->gravityForce()
	- *model_view->massInertia()
	* m_robot_state.jointVelocities * 20;// or 50 or...
    }
    
    if (m_pskip > 0) {
      static int count(0);
      if (0 == (count % m_pskip)) {
	m_command_torques.display("command torque:");
	m_robotAPI->dumpStats("robotAPI stats: ", cerr);
      }
      ++count;
    }
    
    if (logger->isDebugEnabled()) {
      ostringstream msg;
      model->prettyPrint(msg,
			 ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n"
			 "ServoImplementation::torque(): MODEL STATE",
			 "  ");
      msg << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<";
      LOG_DEBUG (logger, msg.str());
    }
    
    return m_robotAPI->writeCommand(m_command_torques);
  }
    
    
  bool ServoImplementation::
  ResetBehavior(TaskModelBase * next_task_model,
		uint8_t requestID,
		int behaviorID)
  {
    if (static_cast<size_t>(behaviorID) > m_behavior.size()) {
      LOG_ERROR (logger,
		 "wbc::ServoImplementation::ResetBehavior():  invalid behaviorID " << behaviorID
		 << " (only " << m_behavior.size() << " available)");
      return false;
    }
    if ( ! next_task_model->Reset(requestID, *m_behavior[behaviorID])) {
      LOG_ERROR (logger,
		 "wbc::ServoImplementation::ResetBehavior(): next_task_model->Reset(" << (int) requestID
		 << ", behavior[" << behaviorID << "]) failed");
      return false;
    }
    LOG_DEBUG (logger,
	       "wbc::ServoImplementation::ResetBehavior() requestID: " << (int) requestID
	       << "  behaviorID: " << behaviorID);
    return true;
  }
  
  
  ServoProcess::
  ServoProcess()
    : Process("servo", 0, -1, wbcnet::ENDIAN_DETECT),
      m_directory_cmd_server(0),
      m_imp(0),
      m_own_imp(false),
      m_state(READY_STATE),
      m_model_channel(0),
      m_user_channel(0),
      m_model_listener(0),
      m_current_behaviorID(-1),
      m_next_behaviorID(-1),
      m_behavior_transition_requestID(0),
      m_init_behavior_transition(false),
      m_model_status(),
      m_user_task_spec(),
      m_user_request(wbcnet::msg::USER_REQUEST),
      m_servo_status(),
      m_robot_state(0),
      m_model_task_spec(),
      m_user_reply(wbcnet::msg::USER_REPLY)
  {
  }
  
  
  bool ServoProcess::
  Step() throw(std::exception)
  {
    LOG_TRACE (logger, "wbc::ServoProcess::Step(): receiving...");
    
    Receive();
    
    if ( ! m_imp->UpdateRobotState(*m_robot_state)) {
      LOG_TRACE (logger, "wbc::ServoProcess::Step(): UpdateRobotState() failed");
      return false;
    }
    
    if (ERROR_STATE == m_state) {
      LOG_ERROR (logger, "wbc::ServoProcess::Step(): ERROR state");
      return false;
    }
    
    if (READY_STATE == m_state) {
      if (m_init_behavior_transition) {
	LOG_TRACE (logger, "wbc::ServoProcess::Step(): READY --> WAIT_MODEL");
	m_state = WAIT_MODEL_STATE;
	m_model_status.status = msg::MODEL_SUCCESS; // trick to kickstart communication
      }
    }
    
    // maybe feed the model with state and maybe even with task spec
    
    if (m_model_status.status != msg::VOID_STATUS) {
      m_model_status.status = msg::VOID_STATUS;
      ++m_robot_state->requestID;
      
      if (m_init_behavior_transition) {
	m_init_behavior_transition = false;
	m_behavior_transition_requestID = m_robot_state->requestID;
	
	// This is work in progress... subclasses should not need to
	// worry about the transition request ID.
	if ( ! m_imp->ResetBehavior(m_model_listener->GetStaleModel(),
				    m_behavior_transition_requestID, m_next_behaviorID)) {
	  LOG_ERROR (logger,
		     "wbc::ServoProcess::Step(): ResetBehavior(..."
		     << (int) m_behavior_transition_requestID << ", "
		     << m_next_behaviorID << ") failed");
	  m_state = ERROR_STATE;
	  // not such a great idea to return without sending the enqueued messages though
	  return false;
	}
	
	// We will detect that the behavior transition has been
	// processed by the model when it sends back matrices that
	// match the m_behavior_transition_requestID. In order to
	// allow the model to work with the latest robot state, and in
	// order to avoid confusion between the robot_state requestID
	// and the task_spec requestID, we force all of them to be the
	// same and based on the robot_state.
	
	m_model_task_spec.requestID = m_behavior_transition_requestID;
	m_model_task_spec.behaviorID = m_next_behaviorID;
	if (logger->isTraceEnabled()) {
	  ostringstream msg;
	  msg << "wbc::ServoProcess::Step(): enqueueing model task spec\n";
	  m_model_task_spec.display(msg, "  ");
	  LOG_TRACE (logger, msg.str());
	}
	EnqueueMessage(m_model_channel, &m_model_task_spec, true, false);
      }
      
      LOG_TRACE (logger, "wbc::ServoProcess::Step(): feed robot state to model");
      // m_robot_state->requestID has been incremented above
      EnqueueMessage(m_model_channel, m_robot_state, true, false);
      
      LOG_TRACE (logger, "  enqueueing COMPUTE_MODEL status");
      m_servo_status.status = msg::COMPUTE_MODEL;
      EnqueueMessage(m_model_channel, &m_servo_status, true, false);
      
    } // end if (m_model_status.status != msg::VOID)
    
    else
      LOG_TRACE (logger, "wbc::ServoProcess::Step(): model status is VOID, do not feed it");
    
    if ((READY_STATE == m_state) || (WAIT_MODEL_STATE == m_state)) {
      LOG_TRACE (logger, "wbc::ServoProcess::Step(): READY or WAIT_MODEL: use NULL torque");
      if ( ! m_imp->NullTorqueCommand()) {
	LOG_ERROR (logger, "wbc::ServoProcess::Step(): NullTorqueCommand() failed");
	return false;
      }
    }
    else { //if (RUNNING == m_state) {
      LOG_TRACE (logger, "wbc::ServoProcess::Step(): RUNNING: update torque");
      if ( ! m_imp->UpdateTorqueCommand(m_model_listener->GetLastUpdatedModel(),
					m_current_behaviorID,
					false)) {
	LOG_ERROR (logger,
		   "wbc::ServoProcess::Step(): UpdateTorqueCommand(..., "
		   << m_current_behaviorID << ") failed");
	return false;
      }
    }
    
    LOG_TRACE (logger, "wbc::ServoProcess::Step(): sending...");
    
    Send();
    
    return true;
  }
  
  
  ServoProcess::
  ~ServoProcess()
  {
    delete m_robot_state;
    delete m_model_channel;
    delete m_user_channel;
    if (m_own_imp)
      delete m_imp;
    delete m_directory_cmd_server;
  }
  
  
  /**
     \todo hardcoded max_n_snd and max_n_rcv.
  */
  void ServoProcess::
  Init(ServoImplementation * imp,
       bool own_imp,
       wbcnet::NetConfig const & netconf,
       uint8_t npos, uint8_t nvel,
       uint8_t force_nrows, uint8_t force_ncols) throw(std::exception)
  {
    if (m_model_channel) {
      // We could also just return, but maybe people think they can
      // re-configure us by calling Init() with a different NetConfig,
      // but that's trickier than it might seem because of the
      // incoming and outgoing message queues.
      throw runtime_error("wbc::ServoProcess::Init(): already initialized");
    }
    
    m_imp = imp;
    m_own_imp = own_imp;
    
    m_model_listener = m_imp->GetTaskModelListener();
    if (0 == m_model_listener)
      throw runtime_error("wbc::ServoProcess::Init(): GetTaskModelListener() failed");
    
    m_model_channel = netconf.CreateChannel(wbcnet::NetConfig::SERVO, wbcnet::NetConfig::MODEL);
    if ( ! m_model_channel)
      throw runtime_error("wbc::ServoProcess::Init(): could not create model channel");
    AddSink(m_model_channel, 100);
    AddSource(m_model_channel, 100);
    
    m_user_channel = netconf.CreateChannel(wbcnet::NetConfig::SERVO, wbcnet::NetConfig::USER);
    if ( ! m_user_channel)
      throw runtime_error("wbc::ServoProcess::Init(): could not create user channel");
    AddSink(m_user_channel, 100);
    AddSource(m_user_channel, 1); // limit the max rate of user requests to one per cycle
    
    m_robot_state = new msg::RobotState(false, npos, nvel, force_nrows, force_ncols);
    m_model_status.status = msg::VOID_STATUS;
    
    CreateHandler(wbcnet::msg::STATUS, "model_status", & m_model_status);
    CreateHandler(wbcnet::msg::TASK_SPEC, "user_task_spec", & m_user_task_spec);
    CreateHandler(wbcnet::msg::USER_REQUEST, "user_request", & m_user_request);
    
    m_muldex.SetHandler(wbcnet::msg::TASK_MATRIX,
			new wbcnet::ProxyHandler("task_matrix",
						 m_model_listener->task_matrix,
						 true, m_model_listener));
  }
  
  
  int ServoProcess::
  HandleMessagePayload(wbcnet::unique_id_t msg_id)
  {
    if (wbcnet::msg::STATUS == msg_id) {
      LOG_TRACE (logger, "wbc::ServoProcess::HandleMessagePayload(): got STATUS");
      
      if (msg::MODEL_SUCCESS == m_model_status.status) {
	LOG_TRACE (logger, "  MODEL_SUCCESS");
	if (WAIT_MODEL_STATE == m_state) {
	  LOG_TRACE (logger, "  transition state WAIT_MODEL to RUNNING");
	  m_state = RUNNING_STATE;
	}
	TaskModelBase * task_model(m_model_listener->GetLastUpdatedModel());
	if ( ! task_model) {
	  LOG_ERROR (logger,
		     "wbc::ServoProcess::HandleMessagePayload():\n"
		     << "  BUG? m_model_listener->GetLastUpdatedModel() returned NULL\n"
		     << "  but it should have received at least one matrix message before\n"
		     << "  arriving here.");
	  return 777;
	}
	uint8_t const requestID(task_model->GetRequestID());
	if ((m_current_behaviorID != m_next_behaviorID)
	    && (requestID == m_behavior_transition_requestID)) {
	  LOG_TRACE (logger,
		     "  finish behavior transition\n"
		     << "    m_current_behaviorID = " << m_current_behaviorID << "\n"
		     << "    m_next_behaviorID = " << m_next_behaviorID << "\n"
		     << "    requestID = " << (int) requestID << "\n"
		     << "    m_behavior_transition_requestID = "
		     << (int) m_behavior_transition_requestID);
	  m_current_behaviorID = m_next_behaviorID;
	}
	else
	  if (logger->isTraceEnabled()) {
	    if (m_current_behaviorID == m_next_behaviorID) {
	      LOG_TRACE (logger,
			 "  got a fresh model for the still running behavior "
			 << m_current_behaviorID);
	    }
	    else {
	      LOG_TRACE (logger,
			 "  cannot finish behavior transition:\n"
			 << "   probably this means we got an update for the old behavior\n"
			 << "   or maybe it's a bug in request ID handling\n"
			 << "     m_current_behaviorID = " << m_current_behaviorID << "\n"
			 << "     m_next_behaviorID = " << m_next_behaviorID << "\n"
			 << "     requestID = " << (int) requestID << "\n"
			 << "     m_behavior_transition_requestID = "
			 << (int) m_behavior_transition_requestID);
	    }
	  }
	////	UpdateTaskModel(task_model, m_current_behaviorID);
      }
      
      else if (msg::MODEL_ERROR == m_model_status.status) {
	LOG_TRACE (logger, "  MODEL_ERROR --> just retry");
	////maybe one day//// HandleModelFailure();
      }
      
      else
	LOG_TRACE (logger, "  ignoring status " << (int) m_model_status.status << "");
    }

    else if (wbcnet::msg::TASK_SPEC == msg_id) {
      LOG_TRACE (logger, "wbc::ServoProcess::HandleMessagePayload(): got TASK_SPEC");
      BeginBehaviorTransition(m_user_task_spec.behaviorID);
    }
    
    else if (wbcnet::msg::USER_REQUEST == msg_id) {
      LOG_INFO (logger, "wbc::ServoProcess::HandleMessagePayload(): got USER_REQUEST");
      if ( ! m_directory_cmd_server) {
	m_directory_cmd_server = new DirectoryCmdServer(m_imp->GetBehaviorLibrary(), this);
      }
      if ( ! m_directory_cmd_server->Dispatch(m_user_request, m_user_reply)) {
	if (logger->isWarnEnabled()) {
	  ostringstream msg;
	  msg << "wbc::ServoProcess::HandleMessagePayload(): DirectoryCmdServer::Dispatch() failed\n"
	      << "  It did not seem like the user request:\n";
	  m_user_request.Dump(msg, "    ");
	  LOG_WARN (logger, msg.str());
	}
      }
      EnqueueMessage(m_user_channel, &m_user_reply, true, false);
    }
    
    else {
      LOG_TRACE (logger,
		 "wbc::ServoProcess::HandleMessagePayload()\n"
		 << "  unknown message ID " << (int) msg_id
		 << " [" << wbcnet::msg::get_id_str(msg_id));
      return 999;
    }
    
    return 0;
  }


  wbcnet::srv_result_t ServoProcess::
  BeginBehaviorTransition(int behaviorID)
  {
    if ((0 > behaviorID) || (static_cast<size_t>(behaviorID) >= m_imp->GetBehaviorLibrary().size())) {
      LOG_TRACE (logger,
		 "ServoProcess::BeginBehaviorTransition(" << behaviorID
		 << "): INVALID CODE, only " << m_imp->GetBehaviorLibrary().size() << " behaviors available");
      return wbcnet::SRV_INVALID_CODE;
    }
    
    if (m_current_behaviorID == behaviorID) {
      LOG_TRACE (logger,
		 "ServoProcess::BeginBehaviorTransition(" << behaviorID
		 << "): SUCCESS (already running that behavior)");
      return wbcnet::SRV_SUCCESS;
    }
    
    if (m_current_behaviorID != m_next_behaviorID) {
      LOG_TRACE (logger,
		 "ServoProcess::BeginBehaviorTransition(" << behaviorID
		 << "): TRY AGAIN, already transitioning from " << m_current_behaviorID
		 << " to " << (int) m_next_behaviorID);
      return wbcnet::SRV_TRY_AGAIN;
    }
    
    LOG_TRACE (logger,
	       "ServoProcess::BeginBehaviorTransition(" << behaviorID
	       << "): SUCCESS, initializing transition from " << m_current_behaviorID);
    
    // It is important to always start transitions with the most
    // recent robot state available, so we defer actually sending the
    // task spec until the next time we got a robot state update in
    // Step().
    m_next_behaviorID = behaviorID;
    m_init_behavior_transition = true;
    
    return wbcnet::SRV_SUCCESS;
  }
  
  
  BranchingRepresentation * ServoProcess::
  GetBranching()
  {
    return m_imp->m_robmodel->branching();
  }
  
  
  Kinematics * ServoProcess::
  GetKinematics()
  {
    return m_imp->m_kinematics;
  }
  
  
  SAIVector const & ServoProcess::
  GetCommandTorques()
  {
    return m_imp->m_command_torques;
  }
  
  
  BehaviorDescription * ServoProcess::
  GetCurrentBehavior()
  {
    return m_imp->m_current_behavior;
  }
  
}
