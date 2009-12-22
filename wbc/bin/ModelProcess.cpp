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

#include "ModelProcess.hpp"
#include <wbcrun/message_id.hpp>
#include <wbcnet/NetConfig.hpp>
#include <wbcnet/log.hpp>
#include <wbcrun/msg/RobotState.hpp>
#include <wbcnet/msg/TaskSpec.hpp>
#include <wbcnet/msg/TaskMatrix.hpp>
#include <wbcnet/strutil.hpp>
#include <iostream>

#include <wbc/core/TaskModelBase.hpp>
#include <wbc/core/Kinematics.hpp>
#include <wbc/core/Dynamics.hpp>
#include <wbc/core/BehaviorDescription.hpp>
#include <wbc/core/SAIVectorAPI.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));

using namespace std; 


namespace wbc {
  
  
  ModelProcess::
  ModelProcess()
    : Process("model", 0, -1, wbcnet::ENDIAN_DETECT),
      m_imp(0),
      m_own_imp(false),
      m_state(IDLE),
      m_channel(0),
      m_servo_status(),
      m_robot_state(0),
      m_task_spec(),
      m_model_status(),
      m_task_matrix()
  {
  }
  
  
  /**
     \todo Hardcoded sleep_usecs values for the blocking communication
     calls.
  */
  bool ModelProcess::
  Step() throw(std::exception)
  {
    LOG_TRACE (logger, "wbc::ModelProcess::Step(): receiving...");
    
    ReceiveWait(1000, 1);
    
    if (IDLE == m_state) {
      LOG_TRACE (logger, "wbc::ModelProcess::Step(): in IDLE state, just return");
      return true;
    }
    
    if (COMPUTE == m_state) {
      if (logger->isTraceEnabled()) {
	ostringstream msg;
	msg << "wbc::ModelProcess::Step(): in COMPUTE, here's the task_spec:\n";
	m_task_spec.display(msg, "  ");
	LOG_TRACE (logger, msg.str());
      }
      
      if (m_imp->ComputeModel(*m_robot_state, m_task_spec, false)) {
	LOG_TRACE (logger, "wbc::ModelProcess::Step(): computed model");
	m_state = SEND_SUCCESS;
      }
      else {
	LOG_TRACE (logger, "wbc::ModelProcess::Step(): FAILED to compute model");
	m_state = SEND_FAILURE;
      }
    }
    
    if (SEND_SUCCESS == m_state) {
      
      LOG_TRACE (logger, "wbc::ModelProcess::Step(): in SEND_SUCCESS state");
      
      // get the model and prepare constant parts of the matrix messages
      TaskModelBase const * model(m_imp->GetTaskModel());
      if (0 == model)
	throw runtime_error("wbc::ModelProcess::Step(): no task model");
      m_task_matrix.requestID = model->GetRequestID();
      m_task_matrix.acquisitionTime = model->GetAcquisitionTime();
      
      int const ngenmx(model->GetNGeneralMatrices());
      int const nsets(model->GetNSets());
      int const nsetmx(model->GetNSetMatrices());
      int const ttntasks(model->GetTTNTasks());
      int const ntaskmx(model->GetNTaskMatrices());
      if ((ngenmx < 0) || (nsets < 0) || (nsetmx < 0) || (ttntasks < 0) || (ntaskmx < 0)) {
	LOG_ERROR (logger,
		   "wbc::ModelProcess::Step(): invalid number of matrices, sets, and/or tasks\n"
		   << "  The implementing subclass is responsible for setting these in the\n"
		   << "  TaskModelAPI returned by GetTaskModel()\n"
		   << "    ngenmx:   " << ngenmx << "\n"
		   << "    nsets:    " << nsets << "\n"
		   << "    nsetmx:   " << nsetmx << "\n"
		   << "    ttntasks: " << ttntasks << "\n"
		   << "    ntaskmx:  " << ntaskmx);
	return false;
      }
      
      LOG_TRACE (logger,
		 "wbc::ModelProcess::Step(): sending matrices\n"
		 << "  ngenmx:   " << ngenmx << "\n"
		 << "  nsets:    " << nsets << "\n"
		 << "  nsetmx:   " << nsetmx << "\n"
		 << "  ttntasks: " << ttntasks << "\n"
		 << "  ntaskmx:  " << ntaskmx);
      
      // send over the task-independent matrices
      m_task_matrix.setID = -1;
      m_task_matrix.taskID = -1;
      for (int igenmx(0); igenmx < ngenmx; ++igenmx) {
	m_task_matrix.dataptr = model->GetMatrix(-1, -1, igenmx);
	if (0 == m_task_matrix.dataptr)
	  throw runtime_error("wbc::ModelProcess::Step(): no general matrix number "
			      + sfl::to_string(igenmx));
	m_task_matrix.matrixID = igenmx;
	m_task_matrix.nRows = m_task_matrix.dataptr->NRows();
	m_task_matrix.nColumns = m_task_matrix.dataptr->NColumns();
	LOG_TRACE (logger,
		   "wbc::ModelProcess::Step(): sending task-independent matrix "
		   << model->GetName(-1, -1, igenmx));
	EnqueueMessage(m_channel, &m_task_matrix, false, false);
	SendWait(1000);
      }
      
      // send over the set-dependent matrices
      m_task_matrix.taskID = -1; // redundant
      for (int iset(0); iset < nsets; ++iset) {
	m_task_matrix.setID = iset;
	for (int isetmx(0); isetmx < nsetmx; ++isetmx) {
	  m_task_matrix.dataptr = model->GetMatrix(iset, -1, isetmx);
	  if (0 == m_task_matrix.dataptr)
	    throw runtime_error("wbc::ModelProcess::Step(): no task matrix number "
				+ sfl::to_string(isetmx) + " for set number "
				+ sfl::to_string(iset));
	  m_task_matrix.matrixID = isetmx;
	  m_task_matrix.nRows = m_task_matrix.dataptr->NRows();
	  m_task_matrix.nColumns = m_task_matrix.dataptr->NColumns();
	  LOG_TRACE (logger,
		     "wbc::ModelProcess::Step(): sending set-dependent matrix "
		     << model->GetName(iset, -1, isetmx) << "[" << iset << "]");
	  EnqueueMessage(m_channel, &m_task_matrix, false, false);
	  SendWait(1000);
	}
      }
      
      // send over the task-dependent matrices
      m_task_matrix.setID = 0;	// redundant
      for (int itask(0); itask < ttntasks; ++itask) {
	m_task_matrix.taskID = itask;
	for (int itaskmx(0); itaskmx < ntaskmx; ++itaskmx) {
	  m_task_matrix.dataptr = model->GetMatrix(/* confusing: ignored, but must not be -1 */ 0,
						   itask, itaskmx);
	  if (0 == m_task_matrix.dataptr)
	    throw runtime_error("wbc::ModelProcess::Step(): no task matrix number "
				+ sfl::to_string(itaskmx) + " for task number "
				+ sfl::to_string(itask));
	  m_task_matrix.matrixID = itaskmx;
	  m_task_matrix.nRows = m_task_matrix.dataptr->NRows();
	  m_task_matrix.nColumns = m_task_matrix.dataptr->NColumns();
	  LOG_TRACE (logger,
		     "wbc::ModelProcess::Step(): sending task-dependent matrix "
		     << model->GetName(/* confusing: ignored, but must not be -1 */ 0,
				       itask, itaskmx) << "[" << itask << "]");
	  EnqueueMessage(m_channel, &m_task_matrix, false, false);
	  SendWait(1000);
	}
      }
      
      // tell the servo that the model update is finished
      m_model_status.status = wbcrun::msg::MODEL_SUCCESS;
      LOG_TRACE (logger, "wbc::ModelProcess::Step(): enqueuing and sending MODEL_SUCCESS");
      EnqueueMessage(m_channel, &m_model_status, false, false);
      SendWait(1000);
    }
    
    if (SEND_FAILURE == m_state) {
      LOG_TRACE (logger, "wbc::ModelProcess::Step(): in SEND_FAILURE state");
      m_model_status.status = wbcrun::msg::MODEL_ERROR;
      EnqueueMessage(m_channel, &m_model_status, false, false);
      SendWait(1000);
    }
    
    LOG_TRACE (logger, "wbc::ModelProcess::Step(): done, going IDLE");
    
    m_state = IDLE;
    
    return true;
  }
  
  
  ModelProcess::
  ~ModelProcess()
  {
    delete m_robot_state;
    delete m_channel;
    if (m_own_imp)
      delete m_imp;
  }
  
  
  /**
     \todo
     - NetConfig should be passed as parameter.
     - Need a way to pass in ndof and nvel, without needing a tao branching model.
     - Might need a way to get at behaviors, without needing WBC code.
  */
  void ModelProcess::
  Init(ModelImplementation * imp,
       bool own_imp,
       wbcnet::NetConfig const & netconf,
       uint8_t npos, uint8_t nvel,
       uint8_t force_nrows, uint8_t force_ncols) throw(std::exception)
  {
    if (m_channel) {
      // We could also just return, but maybe people think they can
      // re-configure us by calling Init() with a different NetConfig,
      // but that's trickier than it might seem because of the
      // incoming and outgoing message queues.
      throw runtime_error("wbc::ModelProcess::DoInit(): already initialized");
    }
    
    m_imp = imp;
    m_own_imp = own_imp;
    
    m_channel = netconf.CreateChannel(wbcnet::NetConfig::MODEL, wbcnet::NetConfig::SERVO);
    if ( ! m_channel)
      throw runtime_error("wbc::ModelProcess::DoInit(): could not create channel");
    AddSink(m_channel, 100);
    AddSource(m_channel, 100);
    
    m_servo_status.status = wbcrun::msg::VOID_STATUS;
    m_robot_state = new wbcrun::msg::RobotState(false, npos, nvel, force_nrows, force_ncols);
    
    CreateHandler(wbcrun::msg::STATUS, "servo_status", &m_servo_status);
    CreateHandler(wbcrun::msg::ROBOT_STATE, "robot_state", m_robot_state);
    CreateHandler(wbcrun::msg::TASK_SPEC, "task_spec", &m_task_spec);
  }
  
  
  int ModelProcess::
  HandleMessagePayload(wbcnet::unique_id_t msg_id)
  {
    if (wbcrun::msg::STATUS == msg_id) {
      LOG_TRACE (logger, "wbc::ModelProcess::HandleMessagePayload(): got STATUS");
      if (wbcrun::msg::COMPUTE_MODEL == m_servo_status.status) {
	if (IDLE == m_state) {
	  LOG_TRACE (logger, "  setting state to COMPUTE");
	  m_state = COMPUTE;
	}
	else { // actually "never" happens because of blocking communication in Step()
	  LOG_TRACE (logger, "  oops, am not idle, setting state to SEND_ERROR");
	  m_state = SEND_FAILURE;
	}
      }
      else LOG_TRACE (logger,
		      "  WARNING ignoring invalid STATUS " << (int) m_servo_status.status);
    }

    else if (wbcrun::msg::ROBOT_STATE == msg_id) {
      if (logger->isTraceEnabled()) {
	ostringstream msg;
	msg << "wbc::ModelProcess::HandleMessagePayload(): got ROBOT_STATE\n";
	m_robot_state->display(msg, "  ");
	LOG_TRACE (logger, msg.str());
      }
      // just accumulate until the servo tells us msg::COMPUTE_MODEL (see above)
    }

    else if (wbcrun::msg::TASK_SPEC == msg_id) {
      if (logger->isTraceEnabled()) {
	ostringstream msg;
	msg << "wbc::ModelProcess::HandleMessagePayload(): got TASK_SPEC\n";
	m_task_spec.display(msg, "  ");
	LOG_TRACE (logger, msg.str());
      }
      // just accumulate until the servo tells us msg::COMPUTE_MODEL (see above)
    }

    else {
      LOG_TRACE (logger,
		 "wbc::ModelProcess::HandleMessagePayload()\n"
		 << "  unknown message ID " << (int) msg_id
		 << " [" << wbcrun::msg::get_id_str(msg_id));
      return 999;
    }
    
    return 0;
  }
  
  
  ModelImplementation::
  ModelImplementation(TaskModelBase * task_model,
		      RobotControlModel * robmodel,
		      std::vector<BehaviorDescription*> const & behaviors)
    : m_task_model(task_model),
      m_robmodel(robmodel),
      m_behaviors(behaviors),
      m_current_behavior(0),
      m_previous_behavior(0)
  {
  }
  
  
  bool ModelImplementation::
  ComputeModel(wbcrun::msg::RobotState const & robot_state,
	       wbcrun::msg::TaskSpec const & task_spec,
	       bool skip_behavior_update)
  {
    LOG_DEBUG (logger,
	       "wbc::ModelImplementation::ComputeModel()\n  robot_state:\n"
	       << wbcnet::displayString(robot_state, "    ")
	       << "  task_spec:\n"
	       << wbcnet::displayString(task_spec, "    "));
    
    // Retrieve current behavior. If it differs from the previous
    // one, we have to reset it.
    if (task_spec.behaviorID > m_behaviors.size()) {
      LOG_ERROR (logger,
		 "wbc::ModelImplementation::ComputeModel()\n"
		 << "  task_spec.behaviorID is " << (int) task_spec.behaviorID << "\n"
		 << "  but there are only " << m_behaviors.size() << " behaviors available");
      return false;
    }
    m_current_behavior = m_behaviors[task_spec.behaviorID];
    if (m_current_behavior != m_previous_behavior)
      m_current_behavior->reset();
    m_previous_behavior = m_current_behavior;
    
    // Regardless of whether the behavior has changed or not, the
    // task model needs to by synched with the request ID.
    if ( ! m_task_model->Reset(task_spec.requestID, *m_current_behavior)) {
      LOG_ERROR (logger,
		 "wbc::ModelImplementation::ComputeModel():\n"
		 << "  m_task_model->Reset(" << (int) task_spec.requestID << ", ...) failed");
      return false;
    }
    
    LOG_DEBUG (logger, "wbc::ModelImplementation::ComputeModel(): updating...");
    
    SAIVectorAPI const jointAngles(robot_state.jointAngles);
    SAIVectorAPI const jointVelocities(robot_state.jointVelocities);
    m_robmodel->kinematics()->onUpdate(jointAngles, jointVelocities);
    m_robmodel->dynamics()->onUpdate(jointAngles, jointVelocities); 
    
    if ( ! skip_behavior_update) {
      LOG_TRACE (logger, "ModelImplementation::ComputeModel(): updating behavior");
      m_current_behavior->onUpdate();
    }
    
    LOG_TRACE (logger, "ModelImplementation::ComputeModel(): updating task model");
    
    if ( ! m_task_model->Update(*m_current_behavior, *m_robmodel)) {
      LOG_ERROR (logger, "ModelImplementation::ComputeModel(): m_task_model->Update() failed");
      return false;
    }
    
    if (logger->isDebugEnabled()) {
      ostringstream msg;
      m_task_model->prettyPrint(msg,
				"--------------------------------------------------\n"
				"ModelImplementation::ComputeModel(): result",
				"  ");
      msg << "--------------------------------------------------";
      LOG_DEBUG (logger, msg.str());
    }
    
    return true;
  }
  
}
