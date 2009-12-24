/*
 * Copyright (c) 2009 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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
   \file ServoProcess.hpp
   \author Roland Philippsen (execution control) and Luis Sentis (servo algorithm)
*/

#ifndef WBC_SERVO_PROCESS_HPP
#define WBC_SERVO_PROCESS_HPP

#include <wbcrun/Process.hpp>
#include <wbcnet/msg/Service.hpp>
#include <wbcrun/msg/Status.hpp>
#include <wbcrun/msg/TaskSpec.hpp>
#include <wbc/core/SAIVectorAPI.hpp>
#include <wbc/core/SAIMatrixAPI.hpp>
#include <wbcrun/directory.hpp>
#include <wbcrun/msg/RobotState.hpp>

class taoDNode;

namespace wbcnet {
  class Channel;
  class DelayHistogram;
  class NetConfig;
}


namespace wbc {

  class TaskModelBase;
  class RobotAPI;
  class DirectoryCmdServer;
  class Dynamics;
  class Kinematics;
  class Contact;
  class ServoBehaviorsAPI;
  class TaskModelListener;
  class BehaviorDescription;
  class RobotControlModel;
  class BranchingRepresentation;
  
  
  /** \note Just for implementing DirectoryCmdServer in a way that is
      agnostic of multi-rate or one-process servo/model setup... BUT
      this quick rfct introduces multiple inheritance for ServoProcess
      and ServoModelProcess. */
  class ServoProcessAPI
  {
  public:
    virtual ~ServoProcessAPI() {}
    virtual wbcnet::srv_result_t BeginBehaviorTransition(int behaviorID) = 0;
    virtual BranchingRepresentation * GetBranching() = 0;
    virtual Kinematics * GetKinematics() = 0;
    virtual SAIVector const & GetCommandTorques() = 0;
  };
  
  
  class ServoImplementation
  {
  public:
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
			int pskip);

    virtual ~ServoImplementation();
    
    virtual bool UpdateRobotState(wbcrun::msg::RobotState & state);
    
    /** When no behavior or model is ready to run, this gets called
	instead of UpdateTorqueCommand(). Subclasses must send a null
	torque command to the robot (which might be to brake the
	joints or whatever is appropriate during initialization of the
	system). */
    virtual bool NullTorqueCommand();
    
    /** Subclasses must compute the torque command based on the given
	model and behavior. The calling process must garantee that the
	model corresponds to the behavior and is up to date. */
    virtual bool UpdateTorqueCommand(TaskModelBase const * model,
				     int behaviorID,
				     /** Set this to true to AVOID
					 calling
					 BehaviorDescription::onUpdate()
					 of the current behavior
					 inside this method. This is
					 useful e.g. in case the
					 behavior already got updated
					 elsewhere. */
				     bool skip_behavior_update);

    /** Work in progress... requires subclasses to reset the
	next_task_model with the provided requestID and the nlayers of
	the given behaviorID. */
    virtual bool ResetBehavior(TaskModelBase * next_task_model,
			       uint8_t requestID,
			       int behaviorID);
    
    /** \todo ...serialization stuff should not concern servo implementers... */
    virtual class TaskModelListener * GetTaskModelListener();
    
    std::vector<BehaviorDescription*> const & GetBehaviorLibrary() { return m_behavior; }
    
    
  protected:
    // Should maybe fuse these two classes (they used to be one
    // anyway, back in the days of innocence).
    friend class ServoProcess;
    
    // ...but then this should be fused as well, and it's kinda nice
    // to have the process and communication logic rather separate
    // from the number crunching...
    friend class ServoModelProcess;
    
    timeval m_robot_state_tstamp;
    SAIVectorAPI m_joint_angles;
    SAIVectorAPI m_joint_velocities;
    SAIMatrixAPI m_contact_forces;
    SAIVectorAPI m_command_torques;

    Dynamics * m_dynamics;
    Kinematics * m_kinematics;
    Contact * m_contact;

    BehaviorDescription * m_current_behavior;
    ServoBehaviorsAPI * m_servoBehaviors;
    
    std::vector<BehaviorDescription*> const & m_behavior;
    TaskModelListener * m_task_model_listener;
    RobotAPI * m_robotAPI;
    size_t m_ndof_actuated;

    wbcnet::DelayHistogram * m_dhist;
    int m_pskip;
    
    wbcrun::msg::RobotState m_robot_state;
    RobotControlModel * m_robmodel;
    taoDNode * m_end_effector;
  };
  
  
  class ServoProcess
    : public wbcrun::Process,
      public ServoProcessAPI
  {
  public:
    ServoProcess();
    ~ServoProcess();
    
    int HandleMessagePayload(wbcnet::unique_id_t msg_id);
    
    bool Step() throw(std::exception);
    
    void Init(ServoImplementation * imp,
	      /** if true, the imp will be deleted in our destructor */
	      bool own_imp,
	      wbcnet::NetConfig const & netconf,
	      uint8_t npos, uint8_t nvel,
	      uint8_t force_nrows, uint8_t force_ncols) throw(std::exception); 
    
    /** Mostly for test and debug. */
    wbcrun::msg::RobotState const & GetRobotState() const { return *m_robot_state; }
    
    /**
       Checks that it is OK to transition to the wanted behavior,
       notifies subclasses using ResetBehavior(), resets and prepares
       the next task model, and then prepares the model messages and
       enqueues it.
    */
    virtual wbcnet::srv_result_t BeginBehaviorTransition(int behaviorID);
    
    virtual BranchingRepresentation * GetBranching();
    virtual Kinematics * GetKinematics();
    virtual SAIVector const & GetCommandTorques();
    
    /** Access internal data for testing and debugging. */
    int GetCurrentBehaviorID() const { return m_current_behaviorID; }
    
    /** Access internal data for testing and debugging. */
    int GetNextBehaviorID() const { return m_next_behaviorID; }
    
    /** Access internal data for testing and debugging. */
    uint8_t GetBehaviorTransitionRequestID() const { return m_behavior_transition_requestID; }
    
    
  protected:
    friend class ModelServoTest;
    
    typedef enum {
      READY_STATE,	       /**< initialized, but no behavior chosen yet */
      WAIT_MODEL_STATE,      /**< waiting for the first model update to flush through the system */
      RUNNING_STATE,	       /**< running a behavior */
      ERROR_STATE	       /**< placeholder for later extension */
    } state_t;
    
    DirectoryCmdServer * m_directory_cmd_server;
    
    ServoImplementation * m_imp;
    bool m_own_imp;
    
    state_t m_state;
    
    wbcnet::Channel * m_model_channel;
    wbcnet::Channel * m_user_channel;
    TaskModelListener * m_model_listener;
    
    int m_current_behaviorID;
    int m_next_behaviorID;
    uint8_t m_behavior_transition_requestID;
    bool m_init_behavior_transition;
    
    // incoming messages
    wbcrun::msg::Status m_model_status;
    wbcrun::msg::TaskSpec m_user_task_spec;
    wbcnet::msg::Service m_user_request;
    
    // outgoing messages
    wbcrun::msg::Status m_servo_status;
    wbcrun::msg::RobotState * m_robot_state;
    wbcrun::msg::TaskSpec m_model_task_spec;
    wbcnet::msg::Service m_user_reply;
  };
  
}

#endif // WBC_SERVO_PROCESS_HPP
