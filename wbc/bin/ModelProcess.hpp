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

#ifndef WBC_MODEL_PROCESS_HPP
#define WBC_MODEL_PROCESS_HPP

#include <wbcrun/Process.hpp>
#include <wbcrun/msg/Status.hpp>
#include <wbcrun/msg/TaskSpec.hpp>
#include <wbcrun/msg/TaskMatrix.hpp>

namespace wbcnet {
  class Channel;
  class NetConfig;
}

namespace wbcrun {
  namespace msg {
    class RobotState;
  }
  class TaskModelAPI;
}
  

namespace wbc {

  class TaskModelBase;
  class RobotControlModel;
  class BehaviorDescription;


  class ModelImplementation
  {
  public:
    ModelImplementation(TaskModelBase * task_model,
			RobotControlModel * robmodel,
			std::vector<BehaviorDescription*> const & behaviors);
    
    /**
       - reset the model according to the task_spec
       - update it according to the robot_state
       - make sure that upon serialization, the task_spec.requestID
         gets send over the wire in the reply, so that the
         ServoProcess can detect ends of behavior transitions
       
       \return true if everything went well, false if an error
       occurred, in which case we keep running but send
       msg::MODEL_ERROR as status.
     */
    bool ComputeModel(wbcrun::msg::RobotState const & robot_state,
		      wbcrun::msg::TaskSpec const & task_spec,
		      /** Set this to true to AVOID calling
			  BehaviorDescription::onUpdate() of the
			  current behavior inside this method. This is
			  useful e.g. in case the behavior already got
			  updated elsewhere. */
		      bool skip_behavior_update);
    
    /** \todo ...work in progress... needed for serialization, should
	not concern the model implementation */
    inline TaskModelBase const * GetTaskModel() const { return m_task_model; }
    
  protected:
    TaskModelBase * m_task_model;
    RobotControlModel * m_robmodel;
    std::vector<BehaviorDescription*> const & m_behaviors;
    BehaviorDescription * m_current_behavior;
    BehaviorDescription * m_previous_behavior;
  };
  
  
  class ModelProcess
    : public wbcrun::Process
  {
  public:
    ModelProcess();
    ~ModelProcess();
    
    int HandleMessagePayload(wbcnet::unique_id_t msg_id);
    
    bool Step() throw(std::exception);
    
    /**
       e.g.
       \code
       static string const comspec("mq");
       NetConfig * netconf(NetConfig::Create(comspec));
       if ( ! netcfg)
         throw runtime_error("NetConfig::Create(" + comspec + ") failed");
       try {
         model_process.Init(*netconf);
       }
       catch (...) {
         delete netconf;
	 throw;
       }
       delete netconf;
       \endcode
    */
    void Init(ModelImplementation * imp,
	      /** if true, the ModelImplementationAPI will be deleted in the destructor */
	      bool own_imp,
	      wbcnet::NetConfig const & netconf,
	      uint8_t npos, uint8_t nvel,
	      uint8_t force_nrows, uint8_t force_ncols) throw(std::exception);
    
    wbcrun::msg::RobotState const & GetRobotState() const { return *m_robot_state; }
    
  protected:
    friend class ModelServoTest;
    
    typedef enum {
      IDLE,
      COMPUTE,
      SEND_SUCCESS,
      SEND_FAILURE
    } state_t;
    
    ModelImplementation * m_imp;
    bool m_own_imp;
    
    state_t m_state;
    wbcnet::Channel * m_channel;
    
    // incoming messages
    wbcrun::msg::Status m_servo_status;
    wbcrun::msg::RobotState * m_robot_state; // cannot allocate until ndof etc are known
    wbcrun::msg::TaskSpec m_task_spec;
    
    // outgoing messages
    wbcrun::msg::Status m_model_status;
    wbcrun::msg::TaskMatrix m_task_matrix;
  };
  
}

#endif // WBC_MODEL_PROCESS_HPP
