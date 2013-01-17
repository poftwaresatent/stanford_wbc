/*
 * Shared copyright notice and LGPLv3 license statement.
 *
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 * Copyright (C) 2011 University of Texas at Austin. All rights reserved.
 *
 * Authors: Roland Philippsen (Stanford) and Luis Sentis (UT Austin)
 *          http://cs.stanford.edu/group/manips/
 *          http://www.me.utexas.edu/~hcrl/
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

#ifndef OPSPACE_TASK_LIBRARY_HPP
#define OPSPACE_TASK_LIBRARY_HPP

#include <opspace/Task.hpp>

namespace opspace {
  
  class TypeIOTGCursor;
  

  /**
     Base class for tasks with proportional-derivative control. It
     provides a reusable implementation for the parameters and
     algorithm of a PD controller with velocity saturation.
     
     Parameters:
     - goalpos (vector): desired position
     - goalvel (vector): desired velocity
     - kp (vector): proportional gain
     - kd (vector): derivative gain
     - maxvel (vector): velocity saturation limit
     
     Subclasses should call initPDTask() from within their init()
     method, and computePDCommand() from their computeCommand().
   */  
  class PDTask
    : public Task
  {
  public:
    /**
       Verifies that kp, kd, and maxvel are non-negative. If
       initialized, also verifies the vector dimension for kp, kd,
       maxvel, goalpos, and goalvel.
       
       \return Failure if any of the mentioned checks fail, and
       success otherwise.
    */
    virtual Status check(Vector const * param, Vector const & value) const;
    
    inline void quickSetup(Vector const & kp, Vector const & kd, Vector const & maxvel)
    {
      kp_ = kp;
      kd_ = kd;
      maxvel_ = maxvel;
    }
    
  protected:
    /**
       Velocity saturation policy.
       - SATURATION_OFF: do not saturate commands (use with caution)
       - SATURATION_COMPONENT_WISE: each component of the
         position-error is scaled according to an individual
         saturation term
       - SATURATION_MAX_COMPONENT: similar to
         SATURATION_COMPONENT_WISE, but the most saturated component
         determines the overall scaling.
       - SATURATION_NORM: the vector norm of the position error is
         used to determine the saturation factor (this only makes
         sense if kp, kd, and maxvel are one-dimensional values, so
         this constraint is enforced in other parts of the PDTask
         implementation as well)
    */
    typedef enum {
      SATURATION_OFF,
      SATURATION_COMPONENT_WISE,
      SATURATION_MAX_COMPONENT,
      SATURATION_NORM
    } saturation_policy_t;
    
    PDTask(std::string const & name, saturation_policy_t saturation_policy);
    
    /**
       Initialize the goalpos to initpos and the goalvel to zero. Also
       performs sanity checks on kp, kd, and maxvel. If saturation
       policy is opspace::PDTask::SATURATION_NORM, then kp, kd, and
       maxvel must be single-dimensional. Otherwise, if they are
       single-dimensional, this method converts them to N-dimensional
       vectors by filling them with N copies of the value. E.g. if
       kp=[100.0] and initpos is 3-dimensional, kp would end up as
       [100.0, 100.0, 100.0].
       
       \return Success if everything went well, failure otherwise.
    */
    Status initPDTask(Vector const & initpos);
    
    /**
       Compute PD command, with velocity saturation determined by the
       saturation_policy specified at construction time. This drives
       the task to achieving goalpos with goalvel.
              
       \return Success if everything went well, failure otherwise.
    */
    Status computePDCommand(Vector const & curpos,
			    Vector const & curvel,
			    Vector & command);
    
    saturation_policy_t const saturation_policy_;
    bool initialized_;
    Vector goalpos_;
    Vector goalvel_;
    Vector errpos_;
    Vector errvel_;
    Vector kp_;
    Vector kd_;
    Vector maxvel_;
  };
  
  
  class DraftPIDTask
    : public PDTask
  {
  public:
    explicit DraftPIDTask(std::string const & name);
    
    virtual Status check(double const * param, double value) const;
    virtual Status check(Vector const * param, Vector const & value) const;
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    Status initDraftPIDTask(Vector const & initpos);
    
    Status computeDraftPIDCommand(Vector const & curpos,
				  Vector const & curvel,
				  Vector & command);
    
    double dt_seconds_;
    Vector ki_;
    Vector errsum_;
    Vector limitpos_;
    Vector limitvel_;
    Vector triggerpos_;
  };
  
  
  /**
     Cartesian position task. Servos a control point, specified with
     respect to a given end_effector link, to the goal position.
     
     \note This task is always three dimensional, and it relies on the
     PDTask::SATURATION_NORM policy, so the gains and maxvel have to
     be one-dimensional.
     
     Parameters (see also PDTask for inherited parameters):
     - end_effector (string): name of the end effector link
     - control_point (vector): reference point wrt end effector frame
  */
  class CartPosTask
    : public PDTask
  {
  public:
    explicit CartPosTask(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual Status check(std::string const * param, std::string const & value) const;
    
    inline void quickSetup(Vector const & kp, Vector const & kd, Vector const & maxvel,
			   std::string const & name, Vector const & control_point)
    {
      PDTask::quickSetup(kp, kd, maxvel);
      end_effector_name_ = name;
      control_point_ = control_point;
    }
    
  protected:
    std::string end_effector_name_;
    Vector control_point_;
    
    mutable taoDNode const * end_effector_node_;
    
    taoDNode const * updateActual(Model const & model);
  };
  
  
  /**
     Joint-space posture task. Servos the joint position towards a
     desired posture using acceleration-bounded trajectories.
     
     \note Uses component-wise velocity saturation.

     Parameters: inherited from PDTask.
  */
  class JPosTask
    : public PDTask
  {
  public:
    explicit JPosTask(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
  };
  
  
  /**
     A test task which drives a subset of DOF to zero using
     non-saturated PD control.
     
     Parameters:
     - selection (vector): values > 0.5 switch on the corresponding DOF
     - kp (vector): proportional gain
     - kd (vector): derivative gain
  */
  class SelectedJointPostureTask
    : public Task
  {
  public:
    explicit SelectedJointPostureTask(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual Status check(double const * param, double value) const;
    virtual Status check(Vector const * param, Vector const & value) const;
    
  protected:
    Vector selection_;
    double kp_;
    double kd_;
    bool initialized_;
    std::vector<size_t> active_joints_;
  };
  
  
  /**
     Base class for acceleration-bounded trajectory tasks. Uses a PD
     control law to follow a trajectory generated using the
     reflexxes_otg library,
     
     Parameters inherited from PDTask (this list may be outdated):
     - goalpos (vector): desired position
     - goalvel (vector): desired velocity
     - kp (vector): proportional gain
     - kd (vector): derivative gain
     - maxvel (vector): velocity saturation limit

     Parameters added by this base class:
     - dt_seconds (real): iteration timestep, for trajectory generation
     - trjgoal (vector): trajectory end point
     - maxacc (vector): maximum acceleration of the trajectory
     
     Subclasses should call initTrajectoryTask() from their init(),
     and computeTrajectoryCommand() from their computeCommand().
  */
  class TrajectoryTask
    : public PDTask
  {
  public:
    virtual ~TrajectoryTask();
    
    /**
       Checks that dt_seconds is positive and calls PDTask.
    */
    virtual Status check(double const * param, double value) const;
    
    /**
       If initialized, checks the validity of trjgoal and calls
       PDTask::check().
    */
    virtual Status check(Vector const * param, Vector const & value) const;
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    typedef PDTask::saturation_policy_t saturation_policy_t;
    
    TrajectoryTask(std::string const & name, saturation_policy_t saturation_policy);
    
    /**
       Initializes the trajectory to be at the current position with
       zero velocity. Also does some sanity checking and optional
       conversion of parameters. Similarly to PDTask::initPDTask(),
       single-dimensional parameters can get blown up to the right
       size, depending on the saturation policy.
    */
    Status initTrajectoryTask(Vector const & initpos);
    
    /**
       Computes the command for following the trajectory. It advances
       the cursor by dt_seconds toward the trjgoal and servos to that
       position and velocity using PDTask::computePDCommand().
    */
    Status computeTrajectoryCommand(Vector const & curpos,
				    Vector const & curvel,
				    Vector & command);
    
    TypeIOTGCursor * cursor_;
    double dt_seconds_;
    Vector trjgoal_;
    Vector maxacc_;
    mutable Vector qh_maxvel_;	// grr, cursor always needs multi-dim maxvel, even if saturation_norm
  };
  
  
  /**
     Cartesian position trajectory task. Servos a control point,
     specified with respect to a given end_effector link, to the goal
     position.
     
     \note This task is always three dimensional.
     
     Parameters (see also TrajectoryTask for inherited parameters):
     - end_effector_id (integer): identifier of the end effector link
     - control_point (vector): reference point wrt end effector frame
  */
  class CartPosTrjTask
    : public TrajectoryTask
  {
  public:
    explicit CartPosTrjTask(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    
  protected:
    int end_effector_id_;
    Vector control_point_;
    
    taoDNode const * updateActual(Model const & model);
  };
  
  
  /**
     Joint-space posture trajectory task. Servos the joint position
     towards a desired posture using acceleration-bounded
     trajectories.
     
     Parameters: inherited from TrajectoryTask.
  */
  class JPosTrjTask
    : public TrajectoryTask
  {
  public:
    explicit JPosTrjTask(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    
    /**
       \todo Maybe move this (or something similar) to the superclass?
    */
    void quickSetup(double dt_seconds, double kp, double kd, double maxvel, double maxacc);
  };


  class JointLimitTask
    : public Task
  {
  public:
    explicit JointLimitTask(std::string const & name);
    virtual ~JointLimitTask();
    
    virtual Status check(Vector const * param, Vector const & value) const;
    virtual Status check(double const * param, double const & value) const;
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    // parameters
    Vector upper_stop_deg_;
    Vector upper_trigger_deg_;
    Vector lower_stop_deg_;
    Vector lower_trigger_deg_;
    double dt_seconds_;
    Vector maxvel_;
    Vector maxacc_;
    Vector kp_;
    Vector kd_;
    
    // non-parameters
    Vector upper_stop_;
    Vector upper_trigger_;
    Vector lower_stop_;
    Vector lower_trigger_;
    
    std::vector<TypeIOTGCursor *> cursor_;
    Vector goal_;

    void updateState(Model const & model);
  };
  
  
  class OrientationTask
    : public Task
  {
  public:
    explicit OrientationTask(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    taoDNode const * updateActual(Model const & model);
    
    //    Vector goal_;		// direction cosines
    Eigen::Vector3d goal_x_;
    Eigen::Vector3d goal_y_;
    Eigen::Vector3d goal_z_;
    Eigen::Vector3d actual_x_;
    Eigen::Vector3d actual_y_;
    Eigen::Vector3d actual_z_;
    Vector velocity_;		// instantaneous omega
    Vector delta_;		// instantaneous angular difference
    int end_effector_id_;
    double kp_;
    double kd_;
    double maxvel_;
    Vector eepos_;		// just for logging...
  };

  
}

#endif // OPSPACE_TASK_LIBRARY_HPP
