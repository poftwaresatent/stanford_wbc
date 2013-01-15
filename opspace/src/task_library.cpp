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

#include <opspace/task_library.hpp>
#include <opspace/TypeIOTGCursor.hpp>

using jspace::pretty_print;

namespace opspace {


  PDTask::
  PDTask(std::string const & name,
	 saturation_policy_t saturation_policy)
    : Task(name),
      saturation_policy_(saturation_policy),
      initialized_(false)
  {
    declareParameter("goalpos", &goalpos_);
    declareParameter("goalvel", &goalvel_);
    declareParameter("errpos", &errpos_);
    declareParameter("errvel", &errvel_);
    declareParameter("kp", &kp_, PARAMETER_FLAG_NOLOG);
    declareParameter("kd", &kd_, PARAMETER_FLAG_NOLOG);
    declareParameter("maxvel", &maxvel_, PARAMETER_FLAG_NOLOG);
  }
  
  
  Status PDTask::
  check(Vector const * param, Vector const & value) const
  {
    if ((param == &errpos_) || (param == &errvel_)) {
      return Status(false, "error signals are read-only");
    }
    if ((param == &kp_) || (param == &kd_) || (param == &maxvel_)) {
      if (SATURATION_NORM == saturation_policy_) {
	if (1 != value.rows()) {
	  return Status(false, "SATURATION_NORM requires one-dimensional gains and limits");
	}
      }
      for (size_t ii(0); ii < value.rows(); ++ii) {
	if (0 > value[ii]) {
	  return Status(false, "gains and limits must be >= 0");
	}
      }
    }
    if (initialized_) {
      if (SATURATION_NORM != saturation_policy_) {
	if ((param == &kp_) || (param == &kd_) || (param == &maxvel_)) {
	  if (goalpos_.rows() != value.rows()) {
	    return Status(false, "invalid dimension");
	  }
	}
      }
      if ((param == &goalpos_) || (param == &goalvel_)) {
	if (goalpos_.rows() != value.rows()) {
	  return Status(false, "invalid dimension");
	}
      }
    }
    return Status();
  }
  
  
  Status PDTask::
  initPDTask(Vector const & initpos)
  {
    int const ndim(initpos.rows());
    
    if (SATURATION_NORM == saturation_policy_) {
      if (1 != kp_.rows()) {
	return Status(false, "kp must be one-dimensional for SATURATION_NORM policy");
      }
      if (1 != kd_.rows()) {
	return Status(false, "kd must be one-dimensional for SATURATION_NORM policy");
      }
      if (1 != maxvel_.rows()) {
	return Status(false, "maxvel must be one-dimensional for SATURATION_NORM policy");
      }
    }
    else {
      if (ndim != kp_.rows()) {
	if ((ndim != 1) && (1 == kp_.rows())) {
	  kp_ = kp_[0] * Vector::Ones(ndim);
	}
	else {
	  return Status(false, "invalid kp dimension");
	}
      }
      if (ndim != kd_.rows()) {
	if ((ndim != 1) && (1 == kd_.rows())) {
	  kd_ = kd_[0] * Vector::Ones(ndim);
	}
	else {
	  return Status(false, "invalid kd dimension");
	}
      }
      if (ndim != maxvel_.rows()) {
	if ((ndim != 1) && (1 == maxvel_.rows())) {
	  maxvel_ = maxvel_[0] * Vector::Ones(ndim);
	}
	else {
	  return Status(false, "invalid maxvel dimension");
	}
      }
    }
    
    goalpos_ = initpos;
    goalvel_ = Vector::Zero(ndim);
    errpos_ = Vector::Zero(ndim);
    errvel_ = Vector::Zero(ndim);
    initialized_ = true;
    
    Status ok;
    return ok;
  }
  
  
  Status PDTask::
  computePDCommand(Vector const & curpos,
		   Vector const & curvel,
		   Vector & command)
  {
    Status st;
    if ( ! initialized_) {
      st.ok = false;
      st.errstr = "not initialized";
      return st;
    }
    
    errpos_ = goalpos_ - curpos;
    errvel_ = goalvel_ - curvel;
    
    if (SATURATION_NORM == saturation_policy_) {
      command = kp_[0] * errpos_;
      if ((maxvel_[0] > 1e-4) && (kd_[0] > 1e-4)) { // beware of div by zero
	double const sat(command.norm() / maxvel_[0] / kd_[0]);
	if (sat > 1.0) {
	  command /= sat;
	}
      }
      command += kd_[0] * errvel_;
      return st;
    }    
    
    command = kp_.array() * errpos_.array();
    
    if (SATURATION_COMPONENT_WISE == saturation_policy_) {
      for (int ii(0); ii < command.rows(); ++ii) {
	if ((maxvel_[ii] > 1e-4) && (kd_[ii] > 1e-4)) { // beware of div by zero
	  double const sat(fabs((command[ii] / maxvel_[ii]) / kd_[ii]));
	  if (sat > 1.0) {
	    command[ii] /= sat;
	  }
	}
      }
    }
    
    else if (SATURATION_MAX_COMPONENT == saturation_policy_) {
      double saturation(0.0);
      for (int ii(0); ii < command.rows(); ++ii) {
	if ((maxvel_[ii] > 1e-4) && (kd_[ii] > 1e-4)) { // beware of div by zero
	  double const sat(fabs((command[ii] / maxvel_[ii]) / kd_[ii]));
	  if (sat > saturation) {
	    saturation = sat;
	  }
	}
      }
      if (saturation > 1.0) {
	command /= saturation;
      }
    }
    
    // else (...) : other saturation policies would go here. For now
    // we silently assume that any other value of saturation_policy
    // means SATURATION_OFF.
    
    command.array() += kd_.array() * errvel_.array();
    
    return st;
  }
  
  
  DraftPIDTask::
  DraftPIDTask(std::string const & name)
    : PDTask(name, PDTask::SATURATION_COMPONENT_WISE),
      dt_seconds_(-1)
  {
    declareParameter("dt_seconds", &dt_seconds_, PARAMETER_FLAG_NOLOG);
    declareParameter("ki", &ki_, PARAMETER_FLAG_NOLOG);
    declareParameter("errsum", &errsum_);
    declareParameter("limitpos", &limitpos_, PARAMETER_FLAG_NOLOG);
    declareParameter("limitvel", &limitvel_, PARAMETER_FLAG_NOLOG);
    declareParameter("triggerpos", &triggerpos_);
  }
  
  
  Status DraftPIDTask::
  check(double const * param, double value) const
  {
    if (param == &dt_seconds_) {
      if (0 >= value) {
	return Status(false, "dt_seconds must be > 0");
      }
    }
    return Status();
  }
  
  
  Status DraftPIDTask::
  check(Vector const * param, Vector const & value) const
  {
    if (param == &triggerpos_) {
      return Status(false, "triggerpos is read-only");
    }
    if (param == &errsum_) {
      return Status(false, "errsum is read-only");
    }
    if ((param == &ki_) || (param == &limitpos_) || (param == &limitvel_)) {
      for (size_t ii(0); ii < value.rows(); ++ii) {
	if (0 > value[ii]) {
	  return Status(false, "gains and limits must be >= 0");
	}
      }
      if (initialized_) {
	if (goalpos_.rows() != value.rows()) {
	  return Status(false, "invalid dimension");
	}
      }
    }
    return Status();
  }
  
  
  Status DraftPIDTask::
  initDraftPIDTask(Vector const & initpos)
  {
    Status st(initPDTask(initpos));
    if ( ! st) {
      return st;
    }
    
    if (0 >= dt_seconds_) {
      return Status(false, "dt_seconds_ must be > 0");
    }
    for (size_t ii(0); ii < ki_.rows(); ++ii) {
      if (0 > ki_[ii]) {
	return Status(false, "ki must be >= 0");
      }
    }
    for (size_t ii(0); ii < limitpos_.rows(); ++ii) {
      if (0 > limitpos_[ii]) {
	return Status(false, "limitpos must be >= 0");
      }
    }
    for (size_t ii(0); ii < limitvel_.rows(); ++ii) {
      if (0 > limitvel_[ii]) {
	return Status(false, "limitvel must be >= 0");
      }
    }
    
    int const ndim(initpos.rows());
    if (ndim != ki_.rows()) {
      if ((ndim != 1) && (1 == ki_.rows())) {
	ki_ = ki_[0] * Vector::Ones(ndim);
      }
      else {
	return Status(false, "invalid ki dimension");
      }
    }
    if (ndim != limitpos_.rows()) {
      if ((ndim != 1) && (1 == limitpos_.rows())) {
	limitpos_ = limitpos_[0] * Vector::Ones(ndim);
      }
      else {
	return Status(false, "invalid limitpos dimension");
      }
    }
    if (ndim != limitvel_.rows()) {
      if ((ndim != 1) && (1 == limitvel_.rows())) {
	limitvel_ = limitvel_[0] * Vector::Ones(ndim);
      }
      else {
	return Status(false, "invalid limitvel dimension");
      }
    }
    
    errsum_ = Vector::Zero(ndim);
    triggerpos_ = - 1.0 * Vector::Ones(ndim);
    
    return st;
  }
  
  
  Status DraftPIDTask::
  computeDraftPIDCommand(Vector const & curpos,
			 Vector const & curvel,
			 Vector & command)
  {
    Status st(computePDCommand(curpos, curvel, command));
    if ( ! st) {
      return st;
    }
    
    // update "state" for each DOF: if triggerpos_[ii] is positive
    // after this loop, it means we should enable the integral term
    // for it further down.
    for (int ii(0); ii < triggerpos_.rows(); ++ii) {
      if (triggerpos_[ii] > 0) {
	// integral term currently ON
	if (fabs(errpos_[ii]) > triggerpos_[ii]) {
	  // overshoot: switch off integration
	  triggerpos_[ii] = -1;
	}
	else if (fabs(errvel_[ii]) > 2.0 * limitvel_[ii]) {
	  // too fast: switch off integration
	  triggerpos_[ii] = -1;
	}
	// else it remains on
      }
      else {
	// integral term currently OFF
	if ((fabs(errpos_[ii]) < limitpos_[ii])
	    && (fabs(errvel_[ii]) < limitvel_[ii])) {
	  // close to the goal: switch on integration
	  triggerpos_[ii] = 2.0 * fabs(errpos_[ii]);
	  errsum_[ii] = 0;
	}
      }
    }
    
    // for each DOF with integral term enabled, add it to the command
    for (int ii(0); ii < triggerpos_.rows(); ++ii) {
      if (triggerpos_[ii] > 0) {
	errsum_[ii] += errpos_[ii] * dt_seconds_;
	command[ii] += ki_[ii] * errsum_[ii];
      }
    }
    
    return st;
  }
  
  
  Status DraftPIDTask::
  init(Model const & model)
  {
    jacobian_ = Matrix::Identity(model.getNDOF(), model.getNDOF());
    actual_ = model.getState().position_;
    return initDraftPIDTask(model.getState().position_);
  }
  
  
  Status DraftPIDTask::
  update(Model const & model)
  {
    actual_ = model.getState().position_;
    return computeDraftPIDCommand(actual_,
				  model.getState().velocity_,
				  command_);
  }
  
  
  void DraftPIDTask::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "draft PID task: `" << instance_name_ << "'\n";
    pretty_print(errpos_, os, prefix + "  errpos", prefix + "    ");
    pretty_print(triggerpos_, os, prefix + "  triggerpos", prefix + "    ");
    pretty_print(ki_, os, prefix + "  ki", prefix + "    ");
    pretty_print(errsum_, os, prefix + "  errsum", prefix + "    ");
  }

  
  
  CartPosTask::
  CartPosTask(std::string const & name)
    : PDTask(name, PDTask::SATURATION_NORM),
      end_effector_name_(""),
      control_point_(Vector::Zero(3)),
      end_effector_node_(0)
  {
    declareParameter("end_effector", &end_effector_name_, PARAMETER_FLAG_NOLOG);
    declareParameter("control_point", &control_point_, PARAMETER_FLAG_NOLOG);
  }
  
  
  Status CartPosTask::
  init(Model const & model)
  {
    if (end_effector_name_.empty()) {
      return Status(false, "no end_effector");
    }
    if (3 != control_point_.rows()) {
      return Status(false, "control_point must have 3 dimensions");
    }
    end_effector_node_ = updateActual(model);
    if ( ! end_effector_node_) {
      return Status(false, "invalid end_effector");
    }
    return initPDTask(actual_);
  }
  
  
  Status CartPosTask::
  update(Model const & model)
  {
    end_effector_node_ = updateActual(model);
    if ( ! end_effector_node_) {
      return Status(false, "invalid end_effector");
    }
    
    Matrix Jfull;
    if ( ! model.computeJacobian(end_effector_node_, actual_[0], actual_[1], actual_[2], Jfull)) {
      return Status(false, "failed to compute Jacobian (unsupported joint type?)");
    }
    jacobian_ = Jfull.block(0, 0, 3, Jfull.cols());
    
    return computePDCommand(actual_,
			    jacobian_ * model.getState().velocity_,
			    command_);
  }
  
  
  Status CartPosTask::
  check(std::string const * param, std::string const & value) const
  {
    if (param == &end_effector_name_) {
      end_effector_node_ = 0; // lazy re-init... would be nice to detect errors here though, but need model
    }
    Status ok;
    return ok;
  }
  
  
  taoDNode const * CartPosTask::
  updateActual(Model const & model)
  {
    if ( ! end_effector_node_) {
      end_effector_node_ = model.getNodeByName(end_effector_name_);
    }
    if (end_effector_node_) {
      jspace::Transform ee_transform;
      model.computeGlobalFrame(end_effector_node_,
			       control_point_[0],
			       control_point_[1],
			       control_point_[2],
			       ee_transform);
      actual_ = ee_transform.translation();
    }
    return end_effector_node_;
  }
  
  
  JPosTask::
  JPosTask(std::string const & name)
    : PDTask(name, PDTask::SATURATION_COMPONENT_WISE)
  {
  }
  
  
  Status JPosTask::
  init(Model const & model)
  {
    jacobian_ = Matrix::Identity(model.getNDOF(), model.getNDOF());
    actual_ = model.getState().position_;
    return initPDTask(model.getState().position_);
  }
  
  
  Status JPosTask::
  update(Model const & model)
  {
    actual_ = model.getState().position_;
    return computePDCommand(actual_,
			    model.getState().velocity_,
			    command_);
  }
  
  
  SelectedJointPostureTask::
  SelectedJointPostureTask(std::string const & name)
    : Task(name),
      kp_(100.0),
      kd_(20.0),
      initialized_(false)
  {
    declareParameter("selection", &selection_, PARAMETER_FLAG_NOLOG);
    declareParameter("kp", &kp_, PARAMETER_FLAG_NOLOG);
    declareParameter("kd", &kd_, PARAMETER_FLAG_NOLOG);
  }
  
  
  Status SelectedJointPostureTask::
  init(Model const & model)
  {
    size_t const ndof(model.getNDOF());
    active_joints_.clear();	// in case we get called multiple times
    for (size_t ii(0); ii < selection_.rows(); ++ii) {
      if (ii >= ndof) {
	break;
      }
      if (selection_[ii] > 0.5) {
	active_joints_.push_back(ii);
      }
    }
    if (active_joints_.empty()) {
      return Status(false, "no active joints");
    }
    size_t const ndim(active_joints_.size());
    actual_ = Vector::Zero(ndim);
    command_ = Vector::Zero(ndim);
    jacobian_ = Matrix::Zero(ndim, ndof);
    for (size_t ii(0); ii < ndim; ++ii) {
      actual_[ii] = model.getState().position_[active_joints_[ii]];
      jacobian_.coeffRef(ii, active_joints_[ii]) = 1.0;
    }
    initialized_ = true;
    Status ok;
    return ok;
  }
  
  
  Status SelectedJointPostureTask::
  update(Model const & model)
  {
    Status st;
    if ( ! initialized_) {
      st.ok = false;
      st.errstr = "not initialized";
      return st;
    }
    Vector vel(actual_.rows());
    for (size_t ii(0); ii < active_joints_.size(); ++ii) {
      actual_[ii] = model.getState().position_[active_joints_[ii]];
      vel[ii] = model.getState().velocity_[active_joints_[ii]];
    }
    command_ = -kp_ * actual_ - kd_ * vel;
    return st;
  }
  
  
  Status SelectedJointPostureTask::
  check(double const * param, double value) const
  {
    Status st;
    if (((&kp_ == param) && (value < 0)) || ((&kd_ == param) && (value < 0))) {
      st.ok = false;
      st.errstr = "gains must be >= 0";
    }
    return st;
  }
  
  
  Status SelectedJointPostureTask::
  check(Vector const * param, Vector const & value) const
  {
    Status st;
    if ((&selection_ == param) && (value.rows() == 0)) {
      st.ok = false;
      st.errstr = "selection must not be empty";
    }
    return st;
  }
  
  
  TrajectoryTask::
  TrajectoryTask(std::string const & name, saturation_policy_t saturation_policy)
    : PDTask(name, saturation_policy),
      cursor_(0),
      dt_seconds_(-1)
  {
    declareParameter("dt_seconds", &dt_seconds_, PARAMETER_FLAG_NOLOG);
    declareParameter("trjgoal", &trjgoal_);
    declareParameter("maxacc", &maxacc_, PARAMETER_FLAG_NOLOG);
  }
  
  
  TrajectoryTask::
  ~TrajectoryTask()
  {
    delete cursor_;
  }
  
  
  Status TrajectoryTask::
  initTrajectoryTask(Vector const & initpos)
  {
    Status st(initPDTask(initpos));
    if ( ! st) {
      return st;
    }
    
    if (0 > dt_seconds_) {
      st.ok = false;
      st.errstr = "you did not (correctly) set dt_seconds";
      return st;
    }
    
    int const ndim(initpos.rows());
    if (ndim != maxacc_.rows()) {
      if ((ndim != 1) && (1 == maxacc_.rows())) {
	maxacc_ = maxacc_[0] * Vector::Ones(ndim);
      }
      else {
	return Status(false, "invalid maxacc dimension");
      }
    }
    
    if (cursor_) {
      if (cursor_->dt_seconds_ != dt_seconds_) {
	delete cursor_;
	cursor_ = 0;
      }
    }
    if ( ! cursor_) {
      cursor_ = new TypeIOTGCursor(ndim, dt_seconds_);
    }
    
    trjgoal_ = initpos;
    cursor_->position() = initpos;
    cursor_->velocity() = Vector::Zero(ndim);
    
    if (SATURATION_NORM == saturation_policy_) {
      qh_maxvel_ = maxvel_[0] * Vector::Ones(ndim);
    }
    else {
      qh_maxvel_ = maxvel_;
    }
    
    return st;
  }
  
  
  Status TrajectoryTask::
  computeTrajectoryCommand(Vector const & curpos,
			   Vector const & curvel,
			   Vector & command)
  {
    if ( ! cursor_) {
      return Status(false, "not initialized");
    }
    
    int const trjstatus(cursor_->next(qh_maxvel_, maxacc_, trjgoal_));
    if (0 > trjstatus) {
      std::ostringstream msg;
      msg << "trajectory generation error code "
	  << trjstatus << ": " << otg_errstr(trjstatus);
      return Status(false, msg.str());
    }
    
    goalpos_ = cursor_->position();
    goalvel_ = cursor_->velocity();
    
    return computePDCommand(curpos, curvel, command);
  }
  
  
  Status TrajectoryTask::
  check(double const * param, double value) const
  {
    if ((param == &dt_seconds_) && (0 >= value)) {
      return Status(false, "dt_seconds must be > 0");
    }
    return Status(); // if we had one in the superclass, we'd call PDTask::check(param, value);
  }
  
  
  Status TrajectoryTask::
  check(Vector const * param, Vector const & value) const
  {
    Status st(PDTask::check(param, value));
    if ( ! st) {
      return st;
    }
    
    if ( ! cursor_) {
      return st;
    }
    
    if (param == &trjgoal_) {
      if (cursor_->ndof_ != value.rows()) {
	return Status(false, "invalid goal dimension");
      }
    }
    
    if (param == &maxacc_) {
      if (cursor_->ndof_ != value.rows()) {
	return Status(false, "invalid maxacc dimension");
      }
      for (size_t ii(0); ii < value.rows(); ++ii) {
	if (0 > value[ii]) {
	  return Status(false, "maxacc must be >= 0");
	}
      }
    }
    
    if (param == &maxvel_) {
      if (SATURATION_NORM == saturation_policy_) {
	qh_maxvel_ = value[0] * Vector::Ones(cursor_->ndof_);
      }
      else {
	qh_maxvel_ = value;
      }
    }
    
    return st;
  }
  
  
  void TrajectoryTask::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "trajectory task: `" << instance_name_ << "'\n";
    if ( ! cursor_) {
      os << prefix << "  NOT INITIALIZED\n";
    }
    else {
      pretty_print(actual_, os, prefix + "  actual", prefix + "    ");
      pretty_print(cursor_->position(), os, prefix + "  carrot", prefix + "    ");
      pretty_print(trjgoal_, os, prefix + "  trjgoal", prefix + "    ");
    }
  }
  
  
  CartPosTrjTask::
  CartPosTrjTask(std::string const & name)
    : TrajectoryTask(name, PDTask::SATURATION_NORM),
      end_effector_id_(-1),
      control_point_(Vector::Zero(3))
  {
    declareParameter("end_effector_id", &end_effector_id_, PARAMETER_FLAG_NOLOG);
    declareParameter("control_point", &control_point_, PARAMETER_FLAG_NOLOG);
  }
  
  
  Status CartPosTrjTask::
  init(Model const & model)
  {
    if (0 > end_effector_id_) {
      return Status(false, "you did not (correctly) set end_effector_id");
    }
    if (3 != control_point_.rows()) {
      return Status(false, "control_point needs to be three dimensional");
    }
    if (0 == updateActual(model)) {
      return Status(false, "updateActual() failed, did you specify a valid end_effector_id?");
    }
    return initTrajectoryTask(actual_);
  }
  
  
  Status CartPosTrjTask::
  update(Model const & model)
  {
    taoDNode const * ee_node(updateActual(model));
    if (0 == ee_node) {
      return Status(false, "updateActual() failed, did you specify a valid end_effector_id?");
    }
    
    Matrix Jfull;
    if ( ! model.computeJacobian(ee_node, actual_[0], actual_[1], actual_[2], Jfull)) {
      return Status(false, "failed to compute Jacobian (unsupported joint type?)");
    }
    jacobian_ = Jfull.block(0, 0, 3, Jfull.cols());
    
    return computeTrajectoryCommand(actual_,
				    jacobian_ * model.getState().velocity_,
				    command_);
  }
  
  
  taoDNode const * CartPosTrjTask::
  updateActual(Model const & model)
  {
    taoDNode * ee_node(model.getNode(end_effector_id_));
    if (ee_node) {
      jspace::Transform ee_transform;
      model.computeGlobalFrame(ee_node,
			       control_point_[0],
			       control_point_[1],
			       control_point_[2],
			       ee_transform);
      actual_ = ee_transform.translation();
    }
    return ee_node;
  }
  
  
  JPosTrjTask::
  JPosTrjTask(std::string const & name)
    : TrajectoryTask(name, PDTask::SATURATION_COMPONENT_WISE)
  {
  }
  
  
  Status JPosTrjTask::
  init(Model const & model)
  {
    jacobian_ = Matrix::Identity(model.getNDOF(), model.getNDOF());
    actual_ = model.getState().position_;
    return initTrajectoryTask(model.getState().position_);
  }
  
  
  Status JPosTrjTask::
  update(Model const & model)
  {
    actual_ = model.getState().position_;
    return computeTrajectoryCommand(actual_, model.getState().velocity_, command_);
  }
  
  
  void JPosTrjTask::
  quickSetup(double dt_seconds, double kp, double kd, double maxvel, double maxacc)
  {
    dt_seconds_ = dt_seconds;
    kp_ = kp * Vector::Ones(1);
    kd_ = kd * Vector::Ones(1);
    maxvel_ = maxvel * Vector::Ones(1);
    maxacc_ = maxacc * Vector::Ones(1);
  }
  
  
  JointLimitTask::
  JointLimitTask(std::string const & name)
    : Task(name),
      dt_seconds_(-1)
  {
    declareParameter("dt_seconds", &dt_seconds_, PARAMETER_FLAG_NOLOG);
    declareParameter("upper_stop_deg", &upper_stop_deg_, PARAMETER_FLAG_NOLOG);
    declareParameter("upper_trigger_deg", &upper_trigger_deg_, PARAMETER_FLAG_NOLOG);
    declareParameter("lower_stop_deg", &lower_stop_deg_, PARAMETER_FLAG_NOLOG);
    declareParameter("lower_trigger_deg", &lower_trigger_deg_, PARAMETER_FLAG_NOLOG);
    declareParameter("kp", &kp_, PARAMETER_FLAG_NOLOG);
    declareParameter("kd", &kd_, PARAMETER_FLAG_NOLOG);
    declareParameter("maxvel", &maxvel_, PARAMETER_FLAG_NOLOG);
    declareParameter("maxacc", &maxacc_, PARAMETER_FLAG_NOLOG);
  }
  
  
  JointLimitTask::
  ~JointLimitTask()
  {
    for (size_t ii(0); ii < cursor_.size(); ++ii) {
      delete cursor_[ii];
    }
  }
  
  
  Status JointLimitTask::
  check(Vector const * param, Vector const & value) const
  {
    if ((param == &kp_) || (param == &kd_) || (param == &maxvel_) || (param == &maxacc_)) {
      for (size_t ii(0); ii < value.rows(); ++ii) {
	if (0 > value[ii]) {
	  return Status(false, "gains and limits must be >= 0");
	}
      }
    }
    if ( ! cursor_.empty()) {	// we are initialized
      if ((param == &kp_) || (param == &kd_) || (param == &maxvel_) || (param == &maxacc_)) {
	if (cursor_.size() != value.rows()) {
	  return Status(false, "invalid dimension");
	}
      }
    }
    return Status();
  }
  
  
  Status JointLimitTask::
  check(double const * param, double const & value) const
  {
    if ((param == &dt_seconds_) && (0 >= value)) {
      return Status(false, "dt_seconds must be > 0");
    }
    return Status();
  }
  
  
  Status JointLimitTask::
  init(Model const & model)
  {
    if (dt_seconds_ <= 0) {
      return Status(false, "dt_seconds must be positive");
    }
    size_t const ndof(model.getNDOF());
    if (upper_stop_deg_.rows() != ndof) {
      return Status(false, "upper_stop dimension mismatch");
    }
    if (upper_trigger_deg_.rows() != ndof) {
      return Status(false, "upper_trigger dimension mismatch");
    }
    if (lower_stop_deg_.rows() != ndof) {
      return Status(false, "lower_stop dimension mismatch");
    }
    if (lower_trigger_deg_.rows() != ndof) {
      return Status(false, "lower_trigger dimension mismatch");
    }
    if (maxvel_.rows() != ndof) {
      return Status(false, "maxvel dimension mismatch");
    }
    if (maxacc_.rows() != ndof) {
      return Status(false, "maxacc dimension mismatch");
    }
    if (kp_.rows() != ndof) {
      return Status(false, "kp dimension mismatch");
    }
    if (kd_.rows() != ndof) {
      return Status(false, "kd dimension mismatch");
    }
    
    upper_stop_ = M_PI * upper_stop_deg_ / 180.0;
    upper_trigger_ = M_PI * upper_trigger_deg_ / 180.0;
    lower_stop_ = M_PI * lower_stop_deg_ / 180.0;
    lower_trigger_ = M_PI * lower_trigger_deg_ / 180.0;
    
    cursor_.assign(ndof, 0);
    goal_ = Vector::Zero(ndof);
    jacobian_.resize(0, 0);
    
    // builds/updates jacobian, initializes cursors and goals, set actual_
    updateState(model);
    
    Status ok;
    return ok;
  }
  
  
  Status JointLimitTask::
  update(Model const & model)
  {
    if (dt_seconds_ <= 0) {
      return Status(false, "not initialized");
    }
    
    updateState(model);
    command_.resize(jacobian_.rows());
    size_t task_index(0);
    
    for (size_t joint_index(0); joint_index < cursor_.size(); ++joint_index) {
      if (cursor_[joint_index]) {
	if (0 > cursor_[joint_index]->next(maxvel_[joint_index], maxacc_[joint_index], goal_[joint_index])) {
	  return Status(false, "trajectory generation error");
	}
	double com(kp_[joint_index] * (cursor_[joint_index]->position()[0] - actual_[task_index]));
	if ((maxvel_[joint_index] > 1e-4) && (kd_[joint_index] > 1e-4)) {
	  double const sat(fabs((com / maxvel_[joint_index]) / kd_[joint_index]));
	  if (sat > 1.0) {
	    com /= sat;
	  }
	}
	command_[task_index]
	  = com
	  + kd_[joint_index] * (cursor_[joint_index]->velocity()[0] - model.getState().velocity_[joint_index]);
	++task_index;
      }
    }
    
    Status ok;
    return ok;
  }
  
  
  void JointLimitTask::
  updateState(Model const & model)
  {
    size_t const ndof(model.getNDOF());
    Vector const & jpos(model.getState().position_);
    bool dimension_changed(false);
    size_t task_dimension(0);
    
    for (size_t ii(0); ii < ndof; ++ii) {
      if (cursor_[ii]) {
	++task_dimension;
      }
      else {
	if (jpos[ii] > upper_trigger_[ii]) {
	  ++task_dimension;
	  dimension_changed = true;
	  cursor_[ii] = new TypeIOTGCursor(1, dt_seconds_);
	  cursor_[ii]->position()[0] = jpos[ii];
	  cursor_[ii]->velocity()[0] = model.getState().velocity_[ii];
	  goal_[ii] = upper_stop_[ii];
	}
	else if (jpos[ii] < lower_trigger_[ii]) {
	  ++task_dimension;
	  dimension_changed = true;
          cursor_[ii] = new TypeIOTGCursor(1, dt_seconds_);
          cursor_[ii]->position()[0] = jpos[ii];
          cursor_[ii]->velocity()[0] = model.getState().velocity_[ii];
          goal_[ii] = lower_stop_[ii];
	}
      }
    }
    
    if (dimension_changed) {
      jacobian_ = Matrix::Zero(task_dimension, ndof);
      size_t task_index(0);
      for (size_t joint_index(0); joint_index < ndof; ++joint_index) {
	if (cursor_[joint_index]) {
	  jacobian_.coeffRef(task_index, joint_index) = 1.0;
	  ++task_index;
	}
      }
    }
    
    actual_.resize(task_dimension);
    size_t task_index(0);
    for (size_t joint_index(0); joint_index < ndof; ++joint_index) {
      if (cursor_[joint_index]) {
	actual_[task_index] = jpos[joint_index];
	++task_index;
      }
    }
  }
  
  
  void JointLimitTask::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "joint limit task: `" << instance_name_ << "'\n";
    if (cursor_.empty()) {
      os << prefix << "  NOT INITIALIZED\n";
    }
    pretty_print(actual_, os, prefix + "  actual", prefix + "    ");
    pretty_print(goal_, os, prefix + "  goal", prefix + "    ");
    pretty_print(jacobian_, os, prefix + "  jacobian", prefix + "    ");
  }


  OrientationTask::
  OrientationTask(std::string const & name)
    : Task(name),
      end_effector_id_(-1),
      kp_(50),
      kd_(5),
      maxvel_(0.5)
  {
    declareParameter("end_effector_id", &end_effector_id_, PARAMETER_FLAG_NOLOG);
    declareParameter("kp", &kp_, PARAMETER_FLAG_NOLOG);
    declareParameter("kd", &kd_, PARAMETER_FLAG_NOLOG);
    declareParameter("maxvel", &maxvel_, PARAMETER_FLAG_NOLOG);
    declareParameter("eepos", &eepos_);
  }
  
  
  Status OrientationTask::
  init(Model const & model)
  {
    if (0 > end_effector_id_) {
      return Status(false, "I need an end effector ID please");
    }
    if ( ! updateActual(model)) {
      return Status(false, "invalid end effector ID or failure to compute Jacobian");
    }
    goal_x_ = actual_x_;
    goal_y_ = actual_y_;
    goal_z_ = actual_z_;
    Status ok;
    return ok;
  }
  
  
  taoDNode const * OrientationTask::
  updateActual(Model const & model)
  {
    taoDNode * ee_node(model.getNode(end_effector_id_));
    if ( ! ee_node) {
      return 0;
    }
    
    jspace::Transform ee_transform;
    model.computeGlobalFrame(ee_node, Vector::Zero(3), ee_transform);
    Matrix Jfull;
    eepos_ = ee_transform.translation();
    if ( ! model.computeJacobian(ee_node,
				 eepos_[0],
				 eepos_[1],
				 eepos_[2],
				 Jfull)) {
      return 0;
    }
    
    jacobian_ = Jfull.block(3, 0, 3, Jfull.cols());
    
    actual_x_ = ee_transform.linear().block(0, 0, 3, 1);
    actual_y_ = ee_transform.linear().block(0, 1, 3, 1);
    actual_z_ = ee_transform.linear().block(0, 2, 3, 1);

    actual_.resize(9);
    actual_.block(0, 0, 3, 1) = actual_x_;
    actual_.block(3, 0, 3, 1) = actual_y_;
    actual_.block(6, 0, 3, 1) = actual_z_;
    
    velocity_ = jacobian_ * model.getState().velocity_;
    
    return ee_node;
  }
  
  
  Status OrientationTask::
  update(Model const & model)
  {
    if ( ! updateActual(model)) {
      return Status(false, "invalid end effector ID");
    }
    delta_ = actual_x_.cross(goal_x_) + actual_y_.cross(goal_y_) + actual_z_.cross(goal_z_);
    delta_ *= -0.5;
    
    command_ = -delta_ * kp_;
    if ((maxvel_ > 1e-4) && (kd_ > 1e-4)) {
      double const sat(fabs((command_.norm() / maxvel_) / kd_));
      if (sat > 1.0) {
	command_ /= sat;
      }
    }
    
    command_ -= velocity_ * kd_;
    
    Status ok;
    return ok;
  }
  
  
  void OrientationTask::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "orientation task: `" << instance_name_ << "'\n";
    pretty_print(velocity_, os, prefix + "  velocity", prefix + "    ");
    pretty_print(goal_x_, os, prefix + "  goal_x", prefix + "    ");
    pretty_print(goal_y_, os, prefix + "  goal_y", prefix + "    ");
    pretty_print(goal_z_, os, prefix + "  goal_z", prefix + "    ");
    Vector foo(3);
    foo << goal_x_.norm(), goal_y_.norm(), goal_z_.norm();
    pretty_print(foo, os, prefix + "  length of goal unit vectors", prefix + "    ");
    pretty_print(actual_x_, os, prefix + "  actual_x", prefix + "    ");
    pretty_print(actual_y_, os, prefix + "  actual_y", prefix + "    ");
    pretty_print(actual_z_, os, prefix + "  actual_z", prefix + "    ");
    foo << actual_x_.norm(), actual_y_.norm(), actual_z_.norm();
    pretty_print(foo, os, prefix + "  length of actual unit vectors", prefix + "    ");
    pretty_print(delta_, os, prefix + "  delta", prefix + "    ");
    pretty_print(command_, os, prefix + "  command", prefix + "    ");
  }

}
