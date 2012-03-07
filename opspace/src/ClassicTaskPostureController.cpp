/*
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 *
 * Author: Roland Philippsen
 *         http://cs.stanford.edu/group/manips/
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

#include <opspace/ClassicTaskPostureController.hpp>
#include <opspace/pseudo_inverse.hpp>

// hmm...
#include <Eigen/LU>
#include <Eigen/SVD>
#include <opspace/task_library.hpp>

using jspace::pretty_print;
using boost::shared_ptr;

namespace opspace {
  
  
  ClassicTaskPostureController::
  ClassicTaskPostureController(std::string const & name)
    : Controller(name)
  {
    declareParameter("jpos", &jpos_);
    declareParameter("jvel", &jvel_);
    declareParameter("gamma", &gamma_);
    declareParameter("fstar", &fstar_);
    declareParameter("lambda", &lambda_);
    declareParameter("jbar", &jbar_);
    declareParameter("nullspace", &nullspace_);
  }
  
  
  Status ClassicTaskPostureController::
  init(Model const & model)
  {
    return Status();
  }
  
  
  Status ClassicTaskPostureController::
  computeCommand(Model const & model,
		 Skill & skill,
		 Vector & gamma)
  {
    Status st(skill.update(model));
    if ( ! st) {
      return st;
    }
    
    Skill::task_table_t const * tasks(skill.getTaskTable());
    if ( ! tasks) {
      st.ok = false;
      st.errstr = "null task table";
      return st;
    }
    if (2 != tasks->size()) {
      st.ok = false;
      st.errstr = "task table must have exactly 2 entries";
      return st;
    }
    
    Task const * task((*tasks)[0]);
    Task const * posture((*tasks)[1]);
    
    Matrix ainv;
    if ( ! model.getInverseMassInertia(ainv)) {
      st.ok = false;
      st.errstr = "failed to retrieve inverse mass inertia";
      return st;
    }
    Vector grav;
    if ( ! model.getGravity(grav)) {
      st.ok = false;
      st.errstr = "failed to retrieve gravity torques";
      return st;
    }
    
    size_t const ndof(model.getNDOF());
    
    Matrix const & jac(task->getJacobian());
    if (jac.cols() != ainv.rows()) {
      st.ok = false;
      st.errstr = "invalid Jacobian dimension (did you initialize and update the model?)";
      return st;
    }
    
    pseudoInverse(jac * ainv * jac.transpose(),
		  task->getSigmaThreshold(),
		  lambda_,
		  0);
    fstar_ = lambda_ * task->getCommand();
    jbar_ = ainv * jac.transpose() * lambda_;
    nullspace_ = Matrix::Identity(ndof, ndof) - jac.transpose() * jbar_.transpose();
    
    gamma_ = jac.transpose() * fstar_ + nullspace_ * posture->getCommand() + grav;
    gamma = gamma_;
    
    jpos_ = model.getState().position_;
    jvel_ = model.getState().velocity_;
    
    return st;
  }
  
  
  void ClassicTaskPostureController::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    pretty_print(jpos_, os, prefix + "jpos", prefix + "  ");
    pretty_print(jvel_, os, prefix + "jvel", prefix + "  ");
    pretty_print(fstar_, os, prefix + "fstar", prefix + "  ");
    pretty_print(lambda_, os, prefix + "lambda", prefix + "  ");
    pretty_print(jbar_, os, prefix + "jbar", prefix + "  ");
    pretty_print(gamma_, os, prefix + "gamma", prefix + "  ");
  }
  
}
