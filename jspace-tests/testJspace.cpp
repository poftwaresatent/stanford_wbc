/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
   \file testJspace.cpp
   \author Roland Philippsen
*/

#include "model_library.hpp"
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoDynamics.h>
#include <tao/dynamics/taoJoint.h>
#include <jspace/tao_dump.hpp>
#include <jspace/vector_util.hpp>
#include <jspace/controller_library.hpp>
#include <jspace/ros/urdf_to_tao.hpp>
#include <jspace/ros/urdf_dump.hpp>
#include <wbcnet/strutil.hpp>
// #include <wbc/core/RobotControlModel.hpp>
#include <wbc/parse/BRParser.hpp>
#include <urdf/model.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <gtest/gtest.h>
#include <errno.h>
// #include <string.h>

using namespace std;
using namespace jspace::test;

static std::string create_puma_frames() throw(runtime_error);
static urdf::Model * create_unit_mass_5R_urdf() throw(runtime_error);


TEST (jspaceModel, state)
{
  jspace::Model * model(0);
  try {
    model = create_puma_model();
    int const ndof(model->getNDOF());
    jspace::State state_in(ndof, ndof, 0);
    for (int ii(0); ii < ndof; ++ii) {
      state_in.position_[ii] = -3 + ii * 0.5;
      state_in.velocity_[ii] = 3 - ii * 0.25;
    }
    model->setState(state_in);
    {
      jspace::State state_out(ndof, ndof, 0);
      state_out = model->getState();
      EXPECT_TRUE (state_out.equal(state_in, jspace::State::COMPARE_ALL, 1e-6))
	<< "jspace::State assignment or jspace::Model::getState() is buggy";
    }
    {
      jspace::State state_out(model->getState());
      EXPECT_TRUE (state_out.equal(state_in, jspace::State::COMPARE_ALL, 1e-6))
	<< "jspace::State copy ctor or jspace::Model::getState() is buggy";
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceModel, branching)
{
  jspace::Model * model(0);
  try {
    model = create_puma_model();
    EXPECT_EQ (model->getNNodes(), 6) << "Puma should have 6 nodes";
    EXPECT_EQ (model->getNJoints(), 6) << "Puma should have 6 joints";
    EXPECT_EQ (model->getNDOF(), 6) << "Puma should have 6 DOF";
    char const * node_name[] = {
      "base",
      "upper_arm",
      "lower_arm",
      "wrist-hand",
      "wrist-finger",
      "end-effector"
    };
    char const * joint_name[] = {
      "shoulder-yaw",
      "shoulder-pitch",
      "elbow",
      "wrist-roll1",
      "wrist-pitch",
      "wrist-roll2"
    };
    for (int ii(0); ii < 6; ++ii) {
      EXPECT_NE ((void*)0, model->getNode(ii))
	<< "Could not get node " << ii;
      EXPECT_NE ((void*)0, model->getNodeByName(node_name[ii]))
	<< "Could not get node by name \"" << node_name[ii] << "\"";
      if (model->getNodeByName(node_name[ii])) {
	EXPECT_EQ (ii, model->getNodeByName(node_name[ii])->getID())
	  << "Node with name \"" << node_name[ii] << "\" should have ID " << ii;
      }
      EXPECT_NE ((void*)0, model->getNodeByJointName(joint_name[ii]))
	<< "Could not get node by joint name \"" << joint_name[ii] << "\"";
      if (model->getNodeByJointName(joint_name[ii])) {
	EXPECT_EQ (ii, model->getNodeByJointName(joint_name[ii])->getID())
	  << "Node with joint name \"" << joint_name[ii] << "\" should have ID " << ii;
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceModel, kinematics)
{
  jspace::Model * model(0);
  try {
    model = create_puma_model();
    int const ndof(model->getNDOF());
    jspace::State state(ndof, ndof, 0);
    
    string const frames_filename(create_puma_frames());
    ifstream is(frames_filename.c_str());

    string line;
    int joint_positions_count(0);
    int link_origins_count(0);
    int line_count(0);

    while (getline(is, line)) {
      ++line_count;

      if (line.empty() || ('=' == line[0])) {
	continue;
      }
      vector<string> token;
      if (1 > sfl::tokenize(line, ':', token)) {
	continue;
      }
      
      if ("joint_positions" == token[0]) {
	if (2 > token.size()) {
	  FAIL () << frames_filename << ": line " << line_count << ": no joint positions, expected " << ndof;
	}
	istringstream ipos(token[1]);
	for (int ii(0); ii < ndof; ++ii) {
	  ipos >> state.position_[ii];
	}
	if ( ! ipos) {
	  FAIL () << frames_filename << ": line " << line_count << ": not enough joint positions, expected " << ndof;
	}
	model->update(state);
	++joint_positions_count;
	continue;
      }
      
      if ("link_origins" == token[0]) {
	++link_origins_count;
	continue;
      }
      
      if (4 > token.size()) {
	FAIL () << frames_filename << ": line " << line_count << ": expected ID-frame entry";
      }
      
      if (joint_positions_count != link_origins_count) {
	FAIL () << frames_filename << ": line " << line_count << ": joint_positions_count != link_origins_count";
      }
      
      int id;
      float rx, ry, rz, rw, tx, ty, tz;
      if (8 != sscanf(line.c_str(),
		      " ID %d: r: { %f %f %f %f } t: { %f %f %f }",
		      &id, &rx, &ry, &rz, &rw, &tx, &ty, &tz)) {
	FAIL () << frames_filename << ": line " << line_count << ": could not parse ID-frame entry";
      }
      if (ndof <= id) {
	FAIL () << frames_filename << ": line " << line_count << ": ID-frame entry " << id << " exceeds NDOF " << ndof;
      }
      
      SAITransform transform;
      if ( ! model->getGlobalFrame(model->getNode(id), transform)) {
	FAIL() << frames_filename << ": line " << line_count << ": could not get global frame " << id << " from model";
      }
      EXPECT_TRUE (transform.rotation().equal(SAIQuaternion(rw, rx, ry, rz), 1e-6))
	<< "rotation mismatch\n"
	<< "  entry: " << joint_positions_count << "\n"
	<< "  pos: " << state.position_ << "\n"
	<< "  ID: " << id << "\n"
	<< "  expected: " << SAIQuaternion(rw, rx, ry, rz) << "\n"
	<< "  computed: " << transform.rotation();
      EXPECT_TRUE (transform.translation().equal(SAIVector3(tx, ty, tz), 1e-6))
	<< "translation mismatch\n"
	<< "  entry: " << joint_positions_count << "\n"
	<< "  pos: " << state.position_ << "\n"
	<< "  ID: " << id << "\n"
	<< "  expected: " << SAIVector3(tx, ty, tz) << "\n"
	<< "  computed: " << transform.translation();
      
#ifdef VERBOSE
      cout << "PASSED transform check\n"
	   << "  entry: " << joint_positions_count << "\n"
	   << "  pos: " << state.position_ << "\n"
	   << "  ID: " << id << "\n"
	   << "  rotation:\n"
	   << "    expected: " << SAIQuaternion(rw, rx, ry, rz) << "\n"
	   << "    computed: " << transform.rotation() << "\n"
	   << "  translation:\n"
	   << "    expected: " << SAIVector3(tx, ty, tz) << "\n"
	   << "    computed: " << transform.translation() << "\n";
#endif // VERBOSE
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


static double smart_delta(double have, double want)
{
  if (fabs(want) > 1e-6) {
    return (have - want) / want;
  }
  return have - want;
}


template<typename mtype>
int mnrows(mtype const & mm) { return mm.row(); }

template<>
int mnrows(deMatrix3 const & mm) { return 3; }

template<typename mtype>
int mncols(mtype const & mm) { return mm.column(); }

template<>
int mncols(deMatrix3 const & mm) { return 3; }


template<typename mtype>
bool check_matrix(char const * name,
		  mtype const & want,
		  mtype const & have,
		  double precision,
		  std::ostringstream & msg)
{
  int const nrows(mnrows(want));
  if (nrows != mnrows(have)) {
    msg << "check_matrix(" << name << ") size mismatch: have " << mnrows(have)
	<< " rows but want " << nrows << "\n";
    return false;
  }
  int const ncolumns(mncols(want));
  if (ncolumns != mncols(have)) {
    msg << "check_matrix(" << name << ") size mismatch: have " << mncols(have)
	<< " columns but want " << ncolumns << "\n";
    return false;
  }
  
  precision = fabs(precision);
  double maxdelta(0);
  SAIMatrix delta(nrows, ncolumns);
  for (int ii(0); ii < nrows; ++ii) {
    for (int jj(0); jj < ncolumns; ++jj) {
      delta[ii][jj] = fabs(smart_delta(have[ii][jj], want[ii][jj]));
      if (delta[ii][jj] > precision) {
	maxdelta = delta[ii][jj];
      }
    }
  }
  double const halfmax(0.5 * maxdelta);
  double const tenprecision(10 * precision);
  
  if (maxdelta <= precision) {
    msg << "check_matrix(" << name << ") OK\n";
  }
  else {
    msg << "check_matrix(" << name << ") FAILED\n";
  }
  msg << "  precision = " << precision << "\n"
      << "  maxdelta = " << maxdelta << "\n";
  delta.prettyPrint(msg, "  delta", "    ");
  msg << "  error pattern\n";
  for (int ii(0); ii < nrows; ++ii) {
    msg << "    ";
    for (int jj(0); jj < ncolumns; ++jj) {
      if (delta[ii][jj] <= precision) {
	if (delta[ii][jj] < halfmax) {
	  msg << ".";
	}
	else {
	  msg << "o";
	}
      }
      else if (delta[ii][jj] >= tenprecision) {
	msg << "#";
      }
      else {
	msg << "*";
      }
    }
    msg << "\n";
  }
  
  return maxdelta <= precision;
}


template<typename vtype>
void print_vector(vtype const & vv,
		  std::ostream & msg,
		  std::string const & title,
		  std::string const & prefix)
{
  msg << title << "\n" << prefix << vv << "\n";
}


template<>
void print_vector(SAIVector const & vv,
		  std::ostream & msg,
		  std::string const & title,
		  std::string const & prefix)
{
  vv.prettyPrint(msg, title, prefix);
}


template<typename vtype>
int vsize(vtype const & vv) { return vv.size(); }

template<>
int vsize(deVector3 const & vv) { return 3; }

template<>
int vsize(deVector6 const & vv) { return 6; }


template<typename vtype>
double vget(vtype const & vv, int idx) { return vv[idx]; }

template<>
double vget(deVector6 const & vv, int idx) { return vv.elementAt(idx); }


template<typename vtype>
bool check_vector(char const * name,
		  vtype const & want,
		  vtype const & have,
		  double precision,
		  std::ostream & msg)
{
  int const nelems(vsize(want));
  if (nelems != vsize(have)) {
    msg << "check_vector(" << name << ") size mismatch: have " << vsize(have)
	<< " elements but want " << nelems << "\n";
    return false;
  }
  
  precision = fabs(precision);
  double maxdelta(0);
  SAIVector delta(nelems);
  for (int ii(0); ii < nelems; ++ii) {
    delta[ii] = fabs(smart_delta(vget(have, ii), vget(want, ii)));
    if (delta[ii] > precision) {
      maxdelta = delta[ii];
    }
  }
  double const halfmax(0.5 * maxdelta);
  double const tenprecision(10 * precision);
  
  if (maxdelta <= precision) {
    msg << "check_vector(" << name << ") OK\n";
  }
  else {
    msg << "check_vector(" << name << ") FAILED\n";
  }
  msg << "  precision = " << precision << "\n"
      << "  maxdelta = " << maxdelta << "\n";
  print_vector(delta, msg, "  delta", "    ");
  msg << "  error pattern\n    ";
  for (int ii(0); ii < nelems; ++ii) {
    if (delta[ii] <= precision) {
      if (delta[ii] < halfmax) {
	msg << ".";
      }
      else {
	msg << "o";
      }
    }
    else if (delta[ii] >= tenprecision) {
      msg << "#";
    }
    else {
      msg << "*";
    }
  }
  msg << "\n";
  
  return maxdelta <= precision;
}


TEST (jspaceModel, Jacobian_R)
{
  jspace::Model * model(0);
  try {
    model = create_unit_mass_RR_model();
    taoDNode * ee(model->getNode(0));
    ASSERT_NE ((void*)0, ee) << "no end effector (node ID 0)";
    jspace::State state(2, 2, 0); // here we're only gonna test the first joint though
    jspace::zero(state.position_);
    jspace::zero(state.velocity_);
    SAITransform ee_lframe;
    ee_lframe.translation().elementAt(1) = 1;
    
    for (double qq(-M_PI); qq <= M_PI; qq += 2 * M_PI / 7) {
      state.position_[0] = qq;
      model->update(state);
      
      double const q1(state.position_[0]);
      double const c1(cos(q1));
      double const s1(sin(q1));
      
      SAITransform ee_gframe;
      ASSERT_TRUE (model->computeGlobalFrame(ee, ee_lframe, ee_gframe));
      SAIVector const & ee_gpos(ee_gframe.translation());
      {
	SAIVector ee_gpos_check(3);
	ee_gpos_check.elementAt(1) = c1;
	ee_gpos_check.elementAt(2) = s1;
	std::ostringstream msg;
	msg << "Verifying end-effector frame (position only) for q = " << state.position_ << "\n";
	ee_gpos_check.prettyPrint(msg, "  want", "    ");
	ee_gpos.prettyPrint(msg, "  have", "    ");
	bool const gpos_ok(check_vector("ee_pos", ee_gpos_check, ee_gpos, 1e-3, msg));
	EXPECT_TRUE (gpos_ok) << msg.str();
	if ( ! gpos_ok) {
	  continue;		// no use checking Jg as well, it'll be off
	}
      }
      
      SAIMatrix Jg_all(6, 2);
      ASSERT_TRUE (model->computeJacobian(ee, ee_gpos, Jg_all));
      SAIMatrix const Jg(Jg_all.submatrix(0, 0, 6, 1));
      {
	SAIMatrix Jg_check(6, 1);
	Jg_check.elementAt(1, 0) = -s1;
	Jg_check.elementAt(2, 0) =  c1;
	Jg_check.elementAt(3, 0) =  1;
	std::ostringstream msg;
	msg << "Checking Jacobian for q = " << state.position_ << "\n";
	Jg_check.prettyPrint(msg, "  want", "    ");
	Jg.prettyPrint(msg, "  have", "    ");
	EXPECT_TRUE (check_matrix("Jacobian", Jg_check, Jg, 1e-3, msg)) << msg.str();
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceModel, Jacobian_RR)
{
  jspace::Model * model(0);
  try {
    model = create_unit_mass_RR_model();
    taoDNode * ee(model->getNode(1));
    ASSERT_NE ((void*)0, ee) << "no end effector (node ID 1)";
    jspace::State state(2, 2, 0);
    jspace::zero(state.velocity_);
    SAITransform ee_lframe;
    ee_lframe.translation().elementAt(1) = 1;
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-M_PI); q2 <= M_PI; q2 += 2 * M_PI / 7) {
 	state.position_[0] = q1;
 	state.position_[1] = q2;
	model->update(state);
	
	double const c1(cos(q1));
	double const c12(cos(q1+q2));
	double const s1(sin(q1));
	double const s12(sin(q1+q2));
	
	SAITransform ee_gframe;
	ASSERT_TRUE (model->computeGlobalFrame(ee, ee_lframe, ee_gframe));
	SAIVector const & ee_gpos(ee_gframe.translation());
	{
	  SAIVector ee_gpos_check(3);
	  ee_gpos_check.elementAt(1) = c1 + c12;
	  ee_gpos_check.elementAt(2) = s1 + s12;
	  std::ostringstream msg;
	  msg << "Verifying end-effector frame (position only) for q = " << state.position_ << "\n";
	  ee_gpos_check.prettyPrint(msg, "  want", "    ");
	  ee_gpos.prettyPrint(msg, "  have", "    ");
	  bool const gpos_ok(check_vector("ee_pos", ee_gpos_check, ee_gpos, 1e-3, msg));
	  EXPECT_TRUE (gpos_ok) << msg.str();
	  if ( ! gpos_ok) {
	    continue;		// no use checking Jg as well, it'll be off
	  }
	}
	
	SAIMatrix Jg(6, 2);
	ASSERT_TRUE (model->computeJacobian(ee, ee_gpos, Jg));
	{
	  SAIMatrix Jg_check(6, 2);
	  Jg_check.elementAt(1, 0) = -s1 - s12;
	  Jg_check.elementAt(2, 0) =  c1 + c12;
	  Jg_check.elementAt(3, 0) =  1;
	  Jg_check.elementAt(1, 1) = -s12;
	  Jg_check.elementAt(2, 1) =  c12;
	  Jg_check.elementAt(3, 1) =  1;
	  std::ostringstream msg;
	  msg << "Checking Jacobian for q = " << state.position_ << "\n";
	  Jg_check.prettyPrint(msg, "  want", "    ");
	  Jg.prettyPrint(msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("Jacobian", Jg_check, Jg, 1e-3, msg)) << msg.str();
	}
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceModel, Jacobian_RP)
{
  jspace::Model * model(0);
  try {
    model = create_unit_mass_RP_model();
    taoDNode * ee(model->getNode(1));
    ASSERT_NE ((void*)0, ee) << "no end effector (node ID 1)";
    jspace::State state(2, 2, 0);
    jspace::zero(state.velocity_);
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-1); q2 <= 1; q2 += 2.0 / 7) {
 	state.position_[0] = q1;
 	state.position_[1] = q2;
	model->update(state);
	
	double const c1(cos(q1));
	double const s1(sin(q1));
	
	{
	  SAITransform ee_gframe;
	  ASSERT_TRUE (model->getGlobalFrame(ee, ee_gframe));
	  SAIVector const & ee_gpos(ee_gframe.translation());
	  SAIVector ee_gpos_check(3);
	  ee_gpos_check.elementAt(1) = c1 - s1 * q2;
	  ee_gpos_check.elementAt(2) = s1 + c1 * q2;
	  std::ostringstream msg;
	  msg << "Verifying end-effector frame (position only) for q = " << state.position_ << "\n";
	  ee_gpos_check.prettyPrint(msg, "  want", "    ");
	  ee_gpos.prettyPrint(msg, "  have", "    ");
	  bool const gpos_ok(check_vector("ee_pos", ee_gpos_check, ee_gpos, 1e-3, msg));
	  EXPECT_TRUE (gpos_ok) << msg.str();
	  if ( ! gpos_ok) {
	    continue;		// no use checking Jg as well, it'll be off
	  }
	}
	
	SAIMatrix Jg(6, 2);
	ASSERT_TRUE (model->computeJacobian(ee, Jg));
	{
	  SAIMatrix Jg_check(6, 2);
	  Jg_check.elementAt(1, 0) = -s1 - c1 * q2;
	  Jg_check.elementAt(2, 0) =  c1 - s1 * q2;
	  Jg_check.elementAt(3, 0) =  1;
	  Jg_check.elementAt(1, 1) = -s1;
	  Jg_check.elementAt(2, 1) =  c1;
	  Jg_check.elementAt(3, 1) =  0;
	  std::ostringstream msg;
	  msg << "Checking Jacobian for q = " << state.position_ << "\n";
	  Jg_check.prettyPrint(msg, "  want", "    ");
	  Jg.prettyPrint(msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("Jacobian", Jg_check, Jg, 1e-3, msg)) << msg.str();
	}
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


static void compute_unit_mass_RR_mass_inertia(double q1, double q2, SAIMatrix & MM_check)
{
  double const c1(cos(q1));
  double const c12(cos(q1+q2));
  double const s1(sin(q1));
  double const s12(sin(q1+q2));
  MM_check.setSize(2, 2);
  MM_check.elementAt(0, 0) = 1 + pow(s1 + s12, 2) + pow(c1 + c12, 2);
  MM_check.elementAt(1, 0) = s12 * (s1 + s12) + c12 * (c1 + c12);
  MM_check.elementAt(0, 1) = MM_check.elementAt(1, 0);
  MM_check.elementAt(1, 1) = 1;
}


static void compute_unit_inertia_RR_mass_inertia(double q1, double q2, SAIMatrix & MM_check)
{
  double const c1(cos(q1));
  double const c12(cos(q1+q2));
  double const s1(sin(q1));
  double const s12(sin(q1+q2));
  MM_check.setSize(2, 2);
  MM_check.elementAt(0, 0) = 3 + pow(2*s1 + s12, 2) + pow(2*c1 + c12, 2);
  MM_check.elementAt(1, 0) = 1 + s12 * (2*s1 + s12) + c12 * (2*c1 + c12);
  MM_check.elementAt(0, 1) = MM_check.elementAt(1, 0);
  MM_check.elementAt(1, 1) = 2;
}


TEST (jspaceModel, mass_inertia_RR)
{
  typedef jspace::Model * (*create_model_t)();
  create_model_t create_model[] = {
    create_unit_mass_RR_model,
    create_unit_inertia_RR_model
  };
  
  typedef void (*compute_mass_inertia_t)(double, double, SAIMatrix &);
  compute_mass_inertia_t compute_mass_inertia[] = {
    compute_unit_mass_RR_mass_inertia,
    compute_unit_inertia_RR_mass_inertia
  };
  
  for (size_t test_index(0); test_index < 2; ++test_index) {
    jspace::Model * model(0);
    try {
      model = create_model[test_index]();
      taoDNode * n1(model->getNode(0));
      taoDNode * n2(model->getNode(1));
      ASSERT_NE ((void*)0, n1);
      ASSERT_NE ((void*)0, n2);
      jspace::State state(2, 2, 0);
      jspace::zero(state.velocity_);
      
      for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
	for (double q2(-M_PI); q2 <= M_PI; q2 += 2 * M_PI / 7) {
	  state.position_[0] = q1;
	  state.position_[1] = q2;
	  model->update(state);
	  
	  SAIMatrix MM(2, 2);
	  model->getMassInertia(MM);
 	  SAIMatrix MM_check;
	  compute_mass_inertia[test_index](q1, q2, MM_check);
	  {
	    std::ostringstream msg;
	    msg << "Checking mass_inertia for test_index " << test_index
		<< " q = " << state.position_ << "\n";
	    MM_check.prettyPrint(msg, "  want", "    ");
	    MM.prettyPrint(msg, "  have", "    ");
	    EXPECT_TRUE (check_matrix("mass_inertia", MM_check, MM, 1e-3, msg)) << msg.str();
	  }
	  
	  SAIMatrix MMinv(2, 2);
	  model->getInverseMassInertia(MMinv);
	  SAIMatrix MMinv_check;
	  MM_check.inverse(MMinv_check);
	  {
	    std::ostringstream msg;
	    msg << "Checking inv_mass_inertia for test_index " << test_index
		<< " q = " << state.position_ << "\n";
	    MMinv_check.prettyPrint(msg, "  want", "    ");
	    MMinv.prettyPrint(msg, "  have", "    ");
	    EXPECT_TRUE (check_matrix("inv_mass_inertia", MMinv_check, MMinv, 1e-3, msg)) << msg.str();
	  }
	}
      }
    }
    catch (std::exception const & ee) {
      ADD_FAILURE () << "exception " << ee.what();
    }
    delete model;
  }
}


TEST (jspaceModel, mass_inertia_5R_nonzero)
{
  jspace::Model * model(0);
  try {
    model = create_unit_mass_5R_model();
    jspace::State state(5, 5, 0);
    jspace::zero(state.velocity_);
    
    for (size_t ii(0); ii < 5; ++ii) {
      for (double qq(-M_PI); qq <= M_PI; qq += 2 * M_PI / 7) {
	memset(&state.position_[0], 0, 5 * sizeof(double));
	state.position_[ii] = qq;
	model->update(state);
	
	SAIMatrix MM;
	model->getMassInertia(MM);
	std::ostringstream msg;
	bool ok(true);
	for (size_t jj(0); jj < 5; ++jj) {
	  if (MM.elementAt(jj, jj) < 1e-3) {
	    ok = false;
	    msg << "  MM[" << jj << "][" << jj << "] = " << MM.elementAt(jj, jj) << "\n";
	  }
	}
	EXPECT_TRUE (ok) << "diagonal element below threshold\n" << msg.str();
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceModel, mass_inertia_RP)
{
  jspace::Model * model(0);
  try {
    model = create_unit_mass_RP_model();
    taoDNode * n1(model->getNode(0));
    taoDNode * n2(model->getNode(1));
    ASSERT_NE ((void*)0, n1);
    ASSERT_NE ((void*)0, n2);
    jspace::State state(2, 2, 0);
    jspace::zero(state.velocity_);
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-1); q2 <= 1; q2 += 2.0 / 7) {
 	state.position_[0] = q1;
 	state.position_[1] = q2;
	model->update(state);
	
	double const c1(cos(q1));
	double const s1(sin(q1));
	
	SAIMatrix MM(2, 2);
	model->getMassInertia(MM);
	SAIMatrix MM_check(2, 2);
	MM_check.elementAt(0, 0) = 1 + pow(s1 + c1 * q2, 2) + pow(c1 - s1 * q2, 2);
	MM_check.elementAt(1, 0) = s1 * (s1 + c1 * q2) + c1 * (c1 - s1 * q2);
	MM_check.elementAt(0, 1) = MM_check.elementAt(1, 0);
	MM_check.elementAt(1, 1) = 1;
	{
	  std::ostringstream msg;
	  msg << "Checking mass_inertia for q = " << state.position_ << "\n";
	  MM_check.prettyPrint(msg, "  want", "    ");
	  MM.prettyPrint(msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("mass_inertia", MM_check, MM, 1e-3, msg)) << msg.str();
	}
	
	SAIMatrix MMinv(2, 2);
	model->getInverseMassInertia(MMinv);
	{
	  SAIMatrix id_check;
	  id_check.identity(2);
	  SAIMatrix id;
	  id = MM * MMinv;
	  std::ostringstream msg;
	  msg << "Checking A * Ainv = I for q = " << state.position_ << "\n";
	  id_check.prettyPrint(msg, "  want", "    ");
	  id.prettyPrint(msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("identity", id_check, id, 1e-3, msg)) << msg.str();
	}
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceController, mass_inertia_compensation_RR)
{
  jspace::Model * model(0);
  try {
    model = create_unit_mass_RR_model();
    ASSERT_EQ (model->getNDOF(), 2);
    jspace::State state(2, 2, 0);
    model->update(state);	// otherwise ctrl.init() complains (further down)
    std::vector<double> kp, kd;
    kp.push_back(100);
    kp.push_back(100);
    kd.push_back(20);
    kd.push_back(20);
    jspace::JointGoalController ctrl(jspace::COMP_MASS_INERTIA, kp, kd);
    jspace::Status status(ctrl.init(*model));
    ASSERT_TRUE (status) << "ctrl.init failed: " << status.errstr;
    status = ctrl.setGoal(state.position_);
    ASSERT_TRUE (status) << "ctrl.setGoal failed: " << status.errstr;
    std::vector<double> tau;
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-M_PI); q2 <= M_PI; q2 += 2 * M_PI / 7) {
	state.position_[0] = q1;
	state.position_[1] = q2;
	model->update(state);
	status = ctrl.computeCommand(*model, tau);
	ASSERT_EQ (tau.size(), 2);
	
	SAIMatrix MM;
	compute_unit_mass_RR_mass_inertia(q1, q2, MM);
	double const m11(MM.elementAt(0, 0));
	double const m12(MM.elementAt(0, 1));
	double const m22(MM.elementAt(1, 1));
	std::vector<double> tau_check(2);
	tau_check[0] = - kp[0] * (m11 * q1 + m12 * q2);
	tau_check[1] = - kp[0] * (m12 * q1 + m22 * q2);
	
	std::ostringstream msg;
	msg << "Verifying command for q = " << state.position_ << "\n"
	    << "  want: " << tau_check << "\n"
	    << "  have: " << tau << "\n";
	EXPECT_TRUE (check_vector("tau", tau_check, tau, 1e-3, msg)) << msg.str();
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceROSModel, urdf_links_5R)
{
  urdf::Model * urdf_model(0);
  jspace::tao_tree_info_s * tree(0);
  wbc::BranchingRepresentation * brep_model(0);
  jspace::tao_tree_info_s * tree_check(0);
  
  try {
    urdf_model = create_unit_mass_5R_urdf();
    tree = convert_urdf_to_tao(*urdf_model, "world", jspace::ros::DefaultLinkFilter());
    ASSERT_EQ (5, tree->info.size());
    
    brep_model = create_unit_mass_5R_brep();
    tree_check = brep_model->createTreeInfo();
    ASSERT_EQ (5, tree_check->info.size());
    
    for (size_t ii(0); ii < 5; ++ii) {
      taoDNode * node(tree->info[ii].node);
      taoDNode * node_check(tree_check->info[ii].node);
      taoDNode * parent(node->getDParent());
      taoDNode * parent_check(node_check->getDParent());
      
      EXPECT_EQ (node->getID(), node_check->getID());
      
      if ( ! parent) {
	EXPECT_EQ ((taoDNode*)0, parent_check);
      }
      else {
	EXPECT_EQ (parent->getID(), parent_check->getID());
      }
      
      if ( ! node->mass()) {
	EXPECT_EQ ((deFloat*)0, node_check->mass());
      }
      else {
	EXPECT_EQ (*(node->mass()), *(node_check->mass()));
      }
      
      if ( ! node->center()) {
	EXPECT_EQ ((deVector3*)0, node_check->center());
      }
      else {
	std::ostringstream msg;
	msg << "COM check\n"
	    << "  want = " << *(node_check->center()) << "\n"
	    << "  have = " << *(node->center()) << "\n";
	EXPECT_TRUE (check_vector("COM", *(node_check->center()), *(node->center()), 1e-3, msg)) << msg.str();
      }
      
      if ( ! node->inertia()) {
	EXPECT_EQ ((deMatrix3*)0, node_check->inertia());
      }
      else {
	std::ostringstream msg;
	msg << "inertia check\n"
	    << "  want = " << jspace::inertia_matrix_to_string(*(node_check->inertia())) << "\n"
	    << "  have = " << jspace::inertia_matrix_to_string(*(node->inertia())) << "\n";
	EXPECT_TRUE (check_matrix("inertia", *(node_check->inertia()), *(node->inertia()), 1e-3, msg)) << msg.str();
      }
    }    
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }

  delete tree_check;
  delete brep_model;
  delete tree;
  delete urdf_model;
}


TEST (jspaceROSModel, urdf_joints_5R)
{
  urdf::Model * urdf_model(0);
  jspace::tao_tree_info_s * tree(0);
  wbc::BranchingRepresentation * brep_model(0);
  jspace::tao_tree_info_s * tree_check(0);
  
  try {
    urdf_model = create_unit_mass_5R_urdf();
    tree = convert_urdf_to_tao(*urdf_model, "world", jspace::ros::DefaultLinkFilter());
    ASSERT_EQ (5, tree->info.size());
    
    brep_model = create_unit_mass_5R_brep();
    tree_check = brep_model->createTreeInfo();
    ASSERT_EQ (5, tree_check->info.size());
    
    for (size_t ii(0); ii < 5; ++ii) {
      taoDNode * node(tree->info[ii].node);
      taoDNode * node_check(tree_check->info[ii].node);
      
      taoJoint * joint(node->getJointList());
      taoJoint * joint_check(node_check->getJointList());
      
      if ( ! joint) {
	EXPECT_EQ ((taoJoint*)0, joint_check);
	continue;
      }
      if ( ! joint_check) {
	ADD_FAILURE () << "joint_check is NULL but joint is not";
	continue;
      }
      
      EXPECT_EQ ((taoJoint*)0, joint->getNext());
      EXPECT_EQ ((taoJoint*)0, joint_check->getNext());
      EXPECT_EQ (joint->getDOF(), joint_check->getDOF());
      EXPECT_EQ (joint->getType(), joint_check->getType());
      
      taoJointDOF1 * jdof1(dynamic_cast<taoJointDOF1*>(joint));
      taoJointDOF1 * jdof1_check(dynamic_cast<taoJointDOF1*>(joint_check));
      
      if ( ! jdof1) {
	EXPECT_EQ ((taoJointDOF1*)0, jdof1_check);
	continue;
      }
      if ( ! jdof1_check) {
	ADD_FAILURE () << "jdof1_check is NULL but jdof1 is not";
	continue;
      }
      
      EXPECT_EQ (jdof1->getAxis(), jdof1_check->getAxis());
      
      deVector6 const & sv(jdof1->getS());
      deVector6 const & sv_check(jdof1_check->getS());
      
      std::ostringstream msg;
      msg << "getS() check\n"
	  << "  want = " << sv_check << "\n"
	  << "  have = " << sv << "\n";
      EXPECT_TRUE (check_vector("sv", sv_check, sv, 1e-3, msg)) << msg.str();
      
    }    
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }

  delete tree_check;
  delete brep_model;
  delete tree;
  delete urdf_model;
}


static jspace::tao_tree_info_s * reparse_tao_tree(jspace::tao_tree_info_s * orig)  throw(runtime_error)
{
  static char const * fname_template("reparse_sai.xml.XXXXXX");
  static char tmpname[64];
  memset(tmpname, '\0', 64);
  strncpy(tmpname, fname_template, 63);
  int const tmpfd(mkstemp(tmpname));
  if (-1 == tmpfd) {
    throw runtime_error("reparse_tao_tree(): mkstemp(): " + string(strerror(errno)));
  }
  
  std::ostringstream os;
  xmldump_tao_tree_info(os, orig);
  std::string const contents(os.str());  
  size_t const len(contents.size());
  if (static_cast<ssize_t>(len) != write(tmpfd, contents.c_str(), len)) {
    throw runtime_error("reparse_tao_tree(): write(): " + string(strerror(errno)));
  }
  close(tmpfd);
  
  wbc::BranchingRepresentation * brep(wbc::BRParser::parse("sai", tmpname));
  jspace::tao_tree_info_s * tree(brep->createTreeInfo());
  delete brep;
  
  return tree;
}


TEST (jspaceROSModel, urdf_dynamics_5R)
{
  urdf::Model const * urdf_model(0);
  std::vector<jspace::tao_tree_info_s*> tao_trees;
  jspace::Model * model(0);
  jspace::Model * model_check(0);
  
  try {
    urdf_model = create_unit_mass_5R_urdf();

#undef CRAZY_WORKAROUND
#ifdef CRAZY_WORKAROUND
    jspace::tao_tree_info_s * wtf(convert_urdf_to_tao(*urdf_model, "world", jspace::ros::DefaultLinkFilter()));
    tao_trees.push_back(reparse_tao_tree(wtf));
    tao_trees.push_back(reparse_tao_tree(wtf));
    delete wtf;
#else // CRAZY_WORKAROUND
    static const size_t n_tao_trees(2);
    convert_urdf_to_tao_n(*urdf_model, "world", jspace::ros::DefaultLinkFilter(), tao_trees, n_tao_trees);
#endif // CRAZY_WORKAROUND
    
    model = new jspace::Model(tao_trees[0], tao_trees[1]);
    taoDNode * ee(model->getNode(4));
    ASSERT_NE ((void*)0, ee);
    
    model_check = create_unit_mass_5R_model();
    taoDNode * ee_check(model_check->getNode(4));
    ASSERT_NE ((void*)0, ee_check);
    
    jspace::State state(5, 5, 0);
    for (size_t ii(0); ii < 5; ++ii) {
      for (double pos(-M_PI); pos <= M_PI; pos += 2 * M_PI / 7) {
	memset(&state.position_[0], 0, 5 * sizeof(double));
	state.position_[ii] = pos;
	
	model->update(state);
	model_check->update(state);
	
	SAIMatrix Jg, Jg_check, MM, MM_check;
	SAIVector GG, GG_check;
	
	ASSERT_TRUE (model->computeJacobian(ee, Jg));
	ASSERT_TRUE (model_check->computeJacobian(ee_check, Jg_check));
	ASSERT_TRUE (model->getMassInertia(MM));
	ASSERT_TRUE (model_check->getMassInertia(MM_check));
	ASSERT_TRUE (model->getGravity(GG));
	ASSERT_TRUE (model_check->getGravity(GG_check));
	
	{
	  std::ostringstream msg;
	  msg << "Jg check\n"
	      << "  pos = " << state.position_ << "\n"
	      << "  vel = " << state.velocity_ << "\n";
	  Jg_check.prettyPrint(msg, "  want", "    ");
	  Jg.prettyPrint(msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("Jg", Jg_check, Jg, 1e-3, msg)) << msg.str();
	}
	{
	  std::ostringstream msg;
	  msg << "MM check\n"
	      << "  pos = " << state.position_ << "\n"
	      << "  vel = " << state.velocity_ << "\n";
	  MM_check.prettyPrint(msg, "  want", "    ");
	  MM.prettyPrint(msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("MM", MM_check, MM, 1e-3, msg)) << msg.str();
	}
	{
	  std::ostringstream msg;
	  msg << "GG check\n"
	      << "  pos = " << state.position_ << "\n"
	      << "  vel = " << state.velocity_ << "\n";
	  GG_check.prettyPrint(msg, "  want", "    ");
	  GG.prettyPrint(msg, "  have", "    ");
	  EXPECT_TRUE (check_vector("GG", GG_check, GG, 1e-3, msg)) << msg.str();
	}
	
	for (double vel(-M_PI); vel <= M_PI; vel += 2 * M_PI / 7) {
	  memset(&state.velocity_[0], 0, 5 * sizeof(double));
	  state.velocity_[ii] = vel;
	  
	  model->update(state);
	  model_check->update(state);
	  
	  SAIVector BB, BB_check;
	  ASSERT_TRUE (model->getCoriolisCentrifugal(BB));
	  ASSERT_TRUE (model_check->getCoriolisCentrifugal(BB_check));
	  {
	    std::ostringstream msg;
	    msg << "BB check\n"
		<< "  pos = " << state.position_ << "\n"
		<< "  vel = " << state.velocity_ << "\n";
	    BB_check.prettyPrint(msg, "  want", "    ");
	    BB.prettyPrint(msg, "  have", "    ");
	    EXPECT_TRUE (check_vector("BB", BB_check, BB, 1e-3, msg)) << msg.str();
	  }
	}
      }
    }
    
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }

  delete model_check;
  delete model;
  //   for (size_t ii(0); ii < tao_trees.size(); ++ii) {
  //     delete tao_trees[ii];
  //   }
  delete urdf_model;
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS ();
}


std::string create_puma_frames() throw(runtime_error)
{
  static char const * frames = 
    "==================================================\n"
    "joint_positions:	0	0	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.707107  0  0  0.707107 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0.7854	0	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0.382684  0.923879 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.653281  -0.270599  0.270599  0.653281 }  t: { -0.172181  0.17218  0 }\n"
    "  ID 2: r: { -0.653281  -0.270599  0.270599  0.653281 }  t: { 0.199191  0.411466  -2.77556e-17 }\n"
    "  ID 3: r: { 0  0  0.382684  0.923879 }  t: { 0.184837  0.397112  0.4331 }\n"
    "  ID 4: r: { -0.653281  -0.270599  0.270599  0.653281 }  t: { 0.184837  0.397112  0.4331 }\n"
    "  ID 5: r: { 0  0  0.382684  0.923879 }  t: { 0.184837  0.397112  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0.7854	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.653281  0.270599  0.270599  0.653281 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.653281  0.270599  0.270599  0.653281 }  t: { 0.305328  0.1501  -0.305329 }\n"
    "  ID 3: r: { 0  0.382684  8.32667e-17  0.923879 }  t: { 0.597222  0.1501  0.0152724 }\n"
    "  ID 4: r: { -0.653281  0.270599  0.270599  0.653281 }  t: { 0.597222  0.1501  0.0152724 }\n"
    "  ID 5: r: { 0  0.382684  8.32667e-17  0.923879 }  t: { 0.597222  0.1501  0.0152724 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0.7854	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.653281  0.270599  0.270599  0.653281 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0.382684  8.32667e-17  0.923879 }  t: { 0.723694  0.1501  0.320602 }\n"
    "  ID 4: r: { -0.653281  0.270599  0.270599  0.653281 }  t: { 0.723694  0.1501  0.320602 }\n"
    "  ID 5: r: { 0  0.382684  8.32667e-17  0.923879 }  t: { 0.723694  0.1501  0.320602 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0.7854	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0.382684  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.653281  -0.270599  0.270599  0.653281 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  0.382684  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	0.7854	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.653281  0.270599  0.270599  0.653281 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0.382684  8.32667e-17  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	0	0.7854	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.707107  0  0  0.707107 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  0.382684  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	-0.7854	0	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  -0.382684  0.923879 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.653281  0.270599  -0.270599  0.653281 }  t: { 0.172181  0.17218  0 }\n"
    "  ID 2: r: { -0.653281  0.270599  -0.270599  0.653281 }  t: { 0.411465  -0.199193  -2.77556e-17 }\n"
    "  ID 3: r: { 0  0  -0.382684  0.923879 }  t: { 0.397111  -0.184838  0.4331 }\n"
    "  ID 4: r: { -0.653281  0.270599  -0.270599  0.653281 }  t: { 0.397111  -0.184838  0.4331 }\n"
    "  ID 5: r: { 0  0  -0.382684  0.923879 }  t: { 0.397111  -0.184838  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	-0.7854	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.653281  -0.270599  -0.270599  0.653281 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.653281  -0.270599  -0.270599  0.653281 }  t: { 0.305328  0.1501  0.305329 }\n"
    "  ID 3: r: { 0  -0.382684  -8.32667e-17  0.923879 }  t: { -0.0152746  0.1501  0.597222 }\n"
    "  ID 4: r: { -0.653281  -0.270599  -0.270599  0.653281 }  t: { -0.0152746  0.1501  0.597222 }\n"
    "  ID 5: r: { 0  -0.382684  -8.32667e-17  0.923879 }  t: { -0.0152746  0.1501  0.597222 }\n"
    "==================================================\n"
    "joint_positions:	0	0	-0.7854	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.653281  -0.270599  -0.270599  0.653281 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  -0.382684  -8.32667e-17  0.923879 }  t: { 0.111197  0.1501  0.291893 }\n"
    "  ID 4: r: { -0.653281  -0.270599  -0.270599  0.653281 }  t: { 0.111197  0.1501  0.291893 }\n"
    "  ID 5: r: { 0  -0.382684  -8.32667e-17  0.923879 }  t: { 0.111197  0.1501  0.291893 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	-0.7854	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  -0.382684  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.653281  0.270599  -0.270599  0.653281 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  -0.382684  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	-0.7854	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.653281  -0.270599  -0.270599  0.653281 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  -0.382684  -8.32667e-17  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	0	-0.7854	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.707107  0  0  0.707107 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  -0.382684  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	1.5708	0	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0.707108  0.707105 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.499999  -0.500001  0.500001  0.499999 }  t: { -0.2435  -8.94425e-07  0 }\n"
    "  ID 2: r: { -0.499999  -0.500001  0.500001  0.499999 }  t: { -0.150102  0.431799  -5.55112e-17 }\n"
    "  ID 3: r: { 0  0  0.707108  0.707105 }  t: { -0.150102  0.411499  0.4331 }\n"
    "  ID 4: r: { -0.499999  -0.500001  0.500001  0.499999 }  t: { -0.150102  0.411499  0.4331 }\n"
    "  ID 5: r: { 0  0  0.707108  0.707105 }  t: { -0.150102  0.411499  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	1.5708	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.499999  0.500001  0.500001  0.499999 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.499999  0.500001  0.500001  0.499999 }  t: { -1.58609e-06  0.1501  -0.4318 }\n"
    "  ID 3: r: { 0  0.707108  1.11022e-16  0.707105 }  t: { 0.433098  0.1501  -0.411502 }\n"
    "  ID 4: r: { -0.499999  0.500001  0.500001  0.499999 }  t: { 0.433098  0.1501  -0.411502 }\n"
    "  ID 5: r: { 0  0.707108  1.11022e-16  0.707105 }  t: { 0.433098  0.1501  -0.411502 }\n"
    "==================================================\n"
    "joint_positions:	0	0	1.5708	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.499999  0.500001  0.500001  0.499999 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0.707108  1.11022e-16  0.707105 }  t: { 0.8649  0.1501  0.0202984 }\n"
    "  ID 4: r: { -0.499999  0.500001  0.500001  0.499999 }  t: { 0.8649  0.1501  0.0202984 }\n"
    "  ID 5: r: { 0  0.707108  1.11022e-16  0.707105 }  t: { 0.8649  0.1501  0.0202984 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	1.5708	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0.707108  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.499999  -0.500001  0.500001  0.499999 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  0.707108  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	1.5708	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.499999  0.500001  0.500001  0.499999 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0.707108  1.11022e-16  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	0	1.5708	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.707107  0  0  0.707107 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  0.707108  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	-1.5708	0	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  -0.707108  0.707105 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.499999  0.500001  -0.500001  0.499999 }  t: { 0.2435  -8.94425e-07  0 }\n"
    "  ID 2: r: { -0.499999  0.500001  -0.500001  0.499999 }  t: { 0.150098  -0.431801  2.77556e-17 }\n"
    "  ID 3: r: { 0  0  -0.707108  0.707105 }  t: { 0.150098  -0.411501  0.4331 }\n"
    "  ID 4: r: { -0.499999  0.500001  -0.500001  0.499999 }  t: { 0.150098  -0.411501  0.4331 }\n"
    "  ID 5: r: { 0  0  -0.707108  0.707105 }  t: { 0.150098  -0.411501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	-1.5708	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.499999  -0.500001  -0.500001  0.499999 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.499999  -0.500001  -0.500001  0.499999 }  t: { -1.58609e-06  0.1501  0.4318 }\n"
    "  ID 3: r: { 0  -0.707108  -1.11022e-16  0.707105 }  t: { -0.433102  0.1501  0.411498 }\n"
    "  ID 4: r: { -0.499999  -0.500001  -0.500001  0.499999 }  t: { -0.433102  0.1501  0.411498 }\n"
    "  ID 5: r: { 0  -0.707108  -1.11022e-16  0.707105 }  t: { -0.433102  0.1501  0.411498 }\n"
    "==================================================\n"
    "joint_positions:	0	0	-1.5708	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.499999  -0.500001  -0.500001  0.499999 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  -0.707108  -1.11022e-16  0.707105 }  t: { -0.00129993  0.1501  -0.0203016 }\n"
    "  ID 4: r: { -0.499999  -0.500001  -0.500001  0.499999 }  t: { -0.00129993  0.1501  -0.0203016 }\n"
    "  ID 5: r: { 0  -0.707108  -1.11022e-16  0.707105 }  t: { -0.00129993  0.1501  -0.0203016 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	-1.5708	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  -0.707108  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.499999  0.500001  -0.500001  0.499999 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  -0.707108  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	-1.5708	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.499999  -0.500001  -0.500001  0.499999 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  -0.707108  -1.11022e-16  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	0	-1.5708	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.707107  0  0  0.707107 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  -0.707108  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n";
  std::string result(create_tmpfile("puma.frames.XXXXXX", frames));
  return result;
}


urdf::Model * create_unit_mass_5R_urdf() throw(runtime_error)
{
  static char const * urdf_string =
    "<?xml version=\"1.0\"?>\n"
    "<robot name=\"continuous_chain\"\n"
    "       xmlns:xi=\"http://www.w3.org/2001/XInclude\"\n"
    "       xmlns:gazebo=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#gz\"\n"
    "       xmlns:model=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#model\"\n"
    "       xmlns:sensor=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor\"\n"
    "       xmlns:body=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#body\"\n"
    "       xmlns:geom=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#geom\"\n"
    "       xmlns:joint=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#joint\"\n"
    "       xmlns:interface=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#interface\"\n"
    "       xmlns:rendering=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering\"\n"
    "       xmlns:renderable=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable\"\n"
    "       xmlns:controller=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#controller\"\n"
    "       xmlns:physics=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#physics\">\n"
    "  <link name=\"world\" />\n"
    "  <joint name=\"joint_1\" type=\"continuous\" >\n"
    "    <axis xyz=\"0 0 1\" />\n"
    "    <origin xyz=\"0 0 2\" rpy=\"0 0 0\" />\n"
    "    <parent link=\"world\" />\n"
    "    <child link=\"link_1\" />\n"
    "    <dynamics damping=\"0.0\" />\n"
    "  </joint>\n"
    "  <link name=\"link_1\">\n"
    "    <inertial>\n"
    "      <mass value=\"1\" />\n"
    "      <origin xyz=\"1 0 0\" /> \n"
    "      <inertia  ixx=\"0\" ixy=\"0\" ixz=\"0\" iyy=\"0\" iyz=\"0\" izz=\"0\" />\n"
    "    </inertial>\n"
    "  </link>\n"
    "  <joint name=\"joint_2\" type=\"continuous\" >\n"
    "    <axis xyz=\"0 0 1\" />\n"
    "    <origin xyz=\"1 0 0\" rpy=\"0 0 0\" />\n"
    "    <parent link=\"link_1\" />\n"
    "    <child link=\"link_2\" />\n"
    "    <dynamics damping=\"0.0\" />\n"
    "  </joint>\n"
    "  <link name=\"link_2\">\n"
    "    <inertial>\n"
    "      <mass value=\"1\" />\n"
    "      <origin xyz=\"1 0 0\" /> \n"
    "      <inertia  ixx=\"0\" ixy=\"0\" ixz=\"0\" iyy=\"0\" iyz=\"0\" izz=\"0\" />\n"
    "    </inertial>\n"
    "  </link>\n"
    "  <joint name=\"joint_3\" type=\"continuous\" >\n"
    "    <axis xyz=\"0 0 1\" />\n"
    "    <origin xyz=\"1 0 0\" rpy=\"0 0 0\" />\n"
    "    <parent link=\"link_2\" />\n"
    "    <child link=\"link_3\" />\n"
    "    <dynamics damping=\"0.0\" />\n"
    "  </joint>\n"
    "  <link name=\"link_3\">\n"
    "    <inertial>\n"
    "      <mass value=\"1\" />\n"
    "      <origin xyz=\"1 0 0\" /> \n"
    "      <inertia  ixx=\"0\" ixy=\"0\" ixz=\"0\" iyy=\"0\" iyz=\"0\" izz=\"0\" />\n"
    "    </inertial>\n"
    "  </link>\n"
    "  <joint name=\"joint_4\" type=\"continuous\" >\n"
    "    <axis xyz=\"0 0 1\" />\n"
    "    <origin xyz=\"1 0 0\" rpy=\"0 0 0\" />\n"
    "    <parent link=\"link_3\" />\n"
    "    <child link=\"link_4\" />\n"
    "    <dynamics damping=\"0.0\" />\n"
    "  </joint>\n"
    "  <link name=\"link_4\">\n"
    "    <inertial>\n"
    "      <mass value=\"1\" />\n"
    "      <origin xyz=\"1 0 0\" /> \n"
    "      <inertia  ixx=\"0\" ixy=\"0\" ixz=\"0\" iyy=\"0\" iyz=\"0\" izz=\"0\" />\n"
    "    </inertial>\n"
    "  </link>\n"
    "  <joint name=\"joint_5\" type=\"continuous\" >\n"
    "    <axis xyz=\"0 0 1\" />\n"
    "    <origin xyz=\"1 0 0\" rpy=\"0 0 0\" />\n"
    "    <parent link=\"link_4\" />\n"
    "    <child link=\"link_5\" />\n"
    "    <dynamics damping=\"0.0\" />\n"
    "  </joint>\n"
    "  <link name=\"link_5\">\n"
    "    <inertial>\n"
    "      <mass value=\"1\" />\n"
    "      <origin xyz=\"1 0 0\" /> \n"
    "      <inertia  ixx=\"0\" ixy=\"0\" ixz=\"0\" iyy=\"0\" iyz=\"0\" izz=\"0\" />\n"
    "    </inertial>\n"
    "  </link>\n"
    "</robot>\n";
  TiXmlDocument urdf_xml;
  urdf_xml.Parse(urdf_string);
  TiXmlElement * urdf_root(urdf_xml.FirstChildElement("robot"));
  if ( ! urdf_root) {
    throw runtime_error("no <robot> element");
  }
  urdf::Model * urdf_model(new urdf::Model());
  if ( ! urdf_model->initXml(urdf_root)) {
    delete urdf_model;
    throw runtime_error("urdf_model.initXml() failed");
  }
  return urdf_model;
}
