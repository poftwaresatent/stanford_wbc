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

#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/parse/BRParser.hpp>
#include <wbc/util/dump.hpp>
#include <tao/dynamics/taoDNode.h>
#include <jspace/Model.hpp>
#include <wbcnet/strutil.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <gtest/gtest.h>
#include <errno.h>
#include <string.h>

using namespace std;


static std::string create_puma_frames() throw(runtime_error);
static jspace::Model * create_puma_model() throw(runtime_error);
static jspace::Model * create_unit_RR_model() throw(runtime_error);
static jspace::Model * create_unit_RP_model() throw(runtime_error);


TEST (jspaceModel, state)
{
  jspace::Model * model(0);
  try {
    model = create_puma_model();
    int const ndof(model->getNDOF());
    jspace::State state_in(ndof, ndof);
    if (0 != gettimeofday(&state_in.acquisition_time_, 0)) {
      FAIL () << "gettimeofday(): " << strerror(errno);
    }
    for (int ii(0); ii < ndof; ++ii) {
      state_in.joint_angles_[ii] = -3 + ii * 0.5;
      state_in.joint_velocities_[ii] = 3 - ii * 0.25;
    }
    model->setState(state_in);
    {
      jspace::State state_out(ndof, ndof);
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
    jspace::State state(ndof, ndof);
    
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
	  ipos >> state.joint_angles_[ii];
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
	<< "  pos: " << state.joint_angles_ << "\n"
	<< "  ID: " << id << "\n"
	<< "  expected: " << SAIQuaternion(rw, rx, ry, rz) << "\n"
	<< "  computed: " << transform.rotation();
      EXPECT_TRUE (transform.translation().equal(SAIVector3(tx, ty, tz), 1e-6))
	<< "translation mismatch\n"
	<< "  entry: " << joint_positions_count << "\n"
	<< "  pos: " << state.joint_angles_ << "\n"
	<< "  ID: " << id << "\n"
	<< "  expected: " << SAIVector3(tx, ty, tz) << "\n"
	<< "  computed: " << transform.translation();
      
#ifdef VERBOSE
      cout << "PASSED transform check\n"
	   << "  entry: " << joint_positions_count << "\n"
	   << "  pos: " << state.joint_angles_ << "\n"
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


static bool check_matrix(char const * name,
			 SAIMatrix const & want,
			 SAIMatrix const & have,
			 double precision,
			 std::ostringstream & msg)
{
  int const nrows(want.row());
  if (nrows != have.row()) {
    msg << "check_matrix(" << name << ") size mismatch: have " << have.row()
	<< " rows but want " << nrows << "\n";
    return false;
  }
  int const ncolumns(want.column());
  if (ncolumns != have.column()) {
    msg << "check_matrix(" << name << ") size mismatch: have " << have.column()
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


static bool check_vector(char const * name,
			 SAIVector const & want,
			 SAIVector const & have,
			 double precision,
			 std::ostream & msg)
{
  int const nelems(want.size());
  if (nelems != have.size()) {
    msg << "check_vector(" << name << ") size mismatch: have " << have.size()
	<< " elements but want " << nelems << "\n";
    return false;
  }
  
  precision = fabs(precision);
  double maxdelta(0);
  SAIVector delta(nelems);
  for (int ii(0); ii < nelems; ++ii) {
    delta[ii] = fabs(smart_delta(have[ii], want[ii]));
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
  delta.prettyPrint(msg, "  delta", "    ");
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
    model = create_unit_RR_model();
    taoDNode * ee(model->getNode(0));
    ASSERT_NE ((void*)0, ee) << "no end effector (node ID 0)";
    jspace::State state(2, 2);	// here we're only gonna test the first joint though
    state.joint_angles_.zero();
    state.joint_velocities_.zero();
    SAITransform ee_lframe;
    ee_lframe.translation().elementAt(1) = 1;
    
    for (double qq(-M_PI); qq <= M_PI; qq += 2 * M_PI / 7) {
      state.joint_angles_[0] = qq;
      model->update(state);
      
      double const q1(state.joint_angles_[0]);
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
	msg << "Verifying end-effector frame (position only) for q = " << state.joint_angles_ << "\n";
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
	msg << "Checking Jacobian for q = " << state.joint_angles_ << "\n";
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
    model = create_unit_RR_model();
    taoDNode * ee(model->getNode(1));
    ASSERT_NE ((void*)0, ee) << "no end effector (node ID 1)";
    jspace::State state(2, 2);
    state.joint_velocities_.zero();
    SAITransform ee_lframe;
    ee_lframe.translation().elementAt(1) = 1;
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-M_PI); q2 <= M_PI; q2 += 2 * M_PI / 7) {
 	state.joint_angles_[0] = q1;
 	state.joint_angles_[1] = q2;
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
	  msg << "Verifying end-effector frame (position only) for q = " << state.joint_angles_ << "\n";
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
	  msg << "Checking Jacobian for q = " << state.joint_angles_ << "\n";
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
    model = create_unit_RP_model();
    taoDNode * ee(model->getNode(1));
    ASSERT_NE ((void*)0, ee) << "no end effector (node ID 1)";
    jspace::State state(2, 2);
    state.joint_velocities_.zero();
    SAITransform ee_lframe;
    ee_lframe.translation().elementAt(1) = 1;
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-1); q2 <= 1; q2 += 2.0 / 7) {
 	state.joint_angles_[0] = q1;
 	state.joint_angles_[1] = q2;
	model->update(state);
	
	double const c1(cos(q1));
	double const s1(sin(q1));
	
	SAITransform ee_gframe;
	ASSERT_TRUE (model->computeGlobalFrame(ee, ee_lframe, ee_gframe));
	SAIVector const & ee_gpos(ee_gframe.translation());
	{
	  SAIVector ee_gpos_check(3);
	  ee_gpos_check.elementAt(1) = c1 - s1 * q2;
	  ee_gpos_check.elementAt(2) = s1 + c1 * q2;
	  std::ostringstream msg;
	  msg << "Verifying end-effector frame (position only) for q = " << state.joint_angles_ << "\n";
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
	  Jg_check.elementAt(1, 0) = -s1 - c1 * q2;
	  Jg_check.elementAt(2, 0) =  c1 - s1 * q2;
	  Jg_check.elementAt(3, 0) =  1;
	  Jg_check.elementAt(1, 1) = -s1;
	  Jg_check.elementAt(2, 1) =  c1;
	  Jg_check.elementAt(3, 1) =  0;
	  std::ostringstream msg;
	  msg << "Checking Jacobian for q = " << state.joint_angles_ << "\n";
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


TEST (jspaceModel, mass_inertia_RR)
{
  jspace::Model * model(0);
  try {
    model = create_unit_RR_model();
    taoDNode * n1(model->getNode(0));
    taoDNode * n2(model->getNode(1));
    ASSERT_NE ((void*)0, n1);
    ASSERT_NE ((void*)0, n2);
    jspace::State state(2, 2);
    state.joint_velocities_.zero();
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-M_PI); q2 <= M_PI; q2 += 2 * M_PI / 7) {
 	state.joint_angles_[0] = q1;
 	state.joint_angles_[1] = q2;
	model->update(state);
	
	double const c1(cos(q1));
	double const c12(cos(q1+q2));
	double const s1(sin(q1));
	double const s12(sin(q1+q2));
	
	SAIMatrix MM(2, 2);
	model->getMassInertia(MM);
	SAIMatrix MM_check(2, 2);
	MM_check.elementAt(0, 0) = 1 + pow(s1 + s12, 2) + pow(c1 + c12, 2);
	MM_check.elementAt(1, 0) = s12 * (s1 + s12) + c12 * (c1 + c12);
	MM_check.elementAt(0, 1) = MM_check.elementAt(1, 0);
	MM_check.elementAt(1, 1) = 1;
	{
	  std::ostringstream msg;
	  msg << "Checking mass_inertia for q = " << state.joint_angles_ << "\n";
	  MM_check.prettyPrint(msg, "  want", "    ");
	  MM.prettyPrint(msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("mass_inertia", MM_check, MM, 1e-3, msg)) << msg.str();
	}
	
	SAIMatrix MMinv(2, 2);
	model->getInverseMassInertia(MMinv);
	SAIMatrix MMinv_check;
	MM_check.inverse(MMinv_check);
	{
	  SAIMatrix id_check;
	  id_check.identity(2);
	  SAIMatrix id;
	  id = MM_check * MMinv_check;
	  std::ostringstream msg;
	  msg << "Checking SAIMatrix::inverse() for q = " << state.joint_angles_ << "\n";
	  id_check.prettyPrint(msg, "  want", "    ");
	  id.prettyPrint(msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("identity", id_check, id, 1e-3, msg)) << msg.str();
	}
	{
	  std::ostringstream msg;
	  msg << "Checking inv_mass_inertia for q = " << state.joint_angles_ << "\n";
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




// // static void check_dynamics(jspace::Model * model, jspace::State const & state, taoDNode const * end_effector)
// // {
// //   SAIVector tB(6), tG(6);
// //   SAIMatrix tJ(6, 6), tA(6, 6);
// //   SAIVector mB(6), mG(6);
// //   SAIMatrix mJ(6, 6), mdJ(6, 6), mA(6, 6);
  
// //   getPumaDynamics(state.joint_angles_, state.joint_velocities_,
// // 		  mJ, mdJ, mA, mB, mG);
// //   model->update(state);
// //   ASSERT_TRUE (model->computeJacobian(end_effector, tJ)) << "computeJacobian failed for q = " << state.joint_angles_;
// //   model->getMassInertia(tA);
// //   model->getCoriolisCentrifugal(tB);
// //   model->getGravity(tG);
// //   {
// //     std::ostringstream msg;
// //     msg << "Checking Jacobian for q = "  << state.joint_angles_ << "\n";
// //     mJ.prettyPrint(msg, "  want", "    ");
// //     tJ.prettyPrint(msg, "  have", "    ");
// //     EXPECT_TRUE (check_matrix("Jacobian", mJ, tJ, 1e-1, msg)) << msg.str();
// //   }
// //   {
// //     std::ostringstream msg;
// //     msg << "Checking mass inertia for q = "  << state.joint_angles_ << "\n";
// //     mA.prettyPrint(msg, "  want", "    ");
// //     tA.prettyPrint(msg, "  have", "    ");
// //     EXPECT_TRUE (check_matrix("mass inertia", mA, tA, 1e-1, msg)) << msg.str();
// //   }
// //   {
// //     std::ostringstream msg;
// //     msg << "Checking Coriolis centrifugal for q = "  << state.joint_angles_ << "\n";
// //     mB.prettyPrint(msg, "  want", "    ");
// //     tB.prettyPrint(msg, "  have", "    ");
// //     EXPECT_TRUE (check_vector("Coriolis centrifugal", mB, tB, 1e-1, msg)) << msg.str();
// //   }
// //   {
// //     std::ostringstream msg;
// //     msg << "Checking gravity for q = "  << state.joint_angles_ << "\n";
// //     mG.prettyPrint(msg, "  want", "    ");
// //     tG.prettyPrint(msg, "  have", "    ");
// //     EXPECT_TRUE (check_vector("gravity", mG, tG, 1e-1, msg)) << msg.str();
// //   }
// // }


// // TEST (jspaceModel, dynamics)
// // {
// //   jspace::Model * model(0);
// //   try {
// //     model = create_model();
// //     taoDNode * ee(model->getNode(5));
// //     ASSERT_NE ((void*)0, ee) << "no end effector (node ID 5)";
// //     jspace::State state(6, 6);
// //     state.joint_angles_.zero();
// //     state.joint_velocities_.zero();
// //     check_dynamics(model, state, ee);    
// //     for (double qq(-0.1); qq < 0.11; qq += 0.1) {
// //       for (int ii(0); ii < 6; ++ii) {
// // 	state.joint_angles_.zero();
// // 	state.joint_angles_[ii] = qq;
// // 	check_dynamics(model, state, ee);    
// //       }
// //     }
// //   }
// //   catch (std::exception const & ee) {
// //     ADD_FAILURE () << "exception " << ee.what();
// //   }
// //   delete model;
// // }


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS ();
}


static std::string create_tmpfile(char const * fname_template, char const * contents) throw(runtime_error)
{
  if (strlen(fname_template) >= 64) {
    throw runtime_error("create_tmpfile(): fname_template is too long (max 63 characters)");
  }
  
  static char tmpname[64];
  memset(tmpname, '\0', 64);
  strncpy(tmpname, fname_template, 63);
  int const tmpfd(mkstemp(tmpname));
  if (-1 == tmpfd) {
    throw runtime_error("create_tmpfile(): mkstemp(): " + string(strerror(errno)));
  }
  
  size_t const len(strlen(contents));
  if (static_cast<ssize_t>(len) != write(tmpfd, contents, len)) {
    throw runtime_error("create_tmpfile(): write(): " + string(strerror(errno)));
  }
  close(tmpfd);
  
  string result(tmpname);
  return result;
}


static std::string create_puma_xml() throw(runtime_error)
{
  static char const * xml = 
    "<?xml version=\"1.0\" ?>\n"
    "<dynworld>\n"
    "  <baseNode>\n"
    "    <robotName>Puma</robotName>\n"
    "    <gravity>0, 0, -9.81</gravity>\n"
    "    <pos>0, 0, 0</pos>\n"
    "    <rot>1, 0, 0, 0</rot>\n"
    "    <name>base</name>\n"
    "    <ID>-1</ID>\n"
    "    <jointNode>\n"
    "      <jointName>shoulder-yaw</jointName>\n"
    "      <linkName>base</linkName>\n"
    "      <upperJointLimit>0.1</upperJointLimit>\n"
    "      <lowerJointLimit>-0.1</lowerJointLimit>\n"
    "      <defaultJointPosition>0</defaultJointPosition>\n"
    "      <type>R</type>\n"
    "      <axis>Z</axis>\n"
    "      <mass>34.40</mass>\n"
    "      <inertia>0.00, 0.00, 1.49</inertia>\n"
    "      <com>0, 0, 0</com>\n"
    "      <pos>0, 0, 0</pos>\n"
    "      <rot>0, 0, 1, 0</rot>\n"
    "      <name>NoName</name>\n"
    "      <ID>0</ID>\n"
    "      <jointNode>\n"
    "        <jointName>shoulder-pitch</jointName>\n"
    "        <linkName>upper_arm</linkName>\n"
    "        <upperJointLimit>0.1</upperJointLimit>\n"
    "        <lowerJointLimit>-0.1</lowerJointLimit>\n"
    "        <defaultJointPosition>0</defaultJointPosition>\n"
    "        <type>R</type>\n"
    "        <axis>Z</axis>\n"
    "        <mass>17.40</mass>\n"
    "        <inertia>0.13, 0.524, 5.249</inertia>\n"
    "        <com>0.068, 0.006, -0.016</com>\n"
    "        <pos>0.0, 0.2435, 0.0</pos>\n"
    "        <rot>1, 0, 0, -1.57079632679489661923</rot>\n"
    "        <name>upper_arm</name>\n"
    "        <ID>1</ID>\n"
    "        <jointNode>\n"
    "          <jointName>elbow</jointName>\n"
    "          <linkName>lower_arm</linkName>\n"
    "          <upperJointLimit>0.1</upperJointLimit>\n"
    "          <lowerJointLimit>-0.1</lowerJointLimit>\n"
    "          <defaultJointPosition>0</defaultJointPosition>\n"
    "          <type>R</type>\n"
    "          <axis>Z</axis>\n"
    "          <mass>6.04</mass>\n"
    "          <inertia>0.192, 0.0154, 1.042</inertia>\n"
    "          <com>0, -0.143, 0.014</com>\n"
    "          <pos>0.4318, 0, -0.0934</pos>\n"
    "          <rot>1, 0, 0, 0</rot>\n"
    "          <name>lower_arm</name>\n"
    "          <ID>2</ID>\n"
    "          <jointNode>\n"
    "            <jointName>wrist-roll1</jointName>\n"
    "            <linkName>wrist-hand</linkName>\n"
    "            <upperJointLimit>0.1</upperJointLimit>\n"
    "            <lowerJointLimit>-0.1</lowerJointLimit>\n"
    "            <defaultJointPosition>0</defaultJointPosition>\n"
    "            <type>R</type>\n"
    "            <axis>Z</axis>\n"
    "            <mass>0.82</mass>\n"
    "            <inertia>0.0018, 0.0018, 0.2013</inertia>\n"
    "            <com>0.0, 0.0, -0.019</com>\n"
    "            <pos>-0.0203, -0.4331, 0.0</pos>\n"
    "            <rot>1, 0, 0, 1.57079632679489661923</rot>\n"
    "            <name>wrist-hand</name>\n"
    "            <ID>3</ID>\n"
    "            <jointNode>\n"
    "              <jointName>wrist-pitch</jointName>\n"
    "              <linkName>wrist-finger</linkName>\n"
    "              <upperJointLimit>0.1</upperJointLimit>\n"
    "              <lowerJointLimit>-0.1</lowerJointLimit>\n"
    "              <defaultJointPosition>0</defaultJointPosition>\n"
    "              <type>R</type>\n"
    "              <axis>Z</axis>\n"
    "              <mass>0.34</mass>\n"
    "              <inertia>0.0003, 0.0003, 0.1794</inertia>\n"
    "              <com>0.0, 0.0, 0.0</com>\n"
    "              <pos>0, 0, 0</pos>\n"
    "              <rot>1, 0, 0, -1.57079632679489661923</rot>\n"
    "              <name>wrist-finger</name>\n"
    "              <ID>4</ID>\n"
    "              <jointNode>\n"
    "                <jointName>wrist-roll2</jointName>\n"
    "                <linkName>end-effector</linkName>\n"
    "                <upperJointLimit>0.1</upperJointLimit>\n"
    "                <lowerJointLimit>-0.1</lowerJointLimit>\n"
    "                <defaultJointPosition>0</defaultJointPosition>\n"
    "                <type>R</type>\n"
    "                <axis>Z</axis>\n"
    "                <mass>0.09</mass>\n"
    "                <inertia>0.00015, 0.00015, 0.19304</inertia>\n"
    "                <com>0.0, 0.0, 0.032</com>\n"
    "                <pos>0, 0, 0</pos>\n"
    "                <rot>1, 0, 0, 1.57079632679489661923</rot>\n"
    "                <name>end-effector</name>\n"
    "                <ID>5</ID>\n"
    "              </jointNode>\n"
    "            </jointNode>\n"
    "          </jointNode>\n"
    "        </jointNode>\n"
    "      </jointNode>\n"
    "    </jointNode>\n"
    "  </baseNode>\n"
    "</dynworld>\n";
  std::string result(create_tmpfile("puma.xml.XXXXXX", xml));
  return result;
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


static wbc::BranchingRepresentation * create_puma_brep() throw(runtime_error)
{
  static string xml_filename("");
  if (xml_filename.empty()) {
    xml_filename = create_puma_xml();
  }
  wbc::BranchingRepresentation * brep(wbc::BRParser::parse("sai", xml_filename));
  return brep;
}


jspace::Model * create_puma_model() throw(runtime_error)
{
  wbc::BranchingRepresentation * kg_brep(create_puma_brep());
  wbc::tao_tree_info_s * kg_tree(create_tao_tree_info(*kg_brep));
  delete kg_brep;
  
  wbc::BranchingRepresentation * cc_brep(create_puma_brep());
  wbc::tao_tree_info_s * cc_tree(create_tao_tree_info(*cc_brep));
  delete cc_brep;
  
  //   cout << "created jspace::Model:\n";
  //   wbc::dump_tao_tree_info(cout, kg_tree, "  ", false);
  
  jspace::Model * model(new jspace::Model(kg_tree, cc_tree));
  return model;
}


static std::string create_unit_RR_xml() throw(runtime_error)
{
  static char const * xml = 
    "<?xml version=\"1.0\" ?>\n"
    "<dynworld>\n"
    "  <baseNode>\n"
    "    <gravity>0, 0, -9.81</gravity>\n"
    "    <pos>0, 0, 0</pos>\n"
    "    <rot>1, 0, 0, 0</rot>\n"
    "    <jointNode>\n"
    "      <ID>0</ID>\n"
    "      <type>R</type>\n"
    "      <axis>X</axis>\n"
    "      <mass>1</mass>\n"
    "      <inertia>0, 0, 0</inertia>\n"
    "      <com>0, 1, 0</com>\n"
    "      <pos>0, 0, 0</pos>\n"
    "      <rot>0, 0, 1, 0</rot>\n"
    "      <jointNode>\n"
    "        <ID>1</ID>\n"
    "        <type>R</type>\n"
    "        <axis>X</axis>\n"
    "        <mass>1</mass>\n"
    "        <inertia>0, 0, 0</inertia>\n"
    "        <com>0, 1, 0</com>\n"
    "        <pos>0, 1, 0</pos>\n"
    "        <rot>0, 0, 1, 0</rot>\n"
    "      </jointNode>\n"
    "    </jointNode>\n"
    "  </baseNode>\n"
    "</dynworld>\n";
  std::string result(create_tmpfile("unit_RR.xml.XXXXXX", xml));
  return result;
}


static wbc::BranchingRepresentation * create_unit_RR_brep() throw(runtime_error)
{
  static string xml_filename("");
  if (xml_filename.empty()) {
    xml_filename = create_unit_RR_xml();
  }
  wbc::BranchingRepresentation * brep(wbc::BRParser::parse("sai", xml_filename));
  return brep;
}


jspace::Model * create_unit_RR_model() throw(runtime_error)
{
  wbc::BranchingRepresentation * kg_brep(create_unit_RR_brep());
  wbc::tao_tree_info_s * kg_tree(create_tao_tree_info(*kg_brep));
  delete kg_brep;
  
  wbc::BranchingRepresentation * cc_brep(create_unit_RR_brep());
  wbc::tao_tree_info_s * cc_tree(create_tao_tree_info(*cc_brep));
  delete cc_brep;
  
  //   cout << "created jspace::Model:\n";
  //   wbc::dump_tao_tree_info(cout, kg_tree, "  ", false);
  
  jspace::Model * model(new jspace::Model(kg_tree, cc_tree));
  return model;
}


static std::string create_unit_RP_xml() throw(runtime_error)
{
  static char const * xml = 
    "<?xml version=\"1.0\" ?>\n"
    "<dynworld>\n"
    "  <baseNode>\n"
    "    <gravity>0, 0, -9.81</gravity>\n"
    "    <pos>0, 0, 0</pos>\n"
    "    <rot>1, 0, 0, 0</rot>\n"
    "    <jointNode>\n"
    "      <ID>0</ID>\n"
    "      <type>R</type>\n"
    "      <axis>X</axis>\n"
    "      <mass>1</mass>\n"
    "      <inertia>0, 0, 0</inertia>\n"
    "      <com>0, 1, 0</com>\n"
    "      <pos>0, 0, 0</pos>\n"
    "      <rot>0, 0, 1, 0</rot>\n"
    "      <jointNode>\n"
    "        <ID>1</ID>\n"
    "        <type>P</type>\n"
    "        <axis>Z</axis>\n"
    "        <mass>1</mass>\n"
    "        <inertia>0, 0, 0</inertia>\n"
    "        <com>0, 0, 0</com>\n"
    "        <pos>0, 0, 0</pos>\n"
    "        <rot>0, 0, 1, 0</rot>\n"
    "      </jointNode>\n"
    "    </jointNode>\n"
    "  </baseNode>\n"
    "</dynworld>\n";
  std::string result(create_tmpfile("unit_RP.xml.XXXXXX", xml));
  return result;
}


static wbc::BranchingRepresentation * create_unit_RP_brep() throw(runtime_error)
{
  static string xml_filename("");
  if (xml_filename.empty()) {
    xml_filename = create_unit_RP_xml();
  }
  wbc::BranchingRepresentation * brep(wbc::BRParser::parse("sai", xml_filename));
  return brep;
}


jspace::Model * create_unit_RP_model() throw(runtime_error)
{
  wbc::BranchingRepresentation * kg_brep(create_unit_RP_brep());
  wbc::tao_tree_info_s * kg_tree(create_tao_tree_info(*kg_brep));
  delete kg_brep;
  
  wbc::BranchingRepresentation * cc_brep(create_unit_RP_brep());
  wbc::tao_tree_info_s * cc_tree(create_tao_tree_info(*cc_brep));
  delete cc_brep;
  
  //   cout << "created jspace::Model:\n";
  //   wbc::dump_tao_tree_info(cout, kg_tree, "  ", false);
  
  jspace::Model * model(new jspace::Model(kg_tree, cc_tree));
  return model;
}
