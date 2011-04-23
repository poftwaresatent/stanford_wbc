/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (C) 2010 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
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

#include <jspace/inertia_util.hpp>
#include <jspace/test/model_library.hpp>
#include <jspace/test/util.hpp>
#include <jspace/test/sai_brep_parser.hpp>
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoDynamics.h>
#include <tao/dynamics/taoJoint.h>
#include <jspace/tao_dump.hpp>
#include <jspace/vector_util.hpp>
#include <jspace/controller_library.hpp>
#include <jspace/strutil.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <gtest/gtest.h>
#include <errno.h>

#include <Eigen/SVD>
#include <Eigen/LU>

using namespace std;
using namespace jspace;
using namespace jspace::test;

static std::string create_puma_frames() throw(runtime_error);


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
  
  //// Eigen version trouble?
  // Transform identity_transform(Transform::Identity());
  Transform identity_transform;
  identity_transform.setIdentity();
  
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
      
      jspace::Transform transform;
      if ( ! model->getGlobalFrame(model->getNode(id), transform)) {
	FAIL() << frames_filename << ": line " << line_count << ": could not get global frame " << id << " from model";
      }
      jspace::Quaternion quat_computed(transform.rotation());
      jspace::Quaternion quat_expected(rw, rx, ry, rz);
      double delta_angle(quat_computed.angularDistance(quat_expected));
      EXPECT_TRUE (fabs(delta_angle) < 0.15 * M_PI / 180.0) // looks like it's hard to be more precise...
	<< "rotation mismatch\n"
	<< "  entry: " << joint_positions_count << "\n"
	<< "  pos: " << state.position_ << "\n"
	<< "  ID: " << id << "\n"
	<< "  expected: " << pretty_string(quat_expected) << "\n"
	<< "  computed: " << pretty_string(quat_computed) << "\n"
	<< "  delta_angle: " << delta_angle * 180 / M_PI << " deg\n";
      Eigen::Vector3d trans_expected(tx, ty, tz);
      EXPECT_TRUE (compare(transform.translation(), trans_expected, 1e-6))
	<< "translation mismatch\n"
	<< "  entry: " << joint_positions_count << "\n"
	<< "  pos: " << state.position_ << "\n"
	<< "  ID: " << id << "\n"
	<< "  expected: " << pretty_string(trans_expected) << "\n"
	<< "  computed: " << pretty_string(transform.translation());
      
      // exercise (some of) the jspace::Model::computeGlobalFrame() methods
      {
	Transform check_t;
	
	if ( ! model->computeGlobalFrame(model->getNode(id), identity_transform, check_t)) {
	  FAIL() << frames_filename << ": line " << line_count
		 << ": computeGlobalFrame() failed on local identity transform";
	}
	Quaternion check_q(check_t.rotation());
	delta_angle = quat_computed.angularDistance(check_q);
	EXPECT_TRUE (fabs(delta_angle) < 0.15 * M_PI / 180.0)
	  << "model->computeGlobalFrame() rotation mismatch on local identity transform\n"
	  << "  computed: " << pretty_string(check_q) << "\n"
	  << "  expected: " << pretty_string(quat_computed) << "\n"
	  << "  delta_angle: " << delta_angle * 180 / M_PI << " deg\n";
	EXPECT_TRUE (compare(transform.translation(), check_t.translation(), 1e-6))
	  << "model->computeGlobalFrame() translation mismatch on local identity transform\n"
	  << "  computed: " << pretty_string(check_t.translation()) << "\n"
	  << "  expected: " << pretty_string(transform.translation());
	
	if ( ! model->computeGlobalFrame(model->getNode(id), 0, 0, 0, check_t)) {
	  FAIL() << frames_filename << ": line " << line_count
		 << ": computeGlobalFrame() failed on (0,0,0) translation";
	}
	check_q = check_t.rotation();
	delta_angle = quat_computed.angularDistance(check_q);
	EXPECT_TRUE (fabs(delta_angle) < 0.15 * M_PI / 180.0)
	  << "model->computeGlobalFrame() rotation mismatch on (0,0,0) translation\n"
	  << "  computed: " << pretty_string(check_q) << "\n"
	  << "  expected: " << pretty_string(quat_computed) << "\n"
	  << "  delta_angle: " << delta_angle * 180 / M_PI << " deg\n";
	EXPECT_TRUE (compare(transform.translation(), check_t.translation(), 1e-6))
	  << "model->computeGlobalFrame() translation mismatch on (0,0,0) translation\n"
	  << "  computed: " << pretty_string(check_t.translation()) << "\n"
	  << "  expected: " << pretty_string(transform.translation());
	
	Vector zerov(Vector::Zero(3));
	if ( ! model->computeGlobalFrame(model->getNode(id), zerov, check_t)) {
	  FAIL() << frames_filename << ": line " << line_count
		 << ": computeGlobalFrame() failed on zero translation";
	}
	check_q = check_t.rotation();
	delta_angle = quat_computed.angularDistance(check_q);
	EXPECT_TRUE (fabs(delta_angle) < 0.15 * M_PI / 180.0)
	  << "model->computeGlobalFrame() rotation mismatch on zero translation\n"
	  << "  computed: " << pretty_string(check_q) << "\n"
	  << "  expected: " << pretty_string(quat_computed) << "\n"
	  << "  delta_angle: " << delta_angle * 180 / M_PI << " deg\n";
	EXPECT_TRUE (compare(transform.translation(), check_t.translation(), 1e-6))
	  << "model->computeGlobalFrame() translation mismatch on zero translation\n"
	  << "  computed: " << pretty_string(check_t.translation()) << "\n"
	  << "  expected: " << pretty_string(transform.translation());
      }
      
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceModel, Jacobian_R)
{
  jspace::Model * model(0);
  try {
    model = create_unit_mass_RR_model();
    taoDNode * ee(model->getNode(0));
    ASSERT_NE ((void*)0, ee) << "no end effector (node ID 0)";
    jspace::State state(2, 2, 0); // here we're only gonna test the first joint though
    jspace::Transform ee_lframe(Eigen::Translation<double, 3>(0, 1, 0));
    
    for (double qq(-M_PI); qq <= M_PI; qq += 2 * M_PI / 7) {
      state.position_[0] = qq;
      model->update(state);
      
      double const q1(state.position_[0]);
      double const c1(cos(q1));
      double const s1(sin(q1));
      
      jspace::Transform ee_gframe;
      ASSERT_TRUE (model->computeGlobalFrame(ee, ee_lframe, ee_gframe));
      jspace::Vector const ee_gpos(ee_gframe.translation());
      {
	jspace::Vector ee_gpos_check(3);
	ee_gpos_check.coeffRef(0) = 0;
	ee_gpos_check.coeffRef(1) = c1;
	ee_gpos_check.coeffRef(2) = s1;
	std::ostringstream msg;
	msg << "Verifying end-effector frame (position only) for q = " << state.position_ << "\n"
	    << "  want: " << pretty_string(ee_gpos_check) << "\n"
	    << "  have: " << pretty_string(ee_gpos) << "\n";
	bool const gpos_ok(check_vector("ee_pos", ee_gpos_check, ee_gpos, 1e-3, msg));
	EXPECT_TRUE (gpos_ok) << msg.str();
	if ( ! gpos_ok) {
	  continue;		// no use checking Jg as well, it'll be off
	}
      }
      
      jspace::Matrix Jg_all(6, 2);
      ASSERT_TRUE (model->computeJacobian(ee, ee_gpos, Jg_all));
      jspace::Matrix const Jg(Jg_all.block(0, 0, 6, 1));
      {
	jspace::Matrix Jg_check(jspace::Matrix::Zero(6, 1));
	Jg_check.coeffRef(1, 0) = -s1;
	Jg_check.coeffRef(2, 0) =  c1;
	Jg_check.coeffRef(3, 0) =  1;
	std::ostringstream msg;
	msg << "Checking Jacobian for q = " << state.position_ << "\n";
	pretty_print(Jg_check, msg, "  want", "    ");
	pretty_print(Jg, msg, "  have", "    ");
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
    jspace::Transform ee_lframe(Eigen::Translation<double, 3>(0, 1, 0));
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-M_PI); q2 <= M_PI; q2 += 2 * M_PI / 7) {
 	state.position_[0] = q1;
 	state.position_[1] = q2;
	model->update(state);
	
	double const c1(cos(q1));
	double const c12(cos(q1+q2));
	double const s1(sin(q1));
	double const s12(sin(q1+q2));
	
	jspace::Transform ee_gframe;
	ASSERT_TRUE (model->computeGlobalFrame(ee, ee_lframe, ee_gframe));
	jspace::Vector const ee_gpos(ee_gframe.translation());
	{
	  jspace::Vector ee_gpos_check(3);
	  ee_gpos_check.coeffRef(0) = 0;
	  ee_gpos_check.coeffRef(1) = c1 + c12;
	  ee_gpos_check.coeffRef(2) = s1 + s12;
	  std::ostringstream msg;
	  msg << "Verifying end-effector frame (position only) for q = " << state.position_ << "\n";
	  pretty_print(ee_gpos_check, msg, "  want", "    ");
	  pretty_print(ee_gpos, msg, "  have", "    ");
	  bool const gpos_ok(check_vector("ee_pos", ee_gpos_check, ee_gpos, 1e-3, msg));
	  EXPECT_TRUE (gpos_ok) << msg.str();
	  if ( ! gpos_ok) {
	    continue;		// no use checking Jg as well, it'll be off
	  }
	}
	
	jspace::Matrix Jg(6, 2);
	ASSERT_TRUE (model->computeJacobian(ee, ee_gpos, Jg));
	{
	  jspace::Matrix Jg_check(jspace::Matrix::Zero(6, 2));
	  Jg_check.coeffRef(1, 0) = -s1 - s12;
	  Jg_check.coeffRef(2, 0) =  c1 + c12;
	  Jg_check.coeffRef(3, 0) =  1;
	  Jg_check.coeffRef(1, 1) = -s12;
	  Jg_check.coeffRef(2, 1) =  c12;
	  Jg_check.coeffRef(3, 1) =  1;
	  std::ostringstream msg;
	  msg << "Checking Jacobian for q = " << state.position_ << "\n";
	  pretty_print(Jg_check, msg, "  want", "    ");
	  pretty_print(Jg, msg, "  have", "    ");
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


TEST (jspaceModel, com_RR)
{
  jspace::Model * model(0);
  try {
    model = create_unit_mass_RR_model();
    jspace::State state(2, 2, 0);
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-M_PI); q2 <= M_PI; q2 += 2 * M_PI / 7) {
 	state.position_[0] = q1;
 	state.position_[1] = q2;
	model->update(state);
	
	double const c1(cos(q1));
	double const c12(cos(q1+q2));
	double const s1(sin(q1));
	double const s12(sin(q1+q2));
	
	jspace::Vector com;
	jspace::Matrix jcom;
	ASSERT_TRUE (model->computeCOM(com, &jcom)) << "failed to computeCOM()";
	
	// remember, it's a double-pendulum in the YZ plane, and
	// coincides with the Y axis at q=0... so, YZ looks like the
	// X'Y' you get when sketching it out on a piece of paper, and
	// X===0
	{
	  jspace::Vector com_check(3);
	  com_check.coeffRef(0) = 0;
	  com_check.coeffRef(1) = c1 + 0.5 * c12;
	  com_check.coeffRef(2) = s1 + 0.5 * s12;
	  std::ostringstream msg;
	  msg << "Verifying COM position for q = " << state.position_ << "\n";
	  pretty_print(com_check, msg, "  want", "    ");
	  pretty_print(com, msg, "  have", "    ");
	  EXPECT_TRUE (check_vector("com", com_check, com, 1e-3, msg)) << msg.str();
	}
	
	{
	  jspace::Matrix jcom_check(jspace::Matrix::Zero(3, 2));
	  jcom_check.coeffRef(1, 0) = -s1 - 0.5 * s12;
	  jcom_check.coeffRef(2, 0) =  c1 + 0.5 * c12;
	  jcom_check.coeffRef(1, 1) = -0.5 * s12;
	  jcom_check.coeffRef(2, 1) =  0.5 * c12;
	  std::ostringstream msg;
	  msg << "Checking Jacobian COM for q = " << state.position_ << "\n";
	  pretty_print(jcom_check, msg, "  want", "    ");
	  pretty_print(jcom, msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("Jacobian COM", jcom_check, jcom, 1e-3, msg)) << msg.str();
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
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-1); q2 <= 1; q2 += 2.0 / 7) {
 	state.position_[0] = q1;
 	state.position_[1] = q2;
	model->update(state);
	
	double const c1(cos(q1));
	double const s1(sin(q1));
	
	{
	  jspace::Transform ee_gframe;
	  ASSERT_TRUE (model->getGlobalFrame(ee, ee_gframe));
	  jspace::Vector const ee_gpos(ee_gframe.translation());
	  jspace::Vector ee_gpos_check(3);
	  ee_gpos_check.coeffRef(0) = 0;
	  ee_gpos_check.coeffRef(1) = c1 - s1 * q2;
	  ee_gpos_check.coeffRef(2) = s1 + c1 * q2;
	  std::ostringstream msg;
	  msg << "Verifying end-effector frame (position only) for q = " << state.position_ << "\n";
	  pretty_print(ee_gpos_check, msg, "  want", "    ");
	  pretty_print(ee_gpos, msg, "  have", "    ");
	  bool const gpos_ok(check_vector("ee_pos", ee_gpos_check, ee_gpos, 1e-3, msg));
	  EXPECT_TRUE (gpos_ok) << msg.str();
	  if ( ! gpos_ok) {
	    continue;		// no use checking Jg as well, it'll be off
	  }
	}
	
	jspace::Matrix Jg(6, 2);
	ASSERT_TRUE (model->computeJacobian(ee, Jg));
	{
	  jspace::Matrix Jg_check(jspace::Matrix::Zero(6, 2));
	  Jg_check.coeffRef(1, 0) = -s1 - c1 * q2;
	  Jg_check.coeffRef(2, 0) =  c1 - s1 * q2;
	  Jg_check.coeffRef(3, 0) =  1;
	  Jg_check.coeffRef(1, 1) = -s1;
	  Jg_check.coeffRef(2, 1) =  c1;
	  Jg_check.coeffRef(3, 1) =  0;
	  std::ostringstream msg;
	  msg << "Checking Jacobian for q = " << state.position_ << "\n";
	  pretty_print(Jg_check, msg, "  want", "    ");
	  pretty_print(Jg, msg, "  have", "    ");
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


TEST (jspaceModel, com_RP)
{
  jspace::Model * model(0);
  try {
    model = create_unit_mass_RP_model();
    jspace::State state(2, 2, 0);
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-1); q2 <= 1; q2 += 2.0 / 7) {
 	state.position_[0] = q1;
 	state.position_[1] = q2;
	model->update(state);
	
	double const c1(cos(q1));
	double const s1(sin(q1));
	
	jspace::Vector com;
	jspace::Matrix jcom;
	ASSERT_TRUE (model->computeCOM(com, &jcom)) << "failed to computeCOM()";
	
	// remember, it's a pendulum-piston in the YZ plane, and
	// coincides with the Y axis at q=0... so, YZ looks like the
	// X'Y' you get when sketching it out on a piece of paper, and
	// X===0
	{
	  jspace::Vector com_check(3);
	  com_check.coeffRef(0) = 0;
	  com_check.coeffRef(1) = c1 - 0.5 * s1 * q2;
	  com_check.coeffRef(2) = s1 + 0.5 * c1 * q2;
	  std::ostringstream msg;
	  msg << "Verifying COM position only for q = " << state.position_ << "\n";
	  pretty_print(com_check, msg, "  want", "    ");
	  pretty_print(com, msg, "  have", "    ");
	  EXPECT_TRUE (check_vector("ee_pos", com_check, com, 1e-3, msg)) << msg.str();
	}
	
	{
	  jspace::Matrix jcom_check(jspace::Matrix::Zero(3, 2));
	  jcom_check.coeffRef(1, 0) = -s1 - 0.5 * c1 * q2;
	  jcom_check.coeffRef(2, 0) =  c1 - 0.5 * s1 * q2;
	  jcom_check.coeffRef(1, 1) = -0.5 * s1;
	  jcom_check.coeffRef(2, 1) =  0.5 * c1;
	  std::ostringstream msg;
	  msg << "Checking Jacobian COM for q = " << state.position_ << "\n";
	  pretty_print(jcom_check, msg, "  want", "    ");
	  pretty_print(jcom, msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("Jacobian", jcom_check, jcom, 1e-3, msg)) << msg.str();
	}
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceModel, explicit_mass_inertia_RR)
{
  typedef jspace::Model * (*create_model_t)();
  create_model_t create_model[] = {
    create_unit_mass_RR_model,
    create_unit_inertia_RR_model
  };
  
  typedef void (*compute_mass_inertia_t)(double, double, jspace::Matrix &);
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
      
      for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
	for (double q2(-M_PI); q2 <= M_PI; q2 += 2 * M_PI / 7) {
	  state.position_[0] = q1;
	  state.position_[1] = q2;
	  model->update(state);
	  
	  jspace::Matrix MM(2, 2);
	  std::ostringstream dbgos;
	  mass_inertia_explicit_form(*model, MM, &dbgos);
	  jspace::Matrix MM_check;
	  compute_mass_inertia[test_index](q1, q2, MM_check);
	  {
	    std::ostringstream msg;
	    msg << "Checking explicit mass-inertia for test_index " << test_index
		<< " q = " << state.position_ << "\n";
	    pretty_print(MM_check, msg, "  want", "    ");
	    pretty_print(MM, msg, "  have", "    ");
	    EXPECT_TRUE (check_matrix("mass_inertia", MM_check, MM, 1e-3, msg)) << msg.str() << dbgos.str();
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


TEST (jspaceModel, com_frame_fork_4R)
{
  jspace::Model * model(0);
  try {
    model = create_fork_4R_model();
    taoDNode * node[4];
    jspace::Vector lcompos[4];
    for (size_t ii(0); ii < 4; ++ii) {
      node[ii] = model->getNode(ii);
      ASSERT_NE ((void*)0, node[ii]) << "no node " << ii+1 << " (ID " << ii << ")";
      deVector3 const * com(node[ii]->center());
      lcompos[ii] = jspace::Vector::Zero(3);
      if (com) {
	lcompos[ii].x() = com->elementAt(0);
	lcompos[ii].y() = com->elementAt(1);
	lcompos[ii].z() = com->elementAt(2);
      }
    }
    jspace::State state(4, 4, 0);
    
    bool keep_running(true);
    
    for (double q1(-M_PI); keep_running && (q1 <= M_PI); q1 += 2 * M_PI / 7) {
      for (double q2(-M_PI); keep_running && (q2 <= M_PI); q2 += 2 * M_PI / 7) {
	for (double q3(-M_PI); keep_running && (q3 <= M_PI); q3 += 2 * M_PI / 7) {
	  for (double q4(-M_PI); keep_running && (q4 <= M_PI); q4 += 2 * M_PI / 7) {
	    state.position_[0] = q1;
	    state.position_[1] = q2;
	    state.position_[2] = q3;
	    state.position_[3] = q4;
	    model->update(state);
	    
	    for (size_t ii(0); ii < 4; ++ii) {
	      jspace::Transform gframe;
	      ASSERT_TRUE (model->getGlobalFrame(node[ii], gframe))
		<< "failed to get global frame of node " << ii+1 << " (ID " << ii << ")";
	      jspace::Transform gcomframe;
	      ASSERT_TRUE (model->computeGlobalCOMFrame(node[ii], gcomframe))
		<< "failed to compute global COM frame of node " << ii+1 << " (ID " << ii << ")";
	      // Eigen doc says we should be able to just write
	      // "gframe * lcompos[ii]" but that does not actually
	      // work.
	      jspace::Vector check_gcompos((gframe * Eigen::Translation3d(lcompos[ii])).translation());
	      {
		std::ostringstream msg;
		msg << "Verifying COM position of node " << ii+1 << " (ID " << ii << ") for q = "
		    << state.position_ << "\n";
		pretty_print(check_gcompos, msg, "  want", "    ");
		pretty_print(static_cast<jspace::Vector const &>(gcomframe.translation()), msg, "  have", "    ");
		const bool ok(check_vector("gcompos", check_gcompos, gcomframe.translation(), 1e-6, msg));
		EXPECT_TRUE (ok) << msg.str();
		if ( ! ok) {
		  keep_running = false;
		}
	      }
	      {
		std::ostringstream msg;
		msg << "Verifying COM frame rotation of node " << ii+1 << " (ID " << ii << ") for q = "
		    << state.position_ << "\n";
		pretty_print(static_cast<jspace::Matrix const &>(gframe.linear()), msg, "  want", "    ");
		pretty_print(static_cast<jspace::Matrix const &>(gcomframe.linear()), msg, "  have", "    ");
		const bool ok(check_matrix("gcomrot", gframe.linear(), gcomframe.linear(), 1e-6, msg));
		EXPECT_TRUE (ok) << msg.str();
		if ( ! ok) {
		  keep_running = false;
		}
	      }
	    }
	  }
	}
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceModel, kinematics_fork_4R)
{
  jspace::Model * model(0);
  try {
    model = create_fork_4R_model();
    taoDNode * node[4];
    for (size_t ii(0); ii < 4; ++ii) {
      node[ii] = model->getNode(ii);
      ASSERT_NE ((void*)0, node[ii]) << "no node " << ii+1 << " (ID " << ii << ")";
    }
    jspace::State state(4, 4, 0);
    
    struct kinematics_s {
      jspace::Vector origin;
      jspace::Vector com;
      jspace::Matrix Jacobian;
    };
    
    kinematics_s have[4], want[4];
    bool keep_running(true);
    
    for (double q1(-M_PI); keep_running && (q1 <= M_PI); q1 += 2 * M_PI / 7) {
      for (double q2(-M_PI); keep_running && (q2 <= M_PI); q2 += 2 * M_PI / 7) {
	for (double q3(-M_PI); keep_running && (q3 <= M_PI); q3 += 2 * M_PI / 7) {
	  for (double q4(-M_PI); keep_running && (q4 <= M_PI); q4 += 2 * M_PI / 7) {
	    state.position_[0] = q1;
	    state.position_[1] = q2;
	    state.position_[2] = q3;
	    state.position_[3] = q4;
	    model->update(state);
	    
	    for (size_t ii(0); ii < 4; ++ii) {
	      jspace::Transform gframe;
	      ASSERT_TRUE (model->getGlobalFrame(node[ii], gframe))
		<< "failed to get global frame of node " << ii+1 << " (ID " << ii << ")";
	      have[ii].origin = gframe.translation();
	      ASSERT_TRUE (model->computeGlobalCOMFrame(node[ii], gframe))
		<< "failed to compute global COM of node " << ii+1 << " (ID " << ii << ")";
	      have[ii].com = gframe.translation();
	      ASSERT_TRUE (model->computeJacobian(node[ii], have[ii].Jacobian))
		<< "failed to compute Jacobian of node " << ii+1 << " (ID " << ii << ")";
	    }
	    
	    compute_fork_4R_kinematics(q1, q2, q3, q4,
				       want[0].origin, want[1].origin, want[2].origin, want[3].origin,
				       want[0].com, want[1].com, want[2].com, want[3].com,
				       want[0].Jacobian, want[1].Jacobian, want[2].Jacobian, want[3].Jacobian);
	    
	    for (size_t ii(0); ii < 4; ++ii) {
	      std::ostringstream msg;
	      msg << "Verifying origin of node " << ii+1 << " (ID " << ii << ") for q = "
		  << state.position_ << "\n";
	      pretty_print(want[ii].origin, msg, "  want", "    ");
	      pretty_print(have[ii].origin, msg, "  have", "    ");
	      const bool ok(check_vector("origin", want[ii].origin, have[ii].origin, 1e-3, msg));
	      EXPECT_TRUE (ok) << msg.str();
	      if ( ! ok) {
	      	keep_running = false;
	      }
	    }
	    if ( ! keep_running) {
	      break;		// others will fail as well, just skip them...
	    }
	    
	    for (size_t ii(0); ii < 4; ++ii) {
	      std::ostringstream msg;
	      msg << "Verifying COM of node " << ii+1 << " (ID " << ii << ") for q = "
		  << state.position_ << "\n";
	      pretty_print(want[ii].com, msg, "  want", "    ");
	      pretty_print(have[ii].com, msg, "  have", "    ");
	      deVector3 const * local_com(node[ii]->center());
	      if (local_com) {
		msg << "  local COM according to TAO: " << *local_com << "\n"
		    << "  and what we make of it: "
		    << local_com->elementAt(0) << "  "
		    << local_com->elementAt(1) << "  "
		    << local_com->elementAt(2) << "\n";
	      }
	      else {
		msg << "  local COM according to TAO: null\n";
	      }
	      const bool ok(check_vector("COM", want[ii].com, have[ii].com, 1e-3, msg));
	      EXPECT_TRUE (ok) << msg.str();
	      if ( ! ok) {
	      	keep_running = false;
	      }
	    }
	    if ( ! keep_running) {
	      break;		// others will fail as well, just skip them...
	    }
	    
	    for (size_t ii(0); ii < 4; ++ii) {
	      std::ostringstream msg;
	      msg << "Verifying Jacobian of node " << ii+1 << " (ID " << ii << ") for q = "
		  << state.position_ << "\n";
	      pretty_print(want[ii].Jacobian, msg, "  want", "    ");
	      pretty_print(have[ii].Jacobian, msg, "  have", "    ");
	      const bool ok(check_matrix("Jacobian", want[ii].Jacobian, have[ii].Jacobian, 1e-3, msg));
	      EXPECT_TRUE (ok) << msg.str();
	      if ( ! ok) {
	      	keep_running = false;
	      }
	    }
	  }
	}
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceModel, mass_inertia_fork_4R)
{
  jspace::Model * model(0);
  try {
    model = create_fork_4R_model();
    jspace::State state(4, 4, 0);
    
    bool keep_running(true);
    
    for (double q1(-M_PI); keep_running && (q1 <= M_PI); q1 += 2 * M_PI / 7) {
      for (double q2(-M_PI); keep_running && (q2 <= M_PI); q2 += 2 * M_PI / 7) {
	for (double q3(-M_PI); keep_running && (q3 <= M_PI); q3 += 2 * M_PI / 7) {
	  for (double q4(-M_PI); keep_running && (q4 <= M_PI); q4 += 2 * M_PI / 7) {
	    state.position_[0] = q1;
	    state.position_[1] = q2;
	    state.position_[2] = q3;
	    state.position_[3] = q4;
	    model->update(state);
	    
	    Matrix recursive_mass_inertia;
	    if ( ! model->getMassInertia(recursive_mass_inertia)) {
	      FAIL() << "jspace::Model::getMassInertia() failed";
	    }
	    
	    ostringstream dbgos;
	    Matrix explicit_mass_inertia;
	    mass_inertia_explicit_form(*model, explicit_mass_inertia, &dbgos);
	    
	    std::ostringstream msg;
	    msg << "Comparing recursive with explicit mass inertia for q = " << state.position_ << "\n";
	    pretty_print(recursive_mass_inertia, msg, "  recursive", "    ");
	    pretty_print(explicit_mass_inertia, msg, "  explicit", "    ");
	    bool const ok(check_matrix("mass_inertia", recursive_mass_inertia, explicit_mass_inertia, 1e-3, msg));
	    EXPECT_TRUE (ok) << msg.str() << dbgos.str();
	    if ( ! ok) {
	      keep_running = false;
	    }
	  }
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
  typedef jspace::Model * (*create_model_t)();
  create_model_t create_model[] = {
    create_unit_mass_RR_model,
    create_unit_inertia_RR_model
  };
  
  typedef void (*compute_mass_inertia_t)(double, double, jspace::Matrix &);
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
      
      for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
	for (double q2(-M_PI); q2 <= M_PI; q2 += 2 * M_PI / 7) {
	  state.position_[0] = q1;
	  state.position_[1] = q2;
	  model->update(state);
	  
	  jspace::Matrix MM(2, 2);
	  model->getMassInertia(MM);
	  jspace::Matrix MM_check;
	  compute_mass_inertia[test_index](q1, q2, MM_check);
	  {
	    std::ostringstream msg;
	    msg << "Checking mass_inertia for test_index " << test_index
		<< " q = " << state.position_ << "\n";
	    pretty_print(MM_check, msg, "  want", "    ");
	    pretty_print(MM, msg, "  have", "    ");
	    EXPECT_TRUE (check_matrix("mass_inertia", MM_check, MM, 1e-3, msg)) << msg.str();
	  }
	  
	  jspace::Matrix MMinv(2, 2);
	  model->getInverseMassInertia(MMinv);
	  jspace::Matrix MMinv_check(2, 2);
	  MM_check.computeInverse(&MMinv_check);
	  {
	    std::ostringstream msg;
	    msg << "Checking inv_mass_inertia for test_index " << test_index
		<< " q = " << state.position_ << "\n";
	    pretty_print(MMinv_check, msg, "  want", "    ");
	    pretty_print(MMinv, msg, "  have", "    ");
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
    
    for (size_t ii(0); ii < 5; ++ii) {
      for (double qq(-M_PI); qq <= M_PI; qq += 2 * M_PI / 7) {
	memset(&state.position_[0], 0, 5 * sizeof(double));
	state.position_[ii] = qq;
	model->update(state);
	
	jspace::Matrix MM;
	model->getMassInertia(MM);
	std::ostringstream msg;
	bool ok(true);
	for (size_t jj(0); jj < 5; ++jj) {
	  if (MM.coeff(jj, jj) < 1e-3) {
	    ok = false;
	    msg << "  MM[" << jj << "][" << jj << "] = " << MM.coeff(jj, jj) << "\n";
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
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-1); q2 <= 1; q2 += 2.0 / 7) {
 	state.position_[0] = q1;
 	state.position_[1] = q2;
	model->update(state);
	
	double const c1(cos(q1));
	double const s1(sin(q1));
	
	jspace::Matrix MM(2, 2);
	model->getMassInertia(MM);
	jspace::Matrix MM_check(2, 2);
	MM_check.coeffRef(0, 0) = 1 + pow(s1 + c1 * q2, 2) + pow(c1 - s1 * q2, 2);
	MM_check.coeffRef(1, 0) = s1 * (s1 + c1 * q2) + c1 * (c1 - s1 * q2);
	MM_check.coeffRef(0, 1) = MM_check.coeff(1, 0);
	MM_check.coeffRef(1, 1) = 1;
	{
	  std::ostringstream msg;
	  msg << "Checking mass_inertia for q = " << state.position_ << "\n";
	  pretty_print(MM_check, msg, "  want", "    ");
	  pretty_print(MM, msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("mass_inertia", MM_check, MM, 1e-3, msg)) << msg.str();
	}
	
	jspace::Matrix MMinv(2, 2);
	model->getInverseMassInertia(MMinv);
	{
	  jspace::Matrix id_check(2, 2);
	  id_check.setIdentity();
	  jspace::Matrix id;
	  id = MM * MMinv;
	  std::ostringstream msg;
	  msg << "Checking A * Ainv = I for q = " << state.position_ << "\n";
	  pretty_print(id_check, msg, "  want", "    ");
	  pretty_print(id, msg, "  have", "    ");
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
    Vector kp(2), kd(2);
    kp << 100, 100;
    kd << 20, 20;
    jspace::JointGoalController ctrl(jspace::COMP_MASS_INERTIA, kp, kd);
    jspace::Status status(ctrl.init(*model));
    ASSERT_TRUE (status) << "ctrl.init failed: " << status.errstr;
    status = ctrl.setGoal(state.position_);
    ASSERT_TRUE (status) << "ctrl.setGoal failed: " << status.errstr;
    Vector tau;
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-M_PI); q2 <= M_PI; q2 += 2 * M_PI / 7) {
	state.position_[0] = q1;
	state.position_[1] = q2;
	model->update(state);
	status = ctrl.computeCommand(*model, tau);
	ASSERT_EQ (tau.size(), 2);
	
	jspace::Matrix MM;
	compute_unit_mass_RR_mass_inertia(q1, q2, MM);
	double const m11(MM.coeff(0, 0));
	double const m12(MM.coeff(0, 1));
	double const m22(MM.coeff(1, 1));
	Vector tau_check(2);
	tau_check[0] = - kp[0] * (m11 * q1 + m12 * q2);
	tau_check[1] = - kp[0] * (m12 * q1 + m22 * q2);
	
	std::ostringstream msg;
	msg << "Verifying command for q = " << state.position_ << "\n"
	    << "  want: " << tau_check[0] << "  " << tau_check[1] << "\n"
	    << "  have: " << tau[0] << "  " << tau[1] << "\n";
	EXPECT_TRUE (check_vector("tau", tau_check, tau, 1e-3, msg)) << msg.str();
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
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
