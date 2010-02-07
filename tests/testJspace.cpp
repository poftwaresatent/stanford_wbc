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
#include <tao/dynamics/taoDNode.h>
#include <jspace/Model.hpp>
#include <gtest/gtest.h>
#include <errno.h>
#include <string.h>

using namespace std;


static std::string create_tmp_xml() throw(runtime_error);
static wbc::BranchingRepresentation * create_brep() throw(runtime_error);
static jspace::Model * create_model() throw(runtime_error);

static string xml_filename("");


TEST (basic, state)
{
  jspace::Model * model(0);
  try {
    model = create_model();
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


TEST (basic, branching)
{
  jspace::Model * model(0);
  try {
    model = create_model();
    EXPECT_EQ (model->getNNodes(), 7) << "Puma should have 7 nodes";
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
      EXPECT_EQ (ii, model->getNodeByName(node_name[ii])->getID())
	<< "Node with name \"" << node_name[ii] << "\" should have ID " << ii;
      EXPECT_NE ((void*)0, model->getNodeByJointName(joint_name[ii]))
	<< "Could not get node by joint name \"" << joint_name[ii] << "\"";
      EXPECT_EQ (ii, model->getNodeByJointName(joint_name[ii])->getID())
	<< "Node with joint name \"" << joint_name[ii] << "\" should have ID " << ii;
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


std::string create_tmp_xml() throw(runtime_error)
{
  static char tmpname[64];
  memset(tmpname, '\0', 64);
  strncpy(tmpname, "puma.xml.XXXXXX", 63);
  int const tmpfd(mkstemp(tmpname));
  if (-1 == tmpfd) {
    throw runtime_error("create_tmp_xml(): mkstemp(): " + string(strerror(errno)));
  }
  
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
  size_t const len(strlen(xml));
  if (len != write(tmpfd, xml, len)) {
    throw runtime_error("create_tmp_xml(): write(): " + string(strerror(errno)));
  }
  close(tmpfd);
  string result(tmpname);
  return result;
}


wbc::BranchingRepresentation * create_brep() throw(runtime_error)
{
  if (xml_filename.empty()) {
    xml_filename = create_tmp_xml();
  }
  wbc::BranchingRepresentation * brep(wbc::BRParser::parse("sai", xml_filename));
  return brep;
}


jspace::Model * create_model() throw(runtime_error)
{
  jspace::Model * model(new jspace::Model(new wbc::RobotControlModel(create_brep()), true));
  return model;
}
