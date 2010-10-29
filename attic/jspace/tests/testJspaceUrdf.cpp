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
   \file testJspaceUrdf.cpp
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

static urdf::Model * create_unit_mass_5R_urdf() throw(runtime_error);


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


TEST (jspaceROSModel, urdf_dynamics_5R)
{
  urdf::Model const * urdf_model(0);
  std::vector<jspace::tao_tree_info_s*> tao_trees;
  jspace::Model * model(0);
  jspace::Model * model_check(0);
  
  try {
    urdf_model = create_unit_mass_5R_urdf();
    
    static const size_t n_tao_trees(2);
    convert_urdf_to_tao_n(*urdf_model, "world", jspace::ros::DefaultLinkFilter(), tao_trees, n_tao_trees);
    
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
  delete urdf_model;
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS ();
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
