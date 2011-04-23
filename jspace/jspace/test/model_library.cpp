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
   \file model_library.cpp
   \author Roland Philippsen
*/

#include "model_library.hpp"
#include "util.hpp"
#include "sai_brep.hpp"
#include "sai_brep_parser.hpp"
#include <string.h>
#include <errno.h>
#include <stdlib.h>

using namespace std;

namespace jspace {
  namespace test {

    
    typedef std::string (*create_xml_t)();
    typedef BranchingRepresentation * (*create_brep_t)();
    
    
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
    
    
    static BranchingRepresentation * _create_brep(create_xml_t create_xml) throw(runtime_error)
    {
      string const xml_filename(create_xml());
      BRParser brp;
      BranchingRepresentation * brep(brp.parse(xml_filename));
      return brep;
    }
    
    
    static BranchingRepresentation * create_puma_brep() throw(runtime_error)
    {
      return _create_brep(create_puma_xml);
    }
    
    
    static jspace::Model * _create_model(create_brep_t create_brep) throw(runtime_error)
    {
      BranchingRepresentation * kg_brep(create_brep());
      jspace::tao_tree_info_s * kg_tree(kg_brep->createTreeInfo());
      delete kg_brep;
      BranchingRepresentation * cc_brep(create_brep());
      jspace::tao_tree_info_s * cc_tree(cc_brep->createTreeInfo());
      delete cc_brep;
      jspace::Model * model(new jspace::Model());
      std::ostringstream msg;
      if ( 0 != model->init(kg_tree, cc_tree, &msg)) {
	delete model;
	throw std::runtime_error("jspace::test::_create_model(): model->init() failed: " + msg.str());
      }
      return model;
    }
    
    
    jspace::Model * create_puma_model() throw(runtime_error)
    {
      return _create_model(create_puma_brep);
    }
    

    static std::string create_unit_mass_RR_xml() throw(runtime_error)
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
      std::string result(create_tmpfile("unit_mass_RR.xml.XXXXXX", xml));
      return result;
    }


    static BranchingRepresentation * create_unit_mass_RR_brep() throw(runtime_error)
    {
      return _create_brep(create_unit_mass_RR_xml);
    }


    jspace::Model * create_unit_mass_RR_model() throw(runtime_error)
    {
      return _create_model(create_unit_mass_RR_brep);
    }


    static std::string create_unit_mass_5R_xml() throw(runtime_error)
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
	"      <axis>Z</axis>\n"
	"      <mass>1</mass>\n"
	"      <inertia>0, 0, 0</inertia>\n"
	"      <com>1, 0, 0</com>\n"
	"      <pos>0, 0, 2</pos>\n"
	"      <rot>1, 0, 0, 0</rot>\n"
	"      <jointNode>\n"
	"        <ID>1</ID>\n"
	"        <type>R</type>\n"
	"        <axis>Z</axis>\n"
	"        <mass>1</mass>\n"
	"        <inertia>0, 0, 0</inertia>\n"
	"        <com>1, 0, 0</com>\n"
	"        <pos>1, 0, 0</pos>\n"
	"        <rot>1, 0, 0, 0</rot>\n"
	"        <jointNode>\n"
	"          <ID>2</ID>\n"
	"          <type>R</type>\n"
	"          <axis>Z</axis>\n"
	"          <mass>1</mass>\n"
	"          <inertia>0, 0, 0</inertia>\n"
	"          <com>1, 0, 0</com>\n"
	"          <pos>1, 0, 0</pos>\n"
	"          <rot>1, 0, 0, 0</rot>\n"
	"          <jointNode>\n"
	"            <ID>3</ID>\n"
	"            <type>R</type>\n"
	"            <axis>Z</axis>\n"
	"            <mass>1</mass>\n"
	"            <inertia>0, 0, 0</inertia>\n"
	"            <com>1, 0, 0</com>\n"
	"            <pos>1, 0, 0</pos>\n"
	"            <rot>1, 0, 0, 0</rot>\n"
	"            <jointNode>\n"
	"              <ID>4</ID>\n"
	"              <type>R</type>\n"
	"              <axis>Z</axis>\n"
	"              <mass>1</mass>\n"
	"              <inertia>0, 0, 0</inertia>\n"
	"              <com>1, 0, 0</com>\n"
	"              <pos>1, 0, 0</pos>\n"
	"              <rot>1, 0, 0, 0</rot>\n"
	"            </jointNode>\n"
	"          </jointNode>\n"
	"        </jointNode>\n"
	"      </jointNode>\n"
	"    </jointNode>\n"
	"  </baseNode>\n"
	"</dynworld>\n";
      std::string result(create_tmpfile("unit_mass_5R.xml.XXXXXX", xml));
      return result;
    }


    BranchingRepresentation * create_unit_mass_5R_brep() throw(runtime_error)
    {
      return _create_brep(create_unit_mass_5R_xml);
    }
    
    
    jspace::Model * create_unit_mass_5R_model() throw(runtime_error)
    {
      return _create_model(create_unit_mass_5R_brep);
    }


    static std::string create_unit_inertia_RR_xml() throw(runtime_error)
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
	"      <inertia>1, 1, 1</inertia>\n"
	"      <com>0, 1, 0</com>\n"
	"      <pos>0, 0, 0</pos>\n"
	"      <rot>0, 0, 1, 0</rot>\n"
	"      <jointNode>\n"
	"        <ID>1</ID>\n"
	"        <type>R</type>\n"
	"        <axis>X</axis>\n"
	"        <mass>1</mass>\n"
	"        <inertia>1, 1, 1</inertia>\n"
	"        <com>0, 1, 0</com>\n"
	"        <pos>0, 2, 0</pos>\n"
	"        <rot>0, 0, 1, 0</rot>\n"
	"      </jointNode>\n"
	"    </jointNode>\n"
	"  </baseNode>\n"
	"</dynworld>\n";
      std::string result(create_tmpfile("unit_inertia_RR.xml.XXXXXX", xml));
      return result;
    }


    static BranchingRepresentation * create_unit_inertia_RR_brep() throw(runtime_error)
    {
      return _create_brep(create_unit_inertia_RR_xml);
    }

    
    jspace::Model * create_unit_inertia_RR_model() throw(runtime_error)
    {
      return _create_model(create_unit_inertia_RR_brep);
    }


    static std::string create_unit_mass_RP_xml() throw(runtime_error)
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
	"        <pos>0, 1, 0</pos>\n"
	"        <rot>0, 0, 1, 0</rot>\n"
	"      </jointNode>\n"
	"    </jointNode>\n"
	"  </baseNode>\n"
	"</dynworld>\n";
      std::string result(create_tmpfile("unit_mass_RP.xml.XXXXXX", xml));
      return result;
    }


    static BranchingRepresentation * create_unit_mass_RP_brep() throw(runtime_error)
    {
      return _create_brep(create_unit_mass_RP_xml);
    }

    
    jspace::Model * create_unit_mass_RP_model() throw(runtime_error)
    {
      return _create_model(create_unit_mass_RP_brep);
    }
    
    
    static std::string create_fork_4R_xml() throw(runtime_error)
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
	"      <axis>Z</axis>\n"
	"      <mass>1</mass>\n"
	"      <inertia>0.3, 0.2, 0.1</inertia>\n"
	"      <com>0.5, 0, 0</com>\n"
	"      <pos>0, 0, 0</pos>\n"
	"      <rot>1, 0, 0, 0</rot>\n"
	"      <jointNode>\n"
	"        <ID>1</ID>\n"
	"        <type>R</type>\n"
	"        <axis>Z</axis>\n"
	"        <mass>1</mass>\n"
	"        <inertia>0.1, 0.2, 0.3</inertia>\n"
	"        <com>0, 0, 0</com>\n"
	"        <pos>1, 0, 0</pos>\n"
	"	<!-- Beware: the axis is not renormalized by the legacy SAI\n"
	"	     parser! So, 0.57735026919=sqrt(1/3) and\n"
	"	     2.09439510239=2pi/3 will rotate Z onto X, X onto Y, and Y\n"
	"	     onto Z -->\n"
	"        <rot>0.57735026919, 0.57735026919, 0.57735026919, 2.09439510239 </rot>\n"
	"        <jointNode>\n"
	"          <ID>2</ID>\n"
	"          <type>R</type>\n"
	"          <axis>Z</axis>\n"
	"          <mass>1</mass>\n"
	"          <inertia>0.2, 0.2, 0.1</inertia>\n"
	"          <com>0.5, 0, 0</com>\n"
	"          <pos>-1, 0, 0</pos>\n"
	"          <rot>0, 1, 0, -1.57079632679 </rot> -->\n"
	"        </jointNode>\n"
	"        <jointNode>\n"
	"          <ID>3</ID>\n"
	"          <type>R</type>\n"
	"          <axis>Z</axis>\n"
	"          <mass>1</mass>\n"
	"          <inertia>0.1, 0.2, 0.1</inertia>\n"
	"          <com>0.5, 0, 0</com>\n"
	"          <pos>1, 0, 0</pos>\n"
	"          <rot>0.707106781187, 0, 0.707106781187, 3.14159265359 </rot>\n"
	"	 </jointNode>\n"
	"      </jointNode>\n"
	"    </jointNode>\n"
	"  </baseNode>\n"
	"</dynworld>\n";
      std::string result(create_tmpfile("fork_4R.xml.XXXXXX", xml));
      return result;
    }
    
    
    static BranchingRepresentation * create_fork_4R_brep() throw(runtime_error)
    {
      return _create_brep(create_fork_4R_xml);
    }
    
    
    jspace::Model * create_fork_4R_model() throw(std::runtime_error)
    {
      return _create_model(create_fork_4R_brep);
    }
    
    
    void compute_fork_4R_kinematics(double q1, double q2, double q3, double q4,
				    jspace::Vector & o1, jspace::Vector & o2, jspace::Vector & o3, jspace::Vector & o4,
				    jspace::Vector & com1, jspace::Vector & com2,
				    jspace::Vector & com3, jspace::Vector & com4,
				    jspace::Matrix & J1, jspace::Matrix & J2, jspace::Matrix & J3, jspace::Matrix & J4)
    {
      static double const l1(1);
      static double const l2(1);
      
      double const c1(cos(q1));
      double const s1(sin(q1));
      double const c2(cos(q2));
      double const s2(sin(q2));
      double const c3(cos(q3));
      double const s3(sin(q3));
      double const c4(cos(q4));
      double const s4(sin(q4));
      
      Eigen::Matrix4d X1_0, X2_1, X3_2, X4_2;
      X1_0 <<
	c1, -s1, 0, 0,
	s1,  c1, 0, 0,
	0,    0, 1, 0,
	0,    0, 0, 1;
      X2_1 <<
	0,    0, 1, l1,
	c2, -s2, 0,  0,
	s2,  c2, 0,  0,
	0,    0, 0,  1;
      X3_2 <<
	0,    0, -1, -l2,
	s3,  c3,  0,   0,
	c3, -s3,  0,   0,
	0,    0,  0,   1;
      X4_2 <<
	0,     0, 1, l2,
	-s4, -c4, 0,  0,
	c4,  -s4, 0,  0,
	0,     0, 0,  1;
      
      o1 = Eigen::Vector3d::Zero();
      o2 = (X1_0 * X2_1.col(3)).block(0, 0, 3, 1);
      o3 = (X1_0 * (X2_1 * X3_2.col(3))).block(0, 0, 3, 1);
      o4 = (X1_0 * (X2_1 * X4_2.col(3))).block(0, 0, 3, 1);
      
      Eigen::Vector4d com1_1, com2_2, com3_3, com4_4;
      com1_1 << 0.5, 0, 0, 1;
      com2_2 << 0,   0, 0, 1;
      com3_3 << 0.5, 0, 0, 1;
      com4_4 << 0.5, 0, 0, 1;
      com1 = (X1_0 * com1_1).block(0, 0, 3, 1);
      com2 = (X1_0 * X2_1 * com2_2).block(0, 0, 3, 1);
      com3 = (X1_0 * X2_1 * X3_2 * com3_3).block(0, 0, 3, 1);
      com4 = (X1_0 * X2_1 * X4_2 * com4_4).block(0, 0, 3, 1);
      
      // z1 means Z-axis of joint 1 expressed in global frame (which is frame 0)
      Eigen::Vector3d z1, z2, z3, z4;
      z1 = X1_0.block(0, 2, 3, 1);
      z2 = X1_0.block(0, 0, 3, 3) * X2_1.block(0, 2, 3, 1);
      z3 = X1_0.block(0, 0, 3, 3) * X2_1.block(0, 0, 3, 3) * X3_2.block(0, 2, 3, 1);
      z4 = X1_0.block(0, 0, 3, 3) * X2_1.block(0, 0, 3, 3) * X4_2.block(0, 2, 3, 1);
      
      J1 = Eigen::MatrixXd::Zero(6, 4);
      J1.coeffRef(5, 0) = 1;
      
      J2 = Eigen::MatrixXd::Zero(6, 4);
      J2.block(0, 0, 3, 1) = z1.cross(o2 - o1);
      J2.block(3, 0, 3, 1) = z1;
      J2.block(3, 1, 3, 1) = z2;
      
      J3 = Eigen::MatrixXd::Zero(6, 4);
      J3.block(0, 0, 3, 1) = z1.cross(o3 - o1);
      J3.block(3, 0, 3, 1) = z1;
      J3.block(0, 1, 3, 1) = z2.cross(o3 - o2);
      J3.block(3, 1, 3, 1) = z2;
      J3.block(3, 2, 3, 1) = z3;
      
      J4 = Eigen::MatrixXd::Zero(6, 4);
      J4.block(0, 0, 3, 1) = z1.cross(o4 - o1);
      J4.block(3, 0, 3, 1) = z1;
      J4.block(0, 1, 3, 1) = z2.cross(o4 - o2);
      J4.block(3, 1, 3, 1) = z2;
      J4.block(3, 3, 3, 1) = z4;
    }
    
    
    void compute_unit_mass_RR_mass_inertia(double q1, double q2, jspace::Matrix & AA)
    {
      double const c1(cos(q1));
      double const c12(cos(q1+q2));
      double const s1(sin(q1));
      double const s12(sin(q1+q2));
      AA.resize(2, 2);
      AA.coeffRef(0, 0) = 1 + pow(s1 + s12, 2) + pow(c1 + c12, 2);
      AA.coeffRef(1, 0) = s12 * (s1 + s12) + c12 * (c1 + c12);
      AA.coeffRef(0, 1) = AA.coeff(1, 0);
      AA.coeffRef(1, 1) = 1;
    }


    void compute_unit_inertia_RR_mass_inertia(double q1, double q2, jspace::Matrix & AA)
    {
      double const c1(cos(q1));
      double const c12(cos(q1+q2));
      double const s1(sin(q1));
      double const s12(sin(q1+q2));
      AA.resize(2, 2);
      AA.coeffRef(0, 0) = 3 + pow(2*s1 + s12, 2) + pow(2*c1 + c12, 2);
      AA.coeffRef(1, 0) = 1 + s12 * (2*s1 + s12) + c12 * (2*c1 + c12);
      AA.coeffRef(0, 1) = AA.coeff(1, 0);
      AA.coeffRef(1, 1) = 2;
    }

  }
}
