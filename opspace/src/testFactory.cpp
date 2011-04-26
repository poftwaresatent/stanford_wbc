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

#include <gtest/gtest.h>
#include <opspace/Skill.hpp>
#include <opspace/Factory.hpp>
#include <opspace/parse_yaml.hpp>
#include <stdexcept>

using namespace opspace;
using boost::shared_ptr;
using jspace::State;
using namespace std;


TEST (parse, tasks_only)
{
  static char * const yaml_string =
    "- tasks:\n"
    "  - type: opspace::SelectedJointPostureTask\n"
    "    name: odd\n"
    "    selection: [  1.0,  0.0,  1.0,  0.0,  1.0,  0.0 ]\n"
    "    kp: 100.0\n"
    "    kd:  20.0\n"
    "  - type: opspace::SelectedJointPostureTask\n"
    "    name: even\n"
    "    selection: [  0.0,  1.0,  0.0,  1.0,  0.0,  1.0 ]\n"
    "    kp: 100.0\n"
    "    kd:  20.0\n"
    "  - type: opspace::CartPosTrjTask\n"
    "    name: eepos\n"
    "    dt_seconds: 0.002\n"
    "    kp: [ 100.0 ]\n"
    "    kd: [  20.0 ]\n"
    "    maxvel: [ 0.5 ]\n"
    "    maxacc: [ 1.5 ]\n"
    "  - type: opspace::JPosTrjTask\n"
    "    name: posture\n"
    "    dt_seconds: 0.002\n"
    "    kp: [ 400.0, 400.0, 400.0, 100.0, 100.0, 100.0, 100.0 ]\n"
    "    kd: [  40.0,  40.0,  40.0,  20.0,  20.0,  20.0,  20.0 ]\n"
    "    maxvel: [ 3.1416 ]\n"
    "    maxacc: [ 6.2832 ]\n";
  
  try {
    Factory::setDebugStream(&cout);
    Factory factory;
    Status st;
    st = factory.parseString(yaml_string);
    EXPECT_TRUE (st.ok) << st.errstr;
    EXPECT_FALSE (factory.getTaskTable().empty()) << "task table should not be empty";
    EXPECT_TRUE (factory.getSkillTable().empty()) << "skill table should be empty";
    factory.dump(cout, "*** dump of factory", "* ");
  }
  catch (YAML::Exception const & ee) {
    ADD_FAILURE () << "unexpected YAML::Exception: " << ee.what();
  }
  catch (std::runtime_error const & ee) {
    ADD_FAILURE () << "unexpected std::runtime_error: " << ee.what();
  }
}


TEST (parse, tasks_and_skills)
{
  static char * const yaml_string =
    "- tasks:\n"
    "  - type: opspace::SelectedJointPostureTask\n"
    "    name: odd_instance\n"
    "    selection: [  1.0,  0.0,  1.0,  0.0,  1.0,  0.0 ]\n"
    "    kp: 100.0\n"
    "    kd:  20.0\n"
    "  - type: opspace::SelectedJointPostureTask\n"
    "    name: even_instance\n"
    "    selection: [  0.0,  1.0,  0.0,  1.0,  0.0,  1.0 ]\n"
    "    kp: 100.0\n"
    "    kd:  20.0\n"
    "  - type: opspace::CartPosTrjTask\n"
    "    name: eepos_trj\n"
    "    dt_seconds: 0.002\n"
    "    kp: [ 100.0 ]\n"
    "    kd: [  20.0 ]\n"
    "    maxvel: [ 0.5 ]\n"
    "    maxacc: [ 1.5 ]\n"
    "  - type: opspace::JPosTrjTask\n"
    "    name: posture_trj\n"
    "    dt_seconds: 0.002\n"
    "    kp: [ 400.0, 400.0, 400.0, 100.0, 100.0, 100.0, 100.0 ]\n"
    "    kd: [  40.0,  40.0,  40.0,  20.0,  20.0,  20.0,  20.0 ]\n"
    "    maxvel: [ 3.1416 ]\n"
    "    maxacc: [ 6.2832 ]\n"
    "  - type: opspace::CartPosTask\n"
    "    name: eepos_notrj\n"
    "    kp: [ 100.0 ]\n"
    "    kd: [  20.0 ]\n"
    "    maxvel: [ 0.5 ]\n"
    "  - type: opspace::JPosTask\n"
    "    name: posture_notrj\n"
    "    kp: [ 400.0, 400.0, 400.0, 100.0, 100.0, 100.0, 100.0 ]\n"
    "    kd: [  40.0,  40.0,  40.0,  20.0,  20.0,  20.0,  20.0 ]\n"
    "    maxvel: [ 3.1416 ]\n"
    "- skills:\n"
    "  - type: opspace::TaskPostureTrjSkill\n"
    "    name: tpb\n"
    "    slots:\n"
    "      eepos: eepos_trj\n"
    "      posture: posture_trj\n"
    "  - type: opspace::TaskPostureSkill\n"
    "    name: tpb\n"
    "    slots:\n"
    "      eepos: eepos_notrj\n"
    "      posture: posture_notrj\n";
  
  try {
    Factory::setDebugStream(&cout);
    Factory factory;
    Status st;
    st = factory.parseString(yaml_string);
    EXPECT_TRUE (st.ok) << st.errstr;
    EXPECT_FALSE (factory.getTaskTable().empty()) << "task table should not be empty";
    EXPECT_FALSE (factory.getSkillTable().empty()) << "skills table should not be empty";
    factory.dump(cout, "*** dump of factory", "* ");
  }
  catch (YAML::Exception const & ee) {
    ADD_FAILURE () << "unexpected YAML::Exception: " << ee.what();
  }
  catch (std::runtime_error const & ee) {
    ADD_FAILURE () << "unexpected std::runtime_error: " << ee.what();
  }
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS ();
}
