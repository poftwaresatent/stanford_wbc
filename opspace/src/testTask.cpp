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
#include <opspace/task_library.hpp>
#include <opspace/skill_library.hpp>
#include <opspace/ClassicTaskPostureController.hpp>
#include <jspace/test/model_library.hpp>
#include <err.h>

using jspace::Model;
using jspace::State;
using jspace::pretty_print;
using namespace opspace;
using boost::shared_ptr;
using namespace std;


static Model * get_puma()
{
  static Model * puma(0);
  if ( ! puma) {
    puma = jspace::test::create_puma_model();
  }
  size_t const ndof(puma->getNDOF());
  State state(ndof, ndof, 0);
  for (size_t ii(0); ii < ndof; ++ii) {
    state.position_[ii] = 0.01 * ii + 0.08;
    state.velocity_[ii] = 0.02 - 0.005 * ii;
  }
  puma->update(state);
  return puma;
}


TEST (task, basics)
{
  Model * puma(get_puma());
  SelectedJointPostureTask odd("odd");
  Status st;
  
  st = odd.update(*puma);
  EXPECT_FALSE (st.ok) << "update before init should have failed";
  
  st = odd.init(*puma);
  EXPECT_FALSE (st.ok) << "init before selection setting should have failed";
  
  Parameter * selection(odd.lookupParameter("selection", PARAMETER_TYPE_VECTOR));
  ASSERT_NE ((void*)0, selection) << "failed to retrieve selection parameter";
  
  Vector sel(Vector::Zero(puma->getNDOF()));
  for (size_t ii(0); ii < puma->getNDOF(); ii += 2) {
    sel[ii] = 1.0;
  }
  st = selection->set(sel);
  EXPECT_TRUE (st.ok) << "failed to set selection: " << st.errstr;
  
  st = odd.init(*puma);
  EXPECT_TRUE (st.ok) << "init failed: " << st.errstr;
  
  st = odd.update(*puma);
  EXPECT_TRUE (st.ok) << "update failed: " << st.errstr;
}


static shared_ptr<Task> create_sel_jp_task(string const & name, Vector const & selection)
  throw(runtime_error)
{
  SelectedJointPostureTask * task(new SelectedJointPostureTask(name));
  Parameter * sel_p(task->lookupParameter("selection", PARAMETER_TYPE_VECTOR));
  if ( ! sel_p) {
    delete task;
    throw runtime_error("failed to retrieve selection parameter");
  }
  Status const st(sel_p->set(selection));
  if ( ! st) {
    delete task;
    throw runtime_error("failed to set selection: " + st.errstr);
  }
  return shared_ptr<Task>(task);
}


static void append_odd_even_tasks(GenericSkill gb, size_t ndof)
{
  vector<shared_ptr<Task> > task;
  Vector sel_odd(Vector::Zero(ndof));
  Vector sel_even(Vector::Zero(ndof));
  for (size_t ii(0); ii < ndof; ++ii) {
    if (0 == (ii % 2)) {
      sel_even[ii] = 1.0;
    }
    else {
      sel_odd[ii] = 1.0;
    }
  }
  task.push_back(create_sel_jp_task("odd", sel_odd));
  task.push_back(create_sel_jp_task("even", sel_even));
  for (size_t ii(0); ii < task.size(); ++ii) {
    gb.appendTask(task[ii]);
  }
}


static void append_odd_full_tasks(GenericSkill gb, size_t ndof)
{
  vector<shared_ptr<Task> > task;
  Vector sel_odd(Vector::Zero(ndof));
  Vector sel_full(Vector::Ones(ndof));
  for (size_t ii(1); ii < ndof; ii += 2) {
    sel_odd[ii] = 1.0;
  }
  task.push_back(create_sel_jp_task("odd", sel_odd));
  task.push_back(create_sel_jp_task("full", sel_full));
  for (size_t ii(0); ii < task.size(); ++ii) {
    gb.appendTask(task[ii]);
  }
}


TEST (controller, odd_even)
{
  shared_ptr<Task> jpos;
  Vector gamma_jpos;
  
  vector<shared_ptr<ClassicTaskPostureController> > ctrl;
  vector<shared_ptr<GenericSkill> > gb;
  vector<Vector> gamma;

  try {
    Model * puma(get_puma());
    Matrix aa;
    Vector gg;
    ASSERT_TRUE (puma->getMassInertia(aa)) << "failed to get mass inertia";
    ASSERT_TRUE (puma->getGravity(gg)) << "failed to get gravity";
    
    jpos = create_sel_jp_task("all", Vector::Ones(puma->getNDOF()));
    Status st;
    st = jpos->init(*puma);
    EXPECT_TRUE (st.ok) << "failed to init jpos task: " << st.errstr;
    st = jpos->update(*puma);
    EXPECT_TRUE (st.ok) << "failed to update jpos task: " << st.errstr;
    gamma_jpos = aa * jpos->getCommand() + gg;
    
    ctrl.push_back(shared_ptr<ClassicTaskPostureController>(new ClassicTaskPostureController("blah")));
    gb.push_back(shared_ptr<GenericSkill>(new GenericSkill("blah")));
    gamma.push_back(Vector::Zero(puma->getNDOF()));
    
    for (size_t ii(0); ii < ctrl.size(); ++ii) {
      
      append_odd_even_tasks(*gb[ii], puma->getNDOF());
      st = gb[ii]->init(*puma);
      EXPECT_TRUE (st.ok) << "failed to init generic skill #"
			  << ii << ": " << st.errstr;
      st = ctrl[ii]->init(*puma);
      EXPECT_TRUE (st.ok) << "failed to init controller #"
			  << ii << " `" << ctrl[ii]->getName() << "': " << st.errstr;
      st = ctrl[ii]->computeCommand(*puma, *gb[ii], gamma[ii]);
      EXPECT_TRUE (st.ok) << "failed to compute torques #"
			  << ii << " `" << ctrl[ii]->getName() << "': " << st.errstr;
    }
    
    cout << "==================================================\n"
	 << "whole-body torque comparison:\n";
    pretty_print(gamma_jpos, cout, "  reference jpos task", "    ");
    for (size_t ii(0); ii < ctrl.size(); ++ii) {
      pretty_print(gamma[ii], cout, "  controller `" + ctrl[ii]->getName() + "'", "    ");
      Vector const delta(gamma_jpos - gamma[ii]);
      pretty_print(delta, cout, "  delta", "    ");
    }
    
  }
  catch (exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
}


TEST (controller, odd_full)
{
  shared_ptr<Task> jpos;
  Vector gamma_jpos;
  
  vector<shared_ptr<ClassicTaskPostureController> > ctrl;
  vector<shared_ptr<GenericSkill> > gb;
  vector<Vector> gamma;

  try {
    Model * puma(get_puma());
    Matrix aa;
    Vector gg;
    ASSERT_TRUE (puma->getMassInertia(aa)) << "failed to get mass inertia";
    ASSERT_TRUE (puma->getGravity(gg)) << "failed to get gravity";
    
    jpos = create_sel_jp_task("all", Vector::Ones(puma->getNDOF()));
    Status st;
    st = jpos->init(*puma);
    EXPECT_TRUE (st.ok) << "failed to init jpos task: " << st.errstr;
    st = jpos->update(*puma);
    EXPECT_TRUE (st.ok) << "failed to update jpos task: " << st.errstr;
    gamma_jpos = aa * jpos->getCommand() + gg;
    
    ctrl.push_back(shared_ptr<ClassicTaskPostureController>(new ClassicTaskPostureController("blah")));
    gb.push_back(shared_ptr<GenericSkill>(new GenericSkill("blah")));
    gamma.push_back(Vector::Zero(puma->getNDOF()));
    
    for (size_t ii(0); ii < ctrl.size(); ++ii) {
      
      append_odd_full_tasks(*gb[ii], puma->getNDOF());
      st = gb[ii]->init(*puma);
      EXPECT_TRUE (st.ok) << "failed to init generic skill #"
			  << ii << ": " << st.errstr;
      st = ctrl[ii]->init(*puma);
      EXPECT_TRUE (st.ok) << "failed to init controller #"
			  << ii << " `" << ctrl[ii]->getName() << "': " << st.errstr;
      st = ctrl[ii]->computeCommand(*puma, *gb[ii], gamma[ii]);
      EXPECT_TRUE (st.ok) << "failed to compute torques #"
			  << ii << " `" << ctrl[ii]->getName() << "': " << st.errstr;
    }
    
    cout << "==================================================\n"
	 << "whole-body torque comparison:\n";
    pretty_print(gamma_jpos, cout, "  reference jpos task", "    ");
    for (size_t ii(0); ii < ctrl.size(); ++ii) {
      pretty_print(gamma[ii], cout, "  controller `" + ctrl[ii]->getName() + "'", "    ");
      Vector const delta(gamma_jpos - gamma[ii]);
      pretty_print(delta, cout, "  delta", "    ");
    }
    
  }
  catch (exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
}




TEST (task, jlimit)
{
  shared_ptr<JointLimitTask> jlimit(new JointLimitTask("jlimit"));
  
  try {
    Model * puma(get_puma());
    size_t const ndof(puma->getNDOF());
    
    Parameter * param(jlimit->lookupParameter("dt_seconds", PARAMETER_TYPE_REAL));
    ASSERT_NE ((void*)0, param) << "failed to get dt_seconds param";
    Status st(param->set(0.1));
    ASSERT_TRUE (st.ok) << "failed to set dt_seconds: " << st.errstr;

    param = jlimit->lookupParameter("upper_stop_deg", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get upper_stop_deg param";
    Vector foo(30.0 * Vector::Ones(ndof));
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set upper_stop_deg: " << st.errstr;

    param = jlimit->lookupParameter("upper_trigger_deg", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get upper_trigger_deg param";
    foo = 20.0 * Vector::Ones(ndof);
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set upper_trigger_deg: " << st.errstr;

    param = jlimit->lookupParameter("lower_stop_deg", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get lower_stop_deg param";
    foo = -30.0 * Vector::Ones(ndof);
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set lower_stop_deg: " << st.errstr;

    param = jlimit->lookupParameter("lower_trigger_deg", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get lower_trigger_deg param";
    foo = -20.0 * Vector::Ones(ndof);
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set lower_trigger_deg: " << st.errstr;

    param = jlimit->lookupParameter("kp", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get kp param";
    foo = 100.0 * Vector::Ones(ndof);
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set kp: " << st.errstr;

    param = jlimit->lookupParameter("kd", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get kd param";
    foo = 20.0 * Vector::Ones(ndof);
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set kd: " << st.errstr;

    param = jlimit->lookupParameter("maxvel", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get maxvel param";
    foo = 10.0 * M_PI / 180.0 * Vector::Ones(ndof);
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set maxvel: " << st.errstr;

    param = jlimit->lookupParameter("maxacc", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get maxacc param";
    foo = 25.0 * M_PI / 180.0 * Vector::Ones(ndof);
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set maxacc: " << st.errstr;
    
    ClassicTaskPostureController ctrl("ctrl");
    GenericSkill gb("gb");
    gb.appendTask(jlimit);
    
    State state(ndof, ndof, 0);
    state.position_ = Vector::Zero(ndof);
    state.velocity_ = Vector::Zero(ndof);
    puma->update(state);
    st = gb.init(*puma);
    EXPECT_TRUE (st.ok) << "failed to init generic skill: " << st.errstr;
    st = ctrl.init(*puma);
    EXPECT_TRUE (st.ok) << "failed to init controller: " << st.errstr;
    Vector gamma;
    st = ctrl.computeCommand(*puma, gb, gamma);
    EXPECT_FALSE (st.ok) << "computeCommand should have failed due to empty Jacobians";
    
    for (size_t ii(0); ii < ndof; ++ii) {
      if (0 == ii % 2) {
	state.position_[ii] =  4.0 * M_PI;
      }
      else {
	state.position_[ii] = -4.0 * M_PI;
      }
      puma->update(state);
      st = gb.update(*puma);
      EXPECT_TRUE (st.ok) << "failed to update generic skill: " << st.errstr;
      st = ctrl.computeCommand(*puma, gb, gamma);
      EXPECT_TRUE (st.ok) << "failed to computeCommand: " << st.errstr;
    }
    
  }
  catch (exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS ();
}
