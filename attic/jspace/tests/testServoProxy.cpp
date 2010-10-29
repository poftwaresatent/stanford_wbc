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
   \file testServoProxy.cpp
   \author Roland Philippsen
*/

#include <jspace/servo_proxy.hpp>
#include <jspace/proxy_util.hpp>
#include <wbcnet/imp/SPQueue.hpp>
#include <gtest/gtest.h>


namespace {
  
  class DummyServo
    : public jspace::ServoAPI
  {
  public:
    DummyServo();
    
    virtual jspace::Status getInfo(jspace::ServoInfo & info) const;
    virtual jspace::Status getState(jspace::ServoState & state) const;
    virtual jspace::Status selectController(std::string const & name);
    virtual jspace::Status setGoal(jspace::Vector const & goal);
    virtual jspace::Status setGains(jspace::Vector const & kp, jspace::Vector const & kd);

  private:
    std::string active_controller_;
    jspace::Vector goal_;
    jspace::Vector kp_;
    jspace::Vector kd_;
  };
  
  
  class ServoProxyTest
    : public testing::Test
  {
  public:
    virtual void SetUp();
    virtual void TearDown();
    
    DummyServo * servo;
    jspace::ServoProxyServer * server;
    jspace::ServoProxyClient * client;
    
  private:
    wbcnet::Channel * c2s_;
    wbcnet::Channel * s2c_;
  };
  
}


TEST_F (ServoProxyTest, getInfo)
{
  jspace::ServoInfo sinfo(3, 1);
  sinfo.controller_name[0] = "blahblah";
  sinfo.dof_name[0] = "kasejbdf";
  sinfo.limit_lower << 42, 17, -17;
  jspace::Status const sst(servo->getInfo(sinfo));
  ASSERT_TRUE (sst.ok) << sst.errstr;
  
  jspace::ServoInfo cinfo(0, 0);
  jspace::Status const cst(client->getInfo(cinfo));
  ASSERT_TRUE (cst.ok) << cst.errstr;
  
  ASSERT_EQ (sinfo.controller_name.size(), cinfo.controller_name.size());
  for (size_t ii(0); ii < sinfo.controller_name.size(); ++ii) {
    ASSERT_EQ (sinfo.controller_name[ii], cinfo.controller_name[ii]);
  }
  
  ASSERT_EQ (sinfo.dof_name.size(), cinfo.dof_name.size());
  for (size_t ii(0); ii < sinfo.dof_name.size(); ++ii) {
    ASSERT_EQ (sinfo.dof_name[ii], cinfo.dof_name[ii]);
  }
  
  ASSERT_EQ (sinfo.limit_lower.size(), cinfo.limit_lower.size());
  for (ssize_t ii(0); ii < sinfo.limit_lower.size(); ++ii) {
    ASSERT_EQ (sinfo.limit_lower[ii], cinfo.limit_lower[ii]);
  }
  
  ASSERT_EQ (sinfo.limit_upper.size(), cinfo.limit_upper.size());
  for (ssize_t ii(0); ii < sinfo.limit_upper.size(); ++ii) {
    ASSERT_EQ (sinfo.limit_upper[ii], cinfo.limit_upper[ii]);
  }
}


TEST_F (ServoProxyTest, getState)
{
  jspace::ServoState sstate(3);
  sstate.active_controller = "2k34u5";
  for (size_t ii(0); ii < 3; ++ii) {
    double const foo(0.1 * (ii+1));
    sstate.goal[ii] = foo;
    sstate.actual[ii] = -foo;
    sstate.kp[ii] = 100*foo;
    sstate.kd[ii] = 10*foo;
  }
  jspace::Status const sst(servo->getState(sstate));
  ASSERT_TRUE (sst.ok) << sst.errstr;
  
  jspace::ServoState cstate(0);
  jspace::Status const cst(client->getState(cstate));
  ASSERT_TRUE (cst.ok) << cst.errstr;
  
  ASSERT_EQ (sstate.active_controller, cstate.active_controller);

  ASSERT_EQ (sstate.goal.size(), cstate.goal.size());
  for (size_t ii(0); ii < sstate.goal.size(); ++ii) {
    ASSERT_EQ (sstate.goal[ii], cstate.goal[ii]);
  }
  
  ASSERT_EQ (sstate.actual.size(), cstate.actual.size());
  for (size_t ii(0); ii < sstate.actual.size(); ++ii) {
    ASSERT_EQ (sstate.actual[ii], cstate.actual[ii]);
  }

  ASSERT_EQ (sstate.kp.size(), cstate.kp.size());
  for (size_t ii(0); ii < sstate.kp.size(); ++ii) {
    ASSERT_EQ (sstate.kp[ii], cstate.kp[ii]);
  }

  ASSERT_EQ (sstate.kd.size(), cstate.kd.size());
  for (size_t ii(0); ii < sstate.kd.size(); ++ii) {
    ASSERT_EQ (sstate.kd[ii], cstate.kd[ii]);
  }
}


TEST_F (ServoProxyTest, selectController)
{
  jspace::Status sst(servo->selectController("blahblah"));
  ASSERT_TRUE (sst.ok) << sst.errstr;
  
  std::string cname;
  jspace::Status cst(client->selectController("foobar"));
  ASSERT_TRUE (cst.ok) << cst.errstr;
  
  jspace::ServoState sstate(0);
  sst = servo->getState(sstate);
  ASSERT_TRUE (sst.ok) << sst.errstr;
  
  ASSERT_EQ (sstate.active_controller, "foobar");
  
  cst = client->selectController("");
  ASSERT_FALSE (cst.ok) << "selectController() should have failed on empty name";
}


TEST_F (ServoProxyTest, setGoal)
{
  jspace::Vector sgoal(3);
  sgoal << 0.1, 0.2, 0.3;
  jspace::Status sst(servo->setGoal(sgoal));
  ASSERT_TRUE (sst.ok) << sst.errstr;
  
  jspace::Vector cgoal(5);
  cgoal << 1.1, 1.15, 1.2, 1.25, 1.3;
  jspace::Status cst(client->setGoal(cgoal));
  ASSERT_TRUE (cst.ok) << cst.errstr;
  
  jspace::ServoState sstate(0);
  sst = servo->getState(sstate);
  ASSERT_TRUE (sst.ok) << sst.errstr;
  
  ASSERT_EQ (sstate.goal.size(), cgoal.size());
  for (size_t ii(0); ii < cgoal.size(); ++ii) {
    ASSERT_EQ (sstate.goal[ii], cgoal[ii]);
  }
}


TEST_F (ServoProxyTest, setGains)
{
  jspace::Vector skp(3), skd(3);
  skp << 100.0, 200.0, 300.0;
  skd << 100.0/17, 200.0/17, 300.0/17;
  jspace::Status sst(servo->setGains(skp, skd));
  ASSERT_TRUE (sst.ok) << sst.errstr;
  
  jspace::Vector ckp(7), ckd(7);
  ckp << 200, 240, 288, 345.6, 414.72, 497.664, 597.1968;
  ckd << 200/21.3, 240/21.3, 288/21.3, 345.6/21.3, 414.72/21.3, 497.664/21.3, 597.1968/21.3;
  jspace::Status cst(client->setGains(ckp, ckd));
  ASSERT_TRUE (cst.ok) << cst.errstr;
  
  jspace::ServoState sstate(0);
  sst = servo->getState(sstate);
  ASSERT_TRUE (sst.ok) << sst.errstr;
  
  ASSERT_EQ (sstate.kp.size(), ckp.size());
  for (size_t ii(0); ii < ckp.size(); ++ii) {
    ASSERT_EQ (sstate.kp[ii], ckp[ii]);
  }
  
  ASSERT_EQ (sstate.kd.size(), ckd.size());
  for (size_t ii(0); ii < ckd.size(); ++ii) {
    ASSERT_EQ (sstate.kd[ii], ckd[ii]);
  }
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS ();
}


namespace {
  
  DummyServo::
  DummyServo()
    : active_controller_("hyperactive dummy")
  {
  }
  
  
  jspace::Status DummyServo::
  getInfo(jspace::ServoInfo & info) const
  {
    info.controller_name.clear();
    info.dof_name.clear();
    
    info.controller_name.push_back("controller one");
    info.controller_name.push_back("controller two");
    
    info.dof_name.push_back("dof one");
    info.dof_name.push_back("dof two");
    info.dof_name.push_back("dof three");
    
    info.limit_lower = jspace::Vector::Zero(3);
    info.limit_upper = jspace::Vector::Zero(3);
    for (int ii(0); ii <= 2; ++ii) {
      double const foo(0.1 * (ii-1));
      info.limit_lower[ii] = foo - 17;
      info.limit_upper[ii] = foo + 42;
    }
    
    jspace::Status st(true, "dummy servo info");
    return st;
  }
  
  
  jspace::Status DummyServo::
  getState(jspace::ServoState & state) const
  {
    state.active_controller = active_controller_;
    state.goal = goal_;
    state.actual = goal_;
    state.kp = kp_;
    state.kd = kd_;
    
    for (size_t ii(0); ii < goal_.size(); ++ii) {
      state.actual[ii] += 0.1;
    }
    
    jspace::Status st(true, "dummy servo state");
    return st;
  }
  
  
  jspace::Status DummyServo::
  selectController(std::string const & name)
  {
    jspace::Status st(true, "dummy servo select controller");
    if (name.empty()) {
      st.ok = false;
    }
    else {
      active_controller_ = name;
    }
    return st;
  }
  
  
  jspace::Status DummyServo::
  setGoal(jspace::Vector const & goal)
  {
    jspace::Status st(true, "dummy servo set goal");
    if (0 == goal.size()) {
      st.ok = false;
    }
    goal_ = goal;
    return st;
  }
  
  
  jspace::Status DummyServo::
  setGains(jspace::Vector const & kp, jspace::Vector const & kd)
  {
    jspace::Status st(true, "dummy servo set gains");
    if ((0 == kp.size()) || (0 == kd.size())) {
      st.ok = false;
    }
    kp_ = kp;
    kd_ = kd;
    return st;
  }
  
  
  void ServoProxyTest::
  SetUp()
  {
    wbcnet::SPQueue * foo(new wbcnet::SPQueue());
    wbcnet::SPQueue * bar(new wbcnet::SPQueue());
    c2s_ = new wbcnet::ProxyChannel(foo, true, bar, true);
    s2c_ = new wbcnet::ProxyChannel(bar, false, foo, false);
    servo = new DummyServo();
    server = new jspace::ServoProxyServer();
    server->init(servo, false, c2s_, false);
    client = new jspace::ServoProxyClient();
    client->init(s2c_, false, CreateSPTransactionPolicy(server), true);
  }
  
  
  void ServoProxyTest::
  TearDown()
  {
    delete client;
    delete server;
    delete servo;
    delete s2c_;
    delete c2s_;
  }
  
}
