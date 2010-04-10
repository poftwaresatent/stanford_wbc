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
    virtual jspace::Status setGoal(std::vector<double> const & goal);
    virtual jspace::Status setGains(std::vector<double> const & kp, std::vector<double> const & kd);

  private:
    std::string active_controller_;
    std::vector<double> goal_;
    std::vector<double> kp_;
    std::vector<double> kd_;
  };
  
  
  class SPQTransactionPolicy
    : public jspace::TransactionPolicy
  {
  public:
    explicit SPQTransactionPolicy(jspace::ServoProxyServer * server);
    
    virtual jspace::Status WaitReceive();
    virtual jspace::Status PreReceive();
    
  private:
    jspace::ServoProxyServer * server_;
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
  jspace::ServoInfo sinfo;
  sinfo.controller_name.push_back("blahblah");
  sinfo.dof_name.push_back("kasejbdf");
  sinfo.limit_lower.push_back(42);
  sinfo.limit_upper.push_back(17);
  sinfo.limit_upper.push_back(-17);
  jspace::Status const sst(servo->getInfo(sinfo));
  ASSERT_TRUE (sst.ok) << sst.errstr;
  
  jspace::ServoInfo cinfo;
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
  for (size_t ii(0); ii < sinfo.limit_lower.size(); ++ii) {
    ASSERT_EQ (sinfo.limit_lower[ii], cinfo.limit_lower[ii]);
  }
  
  ASSERT_EQ (sinfo.limit_upper.size(), cinfo.limit_upper.size());
  for (size_t ii(0); ii < sinfo.limit_upper.size(); ++ii) {
    ASSERT_EQ (sinfo.limit_upper[ii], cinfo.limit_upper[ii]);
  }
}


TEST_F (ServoProxyTest, getState)
{
  jspace::ServoState sstate;
  sstate.active_controller = "2k34u5";
  for (double foo(0.1); foo <= 0.3; foo += 0.1) {
    sstate.goal.push_back(foo);
    sstate.actual.push_back(-foo);
    sstate.kp.push_back(100*foo);
    sstate.kd.push_back(10*foo);
  }
  jspace::Status const sst(servo->getState(sstate));
  ASSERT_TRUE (sst.ok) << sst.errstr;
  
  jspace::ServoState cstate;
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
  
  jspace::ServoState sstate;
  sst = servo->getState(sstate);
  ASSERT_TRUE (sst.ok) << sst.errstr;
  
  ASSERT_EQ (sstate.active_controller, "foobar");
  
  cst = client->selectController("");
  ASSERT_FALSE (cst.ok) << "selectController() should have failed on empty name";
}


TEST_F (ServoProxyTest, setGoal)
{
  std::vector<double> sgoal;
  for (double foo(0.1); foo <= 0.3; foo += 0.1) {
    sgoal.push_back(foo);
  }
  jspace::Status sst(servo->setGoal(sgoal));
  ASSERT_TRUE (sst.ok) << sst.errstr;
  
  std::vector<double> cgoal;
  for (double foo(1.1); foo <= 1.3; foo += 0.05) {
    cgoal.push_back(foo);
  }
  jspace::Status cst(client->setGoal(cgoal));
  ASSERT_TRUE (cst.ok) << cst.errstr;
  
  jspace::ServoState sstate;
  sst = servo->getState(sstate);
  ASSERT_TRUE (sst.ok) << sst.errstr;
  
  ASSERT_EQ (sstate.goal.size(), cgoal.size());
  for (size_t ii(0); ii < cgoal.size(); ++ii) {
    ASSERT_EQ (sstate.goal[ii], cgoal[ii]);
  }
}


TEST_F (ServoProxyTest, setGains)
{
  std::vector<double> skp, skd;
  for (double foo(100); foo <= 300; foo += 100) {
    skp.push_back(foo);
    skd.push_back(foo/17);
  }
  jspace::Status sst(servo->setGains(skp, skd));
  ASSERT_TRUE (sst.ok) << sst.errstr;
  
  std::vector<double> ckp, ckd;
  for (double foo(200); foo <= 1000; foo *= 1.2) {
    ckp.push_back(foo);
    ckd.push_back(foo/21.3);
  }
  jspace::Status cst(client->setGains(ckp, ckd));
  ASSERT_TRUE (cst.ok) << cst.errstr;
  
  jspace::ServoState sstate;
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
    info.limit_lower.clear();
    info.limit_upper.clear();
    
    info.controller_name.push_back("controller one");
    info.controller_name.push_back("controller two");
    
    info.dof_name.push_back("dof one");
    info.dof_name.push_back("dof two");
    info.dof_name.push_back("dof three");
    
    for (double foo(-0.1); foo <= 0.1; foo += 0.1) {
      info.limit_lower.push_back(foo - 17);
      info.limit_upper.push_back(foo + 42);
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
  setGoal(std::vector<double> const & goal)
  {
    jspace::Status st(true, "dummy servo set goal");
    if (goal.empty()) {
      st.ok = false;
    }
    goal_ = goal;
    return st;
  }
  
  
  jspace::Status DummyServo::
  setGains(std::vector<double> const & kp, std::vector<double> const & kd)
  {
    jspace::Status st(true, "dummy servo set gains");
    if ((kp.empty()) || (kd.empty())) {
      st.ok = false;
    }
    kp_ = kp;
    kd_ = kd;
    return st;
  }
  
  
  SPQTransactionPolicy::
  SPQTransactionPolicy(jspace::ServoProxyServer * server)
    : server_(server)
  {
  }
  
  
  jspace::Status SPQTransactionPolicy::
  WaitReceive()
  {
    jspace::Status zonk(false, "SPQTransactionPolicy::WaitReceive() should never be called");
    return zonk;
  }
  
  
  jspace::Status SPQTransactionPolicy::
  PreReceive()
  {
    return server_->handle();
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
    client->init(s2c_, false, new SPQTransactionPolicy(server), true);
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
