#error 'XXXX to do: resurrect this test (in ../bin or so)'

/*
 * Copyright (c) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <wbcrun/test/TestDirectory.hpp>
#include <wbcrun/ModelProcess.hpp>
#include <wbcrun/ServoProcess.hpp>
#include <wbcrun/UserProcess.hpp>
#include <wbcnet/NetConfig.hpp>
#include <wbcrun/TaskModelAPI.hpp>
#include <wbcrun/TaskModelListener.hpp>
#include <wbcrun/msg/RobotState.hpp>
#include <wbcnet/strutil.hpp>
#include <wbcnet/SPQueue.hpp>
#include <wbcnet/log.hpp>
#include <gtest/gtest.h>
#include <iostream>
#include <err.h>

using namespace wbcrun;
using namespace wbcnet;
using namespace std;

static int const n_task_sets(0);
static int const n_task_levels(2);

namespace wbcrun {
  
  typedef wbcnet::Matrix<double> test_matrix_t;
  
  class TestTaskModel
    : public TaskModel<test_matrix_t>
  {
  public:
    ////    typedef wbcnet::Matrix<double> matrix_t;
    test_matrix_t * massInertia;
    container_t::matrix_array_t * LambdaStar;
    
    TestTaskModel()
      : TaskModel<test_matrix_t>(wbcnet::ENDIAN_DETECT)
    {
      massInertia = AddIndepMx("massInertia");
      LambdaStar = AddTaskMx("LambdaStar", n_task_levels);
      //
      massInertia->SetSize(2, 7);
      for (int ii(0); ii < n_task_levels; ++ii)
	(*LambdaStar)[ii]->SetSize(3, 5);
    }
    
    inline container_t & GetContainer() { return m_container; }
  };
  
  
  class TestModelImplementation
    : public ModelImplementationAPI
  {
  public:
    TestModelImplementation()
      : ModelImplementationAPI()
    {
      ChangeSeed();
    }
    
    TestTaskModel model;
    TestTaskModel check_model;
    ssize_t seed;
    
    void ChangeSeed()
    {
      struct timeval stamp;
      if (0 != gettimeofday(&stamp, 0))
	err(EXIT_FAILURE, "TestModelImplementation::ChangeSeed(): gettimeofday()");
      if (seed == stamp.tv_usec)
	++seed;
      else
	seed = stamp.tv_usec;
    }
    
    virtual bool ComputeModel(msg::RobotState const & robot_state,
			      msg::TaskSpec const & task_spec) throw(std::exception)
    {
      // we have to do everything twice because serialization can swap
      // the byte order in the model that we send over the wire
      model.Reset(robot_state.requestID, 1, 0, 0, model.LambdaStar->size(), 1);
      check_model.Reset(robot_state.requestID, 1, 0, 0, model.LambdaStar->size(), 1);
      model.SetAcquisitionTime(robot_state.acquisitionTime);
      check_model.SetAcquisitionTime(robot_state.acquisitionTime);
      
      cout << "TestModelImplementation::ComputeModel():\n"
	   << "  reset model\n"
	   << "    requestID: " << (int) model.GetRequestID() << "\n"
	   << "    acquisitionTime: " << model.GetAcquisitionTime().tv_sec
	   << "s " << model.GetAcquisitionTime().tv_usec << "us\n";
      if (robot_state.requestID != model.GetRequestID())
	throw runtime_error("BUG in ModelProcess or TaskAtomizer or so: "
			    + sfl::to_string((int) robot_state.requestID)
			    + " == robot_state.requestID != model.GetRequestID() == "
			    + sfl::to_string((int) model.GetRequestID()));
      
      cout << "  massInertia\n";
      for (int irow(0); irow < model.massInertia->NRows(); ++irow) {
	cout << "   ";
	for (int icol(0); icol < model.massInertia->NColumns(); ++icol) {
	  model.massInertia->GetElement(irow, icol) = seed + 20 * irow + icol;
	  check_model.massInertia->GetElement(irow, icol)
	    = model.massInertia->GetElement(irow, icol);
	  cout << " " << model.massInertia->GetElement(irow, icol);
	}
	cout << "\n";
      }
      
      for (size_t ilevel(0); ilevel < model.LambdaStar->size(); ++ilevel) {
	cout << "  LambdaStar[" << ilevel << "]\n";
	for (int irow(0); irow < (*model.LambdaStar)[ilevel]->NRows(); ++irow) {
	  cout << "   ";
	  for (int icol(0); icol < (*model.LambdaStar)[ilevel]->NColumns(); ++icol) {
	    (*model.LambdaStar)[ilevel]->GetElement(irow, icol)
	      = seed + 20 * irow + icol - 5 * ilevel + 3;
	    (*check_model.LambdaStar)[ilevel]->GetElement(irow, icol)
	      = (*model.LambdaStar)[ilevel]->GetElement(irow, icol);
	    cout << " " << (*model.LambdaStar)[ilevel]->GetElement(irow, icol);
	  }
	  cout << "\n";
	}
      }
      return true;
    }
    
    virtual TaskModelAPI const * GetTaskModel() const throw(std::exception)
    { return &model; }
  };
  
  
  class TestServoImplementation
    : public MultirateServoImplementationAPI
  {
  public:
    std::vector<TestTaskModel*> model_pool;
    TaskModelListener * model_listener;
    
    TestServoImplementation()
      : MultirateServoImplementationAPI()
    {
      for (size_t ii(0); ii < 2; ++ii)
	model_pool.push_back(new TestTaskModel());
      model_listener = new TaskModelListener(model_pool[0], model_pool[1]);
      ResetModelPool();
    }
    
    virtual ~TestServoImplementation()
    {
      delete model_listener;
      for (size_t ii(0); ii < model_pool.size(); ++ii)
	delete model_pool[ii];
    }
    
    bool HandleServiceCall(ServiceMessage const & request,
			   ServiceMessage & reply)
    { return false; }

    virtual TaskModelListener * GetTaskModelListener() { return model_listener; }
    
    // //     virtual void UpdateTaskModel(TaskModelAPI * model, uint8_t behaviorID) {
    // //       if (m_verbose_os)
    // // 	*m_verbose_os << "TestServoImplementation::UpdateTaskModel(..., " << (int) behaviorID << ")\n"
    // // 		      << "  should do something smart here, or have the superclass do it\n";
    // //     }
    
    virtual bool UpdateRobotState(msg::RobotState & state) {
      cout << "TestServoImplementation::UpdateRobotState()\n"
	   << "  should do something smart here\n";
      return true;
    }
    
    virtual bool NullTorqueCommand() { return true; }
    
    virtual bool UpdateTorqueCommand(wbcrun::TaskModelAPI const * model, uint8_t behaviorID) {
      cout << "TestServoImplementation::UpdateTorqueCommand(" << (int) behaviorID << ")\n"
	   << "  should do something smart here\n";
      return true;
    }
    
    virtual bool ResetBehavior(TaskModelAPI * next_task_model,
			       uint8_t requestID,
			       uint8_t behaviorID) {
      TestTaskModel * mtm(dynamic_cast<TestTaskModel *>(next_task_model));
      if ( ! mtm) {
	cerr << "TestServoImplementation::ResetBehavior():\n"
	     << "  hey, dynamic_cast<TestTaskModel *>(next_task_model) failed!\n";
	return false;
      }
      static int const ngenmx(mtm->GetContainer().indep_mx.size());
      static int const nsetmx(mtm->GetContainer().set_mx.size());
      static int const nlevmx(mtm->GetContainer().task_mx.size());
      if (wbcnet::TaskAtomizer::OK != mtm->Reset(requestID, ngenmx, nsetmx, n_task_sets, n_task_levels, nlevmx)) {
	cerr << "TestServoImplementation::ResetBehavior():\n"
	     << "  hey, mtm->Reset() failed!\n";
	return false;
      }
      cout << "TestServoImplementation::ResetBehavior(" << (int) behaviorID << ")\n"
	   << "  should do something smart here\n";
      return true;
    }
    
    void ResetModelPool()
    {
      struct timeval stamp;
      stamp.tv_sec = 0;
      stamp.tv_usec = 0;
      for (size_t ipool(0); ipool < model_pool.size(); ++ipool) {
	model_pool[ipool]->SetAcquisitionTime(stamp);
	model_pool[ipool]->Reset(0, 1, 0, 0, model_pool[ipool]->LambdaStar->size(), 1);
	for (int irow(0); irow < model_pool[ipool]->massInertia->NRows(); ++irow)
	  for (int icol(0); icol < model_pool[ipool]->massInertia->NColumns(); ++icol)
	    model_pool[ipool]->massInertia->GetElement(irow, icol) = 0;
	for (size_t ilevel(0); ilevel < model_pool[ipool]->LambdaStar->size(); ++ilevel)
	  for (int irow(0); irow < (*model_pool[ipool]->LambdaStar)[ilevel]->NRows(); ++irow)
	    for (int icol(0); icol < (*model_pool[ipool]->LambdaStar)[ilevel]->NColumns(); ++icol)
	      (*model_pool[ipool]->LambdaStar)[ilevel]->GetElement(irow, icol) = 0;
      }
    }
  };
  
  
  class TestServoProcess
    : public ServoProcess
  {
  public:
    explicit TestServoProcess(TestServoImplementation & test_imp) : m_test_imp(test_imp) {}
    
    /** \todo It is error prone and not necessary anymore to require
	the robot state and task spec to have matching requestIDs */
    bool SendRobotState(uint8_t requestID) {
      if ( ! m_model_channel) {
	cout <<
	  "ERROR in wbcrun::TestServoProcess::SendRobotState():\n"
	  "  no channel, call Init() first\n";
	return false;
      }
      m_robot_state->requestID = requestID;
      try {
	EnqueueMessage(m_model_channel, m_robot_state, false, true);
	Send();
      }
      catch (exception const & ee) {
	cout <<
	  "EXCEPTION in wbcrun::TestServoProcess::SendRobotState():\n"
	  "  " << ee.what() << "\n";
	return false;
      }
      return true;
    }
    
    bool SendTaskSpec(uint8_t requestID, uint8_t behaviorID) {
      // bit of a hack because TaskModelListener will implicitly look for a matching requestID
      m_test_imp.model_pool[0]->Reset(requestID, 1, 0, 0, m_test_imp.model_pool[0]->LambdaStar->size(), 1);
      
      m_model_task_spec.requestID = requestID;
      m_model_task_spec.behaviorID = behaviorID;
      try {
	EnqueueMessage(m_model_channel, &m_model_task_spec, false, true);
	Send();
      }
      catch (exception const & ee) {
	cout <<
	  "EXCEPTION in wbcrun::TestServoProcess::SendTaskSpec():\n"
	  "  " << ee.what() << "\n";
	return false;
      }
      return true;
    }
    
    bool SendComputeModel() {
      if ( ! m_model_channel) {
	cout <<
	  "ERROR in wbcrun::TestServoProcess::SendComputeModel():\n"
	  "  no channel, call Init() first\n";
	return false;
      }
      m_servo_status.status = msg::COMPUTE_MODEL;
      try {
	EnqueueMessage(m_model_channel, &m_servo_status, false, true);
	Send();
      }
      catch (exception const & ee) {
	cout <<
	  "EXCEPTION in wbcrun::TestServoProcess::SendComputeModel():\n"
	  "  " << ee.what() << "\n";
	return false;
      }
      return true;
    }
    
    TestServoImplementation & m_test_imp;
  };
  
  
  class TestUserProcess : public UserProcess {
  public:
    bool SendBehavior(uint8_t requestID, uint8_t behaviorID) {
      m_task_spec.requestID = requestID;
      m_task_spec.behaviorID = behaviorID;
      EnqueueMessage(m_channel, &m_task_spec, true, false);
      SendWait(10000);
      return true;
    }
  };
  
  
  class ModelServoTest
    : public testing::Test
  {
  public:
    TestModelImplementation model_imp;
    TestServoImplementation servo_imp;
    ModelProcess model_process;
    TestServoProcess servo_process;
    TestUserProcess user_process;
    SPQueueNetConfig netconf;
    
    uint8_t const npos;
    uint8_t const nvel;
    uint8_t const force_nrows;
    uint8_t const force_ncols;
    
    
    ModelServoTest()
      : servo_process(servo_imp),
	npos(7),
	nvel(7),
	force_nrows(0),
	force_ncols(0)
    {
    }
    
    
    virtual void SetUp()
    {
      model_process.Init(&model_imp, false, netconf, npos, nvel, force_nrows, force_ncols);
      servo_process.Init(&servo_imp, false, netconf, npos, nvel, force_nrows, force_ncols);
      user_process.Init(netconf);
      FillRobotState(*model_process.m_robot_state);
      usleep(1000);
      FillRobotState(*servo_process.m_robot_state);
    }
    
    
    void FillRobotState(msg::RobotState & rs)
    {
      if ( ! rs.acquisitionTime.gettimeofday(0))
	err(EXIT_FAILURE, "ModelServoTest::FillRobotState(): gettimeofday()");
      cout << "ModelServoTest::FillRobotState():"
	   << "\n  jointAngles:    ";
      for (int ii(0); ii < rs.jointAngles.GetLength(); ++ii) {
	rs.jointAngles[ii] = rs.acquisitionTime.tv_usec + ii;
	cout << " " << rs.jointAngles[ii];
      }
      cout << "\n  jointVelocities:";
      for (int ii(0); ii < rs.jointVelocities.GetLength(); ++ii) {
	rs.jointVelocities[ii] = rs.acquisitionTime.tv_usec - ii;
	cout << " " << rs.jointVelocities[ii];
      }
      cout << "\n  forces:\n";
      for (int irow(0); irow < rs.forces.NRows(); ++irow) {
	cout << "   ";
	for (int icol(0); icol < rs.forces.NColumns(); ++icol) {
	  rs.forces.GetElement(irow, icol)
	    = rs.acquisitionTime.tv_usec + 20 * irow + icol;
	  cout << " " << rs.forces.GetElement(irow, icol);
	}
	cout << "\n";
      }
    }
    
  };
  
}


TEST_F(ModelServoTest, RobotStateWorks)
{
  ASSERT_FALSE( model_process.GetRobotState() == servo_process.GetRobotState() )
    << "robot_state should differ in model and servo after init";
}


static bool equal_models(TestTaskModel const & lhs, TestTaskModel const & rhs)
{
  if (lhs.GetRequestID() != rhs.GetRequestID()) {
    cout << "equal_models(): requestID mismatch\n"
	 << "  lhs: " << lhs.GetRequestID() << "\n"
	 << "  rhs: " << rhs.GetRequestID() << "\n";
    return false;
  }
  if (lhs.GetNGeneralMatrices() != rhs.GetNGeneralMatrices()) {
    cout << "equal_models(): NGeneralMatrices mismatch\n"
	 << "  lhs: " << lhs.GetNGeneralMatrices() << "\n"
	 << "  rhs: " << rhs.GetNGeneralMatrices() << "\n";
    return false;
  }
  if (lhs.GetNSets() != rhs.GetNSets()) {
    cout << "equal_models(): NSets mismatch\n"
	 << "  lhs: " << lhs.GetNSets() << "\n"
	 << "  rhs: " << rhs.GetNSets() << "\n";
    return false;
  }
  if (lhs.GetNSetMatrices() != rhs.GetNSetMatrices()) {
    cout << "equal_models(): NSetMatrices mismatch\n"
	 << "  lhs: " << lhs.GetNSetMatrices() << "\n"
	 << "  rhs: " << rhs.GetNSetMatrices() << "\n";
    return false;
  }
  if (lhs.GetTTNTasks() != rhs.GetTTNTasks()) {
    cout << "equal_models(): TTNTasks mismatch\n"
	 << "  lhs: " << lhs.GetTTNTasks() << "\n"
	 << "  rhs: " << rhs.GetTTNTasks() << "\n";
    return false;
  }
  if (lhs.GetNTaskMatrices() != rhs.GetNTaskMatrices()) {
    cout << "equal_models(): NTaskMatrices mismatch\n"
	 << "  lhs: " << lhs.GetNTaskMatrices() << "\n"
	 << "  rhs: " << rhs.GetNTaskMatrices() << "\n";
    return false;
  }
  if ((lhs.GetAcquisitionTime().tv_sec != rhs.GetAcquisitionTime().tv_sec)
      || (lhs.GetAcquisitionTime().tv_usec != rhs.GetAcquisitionTime().tv_usec)) {
    cout << "equal_models(): acquisitionTime mismatch\n"
	 << "  lhs: " << lhs.GetAcquisitionTime().tv_sec << "s "
	 << lhs.GetAcquisitionTime().tv_usec << "us\n"
	 << "  rhs: " << rhs.GetAcquisitionTime().tv_sec << "s "
	 << rhs.GetAcquisitionTime().tv_usec << "us\n";
    return false;
  }
  if (*(lhs.massInertia) != *(rhs.massInertia)) {
    cout << "equal_models(): massInertia mismatch\n"
	 << "  lhs:\n";
    lhs.massInertia->Display(cout, "    ");
    cout << "  rhs:\n";
    rhs.massInertia->Display(cout, "    ");
    return false;
  }
  for (int ii(0); ii < lhs.GetTTNTasks(); ++ii)
    if (*(*lhs.LambdaStar)[ii] != *(*rhs.LambdaStar)[ii]) {
      cout << "equal_models(): LambdaStar[" << ii << "] mismatch\n"
	   << "  lhs:\n";
      (*lhs.LambdaStar)[ii]->Display(cout, "    ");
      cout << "  rhs:\n";
      (*rhs.LambdaStar)[ii]->Display(cout, "    ");
      return false;
    }
  cout << "equal_models(): no difference detected\n";
  return true;
}

TEST_F(ModelServoTest, ModelUpdateWorks)
{
  EXPECT_TRUE( equal_models(*servo_imp.model_pool[0], *servo_imp.model_pool[1]) )
    << "both models in the servo's pool should be zeroed out after init";
  
  // we need at least three messages to tell the model what's up
  ASSERT_TRUE( servo_process.SendRobotState(42) );
  ASSERT_TRUE( servo_process.SendTaskSpec(42, 17) );
  ASSERT_TRUE( servo_process.SendComputeModel() );
  
  // each model step handles exactly one message to avoid missing
  // things in the finite state machine
  wbcnet::SPQueue * s2m(netconf.GetSPQueue(NetConfig::SERVO, NetConfig::MODEL));
  ASSERT_NE( s2m, (wbcnet::SPQueue *) 0 ) << "hey, no SPQueue from servo to model???";
  while ( ! s2m->Empty() )
    ASSERT_TRUE( model_process.Step() );
  
  wbcnet::SPQueue * m2s(netconf.GetSPQueue(NetConfig::MODEL, NetConfig::SERVO));
  ASSERT_NE( m2s, (wbcnet::SPQueue *) 0 ) << "hey, no SPQueue from model to servo???";
  while ( ! m2s->Empty() )
    ASSERT_TRUE( servo_process.Step() );
  
  ASSERT_TRUE( equal_models(*servo_imp.model_pool[0], model_imp.check_model) )
    << "servo's model[0] should match the model's model after synch";  
}

TEST_F(ModelServoTest, BehaviorTransitionWorks)
{
  try {
    uint8_t const current_behaviorID(servo_process.GetCurrentBehaviorID());
    static uint8_t const next_behaviorID(42);
    uint8_t const old_transition_requestID(servo_process.GetBehaviorTransitionRequestID());
    
    ASSERT_TRUE( user_process.SendBehavior(17 /* remember: user/servo
						 reqIDs are indep from
						 servo/mode reqIDs! */,
					   next_behaviorID) );
    
    // update servo until it does not add any more messages to the s2m
    // queue (stop at 0 or 100 messages, that would indicate some sort
    // of problem)
    wbcnet::SPQueue * s2m(netconf.GetSPQueue(NetConfig::SERVO, NetConfig::MODEL));
    ASSERT_NE( s2m, (wbcnet::SPQueue *) 0 ) << "hey, no SPQueue from servo to model???";
    // first step the servo receives the transition request, but does
    // not enqueue any messages for it yet (just sets a flag that has
    // an effect on the second step)
    ASSERT_TRUE( servo_process.Step() );
    size_t prev_nmsg(s2m->GetSize());
    while (true) {
      ASSERT_TRUE( servo_process.Step() );
      size_t const nmsg(s2m->GetSize());
      ASSERT_NE( nmsg, (size_t) 0 ) << "empty queue after servo_process.Step()";
      ASSERT_LT( nmsg, (size_t) 100 ) << "runaway servo_process.Step()?";
      if (nmsg == prev_nmsg)
	break;
      prev_nmsg = nmsg;
    }
  
    // servo has received the transition request and initiated the
    // necessary servo/model communication pattern
    uint8_t const new_transition_requestID(servo_process.GetBehaviorTransitionRequestID());
    ASSERT_NE( (int) new_transition_requestID, (int) old_transition_requestID );
    
    // servo is still executing the old behavior but knows it is transitioning
    ASSERT_EQ( (int) servo_process.GetCurrentBehaviorID(), (int) current_behaviorID );
    ASSERT_EQ( (int) servo_process.GetNextBehaviorID(), (int) next_behaviorID );
    ASSERT_NE( (int) servo_process.GetCurrentBehaviorID(), (int) next_behaviorID );
    
    // update model until it has handled all servo messages
    ASSERT_FALSE( s2m->Empty() ) << "s2m queue should not be empty after updating the servo";
    try {
      while ( ! s2m->Empty() )
	ASSERT_TRUE( model_process.Step() );
    }
    catch (exception const & ee) {
      ASSERT_TRUE( false ) << "EXCEPTION while updating model proccess\n  " << ee.what();
    }
    
    // servo is still executing the old behavior but knows it is transitioning
    ASSERT_EQ( (int) servo_process.GetCurrentBehaviorID(), (int) current_behaviorID );
    ASSERT_EQ( (int) servo_process.GetNextBehaviorID(), (int) next_behaviorID );
    ASSERT_NE( (int) servo_process.GetCurrentBehaviorID(), (int) next_behaviorID );
  
    // update servo until it has eaten up all info from model
    wbcnet::SPQueue * m2s(netconf.GetSPQueue(NetConfig::MODEL, NetConfig::SERVO));
    ASSERT_NE( m2s, (wbcnet::SPQueue *) 0 ) << "hey, no SPQueue from model to servo???";
    ASSERT_FALSE( m2s->Empty() ) << "m2s queue should not be empty after updating the model";
    try {
      while ( ! m2s->Empty() )
	ASSERT_TRUE( servo_process.Step() );
    }
    catch (exception const & ee) {
      ASSERT_TRUE( false ) << "EXCEPTION while updating servo proccess\n  " << ee.what();
    }
    
    // servo should now be running the new behavior
    ASSERT_NE( (int) servo_process.GetCurrentBehaviorID(), (int) current_behaviorID );
    ASSERT_EQ( (int) servo_process.GetCurrentBehaviorID(), (int) next_behaviorID );
    
  }
  catch (exception const & ee) {
    ASSERT_TRUE( false ) << "EXCEPTION in user.SendBehavior()\n  " << ee.what();
  }
}


int main(int argc, char ** argv)
{
  wbcnet::configure_logging();
  wbcnet::get_root_logger()->setLevel(log4cxx::Level::getDebug());
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
