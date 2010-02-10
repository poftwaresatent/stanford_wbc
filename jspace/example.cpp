#include "../plugins/fake/RobotFake.hpp"
#include <wbcnet/log.hpp>


static wbc::RobotAPI * robot_api(0);
static wbc::RobotControlModel * robot_control_model(0); // currently we have to wrap one of these
static jspace::Model * jspace_model(0);


static void handle(int signum)
{
  errx(SIGTERM == signum ? EXIT_SUCCESS : EXIT_FAILURE,
       "signal %d (%s)",
       signum, sys_siglist[signum]);
}


static void cleanup()
{
  delete jspace_model;
  delete robot_control_model;
  if (robot_api) {
    robot_api->shutdown();
    delete robot_api;
  }
}


int main(int argc, char*argv[])
{
  if ((0 != cleanup) && (0 != atexit(cleanup)))
    err(EXIT_FAILURE, "atexit()");
  if (SIG_ERR == signal(SIGTERM, handle))
    err(EXIT_FAILURE, "signal(SIGTERM, ...)");
  if (SIG_ERR == signal(SIGPIPE, handle))
    err(EXIT_FAILURE, "signal(SIGPIPE, ...)");
  
  wbcnet::configure_logging();
  wbcnet::manual_logging_verbosity(1);
  
  robot_api = new RobotFake(0);
  try {
    wbc::TiXmlBRParser parser;
    std::string filename("puma.xml");
    if (argc > 1) {
      filename = argv[1];
    }
    robot_control_model = new RobotControlModel(parser.parse(filename));
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION %s", ee.what());
  }
  
  jspace_model = new jspace::Model(robot_control_model);
  
  // size_t nrows, ncols;
  // attr->robmodel->getForceDimension(nrows, ncols);
  jspace::State jspace_state(jspace_model->getNJoints(), jspace_model->getNJoints(), 0, 0);// nrows, ncols);
  SAIVector command_torques(jspace_model->getNJoints());
  
  while (true) {
    
    if ( ! robot_api->readSensors(jspace_state.joint_angles,
				  jspace_state.joint_velocities,
				  jspace_state.acquisition_time,
				  &jspace_state..contact_forces)) {
      errx(EXIT_FAILURE, "robot_api->readSensors() failed");
    }
    
    jspace_model.setState(jspace_state);
    
    // Compute the torques using jspace_state and those parts of
    // jspace_model that you are interested in.
    for (int ii(0); ii < command_torques.size(); ++ii) {
      command_torques[ii] = (ii + jspace_state.acquisition_time.tv_sec) % command_torques.size();
    }
    
    if ( ! robot_api->writeCommand(command_torques)) {
      errx(EXIT_FAILURE, "robot_api->writeCommand() failed");
    }
  }
}
