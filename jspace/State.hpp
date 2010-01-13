#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <sys/time.h>

namespace jspace {

  class State
  {
  public:
    State(): joint_angles(0), joint_velocities(0), external_forces(0, 0) {
      acquisition_time.tv_sec = 0;
      acquisition_time.tv_usec = 0;
    }
    
    State(int npos, int nvel, int force_nrows, int force_ncols)
      : joint_angles(npos),
	joint_velocities(nvel),
	external_forces(force_nrows, force_ncols)
    {
      acquisition_time.tv_sec = 0;
      acquisition_time.tv_usec = 0;
    }
    
    SAIVector joint_angles;
    SAIVector joint_velocities;
    timeval acquisition_time;
    SAIMatrix external_forces;
    
    inline State & operator = (State const & rhs) {
      joint_angles = rhs.joint_angles;
      joint_velocities = rhs.joint_velocities;
      acquisition_time= rhs.acquisition_time;
      external_forces= rhs.external_forces;
      return *this;
    }
  };
  
}
