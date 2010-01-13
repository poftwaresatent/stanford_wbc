#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <sys/time.h>

namespace wbc {

  class JSpaceState
  {
  public:
    SAIVector joint_angles;
    SAIVector joint_velocities;
    timeval acquisition_time;
    SAIMatrix external_forces;
  };
  
}
