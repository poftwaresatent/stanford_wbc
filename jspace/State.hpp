#include <wbcnet/data.hpp>
#include <sys/time.h>

namespace jspace {
  
  typedef wbcnet::Vector<double> vector_t;
  typedef wbcnet::Matrix<double> matrix_t;
  
  class State
  {
  public:
    State();
    State(int npos, int nvel, int force_nrows, int force_ncols);
    ~State();
    
    void init(int npos, int nvel, int force_nrows, int force_ncols);
    State & operator = (State const & rhs);
    
    timeval acquisition_time;
    vector_t joint_angles;
    vector_t joint_velocities;
    matrix_t external_forces;
  };
  
}
