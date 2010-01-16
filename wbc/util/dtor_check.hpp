#include <string>

namespace wbc {
  
  struct dtor_check {
    dtor_check();
    
    void check(void * that);
    
    void * previous_that;
    std::string previous_bt;
  };
  
}
