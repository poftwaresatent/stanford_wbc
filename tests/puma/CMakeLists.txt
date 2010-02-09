### included via subdir() command in tests/CMakeLists.txt in case the testJspace needs to be built

add_library (pumaDynamics STATIC
  XPrintf.cpp
  PrMatrix.cpp
  PrMatrix3.cpp
  PrMatrix6.cpp
  PrVector.cpp
  PrVector3.cpp
  PrVector6.cpp
  pumaDynamics.cpp)
