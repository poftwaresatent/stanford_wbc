cmake_minimum_required (VERSION 2.6)
include (../../../wbc.cmake)
wbc_add_plugin (fake plugin.cpp RobotFake.cpp DebugBehavior.cpp RawController.cpp)
