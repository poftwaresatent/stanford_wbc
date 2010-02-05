/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2009 Stanford University. All rights reserved.
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
   \author Roland Philippsen
*/

#include "readlog.hpp"
#include "writelog.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <err.h>

#ifdef HAVE_LOG4CXX
# include <log4cxx/basicconfigurator.h>
#endif // HAVE_LOG4CXX

using namespace wbc_robotlog_plugin;


namespace {
  
  class FooBot : public wbc::RobotAPI {
  public:
    int const m_ndof;
    timeval m_t;
    explicit FooBot(int ndof): m_ndof(ndof) { m_t.tv_sec = 0; m_t.tv_usec = 0; }
    
    bool readSensors(SAIVector & jointAngles, SAIVector & jointVelocities,
		     timeval & acquisition_time, SAIMatrix * opt_force) {
      jointAngles.setSize(m_ndof);
      jointVelocities.setSize(m_ndof);
      if (opt_force)
	opt_force->setSize(6, m_ndof, true);
      for (int ii(0); ii < m_ndof; ++ii ) {
	jointAngles[ii] = 0.1 * ii;
	jointVelocities[ii] = 0.01 * ii;
	if (opt_force)
	  for (int jj(0); jj < 6; ++jj)
	    opt_force->elementAt(jj, ii) = ii + 0.1 * jj;
      }
      acquisition_time = m_t;
      ++m_t.tv_sec;
      ++m_t.tv_usec;
      return true;
    }
    
    bool writeCommand(SAIVector const & command) { return true; }
    void shutdown() const {}
  };
  
}


TEST (readlog, nonexisting_file)
{
  try {
    RLog rlog("some_nonexisting_file_HEBGJHQWEBF");
    SAIVector foo;
    timeval acquisition_time;
    SAIMatrix bar;
    EXPECT_FALSE ( rlog.readSensors(foo, foo, acquisition_time, &bar) )
      << "rlog.readSensors() should have failed on non-existing file";
    EXPECT_TRUE ( rlog.writeCommand(foo) )
      << "rlog.writeCommand() should always succeed";
  }
  catch (std::exception const & ee) {
    EXPECT_TRUE (false) << "unexpected exception: " << ee.what();
  }
}


TEST (readlog, non_xml_files)
{
  try {
    static std::string const fname("readlog.non_xml_files.xml");
    ofstream os(fname.c_str());
    os << "this is not an xml file\n";
    os.close();
    RLog rlog(fname);
    SAIVector foo;
    timeval acquisition_time;
    SAIMatrix bar;
    EXPECT_FALSE ( rlog.readSensors(foo, foo, acquisition_time, &bar) )
      << "rlog.readSensors() should have failed on non-xml file";
  }
  catch (std::exception const & ee) {
    EXPECT_TRUE (false) << "unexpected exception: " << ee.what();
  }
}


TEST (readlog, valid_xml_files)
{
  static char const * contents =
    "<?xml version=\"1.0\" ?>\n"
    "<robotlog>\n"
    "  <state>\n"
    "    <pos size=\"3\">  0.1  0.2  0.3 </pos>\n"
    "    <vel size=\"3\"> -1.1 -1.2 -1.3 </vel>\n"
    "    <force rows=\"6\" columns=\"2\"> 1  2\n"
    "                                     3  4\n"
    "                                     5  6\n"
    "                                     7  8\n"
    "                                     9 10\n"
    "                                    11 12</force>\n"
    "  </state>\n"
    "</robotlog>\n";
  
  try {
    static std::string const fname("readlog.valid_xml_files.xml");
    ofstream os(fname.c_str(), ios::trunc);
    os << contents;
    os.close();
    RLog rlog(fname);
    SAIVector pos, vel;
    timeval acquisition_time;
    SAIMatrix force;
    EXPECT_TRUE ( rlog.readSensors(pos, vel, acquisition_time, &force) )
      << "rlog.readSensors() should have succeeded on valid xml file";
    pos.display("pos");
    vel.display("vel");
    force.display("force");
  }
  catch (std::exception const & ee) {
    EXPECT_TRUE (false) << "unexpected exception: " << ee.what();
  }
}


TEST (writelog, keep_open)
{
  static std::string const fname("writelog.keep_open.xml");
  static int const ndof(3);
  WLog wlog(fname, new FooBot(ndof), true);
  SAIVector jointAngles(ndof);
  SAIVector jointVelocities(ndof);
  timeval acquisition_time;
  SAIMatrix force(6, ndof);
  for (size_t ii(0); ii < 4; ++ii) {
    EXPECT_TRUE (wlog.readSensors(jointAngles, jointVelocities, acquisition_time, &force))
      << "wlog.readSensors() failed in iteration " << ii;
    SAIVector command(ndof);
    for (int jj(0); jj < ndof; ++jj)
      command[jj] = 0.172 + 0.679 * jj;
    EXPECT_TRUE (wlog.writeCommand(command))
      << "wlog.writeCommand() failed in iteration " << ii;
  }
}


TEST (writelog, no_keep_open)
{
  static std::string const fname("writelog.no_keep_open.xml");
  static int const ndof(3);
  WLog wlog(fname, new FooBot(ndof), false);
  SAIVector jointAngles(ndof);
  SAIVector jointVelocities(ndof);
  timeval acquisition_time;
  SAIMatrix force(6, ndof);
  for (size_t ii(0); ii < 4; ++ii) {
    EXPECT_TRUE (wlog.readSensors(jointAngles, jointVelocities, acquisition_time, &force))
      << "wlog.readSensors() failed in iteration " << ii;
    SAIVector command(ndof);
    for (int jj(0); jj < ndof; ++jj)
      command[jj] = 0.172 + 0.679 * jj;
    EXPECT_TRUE (wlog.writeCommand(command))
      << "wlog.writeCommand() failed in iteration " << ii;
  }
}


TEST (readwritelog, no_keep_open)
{
  static std::string const fname("readwritelog.no_keep_open.xml");
  static int const ndof(3);
  {
    WLog wlog(fname, new FooBot(ndof), false);
    SAIVector jointAngles(ndof);
    SAIVector jointVelocities(ndof);
    timeval acquisition_time;
    SAIMatrix force(6, ndof);
    for (size_t ii(0); ii < 4; ++ii) {
      EXPECT_TRUE (wlog.readSensors(jointAngles, jointVelocities, acquisition_time, &force))
	<< "wlog.readSensors() failed in iteration " << ii;
      SAIVector command(ndof);
      for (int jj(0); jj < ndof; ++jj)
	command[jj] = 0.172 + 0.679 * jj;
      EXPECT_TRUE (wlog.writeCommand(command))
	<< "wlog.writeCommand() failed in iteration " << ii;
    }
  }
  try {
    RLog rlog(fname);
    SAIVector pos, vel, check_pos, check_vel;
    timeval acqt;// maybe later: check_acqt;
    SAIMatrix force, check_force;
    FooBot check(ndof);
    EXPECT_TRUE ( rlog.readSensors(pos, vel, acqt, &force) )
      << "rlog.readSensors() should have succeeded on valid xml file";
    EXPECT_TRUE ( check.readSensors(check_pos, check_vel, acqt, &check_force) )
      << "check.readSensors() failed";
    if ( ! (pos.equal(check_pos, 1e-6))) {
      pos.display("pos");
      check_pos.display("check_pos");
      EXPECT_TRUE (false) << "pos mismatch (see output above)";
    }
    if ( ! (vel.equal(check_vel, 1e-6))) {
      vel.display("vel");
      check_vel.display("check_vel");
      EXPECT_TRUE (false) << "vel mismatch (see output above)";
    }
    if ( ! (force.equal(check_force, 1e-6))) {
      force.display("force");
      check_force.display("check_force");
      EXPECT_TRUE (false) << "force mismatch (see output above)";
    }
  }
  catch (std::exception const & ee) {
    EXPECT_TRUE (false) << "unexpected exception: " << ee.what();
  }
}


int main(int argc, char ** argv)
{
#ifdef HAVE_LOG4CXX
  log4cxx::BasicConfigurator::configure();
#endif // HAVE_LOG4CXX
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
