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
   \file RobotFake.cpp
   \author Roland Philippsen
*/

#include "RobotFake.hpp"
#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <wbcnet/strutil.hpp>
#include <wbcnet/log.hpp>
#include <sstream>
#include <vector>

#ifdef WIN32
#include "wbcnet/win32_compat.hpp"
#else
#include <unistd.h>
#include <sys/time.h>
#endif
#include <time.h>

using sfl::splitstring;
using namespace std;

static wbcnet::logger_t logger(wbcnet::get_logger("wbc_fake_plugin"));


namespace wbc_fake_plugin {


RobotFake::
RobotFake(int extra_usleep, int drift)
  : m_extra_usleep(extra_usleep),
    m_drift(drift),
    m_tick(0)
{
}


bool RobotFake::
readSensors(SAIVector & jointPositions, SAIVector & jointVelocities, timeval & acquisition_time,
	    SAIMatrix * opt_force)
{
  if (0 != m_extra_usleep)
    usleep(m_extra_usleep);
  
  double offset;
  if (0 < m_drift) {
    offset = 0.1 * (m_tick % m_drift);
  }
  else {
    offset = 0.1;
  }
  ++m_tick;
  
  for ( int ii(0); ii < jointPositions.size(); ++ii )
    jointPositions[ii] = offset + 0.001 * ii; // an arbitrary value

  // notice that joint angles might have a different dimension than joint velocities
  // since they might include quaternions to specify orientation of spherical joints
  for ( int ii(0); ii < jointVelocities.size(); ++ii )
    jointVelocities[ii] = - offset - 0.001 * ii; // an arbitary value
  
  if (opt_force) {
    opt_force->setSize(6, jointPositions.size(), true);
    for (int icol(0); icol < opt_force->column(); ++icol)
      for (int irow(0); irow < opt_force->row(); ++irow)
	opt_force->elementAt(irow, icol) = offset + 5 * icol + 0.2 * irow;
  }
  
  if (0 != gettimeofday(&acquisition_time, NULL)) {
    cerr << "RobotFake::readSensors(): gettimeofday() failed\n";
    return false;
  }
  
  return true;
}


bool RobotFake::
writeCommand(SAIVector const & command)
{
  if (0 != m_extra_usleep)
    usleep(m_extra_usleep);
  return true;
}


bool RobotFake::
writeSensors(SAIVector const & jointAngles, SAIVector const & jointVelocities,
	     SAIMatrix const * opt_force)
{
  if (0 != m_extra_usleep)
    usleep(m_extra_usleep);
  return true;
}


bool RobotFake::
readCommand(SAIVector & command)
{
  if (0 != m_extra_usleep)
    usleep(m_extra_usleep);
  
  double offset;
  if (0 < m_drift) {
    offset = 1.0 * (m_tick % m_drift);
  }
  else {
    offset = 1.0;
  }
  ++m_tick;
  
  for (int ii(0); ii < command.size(); ++ii)
    command[ii] = offset - 0.01 * ii;
  
  return true;
}


void RobotFake::
shutdown() const
{
}


RobotFake * FactoryFake::
parse(std::string const & spec, wbc::ServoInspector * servo_inspector)
{
  vector<string> token;
  sfl::tokenize(spec, ':', token);
  
  int extra_usleep(250000);
  int drift(0);
  
  sfl::token_to(token, 0, extra_usleep);
  sfl::token_to(token, 1, drift);
  
  LOG_INFO (logger, "wbc_fake_plugin::FactoryFake::parse(" << spec << "): creating RobotFake(" <<
	    extra_usleep << ", " << drift << ")");
  
  return new RobotFake(extra_usleep, drift);
}
  
  
void FactoryFake::
dumpHelp(std::string const & prefix, std::ostream & os) const
{
  os << prefix << "spec = [ extra_usleep ]\n"
     << prefix << "  default = 250000\n";
}

}
