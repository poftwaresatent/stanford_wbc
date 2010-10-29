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
   \file plugins/robotlog/writelog.cpp
   \author Roland Philippsen
*/

#include "writelog.hpp"
#include <wbcnet/strutil.hpp>
#include <wbc/util/utc.hpp>
#include <wbc/core/Plugin.hpp>
#include <saimatrix/SAIMatrix.h>
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("writelog"));


namespace wbc_robotlog_plugin {

WLog::
WLog(std::string const & outfname,
     wbc::RobotAPI * robot,
     bool keep_open)
  : m_outfname(outfname),
    m_robot(robot),
    m_keep_open(keep_open),
    m_initialized(false)
{
}


void WLog::
init()
{
  if (m_initialized)
    return;
  
  m_outfile.open(m_outfname.c_str(), std::ios::trunc);
  if ( ! m_outfile) {
    LOG_ERROR (logger, "WLog::init(): cannot open file `" << m_outfname << "' for writing");
    return;
  }
  
  m_outfile << "<?xml version=\"1.0\"?>\n";
  try {
    std::string utc(wbc::get_utc());
    m_outfile << "<robotlog utc=\"" << utc << "\">\n";
  }
  catch (...) {
    m_outfile << "<robotlog>\n";
  }
  
  if ( ! m_keep_open)
    m_outfile.close();
  
  m_initialized = true;
}


WLog::
~WLog()
{
  if (m_initialized)
    shutdown();
  delete m_robot;
}


bool WLog::
readSensors(SAIVector & jointAngles, SAIVector & jointVelocities,
	    timeval & acquisition_time, SAIMatrix * opt_force)
{
  if ( ! m_initialized)
    init();
  
  SAIMatrix tmp_force;
  if (0 == opt_force)
    opt_force = &tmp_force;
  if ( ! m_robot->readSensors(jointAngles, jointVelocities, acquisition_time, opt_force)) {
    LOG_ERROR (logger, "WLog::readSensors(): m_robot->readSensors() failed");
    return false;
  }
  
  if ( ! m_keep_open)
    m_outfile.open(m_outfname.c_str(), std::ios::app);
  if ( ! m_outfile) {
    LOG_ERROR (logger, "WLog::readSensors(): cannot open file `" << m_outfname << "' for writing");
    return false;
  }
  
  m_outfile << "  <state>\n    <timeval tv_sec=\"" << acquisition_time.tv_sec
	    << "\" tv_usec=\"" << acquisition_time.tv_usec
	    << "\"/>\n    <pos size=\"" << jointAngles.size() << "\">";
  for (int ii(0); ii < jointAngles.size(); ++ii)
    m_outfile << " " << jointAngles[ii];
  m_outfile << "</pos>\n    <vel size=\"" << jointVelocities.size() << "\">";
  for (int ii(0); ii < jointVelocities.size(); ++ii)
    m_outfile << " " << jointVelocities[ii];
  m_outfile << "</vel>\n    <force rows=\"" << opt_force->row() << "\" columns=\"" << opt_force->column() << "\">";
  for (int ir(0); ir < opt_force->row(); ++ir)
    for (int ic(0); ic < opt_force->column(); ++ic)
      m_outfile << " " << opt_force->elementAt(ir, ic);
  m_outfile << "</force>\n  </state>\n";
  
  if ( ! m_keep_open)
    m_outfile.close();
  
  return true;
}


bool WLog::
writeCommand(SAIVector const & command)
{
  if ( ! m_initialized)
    init();
  
  if ( ! m_robot->writeCommand(command)) {
    LOG_ERROR (logger, "WLog::writeCommand(): m_robot->writeCommand() failed");
    return false;
  }
  
  if ( ! m_keep_open)
    m_outfile.open(m_outfname.c_str(), std::ios::app);
  if ( ! m_outfile) {
    LOG_ERROR (logger, "WLog::writeCommand(): cannot open file `" << m_outfname << "' for writing");
    return false;
  }
  
  m_outfile << "  <command size=\"" << command.size() << "\">";
  for (int ii(0); ii < command.size(); ++ii)
    m_outfile << " " << command[ii];
  m_outfile << "</command>\n";
  
  if ( ! m_keep_open)
    m_outfile.close();
  
  return true;
}


void WLog::
shutdown() const
{
  m_robot->shutdown();
  
  if ( ! m_initialized)
    return;
  
  if ( ! m_keep_open)
    m_outfile.open(m_outfname.c_str(), std::ios::app);
  if ( ! m_outfile) {
    LOG_ERROR (logger, "WLog::shutdown(): cannot open file `" << m_outfname << "' for writing");
  }
  else {
    try {
      string utc(wbc::get_utc());
      m_outfile << "  <shutdown utc=\"" << utc << "\"/>\n";
    }
    catch (...) {
      // ignore
    }
    m_outfile << "</robotlog>\n";
    m_outfile.close();
  }
  
  m_initialized = false;
}


WLogFactory::
WLogFactory(wbc::Extensions & _ext)
  : ext(_ext)
{
}


WLog * WLogFactory::
parse(std::string const & spec, wbc::ServoInspector * servo_inspector)
{
  string outfname;
  string subspec;
  sfl::splitstring(spec, ':', outfname, subspec);
  if (subspec.empty()) {
    LOG_ERROR (logger, "WLogFactory::parse(): no subspec in `" << spec << "'");
    return 0;
  }

  wbc::RobotAPI * robot(ext.robot_registry->parseCreate(subspec, servo_inspector));
  if (0 == robot) {
    LOG_ERROR (logger, "WLogFactory::parse(): no robot for subspec `" << subspec << "'");
    return 0;
  }
  
  if (outfname.empty())
    outfname = "wlog.xml";
  static bool const keep_open(false); // maybe add an option for this...
  
  LOG_DEBUG (logger, "WLogFactory::parse(): logging `" << subspec << "' to `" << outfname << "'");
  WLog * wlog(new WLog(outfname, robot, keep_open));
  return wlog;
}


void WLogFactory::
dumpHelp(std::string const & prefix, std::ostream & os) const
{
  os << prefix << "spec = filename : subspec\n"
     << prefix << "  default filename = `wlog.xml'\n"
     << prefix << "  subspec is the spec of the robot which should be logged\n";
}

}
