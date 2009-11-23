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

#ifndef WBC_PLUGIN_WRITELOG_HPP
#define WBC_PLUGIN_WRITELOG_HPP

#include <wbc/core/RobotAPI.hpp>
#include <wbc/core/RobotFactory.hpp>
#include <fstream>

namespace wbc {
  class Extensions;
}


class WLog
  : public wbc::RobotAPI
{
public:
  WLog(std::string const & outfname,
       /** transfers ownership -- ~WLog() deletes the robot */
       wbc::RobotAPI * robot,
       /** If true, the outfile will be kept open until shutdown() is
	   called. Otherwise, the outfile will be opened and closed
	   for each write operation. */
       bool keep_open);
  ~WLog();
  
  virtual bool readSensors(SAIVector & jointAngles, SAIVector & jointVelocities,
			   timeval & acquisition_time, SAIMatrix * opt_force);
  virtual bool writeCommand(SAIVector const & command);
  virtual void shutdown() const;

protected:
  void init();
  
  std::string const m_outfname;
  wbc::RobotAPI * m_robot;
  bool const m_keep_open;
  mutable std::ofstream m_outfile;
  mutable bool m_initialized;
};


struct WLogFactory
  : public wbc::RobotFactory
{
  explicit WLogFactory(wbc::Extensions & ext);
  
  virtual WLog * parse(std::string const & spec, wbc::ServoInspector * servo_inspector);
  virtual void dumpHelp(std::string const & prefix, std::ostream & os) const;
  
  wbc::Extensions & ext;
};

#endif // WBC_PLUGIN_WRITELOG_HPP
