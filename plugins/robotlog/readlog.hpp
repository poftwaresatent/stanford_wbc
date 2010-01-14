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
   \file plugins/robotlog/readlog.hpp
   \author Roland Philippsen
*/

#ifndef WBC_PLUGIN_READLOG_HPP
#define WBC_PLUGIN_READLOG_HPP

#include <wbc/core/RobotAPI.hpp>
#include <wbc/core/RobotFactory.hpp>
#include <wbc/util/StringBuffer.hpp>
#include <wbc/util/File.hpp>
#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <expat.h>
#include <list>

typedef wbc::StringBuffer<XML_Char> strbuf_t;

class RLog
  : public wbc::RobotAPI
{
public:
  explicit RLog(std::string const & infname);
  ~RLog();
  
  virtual bool readSensors(SAIVector & jointAngles, SAIVector & jointVelocities,
			   timeval & acquisition_time, SAIMatrix * opt_force);
  virtual bool writeCommand(SAIVector const & command);
  virtual void shutdown() const;
  
  void parseChunk() throw(std::runtime_error);
  void parseState() throw(std::runtime_error);
  
  
  XML_ParserStruct * m_parser;
  strbuf_t * m_buffer;
  std::string m_infname;
  wbc::File * m_infile;
  int m_bufsize;
  std::list<SAIVector> m_pos;
  std::list<SAIVector> m_vel;
  std::list<SAIMatrix> m_force;
  SAIVector m_prev_pos;
  SAIVector m_prev_vel;
  SAIMatrix m_prev_force;
  SAIVector m_tmp_pos;
  SAIVector m_tmp_vel;
  SAIMatrix m_tmp_force;
};


struct RLogFactory
  : public wbc::RobotFactory
{
  virtual RLog * parse(std::string const & spec, wbc::ServoInspector * servo_inspector);
  virtual void dumpHelp(std::string const & prefix, std::ostream & os) const;
};

#endif // WBC_PLUGIN_READLOG_HPP
