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
   \file plugins/robotlog/readlog.cpp
   \author Roland Philippsen
*/

#include "readlog.hpp"
#include <wbcnet/strutil.hpp>
#include <wbcnet/log.hpp>
#include <errno.h>
#include <sys/time.h>

static wbcnet::logger_t logger(wbcnet::get_logger("readlog"));


/* expat callbacks */

extern "C" {
  static void start_element_handler(void * userData, const XML_Char * name,
				    const XML_Char ** atts)
    throw(std::runtime_error);
  static void end_element_handler(void * userData, const XML_Char * name)
    throw(std::runtime_error);
  static void character_data_handler(void * userData, const XML_Char * s,
				     int len);
}


RLog::
RLog(std::string const & infname)
  : m_parser(0),
    m_buffer(0),
    m_infname(infname),
    m_infile(0),
    m_bufsize(512)
{
  try {
    m_infile = new wbc::File(m_infname.c_str(), "r");
    m_parser = XML_ParserCreate(NULL);
    XML_SetElementHandler(m_parser, start_element_handler, end_element_handler);
    XML_SetCharacterDataHandler(m_parser, character_data_handler);
    XML_SetUserData(m_parser, this);
  }
  catch (std::runtime_error const & ee) {
    LOG_FATAL (logger, "EXCEPTION in RLog constructor: " << ee.what());
    m_infile = 0;
  }
}


RLog::
~RLog()
{
  if (m_parser)
    XML_ParserFree(m_parser);
  delete m_infile;
}


bool RLog::
readSensors(SAIVector & jointAngles, SAIVector & jointVelocities,
	    timeval & acquisition_time, SAIMatrix * opt_force)
{
  if (m_pos.empty()) {
    try {
      parseState();
    }
    catch (std::runtime_error const & ee) {
      LOG_FATAL (logger, "EXCEPTION in RLog::readSensors(): " << ee.what());
      return false;
    }
  }
  
  if ((m_pos.empty() && (0 == m_prev_pos.size()))
      || (m_vel.empty() && (0 == m_prev_vel.size()))) {
    LOG_FATAL (logger, "RLog::readSensors(): incomplete state in " << m_infname);
    return false;
  }
  
  if ( ! m_pos.empty()) {
    m_prev_pos = m_pos.front();
    m_pos.pop_front();
  }
  if ( ! m_vel.empty()) {
    m_prev_vel = m_vel.front();
    m_vel.pop_front();
  }
  if ( ! m_force.empty()) {
    m_prev_force = m_force.front();
    m_force.pop_front();
  }
  
  if (jointAngles.size() != m_prev_pos.size())
    LOG_WARN (logger,
		  "RLog::readSensors(): requested position dimension " << jointAngles.size()
		  << " overwritten with " << m_prev_pos.size());
  jointAngles = m_prev_pos;
  
  if (jointVelocities.size() != m_prev_vel.size())
    LOG_WARN (logger,
		  "RLog::readSensors(): requested velocity dimension " << jointVelocities.size()
		  << " overwritten with " << m_prev_vel.size());
  jointVelocities = m_prev_vel;
  
  if (opt_force) {
    if ((opt_force->row() != m_prev_force.row()) || (opt_force->column() != m_prev_force.column()))
      LOG_WARN (logger,
		    "RLog::readSensors(): requested force dimension " << opt_force->row()
		    << "x" << opt_force->column()
		    << " overwritten with " << m_prev_force.row() << "x" << m_prev_force.column());
    *opt_force = m_prev_force;
  }
  
  if (0 != gettimeofday(&acquisition_time, 0)) {
    LOG_ERROR (logger, "RLog::readSensors(): gettimeofday() failed");
    return false;
  }
  
  return true;
}


bool RLog::
writeCommand(SAIVector const & command)
{
  return true;
}


void RLog::
shutdown() const
{
}


void RLog::
parseChunk() throw(std::runtime_error)
{
  if ( ! m_parser)
    return;
  
  void * buf(XML_GetBuffer(m_parser, m_bufsize));
  if (NULL == buf)
    throw runtime_error("RLog::parseChunk(): XML_GetBuffer() failed, bufsize = "
			+ sfl::to_string(m_bufsize));
  size_t const bytes_read(fread(buf, 1, m_bufsize, m_infile->stream));
  if (bytes_read == 0) {
    if (ferror(m_infile->stream)) {
      ostringstream os;
      os << "RLog::parseChunk(): " << m_infname << ": fread() failed";
      throw runtime_error(os.str());
    }
    XML_ParserFree(m_parser);
    m_parser = 0;
    return;
  }
  
  if ( ! XML_ParseBuffer(m_parser, bytes_read, bytes_read == 0)) {
    ostringstream os;
    os << "RLog::parseChunk(): " << m_infname
       << ": " << XML_GetCurrentLineNumber(m_parser)
       << ": parse error: " << XML_ErrorString(XML_GetErrorCode(m_parser));
    throw runtime_error(os.str());
  }
}


void RLog::
parseState() throw(std::runtime_error)
{
  // Use side-effect: parseChunk() will destroy and null the parser
  // when the file is finished. Also, if the file was invalid, we will
  // never have a parser instance. Thus, checking for m_parser here
  // allows us to skip invalid files and stop after processing a valid
  // one.
  while (m_parser) {
    parseChunk();
    // The XML callbacks will end up storing pos, vel, and force in
    // the corresponing lists. Each time we encounter a </state> tag,
    // something gets pushed onto those lists, which we can detect
    // here.
    if ( ! m_pos.empty())
      break;
  }
}


RLog * RLogFactory::
parse(std::string const & spec, wbc::ServoInspector * servo_inspector)
{
  string infname;
  string foo;
  sfl::splitstring(spec, ':', infname, foo);
  if (infname.empty())
    infname = "wlog.xml";
  RLog * rlog(new RLog(infname));
  return rlog;
}


void RLogFactory::
dumpHelp(std::string const & prefix, std::ostream & os) const
{
  os << prefix << "spec = filename\n"
     << prefix << "  default filename = `wlog.xml'\n";
}


static int extract_int(const XML_Char * att) throw(std::runtime_error)
{
  int result(0);
  basic_istringstream<XML_Char> is(att);
  is >> result;
  LOG_DEBUG (logger, "readlog extract_int(): att=" << att << " result=" << result);
  if ( ! is) {
    ostringstream os;
    os << "readlog extract_int(): cannot parse `" << att << "'";
    throw runtime_error(os.str());
  }
  return result;
}


void start_element_handler(void * userData, const XML_Char * name,
			   const XML_Char ** atts)
  throw(std::runtime_error)
{
  RLog * rlog(reinterpret_cast<RLog *>(userData));
  basic_string<XML_Char> tag(name);
  
  try {
    
    if ("state" == tag) {
      rlog->m_tmp_pos.setSize(0);
      rlog->m_tmp_vel.setSize(0);
      rlog->m_tmp_force.setSize(0, 0);
    }
    
    else if ("pos" == tag) {
      int size(0);
      while (*atts) {
	if ("size" == basic_string<XML_Char>(*atts)) {
	  size = extract_int(*(atts+1));
	  break;
	}
	atts += 2;
      }
      if (0 >= size)
	throw runtime_error("invalid size");
      LOG_DEBUG (logger, "readlog start_element_handler(): pos size " << size);
      rlog->m_tmp_pos.setSize(size, true);
      rlog->m_buffer = new strbuf_t();
    }
    
    else if ("vel" == tag) {
      int size(0);
      while (*atts) {
	if ("size" == basic_string<XML_Char>(*atts)) {
	  size = extract_int(*(atts+1));
	  break;
	}
	atts += 2;
      }
      if (0 >= size)
	throw runtime_error("invalid size");
      LOG_DEBUG (logger, "readlog start_element_handler(): vel size " << size);
      rlog->m_tmp_vel.setSize(size, true);
      rlog->m_buffer = new strbuf_t();
    }
    
    else if ("force" == tag) {
      int rows(0);
      int columns(0);
      while (*atts) {
	basic_string<XML_Char> att(*atts);
	if ("rows" == att) {
	  rows = extract_int(*(atts+1));
	}
	else if ("columns" == att) {
	  columns = extract_int(*(atts+1));
	}
	atts += 2;
      }
      if ((0 >= rows) || (0 >= columns))
	throw runtime_error("invalid dimension");
      LOG_DEBUG (logger,
		     "readlog start_element_handler(): vel size " << rows << "x" << columns);
      rlog->m_tmp_force.setSize(rows, columns, true);
      rlog->m_buffer = new strbuf_t();
    }
    
  }
  
  catch (runtime_error const & ee) {
    ostringstream os;
    os << "readlog start_element_handler(): " << rlog->m_infname
       << ": " << XML_GetCurrentLineNumber(rlog->m_parser)
       << ": EXCEPTION in tag `" << tag << "': " << ee.what();
    throw runtime_error(os.str());
  }
}


void end_element_handler(void * userData, const XML_Char * name)
  throw(std::runtime_error)
{
  RLog * rlog(reinterpret_cast<RLog *>(userData));
  basic_string<XML_Char> tag(name);

  try {  
    
    if ("state" == tag) {
      rlog->m_pos.push_back(rlog->m_tmp_pos);
      rlog->m_vel.push_back(rlog->m_tmp_vel);
      rlog->m_force.push_back(rlog->m_tmp_force);
    }
    
    else if ("pos" == tag) {
      basic_istringstream<XML_Char> is(rlog->m_buffer->GetString());
      is >> rlog->m_tmp_pos;
      delete rlog->m_buffer;
      rlog->m_buffer = 0;
    }
    
    else if ("vel" == tag) {
      basic_istringstream<XML_Char> is(rlog->m_buffer->GetString());
      is >> rlog->m_tmp_vel;
      delete rlog->m_buffer;
      rlog->m_buffer = 0;
    }
    
    else if ("force" == tag) {
      basic_istringstream<XML_Char> is(rlog->m_buffer->GetString());
      is >> rlog->m_tmp_force;
      delete rlog->m_buffer;
      rlog->m_buffer = 0;
    }
    
  }
  
  catch (exception const & ee) {
    ostringstream os;
    os << "readlog end_element_handler(): " << rlog->m_infname
       << ": " << XML_GetCurrentLineNumber(rlog->m_parser)
       << ": EXCEPTION in tag `" << tag << "': " << ee.what();
    throw runtime_error(os.str());
  }
}


void character_data_handler(void * userData, const XML_Char * s,
			    int len)
{
  RLog * rlog(reinterpret_cast<RLog *>(userData));
  if (rlog->m_buffer)
    rlog->m_buffer->Append(s, len);
}
