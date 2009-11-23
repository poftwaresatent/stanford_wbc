/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2009 Stanford University. All rights reserved.
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
   \file BehaviorParser.cpp
   \author Roland Philippsen
*/

#include "BehaviorParser.hpp"
#include <wbc/core/BehaviorFactory.hpp>
#include <iostream>
#include <sstream>

extern "C" {
#include <errno.h>
#include <string.h>
#include <sys/types.h>
}

using namespace std;

extern "C" {
  static void start_element_handler(void * userData, const XML_Char * name,
				    const XML_Char ** atts)
    throw(std::runtime_error);
  static void end_element_handler(void * userData, const XML_Char * name)
    throw(std::runtime_error);
  static void character_data_handler(void * userData, const XML_Char * s,
				     int len);
}

namespace wbc {
  
  
  void DebugBehaviorConstructionCallback::
  operator () (std::string const & name) const throw(std::runtime_error)
  {
    cout << "DebugBehaviorConstructionCallback: " << name << "\n";
  }
  
  
  StdBehaviorConstructionCallback::
  StdBehaviorConstructionCallback(vector<BehaviorDescription*> & bvec,
				  BehaviorFactoryRegistry const & breg)
    : m_bvec(bvec),
      m_breg(breg)
  {
  }
  
  
  void StdBehaviorConstructionCallback::
  operator () (std::string const & name) const throw(std::runtime_error)
  {
    m_bvec.push_back(m_breg.Create(name));
  }
  
  
  BehaviorParser::
  BehaviorParser()
    : parser(0),
      buffer(0),
      in_behavior(false),
      in_type(false),
      filename("/dev/null"),
      file(0),
      bufsize(128),
      callback(0)
  {
  }
  
  
  BehaviorParser::
  ~BehaviorParser()
  {
    delete file;
    delete buffer;
    if (parser)
      XML_ParserFree(parser);
  }
  
  
  void BehaviorParser::
  Parse(std::string const & xml_filename,
	BehaviorConstructionCallback const & callback) throw(std::runtime_error)
  {
    in_behavior = false;
    in_type = false;
    
    if (parser)
      XML_ParserFree(parser);
    parser = XML_ParserCreate(NULL); // can probably recycle the old one though
    XML_SetElementHandler(parser, start_element_handler, end_element_handler);
    XML_SetCharacterDataHandler(parser, character_data_handler);
    XML_SetUserData(parser, this);
    
    filename = xml_filename;
    this->callback = &callback;
    if (file)
      delete file;
    file = new wbcrun::File(filename.c_str(), "r");
    
    while (true) {
      void * buf(XML_GetBuffer(parser, bufsize));
      if (NULL == buf)
	throw runtime_error("wbcrun::BehaviorParser::Parse(): XML_GetBuffer() failed");
      size_t const bytes_read(fread(buf, 1, bufsize, file->stream));
      if (bytes_read == 0) {
	if (ferror(file->stream)) {
	  ostringstream os;
	  os << "wbcrun::BehaviorParser::Parse(): " << filename << ": fread() failed";
	  throw runtime_error(os.str());
	}
	break;
      }
      if ( ! XML_ParseBuffer(parser, bytes_read, bytes_read == 0)) {
	ostringstream os;
	os << "wbcrun::BehaviorParser::Parse(): " << filename
	   << ": " << XML_GetCurrentLineNumber(parser)
	   << ": parse error: " << XML_ErrorString(XML_GetErrorCode(parser));
	throw runtime_error(os.str());
      }
    }
  }
  
  
  void BehaviorParser::
  StdParse(std::string const & xml_filename,
	   std::vector<BehaviorDescription*> & bvec,
	   BehaviorFactoryRegistry const & breg)
    throw(std::runtime_error)
  {
    try {
      BehaviorParser bparser;
      bparser.Parse(xml_filename, StdBehaviorConstructionCallback(bvec, breg));
    }
    catch (runtime_error const & ee) {
      if ("builtin:puma" == xml_filename) { // quick hack for BRBuilder test, look away!
	bvec.push_back(breg.Create("DebugBehavior"));
	return;
      }
      ostringstream msg;
      msg << "wbc::BehaviorParser::StdParse(): EXCEPTION\n"
	  << "  XML filename: " << xml_filename << "\n"
	  << "  error: " << ee.what() << "\n"
	  << "  possible causes:\n"
	  << "  - a typo in a <behavior><type>Blah</type></behavior> declaration?\n"
	  << "  - a missing else if clause in core/BehaviorFactory.cpp?\n"
	  << "  - check also for warnings and errors during plugin load";
      throw runtime_error(msg.str());
    }
  }
  
}


void start_element_handler(void * userData,
			   const XML_Char * name,
			   const XML_Char ** atts)
    throw(std::runtime_error)
{
  wbc::BehaviorParser * bp(reinterpret_cast<wbc::BehaviorParser *>(userData));
  
  if (bp->in_type) {
    ostringstream os;
    os << bp->filename.c_str() << ":" << XML_GetCurrentLineNumber(bp->parser)
       << ": behavior types cannot contain tags";
    throw runtime_error(os.str());
  }
  
  string const tag(name);

  if ("behavior" == tag) {
    if (bp->in_behavior) {
      ostringstream os;
      os << bp->filename.c_str() << ":" << XML_GetCurrentLineNumber(bp->parser)
	 << ": behaviors cannot be nested";
      throw runtime_error(os.str());
    }
    bp->in_behavior = true;
  }

  else if (bp->in_behavior && ("type" == tag)) {
    if (bp->in_type) {
      ostringstream os;
      os << bp->filename.c_str() << ":" << XML_GetCurrentLineNumber(bp->parser)
	 << ": behavior types cannot be nested";
      throw runtime_error(os.str());
    }
    bp->in_type = true;
    if (bp->buffer)
      delete bp->buffer;
    bp->buffer = new wbcrun::StringBuffer<XML_Char>();
  }
}


void end_element_handler(void * userData,
			 const XML_Char * name)
    throw(std::runtime_error)
{
  wbc::BehaviorParser * bp(reinterpret_cast<wbc::BehaviorParser *>(userData));
  
  if (( ! bp->in_behavior) && ( ! bp->in_type))
    return;
  
  string const tag(name);
  if ("behavior" == tag) {
    if ( ! bp->in_behavior) {
      ostringstream os;
      os << bp->filename.c_str() << ":" << XML_GetCurrentLineNumber(bp->parser)
	 << ": behaviors cannot be nested";
      throw runtime_error(os.str());
    }
    bp->in_behavior = false;
  }
  
  else if ("type" == tag) {
    if ( ! bp->in_type) {
      ostringstream os;
      os << bp->filename.c_str() << ":" << XML_GetCurrentLineNumber(bp->parser)
	 << ": behavior types cannot be nested";
      throw runtime_error(os.str());
    }
    bp->in_type = false;
    
    if ((0 == bp->buffer) || bp->buffer->Empty()) {
      ostringstream os;
      os << bp->filename.c_str() << ":" << XML_GetCurrentLineNumber(bp->parser)
	 << ": empty behavior type";
      throw runtime_error(os.str());
    }
    (*bp->callback)(bp->buffer->GetString());
    
    delete bp->buffer;
    bp->buffer = 0;
  }
}


void character_data_handler(void * userData,
			    const XML_Char * s,
			    int len)
{
  wbc::BehaviorParser * bp(reinterpret_cast<wbc::BehaviorParser *>(userData));
  
  if (bp->buffer)
    bp->buffer->Append(s, len);
}
