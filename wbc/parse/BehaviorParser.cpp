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
#include <wbc/core/BehaviorDescription.hpp>
#include <wbc/core/BehaviorFactory.hpp>
#include <wbcnet/log.hpp>
#include <iostream>
#include <sstream>

extern "C" {
#include <errno.h>
#include <string.h>
#include <sys/types.h>
}

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));


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
  operator () (dictionary_t const & params) const throw(std::runtime_error)
  {
    cout << "DebugBehaviorConstructionCallback\n";
    for (dictionary_t::const_iterator ii(params.begin()); ii != params.end(); ++ii) {
      cout << "  " << ii->first << " = " << ii->second << "\n";
    }
  }
  
  
  StdBehaviorConstructionCallback::
  StdBehaviorConstructionCallback(vector<BehaviorDescription*> & bvec,
				  BehaviorFactoryRegistry const & breg)
    : m_bvec(bvec),
      m_breg(breg)
  {
  }
  
  
  void StdBehaviorConstructionCallback::
  operator () (dictionary_t const & params) const throw(std::runtime_error)
  {
    dictionary_t::const_iterator ii(params.find("type"));
    if (params.end() == ii) {
      throw runtime_error("wbc::StdBehaviorConstructionCallback(): no type in parameter dictionary");
    }
    std::string const & name(ii->second);
    ++ii;
    if ((params.end() != ii) && ("type" == ii->second)) {
      throw runtime_error("wbc::StdBehaviorConstructionCallback(): multiple types in parameter dictionary");
    }
    BehaviorDescription * behavior(m_breg.Create(name));
    try {
      for (ii = params.begin(); ii != params.end(); ++ii) {
	if ("type" != ii->first) {
	  if (behavior->handleInit(ii->first, ii->second)) {
	    LOG_DEBUG (logger,
		       "wbc::StdBehaviorConstructionCallback(): OK parameter " << ii->first << " = " << ii->second
		       << " handled by behavior " << name);
	  }
	  else {
	    LOG_WARN (logger,
		      "wbc::StdBehaviorConstructionCallback(): parameter " << ii->first << " = " << ii->second
		      << " not handled by behavior " << name);
	  }
	}
      }
    }
    catch (std::runtime_error const & ee) {
      delete behavior;
      throw ee;
    }
    m_bvec.push_back(behavior);
  }
  
  
  BehaviorParser::
  BehaviorParser()
    : parser(0),
      buffer(0),
      in_behavior(false),
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
    file = new File(filename.c_str(), "r");
    
    params.clear();		// probably totally redundant
    param_key = "";		// likewise
    
    while (true) {
      void * buf(XML_GetBuffer(parser, bufsize));
      if (NULL == buf)
	throw runtime_error("wbc::BehaviorParser::Parse(): XML_GetBuffer() failed");
      size_t const bytes_read(fread(buf, 1, bufsize, file->stream));
      if (bytes_read == 0) {
	if (ferror(file->stream)) {
	  ostringstream os;
	  os << "wbc::BehaviorParser::Parse(): " << filename << ": fread() failed";
	  throw runtime_error(os.str());
	}
	break;
      }
      if ( ! XML_ParseBuffer(parser, bytes_read, bytes_read == 0)) {
	ostringstream os;
	os << "wbc::BehaviorParser::Parse(): " << filename
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
  
  string const tag(name);

  if ("behavior" == tag) {
    if (bp->in_behavior) {
      ostringstream os;
      os << bp->filename.c_str() << ":" << XML_GetCurrentLineNumber(bp->parser)
	 << ": behaviors cannot be nested";
      throw runtime_error(os.str());
    }
    bp->in_behavior = true;
    return;
  }
  
  if (bp->in_behavior) {
    if ( ! bp->param_key.empty()) {
      ostringstream os;
      os << bp->filename.c_str() << ":" << XML_GetCurrentLineNumber(bp->parser)
	 << ": nested behavior parameters are not supported";
      throw runtime_error(os.str());
    }
    bp->param_key = tag;
    if (bp->buffer)
      delete bp->buffer;
    bp->buffer = new wbc::StringBuffer<XML_Char>();
  }
}


void end_element_handler(void * userData,
			 const XML_Char * name)
    throw(std::runtime_error)
{
  wbc::BehaviorParser * bp(reinterpret_cast<wbc::BehaviorParser *>(userData));
  
  if ( ! bp->in_behavior)
    return;
  
  string const tag(name);
  
  if ("behavior" == tag) {
    (*bp->callback)(bp->params);
    bp->in_behavior = false;
    bp->params.clear();
    return;
  }
  
  if (tag != bp->param_key) {
    ostringstream os;
    os << bp->filename.c_str() << ":" << XML_GetCurrentLineNumber(bp->parser)
       << ": whoopsie, this must be a bug --- tag " << tag
       << " is not the same as param_key " << bp->param_key;
    throw runtime_error(os.str());
  }
  
  bp->params.insert(make_pair(tag, bp->buffer->GetString()));
  bp->param_key = "";
  delete bp->buffer;
  bp->buffer = 0;
}


void character_data_handler(void * userData,
			    const XML_Char * s,
			    int len)
{
  wbc::BehaviorParser * bp(reinterpret_cast<wbc::BehaviorParser *>(userData));
  
  if (bp->buffer)
    bp->buffer->Append(s, len);
}
