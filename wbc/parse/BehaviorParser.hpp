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
   \file BehaviorParser.hpp
   \author Roland Philippsen
*/

#ifndef WBC_BEHAVIOR_PARSER_HPP
#define WBC_BEHAVIOR_PARSER_HPP

#include <wbc/util/StringBuffer.hpp>
#include <wbc/util/File.hpp>
#include <expat.h>
#include <string>
#include <stdexcept>
#include <vector>
#include <map>

namespace wbc {
  
  class BehaviorDescription;
  class BehaviorFactoryRegistry;
  
  
  struct BehaviorConstructionCallback {
    typedef std::multimap<std::string, std::string> dictionary_t;
    
    virtual ~BehaviorConstructionCallback() {}
    
    /** \note Declared const so that you can use a temporary instance
	when calling BehaviorParser::Parse(). */
    virtual void operator () (dictionary_t const & params) const throw(std::runtime_error) = 0;
  };
  
  
  /** Simply logs a message to cout. */
  struct DebugBehaviorConstructionCallback:
    public BehaviorConstructionCallback {
    virtual void operator () (dictionary_t const & params) const throw(std::runtime_error);
  };
  
  
  /**
     Use a BehaviorFactoryRegistry instance to populate a vector of
     BehaviorDescription instances.
  */
  struct StdBehaviorConstructionCallback
    : public BehaviorConstructionCallback
  {
    StdBehaviorConstructionCallback(/** Each successful call to
					operator() adds one behavior
					at the end of this vector. */
				    std::vector<BehaviorDescription*> & bvec,
				    /** The registry of behavior
					factories that will be used
					for creating behaviors based
					on their name.
				    */
				    BehaviorFactoryRegistry const & breg);
    
    /**
       Retrieves the "type" value from the dictionary, asks the
       BehaviorFactoryRegistry to create a behavior from it, calls the
       behavior's handleInit() method on all other dictionary entries,
       and finally adds it to the vector of BehaviorDescription
       instances.
    */
    virtual void operator () (dictionary_t const & params) const throw(std::runtime_error);
    
  protected:  
    std::vector<BehaviorDescription*> & m_bvec;
    BehaviorFactoryRegistry const & m_breg;
  };
  
  
  class BehaviorParser
  {
  public:
    BehaviorParser();
    virtual ~BehaviorParser();
    
    void Parse(std::string const & xml_filename,
	       BehaviorConstructionCallback const & callback) throw(std::runtime_error);
    
    static void StdParse(std::string const & xml_filename,
			 std::vector<BehaviorDescription*> & bvec,
			 BehaviorFactoryRegistry const & breg)
      throw(std::runtime_error);
    
    // Everything is public because we have to access it from C
    // callback functions. That does not mean you can just go and muck
    // with my fields, though.
    
    XML_ParserStruct * parser;
    StringBuffer<XML_Char> * buffer;
    bool in_behavior;
    BehaviorConstructionCallback::dictionary_t params;
    std::string param_key;
    std::string filename;
    File * file;
    int bufsize;
    BehaviorConstructionCallback const * callback;
  };
  
}

#endif // WBC_BEHAVIOR_PARSER_HPP
