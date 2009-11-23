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

#include <wbcrun/util.hpp>
#include <expat.h>
#include <string>
#include <stdexcept>
#include <vector>

namespace wbc {
  
  class BehaviorDescription;
  class BehaviorFactoryRegistry;
  
  
  struct BehaviorConstructionCallback {
    virtual ~BehaviorConstructionCallback() {}
    
    /** \note Declared const so that you can use a temporary instance
	when calling BehaviorParser::Parse(). */
    virtual void operator () (std::string const & name) const throw(std::runtime_error) = 0;
  };
  
  
  /** Simply logs a message to cout. */
  struct DebugBehaviorConstructionCallback:
    public BehaviorConstructionCallback {
    virtual void operator () (std::string const & name) const throw(std::runtime_error);
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
       Asks the BehaviorFactoryRegistry to create a behavior, and adds
       it to the vector of BehaviorDescription instances. Very trivial:
       \code
       m_bvec.push_back(m_breg.Create(name));
       \endcode
       
       \note If the given name is invalid,
       BehaviorFactoryRegistry::Create() will throw an exception.
    */
    virtual void operator () (std::string const & name) const throw(std::runtime_error);
    
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
    wbcrun::StringBuffer<XML_Char> * buffer;
    bool in_behavior;
    bool in_type;
    std::string filename;
    wbcrun::File * file;
    int bufsize;
    BehaviorConstructionCallback const * callback;
  };
  
}

#endif // WBC_BEHAVIOR_PARSER_HPP
