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

/** \author Roland Philippsen */

#ifndef WBC_BR_PARSER_HPP
#define WBC_BR_PARSER_HPP

#include <stdexcept>
#include <string>

namespace wbc {

  class BranchingRepresentation;
  
  class BRParser {
  public:
    virtual ~BRParser() {}
    
    /**
       Create a BranchingRepresentation by parsing a file. The
       interpretation of the file depends on the specific parser that
       this method is being invoked on.
       
       \note Errors are signaled by throwing an exception. If this
       method returns, then you are holding a valid
       BranchingRepresentation instance, which you are responsible for
       destroying when you are done with it.
    */
    virtual BranchingRepresentation * parse(const std::string & fileName)
      throw(std::runtime_error) = 0;
    
    /**
       Create a BranchingRepresentation by looking up a parser based
       on its name and forwarding the call to its parse() method. You
       can pass an empty parserName to instantiate a default parser.
       
       \note Errors can stem from failures in looking up the parser,
       or from the parse() method of the instantiated parser.
    */
    static BranchingRepresentation * parse(const std::string & parserName,
					   const std::string & fileName)
      throw(std::runtime_error);
  };
  
}

#endif // WBC_BR_PARSER_HPP
