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
   \file options.hpp
   \author Roland Philippsen
*/

#ifndef WBC_OPTIONS_HPP
#define WBC_OPTIONS_HPP

#include <iosfwd>
#include <string>

namespace wbc {
  
  /**
     A collection of options that are useful for \c wbcmodel and \c
     wbcservo. Used to initialize wbcnet::attributes instances.
  */
  struct options {
    options();
    virtual ~options() {}
    
    /**
       Parse the given command line option. If an unknown option is
       encountered, the unknown_option() method is called to give you
       a chance to catch it. When an error or the \c -h option is
       encountered, the usage message is printed and \c exit() is
       called.
       
       \note This method can call \c exit(). If it returns, then the
       option fields have been filled in from default values and
       command line arguments.
     */
    void parse(int argc, char ** argv);
    
    /**
       Print a usage message to the given stream. You can append to
       the message by overriding the specific_usage() method.
    */
    void usage(std::ostream & os) const;
    
    /**
       Hook for subclasses, called when a non-standard option is
       encounetered by parse().
       
       \return -1 if failed, or next argnum on success
    */
    virtual int unknown_option(int argc, char ** argv, int argnum) { return -1; }
    
    /**
       Hook for subclasses, called from within usage().
    */
    virtual void specific_usage(std::ostream & os) const {}
    
#ifndef DISABLE_NETWORKING
    std::string communication;
#endif // DISABLE_NETWORKING
    
    std::string xml_filename;
    std::string motion_type;	// hmm... what's more appropriate? cmd line option or parse from XML file?
    std::string brparser_type;
    size_t verbosity;		// 0: default, 1: INFO or more, 2: DEBUG or more, 3: TRACE
    size_t timestats_skip;
  };

}

#endif // WBC_OPTIONS_HPP
