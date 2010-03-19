/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
   \file jspace/Controller.hpp
   \author Roland Philippsen
*/

#include <vector>
#include <string>


namespace jspace {
  
  class Model;
  
  
  struct status_s {
    /** Default ctor sets \c ok=true and \c errstr="" */
    status_s();
    status_s(bool ok, std::string const & errstr);
    
    bool ok;
    std::string errstr;
  };
  
  
  class Controller
  {
  public:
    virtual ~Controller();
    
    /** Default init just returns ok. */
    virtual status_s init(Model const & model);
    
    virtual status_s setGoal(std::vector<double> const & goal) = 0;
    virtual status_s getGoal(std::vector<double> & goal) const = 0;
    virtual status_s getActual(std::vector<double> & actual) const = 0;
    
    virtual status_s setGains(std::vector<double> const & kp, std::vector<double> const & kd) = 0;
    virtual status_s getGains(std::vector<double> & kp, std::vector<double> & kd) const = 0;
    
    virtual status_s computeCommand(Model const & model, std::vector<double> & tau) = 0;
  };
  
}
