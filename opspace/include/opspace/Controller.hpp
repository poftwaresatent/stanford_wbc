/*
 * Shared copyright notice and LGPLv3 license statement.
 *
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 * Copyright (C) 2011 University of Texas at Austin. All rights reserved.
 *
 * Authors: Roland Philippsen (Stanford) and Luis Sentis (UT Austin)
 *          http://cs.stanford.edu/group/manips/
 *          http://www.me.utexas.edu/~hcrl/
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

#ifndef OPSPACE_CONTROLLER_HPP
#define OPSPACE_CONTROLLER_HPP

#include <opspace/Parameter.hpp>
#include <opspace/Skill.hpp>

namespace opspace {
    
  class Controller
    : public ParameterReflection
  {
  protected:
    explicit Controller(std::string const & name);
    
  public:
    virtual Status init(Model const & model) = 0;
    
    virtual Status computeCommand(Model const & model,
				  Skill & skill,
				  Vector & gamma) = 0;
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const {}
  };
  
}

#endif // OPSPACE_CONTROLLER_HPP
