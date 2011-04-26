/*
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 *
 * Author: Roland Philippsen
 *         http://cs.stanford.edu/group/manips/
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

#ifndef OPSPACE_CLASSIC_TASK_POSTURE_CONTROLLER_HPP
#define OPSPACE_CLASSIC_TASK_POSTURE_CONTROLLER_HPP

#include <opspace/Controller.hpp>
#include <boost/shared_ptr.hpp>

namespace opspace {
  
  
  class ClassicTaskPostureController
    : public Controller
  {
  public:
    explicit ClassicTaskPostureController(std::string const & name);
    
    virtual Status init(Model const & model);
    
    virtual Status computeCommand(Model const & model,
				  Skill & skill,
				  Vector & gamma);

    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    Vector jpos_;
    Vector jvel_;
    Vector gamma_;
    Vector fstar_;
    Matrix lambda_;
    Matrix jbar_;
    Matrix nullspace_;
  };

}

#endif // OPSPACE_CLASSIC_TASK_POSTURE_CONTROLLER_HPP
