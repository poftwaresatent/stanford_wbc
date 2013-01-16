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

#ifndef UTA_OPSPACE_CONTROLLER_NG_HPP
#define UTA_OPSPACE_CONTROLLER_NG_HPP

#include <opspace/Controller.hpp>
#include <boost/shared_ptr.hpp>

namespace uta_opspace {
  
  using namespace opspace;
  
  
  class ControllerNG
    : public Controller
  {
  public:
    explicit ControllerNG(std::string const & name);
    
    void setFallbackTask(boost::shared_ptr<Task> task);
    
    virtual Status init(Model const & model);

    virtual Status computeCommand(Model const & model,
				  Skill & skill,
				  Vector & gamma);
    
    virtual Status check(std::string const * param, std::string const & value) const;
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
    Status computeFallback(Model const & model,
			   bool init_required,
			   Vector & gamma);
    
    inline Vector const & getCommand() const { return gamma_; }
    
    void qhlog(Skill & skill, long long timestamp);
    
    
  protected:
    boost::shared_ptr<Task> fallback_task_;
    ////    std::vector<Vector> sv_lstar_; // stored only for dbg()
    bool fallback_;
    std::string fallback_reason_;
    
    std::vector<boost::shared_ptr<ParameterLog> > log_;
    int loglen_;		// <= 0 means disabled
    int logsubsample_;
    std::string logprefix_;
    
    // -1 means off, 0 means init, -2 means maybeWriteLogFiles() will
    // actually write them (this gets set when ==loglen_)
    mutable int logcount_;
    
    // for logging and debugging via Parameter tools, don't bother to
    // implement check() methods because one day real soon now we'll
    // be able to flag parameters as read-only and then the superclass
    // can take care of signaling errors when someone writes to
    // them...
    Vector jpos_;
    Vector jvel_;
    Vector gamma_;
  };

}

#endif // UTA_OPSPACE_CONTROLLER_NG_HPP
