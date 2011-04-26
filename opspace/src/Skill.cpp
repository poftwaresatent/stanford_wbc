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

#include <opspace/Skill.hpp>
#include <opspace/task_library.hpp>

using boost::shared_ptr;


namespace opspace {
  
  
  Skill::
  Skill(std::string const & name)
    : ParameterReflection("skill", name)
  {
  }
  
  
  Skill::
  ~Skill()
  {
  }
  
  
  Status Skill::
  init(Model const & model)
  {
    bool ok(true);

    {
      std::ostringstream msg;
      msg << "missing non-optional task instances:\n";
      for (slot_map_t::const_iterator is(slot_map_.begin()); is != slot_map_.end(); ++is) {
	if ((0 == is->second->getNInstances())
	    && ( ! is->second->isOptional())) {
	  ok = false;
	  msg << "  slot `" << is->first << "' task `" << is->first << "'\n";
	}
      }
      if ( ! ok) {
	return Status(false, msg.str());
      }
    }
    
    {
      std::ostringstream msg;
      msg << "failed task initializations:\n";
      for (slot_map_t::const_iterator is(slot_map_.begin()); is != slot_map_.end(); ++is) {
	for (size_t it(0); it < is->second->getNInstances(); ++it) {
	  shared_ptr<Task> task(is->second->getInstance(it));
	  Status const st(task->init(model));
	  if ( ! st) {
	    ok = false;
	    msg << "  slot `" << is->first << "' task[" << it << "] `" << task->getName()
		<< "': " << st.errstr << "\n";
	  }
	}
      }
      if ( ! ok) {
	return Status(false, msg.str());
      }
    }
    
    return Status();
  }
  
  
  boost::shared_ptr<TaskSlotAPI> Skill::
  lookupSlot(std::string const & name)
  {
    shared_ptr<TaskSlotAPI> slot;
    slot_map_t::iterator ism(slot_map_.find(name));
    if (ism != slot_map_.end()) {
      slot = ism->second;
    }
    return slot;
  }
  
  
  void Skill::
  dump(std::ostream & os,
       std::string const & title,
       std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "skill " << instance_name_ << "\n";
    ParameterReflection::dump(os, prefix + "  parameters:", prefix + "    ");
    os << prefix << "  slots:\n";
    for (slot_map_t::const_iterator is(slot_map_.begin()); is != slot_map_.end(); ++is) {
      os << prefix << "    " << is->first;
      if (0 == is->second->getNInstances()) {
	os << " (EMPTY)\n";
      }
      else if (1 == is->second->getNInstances()) {
	os << ": " << is->second->getInstance(0)->getName() << "\n";
      }
      else {
	os << ":\n";
	for (size_t it(0); it < is->second->getNInstances(); ++it) {
	  os << prefix << "      [" << it << "]: "
	     << is->second->getInstance(it)->getName() << "\n";
	}
      }
    }
  }
  
  
  void Skill::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "skill " << instance_name_ << "\n";
    ParameterReflection::dump(os, prefix + "  parameters", prefix + "    ");
    for (slot_map_t::const_iterator is(slot_map_.begin()); is != slot_map_.end(); ++is) {
      for (size_t it(0); it < is->second->getNInstances(); ++it) {
	Task const * task(is->second->getInstance(it).get());
	task->dbg(os, "  " + is->first + "/" + task->getName(), prefix + "    ");
      }
    }
  }
  
}
