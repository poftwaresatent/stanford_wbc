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

#ifndef OPSPACE_SKILL_HPP
#define OPSPACE_SKILL_HPP

#include <opspace/Task.hpp>
#include <opspace/Parameter.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>

namespace opspace {
  
  
  typedef enum {
    TASK_SLOT_DEFAULT = 0,
    TASK_SLOT_OPTIONAL = 1
  } task_slot_flags_t;
  
  
  class TaskSlotAPI
  {
  public:
    std::string const name_;
    task_slot_flags_t const flags_;
    
    TaskSlotAPI(std::string const & name, task_slot_flags_t flags)
      : name_(name), flags_(flags) {}
    
    virtual ~TaskSlotAPI() {}
    
    virtual Status assign(boost::shared_ptr<Task> instance)
    { return Status(false, "type mismatch"); }
    
    virtual size_t getNInstances() const = 0;
    virtual boost::shared_ptr<Task> getInstance(size_t index) = 0;
    
    inline bool isOptional() const { return flags_ & TASK_SLOT_OPTIONAL; }
  };
  
  
  template<typename task_subtype>
  class TaskSlot
    : public TaskSlotAPI
  {
  public:
    TaskSlot(std::string const & name,
	     task_subtype ** slot,
	     task_slot_flags_t flags)
      : TaskSlotAPI(name, flags), slot_(slot) {}
    
    virtual Status assign(boost::shared_ptr<Task> instance) {
      Task * base(instance.get());
      if ( ! base) {
	return Status(false, "null instance");
      }
      task_subtype * raw(dynamic_cast<task_subtype *>(base));
      if ( ! raw) {
	return Status(false, "type mismatch");
      }
      if (slot_) {
	*slot_ = raw;
      }
      instances_.push_back(instance);
      return Status();
    }

    virtual size_t getNInstances() const { return instances_.size(); }
    virtual boost::shared_ptr<Task> getInstance(size_t index) { return instances_[index]; }
    
  protected:
    task_subtype ** slot_;
    std::vector<boost::shared_ptr<Task> > instances_;
  };
  
  
  class Skill
    : public ParameterReflection
  {
  public:
    typedef std::vector<Task *> task_table_t;
    
    virtual ~Skill();
    
    virtual Status update(Model const & model) = 0;
    virtual task_table_t const * getTaskTable() = 0;
    
    virtual Status init(Model const & model);
    virtual Status checkJStarSV(Task const * task, Vector const & sv) { Status ok; return ok; }
    
    inline std::string const & getName() const { return name_; }
    
    boost::shared_ptr<TaskSlotAPI> lookupSlot(std::string const & name);
    
    virtual void dump(std::ostream & os,
		      std::string const & title,
		      std::string const & prefix) const;
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    Skill(std::string const & name);
    
    template<typename task_subtype>
    boost::shared_ptr<TaskSlotAPI>
    declareSlot(std::string const & name,
		task_subtype ** slot = 0,
		task_slot_flags_t flags = TASK_SLOT_DEFAULT)
    {
      boost::shared_ptr<TaskSlotAPI>
	slot_api(new TaskSlot<task_subtype>(name, slot, flags));
      slot_map_[name] = slot_api;
      return slot_api;
    }
    
    std::string const name_;
    
  private:
    typedef std::map<std::string, boost::shared_ptr<TaskSlotAPI> > slot_map_t;
    slot_map_t slot_map_;
  };
  
}

#endif // OPSPACE_SKILL_HPP
