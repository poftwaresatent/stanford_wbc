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

#include <opspace/skill_library.hpp>
#include <opspace/task_library.hpp>

using boost::shared_ptr;


namespace opspace {
  
  
  GenericSkill::
  GenericSkill(std::string const & name)
    : Skill(name)
  {
    slot_ = declareSlot<Task>("task");
  }
  
  
  Status GenericSkill::
  init(Model const & model)
  {
    Status const st(Skill::init(model));
    if ( ! st) {
      return st;
    }
    if (task_table_.empty()) {
      for (size_t ii(0); ii < slot_->getNInstances(); ++ii) {
	task_table_.push_back(slot_->getInstance(ii).get());
      }
    }
    return st;
  }
  
  
  Status GenericSkill::
  update(Model const & model)
  {
    Status st;
    if ( task_table_.empty()) {
      st.ok = false;
      st.errstr = "empty task table, did you assign any? did you forget to init()?";
    }
    else {
      for (size_t ii(0); ii < task_table_.size(); ++ii) {
	st = task_table_[ii]->update(model);
	if ( ! st) {
	  return st;
	}
      }
    }
    return st;
  }
  
  
  Skill::task_table_t const * GenericSkill::
  getTaskTable()
  {
    return &task_table_;
  }
  
  
  void GenericSkill::
  appendTask(boost::shared_ptr<Task> task)
  {
    slot_->assign(task);
  }
  
  
  TaskPostureSkill::
  TaskPostureSkill(std::string const & name)
    : Skill(name)
  {
    declareSlot("eepos", &eepos_);
    declareSlot("posture", &posture_);
  }
  
  
  Status TaskPostureSkill::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) {
      return st;
    }
    task_table_.push_back(eepos_);
    task_table_.push_back(posture_);
    return st;
  }
  
  
  Status TaskPostureSkill::
  update(Model const & model)
  {
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      Status const st(task_table_[ii]->update(model));
      if ( ! st) {
	return st;
      }
    }
    Status ok;
    return ok;
  }
  
  
  Skill::task_table_t const * TaskPostureSkill::
  getTaskTable()
  {
    return &task_table_;
  }
  
  
  Status TaskPostureSkill::
  checkJStarSV(Task const * task, Vector const & sv)
  {
    if (task == eepos_) {
      if (sv.rows() != 3) {
	return Status(false, "eepos dimension mismatch");
      }
      if (sv[2] < eepos_->getSigmaThreshold()) {
	return Status(false, "singular eepos");
      }
    }
    Status ok;
    return ok;
  }


  TaskPostureTrjSkill::
  TaskPostureTrjSkill(std::string const & name)
    : Skill(name)
  {
    declareSlot("eepos", &eepos_);
    declareSlot("posture", &posture_);
  }
  
  
  Status TaskPostureTrjSkill::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) {
      return st;
    }
    task_table_.push_back(eepos_);
    task_table_.push_back(posture_);
    return st;
  }
  
  
  Status TaskPostureTrjSkill::
  update(Model const & model)
  {
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      Status const st(task_table_[ii]->update(model));
      if ( ! st) {
	return st;
      }
    }
    Status ok;
    return ok;
  }
  
  
  Skill::task_table_t const * TaskPostureTrjSkill::
  getTaskTable()
  {
    return &task_table_;
  }
  
  
  Status TaskPostureTrjSkill::
  checkJStarSV(Task const * task, Vector const & sv)
  {
    if (task == eepos_) {
      if (sv.rows() != 3) {
	return Status(false, "eepos dimension mismatch");
      }
      if (sv[2] < eepos_->getSigmaThreshold()) {
	return Status(false, "singular eepos");
      }
    }
    Status ok;
    return ok;
  }
  
}
