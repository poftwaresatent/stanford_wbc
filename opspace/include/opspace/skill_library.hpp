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

#ifndef OPSPACE_SKILL_LIBRARY_HPP
#define OPSPACE_SKILL_LIBRARY_HPP

#include <opspace/Skill.hpp>
#include <opspace/task_library.hpp>

namespace opspace {
  
  
  class GenericSkill
    : public Skill
  {
  public:
    GenericSkill(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual task_table_t const * getTaskTable();
    
    void appendTask(boost::shared_ptr<Task> task);
    
  protected:
    task_table_t task_table_;
    boost::shared_ptr<TaskSlotAPI> slot_;
  };
  
  
  class TaskPostureSkill
    : public Skill
  {
  public:
    TaskPostureSkill(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual task_table_t const * getTaskTable();
    virtual Status checkJStarSV(Task const * task, Vector const & sv);
    
  protected:
    CartPosTask * eepos_;
    JPosTask * posture_;
    task_table_t task_table_;
  };
  
  
  class TaskPostureTrjSkill
    : public Skill
  {
  public:
    TaskPostureTrjSkill(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual task_table_t const * getTaskTable();
    virtual Status checkJStarSV(Task const * task, Vector const & sv);
    
  protected:
    CartPosTrjTask * eepos_;
    JPosTrjTask * posture_;
    task_table_t task_table_;
  };
  
}

#endif // OPSPACE_SKILL_LIBRARY_HPP
