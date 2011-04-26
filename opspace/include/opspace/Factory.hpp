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

#ifndef OPSPACE_FACTORY_HPP
#define OPSPACE_FACTORY_HPP

#include <jspace/Status.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <map>


namespace opspace {
  
  using jspace::Status;
  
  class Task;
  class Skill;
  class ReflectionRegistry;
  
  
  template<typename base_type>
  struct ShopAPI {
    virtual ~ShopAPI() {}
    virtual base_type * create(std::string const & name) = 0;
  };
  
  template<typename task_subtype>
  struct TaskShop : public ShopAPI<Task> {
    virtual Task * create(std::string const & name) { return new task_subtype(name); }
  };
  
  template<typename skill_subtype>
  struct SkillShop : public ShopAPI<Skill> {
    virtual Skill * create(std::string const & name) { return new skill_subtype(name); }
  };
  
  
  /**
     Utility for creating Task and Skill instances based on type
     names. For the moment, we use a hardcoded mapping from type names
     to subclasses, but it will be really easy to extend this with
     some sort of plugin approach.
     
     The idea is to call the parseFoo() methods to feed it with task
     and skill specifications, and then retrieve them using
     getTaskTable() and/or getSkillTable().
     
     The YAML format is quite simple: you specify a list of
     dictionaries. When the key is `tasks', then it parses a list of
     tasks. Similarly, it parses skills when the dictionary key is
     `skills'. You can mix and match, but beware that tasks
     referenced by a skill must be defined before they can be used.
     
     For tasks, the specification is again a list of
     dictionaries. Each one specifies a task. The required dictionary
     keys are `type' and `name'. All the others get looked up via
     Task::lookupParameter() and directly specify task parameters.
     
     Skills are very similar, except that dictionary values are
     handled differently depending on whether they are lists or
     dictionaries: if they are dictionaries, they are treated as state
     definitions which define which task goes where in the slots
     provided by the skill. Otherwise, they are treated as
     parameter definitions.
     
     An example task YAML file is:
     \verbatim
     - tasks:
       - type: opspace::PositionTask
         name: eepos
         dt_seconds: 0.002
         kp: [ 100.0 ]
         kd: [  20.0 ]
         maxvel: [ 0.5 ]
         maxacc: [ 1.5 ]
       - type: opspace::PostureTask
         name: posture
         dt_seconds: 0.002
         kp: [ 400.0, 400.0, 400.0, 100.0, 100.0, 100.0, 100.0 ]
         kd: [  40.0,  40.0,  40.0,  20.0,  20.0,  20.0,  20.0 ]
         maxvel: [ 3.1416 ]
         maxacc: [ 6.2832 ]
     - skills:
       - type: opspace::TPSkill
         name: task_posture
	 # dictionaries define task slots
	 default:
	   eepos: eepos
	   posture: posture
	 # key-value pairs define parameters
	 some_param_name: some_param_value
     \endverbatim
  */
  class Factory
  {
  public:
    typedef std::vector<boost::shared_ptr<Task> > task_table_t;
    typedef std::vector<boost::shared_ptr<Skill> > skill_table_t;
    
    static void setDebugStream(std::ostream * dbg);
    
    template<typename task_subtype>
    static void addTaskType(std::string const & type) {
      task_shop__[type].reset(new TaskShop<task_subtype>());
    }
    
    template<typename skill_subtype>
    static void addSkillType(std::string const & type) {
      skill_shop__[type].reset(new SkillShop<skill_subtype>());
    }
    
    static Task * createTask(std::string const & type, std::string const & name);
    
    static Skill * createSkill(std::string const & type, std::string const & name);
    
    /**
       Parse a YAML document contained in a string. Retrieve the
       result using getTaskTable().
    */
    Status parseString(std::string const & yaml_string);
    
    /**
       Parse a YAML file, specified as a filename. Retrieve the result
       using getTaskTable().
    */
    Status parseFile(std::string const & yaml_filename);
    
    /**
       Parse a YAML file from a stream. Retrieve the result using
       getTaskTable().
    */
    Status parseStream(std::istream & yaml_istream);
    
    /**
       The task table contains pointers to all task instances ever
       created by this Factory, in the order that they were
       encountered in the YAML documents.
    */
    task_table_t const & getTaskTable() const;
    
    /**
       The skill table contains pointers to all skill instances
       ever created by this Factory, in the order that they were
       encountered in the YAML documents.
    */
    skill_table_t const & getSkillTable() const;
    
    boost::shared_ptr<Task> findTask(std::string const & name) const;
    boost::shared_ptr<Skill> findSkill(std::string const & name) const;
    
    /**
       Create a ReflectionRegistry and populate it with the currently
       registered task and skill instances.
    */
    ReflectionRegistry * createRegistry();
    
    /**
       Write a human-readable (hopefully, anyway) description of all
       skills and tasks contained in the tables.
    */
    void dump(std::ostream & os,
	      std::string const & title,
	      std::string const & prefix) const;
    
  protected:
    typedef std::map<std::string, boost::shared_ptr<ShopAPI<Task> > > task_shop_t;
    typedef std::map<std::string, boost::shared_ptr<ShopAPI<Skill> > > skill_shop_t;

    static std::ostream * dbg__;
    static task_shop_t task_shop__;
    static skill_shop_t skill_shop__;
    
    task_table_t task_table_;
    skill_table_t skill_table_;
  };
  
}

#endif // OPSPACE_FACTORY_HPP
