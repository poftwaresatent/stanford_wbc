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

#include <opspace/Factory.hpp>
#include <opspace/Skill.hpp>
#include <opspace/task_library.hpp>
#include <opspace/skill_library.hpp>
#include <opspace/parse_yaml.hpp>
#include <fstream>
#include <stdexcept>

using jspace::pretty_print;

namespace opspace {
  

  static bool shops_initialized__(false);
  
  static void init_shops()
  {
    if (shops_initialized__) {
      return;
    }
    shops_initialized__ = true;
    
    Factory::addTaskType<opspace::SelectedJointPostureTask>("opspace::SelectedJointPostureTask");
    Factory::addTaskType<opspace::CartPosTrjTask>("opspace::CartPosTrjTask");
    Factory::addTaskType<opspace::JPosTrjTask>("opspace::JPosTrjTask");
    Factory::addTaskType<opspace::CartPosTask>("opspace::CartPosTask");
    Factory::addTaskType<opspace::JPosTask>("opspace::JPosTask");
    Factory::addTaskType<opspace::JointLimitTask>("opspace::JointLimitTask");
    Factory::addTaskType<opspace::OrientationTask>("opspace::OrientationTask");
    Factory::addTaskType<opspace::DraftPIDTask>("opspace::DraftPIDTask");

    Factory::addSkillType<opspace::GenericSkill>("opspace::GenericSkill");
    Factory::addSkillType<opspace::TaskPostureSkill>("opspace::TaskPostureSkill");
    Factory::addSkillType<opspace::TaskPostureTrjSkill>("opspace::TaskPostureTrjSkill");
  }
  
  
  std::ostream * Factory::dbg__(0);
  Factory::task_shop_t Factory::task_shop__;
  Factory::skill_shop_t Factory::skill_shop__;
  
  
  void Factory::
  setDebugStream(std::ostream * dbg)
  {
    dbg__ = dbg;
  }
  
  
  Task * Factory::
  createTask(std::string const & type, std::string const & name)
  {
    init_shops();
    task_shop_t::iterator ii(task_shop__.find(type));
    if (task_shop__.end() == ii) {
      if (dbg__) {
	(*dbg__) << "Factory::createTask(): no task type `" << type << "'\n"
		 << "  registered types are:\n";
	for (ii = task_shop__.begin(); ii != task_shop__.end(); ++ii) {
	  (*dbg__) << "    " << ii->first << "\n";
	}
      }
      return 0;
    }
    return ii->second->create(name);
  }
  
  
  Skill * Factory::
  createSkill(std::string const & type, std::string const & name)
  {
    init_shops();
    skill_shop_t::iterator ii(skill_shop__.find(type));
    if (skill_shop__.end() == ii) {
      if (dbg__) {
	(*dbg__) << "Factory::createSkill(): no skill type `" << type << "'\n"
		 << "  registered types are:\n";
	for (ii = skill_shop__.begin(); ii != skill_shop__.end(); ++ii) {
	  (*dbg__) << "    " << ii->first << "\n";
	}
      }
      return 0;
    }
    return ii->second->create(name);
  }
  
  
  Status Factory::
  parseString(std::string const & yaml_string)
  {
    std::istringstream is(yaml_string);
    return parseStream(is);
  }
  
  
  Status Factory::
  parseFile(std::string const & yaml_filename)
  {
    std::ifstream is(yaml_filename.c_str());
    if ( ! is) {
      return Status(false, "could not open file `" + yaml_filename + "' for reading");
    }
    return parseStream(is);
  }


  Status Factory::
  parseStream(std::istream & yaml_istream)
  {
    Status st;
    boost::shared_ptr<Task> task;
    
    try {
      YAML::Parser parser(yaml_istream);
      YAML::Node doc;
      TaskTableParser task_table_parser(*this, task_table_, dbg__);
      SkillTableParser skill_table_parser(*this, skill_table_, dbg__);
      
      parser.GetNextDocument(doc); // <sigh>this'll have merge conflicts again</sigh>
      //while (parser.GetNextDocument(doc)) {
      {
	for (YAML::Iterator ilist(doc.begin()); ilist != doc.end(); ++ilist) {
	  for (YAML::Iterator idict(ilist->begin()); idict != ilist->end(); ++idict) {
	    std::string key;
	    idict.first() >> key;
	    if ("tasks" == key) {
	      idict.second() >> task_table_parser;
	    }
	    else if ("skills" == key) {
	      idict.second() >> skill_table_parser;
	    }
	    else if ("behaviors" == key) {
	      throw std::runtime_error("deprecated key `behaviors' (use `skills' instead)");
	    }
	    else {
	      throw std::runtime_error("invalid key `" + key + "'");
	    }
	  }
	}
      }
    }
    catch (YAML::Exception const & ee) {
      if (dbg__) {
	*dbg__ << "YAML::Exception: " << ee.what() << "\n";
      }
      st.ok = false;
      st.errstr = ee.what();
    }
    catch (std::runtime_error const & ee) {
      if (dbg__) {
	*dbg__ << "std::runtime_error: " << ee.what() << "\n";
      }
      st.ok = false;
      st.errstr = ee.what();
    }
    
    return st;
  }
  
  
  Factory::task_table_t const & Factory::
  getTaskTable() const
  {
    return task_table_;
  }
  
  
  Factory::skill_table_t const & Factory::
  getSkillTable() const
  {
    return skill_table_;
  }
  
  
  void Factory::
  dump(std::ostream & os,
       std::string const & title,
       std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "  tasks:\n";
    for (task_table_t::const_iterator it(task_table_.begin());
	 it != task_table_.end(); ++it) {
      (*it)->dump(os, "", prefix + "    ");
    }
    os << prefix << "  skills:\n";
    for (skill_table_t::const_iterator it(skill_table_.begin());
	 it != skill_table_.end(); ++it) {
      (*it)->dump(os, "", prefix + "    ");
    }
  }

  
  boost::shared_ptr<Task> Factory::
  findTask(std::string const & name)
    const
  {
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      if (name == task_table_[ii]->getName()) {
	return task_table_[ii];
      }
    }
    return boost::shared_ptr<Task>();
  }

  
  boost::shared_ptr<Skill> Factory::
  findSkill(std::string const & name)
    const
  {
    for (size_t ii(0); ii < skill_table_.size(); ++ii) {
      if (name == skill_table_[ii]->getName()) {
	return skill_table_[ii];
      }
    }
    return boost::shared_ptr<Skill>();
  }
  
  
  ReflectionRegistry * Factory::
  createRegistry()
  {
    ReflectionRegistry * reg(new ReflectionRegistry());
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      reg->add(task_table_[ii]);
    }
    for (size_t ii(0); ii < skill_table_.size(); ++ii) {
      reg->add(skill_table_[ii]);
    }
    return reg;
  }
  
}
