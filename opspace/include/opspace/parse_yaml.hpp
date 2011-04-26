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

#ifndef OPSPACE_PARSE_YAML_HPP
#define OPSPACE_PARSE_YAML_HPP

#include <yaml-cpp/yaml.h>
#include <opspace/Factory.hpp>
#include <jspace/wrap_eigen.hpp>

namespace opspace {

  using jspace::Vector;
  
  
  class Parser
  {
  public:
    Parser(Factory const & factory, std::ostream * optional_dbg_os = 0);
    virtual ~Parser();
    
    Factory const & factory;
    std::ostream * dbg;
  };
  
  
  class TaskParser
    : public Parser
  {
  public:
    TaskParser(Factory const & factory, std::ostream * optional_dbg_os = 0);
    
    std::string type;
    std::string name;
    
    /** After successfully parsing a YAML node, this contains the
	pointer to the freshly created task. If something goes wrong,
	task will be zero.
	
	\note You are responsible for eventually deleting this Task
	instance.
    */
    Task * task;
  };
  
  
  class SkillParser
    : public Parser
  {
  public:
    SkillParser(Factory const & factory, std::ostream * optional_dbg_os = 0);
    
    std::string type;
    std::string name;
    
    /** After successfully parsing a YAML node, this contains the
	pointer to the freshly created skill. If something goes
	wrong, skill will be zero.
	
	\note You are responsible for eventually deleting this
	Skill instance.
    */
    Skill * skill;
  };
  
  
  class TaskTableParser
    : public Parser
  {
  public:
    TaskTableParser(Factory const & factory,
		    Factory::task_table_t & task_table,
		    std::ostream * optional_dbg_os = 0);
    
    TaskParser task_parser;    
    Factory::task_table_t & task_table;
  };
  
  
  class SkillTableParser
    : public Parser
  {
  public:
    SkillTableParser(Factory const & factory,
			Factory::skill_table_t & skill_table,
			std::ostream * optional_dbg_os = 0);
    
    SkillParser skill_parser;    
    Factory::skill_table_t & skill_table;
  };  
  
  
  void operator >> (YAML::Node const & node, Vector & vector);
  
  void operator >> (YAML::Node const & node, TaskParser & parser);
  
  void operator >> (YAML::Node const & node, SkillParser & parser);
  
  void operator >> (YAML::Node const & node, TaskTableParser & parser);
  
  void operator >> (YAML::Node const & node, SkillTableParser & parser);
  
}

#endif // OPSPACE_PARSE_YAML_HPP
