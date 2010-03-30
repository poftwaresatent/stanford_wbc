/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2009 Stanford University. All rights reserved.
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
//=========================================================================
/*!
 \author     Samir Menon
 \file       CLotusArchitect.cpp (opensim file parser)
 */
//=========================================================================
#ifdef TESTING_FUNCTIONS_ON
#include <iostream>
#endif 
#define STUPID_GOTO_ERROR

#include "CLotusArchitect.hpp"

#include <wbc_tinyxml/wbc_tinyxml.h>

//The general header (contains the logger etc.)
#include <robarch/lotusarchitect/LotusHeaders.hpp>
//And the tinyxml parser for lotus xml files
#include <robarch/lotusarchitect/tixml_parser/CLotusTiXmlParser.hpp>

namespace lotusarchitect {
using namespace wbc_tinyxml;
using namespace robotarchitect;

bool CLotusArchitect::readRobotDefinition(const string arg_file)
{
  TiXmlElement* _robot_element;
  TiXmlHandle _global_settings(NULL), _file_handle(NULL), _world(NULL);

  robot_initialized_ = true;
  TiXmlDocument _file(arg_file.c_str());
  
  //Check if file opened properly
  robot_initialized_ = _file.LoadFile();
  if(true!=robot_initialized_)
  {
    LOG_FATAL(logger, "CLotusArchitect::readRobotDefinition() Fatal Error. Could not open xml file to read controller definition.");
    goto END;
  }
  else
  {
    LOG_DEBUG(logger, "Reading controller robot definition from file:");
    LOG_DEBUG(logger, arg_file);
  }

  printf("\n\nNow parsing the file.... 0% done\n");

  robdef_ = new CRobotDefinition<SControllerRobotLink>(true); //true deferences all pointers when this is destroyed  
  
  //Get handles to the tinyxml loaded ds
  _file_handle = TiXmlHandle( &_file );
  _world = _file_handle.FirstChildElement( "lotus_world" );

  //1. Read the global data
  _global_settings = _world.FirstChildElement( "globals" );
  robot_initialized_ = CLotusTiXmlParser::readGlobalData(_global_settings, robdef_->getGlobData());
  if(false == robot_initialized_) goto END ;

  //2. Read in the links.
  _robot_element = _world.FirstChildElement( "robot" ).ToElement();
  //Iterating with TiXmlElement is faster than TiXmlHandle
	for( _robot_element; _robot_element; _robot_element=_robot_element->NextSiblingElement("robot") )
	{
    TiXmlHandle _robot_handle(_robot_element); //Back to handles

    //Read robot name
    std::string robot_name;
    robot_name = _robot_handle.FirstChildElement("robot_name").ToElement()->FirstChild()->Value();

    //Parse the root node		
    SControllerRobotLink *tmp_link_ds = new SControllerRobotLink();
    tmp_link_ds->robot_name_ = robot_name;
    robot_initialized_ = CLotusTiXmlParser::readLink(_robot_handle.FirstChildElement("root_link"), tmp_link_ds, true);
    if(false == robot_initialized_) goto END ; 
    //Add the root node to the robdef
    robdef_->addLink(*tmp_link_ds, true, tmp_link_ds->link_name_);
    delete tmp_link_ds;

    //Now parse the child nodes
    TiXmlElement* _child_link_element = _robot_handle.FirstChildElement( "link" ).ToElement();
    for( _child_link_element; _child_link_element; _child_link_element=_child_link_element->NextSiblingElement("link") )
	  {
      TiXmlHandle _child_link_handle(_child_link_element); //Back to handles
      tmp_link_ds = new SControllerRobotLink();
      tmp_link_ds->robot_name_ = robot_name;
      robot_initialized_ = CLotusTiXmlParser::readLink(_child_link_handle, tmp_link_ds, false);
      if(false == robot_initialized_) goto END ;
      //Add the root node to the robdef
      robdef_->addLink(*tmp_link_ds, false, tmp_link_ds->link_name_);
      delete tmp_link_ds;
    }
	}//End of loop over robots in the xml file.

  //3. Build the robots
  robot_initialized_ = buildRobotsFromLinks();
  
END:
  if(robot_initialized_ == false) 
  {//Incorrect initialization. Abort.
    delete robdef_; 
    robdef_ = NULL;
  }
  return robot_initialized_;
}

}
