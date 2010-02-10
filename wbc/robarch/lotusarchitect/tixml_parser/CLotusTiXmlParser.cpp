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
 \file       CLotusTiXmlParser.cpp.cpp (processes robots obtained from a lotus model)
 */
//=========================================================================

#include <wbc/robarch/lotusarchitect/tixml_parser/CLotusTiXmlParser.hpp>

//The general header (contains the logger etc.)
#include <wbc/robarch/lotusarchitect/LotusHeaders.hpp>

#include <sstream>

namespace lotusarchitect {

using namespace robotarchitect;

bool CLotusTiXmlParser::readLink(const TiXmlHandle & arg_link_txml, SControllerRobotLink* arg_link_ds, bool arg_is_root)
{
  bool flag = true;
  //Read in the information into arg_link_ds
  TiXmlElement* link_data;

  //Link name
  link_data = arg_link_txml.FirstChildElement( "link_name" ).Element();
  if ( link_data )
  {
    std::stringstream ss(link_data->FirstChild()->Value());
    ss>>arg_link_ds->link_name_;
  }
  else  {LOG_FATAL(logger, "CLotusTiXmlParser::readLink() : Error reading link name"); flag = false; goto END;}
  
  LOG_DEBUG(logger, arg_link_ds->linkName_ );//Remaining messages are for this link name.

  //Pos in parent.
  link_data = arg_link_txml.FirstChildElement( "position_in_parent" ).Element();
  if ( link_data )
  {
    std::stringstream ss(link_data->FirstChild()->Value());
    ss>>arg_link_ds->pos_in_parent_[0];
    ss>>arg_link_ds->pos_in_parent_[1];
    ss>>arg_link_ds->pos_in_parent_[2];
  }
  else  {LOG_FATAL(logger, "CLotusTiXmlParser::readLink() : Error reading pos in parent"); flag = false; goto END;}

  //Orientation in parent.
  link_data = arg_link_txml.FirstChildElement( "orientation_in_parent" ).Element();
  if ( link_data )
  {
    std::stringstream ss(link_data->FirstChild()->Value());
    ss>>arg_link_ds->ori_in_parent_[0];
    ss>>arg_link_ds->ori_in_parent_[1];
    ss>>arg_link_ds->ori_in_parent_[2];
    ss>>arg_link_ds->ori_in_parent_[3];
  }
  else  {LOG_FATAL(logger, "CLotusTiXmlParser::readLink() : Error reading orientation in parent"); flag = false; goto END;}

  //******************************** Child link specific stuff ******************************
  if(!arg_is_root)
  { 
    //Mass
    link_data = arg_link_txml.FirstChildElement( "mass" ).Element();
    if ( link_data )
    {
      std::stringstream ss(link_data->FirstChild()->Value());
      ss>>arg_link_ds->mass_;
    }
    else  {LOG_FATAL(logger, "CLotusTiXmlParser::readLink() : Error reading mass of link"); flag = false; goto END;}

    //Inertia
    link_data = arg_link_txml.FirstChildElement( "inertia" ).Element();
    if ( link_data )
    {
      std::stringstream ss(link_data->FirstChild()->Value());
      ss>>arg_link_ds->inertia_[0];
      ss>>arg_link_ds->inertia_[1];
      ss>>arg_link_ds->inertia_[2];
    }
    else  {LOG_FATAL(logger, "CLotusTiXmlParser::readLink() : Error reading inertia"); flag = false; goto END;}

    //Com
    link_data = arg_link_txml.FirstChildElement( "center_of_mass" ).Element();
    if ( link_data )
    {
      std::stringstream ss(link_data->FirstChild()->Value());
      ss>>arg_link_ds->com_[0];
      ss>>arg_link_ds->com_[1];
      ss>>arg_link_ds->com_[2];
    }
    else  {LOG_FATAL(logger, "CLotusTiXmlParser::readLink() : Error reading com"); flag = false; goto END;}

    //******************************** Joint specific stuff ******************************
    //Joint name
    link_data = arg_link_txml.FirstChildElement( "joint_name" ).Element();
    if ( link_data )
    {
      std::stringstream ss(link_data->FirstChild()->Value());
      ss>>arg_link_ds->joint_name_;
    }
    else  {LOG_DEBUG(logger, "CLotusTiXmlParser::readLink() : Error reading joint name");}

    //Parent link name
    link_data = arg_link_txml.FirstChildElement( "parent_link_name" ).Element();
    if ( link_data )
    {
      std::stringstream ss(link_data->FirstChild()->Value());
      ss>>arg_link_ds->parent_link_name_;
    }
    else  {LOG_DEBUG(logger, "CLotusTiXmlParser::readLink() : Error reading parent link name");}

    //Lower Joint Limit
    link_data = arg_link_txml.FirstChildElement( "lower_joint_limit" ).Element();
    if ( link_data )
    {
      std::stringstream ss(link_data->FirstChild()->Value());
      ss>>arg_link_ds->joint_limit_lower_;
    }
    else  {LOG_DEBUG(logger, "CLotusTiXmlParser::readLink() : Error reading lower joint limit");}

    //Upper Joint Limit
    link_data = arg_link_txml.FirstChildElement( "upper_joint_limit" ).Element();
    if ( link_data )
    {
      std::stringstream ss(link_data->FirstChild()->Value());
      ss>>arg_link_ds->joint_limit_upper_;
    }
    else  {LOG_DEBUG(logger, "CLotusTiXmlParser::readLink() : Error reading upper joint limit");}

    //Default Joint Pos
    link_data = arg_link_txml.FirstChildElement( "default_joint_position" ).Element();
    if ( link_data )
    {
      std::stringstream ss(link_data->FirstChild()->Value());
      ss>>arg_link_ds->joint_default_pos_;
    }
    else  {LOG_DEBUG(logger, "CLotusTiXmlParser::readLink() : Error reading default joint position");}

    //Joint type
    link_data = arg_link_txml.FirstChildElement( "joint_type" ).Element();
    if ( link_data )
    {
      std::stringstream ss(link_data->FirstChild()->Value());
      int jtype;
      ss>>jtype;
      switch(jtype)
      {
      case 0: arg_link_ds->joint_type_ = JT_PRISMATIC; break;
      case 1: arg_link_ds->joint_type_ = JT_REVOLUTE; break;
      case 2: arg_link_ds->joint_type_ = JT_SPHERICAL; break;
      case 3: arg_link_ds->joint_type_ = JT_MAX; break;
      default: arg_link_ds->joint_type_ = JT_NOTASSIGNED; break;
      }      
    }
    else  {LOG_FATAL(logger, "CLotusTiXmlParser::readLink() : Error reading joint axis"); flag = false; goto END;}

    //NOTE TODO : If there is any special processing for spherical joints, it will take place here

    //Joint axis
    link_data = arg_link_txml.FirstChildElement( "axis" ).Element();
    if ( link_data )
    {
      std::stringstream ss(link_data->FirstChild()->Value());
      ss>>arg_link_ds->joint_axis_;
    }
    else  {LOG_FATAL(logger, "CLotusTiXmlParser::readLink() : Error reading joint axis"); flag = false; goto END;}

    //************** END OF Joint specific stuff
  } //************** END OF Child link specific stuff


  //******************************** Graphics Stuff ******************************
  //Graphics : obj file
  link_data = arg_link_txml.FirstChildElement( "graphics" ).FirstChildElement("obj_file").Element();
  if ( !link_data ) {LOG_WARN(logger, "CLotusTiXmlParser::readLink() : No obj files found");}
  for( link_data; link_data; link_data=link_data->NextSiblingElement("obj_file") )
  {
    std::string ss = link_data->FirstChild()->Value();
    arg_link_ds->obj_file_vec_.push_back(ss);
  }
    
  //Graphics : collision type
  link_data = arg_link_txml.FirstChildElement( "graphics" ).FirstChildElement("collision_type").Element();
  if ( link_data )
  {
    std::stringstream ss(link_data->FirstChild()->Value());
    ss>>arg_link_ds->collision_type_;
  }
  else  {LOG_DEBUG(logger, "CLotusTiXmlParser::readLink() : Error reading collision type");}

END:
  return flag;
}

bool CLotusTiXmlParser::readGlobalData(const TiXmlHandle & arg_glob_data_txml, SGlobalRobotDS* arg_glob_ds)
{
  bool flag = true;
  //Read in the information into arg_glob_ds
  TiXmlElement* glob_data;
  
  //Gravity
  glob_data = arg_glob_data_txml.FirstChildElement( "gravity" ).Element();
  if ( glob_data )
  {
    std::stringstream ss(glob_data->FirstChild()->Value());
    ss>>arg_glob_ds->gravity_[0];
    ss>>arg_glob_ds->gravity_[1];
    ss>>arg_glob_ds->gravity_[2];
  }
  else  {LOG_FATAL(logger, "CLotusTiXmlParser::readGlobalData() : Error reading gravity"); flag = false; goto END;}

  //Camera orientation and position
  glob_data = arg_glob_data_txml.FirstChildElement( "camera" ).FirstChildElement( "pos" ).Element();
  if ( glob_data )
  {
    std::stringstream ss(glob_data->FirstChild()->Value());
    ss>>arg_glob_ds->camera_.pos_[0];
    ss>>arg_glob_ds->camera_.pos_[1];
    ss>>arg_glob_ds->camera_.pos_[2];
  }
  else  {LOG_DEBUG(logger, "CLotusTiXmlParser::readGlobalData() : Error reading camera pos");}

  glob_data = arg_glob_data_txml.FirstChildElement( "camera" ).FirstChildElement( "lookat" ).Element();
  if ( glob_data )
  {
    std::stringstream ss(glob_data->FirstChild()->Value());
    ss>>arg_glob_ds->camera_.lookat_[0];
    ss>>arg_glob_ds->camera_.lookat_[1];
    ss>>arg_glob_ds->camera_.lookat_[2];
  }
  else  {LOG_DEBUG(logger, "CLotusTiXmlParser::readGlobalData() : Error reading camera lookat");}

  glob_data = arg_glob_data_txml.FirstChildElement( "camera" ).FirstChildElement( "up" ).Element();
  if ( glob_data )
  {
    std::stringstream ss(glob_data->FirstChild()->Value());
    ss>>arg_glob_ds->camera_.up_[0];
    ss>>arg_glob_ds->camera_.up_[1];
    ss>>arg_glob_ds->camera_.up_[2];
  }
  else  {LOG_DEBUG(logger, "CLotusTiXmlParser::readGlobalData() : Error reading camera up");}

  glob_data = arg_glob_data_txml.FirstChildElement( "camera" ).FirstChildElement( "name" ).Element();
  if ( glob_data )
  {
    std::stringstream ss(glob_data->FirstChild()->Value());
    ss>>arg_glob_ds->camera_.name_;
  }
  else  {LOG_DEBUG(logger, "CLotusTiXmlParser::readGlobalData() : Error reading camera name");}

END:
  return flag;
}

}
