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
 \file       CLotusTiXmlParser.cpp.h (stores all data obtained from a skeletal model)
 */
//=========================================================================
#ifndef CLOTUSTIXMLPARSER_H_
#define CLOTUSTIXMLPARSER_H_

#include <iostream>
#include <string>
#include <tinyxml/tinyxml.h>

//Include the data structures
#include <robarch/glob_rob_ds/SControllerRobotLink.hpp>
#include <robarch/glob_rob_ds/SGlobalRobotDS.hpp>

namespace lotusarchitect {

class CLotusTiXmlParser
{
private:

public:
  CLotusTiXmlParser() {}
  ~CLotusTiXmlParser() {}

  //Reads single links
  static bool readLink(const TiXmlHandle & arg_link_txml, robotarchitect::SControllerRobotLink* arg_link_ds, bool arg_is_root);
  static bool readGlobalData(const TiXmlHandle &arg_glob_data_txml, robotarchitect::SGlobalRobotDS* arg_glob_ds);
  
};

}
#endif /* CLOTUSTIXMLPARSER_H_ */
