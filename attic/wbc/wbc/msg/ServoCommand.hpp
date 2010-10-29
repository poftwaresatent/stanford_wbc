/*
 * Copyright (c) 2010 Stanford University
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

/**
   \file ServoCommand.hpp
   \author Roland Philippsen
*/

#ifndef WBC_MSG_SERVO_COMMAND_HPP
#define WBC_MSG_SERVO_COMMAND_HPP

#include <wbcnet/msg/ServoCommand.hpp>
#include <wbc/core/SAIVectorAPI.hpp>

namespace wbc {
  
  namespace msg {
    
    /**
       Instantiation of wbcnet::msg::ServoCommand compatible with both
       saimatrix and wbcnet.
    */
    class ServoCommand
      : public wbcnet::msg::ServoCommand<wbc::SAIVectorAPI>
    {
    public:
      inline ServoCommand(bool auto_resize, uint8_t ncommands)
	: wbcnet::msg::ServoCommand<wbc::SAIVectorAPI>(wbcnet::msg::SERVO_COMMAND, auto_resize, ncommands) {}
    };
    
  }
  
}

#endif // WBC_MSG_SERVO_COMMAND_HPP
