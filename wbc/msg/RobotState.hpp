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
   \file RobotState.hpp
   \author Roland Philippsen
*/

#ifndef WBC_MSG_ROBOT_STATE_HPP
#define WBC_MSG_ROBOT_STATE_HPP

#include <wbcnet/msg/RobotState.hpp>
#include <wbc/core/SAIVectorAPI.hpp>
#include <wbc/core/SAIMatrixAPI.hpp>

// We really should find a better way than having a global header for
// this... maybe a checksum on the names of registered header and
// payload fields?
#include <wbcnet/message_id.hpp>

namespace wbc {
  
  namespace msg {
    
    class RobotState
      : public wbcnet::msg::RobotState<wbc::SAIVectorAPI,
				       wbc::SAIMatrixAPI>
    {
    public:
      inline RobotState(bool auto_resize,
			uint8_t npos,
			uint8_t nvel,
			uint8_t forces_nrows,
			uint8_t forces_ncolumns)
	: wbcnet::msg::RobotState<wbc::SAIVectorAPI,
				  wbc::SAIMatrixAPI>(wbcnet::msg::ROBOT_STATE,
						     auto_resize, npos, nvel,
						     forces_nrows, forces_ncolumns) {}
    };
    
  }
  
}

#endif // WBC_MSG_ROBOT_STATE_HPP
