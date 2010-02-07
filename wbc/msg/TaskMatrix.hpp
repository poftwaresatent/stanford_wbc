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
   \file TaskMatrix.hpp
   \author Roland Philippsen
*/

#ifndef WBC_MSG_TASK_MATRIX_HPP
#define WBC_MSG_TASK_MATRIX_HPP

#include <wbcnet/msg/TaskMatrix.hpp>
#include <wbcnet/message_id.hpp>
#include <limits>

namespace wbc {
  
  namespace msg {
    
    /**
       Instantiation of wbcnet::msg::TaskMatrixWrap that specifies
       maximum possible matrix dimensions and hardcodes the message
       ID.
    */
    class TaskMatrix : public wbcnet::msg::TaskMatrixWrap
    {
    public:
      static uint8_t maxNRowsCols()
      { return std::numeric_limits<uint8_t>::max(); }
      
      TaskMatrix()
	: wbcnet::msg::TaskMatrixWrap(wbcnet::msg::TASK_MATRIX, maxNRowsCols(), maxNRowsCols(), 0) {}
    };
    
  }
  
}

#endif // WBC_MSG_TASK_MATRIX_HPP
