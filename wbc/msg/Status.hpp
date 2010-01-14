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
   \file Status.hpp
   \author Roland Philippsen
*/

#ifndef WBC_MSG_STATUS_HPP
#define WBC_MSG_STATUS_HPP

#include <wbcnet/msg/Status.hpp>
#include <wbcnet/message_id.hpp>

namespace wbc {

  namespace msg {
    
    typedef enum {
      VOID_STATUS,
      COMPUTE_MODEL,		// servo --> model: please compute matrices
      MODEL_ERROR,		// model --> servo: something broke, try again
      MODEL_SUCCESS		// model --> servo: sent all updated matrices
    } status_id_t;
    
    class Status : public wbcnet::msg::Status {
    public:
      Status() : wbcnet::msg::Status(wbcnet::msg::STATUS) { status = VOID_STATUS; }
    };
    
  }
  
}

#endif // WBC_MSG_STATUS_HPP
