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
   \file TaskSpec.hpp
   \author Roland Philippsen
*/

#ifndef WBC_MSG_TASK_SPEC_HPP
#define WBC_MSG_TASK_SPEC_HPP

#include <wbcnet/msg/TaskSpec.hpp>
#include <wbcnet/message_id.hpp>

namespace wbc {
  
  namespace msg {
    
    /**
       Instantiation of wbcnet::msg::TaskSpec that simply adds a
       hardcoded message ID.
    */
    class TaskSpec : public wbcnet::msg::TaskSpec {
    public:
      TaskSpec() : wbcnet::msg::TaskSpec(wbcnet::msg::TASK_SPEC) {}
    };

  }
  
}

#endif // WBC_MSG_TASK_SPEC_HPP
