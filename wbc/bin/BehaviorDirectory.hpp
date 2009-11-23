/*
 * Copyright (c) 2009 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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

#ifndef WBC_BEHAVIOR_DIRECTORY_HPP
#define WBC_BEHAVIOR_DIRECTORY_HPP

#include <wbcrun/directory.hpp>

namespace wbc {
  
  class BehaviorDescription;
  class ServoImplementation;
  
  class BehaviorDirectory
    : public wbcrun::Directory
  {
  public:
    BehaviorDirectory(std::vector<BehaviorDescription*> const & behavior,
		      /** can be NULL, but then HandleServoCmd() will always fail */
		      ServoImplementation * servo);
  
    virtual wbcrun::srv::result_t
    HandleServoCmd(int requestID,
		   wbcrun::ServiceMessage::vector_type const * code_in,
		   wbcrun::ServiceMessage::matrix_type const * data_in,
		   wbcrun::ServiceMessage::vector_type * code_out,
		   wbcrun::ServiceMessage::matrix_type * data_out);
    
    virtual wbcrun::srv::result_t
    ListBehaviors(wbcrun::listing_t & behaviors) const;
  
    virtual wbcrun::srv::result_t
    ListBehaviorCmds(int behaviorID,
		     wbcrun::request_list_t & requests) const;
  
    virtual wbcrun::srv::result_t
    HandleBehaviorCmd(int behaviorID,
		      int requestID,
		      wbcrun::ServiceMessage::vector_type const * code_in,
		      wbcrun::ServiceMessage::matrix_type const * data_in,
		      wbcrun::ServiceMessage::vector_type * code_out,
		      wbcrun::ServiceMessage::matrix_type * data_out);
  
    virtual wbcrun::srv::result_t
    ListTasks(int behaviorID,
	      wbcrun::listing_t & tasks) const;
  
    virtual wbcrun::srv::result_t
    ListTaskCmds(int behaviorID,
		 int taskID,
		 wbcrun::request_list_t & requests) const;
  
    virtual wbcrun::srv::result_t
    HandleTaskCmd(int behaviorID,
		  int taskID,
		  int requestID,
		  wbcrun::ServiceMessage::vector_type const * code_in,
		  wbcrun::ServiceMessage::matrix_type const * data_in,
		  wbcrun::ServiceMessage::vector_type * code_out,
		  wbcrun::ServiceMessage::matrix_type * data_out);
  
  protected:
    std::vector<BehaviorDescription*> const & m_behavior;
    ServoImplementation * m_servo;
  };

}

#endif // WBC_BEHAVIOR_DIRECTORY_HPP
