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

#ifndef WBC_DIRECTORY_CMD_SERVER_HPP
#define WBC_DIRECTORY_CMD_SERVER_HPP

#include <wbcrun/directory.hpp>

namespace wbc {
  
  class BehaviorDescription;
  class ServoProcessAPI;
  
  
  class DirectoryCmdServer
    : public wbcrun::DirectoryCmdServer
  {
  public:
    DirectoryCmdServer(std::vector<BehaviorDescription*> const & behavior,
		       /** can be NULL, but then several things will
			   always fail and be flagged
			   SRV_OTHER_ERROR+1 */
		       ServoProcessAPI * servo);
    
    virtual wbcnet::srv_result_t
    HandleServoCmd(int commandID,
		   wbcnet::msg::Service::vector_type const * code_in,
		   wbcnet::msg::Service::matrix_type const * data_in,
		   wbcnet::msg::Service::vector_type * code_out,
		   wbcnet::msg::Service::matrix_type * data_out);
    
    virtual wbcnet::srv_result_t
    ListBehaviors(wbcrun::listing_t & behaviors) const;
  
    virtual wbcnet::srv_result_t
    ListBehaviorCmds(int behaviorID,
		     wbcrun::command_list_t & commands) const;
  
    virtual wbcnet::srv_result_t
    HandleBehaviorCmd(int behaviorID,
		      int commandID,
		      wbcnet::msg::Service::vector_type const * code_in,
		      wbcnet::msg::Service::matrix_type const * data_in,
		      wbcnet::msg::Service::vector_type * code_out,
		      wbcnet::msg::Service::matrix_type * data_out);
  
    virtual wbcnet::srv_result_t
    ListTasks(int behaviorID,
	      wbcrun::listing_t & tasks) const;
  
    virtual wbcnet::srv_result_t
    ListTaskCmds(int behaviorID,
		 int taskID,
		 wbcrun::command_list_t & commands) const;
  
    virtual wbcnet::srv_result_t
    HandleTaskCmd(int behaviorID,
		  int taskID,
		  int commandID,
		  wbcnet::msg::Service::vector_type const * code_in,
		  wbcnet::msg::Service::matrix_type const * data_in,
		  wbcnet::msg::Service::vector_type * code_out,
		  wbcnet::msg::Service::matrix_type * data_out);
  
  protected:
    std::vector<BehaviorDescription*> const & m_behavior;
    ServoProcessAPI * m_servo;
  };

}

#endif // WBC_DIRECTORY_CMD_SERVER_HPP
