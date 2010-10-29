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
   \file DirectoryCmdServer.hpp
   \author Roland Philippsen
*/

#ifndef WBC_DIRECTORY_CMD_SERVER_HPP
#define WBC_DIRECTORY_CMD_SERVER_HPP

#include <wbc/bin/directory.hpp>

namespace wbc {
  
  class BehaviorDescription;
  class ServoProcessAPI;
  
  
  class DirectoryCmdServer
    : public BaseDirectoryCmdServer
  {
  public:
    DirectoryCmdServer(std::vector<BehaviorDescription*> const & behavior,
		       /** can be NULL, but then several things will
			   always fail and be flagged
			   SRV_OTHER_ERROR+1 */
		       ServoProcessAPI * servo);
    
    virtual wbcnet::srv_result_t
    HandleServoCmd(int commandID,
		   wbcnet::srv_code_t const * code_in,
		   wbcnet::srv_matrix_t const * data_in,
		   listing_t const & str_in,
		   wbcnet::srv_code_t * code_out,
		   wbcnet::srv_matrix_t * data_out,
		   listing_t & str_out);
    
    virtual wbcnet::srv_result_t
    ListBehaviors(listing_t & behaviors) const;
  
    virtual wbcnet::srv_result_t
    ListBehaviorCmds(int behaviorID,
		     command_list_t & commands) const;
  
    virtual wbcnet::srv_result_t
    HandleBehaviorCmd(int behaviorID,
		      int commandID,
		      wbcnet::srv_code_t const * code_in,
		      wbcnet::srv_matrix_t const * data_in,
		      wbcnet::srv_code_t * code_out,
		      wbcnet::srv_matrix_t * data_out);
  
    virtual wbcnet::srv_result_t
    ListTasks(int behaviorID,
	      listing_t & tasks) const;
  
    virtual wbcnet::srv_result_t
    ListTaskCmds(int behaviorID,
		 int taskID,
		 command_list_t & commands) const;
  
    virtual wbcnet::srv_result_t
    HandleTaskCmd(int behaviorID,
		  int taskID,
		  int commandID,
		  wbcnet::srv_code_t const * code_in,
		  wbcnet::srv_matrix_t const * data_in,
		  wbcnet::srv_code_t * code_out,
		  wbcnet::srv_matrix_t * data_out);
  
  protected:
    std::vector<BehaviorDescription*> const & m_behavior;
    ServoProcessAPI * m_servo;
  };

}

#endif // WBC_DIRECTORY_CMD_SERVER_HPP
