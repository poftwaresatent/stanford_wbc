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

/**
   \file TestDirectory.hpp
   \author Roland Philippsen
   \note Originally Copyright (c) 2009 Roland Philippsen, released under a BSD license.
*/

#ifndef WBC_TEST_DIRECTORY_HPP
#define WBC_TEST_DIRECTORY_HPP

#include <wbc/bin/directory.hpp>

namespace wbc {
  
  class TestDirectory
    : public Directory
  {
  public:
    virtual wbcnet::srv_result_t HandleServoCmd(int commandID,
						wbcnet::srv_code_t const * code_in,
						wbcnet::srv_matrix_t const * data_in,
						listing_t const & str_in,
						wbcnet::srv_code_t * code_out,
						wbcnet::srv_matrix_t * data_out,
						listing_t & str_out);
    
    virtual wbcnet::srv_result_t ListBehaviors(listing_t & behaviors) const;
    virtual wbcnet::srv_result_t ListBehaviorCmds(int behaviorID, command_list_t & commands) const;
    
    virtual wbcnet::srv_result_t HandleBehaviorCmd(int behaviorID,
						   int commandID,
						   wbcnet::srv_code_t const * code_in,
						   wbcnet::srv_matrix_t const * data_in,
						   wbcnet::srv_code_t * code_out,
						   wbcnet::srv_matrix_t * data_out);
    
    virtual wbcnet::srv_result_t ListTasks(int behaviorID, listing_t & tasks) const;
    virtual wbcnet::srv_result_t ListTaskCmds(int behaviorID, int taskID, command_list_t & commands) const;
    
    virtual wbcnet::srv_result_t HandleTaskCmd(int behaviorID,
					       int taskID,
					       int commandID,
					       wbcnet::srv_code_t const * code_in,
					       wbcnet::srv_matrix_t const * data_in,
					       wbcnet::srv_code_t * code_out,
					       wbcnet::srv_matrix_t * data_out);
  };
  
}

#endif // WBC_TEST_DIRECTORY_HPP
