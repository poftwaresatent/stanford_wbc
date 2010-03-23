/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
   \file ShuffleVector.hpp
   \author Roland Philippsen
   \note Originally Copyright (c) 2008 Roland Philippsen, released under a BSD license.
*/

#ifndef WBC_TASK_MODEL_LISTENER_HPP
#define WBC_TASK_MODEL_LISTENER_HPP

#include <wbcnet/Muldex.hpp>
#include <wbcnet/msg/TaskMatrix.hpp>

#ifdef WIN32
typedef long ssize_t;
#endif

namespace wbc {
  
  class TaskModelBase;
  
  class TaskModelListener
    : public wbcnet::MdxListener
  {
  public:
    TaskModelListener(wbc::TaskModelBase * model_one,
		      wbc::TaskModelBase * model_two);
    
    virtual int HandleMessageHeader(wbcnet::unique_id_t msg_id);
    virtual int HandleMessagePayload(wbcnet::unique_id_t msg_id);
    
    wbc::TaskModelBase * GetLastUpdatedModel();
    wbc::TaskModelBase * GetStaleModel();
    
    wbcnet::msg::TaskMatrixWrap task_matrix;
    
  protected:
    std::vector<wbc::TaskModelBase*> m_model_pool;
    ssize_t m_last_idx;
    size_t m_stale_idx;
    std::string m_matrix_name;
  };
  
}

#endif // WBC_TASK_MODEL_LISTENER_HPP
