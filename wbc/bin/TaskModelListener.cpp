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
   \file TaskModelListener.cpp
   \author Roland Philippsen
*/

#include "TaskModelListener.hpp"
#include <wbcnet/misc/message_id.hpp>
#include <wbc/core/TaskModelBase.hpp>
#include <wbcnet/log.hpp>
#include <iostream>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));


namespace wbc {
  
  TaskModelListener::
  TaskModelListener(TaskModelBase * model_one,
		    TaskModelBase * model_two)
    : task_matrix(wbcnet::msg::TASK_MATRIX,
		  std::numeric_limits<uint8_t>::max(), // max n rows
		  std::numeric_limits<uint8_t>::max(), // max n cols
		  0		// data pointer
		  ),
      m_last_idx(-1),
      m_stale_idx(0),
      m_matrix_name("(no matrix selected)")
  {
    m_model_pool.push_back(model_one);
    m_model_pool.push_back(model_two);
  }
  
  
  int TaskModelListener::
  HandleMessageHeader(wbcnet::unique_id_t msg_id)
  {
    if (wbcnet::msg::TASK_MATRIX != msg_id) {
      LOG_ERROR (logger,
		     "wbc::TaskModelListener::HandleMessageHeader(): msg_id " << (int) msg_id
		     << " is not wbcnet::msg::TASK_MATRIX (" << wbcnet::msg::TASK_MATRIX << ")");
      return -11;
    }
  
    for (size_t ii(0); ii < m_model_pool.size(); ++ii) {
      TaskModelBase * model(m_model_pool[ii]);
      wbcnet::TaskAtomizer::status const st(model->ProcessHeader(task_matrix));
      if (wbcnet::TaskAtomizer::REQUEST_MISMATCH == st)
	continue; // this model does not match the request ID: try the next one
      if (wbcnet::TaskAtomizer::INIT_ERROR == st)
	continue; // this model was not yet initialized: try the next one
    
      if (wbcnet::TaskAtomizer::OK != st) {
	LOG_ERROR (logger,
		       "wbc::TaskModelListener::HandleMessageHeader()\n"
		       << "  ProcessHeader() failed\n"
		       << "  model pool index: " << ii << "\n"
		       << "  reason: " << wbcnet::TaskAtomizer::StatusString(st));
	return -22;
      }
    
      if (logger->isTraceEnabled()) {
	if ( ! task_matrix.dataptr) {
	  LOG_TRACE (logger,
			 "wbc::TaskModelListener::HandleMessageHeader(): before GetMatrix(): "
			 << m_matrix_name << " (null)");
	}
	else {
	  LOG_TRACE (logger,
			 "wbc::TaskModelListener::HandleMessageHeader(): before GetMatrix(): "
			 << m_matrix_name << " " << task_matrix.dataptr->NRows()
			 << "x" << task_matrix.dataptr->NColumns()
			 << " " << (void*) task_matrix.dataptr);
	}
      }
      
      task_matrix.dataptr =
	model->GetMatrix(task_matrix.setID, task_matrix.taskID, task_matrix.matrixID);
      m_matrix_name = model->GetName(task_matrix.setID, task_matrix.taskID, task_matrix.matrixID);
      if ( ! task_matrix.dataptr) {
	LOG_ERROR (logger,
		       "wbc::TaskModelListener::HandleMessageHeader():\n"
		       << "  GetMatrix() failed\n"
		       << "  model pool index: " << ii << "\n"
		       << "  setID: " << task_matrix.setID << "\n"
		       << "  taskID: " << task_matrix.taskID << "\n"
		       << "  matrix ID: " << task_matrix.matrixID << "\n"
		       << "  matrix name: " << m_matrix_name);
	return -33;
      }
      
      LOG_TRACE (logger,
		     "wbc::TaskModelListener::HandleMessageHeader(): after  GetMatrix(): "
		     << m_matrix_name << " " << task_matrix.dataptr->NRows()
		     << "x" << task_matrix.dataptr->NColumns()
		     << " " << (void*) task_matrix.dataptr);
      
      // found matching model and selected the right one of its matrices
      m_last_idx = ii;
      if (ii == 0)
	m_stale_idx = 1;
      else
	m_stale_idx = 0;
      return 0;
    }
    
    LOG_ERROR (logger,
		   "wbc::TaskModelListener::HandleMessageHeader()\n"
		   << "  no matching model found\n"
		   << "  request ID: " << (int) task_matrix.requestID << "\n"
		   << "  set ID: " << (int) task_matrix.setID << "\n"
		   << "  taskID: " << (int) task_matrix.taskID << "\n"
		   << "  matrix ID: " << (int) task_matrix.matrixID);
    
    return -44;
  }


  int TaskModelListener::
  HandleMessagePayload(wbcnet::unique_id_t msg_id)
  {
    if (wbcnet::msg::TASK_MATRIX != msg_id) {
      LOG_ERROR (logger,
		     "wbc::TaskModelListener::HandleMessagePayload(): msg_id " << (int) msg_id
		     << " is not wbcnet::msg::TASK_MATRIX (" << wbcnet::msg::TASK_MATRIX << ")");
      return -55;
    }
    
    LOG_TRACE (logger,
		   "wbc::TaskModelListener::HandleMessagePayload(): before ProcessPayload(): "
		   << m_matrix_name << " " << task_matrix.dataptr->NRows()
		   << "x" << task_matrix.dataptr->NColumns()
		   << " " << (void*) task_matrix.dataptr);
    
    TaskModelBase * task_model(GetLastUpdatedModel());
    if ( ! task_model) {
      LOG_ERROR (logger,
		     "wbc::TaskModelListener::HandleMessagePayload(): no 'last updated' model\n"
		     << "  BUG? ... should get set in HandleMessageHeader()");
      return -66;
    }
    
    wbcnet::TaskAtomizer::status const st(task_model->ProcessPayload(task_matrix));
    if (wbcnet::TaskAtomizer::OK != st) {
      LOG_ERROR (logger,
		     "wbc::TaskModelListener::HandleMessageHeader(): ProcessPayload() failed\n"
		     << "  reason: " << wbcnet::TaskAtomizer::StatusString(st));
      return -77;
    }
    
    LOG_TRACE (logger,
		   "wbc::TaskModelListener::HandleMessagePayload(): after  ProcessPayload(): "
		   << m_matrix_name << " " << task_matrix.dataptr->NRows()
		   << "x" << task_matrix.dataptr->NColumns()
		   << " " << (void*) task_matrix.dataptr);
    
    return 0;
  }
  
  
  TaskModelBase * TaskModelListener::
  GetLastUpdatedModel()
  {
    if (m_last_idx < 0)
      return 0;
    return m_model_pool[m_last_idx];
  }
  
  
  TaskModelBase * TaskModelListener::
  GetStaleModel()
  {
    return m_model_pool[m_stale_idx];
  }

}
