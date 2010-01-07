/*
 * Copyright (c) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "TaskAtomizer.hpp"
#include <wbcnet/proxy.hpp>
#include <wbcnet/msg/TaskMatrix.hpp>
#include <wbcnet/log.hpp>
#include <stdlib.h>
#include <limits>

#ifndef WIN32
#include <strings.h>
#else
#include "win32/win32_compat.hpp"
#endif

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));

using namespace std;


namespace wbcnet {


  TaskAtomizer::
  TaskAtomizer(endian_mode_t endian_mode)
    : m_endian_mode(endian_mode),
      m_requestID(-1),
      m_ngenmx(-1),
      m_genmx_mask(-1),
      m_genmx_flags(0),
      m_nsets(-1),
      m_set_mask(-1),
      m_set_flags(0),
      m_nsetmx(-1),
      m_setmx_mask(-1),
      m_setmx_flags(0),
      m_ttntasks(-1),
      m_task_mask(-1),
      m_task_flags(0),
      m_ntaskmx(-1),
      m_taskmx_mask(-1),
      m_taskmx_flags(0)
  {
    m_acquisition_time.tv_sec = 0;
    m_acquisition_time.tv_usec = 0;
  }
  
  
  TaskAtomizer::
  ~TaskAtomizer()
  {
    free(m_setmx_flags);
    free(m_taskmx_flags);
  }
  
  
  char const * TaskAtomizer::
  StatusString(status st)
  {
    static char const * str[] = {
      "OK",
      "ALLOC_ERROR",
      "INIT_ERROR",
      "REQUEST_MISMATCH",
      "SET_ID_MISMATCH",
      "TASK_ID_MISMATCH",
      "MATRIX_MISMATCH",
      "INSUFFICIENT_CAPACITY"
    };
    if ((0 > st) || (INSUFFICIENT_CAPACITY < st))
      return "(invalid status)";
    return str[st];
  }
  
  
  TaskAtomizer::status TaskAtomizer::
  Reset(uint8_t requestID,
	int ngenmx,
	int nsets,
	int nsetmx,
	int ttntasks,
	int ntaskmx)
  {
    if ((0 > ngenmx)
	|| (0 > nsets) || (0 > nsetmx)
	|| (0 > ttntasks) || (0 > ntaskmx))
      return INIT_ERROR;
    if ((capacity < ngenmx)
	|| (capacity < nsets) || (capacity < nsetmx)
	|| (capacity < ttntasks) || (capacity < ntaskmx))
      return INSUFFICIENT_CAPACITY;
    
    m_requestID = requestID;
    
    m_ngenmx = ngenmx;
    m_genmx_mask = ComputeMask(ngenmx);
    m_genmx_flags = 0;
    
    m_nsets = nsets;
    m_set_mask = ComputeMask(nsets);
    m_set_flags = 0;
    
    m_nsetmx = nsetmx;
    m_setmx_mask = ComputeMask(nsetmx);
    if (0 == m_setmx_flags) {
      m_setmx_flags = (bitmap_t *) malloc(capacity * sizeof(*m_setmx_flags));
      if (0 == m_setmx_flags)
	return ALLOC_ERROR;
    }
    bzero(m_setmx_flags, capacity * sizeof(*m_setmx_flags));
    
    m_ttntasks = ttntasks;
    m_task_mask = ComputeMask(ttntasks);
    m_task_flags = 0;
    
    m_ntaskmx = ntaskmx;
    m_taskmx_mask = ComputeMask(ntaskmx);
    if (0 == m_taskmx_flags) {
      m_taskmx_flags = (bitmap_t *) malloc(capacity * sizeof(*m_taskmx_flags));
      if (0 == m_taskmx_flags)
	return ALLOC_ERROR;
    }
    bzero(m_taskmx_flags, capacity * sizeof(*m_taskmx_flags));
    
    return OK;
  }
  
  
  TaskAtomizer::status TaskAtomizer::
  ProcessHeader(msg::TaskMatrixWrap const & tmwp)
  {
    if ((0 > m_ngenmx)
	|| (0 > m_nsets) || (0 > m_nsetmx)
	|| (0 > m_ttntasks) || (0 > m_ntaskmx)) {
      LOG_ERROR (logger,
		 "wbcnet::TaskAtomizer::ProcessHeader(): all these numbers should be >= 0\n"
		 << "  number of task-independent matrices (m_ngenmx):      " << m_ngenmx << "\n"
		 << "  number of task sets (m_nsets):                       " << m_nsets << "\n"
		 << "  number of set-dependent matrices per set (m_nsetmx): " << m_nsetmx << "\n"
		 << "  total number of tasks over all sets (m_ttntasks):    " << m_ttntasks << "\n"
		 << "  number of task-dependent matrices (m_ntaskmx):       " << m_ntaskmx);
      return INIT_ERROR;
    }
    
    if (m_requestID != tmwp.requestID) {
      LOG_ERROR (logger,
		     "wbcnet::TaskAtomizer::ProcessHeader(): request ID mismatch\n"
		     << "  expected: " << int(m_requestID) << "\n"
		     << "  received: " << int(tmwp.requestID));
      return REQUEST_MISMATCH;
    }
    
    if (tmwp.setID < -1) {
      LOG_ERROR (logger,
		 "wbcnet::TaskAtomizer::ProcessHeader(): invalid setID " << (int) tmwp.setID);
      return SET_ID_MISMATCH;
    }
    
    if (tmwp.taskID < -1) {
      LOG_ERROR (logger,
		 "wbcnet::TaskAtomizer::ProcessHeader(): invalid taskID " << (int) tmwp.taskID);
      return TASK_ID_MISMATCH;
    }
    
    if (-1 == tmwp.taskID) {
      if (-1 == tmwp.setID) { // either task-independent...
	if (tmwp.matrixID >= m_ngenmx) {
	  LOG_ERROR (logger,
		     "wbcnet::TaskAtomizer::ProcessHeader(): invalid task-independent matrixID "
		     << (int) tmwp.matrixID << " (only " << (int) m_ngenmx << " available)");
	  return MATRIX_MISMATCH;
	}
      }
      else { // ...or set-dependent...
	if (tmwp.setID >= m_nsets) {
	  LOG_ERROR (logger,
		     "wbcnet::TaskAtomizer::ProcessHeader(): invalid setID "
		     << (int) tmwp.setID << " (only " << (int) m_nsets << " available)");
	  return SET_ID_MISMATCH;
	}
	if (tmwp.matrixID >= m_nsetmx) {
	  LOG_ERROR (logger,
		     "wbcnet::TaskAtomizer::ProcessHeader(): invalid set-dependent matrixID "
		     << (int) tmwp.matrixID << " (only " << (int) m_nsetmx << " available)");
	  return MATRIX_MISMATCH;
	}
      }
    }
    else { // ...or task-dependent
      if (tmwp.taskID >= m_ttntasks) {
	LOG_ERROR (logger,
		   "wbcnet::TaskAtomizer::ProcessHeader(): invalid taskID "
		   << (int) tmwp.taskID << " (only " << (int) m_ttntasks << " available)");
	return TASK_ID_MISMATCH;
      }
      if (tmwp.matrixID >= m_ntaskmx) {
	LOG_ERROR (logger,
		   "wbcnet::TaskAtomizer::ProcessHeader(): invalid task-dependent matrixID "
		   << (int) tmwp.matrixID << " (only " << (int) m_ntaskmx << " available)");
	return MATRIX_MISMATCH;
      }
    }
    
    return OK;
  }
  
  
  TaskAtomizer::status TaskAtomizer::
  ProcessPayload(msg::TaskMatrixWrap const & tmwp)
  {
    m_acquisition_time = tmwp.acquisitionTime;
    if (-1 == tmwp.setID) {      
      // task-independent matrix message
      if (m_ngenmx <= tmwp.matrixID) {
	LOG_ERROR (logger,
		       "wbcnet::TaskAtomizer::ProcessPayload(): task-independent matrix mismatch\n"
		       << "  highest valid ID: " << int(m_ngenmx) << "\n"
		       << "  received ID:      " << int(tmwp.matrixID));
	return MATRIX_MISMATCH;
      }
      m_genmx_flags |= ComputeFlag(tmwp.matrixID);
    }
    else if (-1 == tmwp.taskID) {
      // level-independent matrix message
      if (m_nsetmx <= tmwp.matrixID) {
	LOG_ERROR (logger,
		       "wbcnet::TaskAtomizer::ProcessPayload(): level-independent matrix mismatch\n"
		       << "  highest valid ID: " << int(m_nsetmx) << "\n"
		       << "  received ID:      " << int(tmwp.matrixID));
	return MATRIX_MISMATCH;
      }
      m_setmx_flags[tmwp.setID] |= ComputeFlag(tmwp.matrixID);
      if (0 == (m_setmx_flags[tmwp.setID] ^ m_setmx_mask))
	m_set_flags |= ComputeFlag(tmwp.setID);
    }
    else {
      // task-dependent matrix message
      if (m_ntaskmx <= tmwp.matrixID) {
	LOG_ERROR (logger,
		       "wbcnet::TaskAtomizer::ProcessPayload(): task-dependent matrix mismatch\n"
		       << "  highest valid ID: " << int(m_ntaskmx) << "\n"
		       << "  received ID:      " << int(tmwp.matrixID));
	return MATRIX_MISMATCH;
      }
      m_taskmx_flags[tmwp.taskID] |= ComputeFlag(tmwp.matrixID);
      if (0 == (m_taskmx_flags[tmwp.taskID] ^ m_taskmx_mask))
	m_task_flags |= ComputeFlag(tmwp.taskID);
    }
    return OK;
  }
  
  
  bool TaskAtomizer::
  ModelsAreComplete() const
  {
    bool complete(true);
    if ((0 != m_ngenmx) && (0 != (m_genmx_flags ^ m_genmx_mask))) {
      complete = false;
    }
    if (complete && (0 != m_nsets) && (0 != (m_set_flags ^ m_set_mask))) {
      complete = false;
    }
    if (complete && (0 != m_ttntasks) && (0 != (m_task_flags ^ m_task_mask))) {
      complete = false;
    }
    return complete;
  }
  
  
  timeval const & TaskAtomizer::
  GetAcquisitionTime() const
  {
    return m_acquisition_time;
  }
  
  
  void TaskAtomizer::
  SetAcquisitionTime(timeval const & at)
  {
    m_acquisition_time = at;
  }
  
  
  TaskAtomizer::bitmap_t TaskAtomizer::
  ComputeMask(int number)
  {
    if (capacity == number)
      return ~ ((bitmap_t) 0);
    return (((bitmap_t) 1) << number) - 1;
  }
  
  
  TaskAtomizer::bitmap_t TaskAtomizer::
  ComputeFlag(int number)
  {
    return ((bitmap_t) 1) << number;
  }

}
