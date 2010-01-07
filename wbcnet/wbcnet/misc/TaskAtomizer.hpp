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

/** \file TaskAtomizer.hpp Declares class TaskAtomizer. */

#ifndef WBCNET_TASK_ATOMIZER_HPP
#define WBCNET_TASK_ATOMIZER_HPP

#include <wbcnet/endian_mode.hpp>

#ifdef WIN32
# include "win32_compat.hpp"
#else
# include <sys/time.h>
# include <stdint.h>
#endif

namespace wbcnet {

  namespace msg {    
    class TaskMatrixWrap;
  }
  
  /**
     Manage updates of the task model to make them appear atomic from
     the outside. As task model messages are sent in several chunks
     (one per matrix, of which there are three in each task), this
     class performs some housekeeping on behalf of the servo process,
     such that it is easier to determine at what instant a behavioral
     state transition has finished.
  */
  class TaskAtomizer
  {
    typedef unsigned long bitmap_t;
    
  public:
    typedef enum {
      OK,
      ALLOC_ERROR,
      INIT_ERROR,
      REQUEST_MISMATCH,
      SET_ID_MISMATCH,
      TASK_ID_MISMATCH,
      MATRIX_MISMATCH,
      INSUFFICIENT_CAPACITY
    } status;
    
    static int const capacity = 8 * sizeof(bitmap_t);
    
    explicit TaskAtomizer(/** if networking, use ENDIAN_DETECT,
			      otherwise ENDIAN_NEVER_SWAP to avoid
			      swapping when staying within the same
			      computer */
			  endian_mode_t endian_mode);
    virtual ~TaskAtomizer();
    
    static char const * StatusString(status st);

    status Reset(/** identifier of this transition request */
		 uint8_t requestID,
		 /** number of task-independent matrices (mapped to set ID -1) */
		 int ngenmx,
		 /** number of task sets */
		 int nsets,
		 /** number of per-task-set matrices (mapped to task ID -1) */
		 int nsetmx,
		 /** TOTAL number of tasks, over all task sets. For
		     example, if the behavior has two task sets, one
		     with 3 and one with 2 levels, then you have to
		     pass 3+2=5 here.  */
		 int ttntasks,
		 /** number of matrices for each task */
		 int ntaskmx);
    
    status ProcessHeader(msg::TaskMatrixWrap const & tmwp);
    status ProcessPayload(msg::TaskMatrixWrap const & tmwp);
    bool ModelsAreComplete() const;
    
    uint8_t GetRequestID() const { return m_requestID; }
    int GetNGeneralMatrices() const { return m_ngenmx; }
    int GetNSets() const { return m_nsets; }
    int GetNSetMatrices() const { return m_nsetmx; }
    int GetTTNTasks() const { return m_ttntasks; }
    int GetNTaskMatrices() const { return m_ntaskmx; }
    
    timeval const & GetAcquisitionTime() const;
    void SetAcquisitionTime(timeval const & at);
    
  protected:
    timeval m_acquisition_time;
    endian_mode_t m_endian_mode;
    
  private:
    uint8_t m_requestID;
    
    int m_ngenmx;
    bitmap_t m_genmx_mask;
    bitmap_t m_genmx_flags;
    
    int m_nsets;
    bitmap_t m_set_mask;
    bitmap_t m_set_flags;
    
    int m_nsetmx;
    bitmap_t m_setmx_mask;
    bitmap_t * m_setmx_flags;	// array of GetCapacity() dimensions
    
    int m_ttntasks;
    bitmap_t m_task_mask;
    bitmap_t m_task_flags;
    
    int m_ntaskmx;
    bitmap_t m_taskmx_mask;
    bitmap_t * m_taskmx_flags;	// array of GetCapacity() dimensions
    
    /** \pre 0<=number<=GetCapacity() */
    bitmap_t ComputeMask(int number);
    
    /** \pre 0<=number<=GetCapacity() */
    bitmap_t ComputeFlag(int number);
  };
  
}

#endif // WBCNET_TASK_ATOMIZER_HPP
