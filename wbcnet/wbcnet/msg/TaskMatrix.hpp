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

#ifndef WBCNET_MSG_TASK_MATRIX_HPP
#define WBCNET_MSG_TASK_MATRIX_HPP

#include <wbcnet/msg/Matrix.hpp>

namespace wbcnet {
  
  namespace msg {
    
    class TaskMatrixWrap
      : public MatrixWrap
    {
    public:
      TaskMatrixWrap(unique_id_t id,
		     uint8_t max_nRows,
		     uint8_t max_nColumns,
		     MatrixStorageAPI * dataptr);
      
      // header
      uint8_t requestID;
      timestamp acquisitionTime;
      int8_t setID; // signed int, because -1 is valid
      int8_t taskID; // signed int, because -1 is valid
      uint8_t matrixID;

      // payload comes from MatrixWrap
    };
    
    
    class TaskMatrix
      : public TaskMatrixWrap
    {
    public:
      /** \note data is initialized to size(nRows, nColumns), size
	  checks are performed against (max_nRows, max_nColumns). */
      TaskMatrix(unique_id_t id,
		 uint8_t nRows,
		 uint8_t nColumns,
		 uint8_t max_nRows,
		 uint8_t max_nColumns);
      
      bool operator == (TaskMatrix const & rhs) const;
      bool operator != (TaskMatrix const & rhs) const { return ! (*this == rhs); }
      void display(std::ostream & os, char const * prefix) const;
      
      // inherit everything, just add a bit of storage
      Matrix<double> data;
    };
    
  }
  
}

#endif // WBCNET_MSG_TASK_MATRIX_HPP
