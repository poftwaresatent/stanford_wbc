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

#include "TaskMatrix.hpp"
#include <iostream>

namespace wbcnet {
  
  namespace msg {
    
    TaskMatrixWrap::
    TaskMatrixWrap(unique_id_t id,
		   uint8_t max_nRows,
		   uint8_t max_nColumns,
		   MatrixStorageAPI * dataptr)
      : MatrixWrap(id, max_nRows, max_nColumns, dataptr)
    {
      m_header.AddField(new ValuePack<uint8_t, fixed_count>
			("TaskMatrix::requestID", true, &requestID, 1));
      m_header.AddField(new TimestampPack("TaskMatrix::acquisitionTime", true, &acquisitionTime));
      m_header.AddField(new ValuePack<int8_t, fixed_count>
			("TaskMatrix::setID", true, &setID, 1));
      m_header.AddField(new ValuePack<int8_t, fixed_count>
			("TaskMatrix::taskID", true, &taskID, 1));
      m_header.AddField(new ValuePack<uint8_t, fixed_count>
			("TaskMatrix::matrixID", true, &matrixID, 1));
    }
    
    
    TaskMatrix::
    TaskMatrix(unique_id_t id,
		    uint8_t nRows,
		    uint8_t nColumns,
		    uint8_t max_nRows,
		    uint8_t max_nColumns)
      : TaskMatrixWrap(id, max_nRows, max_nColumns, &data),
	data(nRows, nColumns)
    {
    }
  
  
    bool TaskMatrix::
    operator == (TaskMatrix const & rhs) const
    {
      return (requestID == rhs.requestID)
	&&   (acquisitionTime == rhs.acquisitionTime)
	&&   (setID == rhs.setID)
	&&   (taskID == rhs.taskID)
	&&   (matrixID == rhs.matrixID)
	&&   (data == rhs.data);
    }
  
  
    void TaskMatrix::
    display(std::ostream & os, char const * prefix) const
    {
      os << prefix << "requestID: " << requestID << "\n"
	 << prefix << "time: " << acquisitionTime.tv_sec << "s "
	 << acquisitionTime.tv_usec << "usec\n"
	 << prefix << "setID: " << setID << "\n"
	 << prefix << "taskID: " << taskID << "\n"
	 << prefix << "matrixID: " << matrixID << "\n"
	 << prefix << "nRows: " << nRows << "\n"
	 << prefix << "nColumns: " << nColumns << "\n"
	 << prefix << "matrix:\n";
      for (int ii(data.NRows() - 1); ii >= 0; --ii) {
	os << prefix;
	for (int jj(0); jj < data.NColumns(); ++jj)
	  os << "  " << data.GetElement(ii, jj);
	os << "\n";
      }
    }

  }

}
