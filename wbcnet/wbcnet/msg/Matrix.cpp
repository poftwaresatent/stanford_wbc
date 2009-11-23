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

#include "Matrix.hpp"
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));

namespace wbcnet {
  
  namespace msg {
    
    MatrixWrap::
    MatrixWrap(unique_id_t id,
	       uint8_t _max_nRows,
	       uint8_t _max_nColumns,
	       MatrixStorageAPI * _dataptr)
      : Proxy(id),
	max_nRows(_max_nRows),
	max_nColumns(_max_nColumns),
	nRows(_max_nRows),
	nColumns(_max_nColumns),
	dataptr(_dataptr)
    {
      m_header.AddField(new ValuePack<uint8_t, fixed_count>
			("wbcnet::msg::Matrix::nRows", true, &nRows, 1));
      m_header.AddField(new ValuePack<uint8_t, fixed_count>
			("wbcnet::msg::Matrix::nColumns", true, &nColumns, 1));
      m_payload.AddField(new MatrixPack("wbcnet::msg::Matrix::data", true, false, &dataptr));
    }
    
    
    proxy_status MatrixWrap::
    Pack(BufferAPI & buffer, endian_mode_t endian_mode)
    {
      if (dataptr->NRows() > max_nRows) {
	LOG_ERROR (logger,
		       "wbcnet::msg::MatrixWrap::Pack(): too many rows in payload\n"
		       << "  have dataptr->NRows(): " << dataptr->NRows() << "\n"
		       << "  but max_nRows: " << (int) max_nRows);
	return PROXY_RESIZE_ERROR;
      }
      if (dataptr->NColumns() > max_nColumns) {
	LOG_ERROR (logger,
		       "wbcnet::msg::MatrixWrap::Pack(): too many columns in payload\n"
		       << "  have dataptr->NColumns(): " << dataptr->NColumns() << "\n"
		       << "  but max_nColumns: " << (int) max_nColumns);
	return PROXY_RESIZE_ERROR;
      }
      nRows = dataptr->NRows();
      nColumns = dataptr->NColumns();
      return Proxy::Pack(buffer, endian_mode);
    }
    
    
    proxy_status MatrixWrap::
    UnpackPayload(BufferAPI const & buffer, endian_mode_t endian_mode)
    {
      dataptr->SetSize(nRows, nColumns);
      return DoUnpackPayload(buffer, endian_mode);
    }
    
    
    proxy_status MatrixWrap::
    CheckHeader() const
    {
      if ((max_nRows >= nRows) && (max_nColumns >= nColumns))
	return PROXY_OK;
      LOG_ERROR (logger,
		     "wbcnet::msg::MatrixWrap::CheckHeader(): maximum size exceeded:\n"
		     << "  nRows is " << static_cast<unsigned int>(nRows) << " (max "
		     << static_cast<unsigned int>(max_nRows) << ")\n"
		     << "  nColumns is " << static_cast<unsigned int>(nColumns) << " (max "
		     << static_cast<unsigned int>(max_nColumns) << ")");
      return PROXY_DIMENSION_MISMATCH;
    }
    
  }

}
