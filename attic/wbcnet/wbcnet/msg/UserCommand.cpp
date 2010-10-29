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

#include "UserCommand.hpp"
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));

namespace wbcnet {
  
  namespace msg {
    
    
    UserCommandWrap::
    UserCommandWrap(unique_id_t id,
		    uint8_t _max_nCodes,
		    VectorStorageAPI * _codeptr,
		    uint8_t _max_nRows,
		    uint8_t _max_nColumns,
		    MatrixStorageAPI * _matrixptr)
      : StringList(id),
	max_nCodes(_max_nCodes),
	max_nRows(_max_nRows),
	max_nColumns(_max_nColumns),
	requestID(0),
	nCodes(_max_nCodes),
	nRows(_max_nRows),
	nColumns(_max_nColumns),
	codeptr(_codeptr),
	matrixptr(_matrixptr)
    {
      m_header.AddField(new ValuePack<uint8_t, fixed_count>
			("wbcnet::msg::UserCommand::requestID", true, &requestID, 1));
      m_header.AddField(new ValuePack<uint8_t, fixed_count>
			("wbcnet::msg::UserCommand::nCodes", true, &nCodes, 1));
      m_header.AddField(new ValuePack<uint8_t, fixed_count>
			("wbcnet::msg::UserCommand::nRows", true, &nRows, 1));
      m_header.AddField(new ValuePack<uint8_t, fixed_count>
			("wbcnet::msg::UserCommand::nColumns", true, &nColumns, 1));
      m_payload.AddField(new VectorPack("wbcnet::msg::UserCommand::code", true, false,
					codeptr));
      m_payload.AddField(new MatrixPack("wbcnet::msg::UserCommand::matrix", true, false,
					&matrixptr));
    }
    
    
    proxy_status UserCommandWrap::
    Pack(BufferAPI & buffer, endian_mode_t endian_mode)
    {
      if (codeptr->NElements() > max_nCodes) {
	LOG_ERROR (logger,
		       "wbcnet::msg::UserCommandWrap::Pack(): too many codes\n"
		       << "  have codeptr->NElements(): " << codeptr->NElements() << "\n"
		       << "  but max_nCodes: " << (int) max_nCodes);
	return PROXY_RESIZE_ERROR;
      }
      if (matrixptr->NRows() > max_nRows) {
	LOG_ERROR (logger,
		       "wbcnet::msg::UserCommandWrap::Pack(): too many rows\n"
		       << "  have matrixptr->NRows(): " << matrixptr->NRows() << "\n"
		       << "  but max_nRows: " << (int) max_nRows);
	return PROXY_RESIZE_ERROR;
      }
      if (matrixptr->NColumns() > max_nColumns) {
	LOG_ERROR (logger,
		       "wbcnet::msg::UserCommandWrap::Pack(): too many columns\n"
		       << "  have matrixptr->NColumns(): " << matrixptr->NColumns() << "\n"
		       << "  but max_nColumns: " << (int) max_nColumns);
	return PROXY_RESIZE_ERROR;
      }
      nCodes = codeptr->NElements();
      nRows = matrixptr->NRows();
      nColumns = matrixptr->NColumns();
      return StringList::Pack(buffer, endian_mode);
    }
    
    
    proxy_status UserCommandWrap::
    UnpackPayload(BufferAPI const & buffer, endian_mode_t endian_mode)
    {
      codeptr->SetNElements(nCodes);
      matrixptr->SetSize(nRows, nColumns);
      return StringList::UnpackPayload(buffer, endian_mode);
    }
    
    
    proxy_status UserCommandWrap::
    CheckHeader() const
    {
      if (max_nCodes < nCodes) {
	LOG_ERROR (logger,
		       "wbcnet::msg::UserCommandWrap::CheckHeader(): too many codes\n"
		       << "  have nCodes: " << (int) nCodes << "\n"
		       << "  but max_nCodes: " << (int) max_nCodes);
	return PROXY_DIMENSION_MISMATCH;
      }
      if (max_nRows < nRows) {
	LOG_ERROR (logger,
		       "wbcnet::msg::UserCommandWrap::CheckHeader(): too many rows\n"
		       << "  have nRows: " << (int) nRows << "\n"
		       << "  but max_nRows: " << (int) max_nRows);
	return PROXY_DIMENSION_MISMATCH;
      }
      if (max_nColumns < nColumns) {
	LOG_ERROR (logger,
		       "wbcnet::msg::UserCommandWrap::CheckHeader(): too many columns\n"
		       << "  have nColumns: " << (int) nColumns << "\n"
		       << "  but max_nColumns: " << (int) max_nColumns);
	return PROXY_DIMENSION_MISMATCH;
      }
      return StringList::CheckHeader();
    }
    
  }
  
}
