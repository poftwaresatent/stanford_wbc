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

#ifndef WBCNET_MSG_USER_COMMAND_HPP
#define WBCNET_MSG_USER_COMMAND_HPP

#include <wbcnet/msg/StringList.hpp>

namespace wbcnet {
  
  namespace msg {
    
    
    class UserCommandWrap
      : public StringList
    {
    public:
      UserCommandWrap(unique_id_t id,
		      uint8_t max_nCodes,
		      VectorStorageAPI * codeptr,
		      uint8_t max_nRows,
		      uint8_t max_nColumns,
		      MatrixStorageAPI * matrixptr);
      
      /** Check that payload sizes are OK, synch the header with them,
	  and then delegate to Proxy::Pack(). */
      virtual proxy_status Pack(BufferAPI & buffer, endian_mode_t endian_mode);
      
      virtual proxy_status UnpackPayload(BufferAPI const & buffer, endian_mode_t endian_mode);
      
      /** Yell if max_nCode < nCode, max_nRows < nRows, or max_nColumns < nColumns. */
      virtual proxy_status CheckHeader() const;
      
      uint8_t const max_nCodes;
      uint8_t const max_nRows;
      uint8_t const max_nColumns;
      
      // header
      uint8_t requestID;
      uint8_t nCodes;
      uint8_t nRows;
      uint8_t nColumns;
      
      // payload
      VectorStorageAPI * codeptr;
      MatrixStorageAPI * matrixptr;
    };
    
    
    /**
       \note You'll probably want to use wbcnet::VectorAPI<int32_t> as
       vector_t template parameter.
    */
    template<typename vector_t, typename matrix_t>
    class UserCommand
      : public UserCommandWrap
    {
    public:
      typedef vector_t vector_type;
      typedef matrix_t matrix_type;
      
      /** \note The code vector payload is initialized to nCode size
	  and checks are performed against max_nCode. Likewise, the
	  matrix payload is initialized to size(nRows, nColumns), size
	  checks are performed against (max_nRows, max_nColumns). */
      UserCommand(unique_id_t id,
		  uint8_t nCodes,
		  uint8_t nRows,
		  uint8_t nColumns,
		  uint8_t max_nCodes,
		  uint8_t max_nRows,
		  uint8_t max_nColumns)
	: UserCommandWrap(id, max_nCodes, &code, max_nRows, max_nColumns, &matrix),
	  code(nCodes),
	  matrix(nRows, nColumns) {}
      
      template<typename ostream_t>
      void Dump(ostream_t & os, char const * prefix) const
      {
	os << prefix << "requestID: " << (int) requestID << "\n"
	   << prefix << "nCodes: " << (int) nCodes << "\n"
	   << prefix << "nRows: " << (int) nRows << "\n"
	   << prefix << "nColumns: " << (int) nColumns << "\n"
	   << prefix << "code:\n";
	for (int ii(0); ii < code.NElements(); ++ii)
	  os << prefix << "  " << (int) code[ii] << "\n";
	os << prefix << "matrix:\n";
	for (int ii(0); ii < matrix.NRows(); ++ii) {
	  os << prefix;
	  for (int jj(0); jj < matrix.NColumns(); ++jj)
	    os << "  " << matrix.GetElement(ii, jj);
	  os << "\n";
	}
	os << prefix << "strlist:\n";
	StringList::display(os, prefix);
      }
      
      bool operator == (UserCommand const & rhs) const {
	return (&rhs == this)
	  || ( *(static_cast<StringList const *>(this)) == *(static_cast<StringList const *>(&rhs))
	      && (requestID == rhs.requestID)
	      && (nCodes == rhs.nCodes)
	      && (nRows == rhs.nRows)
	      && (nColumns == rhs.nColumns)
	      && (code == rhs.code)
	      && (matrix == rhs.matrix));
      }
      
      bool operator != (UserCommand const & rhs) const
      { return ! (*this == rhs); }
      
      // inherit everything, just add a bit of storage
      vector_type code;
      matrix_type matrix;
    };
    
  }
  
}

#endif // WBCNET_MSG_USER_COMMAND_HPP
