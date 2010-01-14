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

/** \file data.hpp Pretty generic data interfaces. */

#ifndef WBCNET_DATA_HPP
#define WBCNET_DATA_HPP

#ifdef WIN32
# define NOMINMAX
# include <limits>
# include <windows.h>
# undef max
#else
# include <limits>
#endif

#include <string>

namespace wbcnet {
  
  
  /**
     Minimum functionality of a buffer. It has to
     - have some data accessible though a pointer
     - know how big it is
     - support resizing attempts and yell if they fail
     - be able to attempt copying data from another buffer
  */
  class BufferAPI {
  public:
    virtual ~BufferAPI() {}
    
    virtual char * GetData() const = 0;    
    virtual int GetSize() const = 0;
    virtual bool Resize(int size) = 0;
    virtual bool Set(BufferAPI const & rhs) = 0;
    
  private:
    BufferAPI & operator = (BufferAPI const &);
  };
  
  
  std::string hexdump_buffer(char const * buffer, int nbytes);
  
  
  /**
     Minimum requirements for sending vectors over the wire. They have
     to
     - know how many elements they contain
     - know how big each element is
     - be resizeable
     - provide a pointer to a continuous block of storage where their
       elements are kept
       
     \note We assume that the total storage size is NElements() *
     ElementStorageSize(). Everything is declared const so that you
     can use lightweight throw-away wrappers to existing vector
     implementations.
  */
  class VectorStorageAPI {
  public:
    virtual ~VectorStorageAPI() {}
    
    bool operator == (VectorStorageAPI const & rhs) const;
    bool operator != (VectorStorageAPI const & rhs) const { return ! (*this == rhs); }
    
    virtual int NElements() const = 0;
    virtual bool SetNElements(int nelem) = 0;
    virtual int ElementStorageSize() const = 0;
    virtual char const * DataPointer() const = 0;
    virtual char * DataPointer() = 0;

    /**
       Fills the vector with bytes of value zero.
    */
    void SetZero();
    
    /**
       Copies from another vector.
       
       \return true if the assignment succeeded. Failures can be due
       to ElementStorageSize() mismatches or failed
       SetNElements(). The latter "should" always succeed, so as long
       as you are sure that the former matches (and actually implies
       the same type, not just the same size) you should be fine.
    */
    bool Copy(VectorStorageAPI const & rhs);
    
    /**
       Copies a portion of a source vector into a portion of *this
       (the destination vector).
       
       \param dst_begin_idx The index where the splice should be
       inserted into the destination.
       
       \param src The source vector.
       
       \param src_begin_idx The index where the splice should be taken
       from in the source vector.
       
       \param src_end_idx An index ONE PAST the end of the range to be
       spliced from the source vector.
       
       \note
       - Negative indices are treated as being counted
         from the end backwards. E.g. passing "-1" as src_begin_idx is
         equivalent to passing "src.NElements()-1".
       - Indices that, after adjustments for negative values, refer to
         "before the start" are silently treated as zero (i.e. "at the
         start"). Similarly, "after the end" is treated as "at the end".
       - An invalid source range is silently treated like an empty range.
       
       \return true if the splice succeeded. Failures can be due to
       ElementStorageSize() mismatches or failed SetNElements(). The
       latter "should" always succeed --- thus, as long as you are
       sure that the former matches (and actually implies the same
       type, not just the same size) you should be fine.
    */
    bool Splice(int dst_begin_idx,
		VectorStorageAPI const & src,
		int src_begin_idx,
		/** \note use std::numeric_limits<int>::max() if you
		    want to include the end of src without needing to
		    worry about its current size. */
		int src_end_idx);
    };
  
  
  template<typename element_t>
  class VectorAPI
    : public VectorStorageAPI
  {
  public:
    typedef element_t element_type;
    
    virtual int ElementStorageSize() const { return sizeof(element_type); }
    
    template<typename index_t>
    element_type const & operator [] (index_t idx) const
    { return reinterpret_cast<element_type const*>(DataPointer())[idx]; }
    
    template<typename index_t>
    element_type & operator [] (index_t idx)
    { return reinterpret_cast<element_type*>(DataPointer())[idx]; }
    
    template<typename index_t>
    element_type const & GetElement(index_t idx) const
    { return reinterpret_cast<element_type const*>(DataPointer())[idx]; }
    
    template<typename index_t>
    element_type & GetElement(index_t idx)
    { return reinterpret_cast<element_type*>(DataPointer())[idx]; }
    
    element_type const * ElementPointer() const
    { return reinterpret_cast<element_type const *>(DataPointer()); }
    
    element_type * ElementPointer()
    { return reinterpret_cast<element_type *>(DataPointer()); }
    
    template<typename os_t>
    void Display(os_t & os, char const * prefix) const
    {
      os << prefix << "N elements: " << NElements() << "\n"
	 << prefix << "element size: " << ElementStorageSize() << "\n";
      for (int ii(0); ii < NElements(); ++ii)
	os << prefix << "  " << (*this)[ii] << "\n";
    }
  };
  
  
  /**
     Minimum requirements for sending matrices over the wire. They have
     to
     - know how many rows and columns they contain
     - know how big each element is
     - be resizeable
     - provide a pointer to a continuous block of storage where their
       elements are kept
       
     \note We assume that the total storage size is NRows() *
     NColumns() * ElementStorageSize(). Everything is declared const
     so that you can use lightweight throw-away wrappers to existing
     matrix implementations.
  */
  class MatrixStorageAPI {
  public:
    virtual ~MatrixStorageAPI() {}
    
    bool operator == (MatrixStorageAPI const & rhs) const;
    bool operator != (MatrixStorageAPI const & rhs) const { return ! (*this == rhs); }
    
    virtual int NRows() const = 0;
    virtual int NColumns() const = 0;
    virtual bool SetSize(int nrows, int ncols) = 0;
    virtual int ElementStorageSize() const = 0;
    virtual char const * DataPointer() const = 0;
    virtual char * DataPointer() = 0;
    
    /**
       Fills the matrix with bytes of value zero.
    */
    void SetZero();
    
    /**
       \note MatrixStorageAPI does not enforce row- or column- major
       mode, and it has no way of checking whether you are mixing two
       incompatible modes. It simply uses memcpy...
       
       \return true if the assignment succeeded. Failures can be due
       to ElementStorageSize() mismatches or failed SetSize(). The
       latter "should" always succeed, so as long as you are sure that
       the former matches (and actually implies the same type, not
       just the same size) you should be fine.
    */
    bool Copy(MatrixStorageAPI const & rhs);
    
    /**
       Copies a vector, creating a matrix with one column.
       
       \return true if the assignment succeeded. Failures can be due
       to ElementStorageSize() mismatches or failed SetSize(). The
       latter "should" always succeed, so as long as you are sure that
       the former matches (and actually implies the same type, not
       just the same size) you should be fine.
    */
    bool Copy(VectorStorageAPI const & rhs);
  };
  
  
  template<typename element_t>
  class MatrixAPI
    : public MatrixStorageAPI
  {
  public:
    typedef element_t element_type;
    
    virtual int ElementStorageSize() const { return sizeof(element_type); }
    
    template<typename index_t>
    element_type const & GetElement(index_t idx) const
    { return reinterpret_cast<element_type const*>(DataPointer())[idx]; }
    
    template<typename index_t>
    element_type & GetElement(index_t idx)
    { return reinterpret_cast<element_type*>(DataPointer())[idx]; }
    
    template<typename index1_t, typename index2_t>
    element_type const & GetElement(index1_t irow, index2_t icol) const
    { return reinterpret_cast<element_type const*>(DataPointer())[irow * NColumns() + icol]; }
    
    template<typename index1_t, typename index2_t>
    element_type & GetElement(index1_t irow, index2_t icol)
    { return reinterpret_cast<element_type*>(DataPointer())[irow * NColumns() + icol]; }
    
    template<typename ostream_t>
    void Display(ostream_t & os, char const * prefix) const {
      for (int ii(NRows() - 1); ii >= 0; --ii) {
	os << prefix;
	for (int jj(0); jj < NColumns(); ++jj)
	  os << "  " << GetElement(ii, jj);
	os << "\n";
      }
    }
    
    element_type const * ElementPointer() const
    { return reinterpret_cast<element_type const *>(DataPointer()); }
    
    element_type * ElementPointer()
    { return reinterpret_cast<element_type *>(DataPointer()); }
  };
  
  
  /**
     An implementation of BufferAPI using malloc() and free(). You can
     specify at construction-time how the buffer should react to
     resizing attempts by setting the max_size parameter. For example,
     if you have to avoid malloc() or realloc() inside a real-time
     loop, you should construct Buffer instances using size and
     max_size equal to the needs inside the loop. If reallocation is
     not an issue, you can set max_size=-1.
  */
  class Buffer
    : public BufferAPI {
  public:
    /** If max_size<0 then we can grow "indefinitely". */
    Buffer(int size, int max_size);
    Buffer(Buffer const & orig);
    
    /** When copying from a generic BufferAPI we set max_size = size. */
    Buffer(BufferAPI const & orig);
    
    virtual ~Buffer();
    
    virtual char * GetData() const;
    virtual int GetSize() const;
    virtual bool Resize(int size);
    virtual bool Set(BufferAPI const & rhs);
    
  private:
    Buffer & operator = (Buffer const & rhs)
    { Set(rhs); return *this; }
    
  protected:
    mutable char * m_data;
    int m_alloc_size;
    int m_apparent_size;	// never bigger than m_alloc_size
    int const m_max_size;
  };
  
  
  template<typename value_t>
  class Vector
    : public VectorAPI<value_t>
  {
  public:
    typedef value_t value_type;
    
    Vector()
      : m_len(0), m_buf(0, -1) {}
    
    explicit Vector(int len)
      : m_len(len), m_buf(len * sizeof(value_type), -1) {}
    
    Vector(int len, int max_len)
      : m_buf(len * sizeof(value_type), max_len * sizeof(value_type)),
	m_len(len) {}
    
    virtual int NElements() const { return m_len; }
    
    virtual bool SetNElements(int nelem) {
      if (m_buf.Resize(nelem * sizeof(value_type))) {
	m_len = nelem;
	return true;
      }
      return false; }
    
    virtual char const * DataPointer() const { return m_buf.GetData(); }
    
    virtual char * DataPointer() { return m_buf.GetData(); }
    
    /** For clients that know about it, this method is slightly faster
	than the virtual NElements(); */
    int GetLength() const { return m_len; }

    Vector CreateSplice(int begin_idx, int end_idx = std::numeric_limits<int>::max()) const {
      Vector splice(0);
      splice.Splice(0, *this, begin_idx, end_idx);
      return splice;
    }
    
    
  protected:
    int m_len;
    Buffer m_buf;
  };
  
  
  template<typename value_t>
  class Matrix
    : public MatrixAPI<value_t>
  {
  public:
    typedef value_t value_type;
    
    Matrix()
      : m_row(0), m_col(0), m_buf(0, -1) {}
    
    Matrix(int row, int col)
      : m_row(row), m_col(col), m_buf(row * col * sizeof(value_type), -1) {}
    
    virtual int NRows() const { return m_row; }
    
    virtual int NColumns() const { return m_col; }
    
    virtual bool SetSize(int nrows, int ncols) {
      if (m_buf.Resize(nrows * ncols * sizeof(value_type))) {
	m_row = nrows;
	m_col = ncols;
	return true;
      }
      return false; }
    
    virtual char const * DataPointer() const { return m_buf.GetData(); }
    
    virtual char * DataPointer() { return m_buf.GetData(); }
    
  protected:
    int m_row;
    int m_col;
    Buffer m_buf;
  };

}

#endif // WBCNET_DATA_HPP
