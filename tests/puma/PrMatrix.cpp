/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2010 Stanford University. All rights reserved.
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

// *******************************************************************
// PrMatrix.cpp
//
// Implementation of an MxN matrix.
//
// modification history
// --------------------
//
// 06/17/04: Dan Merget: Made PrMatrix3 and PrMatrix6 into subclasses
// 03/??/04: Tine Lefebvre: added eigenvaluedecomposition (eig) and
//           copyToSymmetricMatrix
// 11/12/97: K.C. Chang: created
// *******************************************************************

#include "PrMatrix.h"
#ifdef _WIN32
#include <float.h>
#define finite(num) _finite(num)
#endif

// ===================================================================
// Constructors & Destructors
// ===================================================================

PrMatrix::PrMatrix( int row, int col )
  : m_row( row ), m_col( col ), m_size( row*col ), m_maxSize( row*col )
{
  SAIAssert( m_row >= 0 && m_col >= 0 );

  if(  m_maxSize > 0 )
  {
    m_data = new Float[m_maxSize];
    zero();
  }
  else
  {
    m_data = NULL;
  }
}

PrMatrix::PrMatrix( const PrMatrix& rhs )
  : m_row( rhs.m_row ), m_col( rhs.m_col ),
    m_size( rhs.m_size ), m_maxSize( rhs.m_size )
{
  if( m_maxSize > 0 )
  {
    m_data = new Float[m_maxSize];
    memcpy( m_data, rhs.m_data, sizeof( Float ) * m_size );
  }
  else
  {
    m_data = NULL;
  }
}

PrMatrix::PrMatrix( const Float* rgVals, int row, int col, bool fTranspose )
  : m_row( row ), m_col( col ), m_size( row*col ), m_maxSize( row*col )

{
  SAIAssert( m_row >= 0 && m_col >= 0 );

  if(  m_maxSize > 0 )
  {
    m_data = new Float[m_maxSize];
    if( rgVals != NULL )
    {
      setValues( rgVals, fTranspose );
    }
  }
  else
  {
    m_data = NULL;
  }
}

PrMatrix::~PrMatrix()
{
  if( m_data != NULL )
  {
    delete[] m_data;
  }
}

// ===================================================================
// fFinite(): Return true if all of the elements are finite, or false
// if any element is infinite or NaN.
// ===================================================================
bool PrMatrix::fFinite() const
{
  for( int ii = 0; ii < m_size; ii++ )
  {
    if( ! finite( m_data[ii] ) )
    {
      return false;
    }
  }
  return true;
}

// ===================================================================
// Assignment operator.  Resize this matrix if necessary
// ===================================================================
PrMatrix& PrMatrix::operator=( const PrMatrix& rhs )
{
  if( m_data != rhs.m_data )
  {
    resize( rhs.m_row, rhs.m_col );
    memcpy( m_data, rhs.m_data, sizeof( Float ) * m_size );
  }
  return *this;
}

// ===================================================================
// setValues(): Copy values from rgVals to this matrix.  Use the
// current size of the matrix to determine how many values to copy.
// rgVals is assumed to be in row-major form if fTranspose is true, or
// column-major form otherwise.
// ===================================================================
void PrMatrix::setValues( const Float* rgVals, bool fTranspose )
{
  if( !fTranspose )
  {
    memcpy( m_data, rgVals, sizeof( Float ) * m_size );
  }
  else
  {
    for( int jj = 0; jj < m_col; jj++ )
    {
      for( int ii = 0; ii < m_row; ii++ )
      {
        elementAt(ii, jj) = *rgVals++;
      }
    }
  }
}

// ===================================================================
// getValues(): Copy values from this matrix to rgVals.  Use the size
// of the matrix to determine how many values to copy.  rgVals is
// assumed to be in row-major form if fTranspose is true, or
// column-major form otherwise.
// ===================================================================
void PrMatrix::getValues( Float* rgVals, bool fTranspose ) const
{
  if( !fTranspose )
  {
    memcpy( rgVals, m_data, sizeof( Float ) * m_size );
  }
  else
  {
    for( int jj = 0; jj < m_col; jj++ )
    {
      for( int ii = 0; ii < m_row; ii++ )
      {
        *rgVals++ = elementAt(ii, jj);
      }
    }
  }
}

// ===================================================================
// appendVertically() / appendHorizontally(): Append rhs to this
// matrix.  Resize if necessary.
// ===================================================================
void PrMatrix::appendVertically( const PrMatrix& rhs )
{
  if ( rhs.m_size > 0 )
  {
    SAIAssert( m_col == rhs.m_col || m_size == 0 );
    PrMatrix ans( m_row + rhs.m_row, rhs.m_col );

    ans.setSubmatrix( 0, 0, *this );
    ans.setSubmatrix( m_row, 0, rhs );
    transfer( ans );
  }
}

void PrMatrix::appendHorizontally( const PrMatrix& rhs )
{
  if ( rhs.m_size > 0 )
  {
    SAIAssert( m_row == rhs.m_row || m_size == 0 );
    PrMatrix ans( rhs.m_row, m_col + rhs.m_col );

    ans.setSubmatrix( 0, 0, *this );
    ans.setSubmatrix( 0, m_col, rhs );
    transfer( ans );
  }
}

// ===================================================================
// setSize():  Resize the matrix and optionally zero it.
// ===================================================================
void PrMatrix::setSize( int row, int col, bool fZero )
{
  resize( row, col );
  if( fZero )
  {
    zero();
  }
}

// ===================================================================
// transfer(): Replace contents of this matrix with those of src,
// without allocating memory if possible.  Puts src in an undefined
// state.
//
// If both *this and src are dynamically allocated, then this
// operation efficiently moves the allocated data from src to *this.
// However, if either matrix is non-dynamic (e.g. a PrMatrix3), then
// the data is simply copied.
// ===================================================================
void PrMatrix::transfer( PrMatrix& src )
{
  if( dynamic() && src.dynamic() )
  {
    if( m_data )
    {
      delete[] m_data;
    }
    m_row     = src.m_row;
    m_col     = src.m_col;
    m_size    = src.m_size;
    m_maxSize = src.m_maxSize;
    m_data    = src.m_data;
    src.m_row     = 0;
    src.m_col     = 0;
    src.m_size    = 0;
    src.m_maxSize = 0;
    src.m_data    = NULL;
  }
  else
  {
    *this = src;
  }
}

// ===================================================================
// diagonal(): Retrieve the elements on the diagonal of a square
// matrix.
// ===================================================================
void PrMatrix::diagonal( PrVector& dest ) const
{
  SAIAssert( m_row == m_col );
  dest.setSize( m_row );
  for( int ii = 0; ii < m_row; ii++ )
  {
    dest[ii] = elementAt( ii, ii );
  }
}

// ===================================================================
// setDiagonal(): Set the elements on the diagonal of a square matrix.
// ===================================================================
void PrMatrix::setDiagonal( Float src )
{
  SAIAssert( m_row == m_col );
  for( int ii = 0; ii < m_row; ii++ )
  {
    elementAt(ii, ii) = src;
  }
}

void PrMatrix::setDiagonal( const Float* src )
{
  SAIAssert( m_row == m_col );
  for( int ii = 0; ii < m_row; ii++ )
  {
    elementAt(ii, ii) = src[ii];
  }
}

void PrMatrix::setDiagonal( const PrVector& src )
{
  SAIAssert( m_row == m_col );
  SAIAssert( m_row == src.size() );
  for( int ii = 0; ii < m_row; ii++ )
  {
    elementAt(ii, ii) = src[ii];
  }
}

// ===================================================================
// getRow() / getColumn(): Extract a row or column from the matrix.
// ===================================================================
void PrMatrix::getRow( int row, PrVector& dest ) const
{
  SAIAssert( row >= 0 && row < m_row );
  dest.setSize( m_col );
  for( int jj = 0; jj < m_col; jj++ )
  {
    dest[jj] = elementAt( row, jj );
  }
}

void PrMatrix::getColumn( int col, PrVector& dest ) const
{
  SAIAssert( col >= 0 && col < m_col );
  dest.setSize( m_row );
  for( int ii = 0; ii < m_row; ii++ )
  {
    dest[ii] = elementAt( ii, col );
  }
}

// ===================================================================
// setRow() / setColumn(): Set a row or column in the matrix.
// ===================================================================
void PrMatrix::setRow( int row, const PrVector& src )
{
  SAIAssert( row >= 0 && row < m_row );
  SAIAssert( src.size() == m_col );
  for( int jj = 0; jj < m_col; jj++ )
  {
    elementAt( row, jj ) = src[jj];
  }
}

void PrMatrix::setColumn( int col, const PrVector& src )
{
  SAIAssert( col >= 0 && col < m_col );
  SAIAssert( src.size() == m_row );
  for( int ii = 0; ii < m_row; ii++ )
  {
    elementAt( ii, col ) = src[ii];
  }
}

void PrMatrix::setSubmatrix( Float val, int iRow, int iCol, int cRow, int cCol )
{
  for( int ii = 0; ii < cRow; ii++ )
    for( int jj = 0; jj < cCol; jj++ )
    {
      elementAt( ii + iRow, jj + iCol ) = val;
    }
}

// ===================================================================
// submatrix(): Extract a submatrix.  The "row" & "col" arguments give
// the coordinates of the submatrix, and the dimensions of "dest" give
// the size of the submatrix.
// ===================================================================
void PrMatrix::submatrix( int row, int col, PrMatrix& dest ) const
{
  SAIAssert( row >= 0 && row + dest.m_row <= m_row );
  SAIAssert( col >= 0 && col + dest.m_col <= m_col );

  if( col == 0 && dest.m_col == m_col )
  {
    // If we are copying entire rows, we can use a single memcpy
    memcpy( dest.m_data, m_data + m_col * row, sizeof( Float ) * dest.m_size );
  }
  else
  {
    // Do the copy by hand
    for( int ii = 0; ii < dest.m_row; ii++ )
    {
      for( int jj = 0; jj < dest.m_col; jj++ )
      {
        dest( ii, jj ) = elementAt( row + ii, col + jj );
      }
    }
  }
}

// ===================================================================
// setSubmatrix(): Copy a smaller matrix into a subset of this matrix.
// The "row" & "col" arguments tell where to copy the smaller matrix.
// ===================================================================
void PrMatrix::setSubmatrix( int row, int col, const PrMatrix& src )
{
  SAIAssert( row >= 0 && row + src.m_row <= m_row );
  SAIAssert( col >= 0 && col + src.m_col <= m_col );

  if( col == 0 && src.m_col == m_col )
  {
    // If we are copying entire rows, we can use a single memcpy
    memcpy( m_data + m_col * row, src.m_data, sizeof( Float ) * src.m_size );
  }
  else
  {
    // Do the copy by hand
    for( int ii = 0; ii < src.m_row; ii++ )
    {
      for( int jj = 0; jj < src.m_col; jj++ )
      {
        elementAt( row + ii, col + jj ) = src( ii, jj );
      }
    }
  }
}

// ===================================================================
// subvector(): Extract an Nx1 or 1xN submatrix.
// ===================================================================
void PrMatrix::subvector( int row, int col, PrVector& dest,
                          bool fHorizontal ) const
{
  SAIAssert( row >= 0 && row <= m_row );
  SAIAssert( col >= 0 && col <= m_col );

  if ( fHorizontal )
  {
    SAIAssert( col + dest.size() <= m_col );
    for( int jj = 0; jj < dest.size(); jj++ )
    {
      dest[jj] = elementAt( row, col + jj );
    }
  }
  else
  {
    SAIAssert( row + dest.size() <= m_row );
    for( int ii = 0; ii < dest.size(); ii++ )
    {
      dest[ii] = elementAt( row + ii, col );
    }
  }
}

// ===================================================================
// setSubvector(): Copy a vector into a subset of this matrix.
// ===================================================================
void PrMatrix::setSubvector( int row, int col, const PrVector& src,
                             bool fHorizontal )
{
  SAIAssert( row >= 0 && row <= m_row );
  SAIAssert( col >= 0 && col <= m_col );

  if ( fHorizontal )
  {
    SAIAssert( col + src.size() <= m_col );
    for( int jj = 0; jj < src.size(); jj++ )
    {
      elementAt( row, col + jj ) = src[jj];
    }
  }
  else
  {
    SAIAssert( row + src.size() <= m_row );
    for( int ii = 0; ii < src.size(); ii++ )
    {
      elementAt( row + ii, col ) = src[ii];
    }
  }
}

// ===================================================================
// subset(): Set this matrix to one of its own submatrices.  The new
// dimensions are (rowEnd - rowStart) x (colEnd - colStart).
//
// Do not reallocate memory -- simply shift elements and reset the
// dimensions.
// ===================================================================

void PrMatrix::subset( int rowStart, int colStart, int rowEnd, int colEnd )
{
  SAIAssert( 0 <= rowStart && rowStart < rowEnd && rowEnd <= m_row );
  SAIAssert( 0 <= colStart && colStart < colEnd && colEnd <= m_col );

  if( !(rowStart == 0 && colStart == 0 && colEnd == m_col) )
  {
    int index = 0;
    for( int iRow = rowStart; iRow < rowEnd; iRow++ )
    {
      for( int iCol = colStart; iCol < colEnd; iCol++ )
      {
        m_data[index] = m_data[iRow * m_col + iCol];
        ++index;
      }
    }
  }

  m_row = rowEnd - rowStart;
  m_col = colEnd - colStart;
  m_size = m_row * m_col;
}

// ===================================================================
// Equality operator: This operation tests to see if rhs has the same
// size and contents of this matrix.  The data must match exactly;
// there is no epsilon tolerance.
// ===================================================================
bool PrMatrix::operator==( const PrMatrix& rhs ) const
{
  if( m_data == rhs.m_data )
  {
    return true;
  }
  else if( m_row != rhs.m_row || m_col != rhs.m_col )
  {
    return false;
  }
  else if( m_size == 0 )
  {
    return true;
  }
  else
  {
    return (memcmp(m_data, rhs.m_data, sizeof( Float ) * m_size ) == 0);
  }
}

// ===================================================================
// Arithmetic operations
// ===================================================================

void PrMatrix::negate( PrMatrix& dest ) const
{
  dest.resize( m_row, m_col );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = -m_data[ii];
  }
}

void PrMatrix::add( const PrMatrix& rhs, PrMatrix& dest ) const
{
  SAIAssert( m_row == rhs.m_row  && m_col == rhs.m_col );
  dest.resize( m_row, m_col );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = m_data[ii] + rhs.m_data[ii];
  }
}

void PrMatrix::add( const PrMatrix& rhs )
{
  SAIAssert( m_row == rhs.m_row && m_col == rhs.m_col );
  for( int ii = 0; ii < m_size; ii++ )
  {
    m_data[ii] += rhs.m_data[ii];
  }
}

void PrMatrix::subtract( const PrMatrix& rhs, PrMatrix& dest ) const
{
  SAIAssert( m_row == rhs.m_row && m_col == rhs.m_col );
  dest.resize( m_row, m_col );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = m_data[ii] - rhs.m_data[ii];
  }
}

void PrMatrix::subtract( const PrMatrix& rhs )
{
  SAIAssert( m_row == rhs.m_row && m_col == rhs.m_col );
  for( int ii = 0; ii < m_size; ii++ )
  {
    m_data[ii] -= rhs.m_data[ii];
  }
}

// ===================================================================
// multiply(): Multiply by an (optionally-transposed) matrix.
// ===================================================================
void PrMatrix::multiply( const PrMatrix& rhs,
                         PrMatrix& dest, bool fTranspose ) const
{
  if( !fTranspose )
  {
    SAIAssert( m_col == rhs.m_row );
    SAIAssert( &dest != &rhs && &dest != this );
    dest.resize( m_row, rhs.m_col );
    for( int ii = 0; ii < dest.m_row; ii++ )
    {
      Float* lhsRow = &m_data[m_col * ii];
      for( int jj = 0; jj < dest.m_col; jj++ )
      {
        Float* rhsCol = &rhs.m_data[jj];
        Float destIJ = 0;
        for( int kk = 0; kk < m_col; kk++ )
        {
          destIJ += lhsRow[ kk ] * rhsCol[ rhs.m_col * kk ];
        }
        dest( ii, jj ) = destIJ;
      }
    }
  }

  // transpose the rhs
  else
  {
    SAIAssert( m_col == rhs.m_col );
    SAIAssert( &dest != &rhs && &dest != this );
    dest.resize( m_row, rhs.m_row );
    for( int ii = 0; ii < dest.m_row; ii++ )
    {
      Float* lhsRow = &m_data[m_col * ii];
      for( int jj = 0; jj < dest.m_col; jj++ )
      {
        Float* rhsRow = &rhs.m_data[rhs.m_col * jj];
        Float destIJ = 0;
        for( int kk = 0; kk < m_col; kk++ )
        {
          destIJ += lhsRow[kk] * rhsRow[kk];
        }
        dest( ii, jj ) = destIJ;
      }
    }
  }
}

// ===================================================================
// multiply(): Multiply by an (optionally-transposed) matrix, and
// store the result in the original matrix.
//
// If possible, do the multiplication in-place for some strange
// reason.
// ===================================================================
void PrMatrix::multiply( const PrMatrix& rhs, bool fTranspose, PrVector* pTmpStore )
{
  if( rhs.m_row != rhs.m_col )
  {
    // If rhs is not square, cannot multiply in-place
    PrMatrix res( m_row, fTranspose ? rhs.m_row : rhs.m_col );
    multiply( rhs, res, fTranspose );
    transfer( res );
  }
  else if( !fTranspose )
  {
    // Multiply this * rhs in-place
    SAIAssert( m_col == rhs.m_row );
    SAIAssert( &rhs != this );
    PrVector tmp;
    PrVector& tempRow = pTmpStore != NULL ? *pTmpStore : tmp;
    tempRow.setSize( m_col );

    for( int ii = 0; ii < m_row; ii++ )
    {
      for( int jj = 0; jj < m_col; jj++ )
      {
        Float resIJ = 0.0;
        for( int kk = 0; kk < m_col; kk++ )
        {
          resIJ += elementAt(ii, kk) * rhs.elementAt(kk, jj);
        }
        tempRow[jj] = resIJ;
      }
      for( int jj = 0; jj < m_col; jj++ )
      {
        elementAt(ii, jj) = tempRow[jj];
      }
    }
  }
  else
  {
    // Multiply this * rhs^T in-place
    SAIAssert( m_col == rhs.m_col );
    SAIAssert( &rhs != this );
    PrVector tmp;
    PrVector& tempRow = pTmpStore != NULL ? *pTmpStore : tmp;
    tempRow.setSize( m_col );

    for( int ii = 0; ii < m_row; ii++ )
    {
      for( int jj = 0; jj < rhs.m_row; jj++ )
      {
        Float resIJ = 0.0;
        for( int kk = 0; kk < m_col; kk++ )
        {
          resIJ += elementAt(ii, kk) * rhs.elementAt(jj, kk);
        }
        tempRow[jj] = resIJ;
      }
      for( int jj = 0; jj < m_col; jj++ )
      {
        elementAt(ii, jj) = tempRow[jj];
      }
    }
  }
}

// ===================================================================
// multiply(): Multiply by a vector
// ===================================================================
void PrMatrix::multiply( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( m_col == rhs.size() );
  SAIAssert( &dest != &rhs );
  dest.setSize( m_row );

  for( int ii = 0; ii < m_row; ii++ )
  {
    Float destI = 0.0;
    for( int jj = 0; jj < m_col; jj++ )
    {
      destI += elementAt(ii, jj) * rhs[jj];
    }
    dest[ii] = destI;
  }
}

// ===================================================================
// multiply(): Multiply by a scalar
// ===================================================================
void PrMatrix::multiply( Float rhs, PrMatrix& dest ) const
{
  dest.resize( m_row, m_col );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = m_data[ii] * rhs;
  }
}

void PrMatrix::multiply( Float rhs )
{
  for( int ii = 0; ii < m_size; ii++ )
  {
    m_data[ii] *= rhs;
  }
}

// ===================================================================
// multiplyTranspose(): Multiply this^T by a matrix
// ===================================================================
void PrMatrix::multiplyTranspose(const PrMatrix& rhs, PrMatrix& dest) const
{
  SAIAssert( m_row == rhs.m_row );
  SAIAssert( &dest != &rhs && &dest != this );
  dest.resize( m_col, rhs.m_col );

  for( int ii = 0; ii < dest.m_row; ii++)
  {
    for( int jj = 0; jj < dest.m_col; jj++)
    {
      Float destIJ = 0.0;
      for( int kk = 0; kk < m_row; kk++)
      {
        destIJ += elementAt(kk, ii) * rhs.elementAt(kk, jj);
      }
      dest.elementAt(ii, jj) = destIJ;
    }
  }
}

// ===================================================================
// multiplyTranspose(): Multiply this^T by a vector
// ===================================================================
void PrMatrix::multiplyTranspose(const PrVector& rhs, PrVector& dest) const
{
  SAIAssert( m_row == rhs.size() );
  SAIAssert( &dest != &rhs );
  dest.setSize( m_col );

  for( int jj = 0; jj < m_col; jj++ )
  {
    Float destJ = 0.0;
    for( int ii = 0; ii < m_row; ii++ )
    {
      destJ += elementAt(ii, jj) * rhs[ii];
    }
    dest[jj] = destJ;
  }
}

// ===================================================================
// transpose():   [0 1 2; 3 4 5; 6 7 8]^T = [0 3 6; 1 4 7; 2 5 8]
// ===================================================================
void PrMatrix::transpose( PrMatrix& dest ) const
{
  SAIAssert( &dest != this );
  dest.resize( m_col, m_row );
  
  for( int ii = 0; ii < m_row; ii++ )
  {
    for( int jj = 0; jj < m_col; jj++ )
    {
      dest.elementAt(jj, ii) = elementAt(ii, jj);
    }
  }
}

// ===================================================================
// inverse(): Matrix Inverse by Crout's LU decomposition
// p275-285, Numerical Methods for Engineers by Chapra and Canale
// ===================================================================
void PrMatrix::inverse( PrMatrix& ainv ) const
{
  SAIAssert( m_row == m_col );
  ainv.resize( m_row, m_col );

  PrMatrix lu( NULL, m_row, m_col );
  PrVector x( NULL, m_col );
  PrVector y( m_col ); // y must be mostly 0

  LUdecomp( lu );

  for( int jj = 0; jj < m_col; jj++ )
  {
    y[jj] = 1;  // All other elements of y are zero
    lu.backSub( y, x );
    for( int ii = 0; ii < m_row; ii++ )
    {
      ainv.elementAt( ii, jj ) = x[ii];
    }
    y[jj] = 0;
  }
}

// ===================================================================
// inverseSPD(): Inverse for symmetric positive definite matrix
// by Cholesky's LU decomposition
// p288-290, Numerical Methods for Engineers by Chapra and Canale
// ===================================================================
void PrMatrix::inverseSPD( PrMatrix& ainv ) const
{
  SAIAssert( m_row == m_col );
  ainv.resize( m_row, m_col );

  PrMatrix lu( NULL, m_row, m_col );
  PrVector x( NULL, m_col );
  PrVector y( m_col ); // y must be mostly 0

  LUdecompSPD( lu );

  for( int jj = 0; jj < m_col; jj++ )
  {
    y[jj] = 1;  // All other elements of y are zero
    lu.backSubSPD( y, x );
    for( int ii = 0; ii < m_row; ii++ )
    {
      ainv.elementAt( ii, jj ) = x[ii];
    }
    y[jj] = 0;
  }
}

#ifndef PR_MISSING_NEWMAT10_LIB
// ===================================================================
// svd():  Singular value decomposition, using the newmat10 library.
// matrix = U * D * V^T
//
// Unlike most matrix functions, svd() resizes the output arguments
// for you.
// ===================================================================
void PrMatrix::svd( PrMatrix& U, PrMatrix& D, PrMatrix& V ) const
{
  // Resize output matrices
  U.resize( m_row, m_row );
  D.resize( m_row, m_col );
  V.resize( m_col, m_col );

  // SVD() works only if rows >= cols.  If rows < cols, transpose the matrix
  bool flip = (m_row < m_col);

  Matrix         nm;
  Matrix         nm_U;
  DiagonalMatrix nm_D;
  Matrix         nm_V;

  copyToNewmat( nm, flip );

  SVD( nm, nm_D, nm_U, nm_V );

  if ( flip )
  {
    U.copyFromNewmat( nm_V );
    D.copyFromNewmat( nm_D );
    V.copyFromNewmat( nm_U );
  }
  else
  {
    U.copyFromNewmat( nm_U );
    D.copyFromNewmat( nm_D );
    V.copyFromNewmat( nm_V );
  }
}

// ===================================================================
// eig():  Eigenvalue decomposition, using the newmat10 library.
// matrix = V * D * V^T
// ===================================================================
void PrMatrix::eig( PrMatrix& D, PrMatrix& V ) const
{
  SAIAssert( m_row == m_col );
 
  // Resize output matrices
  D.resize( m_row, m_row );
  V.resize( m_row, m_row );

  SymmetricMatrix nm;
  DiagonalMatrix nm_D;
  Matrix nm_V;

  copyToNewmat( nm );
  EigenValues( nm, nm_D, nm_V );
  D.copyFromNewmat( nm_D );
  V.copyFromNewmat( nm_V );
}

// ===================================================================
// rank(): Calculate the rank of the matrix, by calculating the SVD
// and counting the number of singular values that exceed tol.  tol is
// scaled by the largest singular value.
// ===================================================================
int PrMatrix::rank(float tol) const
{
  // Compute USV.
  PrMatrix U, S, V;
  svd( U, S, V );

  // Get lowest dimension.
  int dim; 
  if( m_row <= m_col )
  {
    dim = m_row;
  }
  else
  {
    dim = m_col;
  }

  // Compute rank.
  int rank = 0;

  if( S( 0, 0 ) > tol )
  {
    rank++;
    Float scaledTol = tol * S( 0, 0 );

    for( int ii = 1; ii < dim; ii++ )
    {
      if(  S( ii, ii ) > scaledTol )
      {
        rank++;
      }
    }
  }

  return rank;
}

// ===================================================================
// pseudoInverse(): Calculate the pseudo-inverse of a matrix, by
// calculating the SVD and inverting the singular values that exceed
// tol.  tol is scaled by the largest singular value.
// ===================================================================
void PrMatrix::pseudoInverse( PrMatrix& dest, Float tol ) const
{
  // Compute USV.
  PrMatrix U, S, V;
  svd( U, S, V );

  // Get lowest dimension.
  int dim; 
  if( m_row <= m_col )
  {
    dim = m_row;
  }
  else
  {
    dim = m_col;
  }

  // Compute rank.
  int rank = 0;

  if( S( 0, 0 ) > tol )
  {
    rank++;
    Float scaledTol = tol * S( 0, 0 );

    for( int ii = 1; ii < dim; ii++ )
    {
      if(  S( ii, ii ) > scaledTol )
      {
        rank++;
      }
    }
  }

  // Calculate minimal U and V, and invert S
  PrMatrix Umin( NULL, m_row, rank );
  PrMatrix Vmin( NULL, m_col, rank );
  PrMatrix Sinv( rank, rank ); // Sinv must be initialized to 0

  U.submatrix( 0, 0, Umin );
  V.submatrix( 0, 0, Vmin );

  for( int ii = 0; ii < rank; ii++ )
  {
    Sinv.elementAt( ii, ii ) = 1/( S.elementAt( ii, ii ) );
  }

  // Compute pseudo inverse:
  // dest = Vmin * Sinv * Umin.transpose();
  Vmin.multiply( Sinv );
  Vmin.multiply( Umin, true/*transpose*/ );// Umin.transpose() ); 
  dest.transfer( Vmin );
}


// ===================================================================
// cappedPseudoInverse(): Calculate the pseudo-inverse of a matrix, by
// calculating the SVD and inverting the singular values.
//
// This routine differs from pseudoInverse() in the way it handles
// rank-deficient matrices.  pseudoInverse() rounds the near-zero
// singular values down to zero, instead of inverting them.
// cappedPseudoInverse rounds the near-zero singular values *up* to
// 1e-3, and then inverts them.
//
// Thus, if eps is a very small number, the pseudoInverse of
// [eps 0; 0 eps] is [0 0; 0 0], but the cappedPseudoInverse is
// [1000 0; 0 1000].
// ===================================================================
void PrMatrix::cappedPseudoInverse( PrMatrix& dest ) const
{
  static const Float MIN_SV = 1e-3;

  // Compute USV.
  PrMatrix U, S, V;
  svd( U, S, V );

  // Get lowest dimension.
  int dim; 
  if( m_row <= m_col )
  {
    dim = m_row;
  }
  else
  {
    dim = m_col;
  }

  // Round the near-zero diagonals of S up to +MIN_SV,
  // or down to -MIN_SV
  for( int ii = 0; ii < dim; ii++ )
  {
    if(  S( ii, ii ) < MIN_SV && S( ii, ii ) > -MIN_SV )
    {
      S( ii, ii ) = ( S( ii, ii ) >= 0 ? MIN_SV : -MIN_SV );
    }
  }

  // Calculate dest = V * S^-1 * U^T
  for( int ii = 0; ii < m_col; ii++ )
  {
    double invS = 1.0 / S( ii, ii );
    for( int jj = 0; jj < m_col; jj++ )
    {
      V( jj, ii ) *= invS;
    }
  }

  V.multiply( U, true /*transpose*/ );
  dest.transfer( V );
}
#endif // PR_MISSING_NEWMAT10_LIB

// ===================================================================
// display():  Display the matrix.
// ===================================================================
void PrMatrix::display( const char* name ) const
{
  if( name != NULL )
  {
    printf( "%s =\n (", name );
  }
  else
  {
    printf( " (", name );
  }

  for( int ii = 0; ii < m_row; ii++ )
  {
    if( ii > 0 )
    {
       printf( ")\n (" );
    }
    for( int jj = 0; jj < m_col; jj++ )
    {
      if( jj > 0 )
      {
        printf( " %10.4g", elementAt(ii, jj) );
      }
      else
      {
        printf( "%10.4g", elementAt(ii, jj) );
      }
    }
  }
  printf(")\n");
}

// ===================================================================
// LUdecomp(): Perform an LU decomposition of a square, full-rank
// matrix.  L and U are combined into a single matrix, with the
// convention that U has a diagonal of [1 1 1 ...].
//
// Warning:  This routine does no pivoting.
// ===================================================================
void PrMatrix::LUdecomp( PrMatrix& lu ) const
{
  SAIAssert( m_row == m_col );
  SAIAssert( &lu != this );
  lu.resize( m_row, m_col );

  Float sum;

  for( int jj = 0; jj < m_col; jj++ )
  {
    for( int ii = jj; ii < m_row; ii++ )
    {
      sum = 0.0;
      for( int kk = 0; kk < jj; kk++ )
      {
        sum += lu( ii, kk ) * lu( kk, jj );
      }
      lu( ii, jj ) = elementAt( ii, jj ) - sum;  /* L */
    }
    for( int kk = jj + 1; kk < m_col; kk++ )
    {
      sum = 0.0;
      for( int ii = 0; ii < jj; ii++ )
      {
        sum += lu( jj, ii ) * lu( ii, kk );
      }
      lu( jj, kk ) = ( elementAt( jj, kk ) - sum ) / lu( jj, jj ); /* U */
    }
  }
}

// ===================================================================
// LUdecompSPD(): Perform a Cholesky LU decomposition of a symmetric
// positive definite matrix.  In a Cholesky, U = L^T.
// ===================================================================
void PrMatrix::LUdecompSPD( PrMatrix& lu ) const
{
  SAIAssert( m_row == m_col );
  SAIAssert( &lu != this );
  lu.resize( m_row, m_col );

  Float sum;

  for( int kk = 0; kk < m_col; kk++ )
  {
    // Calculate off-diagonal elements in row kk / column kk
    for( int ii = 0; ii < kk; ii++ )
    {
      sum = 0.0;
      for( int jj = 0; jj < ii; jj++ )
      {
        sum += lu( ii, jj ) * lu( kk, jj );
      }
      lu( kk, ii ) = (elementAt( kk, ii ) - sum ) / lu( ii, ii );
      lu( ii, kk ) = lu( kk, ii );
    }

    // Calculate diagonal element
    sum = 0.0;
    for( int jj = 0; jj < kk; jj++ )
    {
      sum += lu( kk, jj ) * lu( kk, jj );
    }
    lu( kk, kk ) = sqrt( elementAt( kk, kk ) - sum );
  }
}

// ===================================================================
// backSub(): Solve (LU x = y) for x.  This matrix must be in LU form,
// as returned by LUdecomp().
// ===================================================================
void PrMatrix::backSub( const PrVector& y, PrVector& x ) const
{
  SAIAssert( m_row == m_col );
  SAIAssert( m_row == y.size() );
  x.setSize( m_row );

  Float sum;
  PrVector d( NULL, m_row );

  // Solve (L d = y) for d
  for( int ii = 0; ii < m_row; ii++ )
  {
    sum = 0.0;
    for( int jj = 0; jj < ii; jj++ )
    {
      sum += elementAt( ii, jj ) * d[jj];
    }
    d[ii] = ( y[ii] - sum ) / elementAt( ii, ii );
  }

  // Solve (U x = d) for x
  for( int ii = m_row - 1; ii >= 0; ii-- )
  {
    sum = 0.0;
    for( int jj = ii + 1; jj < m_col; jj++ )
    {
      sum += elementAt( ii, jj ) * x[jj];
    }
    x[ii] = d[ii] - sum;
  }
}

// ===================================================================
// backSubSPD(): Solve (LU x = y) for x.  This matrix must be in
// Cholesky LU form, as returned by LUdecompSPD().
// ===================================================================
void PrMatrix::backSubSPD( const PrVector& y, PrVector& x ) const
{
  SAIAssert( m_row == m_col );
  SAIAssert( m_row == y.size() );
  x.setSize( m_row );

  Float sum;
  PrVector d( NULL, m_row );

  // Solve (L d = y) for d
  for( int ii = 0; ii < m_row; ii++ )
  {
    sum = 0.0;
    for( int jj = 0; jj < ii; jj++ )
    {
      sum += elementAt( ii, jj ) * d[jj];
    }
    d[ii] = ( y[ii] - sum ) / elementAt( ii, ii );
  }

  // Solve (U x = d) for x
  for( int ii = m_row - 1; ii >= 0; ii-- )
  {
    sum = 0.0;
    for( int jj = ii + 1; jj < m_col; jj++ )
    {
      sum += elementAt( ii, jj ) * x[jj];
    }
    x[ii] = ( d[ii] - sum ) / elementAt( ii, ii );
  }
}

#ifndef PR_MISSING_NEWMAT10_LIB
// ===================================================================
// copyToNewmat(): Copy a PrMatrix to a newmat10 matrix.
//
// Note that newmat10 matrices all use 1-based indexing, i.e. the
// upper-left corner is (1,1), not (0,0).
// ===================================================================
void PrMatrix::copyToNewmat( Matrix& dest, bool fTranspose ) const
{
  if( !fTranspose )
  {
    dest.ReSize( m_row, m_col );
    for( int ii = 0; ii < m_row; ii++ )
    {
      for( int jj = 0; jj < m_col; jj++ )
      {
        dest( ii + 1, jj + 1 ) = elementAt( ii, jj );
      }
    }
  }
  else
  {
    dest.ReSize( m_col, m_row );
    for( int ii = 0; ii < m_row; ii++ )
    {
      for( int jj = 0; jj < m_col; jj++ )
      {
        dest( jj + 1, ii + 1 ) = elementAt( ii, jj );
      }
    }
  }
}

void PrMatrix::copyToNewmat( SymmetricMatrix& dest ) const
{
  dest.ReSize( m_row );
  for( int ii = 0; ii < m_row; ii++ )
  {
    for( int jj = 0; jj < m_row; jj++ )
    {
      dest( ii + 1, jj + 1 ) = elementAt( ii, jj );
    }
  }
}

// ===================================================================
// copyFromNewmat(): Load a PrMatrix from a newmat10 matrix.
//
// Note that newmat10 matrices all use 1-based indexing, i.e. the
// upper-left corner is (1,1), not (0,0).
// ===================================================================
void PrMatrix::copyFromNewmat( Matrix& src, bool fTranspose )
{
  if( !fTranspose )
  {
    for( int ii = 0; ii < m_row; ii++ )
    {
      for( int jj = 0; jj < m_col; jj++ )
      {
        elementAt( ii, jj ) = src( ii + 1, jj + 1 );
      }
    }
  }
  else
  {
    for( int ii = 0; ii < m_row; ii++ )
    {
      for( int jj = 0; jj < m_col; jj++ )
      {
        elementAt( ii, jj ) = src( jj + 1, ii + 1 );
      }
    }
  }
}

void PrMatrix::copyFromNewmat( DiagonalMatrix& src )
{
  zero();
  for( int ii = 0; ii < m_row; ii++ )
  {
    elementAt( ii, ii ) = src( ii + 1, ii + 1 );
  }
}
#endif // PR_MISSING_NEWMAT10_LIB

// ===================================================================
// dynamic(): This virtual function is used to determine whether
// m_data was dynamically allocated.  It returns true for PrMatrix,
// and false for PrMatrix3 and PrMatrix6.
//
// If dynamic() is false, then it is illegal to delete m_data, or to
// transfer it to another matrix.
// ===================================================================
bool PrMatrix::dynamic() const
{
  return true;
}

// ===================================================================
// resize(): This virtual function resizes the matrix without
// initializing the data.  It is used by setSize(), the assignment
// operator, etc.
//
// This function is normally overridden in the subclass.  For example,
// since a PrMatrix3 is *always* 3x3, PrMatrix3 overrides this
// function to throw an assertion unless row == col == 3.
// ===================================================================
void PrMatrix::resize( int row, int col )
{
  SAIAssert( row >= 0 && col >= 0 );

  if( m_row != row || m_col != col )
  {
    int newSize = row * col;

    if ( newSize > m_maxSize )
    {
      if ( m_data != NULL )
      {
        delete[] m_data;
      }
      m_data = new Float[newSize];
      m_maxSize = newSize;
    }

    m_row = row;
    m_col = col;
    m_size = newSize;
  }
}



// ===================================================================
// fIdentity(): test if it is an identity matrix
// ===================================================================
bool PrMatrix::fIdentity() const
{
  if( m_row == m_col )
  {
    for( int ii = 0; ii < m_row; ii++ )
    {
      for( int jj = 0; jj < m_col; jj++ )
      {
        Float elem = elementAt( ii, jj );
        if( (ii == jj && elem != 1) || (ii != jj || elem != 0) )
          return false;
      }
    }
    return true;
  }
  return false;
}


// ===================================================================
// fZero(): test if it is a zero matrix
// ===================================================================
bool PrMatrix::fZero() const
{
  for( int ii = 0; ii < m_row; ii++ )
    for( int jj = 0; jj < m_col; jj++ )
      if( elementAt( ii, jj ) != 0 )
        return false;

  return true;
}


// ===================================================================
// preMultiply(): *this = mult x *this
// acpects multiplier not to change dimensionality of the matrix
// after the multiplication (mult is a square.)
// ===================================================================
void PrMatrix::preMultiply( const PrMatrix& mult, PrVector* pTmpStore )
{
  SAIAssert( mult.row() == mult.column() && mult.column() == m_row );
  PrVector tmp;
  PrVector& tmpColumn = pTmpStore == NULL ? tmp : *pTmpStore;
  tmpColumn.setSize( m_row );

  for( int jj = 0; jj < m_col; jj++ )
  {
    this->subvector( 0, jj, tmpColumn );

    for( int ii = 0; ii < mult.m_row; ii++ )
    {
      Float destIJ = 0;
      for( int kk = 0; kk < mult.m_col; kk++ )
      {
        destIJ += mult.elementAt(ii, kk) * tmpColumn[kk];
      }
      this->elementAt( ii, jj ) = destIJ;
    }
  }

}


// ===================================================================
// PrMappedMatrix implementation
// ===================================================================
PrMappedMatrix::PrMappedMatrix( PrMatrix& src, int row, int cRows )
{
  SAIAssert( row < src.row() && row+cRows <= src.row() );

  m_data = &src.elementAt( row, 0 );
  m_row = cRows;
  m_col = src.column();
  m_size = m_row*m_col;
}

PrMappedMatrix::~PrMappedMatrix()
{
  m_data = NULL;
}

bool PrMappedMatrix::dynamic() const
{
  return false;
}

void PrMappedMatrix::resize( int row, int col )
{
  SAIAssert( m_row == row && m_col == col );
}
