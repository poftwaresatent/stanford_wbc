/*
 * saimatrix -- utility library for stanford-wbc.sourceforge.net
 *
 * Copyright (c) 1997-2008 Stanford University. All rights reserved.
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
// SAIMatrix.cpp
//
// Implementation of an MxN matrix.
//
// modification history
// --------------------
//
// 01/24/08: Luis Sentis: Removed CLAPCACK for compatibility with QNX
// 01/23/06: James Warren: Modified to replace newmat with CLAPACK
// 06/17/04: Dan Merget: Made SAIMatrix3 and SAIMatrix6 into subclasses
// 11/12/97: K.C. Chang: created
// *******************************************************************

#include <saimatrix/SAIMatrix.h>
#include "SAIVector3.h"
#include <sstream>

using namespace std;

#ifdef _WIN32
  #include <float.h>
  #define finite(num) _finite(num)
#endif

// ===================================================================
// Constructors & Destructors
// ===================================================================
SAIMatrix::SAIMatrix( int row, int col )
  : m_row( row ), m_col( col ), m_size( row*col ), m_maxSize( row*col )
{
  assert( m_row >= 0 && m_col >= 0 );

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

SAIMatrix::SAIMatrix( const SAIMatrix& rhs )
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

SAIMatrix::SAIMatrix( const Float* rgVals, int row, int col, bool fTranspose )
  : m_row( row ), m_col( col ), m_size( row*col ), m_maxSize( row*col )

{
  assert( m_row >= 0 && m_col >= 0 );

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

SAIMatrix::~SAIMatrix()
{
  if( m_data != NULL )
    {
      ////DO NOT COMMIT THIS////
      ////DO NOT COMMIT THIS////
      ////DO NOT COMMIT THIS////
      ////DO NOT COMMIT THIS////
      ////DO NOT COMMIT THIS////
      ////DO NOT COMMIT THIS////      delete[] m_data;
      ////DO NOT COMMIT THIS////
      ////DO NOT COMMIT THIS////
      ////DO NOT COMMIT THIS////
      ////DO NOT COMMIT THIS////
      ////DO NOT COMMIT THIS////
    }
}

// ===================================================================
// fFinite(): Return true if all of the elements are finite, or false
// if any element is infinite or NaN.
// ===================================================================
bool SAIMatrix::fFinite() const
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
SAIMatrix& SAIMatrix::operator=( const SAIMatrix& rhs )
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
void SAIMatrix::setValues( const Float* rgVals, bool fTranspose )
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
// setConstantValue(): resizes matrix and sets all elements of the 
// matrix to the given value
// ===================================================================
void SAIMatrix::setConstantValue( Float value, int iRow, int iCol )
{
  if( (iRow > 0) && (iCol > 0) ) {
    resize(iRow, iCol);
  }
  for( int ii = 0; ii < m_col; ii++ ) {
    for( int jj = 0; jj < m_row; jj++ ) {
      elementAt(ii, jj) = value;
    }
  }
}

// ===================================================================
// getValues(): Copy values from this matrix to rgVals.  Use the size
// of the matrix to determine how many values to copy.  rgVals is
// assumed to be in row-major form if fTranspose is true, or
// column-major form otherwise.
// ===================================================================
void SAIMatrix::getValues( Float* rgVals, bool fTranspose ) const
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
void SAIMatrix::appendVertically( const SAIMatrix& rhs )
{
  if ( rhs.m_size > 0 )
    {
      assert( m_col == rhs.m_col || m_size == 0 );
      SAIMatrix ans( m_row + rhs.m_row, rhs.m_col );

      ans.setSubmatrix( 0, 0, *this );
      ans.setSubmatrix( m_row, 0, rhs );
      transfer( ans );
    }
}

void SAIMatrix::appendHorizontally( const SAIMatrix& rhs )
{
  if ( rhs.m_size > 0 )
    {
      assert( m_row == rhs.m_row || m_size == 0 );
      SAIMatrix ans( rhs.m_row, m_col + rhs.m_col );

      ans.setSubmatrix( 0, 0, *this );
      ans.setSubmatrix( 0, m_col, rhs );
      transfer( ans );
    }
}

// ===================================================================
// setSize():  Resize the matrix and optionally zero it.
// ===================================================================
void SAIMatrix::setSize( int row, int col, bool fZero )
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
// However, if either matrix is non-dynamic (e.g. a SAIMatrix3), then
// the data is simply copied.
// ===================================================================
void SAIMatrix::transfer( SAIMatrix& src )
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
void SAIMatrix::diagonal( SAIVector& dest ) const
{
  assert( m_row == m_col );
  dest.setSize( m_row );
  for( int ii = 0; ii < m_row; ii++ )
    {
      dest[ii] = elementAt( ii, ii );
    }
}

// ===================================================================
// setDiagonal(): Set the elements on the diagonal of a square matrix.
// ===================================================================
void SAIMatrix::setDiagonal( Float src )
{
  assert( m_row == m_col );
  for( int ii = 0; ii < m_row; ii++ )
    {
      elementAt(ii, ii) = src;
    }
}

void SAIMatrix::setDiagonal( const Float* src )
{
  assert( m_row == m_col );
  for( int ii = 0; ii < m_row; ii++ )
    {
      elementAt(ii, ii) = src[ii];
    }
}

void SAIMatrix::setDiagonal( const SAIVector& src )
{
  assert( m_row == m_col );
  assert( m_row == src.size() );
  for( int ii = 0; ii < m_row; ii++ )
    {
      elementAt(ii, ii) = src[ii];
    }
}

// ===================================================================
// getRow() / getColumn(): Extract a row or column from the matrix.
// ===================================================================
void SAIMatrix::getRow( int row, SAIVector& dest ) const
{
  assert( row >= 0 && row < m_row );
  dest.setSize( m_col );
  for( int jj = 0; jj < m_col; jj++ )
    {
      dest[jj] = elementAt( row, jj );
    }
}

void SAIMatrix::getColumn( int col, SAIVector& dest ) const
{
  assert( col >= 0 && col < m_col );
  dest.setSize( m_row );
  for( int ii = 0; ii < m_row; ii++ )
    {
      dest[ii] = elementAt( ii, col );
    }
}

// ===================================================================
// setRow() / setColumn(): Set a row or column in the matrix.
// ===================================================================
void SAIMatrix::setRow( int row, const SAIVector& src )
{
  assert( row >= 0 && row < m_row );
  assert( src.size() == m_col );
  for( int jj = 0; jj < m_col; jj++ )
    {
      elementAt( row, jj ) = src[jj];
    }
}

void SAIMatrix::setColumn( int col, const SAIVector& src )
{
  assert( col >= 0 && col < m_col );
  assert( src.size() == m_row );
  for( int ii = 0; ii < m_row; ii++ )
    {
      elementAt( ii, col ) = src[ii];
    }
}

void SAIMatrix::setSubmatrix( Float val, int iRow, int iCol, int cRow, int cCol )
{
  for( int ii = 0; ii < cRow; ii++ )
    for( int jj = 0; jj < cCol; jj++ )
      {
	elementAt( ii + iRow, jj + iCol ) = val;
      }
}

void SAIMatrix::zeroSubmatrix( int iRow, int iCol, int cRows, int cCols )
{
  int endRow = iRow + cRows;
  int endCol = iCol + cCols;
  assert( endRow <= m_row && endCol <= m_col );

  for( int ii = iRow; ii < endRow; ii++ )
    for( int jj = iCol; jj < endCol; jj++ )
      elementAt( ii, jj) = 0;
}

// ===================================================================
// submatrix(): Extract a submatrix.  The "row" & "col" arguments give
// the coordinates of the submatrix, and the dimensions of "dest" give
// the size of the submatrix.
// ===================================================================
void SAIMatrix::submatrix( int row, int col, SAIMatrix& dest ) const
{
  assert( row >= 0 && row + dest.m_row <= m_row );
  assert( col >= 0 && col + dest.m_col <= m_col );

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
void SAIMatrix::setSubmatrix( int row, int col, const SAIMatrix& src )
{
  assert( row >= 0 && row + src.m_row <= m_row );
  assert( col >= 0 && col + src.m_col <= m_col );

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
void SAIMatrix::subvector( int row, int col, SAIVector& dest,
			   bool fHorizontal ) const
{
  assert( row >= 0 && row <= m_row );
  assert( col >= 0 && col <= m_col );

  if ( fHorizontal )
    {
      assert( col + dest.size() <= m_col );
      for( int jj = 0; jj < dest.size(); jj++ )
	{
	  dest[jj] = elementAt( row, col + jj );
	}
    }
  else
    {
      assert( row + dest.size() <= m_row );
      for( int ii = 0; ii < dest.size(); ii++ )
	{
	  dest[ii] = elementAt( row + ii, col );
	}
    }
}

// ===================================================================
// setSubvector(): Copy a vector into a subset of this matrix.
// ===================================================================
void SAIMatrix::setSubvector( int row, int col, const SAIVector& src,
			      bool fHorizontal )
{
  assert( row >= 0 && row <= m_row );
  assert( col >= 0 && col <= m_col );

  if ( fHorizontal )
    {
      assert( col + src.size() <= m_col );
      for( int jj = 0; jj < src.size(); jj++ )
	{
	  elementAt( row, col + jj ) = src[jj];
	}
    }
  else
    {
      assert( row + src.size() <= m_row );
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
void SAIMatrix::subset( int rowStart, int colStart, int rowEnd, int colEnd )
{
  assert( 0 <= rowStart && rowStart < rowEnd && rowEnd <= m_row );
  assert( 0 <= colStart && colStart < colEnd && colEnd <= m_col );

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
bool SAIMatrix::operator==( const SAIMatrix& rhs ) const
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


bool SAIMatrix::
equal(SAIMatrix const & rhs, Float precision) const
{
  if (&rhs == this)
    return true;
  if (rhs.m_data == m_data)
    return true;
  if (rhs.m_row != m_row)
    return false;
  if (rhs.m_col != m_col)
    return false;
  for (int ii(0); ii < m_size; ++ii)
    if (fabs(m_data[ii] - rhs.m_data[ii]) > precision)
      return false;
  return true;
}


// ===================================================================
// Arithmetic operations
// ===================================================================
void SAIMatrix::negate( SAIMatrix& dest ) const
{
  dest.resize( m_row, m_col );
  for( int ii = 0; ii < m_size; ii++ )
    {
      dest.m_data[ii] = -m_data[ii];
    }
}

void SAIMatrix::add( const SAIMatrix& rhs, SAIMatrix& dest ) const
{
  assert( m_row == rhs.m_row  && m_col == rhs.m_col );
  dest.resize( m_row, m_col );
  for( int ii = 0; ii < m_size; ii++ )
    {
      dest.m_data[ii] = m_data[ii] + rhs.m_data[ii];
    }
}

void SAIMatrix::add( const SAIMatrix& rhs )
{
  assert( m_row == rhs.m_row && m_col == rhs.m_col );
  for( int ii = 0; ii < m_size; ii++ )
    {
      m_data[ii] += rhs.m_data[ii];
    }
}

void SAIMatrix::add( Float rhs, SAIMatrix& dest ) const
{
  dest.resize( m_row, m_col );
  for( int ii = 0; ii < m_size; ii++ )
    {
      dest.m_data[ii] = m_data[ii] + rhs;
    }
}

void SAIMatrix::add( Float rhs )
{
  for( int ii = 0; ii < m_size; ii++ )
    {
      m_data[ii] += rhs;
    }
}

void SAIMatrix::subtract( const SAIMatrix& rhs, SAIMatrix& dest ) const
{
  assert( m_row == rhs.m_row && m_col == rhs.m_col );
  dest.resize( m_row, m_col );
  for( int ii = 0; ii < m_size; ii++ )
    {
      dest.m_data[ii] = m_data[ii] - rhs.m_data[ii];
    }
}

void SAIMatrix::subtract( const SAIMatrix& rhs )
{
  assert( m_row == rhs.m_row && m_col == rhs.m_col );
  for( int ii = 0; ii < m_size; ii++ )
    {
      m_data[ii] -= rhs.m_data[ii];
    }
}

// ===================================================================
// multiply(): Multiply by an (optionally-transposed) matrix.
// ===================================================================
void SAIMatrix::multiply( const SAIMatrix& rhs,
			  SAIMatrix& dest, bool fTranspose ) const
{
  assert( &dest != &rhs && &dest != this );

  if( !fTranspose )
    {
      assert( m_col == rhs.m_row );

      dest.resize( m_row, rhs.m_col );
      int rhsColSize = rhs.m_col;
      int rhsColSize2 = rhsColSize+rhsColSize;
      int rhsColSize3 = rhsColSize2+rhsColSize;
      int rhsColSize4 = rhsColSize3+rhsColSize;
      int rhsColSize5 = rhsColSize4+rhsColSize;
      int rhsColSize6 = rhsColSize5+rhsColSize;
      int rhsColSize7 = rhsColSize6+rhsColSize;
      int rhsColSize8 = rhsColSize7+rhsColSize;

      for( int ii = 0; ii < dest.m_row; ii++ )
	{
	  for( int jj = 0; jj < dest.m_col; jj++ )
	    {
	      Float* lhsRow = &m_data[m_col * ii];
	      Float* rhsCol = &rhs.m_data[jj];
	      Float destIJ = 0;
	      int numTerms = m_col;
	      while( numTerms >= 8 )
		{
		  destIJ += ( lhsRow[0] * rhsCol[0] +
			      lhsRow[1] * rhsCol[rhsColSize] +
			      lhsRow[2] * rhsCol[rhsColSize2] +
			      lhsRow[3] * rhsCol[rhsColSize3] +
			      lhsRow[4] * rhsCol[rhsColSize4] +
			      lhsRow[5] * rhsCol[rhsColSize5] +
			      lhsRow[6] * rhsCol[rhsColSize6] +
			      lhsRow[7] * rhsCol[rhsColSize7] );
		  lhsRow += 8;
		  rhsCol += rhsColSize8;
		  numTerms -= 8;
		}
	      switch( numTerms )
		{
		case 7:
		  destIJ += lhsRow[6] * rhsCol[rhsColSize6];
		  // FALLTHRU
		case 6:
		  destIJ += lhsRow[5] * rhsCol[rhsColSize5];
		  // FALLTHRU
		case 5:
		  destIJ += lhsRow[4] * rhsCol[rhsColSize4];
		  // FALLTHRU
		case 4:
		  destIJ += lhsRow[3] * rhsCol[rhsColSize3];
		  // FALLTHRU
		case 3:
		  destIJ += lhsRow[2] * rhsCol[rhsColSize2];
		  // FALLTHRU
		case 2:
		  destIJ += lhsRow[1] * rhsCol[rhsColSize];
		  // FALLTHRU
		case 1:
		  destIJ += lhsRow[0] * rhsCol[0];
		  // FALLTHRU
		default:
		  break;
		}
	      dest( ii, jj ) = destIJ;
	    }
	}
    }

  // transpose the rhs
  else
    {
      assert( m_col == rhs.m_col );
      dest.resize( m_row, rhs.m_row );
      for( int ii = 0; ii < dest.m_row; ii++ )
	{
	  for( int jj = 0; jj < dest.m_col; jj++ )
	    {
	      Float* lhsRow = &m_data[m_col * ii];
	      Float* rhsRow = &rhs.m_data[rhs.m_col * jj];
	      Float destIJ = 0;
	      int numTerms = m_col;
	      while( numTerms >= 8 )
		{
		  destIJ += ( lhsRow[0] * rhsRow[0] +
			      lhsRow[1] * rhsRow[1] +
			      lhsRow[2] * rhsRow[2] +
			      lhsRow[3] * rhsRow[3] +
			      lhsRow[4] * rhsRow[4] +
			      lhsRow[5] * rhsRow[5] +
			      lhsRow[6] * rhsRow[6] +
			      lhsRow[7] * rhsRow[7] );
		  lhsRow += 8;
		  rhsRow += 8;
		  numTerms -= 8;
		}
	      switch( numTerms )
		{
		case 7:
		  destIJ += lhsRow[6] * rhsRow[6];
		  // FALLTHRU
		case 6:
		  destIJ += lhsRow[5] * rhsRow[5];
		  // FALLTHRU
		case 5:
		  destIJ += lhsRow[4] * rhsRow[4];
		  // FALLTHRU
		case 4:
		  destIJ += lhsRow[3] * rhsRow[3];
		  // FALLTHRU
		case 3:
		  destIJ += lhsRow[2] * rhsRow[2];
		  // FALLTHRU
		case 2:
		  destIJ += lhsRow[1] * rhsRow[1];
		  // FALLTHRU
		case 1:
		  destIJ += lhsRow[0] * rhsRow[0];
		  // FALLTHRU
		default:
		  break;
		}
	      dest( ii, jj ) = destIJ;
	    }
	}
    }
}

void SAIMatrix::multiplySubmatrix( int stRow, int stCol, int nRows, int nCols, 
				   const SAIMatrix& rhs, int r_stRow, int r_stCol, int r_nRows, int r_nCols, 
				   SAIMatrix& dest, bool fTranspose ) const
{
  assert( &dest != &rhs && &dest != this );
  assert( stRow+nRows <= m_row && stCol+nCols <= m_col );
  assert( r_stRow+r_nRows <= rhs.m_row && r_stCol+r_nCols <= rhs.m_col );

  if( !fTranspose )
    {
      assert( nCols == r_nRows );

      dest.resize( nRows, r_nCols );
      int rhsColSize = rhs.m_col;
      int rhsColSize2 = rhsColSize+rhsColSize;
      int rhsColSize3 = rhsColSize2+rhsColSize;
      int rhsColSize4 = rhsColSize3+rhsColSize;
      int rhsColSize5 = rhsColSize4+rhsColSize;
      int rhsColSize6 = rhsColSize5+rhsColSize;
      int rhsColSize7 = rhsColSize6+rhsColSize;
      int rhsColSize8 = rhsColSize7+rhsColSize;

      for( int ii = 0; ii < dest.m_row; ii++ )
	{
	  for( int jj = 0; jj < dest.m_col; jj++ )
	    {
	      Float* lhsRow = &m_data[m_col * (ii+stRow)+stCol];
	      Float* rhsCol = &rhs.m_data[jj+r_stCol+r_stRow*rhs.m_col];
	      Float destIJ = 0;
	      int numTerms = nCols;
	      while( numTerms >= 8 )
		{
		  destIJ += ( lhsRow[0] * rhsCol[0] +
			      lhsRow[1] * rhsCol[rhsColSize] +
			      lhsRow[2] * rhsCol[rhsColSize2] +
			      lhsRow[3] * rhsCol[rhsColSize3] +
			      lhsRow[4] * rhsCol[rhsColSize4] +
			      lhsRow[5] * rhsCol[rhsColSize5] +
			      lhsRow[6] * rhsCol[rhsColSize6] +
			      lhsRow[7] * rhsCol[rhsColSize7] );
		  lhsRow += 8;
		  rhsCol += rhsColSize8;
		  numTerms -= 8;
		}
	      switch( numTerms )
		{
		case 7:
		  destIJ += lhsRow[6] * rhsCol[rhsColSize6];
		  // FALLTHRU
		case 6:
		  destIJ += lhsRow[5] * rhsCol[rhsColSize5];
		  // FALLTHRU
		case 5:
		  destIJ += lhsRow[4] * rhsCol[rhsColSize4];
		  // FALLTHRU
		case 4:
		  destIJ += lhsRow[3] * rhsCol[rhsColSize3];
		  // FALLTHRU
		case 3:
		  destIJ += lhsRow[2] * rhsCol[rhsColSize2];
		  // FALLTHRU
		case 2:
		  destIJ += lhsRow[1] * rhsCol[rhsColSize];
		  // FALLTHRU
		case 1:
		  destIJ += lhsRow[0] * rhsCol[0];
		  // FALLTHRU
		default:
		  break;
		}
	      dest( ii, jj ) = destIJ;
	    }
	}
    }

  // transpose the rhs
  else
    {
      assert( nCols == r_nCols );

      dest.resize( nRows, r_nRows );
      for( int ii = 0; ii < dest.m_row; ii++ )
	{
	  for( int jj = 0; jj < dest.m_col; jj++ )
	    {
	      Float* lhsRow = &m_data[m_col * (ii+stRow)+stCol];
	      Float* rhsRow = &rhs.m_data[rhs.m_col * (jj+r_stRow) + r_stCol];
	      Float destIJ = 0;
	      int numTerms = nCols;
	      while( numTerms >= 8 )
		{
		  destIJ += ( lhsRow[0] * rhsRow[0] +
			      lhsRow[1] * rhsRow[1] +
			      lhsRow[2] * rhsRow[2] +
			      lhsRow[3] * rhsRow[3] +
			      lhsRow[4] * rhsRow[4] +
			      lhsRow[5] * rhsRow[5] +
			      lhsRow[6] * rhsRow[6] +
			      lhsRow[7] * rhsRow[7] );
		  lhsRow += 8;
		  rhsRow += 8;
		  numTerms -= 8;
		}
	      switch( numTerms )
		{
		case 7:
		  destIJ += lhsRow[6] * rhsRow[6];
		  // FALLTHRU
		case 6:
		  destIJ += lhsRow[5] * rhsRow[5];
		  // FALLTHRU
		case 5:
		  destIJ += lhsRow[4] * rhsRow[4];
		  // FALLTHRU
		case 4:
		  destIJ += lhsRow[3] * rhsRow[3];
		  // FALLTHRU
		case 3:
		  destIJ += lhsRow[2] * rhsRow[2];
		  // FALLTHRU
		case 2:
		  destIJ += lhsRow[1] * rhsRow[1];
		  // FALLTHRU
		case 1:
		  destIJ += lhsRow[0] * rhsRow[0];
		  // FALLTHRU
		default:
		  break;
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
// If possible, do the multiplication in-place to avoid extra memory
// allocation.
// ===================================================================
void SAIMatrix::multiply( const SAIMatrix& rhs, bool fTranspose,
			  SAIVector* pTmpStore )
{
  if( rhs.m_row != rhs.m_col )
    {
      // If rhs is not square, cannot multiply in-place
      SAIMatrix res( m_row, fTranspose ? rhs.m_row : rhs.m_col );
      multiply( rhs, res, fTranspose );
      transfer( res );
    }
  else if( !fTranspose )
    {
      // Multiply this * rhs in-place
      assert( m_col == rhs.m_row );
      assert( &rhs != this );
      SAIVector tmp;
      SAIVector& tempRow = pTmpStore != NULL ? *pTmpStore : tmp;
      tempRow.setSize( m_col );
      int rhsColSize = rhs.m_col;

      for( int ii = 0; ii < m_row; ii++ )
	{
	  for( int jj = 0; jj < m_col; jj++ )
	    {
	      Float* lhsRow = &m_data[m_col * ii];
	      Float* rhsCol = &rhs.m_data[jj];
	      Float destIJ = 0;
	      int numTerms = m_col;
	      while( numTerms >= 8 )
		{
		  destIJ += ( lhsRow[0] * rhsCol[0 * rhsColSize] +
			      lhsRow[1] * rhsCol[1 * rhsColSize] +
			      lhsRow[2] * rhsCol[2 * rhsColSize] +
			      lhsRow[3] * rhsCol[3 * rhsColSize] +
			      lhsRow[4] * rhsCol[4 * rhsColSize] +
			      lhsRow[5] * rhsCol[5 * rhsColSize] +
			      lhsRow[6] * rhsCol[6 * rhsColSize] +
			      lhsRow[7] * rhsCol[7 * rhsColSize] );
		  lhsRow += 8;
		  rhsCol += 8 * rhsColSize;
		  numTerms -= 8;
		}
	      switch( numTerms )
		{
		case 7:
		  destIJ += lhsRow[6] * rhsCol[6 * rhsColSize];
		  // FALLTHRU
		case 6:
		  destIJ += lhsRow[5] * rhsCol[5 * rhsColSize];
		  // FALLTHRU
		case 5:
		  destIJ += lhsRow[4] * rhsCol[4 * rhsColSize];
		  // FALLTHRU
		case 4:
		  destIJ += lhsRow[3] * rhsCol[3 * rhsColSize];
		  // FALLTHRU
		case 3:
		  destIJ += lhsRow[2] * rhsCol[2 * rhsColSize];
		  // FALLTHRU
		case 2:
		  destIJ += lhsRow[1] * rhsCol[1 * rhsColSize];
		  // FALLTHRU
		case 1:
		  destIJ += lhsRow[0] * rhsCol[0 * rhsColSize];
		  // FALLTHRU
		default:
		  break;
		}
	      tempRow[jj] = destIJ;
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
      assert( m_col == rhs.m_col );
      assert( &rhs != this );
      SAIVector tmp;
      SAIVector& tempRow = pTmpStore != NULL ? *pTmpStore : tmp;
      tempRow.setSize( m_col );

      for( int ii = 0; ii < m_row; ii++ )
	{
	  for( int jj = 0; jj < rhs.m_row; jj++ )
	    {
	      Float* lhsRow = &m_data[m_col * ii];
	      Float* rhsRow = &rhs.m_data[rhs.m_col * jj];
	      Float destIJ = 0;
	      int numTerms = m_col;
	      while( numTerms >= 8 )
		{
		  destIJ += ( lhsRow[0] * rhsRow[0] +
			      lhsRow[1] * rhsRow[1] +
			      lhsRow[2] * rhsRow[2] +
			      lhsRow[3] * rhsRow[3] +
			      lhsRow[4] * rhsRow[4] +
			      lhsRow[5] * rhsRow[5] +
			      lhsRow[6] * rhsRow[6] +
			      lhsRow[7] * rhsRow[7] );
		  lhsRow += 8;
		  rhsRow += 8;
		  numTerms -= 8;
		}
	      switch( numTerms )
		{
		case 7:
		  destIJ += lhsRow[6] * rhsRow[6];
		  // FALLTHRU
		case 6:
		  destIJ += lhsRow[5] * rhsRow[5];
		  // FALLTHRU
		case 5:
		  destIJ += lhsRow[4] * rhsRow[4];
		  // FALLTHRU
		case 4:
		  destIJ += lhsRow[3] * rhsRow[3];
		  // FALLTHRU
		case 3:
		  destIJ += lhsRow[2] * rhsRow[2];
		  // FALLTHRU
		case 2:
		  destIJ += lhsRow[1] * rhsRow[1];
		  // FALLTHRU
		case 1:
		  destIJ += lhsRow[0] * rhsRow[0];
		  // FALLTHRU
		default:
		  break;
		}
	      tempRow[jj] = destIJ;
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
void SAIMatrix::multiply( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( m_col == rhs.size() );
  assert( &dest != &rhs );
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
void SAIMatrix::multiply( Float rhs, SAIMatrix& dest ) const
{
  dest.resize( m_row, m_col );
  for( int ii = 0; ii < m_size; ii++ )
    {
      dest.m_data[ii] = m_data[ii] * rhs;
    }
}

void SAIMatrix::multiply( Float rhs )
{
  for( int ii = 0; ii < m_size; ii++ )
    {
      m_data[ii] *= rhs;
    }
}

// ===================================================================
// multiplyTranspose(): Multiply this^T by a matrix
// ===================================================================
void SAIMatrix::multiplyTranspose(const SAIMatrix& rhs, SAIMatrix& dest) const
{
  assert( m_row == rhs.m_row );
  assert( &dest != &rhs && &dest != this );
  dest.resize( m_col, rhs.m_col );

  int lhsColSize = m_col;
  int rhsColSize = rhs.m_col;
  for( int ii = 0; ii < dest.m_row; ii++)
    {
      for( int jj = 0; jj < dest.m_col; jj++)
	{
	  Float* lhsCol = &m_data[ii];
	  Float* rhsCol = &rhs.m_data[jj];
	  Float destIJ = 0;
	  int numTerms = m_row;
	  while( numTerms >= 8 )
	    {
	      destIJ += ( lhsCol[0 * lhsColSize] * rhsCol[0 * rhsColSize] +
			  lhsCol[1 * lhsColSize] * rhsCol[1 * rhsColSize] +
			  lhsCol[2 * lhsColSize] * rhsCol[2 * rhsColSize] +
			  lhsCol[3 * lhsColSize] * rhsCol[3 * rhsColSize] +
			  lhsCol[4 * lhsColSize] * rhsCol[4 * rhsColSize] +
			  lhsCol[5 * lhsColSize] * rhsCol[5 * rhsColSize] +
			  lhsCol[6 * lhsColSize] * rhsCol[6 * rhsColSize] +
			  lhsCol[7 * lhsColSize] * rhsCol[7 * rhsColSize] );
	      lhsCol += 8 * lhsColSize;
	      rhsCol += 8 * rhsColSize;
	      numTerms -= 8;
	    }
	  switch( numTerms )
	    {
	    case 7:
	      destIJ += lhsCol[6 * lhsColSize] * rhsCol[6 * rhsColSize];
	      // FALLTHRU
	    case 6:
	      destIJ += lhsCol[5 * lhsColSize] * rhsCol[5 * rhsColSize];
	      // FALLTHRU
	    case 5:
	      destIJ += lhsCol[4 * lhsColSize] * rhsCol[4 * rhsColSize];
	      // FALLTHRU
	    case 4:
	      destIJ += lhsCol[3 * lhsColSize] * rhsCol[3 * rhsColSize];
	      // FALLTHRU
	    case 3:
	      destIJ += lhsCol[2 * lhsColSize] * rhsCol[2 * rhsColSize];
	      // FALLTHRU
	    case 2:
	      destIJ += lhsCol[1 * lhsColSize] * rhsCol[1 * rhsColSize];
	      // FALLTHRU
	    case 1:
	      destIJ += lhsCol[0 * lhsColSize] * rhsCol[0 * rhsColSize];
	      // FALLTHRU
	    default:
	      break;
	    }
	  dest( ii, jj ) = destIJ;
	}
    }
}

// ===================================================================
// multiplyTranspose(): Multiply this^T by a vector
// ===================================================================
void SAIMatrix::multiplyTranspose(const SAIVector& rhs, SAIVector& dest) const
{
  assert( m_row == rhs.size() );
  assert( &dest != &rhs );
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

void SAIMatrix::elementMultiply(const SAIMatrix& rhs, SAIMatrix& dest) {
  assert( m_row == rhs.m_row );
  assert( m_col == rhs.m_col );
  assert( &dest != this );

  dest.setSize(m_row, m_col);
  for(int ii=0; ii<m_row; ii++) {
    for(int jj=0; jj<m_col; jj++) {
      dest[ii][jj] = elementAt(ii,jj) * rhs.elementAt(ii,jj);
    }
  }
}

SAIMatrix SAIMatrix::elementMultiply(const SAIMatrix& rhs) {
  SAIMatrix t;
  elementMultiply(rhs, t);
  return t;
}

void SAIMatrix::applyFunction( double (*fcn)(double), SAIMatrix& dest ) const {
  dest.setSize(m_row, m_col);
  for( int ii=0; ii < m_row; ii++ ) {
    for( int jj = 0; jj < m_col; jj++ ) {
      dest.elementAt(ii,jj) = (*fcn)(elementAt(ii,jj));
    }
  }
}

void SAIMatrix::applyFunction( double (*fcn)(double) )  {
  for( int ii=0; ii < m_row; ii++ ) {
    for( int jj=0; jj < m_col; jj++ ) {
      elementAt(ii,jj) = (*fcn)(elementAt(ii,jj));
    }
  }
}

double SAIMatrix::sum() const {
  double s(0);
  for( int ii=0; ii < m_row; ii++ ) {
    for( int jj=0; jj < m_col; jj++ ) {
      s += elementAt(ii,jj);
    }
  }
  return s;
}

double SAIMatrix::maxAbsValue() const {
  double s(0);
  for( int ii=0; ii < m_row; ii++ ) {
    for( int jj=0; jj < m_col; jj++ ) {
      if(fabs(elementAt(ii,jj)) > s) {
        s = fabs(elementAt(ii,jj));
      }
    }
  }
  return s;
}

// ===================================================================
// transpose():   [0 1 2; 3 4 5; 6 7 8]^T = [0 3 6; 1 4 7; 2 5 8]
// ===================================================================
void SAIMatrix::transpose( SAIMatrix& dest ) const
{
  assert( &dest != this );
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
// display():  Display the matrix.
// ===================================================================
void SAIMatrix::display( const char* name ) const
{
  if( name != NULL )
    {
      printf( "%s =\n (", name );
    }
  else
    {
      printf( " (" );
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


void SAIMatrix::
prettyPrint (std::ostream & os, std::string const & title, std::string const & prefix) const
{
  if ( ! title.empty())
    os << title << "\n";
  if ((m_row <= 0) || (m_col <= 0))
    os << prefix << " (empty)\n";
  else {
    static int const buflen(32);
    static char buf[buflen];
    memset(buf, 0, sizeof(buf));
    for (int ir(0); ir < m_row; ++ir) {
      if ( ! prefix.empty())
	os << prefix;
      for (int ic(0); ic < m_col; ++ic) {
	if (isinf(elementAt(ir, ic))) {
	  snprintf(buf, buflen-1, " inf    ");
	}
	else if (isnan(elementAt(ir, ic))) {
	  snprintf(buf, buflen-1, " nan    ");
	}
	else if (fabs(fmod(elementAt(ir, ic), 1)) < 1e-6) {
	  snprintf(buf, buflen-1, "%- 7d  ", static_cast<int>(rint(elementAt(ir, ic))));
	}
	else {
	  snprintf(buf, buflen-1, "% 6.4f  ", elementAt(ir, ic));
	}
	os << buf;
      }
      os << "\n";
    }
  }
}


std::string SAIMatrix::
prettyString(std::string const & title, std::string const & prefix) const
{
  ostringstream pretty;
  prettyPrint(pretty, title, prefix);
  return pretty.str();
}


// ===================================================================
// Streaming operators
// ===================================================================
ostream& operator<<(ostream& os, const SAIMatrix& m)
{
  os.precision(10);
  for(int i=0; i<m.m_row; i++) {
    //os << '[';
    for(int j=0; j<m.m_col; j++) {
      os << m[i][j] << '\t';
    }
    //os << ']';
    os << endl;
  }
  return os;
}

istream& operator>>(istream& is, SAIMatrix& m)
{
  // Assume size is already set
  for(int i=0; i<m.m_row; i++) {
    for(int j=0; j<m.m_col; j++) {
      is >> m[i][j];
    }
  }
  return is;
}

// ===================================================================
// dynamic(): This virtual function is used to determine whether
// m_data was dynamically allocated.  It returns true for SAIMatrix,
// and false for SAIMatrix3 and SAIMatrix6.
//
// If dynamic() is false, then it is illegal to delete m_data, or to
// transfer it to another matrix.
// ===================================================================
bool SAIMatrix::dynamic() const
{
  return true;
}

// ===================================================================
// resize(): This virtual function resizes the matrix without
// initializing the data.  It is used by setSize(), the assignment
// operator, etc.
//
// This function is normally overridden in the subclass.  For example,
// since a SAIMatrix3 is *always* 3x3, SAIMatrix3 overrides this
// function to throw an assertion unless row == col == 3.
// ===================================================================
void SAIMatrix::resize( int row, int col )
{
  assert( row >= 0 && col >= 0 );

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


void SAIMatrix::identity(int dimension)
{
  resize(dimension, dimension);
  zero();
  for (int ii(0); ii < dimension; ii++) {
    elementAt(ii, ii) = 1;
  }
}


// ===================================================================
// fIdentity(): test if it is an identity matrix
// ===================================================================
bool SAIMatrix::fIdentity() const
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
bool SAIMatrix::fZero() const
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
void SAIMatrix::preMultiply( const SAIMatrix& mult, SAIVector* pTmpStore )
{
  assert( mult.row() == mult.column() && mult.column() == m_row );
  SAIVector tmp;
  SAIVector& tmpColumn = pTmpStore == NULL ? tmp : *pTmpStore;
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
// SAIMappedMatrix implementation
// ===================================================================
SAIMappedMatrix::SAIMappedMatrix( SAIMatrix& src, int row, int cRows )
{
  assert( row < src.row() && row+cRows <= src.row() );

  m_data = &src.elementAt( row, 0 );
  m_row = cRows;
  m_col = src.column();
  m_size = m_row*m_col;
}

SAIMappedMatrix::~SAIMappedMatrix()
{
  m_data = NULL;
}

bool SAIMappedMatrix::dynamic() const
{
  return false;
}

void SAIMappedMatrix::resize( int row, int col )
{
  assert( m_row == row && m_col == col );
}

void SAIMatrix::inverse( SAIMatrix& ainv ) const
{
  assert( m_row == m_col );
  ainv.resize( m_row, m_col );

  SAIMatrix lu( NULL, m_row, m_col );
  SAIVector x( NULL, m_col );
  SAIVector y( m_col ); // y must be mostly 0

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
// LUdecomp(): Perform an LU decomposition of a square, full-rank
// matrix.  L and U are combined into a single matrix, with the
// convention that U has a diagonal of [1 1 1 ...].
//
// Warning:  This routine does no pivoting.
// ===================================================================
void SAIMatrix::LUdecomp( SAIMatrix& lu ) const
{
  assert( m_row == m_col );
  assert( &lu != this );
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
	  lu( ii, jj ) = elementAt( ii, jj ) - sum;  //L
	}

      for( int kk = jj + 1; kk < m_col; kk++ )
	{
	  sum = 0.0;
	  for( int ii = 0; ii < jj; ii++ )
	    {
	      sum += lu( jj, ii ) * lu( ii, kk );
	    }
	  lu( jj, kk ) = ( elementAt( jj, kk ) - sum ) / lu( jj, jj ); //U
	}
    }
}

// ===================================================================
// backSub(): Solve (LU x = y) for x.  This matrix must be in LU form,
// as returned by LUdecomp().
// ===================================================================
void SAIMatrix::backSub( const SAIVector& y, SAIVector& x ) const
{
  assert( m_row == m_col );
  assert( m_row == y.size() );
  x.setSize( m_row );

  Float sum;
  SAIVector d( NULL, m_row );

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


SAIMatrix SAIMatrix::
spatialTransform( SAIMatrix const & rot, SAIVector const & dist )
{
  SAIMatrix spatialTrans(6,6);
  spatialTrans.zero();
  spatialTrans.setSubmatrix(3,3,rot);
  SAIVector firstCol, secondCol, thirdCol, firstCol2(3), secondCol2(3), thirdCol2(3);
  
  rot.getColumn(0,firstCol);
  rot.getColumn(1,secondCol);
  rot.getColumn(2,thirdCol);
  
  firstCol = dist.cross( firstCol );
  secondCol = dist.cross( secondCol );
  thirdCol = dist.cross( thirdCol );
  
  firstCol2.zero(); secondCol2.zero(); thirdCol2.zero();
  firstCol2.append(firstCol);
  secondCol2.append(secondCol);
  thirdCol2.append(thirdCol);
  
  spatialTrans.setColumn(0,firstCol2);
  spatialTrans.setColumn(1,secondCol2);
  spatialTrans.setColumn(2,thirdCol2);
  
  spatialTrans.setSubmatrix(0,0,rot);
  
  return spatialTrans;
}
