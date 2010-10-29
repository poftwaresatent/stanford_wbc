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
// SAIMatrix6.cpp
//
// Implementation of a 6x6 matrix.
//
// modification history
// --------------------
//
// 06/17/04: Dan Merget: Made into a subclass of SAIMatrix
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************



#include "SAIMatrix6.h"

const SAIMatrix6 SAIMatrix6::IDENTITY(SAIMatrix3(1, 0, 0,   0, 1, 0,   0, 0, 1),
                                    SAIMatrix3(0, 0, 0,   0, 0, 0,   0, 0, 0),
                                    SAIMatrix3(0, 0, 0,   0, 0, 0,   0, 0, 0),
                                    SAIMatrix3(1, 0, 0,   0, 1, 0,   0, 0, 1));
const SAIMatrix6 SAIMatrix6::ZERO;

// ===================================================================
// Destructor: In C++, the child destructor is called before the
// base-class destructor.  So by setting m_data back to NULL here, we
// ensure that the base class won't try to delete m_buff.
// ===================================================================
SAIMatrix6::~SAIMatrix6()
{
  m_data = NULL;
  m_row  = 0;
  m_col  = 0;
  m_size = 0;
}

// ===================================================================
// setDiagonal(): Set the diagonal elements
// ===================================================================
void SAIMatrix6::setDiagonal( Float d0, Float d1, Float d2,
                             Float d3, Float d4, Float d5 )
{
  m_buff[0][0] = d0;
  m_buff[1][1] = d1;
  m_buff[2][2] = d2;
  m_buff[3][3] = d3;
  m_buff[4][4] = d4;
  m_buff[5][5] = d5;
}

SAIVector6 SAIMatrix6::diagonal() const
{
  return SAIVector6( m_buff[0][0], m_buff[1][1], m_buff[2][2],
                    m_buff[3][3], m_buff[4][4], m_buff[5][5] );
}

void SAIMatrix6::diagonal( SAIVector& dest ) const
{
  dest.setSize( 6 );
  dest[0] = m_buff[0][0];
  dest[1] = m_buff[1][1];
  dest[2] = m_buff[2][2];
  dest[3] = m_buff[3][3];
  dest[4] = m_buff[4][4];
  dest[5] = m_buff[5][5];
}

void SAIMatrix6::setDiagonal( Float src )
{
  m_buff[0][0] = src;
  m_buff[1][1] = src;
  m_buff[2][2] = src;
  m_buff[3][3] = src;
  m_buff[4][4] = src;
  m_buff[5][5] = src;
}

void SAIMatrix6::setDiagonal( const Float* src )
{
  m_buff[0][0] = src[0];
  m_buff[1][1] = src[1];
  m_buff[2][2] = src[2];
  m_buff[3][3] = src[3];
  m_buff[4][4] = src[4];
  m_buff[5][5] = src[5];
}

void SAIMatrix6::setDiagonal( const SAIVector& src )
{
  assert( src.size() == 6 );
  m_buff[0][0] = src[0];
  m_buff[1][1] = src[1];
  m_buff[2][2] = src[2];
  m_buff[3][3] = src[3];
  m_buff[4][4] = src[4];
  m_buff[5][5] = src[5];
}

// ===================================================================
// getRow() / getColumn(): Same as the base-class method, but
// optimized for 6x6 matrices.
// ===================================================================
void SAIMatrix6::getRow( int row, SAIVector& dest ) const
{
  assert( row >= 0 && row < 6 );
  dest.setSize( 6 );
  dest[0] = m_buff[row][0];
  dest[1] = m_buff[row][1];
  dest[2] = m_buff[row][2];
  dest[3] = m_buff[row][3];
  dest[4] = m_buff[row][4];
  dest[5] = m_buff[row][5];
}

void SAIMatrix6::getColumn( int col, SAIVector& dest ) const
{
  assert( col >= 0 && col < 6 );
  dest.setSize( 6 );
  dest[0] = m_buff[0][col];
  dest[1] = m_buff[1][col];
  dest[2] = m_buff[2][col];
  dest[3] = m_buff[3][col];
  dest[4] = m_buff[4][col];
  dest[5] = m_buff[5][col];
}

// ===================================================================
// submatrix(): Same as the base-class method, but optimized for
// extracting 3x3 submatrices
// ===================================================================
void SAIMatrix6::submatrix( int row, int col, SAIMatrix& dest ) const
{
  if( dest.row() != 3 || dest.column() != 3 )
  {
    // If dest is not 3x3, use the generic base-class method
    SAIMatrix::submatrix( row, col, dest );
  }
  else
  {
    // Copy a 3x3 submatrix
    assert( row >= 0 && row <= 3 );
    assert( col >= 0 && col <= 3 );
    const Float* src = &m_buff[row][col];

    dest[0][0] = src[6 * 0 + 0];
    dest[0][1] = src[6 * 0 + 1];
    dest[0][2] = src[6 * 0 + 2];

    dest[1][0] = src[6 * 1 + 0];
    dest[1][1] = src[6 * 1 + 1];
    dest[1][2] = src[6 * 1 + 2];

    dest[2][0] = src[6 * 2 + 0];
    dest[2][1] = src[6 * 2 + 1];
    dest[2][2] = src[6 * 2 + 2];
  }
}

// ===================================================================
// setSubmatrix(): Same as the base-class method, but optimized for
// copying 3x3 submatrices
// ===================================================================
void SAIMatrix6::setSubmatrix( int row, int col, const SAIMatrix& src )
{
  if( src.row() != 3 || src.column() != 3 )
  {
    // If src is not 3x3, use the generic base-class method
    SAIMatrix::setSubmatrix( row, col, src );
  }
  else
  {
    // Copy a 3x3 submatrix
    assert( row >= 0 && row <= 3 );
    assert( col >= 0 && col <= 3 );
    Float* dest = &m_buff[row][col];

    dest[6 * 0 + 0] = src[0][0];
    dest[6 * 0 + 1] = src[0][1];
    dest[6 * 0 + 2] = src[0][2];
    dest[6 * 1 + 0] = src[1][0];
    dest[6 * 1 + 1] = src[1][1];
    dest[6 * 1 + 2] = src[1][2];
    dest[6 * 2 + 0] = src[2][0];
    dest[6 * 2 + 1] = src[2][1];
    dest[6 * 2 + 2] = src[2][2];
  }
}

// ===================================================================
// identity(): Same as the base-class method, but optimized for 6x6
// matrices.
// ===================================================================
void SAIMatrix6::identity()
{
  zero();
  m_buff[0][0] = 1;
  m_buff[1][1] = 1;
  m_buff[2][2] = 1;
  m_buff[3][3] = 1;
  m_buff[4][4] = 1;
  m_buff[5][5] = 1;
}

// ===================================================================
// negate(): Same as the base-class method, but optimized for 6x6
// matrices.
// ===================================================================
void SAIMatrix6::negate( SAIMatrix& dest ) const
{
  dest.setSize( 6, 6 );
  for( int ii = 0; ii < 6; ii++ )
  {
    const Float* srcRow  = m_buff[ii];
    Float* destRow = dest[ii];

    destRow[0] = -srcRow[0];
    destRow[1] = -srcRow[1];
    destRow[2] = -srcRow[2];
    destRow[3] = -srcRow[3];
    destRow[4] = -srcRow[4];
    destRow[5] = -srcRow[5];
  }
}

// ===================================================================
// Arithmetic operations: Same as the base-class method, but optimized
// for 6x6 matrices.
// ===================================================================
void SAIMatrix6::add( const SAIMatrix& rhs, SAIMatrix& dest ) const
{
  assert( rhs.row() == 6 && rhs.column() == 6 );
  dest.setSize( 6, 6 );
  for( int ii = 0; ii < 6; ii++ )
  {
    const Float* lhsRow = m_buff[ii];
    const Float* rhsRow = rhs[ii];
    Float* destRow = dest[ii];

    destRow[0] = lhsRow[0] + rhsRow[0];
    destRow[1] = lhsRow[1] + rhsRow[1];
    destRow[2] = lhsRow[2] + rhsRow[2];
    destRow[3] = lhsRow[3] + rhsRow[3];
    destRow[4] = lhsRow[4] + rhsRow[4];
    destRow[5] = lhsRow[5] + rhsRow[5];
  }
}

void SAIMatrix6::add( const SAIMatrix& rhs )
{
  assert( rhs.row() == 6 && rhs.column() == 6 );
  for( int ii = 0; ii < 6; ii++ )
  {
    Float* lhsRow = m_buff[ii];
    const Float* rhsRow = rhs[ii];

    lhsRow[0] += rhsRow[0];
    lhsRow[1] += rhsRow[1];
    lhsRow[2] += rhsRow[2];
    lhsRow[3] += rhsRow[3];
    lhsRow[4] += rhsRow[4];
    lhsRow[5] += rhsRow[5];
  }
}

void SAIMatrix6::subtract( const SAIMatrix& rhs, SAIMatrix& dest ) const
{
  assert( rhs.row() == 6 && rhs.column() == 6 );
  dest.setSize( 6, 6 );
  for( int ii = 0; ii < 6; ii++ )
  {
    const Float* lhsRow = m_buff[ii];
    const Float* rhsRow = rhs[ii];
    Float* destRow = dest[ii];

    destRow[0] = lhsRow[0] - rhsRow[0];
    destRow[1] = lhsRow[1] - rhsRow[1];
    destRow[2] = lhsRow[2] - rhsRow[2];
    destRow[3] = lhsRow[3] - rhsRow[3];
    destRow[4] = lhsRow[4] - rhsRow[4];
    destRow[5] = lhsRow[5] - rhsRow[5];
  }
}

void SAIMatrix6::subtract( const SAIMatrix& rhs )
{
  assert( rhs.row() == 6 && rhs.column() == 6 );
  for( int ii = 0; ii < 6; ii++ )
  {
    Float* lhsRow = m_buff[ii];
    const Float* rhsRow = rhs[ii];

    lhsRow[0] -= rhsRow[0];
    lhsRow[1] -= rhsRow[1];
    lhsRow[2] -= rhsRow[2];
    lhsRow[3] -= rhsRow[3];
    lhsRow[4] -= rhsRow[4];
    lhsRow[5] -= rhsRow[5];
  }
}

void SAIMatrix6::multiply( const SAIMatrix& rhs, SAIMatrix& dest,
                                               bool fTranspose ) const
{
  if ( rhs.row() != 6 || rhs.column() != 6 )
  {
    // If rhs is not 6x6, then just use the generic parent-class method
    SAIMatrix::multiply( rhs, dest, fTranspose );
  }
  else if ( !fTranspose )
  {
    // dest = this * rhs, where rhs is 6x6
    assert( &dest != static_cast<const SAIMatrix*>(this) && &dest != &rhs );
    dest.setSize( 6, 6 );
    for( int ii = 0; ii < 6; ii++ )
    {
      const Float* a = m_buff[ii];
      const SAIMatrix& b = rhs;
      Float* destRow = dest[ii];
      destRow[0] = a[0] * b[0][0] + a[1] * b[1][0] + a[2] * b[2][0] +
                   a[3] * b[3][0] + a[4] * b[4][0] + a[5] * b[5][0];
      destRow[1] = a[0] * b[0][1] + a[1] * b[1][1] + a[2] * b[2][1] +
                   a[3] * b[3][1] + a[4] * b[4][1] + a[5] * b[5][1];
      destRow[2] = a[0] * b[0][2] + a[1] * b[1][2] + a[2] * b[2][2] +
                   a[3] * b[3][2] + a[4] * b[4][2] + a[5] * b[5][2];
      destRow[3] = a[0] * b[0][3] + a[1] * b[1][3] + a[2] * b[2][3] +
                   a[3] * b[3][3] + a[4] * b[4][3] + a[5] * b[5][3];
      destRow[4] = a[0] * b[0][4] + a[1] * b[1][4] + a[2] * b[2][4] +
                   a[3] * b[3][4] + a[4] * b[4][4] + a[5] * b[5][4];
      destRow[5] = a[0] * b[0][5] + a[1] * b[1][5] + a[2] * b[2][5] +
                   a[3] * b[3][5] + a[4] * b[4][5] + a[5] * b[5][5];
    }
  }
  else
  {
    // dest = this * rhs^T, where rhs is 6x6
    assert( &dest != static_cast<const SAIMatrix*>(this) && &dest != &rhs );
    dest.setSize( 6, 6 );
    for( int ii = 0; ii < 6; ii++ )
    {
      const Float* a = m_buff[ii];
      const SAIMatrix& b = rhs;
      Float* destRow = dest[ii];
      destRow[0] = a[0] * b[0][0] + a[1] * b[0][1] + a[2] * b[0][2]
                 + a[3] * b[0][3] + a[4] * b[0][4] + a[5] * b[0][5];
      destRow[1] = a[0] * b[1][0] + a[1] * b[1][1] + a[2] * b[1][2]
                 + a[3] * b[1][3] + a[4] * b[1][4] + a[5] * b[1][5];
      destRow[2] = a[0] * b[2][0] + a[1] * b[2][1] + a[2] * b[2][2]
                 + a[3] * b[2][3] + a[4] * b[2][4] + a[5] * b[2][5];
      destRow[3] = a[0] * b[3][0] + a[1] * b[3][1] + a[2] * b[3][2]
                 + a[3] * b[3][3] + a[4] * b[3][4] + a[5] * b[3][5];
      destRow[4] = a[0] * b[4][0] + a[1] * b[4][1] + a[2] * b[4][2]
                 + a[3] * b[4][3] + a[4] * b[4][4] + a[5] * b[4][5];
      destRow[5] = a[0] * b[5][0] + a[1] * b[5][1] + a[2] * b[5][2]
                 + a[3] * b[5][3] + a[4] * b[5][4] + a[5] * b[5][5];
    }
  }
}

void SAIMatrix6::multiply( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( &dest != &rhs );
  assert( rhs.size() == 6 );
  dest.setSize( 6 );
  const SAIMatrix6& lhs = *this;

  dest[0] = lhs[0][0] * rhs[0] + lhs[0][1] * rhs[1] + lhs[0][2] * rhs[2]
          + lhs[0][3] * rhs[3] + lhs[0][4] * rhs[4] + lhs[0][5] * rhs[5];
  dest[1] = lhs[1][0] * rhs[0] + lhs[1][1] * rhs[1] + lhs[1][2] * rhs[2]
          + lhs[1][3] * rhs[3] + lhs[1][4] * rhs[4] + lhs[1][5] * rhs[5];
  dest[2] = lhs[2][0] * rhs[0] + lhs[2][1] * rhs[1] + lhs[2][2] * rhs[2]
          + lhs[2][3] * rhs[3] + lhs[2][4] * rhs[4] + lhs[2][5] * rhs[5];
  dest[3] = lhs[3][0] * rhs[0] + lhs[3][1] * rhs[1] + lhs[3][2] * rhs[2]
          + lhs[3][3] * rhs[3] + lhs[3][4] * rhs[4] + lhs[3][5] * rhs[5];
  dest[4] = lhs[4][0] * rhs[0] + lhs[4][1] * rhs[1] + lhs[4][2] * rhs[2]
          + lhs[4][3] * rhs[3] + lhs[4][4] * rhs[4] + lhs[4][5] * rhs[5];
  dest[5] = lhs[5][0] * rhs[0] + lhs[5][1] * rhs[1] + lhs[5][2] * rhs[2]
          + lhs[5][3] * rhs[3] + lhs[5][4] * rhs[4] + lhs[5][5] * rhs[5];
}

void SAIMatrix6::multiply( Float rhs, SAIMatrix& dest ) const
{
  dest.setSize( 6, 6 );
  for( int ii = 0; ii < 6; ii++ )
  {
    const Float* lhsRow = m_buff[ii];
    Float* destRow = dest[ii];

    destRow[0] = lhsRow[0] * rhs;
    destRow[1] = lhsRow[1] * rhs;
    destRow[2] = lhsRow[2] * rhs;
    destRow[3] = lhsRow[3] * rhs;
    destRow[4] = lhsRow[4] * rhs;
    destRow[5] = lhsRow[5] * rhs;
  }
}

void SAIMatrix6::multiply( Float rhs )
{
  for( int ii = 0; ii < 6; ii++ )
  {
    Float* lhsRow = m_buff[ii];
    lhsRow[0] *= rhs;
    lhsRow[1] *= rhs;
    lhsRow[2] *= rhs;
    lhsRow[3] *= rhs;
    lhsRow[4] *= rhs;
    lhsRow[5] *= rhs;
  }
}

void SAIMatrix6::multiplyTranspose( const SAIMatrix& rhs, SAIMatrix& dest ) const
{
  if ( rhs.row() != 6 || rhs.column() != 6 )
  {
    // If rhs is not 6x6, then just use the generic parent-class method
    SAIMatrix::multiplyTranspose( rhs, dest );
  }
  else
  {
    // dest = this^T * rhs, where rhs is 6x6
    assert( &dest != static_cast<const SAIMatrix*>(this) && &dest != &rhs );
    dest.setSize( 6, 6 );
    for( int ii = 0; ii < 6; ii++ )
    {
      const Float* a = &m_buff[0][ii];
      const SAIMatrix& b = rhs;
      Float* destRow = dest[ii];
      destRow[0] = a[6*0] * b[0][0] + a[6*1] * b[1][0] + a[6*2] * b[2][0] +
                   a[6*3] * b[3][0] + a[6*4] * b[4][0] + a[6*5] * b[5][0];
      destRow[1] = a[6*0] * b[0][1] + a[6*1] * b[1][1] + a[6*2] * b[2][1] +
                   a[6*3] * b[3][1] + a[6*4] * b[4][1] + a[6*5] * b[5][1];
      destRow[2] = a[6*0] * b[0][2] + a[6*1] * b[1][2] + a[6*2] * b[2][2] +
                   a[6*3] * b[3][2] + a[6*4] * b[4][2] + a[6*5] * b[5][2];
      destRow[3] = a[6*0] * b[0][3] + a[6*1] * b[1][3] + a[6*2] * b[2][3] +
                   a[6*3] * b[3][3] + a[6*4] * b[4][3] + a[6*5] * b[5][3];
      destRow[4] = a[6*0] * b[0][4] + a[6*1] * b[1][4] + a[6*2] * b[2][4] +
                   a[6*3] * b[3][4] + a[6*4] * b[4][4] + a[6*5] * b[5][4];
      destRow[5] = a[6*0] * b[0][5] + a[6*1] * b[1][5] + a[6*2] * b[2][5] +
                   a[6*3] * b[3][5] + a[6*4] * b[4][5] + a[6*5] * b[5][5];
    }
  }
}

void SAIMatrix6::multiplyTranspose( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( &dest != &rhs );
  assert( rhs.size() == 6 );
  dest.setSize( 6 );
  const SAIMatrix6& lhs = *this;

  dest[0] = lhs[0][0] * rhs[0] + lhs[1][0] * rhs[1] + lhs[2][0] * rhs[2]
          + lhs[3][0] * rhs[3] + lhs[4][0] * rhs[4] + lhs[5][0] * rhs[5];
  dest[1] = lhs[0][1] * rhs[0] + lhs[1][1] * rhs[1] + lhs[2][1] * rhs[2]
          + lhs[3][1] * rhs[3] + lhs[4][1] * rhs[4] + lhs[5][1] * rhs[5];
  dest[2] = lhs[0][2] * rhs[0] + lhs[1][2] * rhs[1] + lhs[2][2] * rhs[2]
          + lhs[3][2] * rhs[3] + lhs[4][2] * rhs[4] + lhs[5][2] * rhs[5];
  dest[3] = lhs[0][3] * rhs[0] + lhs[1][3] * rhs[1] + lhs[2][3] * rhs[2]
          + lhs[3][3] * rhs[3] + lhs[4][3] * rhs[4] + lhs[5][3] * rhs[5];
  dest[4] = lhs[0][4] * rhs[0] + lhs[1][4] * rhs[1] + lhs[2][4] * rhs[2]
          + lhs[3][4] * rhs[3] + lhs[4][4] * rhs[4] + lhs[5][4] * rhs[5];
  dest[5] = lhs[0][5] * rhs[0] + lhs[1][5] * rhs[1] + lhs[2][5] * rhs[2]
          + lhs[3][5] * rhs[3] + lhs[4][5] * rhs[4] + lhs[5][5] * rhs[5];
}

void SAIMatrix6::transpose( SAIMatrix& dest ) const
{
  assert( &dest != static_cast<const SAIMatrix*>(this) );
  dest.setSize( 6, 6 );

  for( int ii = 0; ii < 6; ii++ )
  {
    const Float* srcCol = &m_buff[0][ii];
    Float* destRow = dest[ii];

    destRow[0] = srcCol[6 * 0];
    destRow[1] = srcCol[6 * 1];
    destRow[2] = srcCol[6 * 2];
    destRow[3] = srcCol[6 * 3];
    destRow[4] = srcCol[6 * 4];
    destRow[5] = srcCol[6 * 5];
  }
}

// ===================================================================
// display(): Display the matrix
// ===================================================================
void SAIMatrix6::display( const char* name ) const
{
  if( name != NULL )
  {
    printf("%s =\n", name );
  }

  printf( "[%f %f %f %f %f %f]\n"
          "[%f %f %f %f %f %f]\n"
          "[%f %f %f %f %f %f]\n",
          m_buff[0][0], m_buff[0][1], m_buff[0][2],
          m_buff[0][3], m_buff[0][4], m_buff[0][5],
          m_buff[1][0], m_buff[1][1], m_buff[1][2],
          m_buff[1][3], m_buff[1][4], m_buff[1][5],
          m_buff[2][0], m_buff[2][1], m_buff[2][2],
          m_buff[2][3], m_buff[2][4], m_buff[2][5] );
  printf( "[%f %f %f %f %f %f]\n"
          "[%f %f %f %f %f %f]\n"
          "[%f %f %f %f %f %f]\n",
          m_buff[3][0], m_buff[3][1], m_buff[3][2],
          m_buff[3][3], m_buff[3][4], m_buff[3][5],
          m_buff[4][0], m_buff[4][1], m_buff[4][2],
          m_buff[4][3], m_buff[4][4], m_buff[4][5],
          m_buff[5][0], m_buff[5][1], m_buff[5][2],
          m_buff[5][3], m_buff[5][4], m_buff[5][5] );
}

// ===================================================================
// dynamic(): This virtual function tells the base that m_data was
// not dynamically allocated in this object.
// ===================================================================
bool SAIMatrix6::dynamic() const
{
  return false;
}

// ===================================================================
// resize(): Virtual function that the base class uses to resize the
// matrix.  It is an error to resize a SAIMatrix6 to any size other
// than 6x6.
// ===================================================================
void SAIMatrix6::resize( int row, int col )
{
  assert( row == 6 && col == 6 );
}
