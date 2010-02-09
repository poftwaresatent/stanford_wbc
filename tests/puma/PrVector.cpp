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
// PrVector.cpp
//
// This implements an Nx1 vector.
//
// modification history
// --------------------
//
// 06/16/04: Dan Merget: made PrVector3 & PrVector6 into subclasses
// 11/12/97: K.C. Chang: created.
// *******************************************************************

#include "XPrintf.h"
#include "PrVector.h"
#include "PrMatrix.h"
#include "PrVector3.h"
#include "PrMatrix3.h"
#ifdef _WIN32
#include <float.h>
#define finite(num) _finite(num)
#endif

// ===================================================================
// Constructors & Destructors
// ===================================================================

PrVector::PrVector( int size ) : m_size( size ), m_maxSize( size )
{
  SAIAssert( m_size >= 0 );

  if( m_maxSize > 0 )
  {
    m_data = new Float[m_maxSize];
    zero();
  }
  else
  {
    m_data = NULL;
  }
}

PrVector::PrVector( const PrVector& rhs )
   : m_size( rhs.m_size ), m_maxSize( rhs.m_size )
{
  if( m_maxSize > 0 )
  {
    m_data = new Float[m_maxSize];
    for( int ii = 0; ii < m_size; ii++ )
    {
      m_data[ii] = rhs.m_data[ii];
    }
  }
  else
  {
    m_data = NULL;
  }
}

PrVector::PrVector( const Float* rgVals, int size )
   : m_size( size ), m_maxSize( size )
{
  SAIAssert( m_size >= 0 );
  if( m_maxSize > 0 )
  {
    m_data = new Float[m_maxSize];
    if( rgVals != NULL )
    {
      for( int ii = 0; ii < m_size; ii++ )
      {
        m_data[ii] = rgVals[ii];
      }
    }
  }
  else
  {
    m_data = NULL;
  }
}

PrVector::~PrVector()
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
bool PrVector::fFinite() const
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
// Assignment operator.  Resize this vector if necessary
// ===================================================================
PrVector& PrVector::operator=( const PrVector& rhs )
{
  if( this != &rhs )
  {
    resize( rhs.m_size );
    for( int ii = 0; ii < m_size; ii++ )
    {
      m_data[ii] = rhs.m_data[ii];
    }
  }
  return *this;
}

// ===================================================================
// setValues(): Copy Min(cVals, m_size) values from rgVals to this
// vector.
// ===================================================================
void PrVector::setValues( const Float* rgVals, int cVals )
{
  int cMove = cVals > m_size ? m_size : cVals;
  for( int ii = 0; ii < cMove; ii++ )
  {
    m_data[ii] = rgVals[ii];
  }
}

// ===================================================================
// getValues(): Copy Min(cVals, m_size) values from this vector to
// rgVals.
// ===================================================================
void PrVector::getValues( Float* rgVals, int cVals ) const
{
  int cMove = cVals > m_size ? m_size : cVals;
  for( int ii = 0; ii < cMove; ii++ )
  {
    rgVals[ii] = m_data[ii];
  }
}

// ===================================================================
// append(): Append rhs to the end of this vector.  Resize if needed.
// ===================================================================
void PrVector::append( const PrVector& rhs )
{
  PrVector ans( NULL, m_size + rhs.m_size );

  for( int ii = 0; ii < m_size; ii++ )
  {
    ans[ii] = m_data[ii];
  }
  for( int ii = m_size; ii < m_size + rhs.m_size; ii++ )
  {
    ans[ii] = rhs.m_data[ii - m_size];
  }
  transfer( ans );
}

// ===================================================================
// setSize(): Resize the vector and optionally zero it.
// ===================================================================
void PrVector::setSize( int size, bool fZero )
{
  resize( size );
  if( fZero )
  {
    zero();
  }
}

// ===================================================================
// transfer(): Replace contents of this vector with those of src,
// without allocating memory if possible.  Puts src in an undefined
// state.
//
// If both *this and src are dynamically allocated, then this
// operation efficiently moves the allocated data from src to *this.
// However, if either vector is non-dynamic (e.g. a PrVector3), then
// the data is simply copied.
// ===================================================================
void PrVector::transfer( PrVector& src )
{
  if( dynamic() && src.dynamic() )
  {
    if( m_data != NULL )
    {
      delete[] m_data;
    }
    m_size     = src.m_size;
    m_maxSize  = src.m_maxSize;
    m_data     = src.m_data;
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
// subvector(): Extract a subset of this vector.  The size of "dest"
// tells how many elements to extract.
// ===================================================================
void PrVector::subvector( int start, PrVector& dest ) const
{
  SAIAssert( start >= 0 && start + dest.m_size <= m_size );
  for ( int ii = 0; ii < dest.m_size; ii++ )
  {
    dest.m_data[ii] = m_data[start + ii];
  }
}

// ===================================================================
// setSubvector(): Copy a smaller vector into a subset of this vector.
// ===================================================================
void PrVector::setSubvector( int start, const PrVector& src )
{
  SAIAssert( start >= 0 && start + src.m_size <= m_size );
  for ( int ii = 0; ii < src.m_size; ii++ )
  {
    m_data[start + ii] = src.m_data[ii];
  }
}

// ===================================================================
// Equality operator: This operation tests to see if rhs has the same
// size and contents of this vector.  The data must match exactly;
// there is no epsilon tolerance.
// ===================================================================
bool PrVector::operator==( const PrVector& rhs ) const
{
  if( this != &rhs ) {
    if( m_size != rhs.m_size )
    {
      return false;
    }
    for( int ii = 0; ii < m_size; ii++ )
    {
      if ( m_data[ii] != rhs.m_data[ii] )
      {
        return false;
      }
    }
  }
  return true;
}

// ===================================================================
// Arithmetic operations
// ===================================================================

void PrVector::negate( PrVector& dest ) const
{
  dest.resize( m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = -m_data[ii];
  }
}

void PrVector::add( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( rhs.m_size == m_size );
  dest.resize( m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = m_data[ii] + rhs.m_data[ii];
  }
}

void PrVector::add( const PrVector& rhs )
{
  SAIAssert( rhs.m_size == m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    m_data[ii] += rhs.m_data[ii];
  }
}

void PrVector::subtract( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( rhs.m_size == m_size );
  dest.resize( m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = m_data[ii] - rhs.m_data[ii];
  }
}

void PrVector::subtract( const PrVector& rhs )
{
  SAIAssert( rhs.m_size == m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    m_data[ii] -= rhs.m_data[ii];
  }
}

PrVector PrVector::operator*( const PrMatrix& rhs ) const
{
  // Treat this as a horizontal matrix, and multiply (this * rhs).
  // Equivalent to (rhs^T * this), since PrVector doesn't specify
  // horizontal or vertical.
  PrVector tmp( NULL, rhs.column() );
  rhs.multiplyTranspose( *this, tmp );
  return tmp;
}

void PrVector::multiply( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( rhs.m_size == m_size );
  dest.resize( m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = m_data[ii] * rhs.m_data[ii];
  }
}

void PrVector::multiply( const PrVector& rhs )
{
  SAIAssert( rhs.m_size == m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    m_data[ii] *= rhs.m_data[ii];
  }
}

void PrVector::multiply( Float rhs, PrVector& dest ) const
{
  dest.resize( m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = m_data[ii] * rhs;
  }
}

void PrVector::multiply( Float rhs )
{
  for( int ii = 0; ii < m_size; ii++ )
  {
    m_data[ii] *= rhs;
  }
}

void PrVector::multiplyTransposed( const PrVector& rhs, PrMatrix& dest ) const
{
  dest.setSize( m_size, rhs.m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    for( int jj = 0; jj < rhs.m_size; jj++ )
    {
      dest[ii][jj] = m_data[ii] * rhs.m_data[jj];
    }
  }
}

PrMatrix PrVector::multiplyTransposed( const PrVector& rhs ) const
{
  PrMatrix tmp( NULL, m_size, rhs.m_size );
  multiplyTransposed(rhs, tmp);
  return tmp;
}

Float PrVector::dot( const PrVector& rhs ) const
{
  SAIAssert( rhs.m_size == m_size );
  Float num = 0.0;
  for( int ii = 0; ii < m_size; ii++ )
  {
    num += m_data[ii] * rhs.m_data[ii];
  }
  return num;
}

Float PrVector::lengthSquared() const
{
  Float num = 0;
  for( int ii = 0; ii < m_size; ii++ )
  {
    num += m_data[ii] * m_data[ii];
  }
  return num;
}

// ===================================================================
// cross(): Calculate the cross product between two 3x1 vectors
// ===================================================================
PrVector3 PrVector::cross( const PrVector& rhs ) const
{
  SAIAssert(m_size == 3);
  SAIAssert(rhs.m_size == 3);
  return PrVector3( m_data[1] * rhs.m_data[2] - m_data[2] * rhs.m_data[1],
                    m_data[2] * rhs.m_data[0] - m_data[0] * rhs.m_data[2],
                    m_data[0] * rhs.m_data[1] - m_data[1] * rhs.m_data[0] );
}

void PrVector::cross( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( m_size == 3 );
  SAIAssert( rhs.m_size == 3 );
  SAIAssert( &dest != &rhs && &dest != this );
  dest.resize( 3 );
  dest.m_data[0] = m_data[1] * rhs.m_data[2] - m_data[2] * rhs.m_data[1];
  dest.m_data[1] = m_data[2] * rhs.m_data[0] - m_data[0] * rhs.m_data[2];
  dest.m_data[2] = m_data[0] * rhs.m_data[1] - m_data[1] * rhs.m_data[0];
}

// ===================================================================
// cross(): Create a cross-product matrix.  The matrix for vector
// [x y z] is [0 -z y; z 0 -x; y x 0].
// ===================================================================
PrMatrix3 PrVector::cross() const
{
  SAIAssert( m_size == 3 );
  return PrMatrix3(       0.0, -m_data[2],  m_data[1],
                    m_data[2],        0.0, -m_data[0],
                   -m_data[1],  m_data[0],        0.0 );
}

void PrVector::cross( PrMatrix& dest ) const
{
  SAIAssert( m_size == 3 );
  dest.setSize( 3, 3 );
  dest[0][0]=        0.0;  dest[0][1]= -m_data[2];  dest[0][2]=  m_data[1];
  dest[1][0]=  m_data[2];  dest[1][1]=        0.0;  dest[1][2]= -m_data[0];
  dest[2][0]= -m_data[1];  dest[2][1]=  m_data[0];  dest[2][2]=        0.0;
}

// ===================================================================
// display(): Display the vector.
// ===================================================================
void PrVector::display( const char* name ) const
{
  if( name != NULL )
  {
    printf( "%s = (", name );
  }
  else
  {
     printf( "(" );
  }

  for( int ii = 0; ii < m_size; ii++ )
  {
    if( ii > 0 )
    {
      printf( " %.6f", m_data[ii] );
    }
    else
    {
      printf( "%.6f", m_data[ii] );
    }
  }

  if( name != NULL )
  {
    printf( ")\n" );
  }
  else
  {
    printf( ")" );
  }
}

// ===================================================================
// dynamic(): This virtual function is used to determine whether
// m_data was dynamically allocated.  It returns true for PrVector,
// and false for PrVector3 and PrVector6.
//
// If dynamic() is false, then it is illegal to delete m_data, or to
// transfer it to another vector.
// ===================================================================
bool PrVector::dynamic() const
{
  return true;
}

// ===================================================================
// resize(): This virtual function resizes the vector without
// initializing the data.  It is used by setSize(), the assignment
// operator, etc.
//
// This function is normally overridden in the subclass.  For example,
// since a PrVector3 *always* has 3 elements, PrVector3 overrides this
// function to throw an assertion if newSize != 3.
// ===================================================================
void PrVector::resize( int newSize )
{
  SAIAssert( newSize >= 0 );

  if(  newSize > m_maxSize )
  {
    if( m_data != NULL )
    {
      delete[] m_data;
    }
    m_data = new Float[newSize];
    m_maxSize = newSize;
  }
  m_size = newSize;
}

// ===================================================================
// PrVectorSlice: PrVectorSlice is used by subclasses, especially the
// m_linear and m_angular components of a PrVector6.
// ===================================================================

PrVectorSlice::~PrVectorSlice()
{
  m_data = NULL;
  m_size = 0;
}

bool PrVectorSlice::dynamic() const
{
  return false;
}

void PrVectorSlice::resize( int newSize )
{
  SAIAssert( newSize == m_size );
}
