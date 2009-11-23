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
// SAIMatrix3.cpp
//
// Implementation of a 3x3 matrix.
//
// modification history
// --------------------
//
// 06/17/04: Dan Merget: Made into a subclass of SAIMatrix
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************


#include "SAIMatrix3.h"

const SAIMatrix3 SAIMatrix3::IDENTITY( 1, 0, 0,   0, 1, 0,   0, 0, 1 );
const SAIMatrix3 SAIMatrix3::ZERO;

// ===================================================================
// Destructor: In C++, the child destructor is called before the
// base-class destructor.  So by setting m_data back to NULL here, we
// ensure that the base class won't try to delete m_buff.
// ===================================================================
SAIMatrix3::~SAIMatrix3()
{
  m_data = NULL;
  m_row  = 0;
  m_col  = 0;
  m_size = 0;
}

// ===================================================================
// angularError(): Find the instantaneous angular error between this
// orientation and a desired orientation.  dPhi = R - Rd
//
// this and Rd should both be 3x3 rotation matrices.  This operation
// is the following equation, where "X" means cross product:
//    dPhi = -0.5 * ( R[x] X Rd[x]  +  R[y] X Rd[y]  +  R[z] X Rd[z] )
// ===================================================================
void SAIMatrix3::angularError( const SAIMatrix& Rd, SAIVector& dPhi ) const
{
  assert( Rd.row() == 3 && Rd.column() == 3 );
  dPhi.setSize( 3 );
  const SAIMatrix3& R = *this;

  dPhi[0] = -0.5 * ( R[1][0] * Rd[2][0] - R[2][0] * Rd[1][0] +
                     R[1][1] * Rd[2][1] - R[2][1] * Rd[1][1] +
                     R[1][2] * Rd[2][2] - R[2][2] * Rd[1][2] );
  dPhi[1] = -0.5 * ( R[2][0] * Rd[0][0] - R[0][0] * Rd[2][0] +
                     R[2][1] * Rd[0][1] - R[0][1] * Rd[2][1] +
                     R[2][2] * Rd[0][2] - R[0][2] * Rd[2][2] );
  dPhi[2] = -0.5 * ( R[0][0] * Rd[1][0] - R[1][0] * Rd[0][0] +
                     R[0][1] * Rd[1][1] - R[1][1] * Rd[0][1] +
                     R[0][2] * Rd[1][2] - R[1][2] * Rd[0][2] );
}

// ===================================================================
// setRotation(): Create a rotation matrix defined by a rotation axis
// and angle
// ===================================================================
void SAIMatrix3::setRotation( const SAIVector3& axis, Float angleRadian )
{
  Float c = cos( angleRadian );
  Float s = sin( angleRadian );
  Float v = 1-c;

  SAIVector3 normAxis( axis );
  normAxis.normalize();
  double x = normAxis[0];
  double y = normAxis[1];
  double z = normAxis[2];

  m_buff[0][0] = x*x*v +   c;
  m_buff[0][1] = x*y*v - z*s;
  m_buff[0][2] = x*z*v + y*s;

  m_buff[1][0] = x*y*v + z*s;
  m_buff[1][1] = y*y*v +   c;
  m_buff[1][2] = y*z*v - x*s;

  m_buff[2][0] = x*z*v - y*s;
  m_buff[2][1] = y*z*v + x*s;
  m_buff[2][2] = z*z*v +   c;
}

// ===================================================================
// setValues(): Same as the base-class method, but optimized for 3x3
// matrices.
// ===================================================================
void SAIMatrix3::setValues( const Float* rgVals, bool fTranspose )
{
  if( !fTranspose )
  {
    m_buff[0][0] = rgVals[0];
    m_buff[0][1] = rgVals[1];
    m_buff[0][2] = rgVals[2];
    m_buff[1][0] = rgVals[3];
    m_buff[1][1] = rgVals[4];
    m_buff[1][2] = rgVals[5];
    m_buff[2][0] = rgVals[6];
    m_buff[2][1] = rgVals[7];
    m_buff[2][2] = rgVals[8];
  }
  else
  {
    m_buff[0][0] = rgVals[0];
    m_buff[1][0] = rgVals[1];
    m_buff[2][0] = rgVals[2];
    m_buff[0][1] = rgVals[3];
    m_buff[1][1] = rgVals[4];
    m_buff[2][1] = rgVals[5];
    m_buff[0][2] = rgVals[6];
    m_buff[1][2] = rgVals[7];
    m_buff[2][2] = rgVals[8];
  }
}

// ===================================================================
// zero(): Same as the base-class method, but optimized for 3x3
// matrices.
// ===================================================================
void SAIMatrix3::zero()
{
  m_buff[0][0] = 0;
  m_buff[0][1] = 0;
  m_buff[0][2] = 0;
  m_buff[1][0] = 0;
  m_buff[1][1] = 0;
  m_buff[1][2] = 0;
  m_buff[2][0] = 0;
  m_buff[2][1] = 0;
  m_buff[2][2] = 0;
}

// ===================================================================
// identity(): Same as the base-class method, but optimized for 3x3
// matrices.
// ===================================================================
void SAIMatrix3::identity()
{
  m_buff[0][0] = 1;
  m_buff[0][1] = 0;
  m_buff[0][2] = 0;

  m_buff[1][0] = 0;
  m_buff[1][1] = 1;
  m_buff[1][2] = 0;

  m_buff[2][0] = 0;
  m_buff[2][1] = 0;
  m_buff[2][2] = 1;
}

// ===================================================================
// negate(): Same as the base-class method, but optimized for 3x3
// matrices.
// ===================================================================
void SAIMatrix3::negate( SAIMatrix& dest ) const
{
  dest.setSize( 3, 3 );
  dest[0][0] = -m_buff[0][0];
  dest[0][1] = -m_buff[0][1];
  dest[0][2] = -m_buff[0][2];
  dest[1][0] = -m_buff[1][0];
  dest[1][1] = -m_buff[1][1];
  dest[1][2] = -m_buff[1][2];
  dest[2][0] = -m_buff[2][0];
  dest[2][1] = -m_buff[2][1];
  dest[2][2] = -m_buff[2][2];
}

// ===================================================================
// Arithmetic operations: Same as the base-class method, but optimized
// for 3x3 matrices.
// ===================================================================
void SAIMatrix3::add( const SAIMatrix& rhs, SAIMatrix& dest ) const
{
  assert( rhs.row() == 3 && rhs.column() == 3 );
  dest.setSize( 3, 3 );
  dest[0][0] = m_buff[0][0] + rhs[0][0];
  dest[0][1] = m_buff[0][1] + rhs[0][1];
  dest[0][2] = m_buff[0][2] + rhs[0][2];
  dest[1][0] = m_buff[1][0] + rhs[1][0];
  dest[1][1] = m_buff[1][1] + rhs[1][1];
  dest[1][2] = m_buff[1][2] + rhs[1][2];
  dest[2][0] = m_buff[2][0] + rhs[2][0];
  dest[2][1] = m_buff[2][1] + rhs[2][1];
  dest[2][2] = m_buff[2][2] + rhs[2][2];
}

void SAIMatrix3::add( const SAIMatrix& rhs )
{
  assert( rhs.row() == 3 && rhs.column() == 3 );
  m_buff[0][0] += rhs[0][0];
  m_buff[0][1] += rhs[0][1];
  m_buff[0][2] += rhs[0][2];
  m_buff[1][0] += rhs[1][0];
  m_buff[1][1] += rhs[1][1];
  m_buff[1][2] += rhs[1][2];
  m_buff[2][0] += rhs[2][0];
  m_buff[2][1] += rhs[2][1];
  m_buff[2][2] += rhs[2][2];
}

void SAIMatrix3::subtract( const SAIMatrix& rhs, SAIMatrix& dest ) const
{
  assert( rhs.row() == 3 && rhs.column() == 3 );
  dest.setSize( 3, 3 );
  dest[0][0] = m_buff[0][0] - rhs[0][0];
  dest[0][1] = m_buff[0][1] - rhs[0][1];
  dest[0][2] = m_buff[0][2] - rhs[0][2];
  dest[1][0] = m_buff[1][0] - rhs[1][0];
  dest[1][1] = m_buff[1][1] - rhs[1][1];
  dest[1][2] = m_buff[1][2] - rhs[1][2];
  dest[2][0] = m_buff[2][0] - rhs[2][0];
  dest[2][1] = m_buff[2][1] - rhs[2][1];
  dest[2][2] = m_buff[2][2] - rhs[2][2];
}

void SAIMatrix3::subtract( const SAIMatrix& rhs )
{
  assert( rhs.row() == 3 && rhs.column() == 3 );
  m_buff[0][0] -= rhs[0][0];
  m_buff[0][1] -= rhs[0][1];
  m_buff[0][2] -= rhs[0][2];
  m_buff[1][0] -= rhs[1][0];
  m_buff[1][1] -= rhs[1][1];
  m_buff[1][2] -= rhs[1][2];
  m_buff[2][0] -= rhs[2][0];
  m_buff[2][1] -= rhs[2][1];
  m_buff[2][2] -= rhs[2][2];
}

void SAIMatrix3::multiply( const SAIMatrix& rhs, SAIMatrix& dest,
                                               bool fTranspose ) const
{
  if ( rhs.row() != 3 || rhs.column() != 3 )
  {
    // If rhs is not 3x3, then just use the generic parent-class method
    SAIMatrix::multiply( rhs, dest, fTranspose );
  }
  else if ( !fTranspose )
  {
    // dest = this * rhs, where rhs is 3x3
    assert( &dest != static_cast<const SAIMatrix*>(this) && &dest != &rhs );
    dest.setSize( 3, 3 );
    const Float (*a)[3] = m_buff;
    const Float (*b)[3] = reinterpret_cast<const Float(*)[3]>( rhs.dataPtr() );
    Float       (*c)[3] = reinterpret_cast<Float(*)[3]>( dest.dataPtr() );

    c[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0];
    c[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1];
    c[0][2] = a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2];

    c[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0];
    c[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1];
    c[1][2] = a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2];

    c[2][0] = a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0];
    c[2][1] = a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1];
    c[2][2] = a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2];
  }
  else
  {
    // dest = this * rhs^T, where rhs is 3x3
    assert( &dest != static_cast<const SAIMatrix*>(this) && &dest != &rhs );
    dest.setSize( 3, 3 );
    const Float (*a)[3] = m_buff;
    const Float (*b)[3] = reinterpret_cast<const Float(*)[3]>( rhs.dataPtr() );
    Float       (*c)[3] = reinterpret_cast<Float(*)[3]>( dest.dataPtr() );

    c[0][0] = a[0][0] * b[0][0] + a[0][1] * b[0][1] + a[0][2] * b[0][2];
    c[0][1] = a[0][0] * b[1][0] + a[0][1] * b[1][1] + a[0][2] * b[1][2];
    c[0][2] = a[0][0] * b[2][0] + a[0][1] * b[2][1] + a[0][2] * b[2][2];

    c[1][0] = a[1][0] * b[0][0] + a[1][1] * b[0][1] + a[1][2] * b[0][2];
    c[1][1] = a[1][0] * b[1][0] + a[1][1] * b[1][1] + a[1][2] * b[1][2];
    c[1][2] = a[1][0] * b[2][0] + a[1][1] * b[2][1] + a[1][2] * b[2][2];

    c[2][0] = a[2][0] * b[0][0] + a[2][1] * b[0][1] + a[2][2] * b[0][2];
    c[2][1] = a[2][0] * b[1][0] + a[2][1] * b[1][1] + a[2][2] * b[1][2];
    c[2][2] = a[2][0] * b[2][0] + a[2][1] * b[2][1] + a[2][2] * b[2][2];
  }
}

void SAIMatrix3::multiply( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( &dest != &rhs );
  assert( rhs.size() == 3 );
  dest.setSize( 3 );
  const Float (*lhs)[3] = m_buff;

  dest[0] = lhs[0][0] * rhs[0] + lhs[0][1] * rhs[1] + lhs[0][2] * rhs[2];
  dest[1] = lhs[1][0] * rhs[0] + lhs[1][1] * rhs[1] + lhs[1][2] * rhs[2];
  dest[2] = lhs[2][0] * rhs[0] + lhs[2][1] * rhs[1] + lhs[2][2] * rhs[2];
}

void SAIMatrix3::multiply( Float rhs, SAIMatrix& dest ) const
{
  dest.setSize( 3, 3 );
  dest[0][0] = m_buff[0][0] * rhs;
  dest[0][1] = m_buff[0][1] * rhs;
  dest[0][2] = m_buff[0][2] * rhs;
  dest[1][0] = m_buff[1][0] * rhs;
  dest[1][1] = m_buff[1][1] * rhs;
  dest[1][2] = m_buff[1][2] * rhs;
  dest[2][0] = m_buff[2][0] * rhs;
  dest[2][1] = m_buff[2][1] * rhs;
  dest[2][2] = m_buff[2][2] * rhs;
}

void SAIMatrix3::multiply( Float rhs )
{
  m_buff[0][0] *= rhs;
  m_buff[0][1] *= rhs;
  m_buff[0][2] *= rhs;
  m_buff[1][0] *= rhs;
  m_buff[1][1] *= rhs;
  m_buff[1][2] *= rhs;
  m_buff[2][0] *= rhs;
  m_buff[2][1] *= rhs;
  m_buff[2][2] *= rhs;
}

void SAIMatrix3::multiplyTranspose( const SAIMatrix& rhs, SAIMatrix& dest ) const
{
  if ( rhs.row() != 3 || rhs.column() != 3 )
  {
    // If rhs is not 3x3, then just use the generic parent-class method
    SAIMatrix::multiplyTranspose( rhs, dest );
  }
  else
  {
    // dest = this^T * rhs, where rhs is 3x3
    assert( &dest != static_cast<const SAIMatrix*>(this) && &dest != &rhs );
    dest.setSize( 3, 3 );
    const SAIMatrix3& a = *this;
    const SAIMatrix& b = rhs;

    dest[0][0] = a[0][0] * b[0][0] + a[1][0] * b[1][0] + a[2][0] * b[2][0];
    dest[0][1] = a[0][0] * b[0][1] + a[1][0] * b[1][1] + a[2][0] * b[2][1];
    dest[0][2] = a[0][0] * b[0][2] + a[1][0] * b[1][2] + a[2][0] * b[2][2];
    dest[1][0] = a[0][1] * b[0][0] + a[1][1] * b[1][0] + a[2][1] * b[2][0];
    dest[1][1] = a[0][1] * b[0][1] + a[1][1] * b[1][1] + a[2][1] * b[2][1];
    dest[1][2] = a[0][1] * b[0][2] + a[1][1] * b[1][2] + a[2][1] * b[2][2];
    dest[2][0] = a[0][2] * b[0][0] + a[1][2] * b[1][0] + a[2][2] * b[2][0];
    dest[2][1] = a[0][2] * b[0][1] + a[1][2] * b[1][1] + a[2][2] * b[2][1];
    dest[2][2] = a[0][2] * b[0][2] + a[1][2] * b[1][2] + a[2][2] * b[2][2];
  }
}

void SAIMatrix3::multiplyTranspose( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( &dest != &rhs );
  assert( rhs.size() == 3 );
  dest.setSize( 3 );
  const SAIMatrix3& lhs = *this;

  dest[0] = lhs[0][0] * rhs[0] + lhs[1][0] * rhs[1] + lhs[2][0] * rhs[2];
  dest[1] = lhs[0][1] * rhs[0] + lhs[1][1] * rhs[1] + lhs[2][1] * rhs[2];
  dest[2] = lhs[0][2] * rhs[0] + lhs[1][2] * rhs[1] + lhs[2][2] * rhs[2];
}

void SAIMatrix3::transpose( SAIMatrix& dest ) const
{
  assert( &dest != static_cast<const SAIMatrix*>(this) );
  dest.setSize( 3, 3 );
  dest[0][0] = m_buff[0][0];
  dest[0][1] = m_buff[1][0];
  dest[0][2] = m_buff[2][0];
  dest[1][0] = m_buff[0][1];
  dest[1][1] = m_buff[1][1];
  dest[1][2] = m_buff[2][1];
  dest[2][0] = m_buff[0][2];
  dest[2][1] = m_buff[1][2];
  dest[2][2] = m_buff[2][2];
}

// ===================================================================
// display(): Display the matrix
// ===================================================================
void SAIMatrix3::display( const char* name ) const
{
  if( name != NULL )
  {
    printf( "%s =\n[%.6f %.6f %.6f]\n[%.6f %.6f %.6f]\n[%.6f %.6f %.6f]\n",
            name,
            m_buff[0][0], m_buff[0][1], m_buff[0][2],
            m_buff[1][0], m_buff[1][1], m_buff[1][2],
            m_buff[2][0], m_buff[2][1], m_buff[2][2] );
  }
  else
  {
    printf( "[%.6f %.6f %.6f]\n[%.6f %.6f %.6f]\n[%.6f %.6f %.6f]\n",
            m_buff[0][0], m_buff[0][1], m_buff[0][2],
            m_buff[1][0], m_buff[1][1], m_buff[1][2],
            m_buff[2][0], m_buff[2][1], m_buff[2][2] );
  }
}

// ===================================================================
// dynamic(): This virtual function tells the base that m_data was
// not dynamically allocated in this object.
// ===================================================================
bool SAIMatrix3::dynamic() const
{
  return false;
}

// ===================================================================
// resize(): Virtual function that the base class uses to resize the
// matrix.  It is an error to resize a SAIMatrix3 to any size other
// than 3x3.
// ===================================================================
void SAIMatrix3::resize( int row, int col )
{
  assert( row == 3 && col == 3 );
}
