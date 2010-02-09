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
// PrVector6.cpp
//
// Subclass of PrVector, optimized for 6x1 vectors
//
// modification history
// --------------------
//
// 06/16/04: Dan Merget: Turned into a subclass of PrVector
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************
#include "PrVector3.h"
#include "PrMatrix6.h"
#include "PrVector6.h"

// ===================================================================
// Constructors: The PrVector6 constructors set the base class's
// m_data pointer to point to the internal buffer.  The destructor
// will set m_data back to NULL, so that the base class doesn't try to
// delete it.
// ===================================================================

PrVector6::PrVector6()
  : PrVector( 6, m_buff ),
    m_linearPart( 3, m_buff ), m_angularPart( 3, m_buff + 3 )
{
  m_buff[0] = 0;
  m_buff[1] = 0;
  m_buff[2] = 0;
  m_buff[3] = 0;
  m_buff[4] = 0;
  m_buff[5] = 0;
}

PrVector6::PrVector6( const PrVector6& rhs )
  : PrVector( 6, m_buff ),
    m_linearPart( 3, m_buff ), m_angularPart( 3, m_buff + 3 )
{
  m_buff[0] = rhs.m_buff[0];
  m_buff[1] = rhs.m_buff[1];
  m_buff[2] = rhs.m_buff[2];
  m_buff[3] = rhs.m_buff[3];
  m_buff[4] = rhs.m_buff[4];
  m_buff[5] = rhs.m_buff[5];
}

PrVector6::PrVector6( const PrVector& rhs )
  : PrVector( 6, m_buff ),
    m_linearPart( 3, m_buff ), m_angularPart( 3, m_buff + 3 )
{
  SAIAssert( rhs.size() == 6 );
  m_buff[0] = rhs[0];
  m_buff[1] = rhs[1];
  m_buff[2] = rhs[2];
  m_buff[3] = rhs[3];
  m_buff[4] = rhs[4];
  m_buff[5] = rhs[5];
}

PrVector6::PrVector6( const Float* rgVals )
  : PrVector( 6, m_buff ),
    m_linearPart( 3, m_buff ), m_angularPart( 3, m_buff + 3 )
{
  if( rgVals != NULL )
  {
    m_buff[0] = rgVals[0];
    m_buff[1] = rgVals[1];
    m_buff[2] = rgVals[2];
    m_buff[3] = rgVals[3];
    m_buff[4] = rgVals[4];
    m_buff[5] = rgVals[5];
  }
}

PrVector6::PrVector6( Float v0, Float v1, Float v2,
                      Float v3, Float v4, Float v5 )
  : PrVector( 6, m_buff ),
    m_linearPart( 3, m_buff ), m_angularPart( 3, m_buff + 3 )
{
  m_buff[0] = v0;
  m_buff[1] = v1;
  m_buff[2] = v2;
  m_buff[3] = v3;
  m_buff[4] = v4;
  m_buff[5] = v5;
}

PrVector6::PrVector6( const PrVector3& v0, const PrVector3& v1 )
  : PrVector( 6, m_buff ),
    m_linearPart( 3, m_buff ), m_angularPart( 3, m_buff + 3 )
{
  m_buff[0] = v0[0];
  m_buff[1] = v0[1];
  m_buff[2] = v0[2];

  m_buff[3] = v1[0];
  m_buff[4] = v1[1];
  m_buff[5] = v1[2];
}

// ===================================================================
// Destructor: In C++, the child destructor is called before the
// base-class destructor.  So by setting m_data back to NULL here, we
// ensure that the base class won't try to delete m_buff.
// ===================================================================
PrVector6::~PrVector6()
{
  m_data = NULL;
  m_size = 0;
  m_maxSize = 0;
}

// ===================================================================
// values(): Load all 6 values at once.
// ===================================================================
void PrVector6::values( Float v0, Float v1, Float v2,
                        Float v3, Float v4, Float v5 )
{
  m_buff[0] = v0;
  m_buff[1] = v1;
  m_buff[2] = v2;
  m_buff[3] = v3;
  m_buff[4] = v4;
  m_buff[5] = v5;
}

// ===================================================================
// The following functions are the same as the base-class methods,
// but optimized for 6x1 vectors.
// ===================================================================

PrVector6& PrVector6::operator=( const PrVector& rhs )
{
  SAIAssert(rhs.size() == 6);
  if( this != &rhs )  {
    m_buff[0] = rhs[0];
    m_buff[1] = rhs[1];
    m_buff[2] = rhs[2];
    m_buff[3] = rhs[3];
    m_buff[4] = rhs[4];
    m_buff[5] = rhs[5];
  }
  return *this;
}

void PrVector6::zero()
{
  m_buff[0] = 0;
  m_buff[1] = 0;
  m_buff[2] = 0;
  m_buff[3] = 0;
  m_buff[4] = 0;
  m_buff[5] = 0;
}

bool PrVector6::operator==( const PrVector& rhs ) const
{
  return rhs.size() == 6 &&
         m_buff[0] == rhs[0] &&
         m_buff[1] == rhs[1] &&
         m_buff[2] == rhs[2] &&
         m_buff[3] == rhs[3] &&
         m_buff[4] == rhs[4] &&
         m_buff[5] == rhs[5];
}

void PrVector6::negate( PrVector& dest ) const
{
  dest.setSize( 6 );
  dest[0] = -m_buff[0];
  dest[1] = -m_buff[1];
  dest[2] = -m_buff[2];
  dest[3] = -m_buff[3];
  dest[4] = -m_buff[4];
  dest[5] = -m_buff[5];
}

void PrVector6::add( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( rhs.size() == 6 );
  dest.setSize( 6 );
  dest[0] = m_buff[0] + rhs[0];
  dest[1] = m_buff[1] + rhs[1];
  dest[2] = m_buff[2] + rhs[2];
  dest[3] = m_buff[3] + rhs[3];
  dest[4] = m_buff[4] + rhs[4];
  dest[5] = m_buff[5] + rhs[5];
}

void PrVector6::add( const PrVector& rhs )
{
  SAIAssert( rhs.size() == 6 );
  m_buff[0] += rhs[0];
  m_buff[1] += rhs[1];
  m_buff[2] += rhs[2];
  m_buff[3] += rhs[3];
  m_buff[4] += rhs[4];
  m_buff[5] += rhs[5];
}

void PrVector6::subtract( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( rhs.size() == 6 );
  dest.setSize( 6 );
  dest[0] = m_buff[0] - rhs[0];
  dest[1] = m_buff[1] - rhs[1];
  dest[2] = m_buff[2] - rhs[2];
  dest[3] = m_buff[3] - rhs[3];
  dest[4] = m_buff[4] - rhs[4];
  dest[5] = m_buff[5] - rhs[5];
}

void PrVector6::subtract( const PrVector& rhs )
{
  SAIAssert( rhs.size() == 6 );
  m_buff[0] -= rhs[0];
  m_buff[1] -= rhs[1];
  m_buff[2] -= rhs[2];
  m_buff[3] -= rhs[3];
  m_buff[4] -= rhs[4];
  m_buff[5] -= rhs[5];
}

void PrVector6::multiply( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( rhs.size() == 6 );
  dest.setSize( 6 );
  dest[0] = m_buff[0] * rhs[0];
  dest[1] = m_buff[1] * rhs[1];
  dest[2] = m_buff[2] * rhs[2];
  dest[3] = m_buff[3] * rhs[3];
  dest[4] = m_buff[4] * rhs[4];
  dest[5] = m_buff[5] * rhs[5];
}

void PrVector6::multiply( const PrVector& rhs )
{
  SAIAssert( rhs.size() == 6 );
  m_buff[0] *= rhs[0];
  m_buff[1] *= rhs[1];
  m_buff[2] *= rhs[2];
  m_buff[3] *= rhs[3];
  m_buff[4] *= rhs[4];
  m_buff[5] *= rhs[5];
}

void PrVector6::multiply( Float rhs, PrVector& dest ) const
{
  dest.setSize( 6 );
  dest[0] = m_buff[0] * rhs;
  dest[1] = m_buff[1] * rhs;
  dest[2] = m_buff[2] * rhs;
  dest[3] = m_buff[3] * rhs;
  dest[4] = m_buff[4] * rhs;
  dest[5] = m_buff[5] * rhs;
}

void PrVector6::multiply( Float rhs )
{
  m_buff[0] *= rhs;
  m_buff[1] *= rhs;
  m_buff[2] *= rhs;
  m_buff[3] *= rhs;
  m_buff[4] *= rhs;
  m_buff[5] *= rhs;
}

// ===================================================================
// multiplyTransposed(): Same as the base-class method, but optimized
// for 6x1 vectors. (calculate this * rhs^T)
// ===================================================================

PrMatrix6 PrVector6::multiplyTransposed( const PrVector6& rhs ) const
{
  PrMatrix6 tmp( NULL );
  multiplyTransposed( rhs, tmp );
  return tmp;
}

PrMatrix PrVector6::multiplyTransposed( const PrVector& rhs ) const
{
  PrMatrix tmp( NULL, 6, rhs.size() );
  multiplyTransposed( rhs, tmp );
  return tmp;
}

void PrVector6::multiplyTransposed( const PrVector& rhs, PrMatrix& dest ) const
{
  if ( rhs.size() != 6 )
  {
    // If rhs is not a 6-element vector, then use the generic base-class method
    PrVector::multiplyTransposed( rhs, dest );
  }
  else
  {
    // Multiply by a 1x6 vector to create a 6x6 matrix
    dest.setSize( 6, 6 );

    for( int ii = 0; ii < 6; ii++ )
    {
      Float *destRow = dest[ii];
      Float  lhsI    = m_buff[ii];
      destRow[0] = lhsI * rhs[0];
      destRow[1] = lhsI * rhs[1];
      destRow[2] = lhsI * rhs[2];
      destRow[3] = lhsI * rhs[3];
      destRow[4] = lhsI * rhs[4];
      destRow[5] = lhsI * rhs[5];
    }
  }
}

// ===================================================================
// dot(): Same as the base-class method, but optimized for 6x1
// vectors.
// ===================================================================
Float PrVector6::dot( const PrVector& rhs ) const
{
  SAIAssert( rhs.size() == 6 );
  return m_data[0] * rhs[0] +
         m_data[1] * rhs[1] +
         m_data[2] * rhs[2] +
         m_data[3] * rhs[3] +
         m_data[4] * rhs[4] +
         m_data[5] * rhs[5];
}

// ===================================================================
// display(): Display the vector
// ===================================================================
void PrVector6::display( const char* name ) const
{
  if( name != NULL )
  {
    printf( "%s =\n((%f %f %f) (%f %f %f))\n", name,
            m_buff[0], m_buff[1], m_buff[2], m_buff[3], m_buff[4], m_buff[5] );
  }
  else
  {
    printf( "((%f %f %f) (%f %f %f))",
            m_buff[0], m_buff[1], m_buff[2], m_buff[3], m_buff[4], m_buff[5] );
  }
}

// ===================================================================
// dynamic(): This virtual function tells the base that m_data was
// not dynamically allocated in this object.
// ===================================================================
bool PrVector6::dynamic() const
{
  return false;
}

// ===================================================================
// resize(): Virtual function that the base class uses to resize the
// vector.  It is an error to resize a PrVector6 to any size other
// than 6.
// ===================================================================
void PrVector6::resize( int newSize )
{
  SAIAssert( newSize == 6 );
}
