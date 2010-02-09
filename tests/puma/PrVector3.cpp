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
// PrVector3.cpp
//
// Subclass of PrVector, optimized for 3x1 vectors
//
// modification history
// --------------------
//
// 06/16/04: Dan Merget: Turned into a subclass of PrVector
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************
#include "PrVector3.h"
#include "PrMatrix3.h"

// ===================================================================
// Destructor: In C++, the child destructor is called before the
// base-class destructor.  So by setting m_data back to NULL here, we
// ensure that the base class won't try to delete m_buff.
// ===================================================================
PrVector3::~PrVector3()
{
  m_data = NULL;
  m_size = 0;
  m_maxSize = 0;
}

// ===================================================================
// multiplyTransposed(): Same as the base-class method, but optimized
// for 3x1 vectors. (calculate this * rhs^T)
// ===================================================================

PrMatrix3 PrVector3::multiplyTransposed( const PrVector3& rhs ) const
{
  return PrMatrix3( m_buff[0] * rhs.m_buff[0],
                    m_buff[0] * rhs.m_buff[1],
                    m_buff[0] * rhs.m_buff[2],

                    m_buff[1] * rhs.m_buff[0],
                    m_buff[1] * rhs.m_buff[1],
                    m_buff[1] * rhs.m_buff[2],

                    m_buff[2] * rhs.m_buff[0],
                    m_buff[2] * rhs.m_buff[1],
                    m_buff[2] * rhs.m_buff[2] );
}

PrMatrix PrVector3::multiplyTransposed( const PrVector& rhs ) const
{
  PrMatrix tmp( NULL, 3, rhs.size() );
  multiplyTransposed( rhs, tmp );
  return tmp;
}

void PrVector3::multiplyTransposed( const PrVector& rhs, PrMatrix& dest ) const
{
  if ( rhs.size() != 3 )
  {
    // If rhs is not a 3-element vector, then use the generic base-class method
    PrVector::multiplyTransposed( rhs, dest );
  }
  else
  {
    // Multiply by a 1x3 vector to create a 3x3 matrix
    dest.setSize( 3, 3 );

    dest[0][0]=  m_buff[0] * rhs[0];
    dest[0][1]=  m_buff[0] * rhs[1];
    dest[0][2]=  m_buff[0] * rhs[2];

    dest[1][0]=  m_buff[1] * rhs[0];
    dest[1][1]=  m_buff[1] * rhs[1];
    dest[1][2]=  m_buff[1] * rhs[2];

    dest[2][0]=  m_buff[2] * rhs[0];
    dest[2][1]=  m_buff[2] * rhs[1];
    dest[2][2]=  m_buff[2] * rhs[2];
  }
}

// ===================================================================
// display(): Display the vector
// ===================================================================
void PrVector3::display( const char* name ) const
{
  if( name != NULL )
  {
    printf("%s = (%f %f %f)\n", name, m_buff[0], m_buff[1], m_buff[2] );
  }
  else
  {
    printf("(%f %f %f)", m_buff[0], m_buff[1], m_buff[2] );
  }
}

// ===================================================================
// dynamic(): This virtual function tells the base that m_data was
// not dynamically allocated in this object.
// ===================================================================
bool PrVector3::dynamic() const
{
  return false;
}

// ===================================================================
// resize(): Virtual function that the base class uses to resize the
// vector.  It is an error to resize a PrVector3 to any size other
// than 3.
// ===================================================================
void PrVector3::resize( int newSize )
{
  SAIAssert( newSize == 3 );
}
