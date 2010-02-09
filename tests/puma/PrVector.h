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
// PrVector.h
//
// This class provides an Nx1 vector.
//
//  modification history
// ----------------------
//
//  06/16/04: Dan Merget: made PrVector3 & PrVector6 into subclasses
//  11/12/97: K.C. Chang: created.
// *********************************************************************
#ifndef _PrVector_h
#define _PrVector_h

#include <cstring>
#include "XUtils.h"
#include "PrGlobalDefn.h"
#include "PrMathDefn.h"
class PrMatrix;
class PrVector3;
class PrMatrix3;

// ===================================================================
// PrVector class declaration
// ===================================================================

class PrVector
{
public:
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------

  // Initializes the vector with zero values.
  explicit PrVector( int size );

  // Copy constructor
  PrVector( const PrVector& rhs );

  // Creates empty vector
  PrVector() : m_size( 0 ), m_maxSize( 0 ), m_data( NULL ) {}

  // Load from array.  If rgVals is NULL, set size w/out setting values.
  PrVector( const Float* rgVals, int size );

  virtual ~PrVector();

  // -----------------------------------------------------------------
  // Operations that load data into vector
  // -----------------------------------------------------------------

  PrVector& operator=( const PrVector& rhs );

  // we'll transfer Min( cVals, m_size ) values into m_data
  void setValues( const Float* rgVals, int cVals );
  void setValues( const Float* rgVals )     { setValues( rgVals, m_size ); }
  void getValues( Float* rgVals, int cVals ) const;
  void getValues( Float* rgVals ) const     { getValues( rgVals, m_size ); }

  // load vector with all zeros
  void zero();

  // test if this is a zero vector
  bool fZero() const;

  // Append a vector to end; enlarges this vector to fit
  void append( const PrVector &rhs );

  // Set the dimension and optionally zero the vector
  void setSize( int size, bool fZero = false );

  // Replace contents of this vector with those of src, without
  // allocating memory if possible.  Puts src in an undefined state.
  void transfer( PrVector& src );

  // -----------------------------------------------------------------
  // Basic lookup operations
  // -----------------------------------------------------------------

  Float& operator[](int ii )             { return elementAt(ii); }
  const Float& operator[](int ii ) const { return elementAt(ii); }

  Float& elementAt( int ii );
  const Float& elementAt( int ii ) const;

  int  size()    const { return m_size; };
  bool fFinite() const;
  bool fEmpty()  const { return (m_size == 0); }

  PrVector subvector( int start, int size ) const;
  void     subvector( int start, PrVector& dest ) const;
  void     setSubvector( int start, const PrVector& src );

  // cast operator (for compatability of array of doubles, for example
  operator const Float*() const { return m_data; }
  Float*        dataPtr()       { return m_data; }
  const Float*  dataPtr() const { return m_data; }


  // -----------------------------------------------------------------
  // Arithmetic operations
  // -----------------------------------------------------------------

  bool operator==( const PrVector& rhs ) const;

  PrVector operator-() const;
  void negate( PrVector& dest ) const;

  PrVector operator+( const PrVector& rhs ) const;
  void add( const PrVector& rhs, PrVector& dest ) const;
  void add( const PrVector& rhs );

  PrVector operator-( const PrVector& rhs ) const;   
  void subtract( const PrVector& rhs, PrVector& dest ) const;
  void subtract( const PrVector& rhs );

  PrVector operator*( const PrMatrix& rhs ) const;

  // Element-wise multiplication
  PrVector operator*( const PrVector& rhs ) const;
  void multiply( const PrVector& rhs, PrVector& dest ) const;
  void multiply( const PrVector& rhs );

  PrVector operator*( Float rhs ) const;
  void multiply( Float rhs, PrVector& dest ) const;
  void multiply( Float rhs );

  PrVector operator/(Float rhs ) const;
  void divide( Float rhs, PrVector& dest ) const { multiply( 1/rhs, dest ); }
  void divide( Float rhs )                       { multiply( 1/rhs ); }

  PrVector& operator+=( const PrVector& rhs ) { add(rhs); return *this; }
  PrVector& operator-=( const PrVector& rhs ) { subtract(rhs); return *this; }
  PrVector& operator*=( Float rhs )           { multiply(rhs); return *this; }
  PrVector& operator/=( Float rhs )           { divide(rhs); return *this; }

  // Multiply a vertical vector by a horizontal one, to make a matrix
  PrMatrix multiplyTransposed( const PrVector& rhs ) const;
  void multiplyTransposed( const PrVector& rhs, PrMatrix& dest ) const;

  Float dot( const PrVector& rhs ) const;
  Float lengthSquared() const;
  Float magnitude()     const { return sqrt(lengthSquared()); }
  Float length()        const { return magnitude(); }
  Float abs()           const { return magnitude(); }
  const PrVector& normalize() { *this /= magnitude(); return *this; }

  // 3-element vectors only
  PrVector3 cross( const PrVector& rhs ) const;
  void cross( const PrVector& rhs, PrVector& dest ) const;
  PrMatrix3 cross() const;
  void cross( PrMatrix& dest ) const;

  // -----------------------------------------------------------------
  // Miscellaneous operations
  // -----------------------------------------------------------------
  void display( const char* name = NULL ) const;

protected:
  // Special constructor for non-dynamic subclasses
  PrVector( int size, Float* data );

  // Override these methods in non-dynamic subclasses
  virtual bool dynamic() const;
  virtual void resize( int newSize );

  int    m_size;
  int    m_maxSize;
  Float* m_data;
};

static PrVector s_nullPrVector( 0 );

// ===================================================================
// PrVectorSlice: Used by some subclasses for treating part of a
// PrVector as a full vector
// ===================================================================

class PrVectorSlice : public PrVector
{
public:
  PrVectorSlice( int size, Float* data ) : PrVector( size, data ) {}
  ~PrVectorSlice();
protected:
  virtual bool dynamic() const;
  virtual void resize( int newSize );
};

// ===================================================================
// Inline methods
// ===================================================================

inline PrVector::PrVector( int size, Float* data )
  : m_size( size ), m_maxSize( size ), m_data( data )
{
  SAIAssert( m_size >= 0 );
  SAIAssert( m_size == 0 || m_data != NULL );
}

inline void PrVector::zero()
{
  if( m_size > 0 ) {
    memset( m_data, 0, sizeof( Float ) * m_size );
  }
}

inline Float& PrVector::elementAt( int ii )
{
  SAIAssert( ii >= 0 && ii < m_size );
  return m_data[ii];
}

inline const Float& PrVector::elementAt( int ii ) const
{
  SAIAssert( ii >= 0 && ii < m_size );
  return m_data[ii];
}

inline PrVector PrVector::subvector( int start, int size ) const
{
  PrVector tmp( NULL, size );
  subvector( start, tmp );
  return tmp;
}

inline PrVector PrVector::operator-() const
{
  PrVector tmp( NULL, m_size );
  negate( tmp );
  return tmp;
}

inline PrVector PrVector::operator+( const PrVector& rhs ) const
{
  PrVector tmp( NULL, m_size );
  add( rhs, tmp );
  return tmp;
}

inline PrVector PrVector::operator-( const PrVector& rhs ) const
{
  PrVector tmp( NULL, m_size );
  subtract( rhs, tmp );
  return tmp;
}

inline PrVector PrVector::operator*( const PrVector& rhs ) const
{
  PrVector tmp( NULL, m_size );
  multiply( rhs, tmp );
  return tmp;
}

inline PrVector PrVector::operator*( Float rhs ) const
{
  PrVector tmp( NULL, m_size );
  multiply( rhs, tmp );
  return tmp;
}

inline PrVector operator*( Float lhs, const PrVector& rhs )
{
  PrVector tmp( NULL, rhs.size() );
  rhs.multiply( lhs, tmp );
  return tmp;
}

inline PrVector PrVector::operator/( Float rhs ) const
{
  PrVector tmp( NULL, m_size );
  divide( rhs, tmp );
  return tmp;
}

inline bool PrVector::fZero() const
{
  for( int ii=0; ii < m_size; ii++ )
  {
    if( elementAt( ii ) != 0.0 )
    {
      return false;
    }
  }

  return true;
}

#endif // _PrVector_h
