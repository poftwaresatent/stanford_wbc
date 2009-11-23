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
// SAIVector.h
//
// This class provides an Nx1 vector.
//
//  modification history
// ----------------------
//
//  07/14/05: James Warren: cleaning for stand-alone build
//  06/16/04: Dan Merget: made SAIVector3 & SAIVector6 into subclasses
//  11/12/97: K.C. Chang: created.
// *********************************************************************
#ifndef _SAIVector_h
#define _SAIVector_h

#include <assert.h>
#include <cstring>
#include <iostream>
#include "SAIGlobalDefn.h"
#include "SAIMathDefn.h"
class SAIMatrix;
class SAIVector3;
class SAIMatrix3;

// ===================================================================
// SAIVector class declaration
// ===================================================================

class SAIVector
{
public:
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------

  // Initializes the vector with zero values.
  explicit SAIVector( int size );

  // Copy constructor
  SAIVector( const SAIVector& rhs );

  // Creates empty vector
  SAIVector() : m_size( 0 ), m_maxSize( 0 ), m_data( NULL ) {}

  // Load from array.  If rgVals is NULL, set size w/out setting values.
  SAIVector( const Float* rgVals, int size );

  virtual ~SAIVector();

  // -----------------------------------------------------------------
  // Operations that load data into vector
  // -----------------------------------------------------------------

  SAIVector& operator=( const SAIVector& rhs );

  // transfer Min( cVals, m_size ) values into m_data
  void setValues( const Float* rgVals, int cVals );
  void setValues( const Float* rgVals )     { setValues( rgVals, m_size ); }
  void setConstantValue( Float value, int size = -1 );
  void getValues( Float* rgVals, int cVals ) const;
  void getValues( Float* rgVals ) const     { getValues( rgVals, m_size ); }

  // load vector with all zeros
  void zero();

  // test if this is a zero vector
  bool fZero() const;

  // Append a vector to end; enlarges this vector to fit
  void append( const SAIVector &rhs );

  // Set the dimension and optionally zero the vector
  void setSize( int size, bool fZero = false );

  // matrix conversion
  SAIMatrix mat(bool column = true) const;

  // Replace contents of this vector with those of src, without
  // allocating memory if possible.  Puts src in an undefined state.
  void transfer( SAIVector& src );

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

  SAIVector subvector( int start, int size ) const;
  void     subvector( int start, SAIVector& dest ) const;
  void     setSubvector( int start, const SAIVector& src );

  // cast operator (for compatability of array of doubles, for example
  operator const Float*() const { return m_data; }
  Float*        dataPtr()       { return m_data; }
  const Float*  dataPtr() const { return m_data; }

  // -----------------------------------------------------------------
  // Arithmetic operations
  // -----------------------------------------------------------------
  
  /** A less braindead implementation of operator==() */
  bool equal(SAIVector const & rhs, Float precision) const;
  
  bool operator==( const SAIVector& rhs ) const;

  SAIVector operator-() const;
  void negate( SAIVector& dest ) const;

  SAIVector operator+( const SAIVector& rhs ) const;
  void add( const SAIVector& rhs, SAIVector& dest ) const;
  void add( const SAIVector& rhs );

  SAIVector operator-( const SAIVector& rhs ) const;   
  void subtract( const SAIVector& rhs, SAIVector& dest ) const;
  void subtract( const SAIVector& rhs );

  SAIVector operator*( const SAIMatrix& rhs ) const;

  // Element-wise multiplication
  SAIVector operator*( const SAIVector& rhs ) const;
  void multiply( const SAIVector& rhs, SAIVector& dest ) const;
  void multiply( const SAIVector& rhs );

  SAIVector operator*( Float rhs ) const;
  void multiply( Float rhs, SAIVector& dest ) const;
  void multiply( Float rhs );

  // Element-wise division
  SAIVector operator/( const SAIVector& rhs ) const;
  void divide( const SAIVector& rhs, SAIVector& dest ) const;
  void divide( const SAIVector& rhs );

  SAIVector operator/(Float rhs ) const;
  void divide( Float rhs, SAIVector& dest ) const { multiply( 1/rhs, dest ); }
  void divide( Float rhs )                       { multiply( 1/rhs ); }
  
  SAIVector& operator+=( const SAIVector& rhs ) { add(rhs); return *this; }
  SAIVector& operator-=( const SAIVector& rhs ) { subtract(rhs); return *this; }
  SAIVector& operator*=( Float rhs )           { multiply(rhs); return *this; }
  SAIVector& operator/=( Float rhs )           { divide(rhs); return *this; }

  // Multiply a vertical vector by a horizontal one, to make a matrix
  SAIMatrix multiplyTransposed( const SAIVector& rhs ) const;
  void multiplyTransposed( const SAIVector& rhs, SAIMatrix& dest ) const;

  Float dot( const SAIVector& rhs ) const;
  Float lengthSquared() const;
  Float magnitude()     const { return sqrt(lengthSquared()); }
  Float length()        const { return magnitude(); }
  Float abs()           const { return magnitude(); }
  const SAIVector& normalize() { *this /= magnitude(); return *this; }

  // max/min values
  double maxValue();
  double minValue();

  // apply function to each element
  void applyFunction( double (*fcn)(double), SAIVector& dest ) const;
  void applyFunction( double (*fcn)(double) );

  // 3-element vectors only
  SAIVector3 cross( const SAIVector& rhs ) const;
  void cross( const SAIVector& rhs, SAIVector& dest ) const;
  SAIMatrix3 cross() const;
  void cross( SAIMatrix& dest ) const;

  // -----------------------------------------------------------------
  // Miscellaneous operations
  // -----------------------------------------------------------------
  void display( const char* name = NULL ) const;
  void prettyPrint (std::ostream & os, std::string const & title, std::string const & prefix) const;
  std::string prettyString(std::string const & title, std::string const & prefix) const;
  friend std::ostream& operator<<(std::ostream& os, const SAIVector& v);
  friend std::istream& operator>>(std::istream& is, SAIVector& v);

protected:
  // Special constructor for non-dynamic subclasses
  SAIVector( int size, Float* data );

  // Override these methods in non-dynamic subclasses
  virtual bool dynamic() const;
  virtual void resize( int newSize );

  int    m_size;
  int    m_maxSize;
  Float* m_data;
};

static SAIVector s_nullSAIVector( 0 );

// ===================================================================
// SAIVectorSlice: Used by some subclasses for treating part of a
// SAIVector as a full vector
// ===================================================================

class SAIVectorSlice : public SAIVector
{
public:
  SAIVectorSlice( int size, Float* data ) : SAIVector( size, data ) {}
  ~SAIVectorSlice();
protected:
  virtual bool dynamic() const;
  virtual void resize( int newSize );
};

// ===================================================================
// Inline methods
// ===================================================================

inline SAIVector::SAIVector( int size, Float* data )
  : m_size( size ), m_maxSize( size ), m_data( data )
{
  assert( m_size >= 0 );
  assert( m_size == 0 || m_data != NULL );
}

inline void SAIVector::zero()
{
  if( m_size > 0 ) {
    memset( m_data, 0, sizeof( Float ) * m_size );
  }
}

inline Float& SAIVector::elementAt( int ii )
{
  assert( ii >= 0 && ii < m_size );
  return m_data[ii];
}

inline const Float& SAIVector::elementAt( int ii ) const
{
  assert( ii >= 0 && ii < m_size );
  return m_data[ii];
}

inline SAIVector SAIVector::subvector( int start, int size ) const
{
  SAIVector tmp( NULL, size );
  subvector( start, tmp );
  return tmp;
}

inline SAIVector SAIVector::operator-() const
{
  SAIVector tmp( NULL, m_size );
  negate( tmp );
  return tmp;
}

inline SAIVector SAIVector::operator+( const SAIVector& rhs ) const
{
  SAIVector tmp( NULL, m_size );
  add( rhs, tmp );
  return tmp;
}

inline SAIVector SAIVector::operator-( const SAIVector& rhs ) const
{
  SAIVector tmp( NULL, m_size );
  subtract( rhs, tmp );
  return tmp;
}

inline SAIVector SAIVector::operator*( const SAIVector& rhs ) const
{
  SAIVector tmp( NULL, m_size );
  multiply( rhs, tmp );
  return tmp;
}

inline SAIVector SAIVector::operator/( const SAIVector& rhs ) const
{
  SAIVector tmp( NULL, m_size );
  divide( rhs, tmp );
  return tmp;
}

inline SAIVector SAIVector::operator*( Float rhs ) const
{
  SAIVector tmp( NULL, m_size );
  multiply( rhs, tmp );
  return tmp;
}

inline SAIVector operator*( Float lhs, const SAIVector& rhs )
{
  SAIVector tmp( NULL, rhs.size() );
  rhs.multiply( lhs, tmp );
  return tmp;
}

inline SAIVector SAIVector::operator/( Float rhs ) const
{
  SAIVector tmp( NULL, m_size );
  divide( rhs, tmp );
  return tmp;
}

inline bool SAIVector::fZero() const
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

#endif // _SAIVector_h
