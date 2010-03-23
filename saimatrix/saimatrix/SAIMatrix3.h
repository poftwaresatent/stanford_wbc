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
// SAIMatrix3.h
//
// This class provides a 3x3 matrix.
//
// modification history
// --------------------
//
// 06/17/04: Dan Merget: Made into a subclass of SAIMatrix
// 04/17/98: K.C. Chang: added inverse.
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************
#ifndef _SAIMatrix3_h
#define _SAIMatrix3_h

#include "SAIVector3.h"
#include <saimatrix/SAIMatrix.h>

// ===================================================================
// SAIMatrix3 class declaration
// ===================================================================

class SAIMatrix3 : public SAIMatrix
{
public:
  // -----------------------------------------------------------------
  // A few useful matrices
  // -----------------------------------------------------------------
  static const SAIMatrix3 IDENTITY;
  static const SAIMatrix3 ZERO;
  
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------
  SAIMatrix3(); // initialized to [0]
  SAIMatrix3( const SAIMatrix3& rhs );
  explicit SAIMatrix3( const SAIMatrix& rhs );
  explicit SAIMatrix3( const Float* rgVals, bool fTranspose = false );
  SAIMatrix3( Float x1, Float x2, Float x3,
             Float x4, Float x5, Float x6,
             Float x7, Float x8, Float x9 );
  virtual ~SAIMatrix3();

  // -----------------------------------------------------------------
  // Operations specific to SAIMatrix3
  // -----------------------------------------------------------------
  void loadFromMatrix33( const Float (&data)[3][3] )  { setValues( data[0] ); }

  // dPhi = R - Rd
  SAIVector3 angularError( const SAIMatrix& Rd ) const;
  void angularError( const SAIMatrix& Rd, SAIVector& dPhi ) const;

  void setDiagonal( Float d0, Float d1, Float d2 );

  // Generate a rotation matrix defined by a rotation axis and angle
  void setRotation( const SAIVector3& axis, Float angleRadian );

  // -----------------------------------------------------------------
  // This functions in this section are identical to functions in the
  // base class, except that they have been optimized for 3x3
  // matrices.
  //
  // Some of these functions have been changed to return a SAIMatrix3
  // instead of a SAIMatrix, but the arguments are the same.  For
  // example, the assignment operator accepts any 3x3 SAIMatrix on the
  // rhs, not just a SAIMatrix.  This lets us freely mix SAIMatrix and
  // SAIMatrix3 objects.
  // -----------------------------------------------------------------
  SAIVector3    diagonal() const;
  void         diagonal( SAIVector& dest ) const;
  void         setDiagonal( Float src );
  void         setDiagonal( const Float* src );
  void         setDiagonal( const SAIVector& src );
  void         getRow( int row, SAIVector& dest ) const;
  void         getColumn( int col, SAIVector& dest ) const;
  SAIMatrix3&   operator=( const SAIMatrix& rhs );
  void         setValues( const Float* rgVals, bool fTranspose = false );
  void         zero();
  void         identity();
  Float*       operator[]( int row );
  const Float* operator[]( int row ) const;
  Float&       operator()(int row, int col)        {return elementAt(row,col);}
  const Float& operator()(int row, int col)  const {return elementAt(row,col);}
  Float&       elementAt( int row, int col );
  const Float& elementAt( int row, int col ) const;
  SAIMatrix3    operator-() const;
  void         negate( SAIMatrix& dest ) const;
  SAIMatrix3    operator+( const SAIMatrix& rhs ) const;
  void         add( const SAIMatrix& rhs, SAIMatrix& dest ) const;
  void         add( const SAIMatrix& rhs );
  SAIMatrix3    operator-( const SAIMatrix& rhs ) const;
  void         subtract( const SAIMatrix& rhs, SAIMatrix& dest ) const;
  void         subtract( const SAIMatrix& rhs ); 
  SAIMatrix3    operator*( const SAIMatrix3& rhs ) const;
  SAIMatrix     operator*( const SAIMatrix& rhs ) const;
  void         multiply( const SAIMatrix& rhs, SAIMatrix& dest,
                                              bool fTranspose = false ) const;
  void         multiply( const SAIMatrix& rhs, bool fTranspose = false );
  SAIVector3    operator*( const SAIVector& rhs ) const;
  void         multiply( const SAIVector& rhs, SAIVector& dest ) const;
  SAIMatrix3    operator*( Float rhs ) const;
  void         multiply( Float rhs, SAIMatrix& dest ) const;
  void         multiply( Float rhs );
  SAIMatrix3    operator/( Float rhs ) const;
  void         divide(Float rhs, SAIMatrix& dest) const {multiply(1/rhs,dest);}
  void         divide(Float rhs)                       { multiply( 1/rhs ); }
  SAIMatrix3& operator+=( const SAIMatrix& rhs ) { add(rhs); return *this; }
  SAIMatrix3& operator-=( const SAIMatrix& rhs ) { subtract(rhs); return *this; }
  SAIMatrix3& operator*=( const SAIMatrix& rhs ) { multiply(rhs); return *this; }
  SAIMatrix3& operator*=( Float rhs )           { multiply(rhs); return *this; }
  SAIMatrix3& operator/=( Float rhs )           { divide(rhs); return *this; }
  void      multiplyTranspose( const SAIMatrix& rhs, SAIMatrix& dest) const;
  void      multiplyTranspose( const SAIVector& rhs, SAIVector& dest) const;
  SAIMatrix3 transpose() const;
  void      transpose( SAIMatrix& dest ) const;
  void      display( const char* name = NULL ) const;

private:
  virtual bool dynamic() const;
  virtual void resize( int row, int col );
  Float m_buff[3][3];
};

// ===================================================================
// Inline methods
// ===================================================================

inline SAIMatrix3::SAIMatrix3() : SAIMatrix( 3, 3, m_buff[0] )
{
  zero();
}

inline SAIMatrix3::SAIMatrix3( const SAIMatrix3& rhs )
  : SAIMatrix( 3, 3, m_buff[0] )
{
  setValues( rhs.dataPtr() );
}

inline SAIMatrix3::SAIMatrix3( const Float* rgVals, bool fTranspose )
  : SAIMatrix( 3, 3, m_buff[0] )
{
  if( rgVals != NULL )
  {
    setValues( rgVals, fTranspose );
  }
}

inline SAIMatrix3::SAIMatrix3( const SAIMatrix& rhs )
  : SAIMatrix( 3, 3, m_buff[0] )
{
  assert( rhs.row() == 3 && rhs.column() == 3 );
  setValues( rhs.dataPtr() );
}

inline SAIMatrix3::SAIMatrix3( Float x1, Float x2, Float x3,
                             Float x4, Float x5, Float x6,
                             Float x7, Float x8, Float x9 )
  : SAIMatrix( 3, 3, m_buff[0] )
{
  m_buff[0][0] = x1;  m_buff[0][1] = x2;  m_buff[0][2] = x3;
  m_buff[1][0] = x4;  m_buff[1][1] = x5;  m_buff[1][2] = x6;
  m_buff[2][0] = x7;  m_buff[2][1] = x8;  m_buff[2][2] = x9;
}

inline SAIVector3 SAIMatrix3::angularError( const SAIMatrix& Rd ) const
{
  SAIVector3 tmp( NULL );
  angularError( Rd, tmp );
  return tmp;
}

inline void SAIMatrix3::setDiagonal( Float d0, Float d1, Float d2 )
{
  m_buff[0][0] = d0;
  m_buff[1][1] = d1;
  m_buff[2][2] = d2;
}

inline void SAIMatrix3::getRow( int row, SAIVector& dest ) const
{
  assert( row >= 0 && row < 3 );
  dest.setSize( 3 );
  dest[0] = m_buff[row][0];
  dest[1] = m_buff[row][1];
  dest[2] = m_buff[row][2];
}

inline void SAIMatrix3::getColumn( int col, SAIVector& dest ) const
{
  assert( col >= 0 && col < 3 );
  dest.setSize( 3 );
  dest[0] = m_buff[0][col];
  dest[1] = m_buff[1][col];
  dest[2] = m_buff[2][col];
}

inline SAIVector3 SAIMatrix3::diagonal() const
{
  return SAIVector3( m_buff[0][0], m_buff[1][1], m_buff[2][2] );
}

inline void SAIMatrix3::diagonal( SAIVector& dest ) const
{
  dest.setSize( 3 );
  dest[0] = m_buff[0][0];
  dest[1] = m_buff[1][1];
  dest[2] = m_buff[2][2];
}

inline void SAIMatrix3::setDiagonal( Float src )
{
  m_buff[0][0] = src;
  m_buff[1][1] = src;
  m_buff[2][2] = src;
}

inline void SAIMatrix3::setDiagonal( const Float* src )
{
  m_buff[0][0] = src[0];
  m_buff[1][1] = src[1];
  m_buff[2][2] = src[2];
}

inline void SAIMatrix3::setDiagonal( const SAIVector& src )
{
  assert( src.size() == 3 );
  m_buff[0][0] = src[0];
  m_buff[1][1] = src[1];
  m_buff[2][2] = src[2];
}

inline SAIMatrix3& SAIMatrix3::operator=( const SAIMatrix& rhs )
{
  assert( rhs.row() == 3 && rhs.column() == 3 );
  setValues( rhs.dataPtr() );
  return *this;
}

inline Float* SAIMatrix3::operator[]( int row )
{
  assert( row >= 0 && row < 3 );
  return m_buff[row];
}

inline const Float* SAIMatrix3::operator[]( int row ) const
{
  assert( row >= 0 && row < 3 );
  return m_buff[row];
}

inline Float& SAIMatrix3::elementAt( int row, int col )
{
  assert( row >= 0 && row < 3 && col >= 0 && col < 3 );
  return m_buff[row][col];
}

inline const Float& SAIMatrix3::elementAt( int row, int col ) const
{
  assert( row >= 0 && row < 3 && col >= 0 && col < 3 );
  return m_buff[row][col];
}

inline SAIMatrix3 SAIMatrix3::operator-() const
{
  SAIMatrix3 tmp( NULL );
  negate( tmp );
  return tmp;
}

inline SAIMatrix3 SAIMatrix3::operator+( const SAIMatrix& rhs ) const
{
  SAIMatrix3 tmp( NULL );
  add( rhs, tmp );
  return tmp;
}

inline SAIMatrix3 SAIMatrix3::operator-( const SAIMatrix& rhs ) const
{
  SAIMatrix3 tmp( NULL );
  subtract( rhs, tmp );
  return tmp;
}

inline SAIMatrix3 SAIMatrix3::operator*( const SAIMatrix3& rhs ) const
{
  SAIMatrix3 tmp( NULL );
  multiply( rhs, tmp );
  return tmp;
}

inline SAIMatrix SAIMatrix3::operator*( const SAIMatrix& rhs ) const
{
  SAIMatrix tmp( NULL, 3, rhs.column() );
  multiply( rhs, tmp );
  return tmp;
}

inline void SAIMatrix3::multiply( const SAIMatrix& rhs, bool fTranspose )
{
  assert( rhs.row() == 3 && rhs.column() == 3 );
  SAIMatrix3 lhs = *this;
  lhs.multiply( rhs, *this, fTranspose );
}

inline SAIVector3 SAIMatrix3::operator*( const SAIVector& rhs ) const
{
  SAIVector3 tmp( NULL );
  multiply( rhs, tmp );
  return tmp;
}

inline SAIMatrix3 SAIMatrix3::operator*( Float rhs ) const
{
  SAIMatrix3 tmp( NULL );
  multiply( rhs, tmp );
  return tmp;
}

inline SAIMatrix3 operator*( Float lhs, const SAIMatrix3& rhs )
{
  SAIMatrix3 tmp( NULL );
  rhs.multiply( lhs, tmp );
  return tmp;
}

inline SAIMatrix3 SAIMatrix3::operator/( Float rhs ) const
{
  SAIMatrix3 tmp( NULL );
  divide( rhs, tmp );
  return tmp;
}

inline SAIMatrix3 SAIMatrix3::transpose() const
{
  SAIMatrix3 tmp( NULL );
  transpose( tmp );
  return tmp;
}

#endif // _SAIMatrix3_h
