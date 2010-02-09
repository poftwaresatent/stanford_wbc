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
// PrMatrix3.h
//
// This class provides a 3x3 matrix.
//
// modification history
// --------------------
//
// 06/17/04: Dan Merget: Made into a subclass of PrMatrix
// 04/17/98: K.C. Chang: added inverse.
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************
#ifndef _PrMatrix3_h
#define _PrMatrix3_h

#include "PrVector3.h"
#include "PrMatrix.h"

// ===================================================================
// PrMatrix3 class declaration
// ===================================================================

class PrMatrix3 : public PrMatrix
{
public:
  // -----------------------------------------------------------------
  // A few useful matrices
  // -----------------------------------------------------------------
  static const PrMatrix3 IDENTITY;
  static const PrMatrix3 ZERO;
  
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------
  PrMatrix3(); // initialized to [0]
  PrMatrix3( const PrMatrix3& rhs );
  explicit PrMatrix3( const PrMatrix& rhs );
  explicit PrMatrix3( const Float* rgVals, bool fTranspose = false );
  PrMatrix3( Float x1, Float x2, Float x3,
             Float x4, Float x5, Float x6,
             Float x7, Float x8, Float x9 );
  virtual ~PrMatrix3();

  // -----------------------------------------------------------------
  // Operations specific to PrMatrix3
  // -----------------------------------------------------------------
  void loadFromMatrix33( const Float (&data)[3][3] )  { setValues( data[0] ); }

  // dPhi = R - Rd
  PrVector3 angularError( const PrMatrix& Rd ) const;
  void angularError( const PrMatrix& Rd, PrVector& dPhi ) const;

  void setDiagonal( Float d0, Float d1, Float d2 );

  // Generate a rotation matrix defined by a rotation axis and angle
  void setRotation( const PrVector3& axis, Float angleRadian );

  // -----------------------------------------------------------------
  // This functions in this section are identical to functions in the
  // base class, except that they have been optimized for 3x3
  // matrices.
  //
  // Some of these functions have been changed to return a PrMatrix3
  // instead of a PrMatrix, but the arguments are the same.  For
  // example, the assignment operator accepts any 3x3 PrMatrix on the
  // rhs, not just a PrMatrix.  This lets us freely mix PrMatrix and
  // PrMatrix3 objects.
  // -----------------------------------------------------------------
  PrVector3    diagonal() const;
  void         diagonal( PrVector& dest ) const;
  void         setDiagonal( Float src );
  void         setDiagonal( const Float* src );
  void         setDiagonal( const PrVector& src );
  void         getRow( int row, PrVector& dest ) const;
  void         getColumn( int col, PrVector& dest ) const;
  PrMatrix3&   operator=( const PrMatrix& rhs );
  void         setValues( const Float* rgVals, bool fTranspose = false );
  void         zero();
  void         identity();
  Float*       operator[]( int row );
  const Float* operator[]( int row ) const;
  Float&       operator()(int row, int col)        {return elementAt(row,col);}
  const Float& operator()(int row, int col)  const {return elementAt(row,col);}
  Float&       elementAt( int row, int col );
  const Float& elementAt( int row, int col ) const;
  PrMatrix3    operator-() const;
  void         negate( PrMatrix& dest ) const;
  PrMatrix3    operator+( const PrMatrix& rhs ) const;
  void         add( const PrMatrix& rhs, PrMatrix& dest ) const;
  void         add( const PrMatrix& rhs );
  PrMatrix3    operator-( const PrMatrix& rhs ) const;
  void         subtract( const PrMatrix& rhs, PrMatrix& dest ) const;
  void         subtract( const PrMatrix& rhs ); 
  PrMatrix3    operator*( const PrMatrix3& rhs ) const;
  PrMatrix     operator*( const PrMatrix& rhs ) const;
  void         multiply( const PrMatrix& rhs, PrMatrix& dest,
                                              bool fTranspose = false ) const;
  void         multiply( const PrMatrix& rhs, bool fTranspose = false );
  PrVector3    operator*( const PrVector& rhs ) const;
  void         multiply( const PrVector& rhs, PrVector& dest ) const;
  PrMatrix3    operator*( Float rhs ) const;
  void         multiply( Float rhs, PrMatrix& dest ) const;
  void         multiply( Float rhs );
  PrMatrix3    operator/( Float rhs ) const;
  void         divide(Float rhs, PrMatrix& dest) const {multiply(1/rhs,dest);}
  void         divide(Float rhs)                       { multiply( 1/rhs ); }
  PrMatrix3& operator+=( const PrMatrix& rhs ) { add(rhs); return *this; }
  PrMatrix3& operator-=( const PrMatrix& rhs ) { subtract(rhs); return *this; }
  PrMatrix3& operator*=( const PrMatrix& rhs ) { multiply(rhs); return *this; }
  PrMatrix3& operator*=( Float rhs )           { multiply(rhs); return *this; }
  PrMatrix3& operator/=( Float rhs )           { divide(rhs); return *this; }
  void      multiplyTranspose( const PrMatrix& rhs, PrMatrix& dest) const;
  void      multiplyTranspose( const PrVector& rhs, PrVector& dest) const;
  PrMatrix3 transpose() const;
  void      transpose( PrMatrix& dest ) const;
  void      display( const char* name = NULL ) const;

private:
  virtual bool dynamic() const;
  virtual void resize( int row, int col );
  Float m_buff[3][3];
};

// ===================================================================
// Inline methods
// ===================================================================

inline PrMatrix3::PrMatrix3() : PrMatrix( 3, 3, m_buff[0] )
{
  zero();
}

inline PrMatrix3::PrMatrix3( const PrMatrix3& rhs )
  : PrMatrix( 3, 3, m_buff[0] )
{
  setValues( rhs.dataPtr() );
}

inline PrMatrix3::PrMatrix3( const Float* rgVals, bool fTranspose )
  : PrMatrix( 3, 3, m_buff[0] )
{
  if( rgVals != NULL )
  {
    setValues( rgVals, fTranspose );
  }
}

inline PrMatrix3::PrMatrix3( const PrMatrix& rhs )
  : PrMatrix( 3, 3, m_buff[0] )
{
  SAIAssert( rhs.row() == 3 && rhs.column() == 3 );
  setValues( rhs.dataPtr() );
}

inline PrMatrix3::PrMatrix3( Float x1, Float x2, Float x3,
                             Float x4, Float x5, Float x6,
                             Float x7, Float x8, Float x9 )
  : PrMatrix( 3, 3, m_buff[0] )
{
  m_buff[0][0] = x1;  m_buff[0][1] = x2;  m_buff[0][2] = x3;
  m_buff[1][0] = x4;  m_buff[1][1] = x5;  m_buff[1][2] = x6;
  m_buff[2][0] = x7;  m_buff[2][1] = x8;  m_buff[2][2] = x9;
}

inline PrVector3 PrMatrix3::angularError( const PrMatrix& Rd ) const
{
  PrVector3 tmp( NULL );
  angularError( Rd, tmp );
  return tmp;
}

inline void PrMatrix3::setDiagonal( Float d0, Float d1, Float d2 )
{
  m_buff[0][0] = d0;
  m_buff[1][1] = d1;
  m_buff[2][2] = d2;
}

inline void PrMatrix3::getRow( int row, PrVector& dest ) const
{
  SAIAssert( row >= 0 && row < 3 );
  dest.setSize( 3 );
  dest[0] = m_buff[row][0];
  dest[1] = m_buff[row][1];
  dest[2] = m_buff[row][2];
}

inline void PrMatrix3::getColumn( int col, PrVector& dest ) const
{
  SAIAssert( col >= 0 && col < 3 );
  dest.setSize( 3 );
  dest[0] = m_buff[0][col];
  dest[1] = m_buff[1][col];
  dest[2] = m_buff[2][col];
}

inline PrVector3 PrMatrix3::diagonal() const
{
  return PrVector3( m_buff[0][0], m_buff[1][1], m_buff[2][2] );
}

inline void PrMatrix3::diagonal( PrVector& dest ) const
{
  dest.setSize( 3 );
  dest[0] = m_buff[0][0];
  dest[1] = m_buff[1][1];
  dest[2] = m_buff[2][2];
}

inline void PrMatrix3::setDiagonal( Float src )
{
  m_buff[0][0] = src;
  m_buff[1][1] = src;
  m_buff[2][2] = src;
}

inline void PrMatrix3::setDiagonal( const Float* src )
{
  m_buff[0][0] = src[0];
  m_buff[1][1] = src[1];
  m_buff[2][2] = src[2];
}

inline void PrMatrix3::setDiagonal( const PrVector& src )
{
  SAIAssert( src.size() == 3 );
  m_buff[0][0] = src[0];
  m_buff[1][1] = src[1];
  m_buff[2][2] = src[2];
}

inline PrMatrix3& PrMatrix3::operator=( const PrMatrix& rhs )
{
  SAIAssert( rhs.row() == 3 && rhs.column() == 3 );
  setValues( rhs.dataPtr() );
  return *this;
}

inline Float* PrMatrix3::operator[]( int row )
{
  SAIAssert( row >= 0 && row < 3 );
  return m_buff[row];
}

inline const Float* PrMatrix3::operator[]( int row ) const
{
  SAIAssert( row >= 0 && row < 3 );
  return m_buff[row];
}

inline Float& PrMatrix3::elementAt( int row, int col )
{
  SAIAssert( row >= 0 && row < 3 && col >= 0 && col < 3 );
  return m_buff[row][col];
}

inline const Float& PrMatrix3::elementAt( int row, int col ) const
{
  SAIAssert( row >= 0 && row < 3 && col >= 0 && col < 3 );
  return m_buff[row][col];
}

inline PrMatrix3 PrMatrix3::operator-() const
{
  PrMatrix3 tmp( NULL );
  negate( tmp );
  return tmp;
}

inline PrMatrix3 PrMatrix3::operator+( const PrMatrix& rhs ) const
{
  PrMatrix3 tmp( NULL );
  add( rhs, tmp );
  return tmp;
}

inline PrMatrix3 PrMatrix3::operator-( const PrMatrix& rhs ) const
{
  PrMatrix3 tmp( NULL );
  subtract( rhs, tmp );
  return tmp;
}

inline PrMatrix3 PrMatrix3::operator*( const PrMatrix3& rhs ) const
{
  PrMatrix3 tmp( NULL );
  multiply( rhs, tmp );
  return tmp;
}

inline PrMatrix PrMatrix3::operator*( const PrMatrix& rhs ) const
{
  PrMatrix tmp( NULL, 3, rhs.column() );
  multiply( rhs, tmp );
  return tmp;
}

inline void PrMatrix3::multiply( const PrMatrix& rhs, bool fTranspose )
{
  SAIAssert( rhs.row() == 3 && rhs.column() == 3 );
  PrMatrix3 lhs = *this;
  lhs.multiply( rhs, *this, fTranspose );
}

inline PrVector3 PrMatrix3::operator*( const PrVector& rhs ) const
{
  PrVector3 tmp( NULL );
  multiply( rhs, tmp );
  return tmp;
}

inline PrMatrix3 PrMatrix3::operator*( Float rhs ) const
{
  PrMatrix3 tmp( NULL );
  multiply( rhs, tmp );
  return tmp;
}

inline PrMatrix3 operator*( Float lhs, const PrMatrix3& rhs )
{
  PrMatrix3 tmp( NULL );
  rhs.multiply( lhs, tmp );
  return tmp;
}

inline PrMatrix3 PrMatrix3::operator/( Float rhs ) const
{
  PrMatrix3 tmp( NULL );
  divide( rhs, tmp );
  return tmp;
}

inline PrMatrix3 PrMatrix3::transpose() const
{
  PrMatrix3 tmp( NULL );
  transpose( tmp );
  return tmp;
}

#endif // _PrMatrix3_h
