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
// PrMatrix6.h
//
// This class provides a 6x6 matrix.
//
// modification history
// --------------------
//
// 06/17/04: Dan Merget: Made into a subclass of PrMatrix
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************
#ifndef _PrMatrix6_h
#define _PrMatrix6_h

#include "PrVector6.h"
#include "PrMatrix3.h"
#include "PrMatrix.h"

// ===================================================================
// PrMatrix6 class declaration
// ===================================================================

class PrMatrix6 : public PrMatrix
{
public:
  // -----------------------------------------------------------------
  // A few useful matrices
  // -----------------------------------------------------------------
  static const PrMatrix6 IDENTITY;
  static const PrMatrix6 ZERO;
  
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------
  PrMatrix6(); // initialized to [0]
  PrMatrix6( const PrMatrix6& rhs );
  explicit PrMatrix6( const PrMatrix& rhs );
  explicit PrMatrix6( const Float* rgVals, bool fTranspose = false );
  PrMatrix6( const PrMatrix3& m11, const PrMatrix3& m12,
             const PrMatrix3& m21, const PrMatrix3& m22 );
  virtual ~PrMatrix6();

  // -----------------------------------------------------------------
  // Operations specific to PrMatrix6
  // -----------------------------------------------------------------

  void setDiagonal( Float d0, Float d1, Float d2,
                    Float d3, Float d4, Float d5 );

  // -----------------------------------------------------------------
  // This functions in this section are identical to functions in the
  // base class, except that they have been optimized for 6x6
  // matrices.
  //
  // Some of these functions have been changed to return a PrMatrix6
  // instead of a PrMatrix, but the arguments are the same.  For
  // example, the assignment operator accepts any 6x6 PrMatrix on the
  // rhs, not just a PrMatrix.  This lets us freely mix PrMatrix and
  // PrMatrix6 objects.
  // -----------------------------------------------------------------
  PrVector6    diagonal() const;
  void         diagonal( PrVector& dest ) const;
  void         setDiagonal( Float src );
  void         setDiagonal( const Float* src );
  void         setDiagonal( const PrVector& src );
  void         getRow( int row, PrVector& dest ) const;
  void         getColumn( int col, PrVector& dest ) const;
  PrMatrix     submatrix( int row, int col, int height, int width ) const;
  void         submatrix( int row, int col, PrMatrix& dest ) const;
  void         setSubmatrix( int row, int col, const PrMatrix& src );
  PrMatrix6&   operator=( const PrMatrix& rhs );
  void         identity();
  Float*       operator[]( int row );
  const Float* operator[]( int row ) const;
  Float&       operator()(int row, int col)        {return elementAt(row,col);}
  const Float& operator()(int row, int col)  const {return elementAt(row,col);}
  Float&       elementAt( int row, int col );
  const Float& elementAt( int row, int col ) const;
  PrMatrix6    operator-() const;
  void         negate( PrMatrix& dest ) const;
  PrMatrix6    operator+( const PrMatrix& rhs ) const;
  void         add( const PrMatrix& rhs, PrMatrix& dest ) const;
  void         add( const PrMatrix& rhs );
  PrMatrix6    operator-( const PrMatrix& rhs ) const;
  void         subtract( const PrMatrix& rhs, PrMatrix& dest ) const;
  void         subtract( const PrMatrix& rhs ); 
  PrMatrix6    operator*( const PrMatrix6& rhs ) const;
  PrMatrix     operator*( const PrMatrix& rhs ) const;
  void         multiply( const PrMatrix& rhs, PrMatrix& dest,
                                              bool fTranspose = false ) const;
  void         multiply( const PrMatrix& rhs, bool fTranspose = false );
  PrVector6    operator*( const PrVector& rhs ) const;
  void         multiply( const PrVector& rhs, PrVector& dest ) const;
  PrMatrix6    operator*( Float rhs ) const;
  void         multiply( Float rhs, PrMatrix& dest ) const;
  void         multiply( Float rhs );
  PrMatrix6    operator/( Float rhs ) const;
  void         divide(Float rhs, PrMatrix& dest) const {multiply(1/rhs,dest);}
  void         divide(Float rhs)                       { multiply( 1/rhs ); }
  PrMatrix6& operator+=( const PrMatrix& rhs ) { add(rhs); return *this; }
  PrMatrix6& operator-=( const PrMatrix& rhs ) { subtract(rhs); return *this; }
  PrMatrix6& operator*=( const PrMatrix& rhs ) { multiply(rhs); return *this; }
  PrMatrix6& operator*=( Float rhs )           { multiply(rhs); return *this; }
  PrMatrix6& operator/=( Float rhs )           { divide(rhs); return *this; }
  void      multiplyTranspose( const PrMatrix& rhs, PrMatrix& dest) const;
  void      multiplyTranspose( const PrVector& rhs, PrVector& dest) const;
  PrMatrix6 transpose() const;
  void      transpose( PrMatrix& dest ) const;
  void      display( const char* name = NULL ) const;

private:
  virtual bool dynamic() const;
  virtual void resize( int row, int col );
  Float m_buff[6][6];
};

// ===================================================================
// Inline methods
// ===================================================================

inline PrMatrix6::PrMatrix6() : PrMatrix( 6, 6, m_buff[0] )
{
  zero();
}

inline PrMatrix6::PrMatrix6( const PrMatrix6& rhs )
  : PrMatrix( 6, 6, m_buff[0] )
{
  setValues( rhs.dataPtr() );
}

inline PrMatrix6::PrMatrix6( const Float* rgVals, bool fTranspose )
  : PrMatrix( 6, 6, m_buff[0] )
{
  if( rgVals != NULL )
  {
    setValues( rgVals, fTranspose );
  }
}

inline PrMatrix6::PrMatrix6( const PrMatrix& rhs )
  : PrMatrix( 6, 6, m_buff[0] )
{
  SAIAssert( rhs.row() == 6 && rhs.column() == 6 );
  setValues( rhs.dataPtr() );
}

inline PrMatrix6::PrMatrix6( const PrMatrix3& m11, const PrMatrix3& m12,
                             const PrMatrix3& m21, const PrMatrix3& m22 )
  : PrMatrix( 6, 6, m_buff[0] )
{
  setSubmatrix( 0, 0, m11 );
  setSubmatrix( 0, 3, m12 );
  setSubmatrix( 3, 0, m21 );
  setSubmatrix( 3, 3, m22 );
}

inline PrMatrix PrMatrix6::submatrix( int row, int col,
                                      int height, int width ) const
{
  PrMatrix tmp( height, width );
  submatrix( row, col, tmp );
  return tmp;
}

inline PrMatrix6& PrMatrix6::operator=( const PrMatrix& rhs )
{
  SAIAssert( rhs.row() == 6 && rhs.column() == 6 );
  setValues( rhs.dataPtr() );
  return *this;
}

inline Float* PrMatrix6::operator[]( int row )
{
  SAIAssert( row >= 0 && row < 6 );
  return m_buff[row];
}

inline const Float* PrMatrix6::operator[]( int row ) const
{
  SAIAssert( row >= 0 && row < 6 );
  return m_buff[row];
}

inline Float& PrMatrix6::elementAt( int row, int col )
{
  SAIAssert( row >= 0 && row < 6 && col >= 0 && col < 6 );
  return m_buff[row][col];
}

inline const Float& PrMatrix6::elementAt( int row, int col ) const
{
  SAIAssert( row >= 0 && row < 6 && col >= 0 && col < 6 );
  return m_buff[row][col];
}

inline PrMatrix6 PrMatrix6::operator-() const
{
  PrMatrix6 tmp;
  negate( tmp );
  return tmp;
}

inline PrMatrix6 PrMatrix6::operator+( const PrMatrix& rhs ) const
{
  PrMatrix6 tmp;
  add( rhs, tmp );
  return tmp;
}

inline PrMatrix6 PrMatrix6::operator-( const PrMatrix& rhs ) const
{
  PrMatrix6 tmp;
  subtract( rhs, tmp );
  return tmp;
}

inline PrMatrix6 PrMatrix6::operator*( const PrMatrix6& rhs ) const
{
  PrMatrix6 tmp;
  multiply( rhs, tmp );
  return tmp;
}

inline PrMatrix PrMatrix6::operator*( const PrMatrix& rhs ) const
{
  PrMatrix tmp( 6, rhs.column() );
  multiply( rhs, tmp );
  return tmp;
}

inline void PrMatrix6::multiply( const PrMatrix& rhs, bool fTranspose )
{
  SAIAssert( rhs.row() == 6 && rhs.column() == 6 );
  PrMatrix6 lhs = *this;
  lhs.multiply( rhs, *this, fTranspose );
}

inline PrVector6 PrMatrix6::operator*( const PrVector& rhs ) const
{
  PrVector6 tmp;
  multiply( rhs, tmp );
  return tmp;
}

inline PrMatrix6 PrMatrix6::operator*( Float rhs ) const
{
  PrMatrix6 tmp;
  multiply( rhs, tmp );
  return tmp;
}

inline PrMatrix6 operator*( Float lhs, const PrMatrix6& rhs )
{
  PrMatrix6 tmp;
  rhs.multiply( lhs, tmp );
  return tmp;
}

inline PrMatrix6 PrMatrix6::operator/( Float rhs ) const
{
  PrMatrix6 tmp;
  divide( rhs, tmp );
  return tmp;
}

inline PrMatrix6 PrMatrix6::transpose() const
{
  PrMatrix6 tmp;
  transpose( tmp );
  return tmp;
}

#endif // _PrMatrix6_h
