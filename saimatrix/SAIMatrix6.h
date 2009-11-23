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
// SAIMatrix6.h
//
// This class provides a 6x6 matrix.
//
// modification history
// --------------------
//
// 06/17/04: Dan Merget: Made into a subclass of SAIMatrix
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************
#ifndef _SAIMatrix6_h
#define _SAIMatrix6_h

#include "SAIVector6.h"
#include "SAIMatrix3.h"
#include <saimatrix/SAIMatrix.h>

// ===================================================================
// SAIMatrix6 class declaration
// ===================================================================

class SAIMatrix6 : public SAIMatrix
{
public:
  // -----------------------------------------------------------------
  // A few useful matrices
  // -----------------------------------------------------------------
  static const SAIMatrix6 IDENTITY;
  static const SAIMatrix6 ZERO;
  
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------
  SAIMatrix6(); // initialized to [0]
  SAIMatrix6( const SAIMatrix6& rhs );
  explicit SAIMatrix6( const SAIMatrix& rhs );
  explicit SAIMatrix6( const Float* rgVals, bool fTranspose = false );
  SAIMatrix6( const SAIMatrix3& m11, const SAIMatrix3& m12,
             const SAIMatrix3& m21, const SAIMatrix3& m22 );
  virtual ~SAIMatrix6();

  // -----------------------------------------------------------------
  // Operations specific to SAIMatrix6
  // -----------------------------------------------------------------

  void setDiagonal( Float d0, Float d1, Float d2,
                    Float d3, Float d4, Float d5 );

  // -----------------------------------------------------------------
  // This functions in this section are identical to functions in the
  // base class, except that they have been optimized for 6x6
  // matrices.
  //
  // Some of these functions have been changed to return a SAIMatrix6
  // instead of a SAIMatrix, but the arguments are the same.  For
  // example, the assignment operator accepts any 6x6 SAIMatrix on the
  // rhs, not just a SAIMatrix.  This lets us freely mix SAIMatrix and
  // SAIMatrix6 objects.
  // -----------------------------------------------------------------
  SAIVector6    diagonal() const;
  void         diagonal( SAIVector& dest ) const;
  void         setDiagonal( Float src );
  void         setDiagonal( const Float* src );
  void         setDiagonal( const SAIVector& src );
  void         getRow( int row, SAIVector& dest ) const;
  void         getColumn( int col, SAIVector& dest ) const;
  SAIMatrix     submatrix( int row, int col, int height, int width ) const;
  void         submatrix( int row, int col, SAIMatrix& dest ) const;
  void         setSubmatrix( int row, int col, const SAIMatrix& src );
  SAIMatrix6&   operator=( const SAIMatrix& rhs );
  void         identity();
  Float*       operator[]( int row );
  const Float* operator[]( int row ) const;
  Float&       operator()(int row, int col)        {return elementAt(row,col);}
  const Float& operator()(int row, int col)  const {return elementAt(row,col);}
  Float&       elementAt( int row, int col );
  const Float& elementAt( int row, int col ) const;
  SAIMatrix6    operator-() const;
  void         negate( SAIMatrix& dest ) const;
  SAIMatrix6    operator+( const SAIMatrix& rhs ) const;
  void         add( const SAIMatrix& rhs, SAIMatrix& dest ) const;
  void         add( const SAIMatrix& rhs );
  SAIMatrix6    operator-( const SAIMatrix& rhs ) const;
  void         subtract( const SAIMatrix& rhs, SAIMatrix& dest ) const;
  void         subtract( const SAIMatrix& rhs ); 
  SAIMatrix6    operator*( const SAIMatrix6& rhs ) const;
  SAIMatrix     operator*( const SAIMatrix& rhs ) const;
  void         multiply( const SAIMatrix& rhs, SAIMatrix& dest,
                                              bool fTranspose = false ) const;
  void         multiply( const SAIMatrix& rhs, bool fTranspose = false );
  SAIVector6    operator*( const SAIVector& rhs ) const;
  void         multiply( const SAIVector& rhs, SAIVector& dest ) const;
  SAIMatrix6    operator*( Float rhs ) const;
  void         multiply( Float rhs, SAIMatrix& dest ) const;
  void         multiply( Float rhs );
  SAIMatrix6    operator/( Float rhs ) const;
  void         divide(Float rhs, SAIMatrix& dest) const {multiply(1/rhs,dest);}
  void         divide(Float rhs)                       { multiply( 1/rhs ); }
  SAIMatrix6& operator+=( const SAIMatrix& rhs ) { add(rhs); return *this; }
  SAIMatrix6& operator-=( const SAIMatrix& rhs ) { subtract(rhs); return *this; }
  SAIMatrix6& operator*=( const SAIMatrix& rhs ) { multiply(rhs); return *this; }
  SAIMatrix6& operator*=( Float rhs )           { multiply(rhs); return *this; }
  SAIMatrix6& operator/=( Float rhs )           { divide(rhs); return *this; }
  void      multiplyTranspose( const SAIMatrix& rhs, SAIMatrix& dest) const;
  void      multiplyTranspose( const SAIVector& rhs, SAIVector& dest) const;
  SAIMatrix6 transpose() const;
  void      transpose( SAIMatrix& dest ) const;
  void      display( const char* name = NULL ) const;

private:
  virtual bool dynamic() const;
  virtual void resize( int row, int col );
  Float m_buff[6][6];
};

// ===================================================================
// Inline methods
// ===================================================================

inline SAIMatrix6::SAIMatrix6() : SAIMatrix( 6, 6, m_buff[0] )
{
  zero();
}

inline SAIMatrix6::SAIMatrix6( const SAIMatrix6& rhs )
  : SAIMatrix( 6, 6, m_buff[0] )
{
  setValues( rhs.dataPtr() );
}

inline SAIMatrix6::SAIMatrix6( const Float* rgVals, bool fTranspose )
  : SAIMatrix( 6, 6, m_buff[0] )
{
  if( rgVals != NULL )
  {
    setValues( rgVals, fTranspose );
  }
}

inline SAIMatrix6::SAIMatrix6( const SAIMatrix& rhs )
  : SAIMatrix( 6, 6, m_buff[0] )
{
  assert( rhs.row() == 6 && rhs.column() == 6 );
  setValues( rhs.dataPtr() );
}

inline SAIMatrix6::SAIMatrix6( const SAIMatrix3& m11, const SAIMatrix3& m12,
                             const SAIMatrix3& m21, const SAIMatrix3& m22 )
  : SAIMatrix( 6, 6, m_buff[0] )
{
  setSubmatrix( 0, 0, m11 );
  setSubmatrix( 0, 3, m12 );
  setSubmatrix( 3, 0, m21 );
  setSubmatrix( 3, 3, m22 );
}

inline SAIMatrix SAIMatrix6::submatrix( int row, int col,
                                      int height, int width ) const
{
  SAIMatrix tmp( height, width );
  submatrix( row, col, tmp );
  return tmp;
}

inline SAIMatrix6& SAIMatrix6::operator=( const SAIMatrix& rhs )
{
  assert( rhs.row() == 6 && rhs.column() == 6 );
  setValues( rhs.dataPtr() );
  return *this;
}

inline Float* SAIMatrix6::operator[]( int row )
{
  assert( row >= 0 && row < 6 );
  return m_buff[row];
}

inline const Float* SAIMatrix6::operator[]( int row ) const
{
  assert( row >= 0 && row < 6 );
  return m_buff[row];
}

inline Float& SAIMatrix6::elementAt( int row, int col )
{
  assert( row >= 0 && row < 6 && col >= 0 && col < 6 );
  return m_buff[row][col];
}

inline const Float& SAIMatrix6::elementAt( int row, int col ) const
{
  assert( row >= 0 && row < 6 && col >= 0 && col < 6 );
  return m_buff[row][col];
}

inline SAIMatrix6 SAIMatrix6::operator-() const
{
  SAIMatrix6 tmp;
  negate( tmp );
  return tmp;
}

inline SAIMatrix6 SAIMatrix6::operator+( const SAIMatrix& rhs ) const
{
  SAIMatrix6 tmp;
  add( rhs, tmp );
  return tmp;
}

inline SAIMatrix6 SAIMatrix6::operator-( const SAIMatrix& rhs ) const
{
  SAIMatrix6 tmp;
  subtract( rhs, tmp );
  return tmp;
}

inline SAIMatrix6 SAIMatrix6::operator*( const SAIMatrix6& rhs ) const
{
  SAIMatrix6 tmp;
  multiply( rhs, tmp );
  return tmp;
}

inline SAIMatrix SAIMatrix6::operator*( const SAIMatrix& rhs ) const
{
  SAIMatrix tmp( 6, rhs.column() );
  multiply( rhs, tmp );
  return tmp;
}

inline void SAIMatrix6::multiply( const SAIMatrix& rhs, bool fTranspose )
{
  assert( rhs.row() == 6 && rhs.column() == 6 );
  SAIMatrix6 lhs = *this;
  lhs.multiply( rhs, *this, fTranspose );
}

inline SAIVector6 SAIMatrix6::operator*( const SAIVector& rhs ) const
{
  SAIVector6 tmp;
  multiply( rhs, tmp );
  return tmp;
}

inline SAIMatrix6 SAIMatrix6::operator*( Float rhs ) const
{
  SAIMatrix6 tmp;
  multiply( rhs, tmp );
  return tmp;
}

inline SAIMatrix6 operator*( Float lhs, const SAIMatrix6& rhs )
{
  SAIMatrix6 tmp;
  rhs.multiply( lhs, tmp );
  return tmp;
}

inline SAIMatrix6 SAIMatrix6::operator/( Float rhs ) const
{
  SAIMatrix6 tmp;
  divide( rhs, tmp );
  return tmp;
}

inline SAIMatrix6 SAIMatrix6::transpose() const
{
  SAIMatrix6 tmp;
  transpose( tmp );
  return tmp;
}

#endif // _SAIMatrix6_h
