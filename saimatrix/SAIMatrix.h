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
// SAIMatrix.h
//
// This class provides an MxN matrix.  By KC Chang, Luis Sentis and
// James Warren
//
// modification history
// --------------------
//
// 01/24/08: Luis Sentis: Removed CLAPACK for compatibility with QNX
// 01/23/06: James Warren: Modified to replace newmat with CLAPACK
// 06/17/04: Dan Merget: Made SAIMatrix3 and SAIMatrix6 into subclasses
// 11/12/97: K.C. Chang: created
// *******************************************************************
#ifndef _SAIMatrix_h
#define _SAIMatrix_h

#include <assert.h>
#include <string.h>
#include <iostream>
#include "SAIGlobalDefn.h"
#include "SAIMathDefn.h"
#include <saimatrix/SAIVector.h>

using namespace std;


// ===================================================================
// SAIMatrix class declaration
// ===================================================================

class SAIMatrix
{
public:
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------
  
  // Ranked constructor. Initializes the matrix with zero values.
  SAIMatrix( int row, int col );

  // Creates empty matrix
  SAIMatrix()
     : m_row( 0 ), m_col( 0 ), m_size( 0 ), m_maxSize( 0 ), m_data( NULL ) {}

  // Copy constructor.
  SAIMatrix( const SAIMatrix& rhs );

  // Load from row-major matrix.  If rgVals is NULL, do not initialize data.
  // If fTranspose is set, rgVals is column-major.
  SAIMatrix( const Float* rgVals, int row, int col, bool fTranspose = false );

protected:
  // Special constructor for non-dynamic subclasses
  SAIMatrix( int row, int col, Float* data );

public:
  // Destructor.
  virtual ~SAIMatrix();

  // -----------------------------------------------------------------
  // Operations that load data into matrix
  // -----------------------------------------------------------------

  // Assignment operator. If dimensions differ, it allocates new dimensions.
  SAIMatrix& operator=( const SAIMatrix& rhs );

  // Copy (row * col) values to/from rgVals.  If fTranspose is set,
  // then rgVals is in column-major form
  void setValues( const Float* rgVals, bool fTranspose = false );
  void setConstantValue( Float value, int iRow = -1, int iCol = -1);
  void getValues( Float* rgVals, bool fTranspose = false ) const;

  void setSubmatrix( Float val, int iRow, int iCol, int cRow, int cCol );

  // Zeroing elements and construct identity matrix.
  void zero();
  void identity();
  
  /** Set this matrix to identity, resizing it first if necessary. */
  void identity(int dimension);
  
  /** Create an identity matrix of desired dimension. */
  static inline SAIMatrix const createIdentity(int dimension);
  
  /** \todo Ask Luis what this one is all about, it was unused in
      wbc/model/Contact and had a verbatim copy in
      HumanoidServoBehaviors... all with pass-by-value wasting some
      allocations, btw. */
  static SAIMatrix spatialTransform(SAIMatrix const & rot, SAIVector const & dist);
  
  //                                 | M |
  // Append a matrix vertically: M = |---|
  //                                 | N |
  void appendVertically( const SAIMatrix& rhs);

  // Append a matrix horizontally: M = | M | N |
  void appendHorizontally( const SAIMatrix& rhs);

  // Set the dimensions and optionally zero the matrix
  void setSize( int row, int col, bool fZero = false );

  // Replace contents of this matrix with those of src, without
  // allocating memory if possible.  Puts src in an undefined state.
  void transfer( SAIMatrix& src );

  // -----------------------------------------------------------------
  // Basic lookup operations
  // -----------------------------------------------------------------

  // Dereference operators
  Float* operator[]( int row );
  const Float* operator[]( int row ) const;

  Float&       operator()(int row, int col)       {return elementAt(row,col);}
  const Float& operator()(int row, int col) const {return elementAt(row,col);}

  Float& elementAt( int row, int col );
  const Float& elementAt( int row, int col ) const;

  // Dimensions.
  int row()    const { return m_row;  }
  int column() const { return m_col;  }
  int size()   const { return m_size; }

  // Check if matrix is finite (i.e. no infinite or NaN values)
  bool fFinite() const;

  // Check if matrix is empty.
  bool fEmpty() const { return (m_size == 0); }
  // Check if matrix is an identity.
  bool fIdentity() const;
  // Check if matrix is a zero matrix.
  bool fZero() const;

  // Diagonal.
  SAIVector diagonal() const;
  void diagonal( SAIVector& dest ) const;
  void setDiagonal( Float src );
  void setDiagonal( const Float* src );
  void setDiagonal( const SAIVector& src );

  // submatrix operations
  void getRow( int row, SAIVector& dest ) const;
  void getColumn( int col, SAIVector& dest ) const;
  void setRow( int row, const SAIVector& src );
  void setColumn( int col, const SAIVector& src );

  SAIMatrix submatrix( int row, int col, int height, int width ) const;
  void submatrix( int row, int col, SAIMatrix &dest ) const;
  void setSubmatrix( int row, int col, const SAIMatrix &src );
  void zeroSubmatrix( int iRow, int iCol, int cRows, int cCols );

  SAIVector subvector( int row, int col, int size,
                      bool fHorizontal = false ) const;
  void subvector( int row, int col, SAIVector &dest,
                  bool fHorizontal = false ) const;
  void setSubvector( int row, int col, const SAIVector &src,
                     bool fHorizontal = false );

  // set this matrix to a subset of itself
  void subset( int rowStart, int colStart, int rowEnd, int colEnd );

  // Access the data as a Float*
  Float*       dataPtr()       { return m_data; }
  const Float* dataPtr() const { return m_data; }

  // -----------------------------------------------------------------
  // Basic arithmetic operations
  // -----------------------------------------------------------------
  
  /** A less braindead implementation of operator==() */
  bool equal(SAIMatrix const & rhs, Float precision) const;
  
  bool operator==( const SAIMatrix& rhs ) const;

  SAIMatrix operator-() const;
  void negate( SAIMatrix& dest ) const;

  SAIMatrix operator+( const SAIMatrix& rhs ) const;
  void add( const SAIMatrix& rhs, SAIMatrix& dest ) const;
  void add( const SAIMatrix& rhs );

  SAIMatrix operator+( Float rhs ) const;
  void add( Float rhs, SAIMatrix& dest ) const;
  void add( Float rhs );

  SAIMatrix operator-( const SAIMatrix& rhs ) const;
  void subtract( const SAIMatrix& rhs, SAIMatrix& dest ) const;
  void subtract( const SAIMatrix& rhs );

  // fTranspose term applies to the rhs (e.g. this * rhs^T or this * rhs)
  SAIMatrix operator*( const SAIMatrix& rhs ) const;
  void multiply( const SAIMatrix& rhs, SAIMatrix& dest,
                                      bool fTranspose = false ) const;
  void multiply( const SAIMatrix& rhs, bool fTranspose = false, SAIVector* pTmpStore=NULL );

  void multiplySubmatrix( int stRow, int stCol, int nRows, int nCols, 
                          const SAIMatrix& rhs, int r_stRow, int r_stCol, int r_nRows, int r_nCols, 
                          SAIMatrix& dest, bool fTranspose ) const;

  SAIVector operator*( const SAIVector& rhs ) const;
  void multiply( const SAIVector& rhs, SAIVector& dest ) const;
  void preMultiply( const SAIMatrix& mult, SAIVector* pTmpStore=NULL );

  SAIMatrix operator*( Float rhs ) const;
  void multiply( Float rhs, SAIMatrix& dest ) const;
  void multiply( Float rhs );

  void elementMultiply( const SAIMatrix& rhs, SAIMatrix& dest);
  SAIMatrix elementMultiply( const SAIMatrix& rhs );

  SAIMatrix operator/( Float rhs ) const;
  void divide( Float rhs, SAIMatrix& dest ) const { multiply( 1/rhs, dest ); }
  void divide( Float rhs )                       { multiply( 1/rhs ); }
  
  // miscellaneous functions
  void applyFunction( double (*fcn)(double), SAIMatrix& dest ) const;
  void applyFunction( double (*fcn)(double) );
  double sum() const;
  double maxAbsValue() const;

  SAIMatrix& operator+=( const SAIMatrix& rhs ) { add(rhs); return *this; }
  SAIMatrix& operator-=( const SAIMatrix& rhs ) { subtract(rhs); return *this; }
  SAIMatrix& operator*=( const SAIMatrix& rhs ) { multiply(rhs); return *this; }
  SAIMatrix& operator*=( Float rhs )           { multiply(rhs); return *this; }
  SAIMatrix& operator/=( Float rhs )           { divide(rhs); return *this; }
  SAIMatrix operator~() const;

  // multiplyTranspose for this^T * rhs
  void multiplyTranspose( const SAIMatrix& rhs, SAIMatrix& dest) const;
  void multiplyTranspose( const SAIVector& rhs, SAIVector& dest) const;

  SAIMatrix transpose() const;
  void transpose( SAIMatrix& dest ) const;

  // -----------------------------------------------------------------
  // Linear Algebra
  // -----------------------------------------------------------------
  
  /**
     Matrix Inverse by Crout's LU decomposition. p275-285, Numerical
     Methods for Engineers by Chapra and Canale.
     
     \note It is OK to pass the matrix to be inverted as the \c dest
     parameter.
  */
  void inverse( SAIMatrix& dest ) const;

  void LUdecomp( SAIMatrix& lu ) const;
  void backSub( const SAIVector& y, SAIVector& x ) const;
  
  // Display values.
  void display( const char* name = NULL ) const;
  void prettyPrint (std::ostream & os, std::string const & title, std::string const & prefix) const;
  std::string prettyString(std::string const & title, std::string const & prefix) const;
  friend ostream& operator<<(ostream& os, const SAIMatrix& m);
  friend istream& operator>>(istream& is, SAIMatrix& m);

protected:
  // Override these methods in non-dynamic subclasses
  virtual bool dynamic() const;
  virtual void resize( int row, int col );
  
  int    m_row;
  int    m_col;
  int    m_size;   // m_row * m_col
  int    m_maxSize;
  Float* m_data;
};

// ===================================================================
class SAIMappedMatrix : public SAIMatrix
// ===================================================================
// SAIMappedMatrix class - maps a portion of an existing matrix, with 
// upper left corner at src[row, 0] and cRows x src.m_col dimensionality.
// Be careful using this class.  The source matrix should stay intact 
// through lifetime of SAIMappedMatrix instance.
{
public:
  SAIMappedMatrix( SAIMatrix& src, int row, int cRows );
  ~SAIMappedMatrix();

protected:
  virtual bool dynamic() const;
  virtual void resize( int row, int col );
};


// ===================================================================
// Inline methods
// ===================================================================

inline SAIMatrix::SAIMatrix( int row, int col, Float* data )
  : m_row( row ), m_col( col ), m_size( row * col ), m_maxSize( row * col ),
    m_data( data )
{}

inline void SAIMatrix::zero()
{
  memset( m_data, 0, m_size * sizeof( Float ) );
}

inline void SAIMatrix::identity()
{
  assert( m_row == m_col );
  zero();
  for( int ii = 0; ii < m_row; ii++ ) {
    elementAt( ii, ii ) = 1;
  }
}

inline SAIMatrix const SAIMatrix::createIdentity(int dimension)
{
  SAIMatrix ii(dimension, dimension);
  ii.identity();
  return ii;
}

inline SAIMatrix SAIMatrix::submatrix( int row, int col,
                                     int height, int width ) const
{
  SAIMatrix tmp( NULL, height, width );
  submatrix( row, col, tmp );
  return tmp;
}

inline SAIVector SAIMatrix::subvector( int row, int col, int size,
                                     bool fHorizontal ) const
{
  SAIVector tmp( NULL, size );
  subvector( row, col, tmp, fHorizontal );
  return tmp;
}

inline Float* SAIMatrix::operator[](int row )
{
  assert( row >= 0 && row < m_row );
  return (m_data + row*m_col );
}

inline const Float* SAIMatrix::operator[](int row ) const
{
  assert( row >= 0 && row < m_row );
  return (m_data + row*m_col );
}

inline Float& SAIMatrix::elementAt( int row, int col )
{
  assert( row >= 0 && row < m_row && col >= 0 && col < m_col );
  return m_data[row * m_col + col];
}

inline const Float& SAIMatrix::elementAt( int row, int col ) const
{
  assert( row >= 0 && row < m_row && col >= 0 && col < m_col );
  return m_data[row * m_col + col];
}

inline SAIVector SAIMatrix::diagonal() const
{
  SAIVector tmp(NULL, m_row);
  diagonal(tmp);
  return tmp;
}

inline SAIMatrix SAIMatrix::operator-() const
{
  SAIMatrix tmp( NULL, m_row, m_col );
  negate( tmp );
  return tmp;
}

inline SAIMatrix SAIMatrix::operator+( const SAIMatrix& rhs ) const
{
  SAIMatrix tmp( NULL, m_row, m_col );
  add( rhs, tmp );
  return tmp;
}

inline SAIMatrix SAIMatrix::operator+( Float rhs ) const
{
  SAIMatrix tmp( NULL, m_row, m_col );
  add( rhs, tmp );
  return tmp;
}

inline SAIMatrix SAIMatrix::operator-( const SAIMatrix& rhs ) const
{
  SAIMatrix tmp( NULL, m_row, m_col );
  subtract( rhs, tmp );
  return tmp;
}

inline SAIMatrix SAIMatrix::operator*( const SAIMatrix& rhs ) const
{
  SAIMatrix tmp( NULL, m_row, rhs.m_col );
  multiply( rhs, tmp );
  return tmp;
}

inline SAIVector SAIMatrix::operator*( const SAIVector& rhs ) const
{
  SAIVector tmp( NULL, m_row );
  multiply( rhs, tmp );
  return tmp;
}

inline SAIMatrix SAIMatrix::operator*( Float rhs ) const
{
  SAIMatrix tmp( NULL, m_row, m_col );
  multiply( rhs, tmp );
  return tmp;
}

inline SAIMatrix operator*( Float lhs, const SAIMatrix& rhs )
{
  SAIMatrix tmp( NULL, rhs.row(), rhs.column() );
  rhs.multiply( lhs, tmp );
  return tmp;
}

inline SAIMatrix SAIMatrix::operator/( Float rhs ) const
{
  SAIMatrix tmp( NULL, m_row, m_col );
  divide( rhs, tmp );
  return tmp;
}

inline SAIMatrix SAIMatrix::transpose() const
{
  SAIMatrix tmp( NULL, m_col, m_row );
  transpose( tmp );
  return tmp;
}

inline SAIMatrix SAIMatrix::operator~() const
{
  SAIMatrix tmp( NULL, m_row, m_col );
  inverse( tmp );
  return tmp;
}

#endif //_SAIMatrix_h
