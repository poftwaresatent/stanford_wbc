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
// PrMatrix.h
//
// This class provides an MxN matrix.  By KC Chang, Luis Sentis and
// James Warren, lsentis@robotics.stanford.edu
//
// modification history
// --------------------
//
// 06/17/04: Dan Merget: Made PrMatrix3 and PrMatrix6 into subclasses
// 03/??/04: Tine Lefebvre: added eigenvaluedecomposition (eig) and
//           copyToSymmetricMatrix
// 11/12/97: K.C. Chang: created
// *******************************************************************
#ifndef _PrMatrix_h
#define _PrMatrix_h

#include <string.h>
#include "XUtils.h"
#include "PrGlobalDefn.h"
#include "PrMathDefn.h"
#include "PrVector.h"
#ifndef PR_MISSING_NEWMAT10_LIB
#include "extensions/newmatap.h"
#endif // PR_MISSING_NEWMAT10_LIB

// ===================================================================
// PrMatrix class declaration
// ===================================================================

class PrMatrix
{
public:
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------
  
  // Ranked constructor. Initializes the matrix with zero values.
  PrMatrix( int row, int col );

  // Creates empty matrix
  PrMatrix()
     : m_row( 0 ), m_col( 0 ), m_size( 0 ), m_maxSize( 0 ), m_data( NULL ) {}

  // Copy constructor.
  PrMatrix( const PrMatrix& rhs );

  // Load from row-major matrix.  If rgVals is NULL, do not initialize data.
  // If fTranspose is set, rgVals is column-major.
  PrMatrix( const Float* rgVals, int row, int col, bool fTranspose = false );

  // Destructor.
  virtual ~PrMatrix();

  // -----------------------------------------------------------------
  // Operations that load data into matrix
  // -----------------------------------------------------------------

  // Assignment operator. If dimensions differ, it allocates new dimensions.
  PrMatrix& operator=( const PrMatrix& rhs );

  // Copy (row * col) values to/from rgVals.  If fTranspose is set,
  // then rgVals is in column-major form
  void setValues( const Float* rgVals, bool fTranspose = false );
  void getValues( Float* rgVals, bool fTranspose = false ) const;

  void setSubmatrix( Float val, int iRow, int iCol, int cRow, int cCol );

  // Zeroing elements and construct identity matrix.
  void zero();
  void identity();

  //                                 | M |
  // Append a matrix vertically: M = |---|
  //                                 | N |
  void appendVertically( const PrMatrix& rhs);

  // Append a matrix horizontally: M = | M | N |
  void appendHorizontally( const PrMatrix& rhs);

  // Set the dimensions and optionally zero the matrix
  void setSize( int row, int col, bool fZero = false );

  // Replace contents of this matrix with those of src, without
  // allocating memory if possible.  Puts src in an undefined state.
  void transfer( PrMatrix& src );

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
  PrVector diagonal() const;
  void diagonal( PrVector& dest ) const;
  void setDiagonal( Float src );
  void setDiagonal( const Float* src );
  void setDiagonal( const PrVector& src );

  // submatrix operations
  void getRow( int row, PrVector& dest ) const;
  void getColumn( int col, PrVector& dest ) const;
  void setRow( int row, const PrVector& src );
  void setColumn( int col, const PrVector& src );

  PrMatrix submatrix( int row, int col, int height, int width ) const;
  void submatrix( int row, int col, PrMatrix &dest ) const;
  void setSubmatrix( int row, int col, const PrMatrix &src );

  PrVector subvector( int row, int col, int size,
                      bool fHorizontal = false ) const;
  void subvector( int row, int col, PrVector &dest,
                  bool fHorizontal = false ) const;
  void setSubvector( int row, int col, const PrVector &src,
                     bool fHorizontal = false );

  // set this matrix to a subset of itself
  void subset( int rowStart, int colStart, int rowEnd, int colEnd );

  // Access the data as a Float*
  Float*       dataPtr()       { return m_data; }
  const Float* dataPtr() const { return m_data; }

  // -----------------------------------------------------------------
  // Basic arithmetic operations
  // -----------------------------------------------------------------

  bool operator==( const PrMatrix& rhs ) const;

  PrMatrix operator-() const;
  void negate( PrMatrix& dest ) const;

  PrMatrix operator+( const PrMatrix& rhs ) const;
  void add( const PrMatrix& rhs, PrMatrix& dest ) const;
  void add( const PrMatrix& rhs );

  PrMatrix operator-( const PrMatrix& rhs ) const;
  void subtract( const PrMatrix& rhs, PrMatrix& dest ) const;
  void subtract( const PrMatrix& rhs );

  // fTranspose term applies to the rhs (e.g. this * rhs^T or this * rhs)
  PrMatrix operator*( const PrMatrix& rhs ) const;
  void multiply( const PrMatrix& rhs, PrMatrix& dest,
                                      bool fTranspose = false ) const;
  void multiply( const PrMatrix& rhs, bool fTranspose = false, PrVector* pTmpStore=NULL );

  PrVector operator*( const PrVector& rhs ) const;
  void multiply( const PrVector& rhs, PrVector& dest ) const;
  void preMultiply( const PrMatrix& mult, PrVector* pTmpStore=NULL );

  PrMatrix operator*( Float rhs ) const;
  void multiply( Float rhs, PrMatrix& dest ) const;
  void multiply( Float rhs );

  PrMatrix operator/( Float rhs ) const;
  void divide( Float rhs, PrMatrix& dest ) const { multiply( 1/rhs, dest ); }
  void divide( Float rhs )                       { multiply( 1/rhs ); }
  
  PrMatrix& operator+=( const PrMatrix& rhs ) { add(rhs); return *this; }
  PrMatrix& operator-=( const PrMatrix& rhs ) { subtract(rhs); return *this; }
  PrMatrix& operator*=( const PrMatrix& rhs ) { multiply(rhs); return *this; }
  PrMatrix& operator*=( Float rhs )           { multiply(rhs); return *this; }
  PrMatrix& operator/=( Float rhs )           { divide(rhs); return *this; }

  // multiplyTranspose for this^T * rhs
  void multiplyTranspose( const PrMatrix& rhs, PrMatrix& dest) const;
  void multiplyTranspose( const PrVector& rhs, PrVector& dest) const;

  PrMatrix transpose() const;
  void transpose( PrMatrix& dest ) const;

  // -----------------------------------------------------------------
  // Linear Algebra
  // -----------------------------------------------------------------

  // Inverse. Spd applies to symmetric positive matrixes.
  PrMatrix operator~() const;
  void inverse( PrMatrix& dest ) const;
  void inverseSPD( PrMatrix& dest ) const;

  // Linear system solving. y = A x. Spd applies to symmetric
  // positive matrixes.
  void solve( const PrVector& y, PrVector& x ) const;
  void solveSPD( const PrVector& y, PrVector& x ) const;

#ifndef PR_MISSING_NEWMAT10_LIB
  // Full svd.
  void svd( PrMatrix& U, PrMatrix& S, PrMatrix& V ) const;
  // eigen value decomposition
  void eig( PrMatrix& D, PrMatrix& V ) const;

  // Rank. Don't put tol to a higher value, 
  // causes problems with control module
  int rank( float tol = 1e-15 ) const;

  // Pseudo inverse for non-singular and singular matrixes.
  // Don't put tol to a higher value, 
  // causes problems with control module
  PrMatrix pseudoInverse( Float tol = 1e-15 ) const;
  void pseudoInverse( PrMatrix& dest, Float tol = 1e-15 ) const;
  void cappedPseudoInverse( PrMatrix& res ) const;
#endif // PR_MISSING_NEWMAT10_LIB

  // Display values.
  void display( const char* name = NULL ) const;

protected:
  void LUdecomp( PrMatrix& lu ) const;
  //(symmetric and positive definite )
  void LUdecompSPD( PrMatrix& lu ) const;

  // Solve (LU x = y), where this matrix is LU and y is given
  void backSub( const PrVector& y, PrVector& x ) const;
  //(symmetric and positive definite )
  void backSubSPD( const PrVector& y, PrVector& x ) const;

  // Special copy operations
#ifndef PR_MISSING_NEWMAT10_LIB
  void copyToNewmat( Matrix& dest, bool fTranspose = false ) const;
  void copyToNewmat( SymmetricMatrix& dest ) const;
  void copyFromNewmat( Matrix& src, bool fTranspose = false );
  void copyFromNewmat( DiagonalMatrix& src );
#endif // PR_MISSING_NEWMAT10_LIB

  // Special constructor for non-dynamic subclasses
  PrMatrix( int row, int col, Float* data );

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
class PrMappedMatrix : public PrMatrix
// ===================================================================
// PrMappedMatrix class - maps a portion of an existing matrix, with 
// upper left corner at src[row, 0] and cRows x src.m_col dimensionality.
// Be careful using this class.  The source matrix should stay intact 
// through lifetime of PrMappedMatrix instance.
{
public:
  PrMappedMatrix( PrMatrix& src, int row, int cRows );
  ~PrMappedMatrix();

protected:
  virtual bool dynamic() const;
  virtual void resize( int row, int col );
};


static PrMatrix s_nullPrMatrix( 0, 0 );


// ===================================================================
// Inline methods
// ===================================================================

inline PrMatrix::PrMatrix( int row, int col, Float* data )
  : m_row( row ), m_col( col ), m_size( row * col ), m_maxSize( row * col ),
    m_data( data )
{
}

inline void PrMatrix::zero()
{
  memset( m_data, 0, m_size * sizeof( Float ) );
}

inline void PrMatrix::identity()
{
  SAIAssert( m_row == m_col );
  zero();
  for( int ii = 0; ii < m_row; ii++ ) {
    elementAt( ii, ii ) = 1;
  }
}

inline PrMatrix PrMatrix::submatrix( int row, int col,
                                     int height, int width ) const
{
  PrMatrix tmp( NULL, height, width );
  submatrix( row, col, tmp );
  return tmp;
}

inline PrVector PrMatrix::subvector( int row, int col, int size,
                                     bool fHorizontal ) const
{
  PrVector tmp( NULL, size );
  subvector( row, col, tmp, fHorizontal );
  return tmp;
}

inline Float* PrMatrix::operator[](int row )
{
  SAIAssert( row >= 0 && row < m_row );
  return (m_data + row*m_col );
}

inline const Float* PrMatrix::operator[](int row ) const
{
  SAIAssert( row >= 0 && row < m_row );
  return (m_data + row*m_col );
}

inline Float& PrMatrix::elementAt( int row, int col )
{
  SAIAssert( row >= 0 && row < m_row && col >= 0 && col < m_col );
  return m_data[row * m_col + col];
}

inline const Float& PrMatrix::elementAt( int row, int col ) const
{
  SAIAssert( row >= 0 && row < m_row && col >= 0 && col < m_col );
  return m_data[row * m_col + col];
}

inline PrVector PrMatrix::diagonal() const
{
  PrVector tmp(NULL, m_row);
  diagonal(tmp);
  return tmp;
}

inline PrMatrix PrMatrix::operator-() const
{
  PrMatrix tmp( NULL, m_row, m_col );
  negate( tmp );
  return tmp;
}

inline PrMatrix PrMatrix::operator+( const PrMatrix& rhs ) const
{
  PrMatrix tmp( NULL, m_row, m_col );
  add( rhs, tmp );
  return tmp;
}

inline PrMatrix PrMatrix::operator-( const PrMatrix& rhs ) const
{
  PrMatrix tmp( NULL, m_row, m_col );
  subtract( rhs, tmp );
  return tmp;
}

inline PrMatrix PrMatrix::operator*( const PrMatrix& rhs ) const
{
  PrMatrix tmp( NULL, m_row, rhs.m_col );
  multiply( rhs, tmp );
  return tmp;
}

inline PrVector PrMatrix::operator*( const PrVector& rhs ) const
{
  PrVector tmp( NULL, m_row );
  multiply( rhs, tmp );
  return tmp;
}

inline PrMatrix PrMatrix::operator*( Float rhs ) const
{
  PrMatrix tmp( NULL, m_row, m_col );
  multiply( rhs, tmp );
  return tmp;
}

inline PrMatrix operator*( Float lhs, const PrMatrix& rhs )
{
  PrMatrix tmp( NULL, rhs.row(), rhs.column() );
  rhs.multiply( lhs, tmp );
  return tmp;
}

inline PrMatrix PrMatrix::operator/( Float rhs ) const
{
  PrMatrix tmp( NULL, m_row, m_col );
  divide( rhs, tmp );
  return tmp;
}

inline PrMatrix PrMatrix::transpose() const
{
  PrMatrix tmp( NULL, m_col, m_row );
  transpose( tmp );
  return tmp;
}

inline PrMatrix PrMatrix::operator~() const
{
  PrMatrix tmp( NULL, m_row, m_col );
  inverse( tmp );
  return tmp;
}

inline void PrMatrix::solve( const PrVector& y, PrVector& x ) const
{
  SAIAssert( m_row == m_col && m_row == y.size() );
  x.setSize( m_row );
  PrMatrix lu( NULL, m_row, m_col );
  LUdecomp( lu );
  lu.backSub( y, x );
}

inline void PrMatrix::solveSPD( const PrVector& y, PrVector& x ) const
{
  SAIAssert( m_row == m_col && m_row == y.size() );
  x.setSize( m_row );
  PrMatrix lu( NULL, m_row, m_col );
  LUdecompSPD( lu );
  lu.backSubSPD( y, x );
}

#ifndef PR_MISSING_NEWMAT10_LIB
inline PrMatrix PrMatrix::pseudoInverse( Float tol ) const
{
  PrMatrix tmp( NULL, m_row, m_col );
  pseudoInverse( tmp, tol );
  return tmp;
}
#endif // PR_MISSING_NEWMAT10_LIB

#endif 
