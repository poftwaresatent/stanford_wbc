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
// SAIVector.cpp
//
// This implements an Nx1 vector.
//
// modification history
// --------------------
//
// 06/16/04: Dan Merget: made SAIVector3 & SAIVector6 into subclasses
// 11/12/97: K.C. Chang: created.
// *******************************************************************



#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include "SAIVector3.h"
#include "SAIMatrix3.h"

#ifdef _WIN32
  #include <float.h>
  #define finite(num) _finite(num)
#endif

#include <sstream>

using namespace std;



// ===================================================================
// Constructors & Destructors
// ===================================================================

SAIVector::SAIVector( int size ) : m_size( size ), m_maxSize( size )
{
  assert( m_size >= 0 );

  if( m_maxSize > 0 )
  {
    m_data = new Float[m_maxSize];
    zero();
  }
  else
  {
    m_data = NULL;
  }
}

SAIVector::SAIVector( const SAIVector& rhs )
   : m_size( rhs.m_size ), m_maxSize( rhs.m_size )
{
  if( m_maxSize > 0 )
  {
    m_data = new Float[m_maxSize];
    for( int ii = 0; ii < m_size; ii++ )
    {
      m_data[ii] = rhs.m_data[ii];
    }
  }
  else
  {
    m_data = NULL;
  }
}

SAIVector::SAIVector( const Float* rgVals, int size )
   : m_size( size ), m_maxSize( size )
{
  assert( m_size >= 0 );
  if( m_maxSize > 0 )
  {
    m_data = new Float[m_maxSize];
    if( rgVals != NULL )
    {
      for( int ii = 0; ii < m_size; ii++ )
      {
        m_data[ii] = rgVals[ii];
      }
    }
  }
  else
  {
    m_data = NULL;
  }
}

SAIVector::~SAIVector()
{
  if( m_data != NULL )
  {
    delete[] m_data;
  }
}

SAIMatrix SAIVector::mat(bool column) const
{
  SAIMatrix tmp;
  if(column) {
    tmp.setSize(m_size, 1);
    tmp.setColumn(0, *this);
  }
  else {
    tmp.setSize(1, m_size);
    tmp.setRow(0, *this);
  }
  return tmp;
}

// ===================================================================
// fFinite(): Return true if all of the elements are finite, or false
// if any element is infinite or NaN.
// ===================================================================
bool SAIVector::fFinite() const
{
  for( int ii = 0; ii < m_size; ii++ )
  {
    if( ! finite( m_data[ii] ) )
    {
      return false;
    }
  }
  return true;
}

// ===================================================================
// Assignment operator.  Resize this vector if necessary
// ===================================================================
SAIVector& SAIVector::operator=( const SAIVector& rhs )
{
  if( this != &rhs )
  {
    resize( rhs.m_size );
    for( int ii = 0; ii < m_size; ii++ )
    {
      m_data[ii] = rhs.m_data[ii];
    }
  }
  return *this;
}

// ===================================================================
// setValues(): Copy Min(cVals, m_size) values from rgVals to this
// vector.
// ===================================================================
void SAIVector::setValues( const Float* rgVals, int cVals )
{
  int cMove = cVals > m_size ? m_size : cVals;
  for( int ii = 0; ii < cMove; ii++ )
  {
    m_data[ii] = rgVals[ii];
  }
}

void SAIVector::setConstantValue( Float value, int size )
{
  if(size >= 0)
  {
    setSize(size);
  }
  for( int ii = 0; ii < m_size; ii++ )
  {
    m_data[ii] = value;
  }
}


// ===================================================================
// getValues(): Copy Min(cVals, m_size) values from this vector to
// rgVals.
// ===================================================================
void SAIVector::getValues( Float* rgVals, int cVals ) const
{
  int cMove = cVals > m_size ? m_size : cVals;
  for( int ii = 0; ii < cMove; ii++ )
  {
    rgVals[ii] = m_data[ii];
  }
}

// ===================================================================
// append(): Append rhs to the end of this vector.  Resize if needed.
// ===================================================================
void SAIVector::append( const SAIVector& rhs )
{
  SAIVector ans( NULL, m_size + rhs.m_size );

  for( int ii = 0; ii < m_size; ii++ )
  {
    ans[ii] = m_data[ii];
  }
  for( int ii = m_size; ii < m_size + rhs.m_size; ii++ )
  {
    ans[ii] = rhs.m_data[ii - m_size];
  }
  transfer( ans );
}

// ===================================================================
// setSize(): Resize the vector and optionally zero it.
// ===================================================================
void SAIVector::setSize( int size, bool fZero )
{
  resize( size );
  if( fZero )
  {
    zero();
  }
}

// ===================================================================
// transfer(): Replace contents of this vector with those of src,
// without allocating memory if possible.  Puts src in an undefined
// state.
//
// If both *this and src are dynamically allocated, then this
// operation efficiently moves the allocated data from src to *this.
// However, if either vector is non-dynamic (e.g. a SAIVector3), then
// the data is simply copied.
// ===================================================================
void SAIVector::transfer( SAIVector& src )
{
  if( dynamic() && src.dynamic() )
  {
    if( m_data != NULL )
    {
      delete[] m_data;
    }
    m_size     = src.m_size;
    m_maxSize  = src.m_maxSize;
    m_data     = src.m_data;
    src.m_size    = 0;
    src.m_maxSize = 0;
    src.m_data    = NULL;
  }
  else
  {
    *this = src;
  }
}

// ===================================================================
// subvector(): Extract a subset of this vector.  The size of "dest"
// tells how many elements to extract.
// ===================================================================
void SAIVector::subvector( int start, SAIVector& dest ) const
{
  assert( start >= 0 && start + dest.m_size <= m_size );
  for ( int ii = 0; ii < dest.m_size; ii++ )
  {
    dest.m_data[ii] = m_data[start + ii];
  }
}

// ===================================================================
// setSubvector(): Copy a smaller vector into a subset of this vector.
// ===================================================================
void SAIVector::setSubvector( int start, const SAIVector& src )
{
  assert( start >= 0 && start + src.m_size <= m_size );
  for ( int ii = 0; ii < src.m_size; ii++ )
  {
    m_data[start + ii] = src.m_data[ii];
  }
}

// ===================================================================
// Equality operator: This operation tests to see if rhs has the same
// size and contents of this vector.  The data must match exactly;
// there is no epsilon tolerance.
// ===================================================================
bool SAIVector::operator==( const SAIVector& rhs ) const
{
  if( this != &rhs ) {
    if( m_size != rhs.m_size )
    {
      return false;
    }
    for( int ii = 0; ii < m_size; ii++ )
    {
      if ( m_data[ii] != rhs.m_data[ii] )
      {
        return false;
      }
    }
  }
  return true;
}

// ===================================================================
// Arithmetic operations
// ===================================================================

void SAIVector::negate( SAIVector& dest ) const
{
  dest.resize( m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = -m_data[ii];
  }
}

void SAIVector::add( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( rhs.m_size == m_size );
  dest.resize( m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = m_data[ii] + rhs.m_data[ii];
  }
}

void SAIVector::add( const SAIVector& rhs )
{
  assert( rhs.m_size == m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    m_data[ii] += rhs.m_data[ii];
  }
}

void SAIVector::subtract( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( rhs.m_size == m_size );
  dest.resize( m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = m_data[ii] - rhs.m_data[ii];
  }
}

void SAIVector::subtract( const SAIVector& rhs )
{
  assert( rhs.m_size == m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    m_data[ii] -= rhs.m_data[ii];
  }
}

SAIVector SAIVector::operator*( const SAIMatrix& rhs ) const
{
  // Treat this as a horizontal matrix, and multiply (this * rhs).
  // Equivalent to (rhs^T * this), since SAIVector doesn't specify
  // horizontal or vertical.
  SAIVector tmp( NULL, rhs.column() );
  rhs.multiplyTranspose( *this, tmp );
  return tmp;
}

void SAIVector::multiply( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( rhs.m_size == m_size );
  dest.resize( m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = m_data[ii] * rhs.m_data[ii];
  }
}

void SAIVector::multiply( const SAIVector& rhs )
{
  assert( rhs.m_size == m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    m_data[ii] *= rhs.m_data[ii];
  }
}

void SAIVector::divide( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( rhs.m_size == m_size );
  dest.resize( m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = m_data[ii] / rhs.m_data[ii];
  }
}

void SAIVector::divide( const SAIVector& rhs )
{
  assert( rhs.m_size == m_size);
  for( int ii = 0; ii < m_size; ii++ )
  {
    m_data[ii] /= rhs.m_data[ii];
  }
}

void SAIVector::multiply( Float rhs, SAIVector& dest ) const
{
  dest.resize( m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    dest.m_data[ii] = m_data[ii] * rhs;
  }
}

void SAIVector::multiply( Float rhs )
{
  for( int ii = 0; ii < m_size; ii++ )
  {
    m_data[ii] *= rhs;
  }
}

void SAIVector::multiplyTransposed( const SAIVector& rhs, SAIMatrix& dest ) const
{
  dest.setSize( m_size, rhs.m_size );
  for( int ii = 0; ii < m_size; ii++ )
  {
    for( int jj = 0; jj < rhs.m_size; jj++ )
    {
      dest[ii][jj] = m_data[ii] * rhs.m_data[jj];
    }
  }
}

SAIMatrix SAIVector::multiplyTransposed( const SAIVector& rhs ) const
{
  SAIMatrix tmp( NULL, m_size, rhs.m_size );
  multiplyTransposed(rhs, tmp);
  return tmp;
}

Float SAIVector::dot( const SAIVector& rhs ) const
{
  assert( rhs.m_size == m_size );
  Float num = 0.0;
  for( int ii = 0; ii < m_size; ii++ )
  {
    num += m_data[ii] * rhs.m_data[ii];
  }
  return num;
}

Float SAIVector::lengthSquared() const
{
  Float num = 0;
  for( int ii = 0; ii < m_size; ii++ )
  {
    num += m_data[ii] * m_data[ii];
  }
  return num;
}

// ===================================================================
// cross(): Calculate the cross product between two 3x1 vectors
// ===================================================================
SAIVector3 SAIVector::cross( const SAIVector& rhs ) const
{
  assert(m_size == 3);
  assert(rhs.m_size == 3);
  return SAIVector3( m_data[1] * rhs.m_data[2] - m_data[2] * rhs.m_data[1],
                    m_data[2] * rhs.m_data[0] - m_data[0] * rhs.m_data[2],
                    m_data[0] * rhs.m_data[1] - m_data[1] * rhs.m_data[0] );
}

void SAIVector::cross( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( m_size == 3 );
  assert( rhs.m_size == 3 );
  assert( &dest != &rhs && &dest != this );
  dest.resize( 3 );
  dest.m_data[0] = m_data[1] * rhs.m_data[2] - m_data[2] * rhs.m_data[1];
  dest.m_data[1] = m_data[2] * rhs.m_data[0] - m_data[0] * rhs.m_data[2];
  dest.m_data[2] = m_data[0] * rhs.m_data[1] - m_data[1] * rhs.m_data[0];
}

// ===================================================================
// cross(): Create a cross-product matrix.  The matrix for vector
// [x y z] is [0 -z y; z 0 -x; y x 0].
// ===================================================================
SAIMatrix3 SAIVector::cross() const
{
  assert( m_size == 3 );
  return SAIMatrix3(       0.0, -m_data[2],  m_data[1],
                    m_data[2],        0.0, -m_data[0],
                   -m_data[1],  m_data[0],        0.0 );
}

void SAIVector::cross( SAIMatrix& dest ) const
{
  assert( m_size == 3 );
  dest.setSize( 3, 3 );
  dest[0][0]=        0.0;  dest[0][1]= -m_data[2];  dest[0][2]=  m_data[1];
  dest[1][0]=  m_data[2];  dest[1][1]=        0.0;  dest[1][2]= -m_data[0];
  dest[2][0]= -m_data[1];  dest[2][1]=  m_data[0];  dest[2][2]=        0.0;
}

double SAIVector::maxValue()
{
  if(m_size == 0) {
    return 0;
  }
  double v = m_data[0];
  for(int i=1; i<m_size; i++) {
    if(v > m_data[i]) v = m_data[i];
  }
  return v;
}

double SAIVector::minValue()
{
  if(m_size == 0) {
    return 0;
  }
  double v = m_data[0];
  for(int i=1; i<m_size; i++) {
    if(v < m_data[i]) v = m_data[i];
  }
  return v;
}

void SAIVector::applyFunction( double (*fcn)(double), SAIVector& dest) const 
{ 
  dest.setSize(m_size);
  for(int i=0; i<m_size; i++) {
    dest.elementAt(i) = (*fcn)(elementAt(i));
  }
}

void SAIVector::applyFunction( double (*fcn)(double) )
{
  for(int i=0; i<m_size; i++) {
    elementAt(i) = (*fcn)(elementAt(i));
  }
}

// ===================================================================
// display(): Display the vector.
// ===================================================================
void SAIVector::display( const char* name ) const
{
  if( name != NULL )
  {
    printf( "%s = (", name );
  }
  else
  {
     printf( "(" );
  }

  for( int ii = 0; ii < m_size; ii++ )
  {
    if( ii > 0 )
    {
      printf( " %.6f", m_data[ii] );
    }
    else
    {
      printf( "%.6f", m_data[ii] );
    }
  }

  if( name != NULL )
  {
    printf( ")\n" );
  }
  else
  {
    printf( ")" );
  }
}


void SAIVector::
prettyPrint (std::ostream & os, std::string const & title, std::string const & prefix) const
{
  streamsize const old_precision(os.precision(5));
  streamsize const old_width (os.width(7));
  
  if ( ! title.empty())
    os << title << "\n";
  if ( ! prefix.empty())
    os << prefix;
  if (m_size <= 0)
    os << " (empty)";
  else
    for (int ii(0); ii < m_size; ++ii)
      os << " " << m_data[ii];
  os << "\n";
  
  os.precision(old_precision);
  os.width(old_width);
}


std::string SAIVector::
prettyString(std::string const & title, std::string const & prefix) const
{
  ostringstream pretty;
  prettyPrint(pretty, title, prefix);
  return pretty.str();
}


// ===================================================================
// iostream operator to display the vector
// ===================================================================
std::ostream& operator<<(std::ostream& os, const SAIVector& v)
{
  for(int i=0; i<v.m_size; i++) { os << v[i] << '\t'; }
  return os;
}

std::istream& operator>>(std::istream& is, SAIVector& v)
{
  // ASSUME SIZE HAS BEEN PRESET
  for(int i=0; i<v.m_size; i++) { is >> v[i]; }
  return is;
}

// ===================================================================
// dynamic(): This virtual function is used to determine whether
// m_data was dynamically allocated.  It returns true for SAIVector,
// and false for SAIVector3 and SAIVector6.
//
// If dynamic() is false, then it is illegal to delete m_data, or to
// transfer it to another vector.
// ===================================================================
bool SAIVector::dynamic() const
{
  return true;
}

// ===================================================================
// resize(): This virtual function resizes the vector without
// initializing the data.  It is used by setSize(), the assignment
// operator, etc.
//
// This function is normally overridden in the subclass.  For example,
// since a SAIVector3 *always* has 3 elements, SAIVector3 overrides this
// function to throw an assertion if newSize != 3.
// ===================================================================
void SAIVector::resize( int newSize )
{
  assert( newSize >= 0 );

  if(  newSize > m_maxSize )
  {
    if( m_data != NULL )
    {
      delete[] m_data;
    }
    m_data = new Float[newSize];
    m_maxSize = newSize;
  }
  m_size = newSize;
}

// ===================================================================
// SAIVectorSlice: SAIVectorSlice is used by subclasses, especially the
// m_linear and m_angular components of a SAIVector6.
// ===================================================================

SAIVectorSlice::~SAIVectorSlice()
{
  m_data = NULL;
  m_size = 0;
}

bool SAIVectorSlice::dynamic() const
{
  return false;
}

void SAIVectorSlice::resize( int newSize )
{
  assert( newSize == m_size );
}


bool SAIVector::
equal(SAIVector const & rhs, Float precision) const
{
  if ( &rhs == this )
    return true;
  if ( m_size != rhs.m_size )
    return false;
  for (int ii(0); ii < m_size; ++ii)
    if (fabs(m_data[ii] - rhs.m_data[ii]) > precision)
      return false;
  return true;
}
