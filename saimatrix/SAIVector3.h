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
// SAIVector3.h
//
// This subclass of SAIVector is optimized for 3x1 vectors.  You can
// pass it into any routine that takes a SAIVector.
//
// modification history
//----------------------
//
// 06/16/04: Dan Merget: Turned into a subclass of SAIVector
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************
#ifndef _SAIVector3_h
#define _SAIVector3_h

#include <saimatrix/SAIVector.h>
class SAIMatrix3;

// *******************************************************************
// SAIVector3 class declaration
// *******************************************************************

class SAIVector3 : public SAIVector
{
public:
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------
  SAIVector3();   // initialized to [0, 0, 0]
  SAIVector3( const SAIVector3& rhs );
  explicit SAIVector3( const SAIVector& rhs );
  explicit SAIVector3( const Float* rgVals );
  SAIVector3( Float v0, Float v1, Float v2 );
  virtual ~SAIVector3();

  // -----------------------------------------------------------------
  // Operations specific to SAIVector3
  // -----------------------------------------------------------------
  void values( Float v0, Float v1, Float v2 );

  // -----------------------------------------------------------------
  // This functions in this section are identical to functions in the
  // base class, except that they have been optimized for 3x1 vectors.
  //
  // Some of these functions have been changed to return a SAIVector3
  // instead of a SAIVector, but the arguments are the same.  For
  // example, the assignment operator accepts any 3x1 SAIVector on the
  // rhs, not just a SAIVector3.  This lets us freely mix SAIVector and
  // SAIVector3 objects.
  // -----------------------------------------------------------------

  SAIVector3&   operator=( const SAIVector& rhs );
  void         zero();
  bool         operator==( const SAIVector& rhs ) const;
  Float&       operator[]( int ii )       { return elementAt(ii); }
  const Float& operator[]( int ii ) const { return elementAt(ii); }
  Float&       elementAt( int ii );
  const Float& elementAt( int ii ) const;
  SAIVector3    operator-() const;
  void         negate( SAIVector& dest ) const;
  SAIVector3    operator+( const SAIVector& rhs ) const;
  void         add( const SAIVector& rhs, SAIVector& dest ) const;
  void         add( const SAIVector& rhs );
  SAIVector3    operator-( const SAIVector& rhs ) const;
  void         subtract( const SAIVector& rhs, SAIVector& dest ) const;
  void         subtract( const SAIVector& rhs );
  SAIVector3    operator*( const SAIVector& rhs ) const;
  void         multiply( const SAIVector& rhs, SAIVector& dest ) const;
  void         multiply( const SAIVector& rhs );
  SAIVector3    operator*( Float rhs ) const;
  void         multiply( Float rhs, SAIVector& dest ) const;
  void         multiply( Float rhs );
  SAIVector3    operator/( Float rhs ) const;
  void         divide(Float rhs, SAIVector& dest) const {multiply(1/rhs,dest);}
  void         divide(Float rhs)                       { multiply( 1/rhs ); }

  SAIVector3& operator+=( const SAIVector& rhs ) { add( rhs ); return *this; }
  SAIVector3& operator-=( const SAIVector& rhs ) { subtract(rhs); return *this; }
  SAIVector3& operator*=( Float rhs )           { multiply(rhs); return *this; }
  SAIVector3& operator/=( Float rhs )           { divide(rhs); return *this; }

  SAIMatrix3  multiplyTransposed( const SAIVector3& rhs ) const;
  SAIMatrix   multiplyTransposed( const SAIVector& rhs ) const;
  void       multiplyTransposed( const SAIVector& rhs, SAIMatrix& dest ) const;

  Float dot( const SAIVector& rhs ) const;
  Float lengthSquared() const { return this->dot(*this); }
  Float magnitude()     const { return sqrt(lengthSquared()); }
  Float length()        const { return magnitude(); }
  Float abs()           const { return magnitude(); }
  const SAIVector3& normalize() { *this /= magnitude(); return *this; }

  void display( const char* name = NULL ) const;

private:
  virtual bool dynamic() const;
  virtual void resize( int newSize );
  Float m_buff[3];
};

// ===================================================================
// Inline Methods
// ===================================================================

// The SAIVector3 constructors set the base class's m_data pointer to
// point to the internal buffer.  The destructor will set m_data back
// to NULL, so that the base class doesn't try to delete it.
//
inline SAIVector3::SAIVector3() : SAIVector( 3, m_buff )
{
  values( 0.0, 0.0, 0.0 );
}

inline SAIVector3::SAIVector3( const SAIVector3& rhs ) : SAIVector( 3, m_buff )
{
  values( rhs.m_buff[0], rhs.m_buff[1], rhs.m_buff[2] );
}

inline SAIVector3::SAIVector3( const SAIVector& rhs ) : SAIVector( 3, m_buff )
{
  assert( rhs.size() == 3 );
  values( rhs[0], rhs[1], rhs[2] );
}

inline SAIVector3::SAIVector3( const Float* rgVals ) : SAIVector( 3, m_buff )
{
  if( rgVals != NULL )
  {
    values( rgVals[0], rgVals[1], rgVals[2] );
  }
}

inline SAIVector3::SAIVector3(Float v0, Float v1, Float v2) : SAIVector(3, m_buff)
{
  values( v0, v1, v2 );
}

inline void SAIVector3::values( Float v0, Float v1, Float v2 )
{
  m_buff[0] = v0;
  m_buff[1] = v1;
  m_buff[2] = v2;
}

inline SAIVector3& SAIVector3::operator=( const SAIVector& rhs )
{
  assert(rhs.size() == 3);
  if( this != &rhs )  {
    values(rhs[0], rhs[1], rhs[2]);
  }
  return *this;
}

inline void SAIVector3::zero()
{
  values(0, 0, 0);
}

inline bool SAIVector3::operator==( const SAIVector& rhs ) const
{
  return rhs.size() == 3 &&
         m_buff[0] == rhs[0] &&
         m_buff[1] == rhs[1] &&
         m_buff[2] == rhs[2];
}

inline Float& SAIVector3::elementAt( int ii )
{
  assert( ii >= 0 && ii < 3 );
  return m_buff[ii];
}

inline const Float& SAIVector3::elementAt( int ii ) const
{
  assert( ii >= 0 && ii < 3 );
  return m_buff[ii];
}

inline SAIVector3 SAIVector3::operator-() const
{
  return SAIVector3( -m_buff[0], -m_buff[1], -m_buff[2] );
}

inline void SAIVector3::negate( SAIVector& dest ) const
{
  dest.setSize( 3 );
  dest[0] = -m_buff[0];
  dest[1] = -m_buff[1];
  dest[2] = -m_buff[2];
}

inline SAIVector3 SAIVector3::operator+( const SAIVector& rhs ) const
{
  assert(rhs.size() == 3);
  return SAIVector3( m_buff[0] + rhs[0],
                    m_buff[1] + rhs[1],
                    m_buff[2] + rhs[2] );
}

inline void SAIVector3::add( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( rhs.size() == 3 );
  dest.setSize( 3 );
  dest[0] = m_buff[0] + rhs[0];
  dest[1] = m_buff[1] + rhs[1];
  dest[2] = m_buff[2] + rhs[2];
}

inline void SAIVector3::add( const SAIVector& rhs )
{
  assert( rhs.size() == 3 );
  m_buff[0] += rhs[0];
  m_buff[1] += rhs[1];
  m_buff[2] += rhs[2];
}

inline SAIVector3 SAIVector3::operator-( const SAIVector& rhs ) const
{
  assert(rhs.size() == 3);
  return SAIVector3( m_buff[0] - rhs[0],
                    m_buff[1] - rhs[1],
                    m_buff[2] - rhs[2] );
}

inline void SAIVector3::subtract( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( rhs.size() == 3 );
  dest.setSize( 3 );
  dest[0] = m_buff[0] - rhs[0];
  dest[1] = m_buff[1] - rhs[1];
  dest[2] = m_buff[2] - rhs[2];
}

inline void SAIVector3::subtract( const SAIVector& rhs )
{
  assert( rhs.size() == 3 );
  m_buff[0] -= rhs[0];
  m_buff[1] -= rhs[1];
  m_buff[2] -= rhs[2];
}

inline SAIVector3 SAIVector3::operator*( const SAIVector& rhs ) const
{
  assert(rhs.size() == 3);
  return SAIVector3( m_buff[0] * rhs[0],
                    m_buff[1] * rhs[1],
                    m_buff[2] * rhs[2] );
}

inline void SAIVector3::multiply( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( rhs.size() == 3 );
  dest.setSize( 3 );
  dest[0] = m_buff[0] * rhs[0];
  dest[1] = m_buff[1] * rhs[1];
  dest[2] = m_buff[2] * rhs[2];
}

inline void SAIVector3::multiply( const SAIVector& rhs )
{
  assert( rhs.size() == 3 );
  m_buff[0] *= rhs[0];
  m_buff[1] *= rhs[1];
  m_buff[2] *= rhs[2];
}

inline SAIVector3 SAIVector3::operator*( Float rhs ) const
{
  return SAIVector3( m_buff[0] * rhs, m_buff[1] * rhs, m_buff[2] * rhs );
}

inline SAIVector3 operator*( Float lhs, const SAIVector3& rhs )
{
  return SAIVector3( lhs * rhs[0], lhs * rhs[1], lhs * rhs[2] );
}

inline void SAIVector3::multiply( Float rhs, SAIVector& dest ) const
{
  dest.setSize( 3 );
  dest[0] = m_buff[0] * rhs;
  dest[1] = m_buff[1] * rhs;
  dest[2] = m_buff[2] * rhs;
}

inline void SAIVector3::multiply( Float rhs )
{
  m_buff[0] *= rhs;
  m_buff[1] *= rhs;
  m_buff[2] *= rhs;
}

inline SAIVector3 SAIVector3::operator/( Float rhs ) const
{
  Float rhsInv = 1 / rhs;
  return SAIVector3( m_buff[0] * rhsInv,
                    m_buff[1] * rhsInv,
                    m_buff[2] * rhsInv );
}

inline Float SAIVector3::dot( const SAIVector& rhs ) const
{
  assert( rhs.size() == 3 );
  return m_buff[0] * rhs[0] + m_buff[1] * rhs[1] + m_buff[2] * rhs[2];
}

#endif // _SAIVector3_h
