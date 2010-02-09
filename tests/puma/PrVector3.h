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
// PrVector3.h
//
// This subclass of PrVector is optimized for 3x1 vectors.  You can
// pass it into any routine that takes a PrVector.
//
// modification history
//----------------------
//
// 06/16/04: Dan Merget: Turned into a subclass of PrVector
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************
#ifndef _PrVector3_h
#define _PrVector3_h

#include "PrVector.h"
class PrMatrix3;

// *******************************************************************
// PrVector3 class declaration
// *******************************************************************

class PrVector3 : public PrVector
{
public:
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------
  PrVector3();   // initialized to [0, 0, 0]
  PrVector3( const PrVector3& rhs );
  explicit PrVector3( const PrVector& rhs );
  explicit PrVector3( const Float* rgVals );
  PrVector3( Float v0, Float v1, Float v2 );
  virtual ~PrVector3();

  // -----------------------------------------------------------------
  // Operations specific to PrVector3
  // -----------------------------------------------------------------
  void values( Float v0, Float v1, Float v2 );

  // -----------------------------------------------------------------
  // This functions in this section are identical to functions in the
  // base class, except that they have been optimized for 3x1 vectors.
  //
  // Some of these functions have been changed to return a PrVector3
  // instead of a PrVector, but the arguments are the same.  For
  // example, the assignment operator accepts any 3x1 PrVector on the
  // rhs, not just a PrVector3.  This lets us freely mix PrVector and
  // PrVector3 objects.
  // -----------------------------------------------------------------

  PrVector3&   operator=( const PrVector& rhs );
  void         zero();
  bool         operator==( const PrVector& rhs ) const;
  Float&       operator[]( int ii )       { return elementAt(ii); }
  const Float& operator[]( int ii ) const { return elementAt(ii); }
  Float&       elementAt( int ii );
  const Float& elementAt( int ii ) const;
  PrVector3    operator-() const;
  void         negate( PrVector& dest ) const;
  PrVector3    operator+( const PrVector& rhs ) const;
  void         add( const PrVector& rhs, PrVector& dest ) const;
  void         add( const PrVector& rhs );
  PrVector3    operator-( const PrVector& rhs ) const;
  void         subtract( const PrVector& rhs, PrVector& dest ) const;
  void         subtract( const PrVector& rhs );
  PrVector3    operator*( const PrVector& rhs ) const;
  void         multiply( const PrVector& rhs, PrVector& dest ) const;
  void         multiply( const PrVector& rhs );
  PrVector3    operator*( Float rhs ) const;
  void         multiply( Float rhs, PrVector& dest ) const;
  void         multiply( Float rhs );
  PrVector3    operator/( Float rhs ) const;
  void         divide(Float rhs, PrVector& dest) const {multiply(1/rhs,dest);}
  void         divide(Float rhs)                       { multiply( 1/rhs ); }

  PrVector3& operator+=( const PrVector& rhs ) { add( rhs ); return *this; }
  PrVector3& operator-=( const PrVector& rhs ) { subtract(rhs); return *this; }
  PrVector3& operator*=( Float rhs )           { multiply(rhs); return *this; }
  PrVector3& operator/=( Float rhs )           { divide(rhs); return *this; }

  PrMatrix3  multiplyTransposed( const PrVector3& rhs ) const;
  PrMatrix   multiplyTransposed( const PrVector& rhs ) const;
  void       multiplyTransposed( const PrVector& rhs, PrMatrix& dest ) const;

  Float dot( const PrVector& rhs ) const;
  Float lengthSquared() const { return this->dot(*this); }
  Float magnitude()     const { return sqrt(lengthSquared()); }
  Float length()        const { return magnitude(); }
  Float abs()           const { return magnitude(); }
  const PrVector3& normalize() { *this /= magnitude(); return *this; }

  void display( const char* name = NULL ) const;

private:
  virtual bool dynamic() const;
  virtual void resize( int newSize );
  Float m_buff[3];
};

// ===================================================================
// Inline Methods
// ===================================================================

// The PrVector3 constructors set the base class's m_data pointer to
// point to the internal buffer.  The destructor will set m_data back
// to NULL, so that the base class doesn't try to delete it.
//
inline PrVector3::PrVector3() : PrVector( 3, m_buff )
{
  values( 0.0, 0.0, 0.0 );
}

inline PrVector3::PrVector3( const PrVector3& rhs ) : PrVector( 3, m_buff )
{
  values( rhs.m_buff[0], rhs.m_buff[1], rhs.m_buff[2] );
}

inline PrVector3::PrVector3( const PrVector& rhs ) : PrVector( 3, m_buff )
{
  SAIAssert( rhs.size() == 3 );
  values( rhs[0], rhs[1], rhs[2] );
}

inline PrVector3::PrVector3( const Float* rgVals ) : PrVector( 3, m_buff )
{
  if( rgVals != NULL )
  {
    values( rgVals[0], rgVals[1], rgVals[2] );
  }
}

inline PrVector3::PrVector3(Float v0, Float v1, Float v2) : PrVector(3, m_buff)
{
  values( v0, v1, v2 );
}

inline void PrVector3::values( Float v0, Float v1, Float v2 )
{
  m_buff[0] = v0;
  m_buff[1] = v1;
  m_buff[2] = v2;
}

inline PrVector3& PrVector3::operator=( const PrVector& rhs )
{
  SAIAssert(rhs.size() == 3);
  if( this != &rhs )  {
    values(rhs[0], rhs[1], rhs[2]);
  }
  return *this;
}

inline void PrVector3::zero()
{
  values(0, 0, 0);
}

inline bool PrVector3::operator==( const PrVector& rhs ) const
{
  return rhs.size() == 3 &&
         m_buff[0] == rhs[0] &&
         m_buff[1] == rhs[1] &&
         m_buff[2] == rhs[2];
}

inline Float& PrVector3::elementAt( int ii )
{
  SAIAssert( ii >= 0 && ii < 3 );
  return m_buff[ii];
}

inline const Float& PrVector3::elementAt( int ii ) const
{
  SAIAssert( ii >= 0 && ii < 3 );
  return m_buff[ii];
}

inline PrVector3 PrVector3::operator-() const
{
  return PrVector3( -m_buff[0], -m_buff[1], -m_buff[2] );
}

inline void PrVector3::negate( PrVector& dest ) const
{
  dest.setSize( 3 );
  dest[0] = -m_buff[0];
  dest[1] = -m_buff[1];
  dest[2] = -m_buff[2];
}

inline PrVector3 PrVector3::operator+( const PrVector& rhs ) const
{
  SAIAssert(rhs.size() == 3);
  return PrVector3( m_buff[0] + rhs[0],
                    m_buff[1] + rhs[1],
                    m_buff[2] + rhs[2] );
}

inline void PrVector3::add( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( rhs.size() == 3 );
  dest.setSize( 3 );
  dest[0] = m_buff[0] + rhs[0];
  dest[1] = m_buff[1] + rhs[1];
  dest[2] = m_buff[2] + rhs[2];
}

inline void PrVector3::add( const PrVector& rhs )
{
  SAIAssert( rhs.size() == 3 );
  m_buff[0] += rhs[0];
  m_buff[1] += rhs[1];
  m_buff[2] += rhs[2];
}

inline PrVector3 PrVector3::operator-( const PrVector& rhs ) const
{
  SAIAssert(rhs.size() == 3);
  return PrVector3( m_buff[0] - rhs[0],
                    m_buff[1] - rhs[1],
                    m_buff[2] - rhs[2] );
}

inline void PrVector3::subtract( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( rhs.size() == 3 );
  dest.setSize( 3 );
  dest[0] = m_buff[0] - rhs[0];
  dest[1] = m_buff[1] - rhs[1];
  dest[2] = m_buff[2] - rhs[2];
}

inline void PrVector3::subtract( const PrVector& rhs )
{
  SAIAssert( rhs.size() == 3 );
  m_buff[0] -= rhs[0];
  m_buff[1] -= rhs[1];
  m_buff[2] -= rhs[2];
}

inline PrVector3 PrVector3::operator*( const PrVector& rhs ) const
{
  SAIAssert(rhs.size() == 3);
  return PrVector3( m_buff[0] * rhs[0],
                    m_buff[1] * rhs[1],
                    m_buff[2] * rhs[2] );
}

inline void PrVector3::multiply( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( rhs.size() == 3 );
  dest.setSize( 3 );
  dest[0] = m_buff[0] * rhs[0];
  dest[1] = m_buff[1] * rhs[1];
  dest[2] = m_buff[2] * rhs[2];
}

inline void PrVector3::multiply( const PrVector& rhs )
{
  SAIAssert( rhs.size() == 3 );
  m_buff[0] *= rhs[0];
  m_buff[1] *= rhs[1];
  m_buff[2] *= rhs[2];
}

inline PrVector3 PrVector3::operator*( Float rhs ) const
{
  return PrVector3( m_buff[0] * rhs, m_buff[1] * rhs, m_buff[2] * rhs );
}

inline PrVector3 operator*( Float lhs, const PrVector3& rhs )
{
  return PrVector3( lhs * rhs[0], lhs * rhs[1], lhs * rhs[2] );
}

inline void PrVector3::multiply( Float rhs, PrVector& dest ) const
{
  dest.setSize( 3 );
  dest[0] = m_buff[0] * rhs;
  dest[1] = m_buff[1] * rhs;
  dest[2] = m_buff[2] * rhs;
}

inline void PrVector3::multiply( Float rhs )
{
  m_buff[0] *= rhs;
  m_buff[1] *= rhs;
  m_buff[2] *= rhs;
}

inline PrVector3 PrVector3::operator/( Float rhs ) const
{
  Float rhsInv = 1 / rhs;
  return PrVector3( m_buff[0] * rhsInv,
                    m_buff[1] * rhsInv,
                    m_buff[2] * rhsInv );
}

inline Float PrVector3::dot( const PrVector& rhs ) const
{
  SAIAssert( rhs.size() == 3 );
  return m_buff[0] * rhs[0] + m_buff[1] * rhs[1] + m_buff[2] * rhs[2];
}

#endif // _PrVector3_h
