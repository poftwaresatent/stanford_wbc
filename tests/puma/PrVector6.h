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
// PrVector6.h
//
// This subclass of PrVector is optimized for 6x1 vectors.  You can
// pass it into any routine that takes a PrVector.
//
// modification history
//----------------------
//
// 06/16/04: Dan Merget: Turned into a subclass of PrVector
// 04/10/98: K.C. Chang: added zero(), elementAt().
// 11/20/97: K.C. Chang: added inline methods.
// 11/10/97: K.C. Chang: created.
// *******************************************************************
#ifndef _PrVector6_h
#define _PrVector6_h

#include "PrVector.h"
class PrVector3;
class PrMatrix6;

// *******************************************************************
// PrVector6 class declaration
// *******************************************************************

class PrVector6 : public PrVector
{
public:
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------
  PrVector6();   // initialized to [0, 0, 0, 0, 0, 0]
  PrVector6( const PrVector6& rhs );
  explicit PrVector6( const PrVector& rhs );
  explicit PrVector6( const Float* rgVals );
  PrVector6( Float v0, Float v1, Float v2, Float v3, Float v4, Float v5 );
  PrVector6( const PrVector3& v0, const PrVector3& v1 );
  virtual ~PrVector6();

  // -----------------------------------------------------------------
  // Operations specific to PrVector6
  // -----------------------------------------------------------------

  void values( Float v0, Float v1, Float v2, Float v3, Float v4, Float v5 );

  PrVector&       linearPart()        { return m_linearPart;  }
  const PrVector& linearPart()  const { return m_linearPart;  }
  PrVector&       angularPart()       { return m_angularPart; }
  const PrVector& angularPart() const { return m_angularPart; }

  // -----------------------------------------------------------------
  // This functions in this section are identical to functions in the
  // base class, except that they have been optimized for 6x1 vectors.
  //
  // Some of these functions have been changed to return a PrVector6
  // instead of a PrVector, but the arguments are the same.  For
  // example, the assignment operator accepts any 6x1 PrVector on the
  // rhs, not just a PrVector6.  This lets us freely mix PrVector and
  // PrVector6 objects.
  // -----------------------------------------------------------------

  PrVector6&   operator=( const PrVector& rhs );
  void         zero();
  bool         operator==( const PrVector& rhs ) const;
  Float&       operator[]( int ii )       { return elementAt(ii); }
  const Float& operator[]( int ii ) const { return elementAt(ii); }
  Float&       elementAt( int ii );
  const Float& elementAt( int ii ) const;
  PrVector6    operator-() const;
  void         negate( PrVector& dest ) const;
  PrVector6    operator+( const PrVector& rhs ) const;
  void         add( const PrVector& rhs, PrVector& dest ) const;
  void         add( const PrVector& rhs );
  PrVector6    operator-( const PrVector& rhs ) const;
  void         subtract( const PrVector& rhs, PrVector& dest ) const;
  void         subtract( const PrVector& rhs );
  PrVector6    operator*( const PrVector& rhs ) const;
  void         multiply( const PrVector& rhs, PrVector& dest ) const;
  void         multiply( const PrVector& rhs );
  PrVector6    operator*( Float rhs ) const;
  void         multiply( Float rhs, PrVector& dest ) const;
  void         multiply( Float rhs );
  PrVector6    operator/( Float rhs ) const;
  void         divide(Float rhs, PrVector& dest) const {multiply(1/rhs,dest);}
  void         divide(Float rhs)                       { multiply( 1/rhs ); }

  PrVector6& operator+=( const PrVector& rhs ) { add( rhs ); return *this; }
  PrVector6& operator-=( const PrVector& rhs ) { subtract(rhs); return *this; }
  PrVector6& operator*=( Float rhs )           { multiply(rhs); return *this; }
  PrVector6& operator/=( Float rhs )           { divide(rhs); return *this; }

  PrMatrix6  multiplyTransposed( const PrVector6& rhs ) const;
  PrMatrix   multiplyTransposed( const PrVector& rhs ) const;
  void       multiplyTransposed( const PrVector& rhs, PrMatrix& dest ) const;

  Float dot( const PrVector& rhs ) const;
  Float lengthSquared() const { return this->dot(*this); }
  Float magnitude()     const { return sqrt(lengthSquared()); }
  Float length()        const { return magnitude(); }
  Float abs()           const { return magnitude(); }
  const PrVector6& normalize() { *this /= magnitude(); return *this; }

  void display( const char* name = NULL ) const;

private:
  virtual bool dynamic() const;
  virtual void resize( int newSize );
  PrVectorSlice m_linearPart;
  PrVectorSlice m_angularPart;
  Float m_buff[6];
};

// ===================================================================
// Inline Methods
// ===================================================================

inline Float& PrVector6::elementAt( int ii )
{
  SAIAssert( ii >= 0 && ii < 6 );
  return m_buff[ii];
}

inline const Float& PrVector6::elementAt( int ii ) const
{
  SAIAssert( ii >= 0 && ii < 6 );
  return m_buff[ii];
}

inline PrVector6 PrVector6::operator-() const
{
  PrVector6 tmp( NULL );
  negate( tmp );
  return tmp;
}

inline PrVector6 PrVector6::operator+( const PrVector& rhs ) const
{
  PrVector6 tmp( NULL );
  add( rhs, tmp );
  return tmp;
}

inline PrVector6 PrVector6::operator-( const PrVector& rhs ) const
{
  PrVector6 tmp( NULL );
  subtract( rhs, tmp );
  return tmp;
}

inline PrVector6 PrVector6::operator*( const PrVector& rhs ) const
{
  PrVector6 tmp( NULL );
  multiply( rhs, tmp );
  return tmp;
}

inline PrVector6 PrVector6::operator*( Float rhs ) const
{
  PrVector6 tmp( NULL );
  multiply( rhs, tmp );
  return tmp;
}

inline PrVector6 operator*( Float lhs, const PrVector6& rhs )
{
  PrVector6 tmp( NULL );
  rhs.multiply( lhs, tmp );
  return tmp;
}

inline PrVector6 PrVector6::operator/( Float rhs ) const
{
  PrVector6 tmp( NULL );
  divide( rhs, tmp );
  return tmp;
}

#endif // _PrVector6_h
