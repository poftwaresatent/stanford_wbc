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
// SAIVector6.h
//
// This subclass of SAIVector is optimized for 6x1 vectors.  You can
// pass it into any routine that takes a SAIVector.
//
// modification history
//----------------------
//
// 06/16/04: Dan Merget: Turned into a subclass of SAIVector
// 04/10/98: K.C. Chang: added zero(), elementAt().
// 11/20/97: K.C. Chang: added inline methods.
// 11/10/97: K.C. Chang: created.
// *******************************************************************
#ifndef _SAIVector6_h
#define _SAIVector6_h

#include <saimatrix/SAIVector.h>
class SAIVector3;
class SAIMatrix6;

// *******************************************************************
// SAIVector6 class declaration
// *******************************************************************

class SAIVector6 : public SAIVector
{
public:
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------
  SAIVector6();   // initialized to [0, 0, 0, 0, 0, 0]
  SAIVector6( const SAIVector6& rhs );
  explicit SAIVector6( const SAIVector& rhs );
  explicit SAIVector6( const Float* rgVals );
  SAIVector6( Float v0, Float v1, Float v2, Float v3, Float v4, Float v5 );
  SAIVector6( const SAIVector3& v0, const SAIVector3& v1 );
  virtual ~SAIVector6();

  // -----------------------------------------------------------------
  // Operations specific to SAIVector6
  // -----------------------------------------------------------------

  void values( Float v0, Float v1, Float v2, Float v3, Float v4, Float v5 );

  SAIVector&       linearPart()        { return m_linearPart;  }
  const SAIVector& linearPart()  const { return m_linearPart;  }
  SAIVector&       angularPart()       { return m_angularPart; }
  const SAIVector& angularPart() const { return m_angularPart; }

  // -----------------------------------------------------------------
  // This functions in this section are identical to functions in the
  // base class, except that they have been optimized for 6x1 vectors.
  //
  // Some of these functions have been changed to return a SAIVector6
  // instead of a SAIVector, but the arguments are the same.  For
  // example, the assignment operator accepts any 6x1 SAIVector on the
  // rhs, not just a SAIVector6.  This lets us freely mix SAIVector and
  // SAIVector6 objects.
  // -----------------------------------------------------------------

  SAIVector6&   operator=( const SAIVector& rhs );
  void         zero();
  bool         operator==( const SAIVector& rhs ) const;
  Float&       operator[]( int ii )       { return elementAt(ii); }
  const Float& operator[]( int ii ) const { return elementAt(ii); }
  Float&       elementAt( int ii );
  const Float& elementAt( int ii ) const;
  SAIVector6    operator-() const;
  void         negate( SAIVector& dest ) const;
  SAIVector6    operator+( const SAIVector& rhs ) const;
  void         add( const SAIVector& rhs, SAIVector& dest ) const;
  void         add( const SAIVector& rhs );
  SAIVector6    operator-( const SAIVector& rhs ) const;
  void         subtract( const SAIVector& rhs, SAIVector& dest ) const;
  void         subtract( const SAIVector& rhs );
  SAIVector6    operator*( const SAIVector& rhs ) const;
  void         multiply( const SAIVector& rhs, SAIVector& dest ) const;
  void         multiply( const SAIVector& rhs );
  SAIVector6    operator*( Float rhs ) const;
  void         multiply( Float rhs, SAIVector& dest ) const;
  void         multiply( Float rhs );
  SAIVector6    operator/( Float rhs ) const;
  void         divide(Float rhs, SAIVector& dest) const {multiply(1/rhs,dest);}
  void         divide(Float rhs)                       { multiply( 1/rhs ); }

  SAIVector6& operator+=( const SAIVector& rhs ) { add( rhs ); return *this; }
  SAIVector6& operator-=( const SAIVector& rhs ) { subtract(rhs); return *this; }
  SAIVector6& operator*=( Float rhs )           { multiply(rhs); return *this; }
  SAIVector6& operator/=( Float rhs )           { divide(rhs); return *this; }

  SAIMatrix6  multiplyTransposed( const SAIVector6& rhs ) const;
  SAIMatrix   multiplyTransposed( const SAIVector& rhs ) const;
  void       multiplyTransposed( const SAIVector& rhs, SAIMatrix& dest ) const;

  Float dot( const SAIVector& rhs ) const;
  Float lengthSquared() const { return this->dot(*this); }
  Float magnitude()     const { return sqrt(lengthSquared()); }
  Float length()        const { return magnitude(); }
  Float abs()           const { return magnitude(); }
  const SAIVector6& normalize() { *this /= magnitude(); return *this; }

  void display( const char* name = NULL ) const;

private:
  virtual bool dynamic() const;
  virtual void resize( int newSize );
  SAIVectorSlice m_linearPart;
  SAIVectorSlice m_angularPart;
  Float m_buff[6];
};

// ===================================================================
// Inline Methods
// ===================================================================

inline Float& SAIVector6::elementAt( int ii )
{
  assert( ii >= 0 && ii < 6 );
  return m_buff[ii];
}

inline const Float& SAIVector6::elementAt( int ii ) const
{
  assert( ii >= 0 && ii < 6 );
  return m_buff[ii];
}

inline SAIVector6 SAIVector6::operator-() const
{
  SAIVector6 tmp( NULL );
  negate( tmp );
  return tmp;
}

inline SAIVector6 SAIVector6::operator+( const SAIVector& rhs ) const
{
  SAIVector6 tmp( NULL );
  add( rhs, tmp );
  return tmp;
}

inline SAIVector6 SAIVector6::operator-( const SAIVector& rhs ) const
{
  SAIVector6 tmp( NULL );
  subtract( rhs, tmp );
  return tmp;
}

inline SAIVector6 SAIVector6::operator*( const SAIVector& rhs ) const
{
  SAIVector6 tmp( NULL );
  multiply( rhs, tmp );
  return tmp;
}

inline SAIVector6 SAIVector6::operator*( Float rhs ) const
{
  SAIVector6 tmp( NULL );
  multiply( rhs, tmp );
  return tmp;
}

inline SAIVector6 operator*( Float lhs, const SAIVector6& rhs )
{
  SAIVector6 tmp( NULL );
  rhs.multiply( lhs, tmp );
  return tmp;
}

inline SAIVector6 SAIVector6::operator/( Float rhs ) const
{
  SAIVector6 tmp( NULL );
  divide( rhs, tmp );
  return tmp;
}

#endif // _SAIVector6_h
