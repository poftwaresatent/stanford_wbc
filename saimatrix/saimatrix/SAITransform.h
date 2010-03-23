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
// SAITransform.h
//
// This class provides an affine transform [R, P], where R is a
// rotation and P is a translation.
//
// modification history
// --------------------
//
// 06/21/04: Dan Merget: Added comments, made compatible with new
//                       SAIMatrix6 and SAIVector6 structures
// 11/20/97: K.C. Chang: added inline methods.
// 11/06/97: K.C. Chang: created.
// *******************************************************************
#ifndef _SAITransform_h
#define _SAITransform_h

#include "SAIGlobalDefn.h"
#include "SAIVector3.h"
#include "SAIQuaternion.h"
#include "SAIVector6.h"
#include "SAIMatrix6.h"

// ===================================================================
// SAITransform class declaration
// ===================================================================

class SAITransform
{
public:
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------
  SAITransform() {}  // initialized to [I 0]
  SAITransform( const SAITransform& rhs );
  SAITransform( const SAIQuaternion& rot, const SAIVector& trans );
  SAITransform( const SAIQuaternion& rot ) : m_rot( rot ) {}
  SAITransform( const SAIVector& trans )   : m_trans( trans ) {}
  ~SAITransform() {}

  // -----------------------------------------------------------------
  // Access methods
  // -----------------------------------------------------------------

  void identity();

  SAIQuaternion&       rotation()          { return m_rot; }
  const SAIQuaternion& rotation()    const { return m_rot; }
  SAIVector3&          translation()       { return m_trans; }
  const SAIVector3&    translation() const { return m_trans; }

  // -----------------------------------------------------------------
  // Arithmetic operations
  // -----------------------------------------------------------------

  SAITransform& operator=( const SAITransform& rhs );

  // [r1, p1] * [r2, p2] = [r1*r2, r1*p2 + p1]
  SAITransform operator*( const SAITransform& rhs ) const;
  void multiply( const SAITransform& rhs, SAITransform& dest ) const;

  // [r, p] * v = r*v + p
  SAIVector3 operator*( const SAIVector& v ) const;
  void multiply( const SAIVector& v, SAIVector& dest ) const;

  SAITransform& operator*=( const SAITransform& rhs );

  // ~[r, p] = [~r, -(~r * p)]
  SAITransform operator~() const;
  void inverse( SAITransform& dest ) const;

  // -----------------------------------------------------------------
  // Transformations applied to 6x1 vectors,
  // where the vector consists of the linear & angular velocity,
  // or the linear & angular error.
  // -----------------------------------------------------------------

  // e = T - Td
  SAIVector6 error( const SAITransform& Td );
  void error( const SAITransform& Td, SAIVector& dest );

  SAIVector6 Xform( const SAIVector6& rhs ) const;
  SAIVector6 Xform( const SAIVector& rhs ) const;
  void Xform( const SAIVector6& rhs, SAIVector6& dest ) const;
  void Xform( const SAIVector& rhs, SAIVector& dest ) const;

  SAIVector6 XformT( const SAIVector6& rhs ) const;
  SAIVector6 XformT( const SAIVector& rhs ) const;
  void XformT( const SAIVector6& rhs, SAIVector6& dest ) const;
  void XformT( const SAIVector& rhs, SAIVector& dest ) const;

  // -----------------------------------------------------------------
  // Miscellaneous
  // -----------------------------------------------------------------
  void display( const char* name = NULL ) const;
  friend std::ostream& operator<<(std::ostream& os, const SAITransform& v);

private:
  SAIQuaternion m_rot;   // rotation
  SAIVector3    m_trans; // translation
};

// ===================================================================
// Inline methods
// ===================================================================
inline SAITransform::SAITransform( const SAITransform& rhs )
  : m_rot( rhs.m_rot ), m_trans( rhs.m_trans )
{
}

inline SAITransform::SAITransform( const SAIQuaternion& rot,
                                 const SAIVector& trans )
  : m_rot( rot ), m_trans( trans )
{
}

inline void SAITransform::identity()
{
  m_rot.identity();
  m_trans.zero();
}

inline SAITransform& SAITransform::operator=( const SAITransform& rhs )
{
  if( this != &rhs )
  {
    m_rot   = rhs.m_rot;
    m_trans = rhs.m_trans;
  }
  return (*this );
}

inline SAITransform SAITransform::operator*( const SAITransform& rhs ) const
{
  SAITransform tmp;
  multiply( rhs, tmp );
  return tmp;
}

inline SAIVector3 SAITransform::operator*( const SAIVector& rhs ) const
{
  SAIVector3 tmp;
  multiply( rhs, tmp );
  return tmp;
}

inline SAITransform& SAITransform::operator*=( const SAITransform& rhs )
{
  SAITransform lhs = *this;
  lhs.multiply( rhs, *this );
  return *this;
}

inline SAITransform SAITransform::operator~() const
{
  SAITransform tmp;
  inverse( tmp );
  return tmp;
}

inline SAIVector6 SAITransform::Xform( const SAIVector6& rhs ) const
{
  SAIVector6 tmp;
  Xform( rhs, tmp );
  return tmp;
}

inline SAIVector6 SAITransform::Xform( const SAIVector& rhs ) const
{
  SAIVector6 tmp;
  Xform( rhs, tmp );
  return tmp;
}

inline SAIVector6 SAITransform::XformT( const SAIVector6& rhs ) const
{
  SAIVector6 tmp;
  XformT( rhs, tmp );
  return tmp;
}

inline SAIVector6 SAITransform::XformT( const SAIVector& rhs ) const
{
  SAIVector6 tmp;
  XformT( rhs, tmp );
  return tmp;
}

#endif // _SAITransform_h
