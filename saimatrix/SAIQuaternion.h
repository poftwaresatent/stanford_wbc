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
// SAIQuaternion.h
//
// This class provides a quaternion, specialized for rotation.
//
// A quaternion is similar to a complex number, except that there are
// three separate imaginary numbers i, j, and k.  Multiplication is
// defined by the rule "i*i = j*j = k*k = i*j*k = -1"; it is
// associative but not commutative.  Other rules (such as i*j = k and
// j*i = -k) can de derived from this basic definition.
//
// A quaternion can also be thought of as a combination of a scalar
// and a 3x1 vector (w, v).  In this case, multiplication is defined
// as (w1, v1) * (w2, v2) = ( w1*w2 - v1.v2 , w1*v2 + w2*v1 + v1Xv2 ).
// "v1.v2" and "v1Xv2" represents the dot-product and the cross-
// product of the vectors, respectively.
//
// Unit quaternions are useful as Euler Parameters, which can rotate a
// vector around an axis of rotation r by an angle u.  We create a
// unit quaternion q = (cos(u/2), sin(u/2) * r) and its conjugate q' =
// (cos(u/2), -sin(u/2) * r).  Now, if we take an arbitrary quaternion
// (w,v), the product q * (w,v) * q' rotates v without changing w.  (w
// is usually 0, anyways.)
//
// The four quaternion parameters are often called lambda_0, lambda_1,
// lambda_2, and lambda_3 (pretend that you're looking at Greek
// letters with subscripts), where lambda_0 is the scalar part and
// (lambda_1, lambda_2, lambda_3) is the vector.
//
// Quaternions are more compact than rotation matrices, and easier to
// normalize with minimal error.  If we want to combine several
// rotations, it's faster to multiply two quaternions than two
// rotation matrices.  However, if we want to rotate a vector, it's
// faster to multiply by a rotation matrix than to apply a quaternion.
//
// modification history
// --------------------
//
// 06/17/04: Dan Merget: Added comments
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************

#ifndef _SAIQuaternion_h
#define _SAIQuaternion_h

#include "SAIGlobalDefn.h"
#include "SAIVector3.h"
#include <saimatrix/SAIVector.h>
#include "SAIMatrix3.h"

// ===================================================================
// SAIQuaternion class declaration
// ===================================================================
class SAIQuaternion
{
public:
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------
  SAIQuaternion() : m_scalar( 1.0 ) {} // initialized to [I]
  SAIQuaternion( const SAIQuaternion& rhs );
  SAIQuaternion( Float scalar, const SAIVector& vector );
  SAIQuaternion( Float scalar, Float vector0, Float vector1, Float vector2 );
  SAIQuaternion( const SAIVector& axis, Float angle );
  SAIQuaternion( const SAIVector& vFrom, const SAIVector& vTo );
  SAIQuaternion( const SAIVector& v4 );
  SAIQuaternion( const SAIMatrix& rotationMatrix );
  ~SAIQuaternion() {}

  // -----------------------------------------------------------------
  // Operations that load data into quaternion
  // -----------------------------------------------------------------

  SAIQuaternion& operator=( const SAIQuaternion& rhs );

  void values( Float scalar, Float vector0, Float vector1, Float vector2 );
  void values( const SAIVector& axis, Float angle );
  void values( const SAIVector& vFrom, const SAIVector& vTo );
  void values( const SAIVector& v4 );
  void values( const SAIMatrix& rotationMatrix );

  const SAIQuaternion& identity();

  // -----------------------------------------------------------------
  // Lookup and conversion operations
  // -----------------------------------------------------------------

  const SAIVector3& v() const { return m_vector; }
  Float w() const            { return m_scalar; }

  // convert to axis-angle notation
  void axisAngle( SAIVector& axis, Float& angle ) const;
  void loadFromAxisAngle( SAIVector& axis, Float angle );

  // convert to Euler angles (x-convention)
  void eulerAngles( Float& psi, Float& theta, Float& phi ) const;
  void loadFromEulerAngles( Float psi, Float theta, Float phi );

  // convert to Euler angles (y-convention)
  void eulerAnglesY( Float& psi, Float& theta, Float& phi ) const;
  void loadFromEulerAnglesY( Float psi, Float theta, Float phi );

  // Returns rotation matrix.
  SAIMatrix3 rotationMatrix() const;
  void rotationMatrix( SAIMatrix& dest ) const;
  SAIMatrix3 matrix() const;
  void matrix( SAIMatrix& dest ) const      { rotationMatrix( dest ); }

  // One column of rotation matrix
  SAIVector3 column( int col ) const;
  void column( int col, SAIVector& dest ) const;

  // vector form
  SAIVector vecForm() const;
  SAIMatrix EMatrix() const;

  // -----------------------------------------------------------------
  // Basic arithmetic operations
  // -----------------------------------------------------------------

  int operator==( const SAIQuaternion& rhs );

  SAIQuaternion operator-() const;
  void negate( SAIQuaternion& dest ) const;

  // inversion operator only works for unit quaternions
  SAIQuaternion operator~() const;
  void inverse( SAIQuaternion& dest ) const;

  SAIQuaternion operator+( const SAIQuaternion& rhs ) const;
  void add( const SAIQuaternion& rhs, SAIQuaternion& dest ) const;

  SAIQuaternion operator-( const SAIQuaternion& rhs ) const;
  void subtract( const SAIQuaternion& rhs, SAIQuaternion& dest ) const;

  SAIQuaternion operator*( const SAIQuaternion& rhs ) const;
  void multiply( const SAIQuaternion& rhs, SAIQuaternion& dest ) const;

  SAIVector3 operator*( const SAIVector& rhs ) const;
  void multiply( const SAIVector& rhs, SAIVector& dest ) const;

  SAIQuaternion operator*( Float rhs ) const;
  void multiply( Float rhs, SAIQuaternion& dest ) const;

  SAIQuaternion operator/( Float rhs ) const;
  void divide( Float rhs, SAIQuaternion& dest ) const { multiply(1/rhs, dest); }

  SAIQuaternion& operator+=( const SAIQuaternion& rhs );
  SAIQuaternion& operator-=( const SAIQuaternion& rhs );
  SAIQuaternion& operator*=( const SAIQuaternion& rhs );
  SAIQuaternion& operator*=( Float rhs );
  SAIQuaternion& operator/=( Float rhs );

  Float dot( const SAIQuaternion& rhs ) const;
  void normalize();
  SAIQuaternion& unit() { normalize(); return *this; }

  // -----------------------------------------------------------------
  // Miscellaneous
  // -----------------------------------------------------------------

  // dPhi = E_inv * (this - qd)
  SAIVector3 angularError( const SAIQuaternion& qd ) const;
  void angularError( const SAIQuaternion& qd, SAIVector& dPhi ) const;

  // dq = E * omega
  SAIQuaternion velocity( const SAIVector& omega ) const;
  void velocity( const SAIVector& omega, SAIQuaternion& dq ) const;

  // interpolates btw quaternions to the specified fraction using spherical interpolation
  static SAIQuaternion interpolate( const SAIQuaternion& qA, const SAIQuaternion& qB, double fraction );

  void display( const char* name = NULL ) const;
  friend std::ostream& operator<<(std::ostream& os, const SAIQuaternion& v);
  friend std::istream& operator>>(std::istream& is, SAIQuaternion& v);

private:
  Float m_scalar;
  SAIVector3 m_vector;
};

static SAIQuaternion s_nullSAIQuaternion;

// ===================================================================
// Inline methods
// ===================================================================

inline SAIQuaternion::SAIQuaternion( const SAIQuaternion& rhs )
  : m_scalar( rhs.m_scalar ), m_vector( rhs.m_vector )
{
}

inline SAIQuaternion::SAIQuaternion( Float scalar, const SAIVector& vector )
  : m_scalar( scalar ), m_vector( vector )
{
}

inline SAIQuaternion::SAIQuaternion( Float scalar, Float vector0,
                                   Float vector1, Float vector2 )
{
  values( scalar, vector0, vector1, vector2 );
}

inline SAIQuaternion::SAIQuaternion( const SAIVector& axis, Float angle )
{
  values( axis, angle );
}

inline SAIQuaternion::SAIQuaternion( const SAIVector& vFrom, const SAIVector& vTo )
{
  values( vFrom, vTo );
}

inline SAIQuaternion::SAIQuaternion( const SAIVector& v4 )
{
  values( v4 );
}

inline SAIQuaternion::SAIQuaternion( const SAIMatrix& rotationMatrix )
{
  values( rotationMatrix );
}

inline SAIQuaternion& SAIQuaternion::operator=( const SAIQuaternion& rhs )
{
  if( this != &rhs )
  {
    m_scalar = rhs.m_scalar;
    m_vector = rhs.m_vector;
  }
  return (*this );
}

inline const SAIQuaternion& SAIQuaternion::identity()
{
  m_scalar = 1.0;
  m_vector.zero();
  return *this;
}

inline void SAIQuaternion::values( Float scalar, Float vector0,
                                  Float vector1, Float vector2 )
{
  m_scalar = scalar;
  m_vector.values( vector0, vector1, vector2 ); 
}

inline void SAIQuaternion::values( const SAIVector& axis, Float angle )
{
  assert( axis.size() == 3 );
  m_scalar = cos( 0.5 * angle );
  axis.multiply( sin( 0.5 * angle ), m_vector );
}

inline void SAIQuaternion::values( const SAIVector &v4 )
{
  assert( v4.size() == 4);
  m_scalar = v4[0];
  m_vector.values( v4[1], v4[2], v4[3] );
}

inline SAIMatrix3 SAIQuaternion::rotationMatrix() const
{
  SAIMatrix3 tmp;
  rotationMatrix(tmp);
  return tmp;
}

inline SAIMatrix3 SAIQuaternion::matrix() const
{
  SAIMatrix3 tmp;
  matrix(tmp);
  return tmp;
}

inline SAIVector3 SAIQuaternion::column( int col ) const
{
  SAIVector3 tmp;
  column(col, tmp);
  return tmp;
}

inline SAIQuaternion SAIQuaternion::operator-() const
{
  return SAIQuaternion( -m_scalar, -m_vector[0], -m_vector[1], -m_vector[2] );
}

inline SAIQuaternion SAIQuaternion::operator~() const
{
  return SAIQuaternion( m_scalar, -m_vector[0], -m_vector[1], -m_vector[2] );
}

inline void SAIQuaternion::inverse( SAIQuaternion& dest ) const
{
  dest.m_scalar = m_scalar;
  m_vector.negate(dest.m_vector);
}

inline SAIQuaternion SAIQuaternion::operator+( const SAIQuaternion& rhs ) const
{
  return SAIQuaternion( m_scalar + rhs.m_scalar,
                       m_vector[0] + rhs.m_vector[0],
                       m_vector[1] + rhs.m_vector[1],
                       m_vector[2] + rhs.m_vector[2] );
}

inline void SAIQuaternion::add( const SAIQuaternion& rhs,
                               SAIQuaternion& dest ) const
{
  dest.m_scalar = m_scalar + rhs.m_scalar;
  m_vector.add( rhs.m_vector, dest.m_vector );
}

inline SAIQuaternion SAIQuaternion::operator-( const SAIQuaternion& rhs ) const
{
  return SAIQuaternion( m_scalar - rhs.m_scalar,
                       m_vector[0] - rhs.m_vector[0],
                       m_vector[1] - rhs.m_vector[1],
                       m_vector[2] - rhs.m_vector[2] );
}

inline void SAIQuaternion::subtract( const SAIQuaternion& rhs,
                                    SAIQuaternion& dest ) const
{
  dest.m_scalar = m_scalar - rhs.m_scalar;
  m_vector.subtract( rhs.m_vector, dest.m_vector );
}

inline SAIQuaternion SAIQuaternion::operator*( const SAIQuaternion& rhs ) const
{
  SAIQuaternion tmp;
  multiply( rhs, tmp );
  return tmp;
}

inline SAIVector3 SAIQuaternion::operator*( const SAIVector& rhs ) const
{
  SAIVector3 tmp;
  multiply( rhs, tmp );
  return tmp;
}

inline SAIQuaternion SAIQuaternion::operator*( Float rhs ) const
{
  return SAIQuaternion( m_scalar * rhs,
                       m_vector[0] * rhs,
                       m_vector[1] * rhs,
                       m_vector[2] * rhs );
}

inline void SAIQuaternion::multiply( Float rhs, SAIQuaternion& dest ) const
{
  dest.m_scalar = m_scalar * rhs;
  m_vector.multiply( rhs, dest.m_vector );
}

inline SAIQuaternion SAIQuaternion::operator/( Float rhs ) const
{
  Float rhsInv = 1 / rhs;
  return SAIQuaternion( m_scalar * rhsInv,
                       m_vector[0] * rhsInv,
                       m_vector[1] * rhsInv,
                       m_vector[2] * rhsInv );
}

inline SAIQuaternion& SAIQuaternion::operator+=( const SAIQuaternion& rhs )
{
  m_scalar += rhs.m_scalar;
  m_vector += rhs.m_vector;
  return *this;
}

inline SAIQuaternion& SAIQuaternion::operator-=( const SAIQuaternion& rhs )
{
  m_scalar -= rhs.m_scalar;
  m_vector -= rhs.m_vector;
  return *this;
}

inline SAIQuaternion& SAIQuaternion::operator*=( const SAIQuaternion& rhs )
{
  SAIQuaternion lhs = *this;
  lhs.multiply( rhs, *this );
  return *this;
}

inline SAIQuaternion& SAIQuaternion::operator*=( Float rhs )
{
  m_scalar *= rhs;
  m_vector *= rhs;
  return *this;
}

inline SAIQuaternion& SAIQuaternion::operator/=( Float rhs )
{
  Float rhsInv = 1 / rhs;
  m_scalar *= rhsInv;
  m_vector *= rhsInv;
  return *this;
}

inline Float SAIQuaternion::dot( const SAIQuaternion& rhs ) const
{
  return ( m_scalar * rhs.m_scalar + m_vector.dot( rhs.m_vector ) );
}

inline void SAIQuaternion::normalize()
{
  ( *this ) /= sqrt( dot( *this ) );
}

inline SAIVector3 SAIQuaternion::angularError( const SAIQuaternion& qd ) const
{
  SAIVector3 tmp;
  angularError( qd, tmp );
  return tmp;
}

inline SAIQuaternion SAIQuaternion::velocity( const SAIVector& omega ) const
{
  SAIQuaternion tmp;
  velocity( omega, tmp );
  return tmp;
}

#endif // _SAIQuaternion_h



