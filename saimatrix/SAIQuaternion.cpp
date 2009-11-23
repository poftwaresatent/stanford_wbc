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
// SAIQuaternion.cpp
//
// This implements a quaternion class.  See SAIQuaternion.h for
// details.
//
// modification history
// --------------------
//
// 06/17/04: Dan Merget: Added comments
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************


#include "SAIQuaternion.h"

#define SAI_QUATERNION_EPSILON 0.00001
#define SAI_QUATERNION_COS_THRESHHOLD (1.0 - SAI_QUATERNION_EPSILON )

// ===================================================================
// values(): Create a rotation quaternion that will rotate unit vector
// vFrom to +vTo or -vTo.
// ===================================================================
void SAIQuaternion::values( const SAIVector& vFrom, const SAIVector& vTo )
{
  assert( vFrom.size() == 3 );
  assert( vTo.size() == 3 );

  Float fCosTheta = vFrom.dot( vTo );

  if( fCosTheta > SAI_QUATERNION_COS_THRESHHOLD ) 
  {
    // the vectors are the same
    identity();
  }
  else if( fCosTheta < -SAI_QUATERNION_COS_THRESHHOLD ) 
  {
    // the vectors are opposing
    identity();
  }
  else
  {
    SAIVector3 axis;
    vFrom.cross( vTo, axis );
    axis.normalize();
    values(axis, acos( fCosTheta ) );
  }
}

// ============================================================================
// values(): Loads quarternion from specified rotation matrix
// (direction cosines).  Converts up to the sign.
// ============================================================================
void SAIQuaternion::values( const SAIMatrix& m )
{
  assert( m.row() == 3 && m.column() == 3 );

  double e1 = 1 + m[0][0] - m[1][1] - m[2][2];
  double e2 = 1 - m[0][0] + m[1][1] - m[2][2];
  double e3 = 1 - m[0][0] - m[1][1] + m[2][2];
  double e4 = 1 + m[0][0] + m[1][1] + m[2][2];

  // divide by the greater number; one element is for sure greater than .5
  if( e4 >= 1.0 ) 
  {
    e4          = sqrt( e4 );
    m_scalar    = .5 * e4;
    m_vector[0] = (m[2][1] - m[1][2]) / (2*e4);  
    m_vector[1] = (m[0][2] - m[2][0]) / (2*e4);
    m_vector[2] = (m[1][0] - m[0][1]) / (2*e4);
  } 
  else if( e1 >= 1.0 ) 
  {
    e1          = sqrt( e1 );
    m_scalar    = (m[2][1] - m[1][2]) / (2*e1);
    m_vector[0] = .5 * e1;
    m_vector[1] = (m[1][0] + m[0][1]) / (2*e1);
    m_vector[2] = (m[2][0] + m[0][2]) / (2*e1);
  } 
  else if( e2 >= 1.0 ) 
  {
    e2          = sqrt( e2 );
    m_scalar    = (m[0][2] - m[2][0]) / (2*e2);
    m_vector[0] = (m[0][1] + m[1][0]) / (2*e2);
    m_vector[1] = .5 * e2;
    m_vector[2] = (m[1][2] + m[2][1]) / (2*e2);
  } 
  else if( e3 >= 1.0 ) 
  {
    e3          = sqrt( e3 );
    m_scalar    = (m[1][0] - m[0][1]) / (2*e3);
    m_vector[0] = (m[2][0] + m[0][2]) / (2*e3);
    m_vector[1] = (m[2][1] + m[1][2]) / (2*e3);
    m_vector[2] = .5 * e3;
  }
  else
  {
    //assert( false, "Renormalization of quarternion needed." );
    assert( false );
  }
}

// ===================================================================
// axisAngle():  Convert to axis-angle notation.
// ===================================================================
void SAIQuaternion::axisAngle( SAIVector& axis, Float& angle ) const
{
  assert( axis.size() == 3 );

  if( m_scalar > SAI_QUATERNION_COS_THRESHHOLD ||
      m_scalar < -SAI_QUATERNION_COS_THRESHHOLD )
  {
    // no rotation
    angle   = 0;
    axis[0] = 1;
    axis[1] = 0;
    axis[2] = 0;
  }
  else
  {
    angle = 2 * acos( m_scalar );
    Float magnitude = m_vector.magnitude();
    axis = m_vector / magnitude;
  }
}

// ===================================================================
// loadFromAxisAngle():  Load the quaternion from axis angle
// representation.
// ===================================================================
void SAIQuaternion::loadFromAxisAngle( SAIVector& axis, Float angle )
{
  assert( axis.size() == 3 );

  m_scalar = cos(angle / 2);
  m_vector = axis.normalize();
  m_vector.multiply( sin(angle / 2) );
}

// ===================================================================
// eulerAngles(): Convert to Euler angles, using the usual
// x-convention (rotate around z-axis, then x-axis, then z-axis).
// ===================================================================
void SAIQuaternion::eulerAngles( Float& psi, Float& theta, Float& phi ) const
{
   SAIMatrix3 rot;  // Rotation matrix
   rotationMatrix( rot );

   Float cTheta = rot[2][2];
   Float sTheta = sqrt( rot[0][2] * rot[0][2] + rot[1][2] * rot[1][2] );
   if( rot[1][2] > 0 )
   {
      sTheta = -sTheta;   // Choose euler angles such that cos(psi) >= 0
   }

   if( sTheta > SAI_QUATERNION_EPSILON )
   {
      // theta > 0
      theta = atan2( sTheta, cTheta );
      psi = atan2( rot[0][2], -rot[1][2] );
      phi = atan2( rot[2][0],  rot[2][1] );
   }
   else if( sTheta < -SAI_QUATERNION_EPSILON )
   {
      // theta < 0
      theta = atan2( sTheta, cTheta );
      psi = atan2( -rot[0][2],  rot[1][2] );
      phi = atan2( -rot[2][0], -rot[2][1] );
   }
   else if( cTheta > 0 )
   {
      // theta = 0, which means that we cannot find psi and phi, but
      // only psi+phi.  Split the difference between them.
      theta = 0;
      psi = atan2( rot[1][0], rot[0][0] ) / 2;
      phi = psi;
   }
   else
   {
      // theta = pi, which means that we cannot find psi and phi, but
      // only psi-phi.  Split the difference between them.
      theta = M_PI;
      psi = atan2( rot[1][0], rot[0][0] ) / 2;
      phi = -psi;
   }
}

// ===================================================================
// loadFromEulerAngles(): Load the quaternion from Euler angles,
// using the x-convention.
// ===================================================================
void SAIQuaternion::loadFromEulerAngles( Float psi, Float theta, Float phi )
{
   SAIQuaternion psiRotation(   cos( psi/2 ),   0, 0,   sin( psi/2 ) );
   SAIQuaternion thetaRotation( cos( theta/2 ), sin( theta/2 ), 0, 0 );
   SAIQuaternion phiRotation(   cos( phi/2 ),   0, 0,   sin( phi/2 ) );

   *this = psiRotation;
   *this *= thetaRotation;
   *this *= phiRotation;
}

// ===================================================================
// eulerAnglesY(): Convert to Euler angles, using the y-convention
// (rotate around z-axis, then x-axis, then z-axis).
// ===================================================================
void SAIQuaternion::eulerAnglesY( Float& psi, Float& theta, Float& phi ) const
{
   SAIMatrix3 rot;  // Rotation matrix
   rotationMatrix( rot );

   Float cTheta = rot[2][2];
   Float sTheta = sqrt( rot[0][2] * rot[0][2] + rot[1][2] * rot[1][2] );
   if( rot[0][2] < 0 )
   {
      sTheta = -sTheta;   // Choose euler angles such that cos(psi) >= 0
   }

   if( sTheta > SAI_QUATERNION_EPSILON )
   {
      // theta > 0
      theta = atan2( sTheta, cTheta );
      psi = atan2( rot[1][2],  rot[0][2] );
      phi = atan2( rot[2][1], -rot[2][0] );
   }
   else if( sTheta < -SAI_QUATERNION_EPSILON )
   {
      // theta < 0
      theta = atan2( sTheta, cTheta );
      psi = atan2( -rot[1][2], -rot[0][2] );
      phi = atan2( -rot[2][1],  rot[2][0] );
   }
   else if( cTheta > 0 )
   {
      // theta = 0, which means that we cannot find psi and phi, but
      // only psi+phi.  Split the difference between them.
      theta = 0;
      psi = atan2( rot[1][0], rot[0][0] ) / 2;
      phi = psi;
   }
   else
   {
      // theta = pi, which means that we cannot find psi and phi, but
      // only psi-phi.  Split the difference between them.
      theta = M_PI;
      psi = atan2( -rot[1][0], -rot[0][0] ) / 2;
      phi = -psi;
   }
}

// ===================================================================
// loadFromEulerAnglesY(): Load the quaternion from Euler angles,
// using the y-convention.
// ===================================================================
void SAIQuaternion::loadFromEulerAnglesY( Float psi, Float theta, Float phi )
{
   SAIQuaternion psiRotation(   cos( psi/2 ),   0, 0,   sin( psi/2 ) );
   SAIQuaternion thetaRotation( cos( theta/2 ), 0, sin( theta/2 ), 0 );
   SAIQuaternion phiRotation(   cos( phi/2 ),   0, 0,   sin( phi/2 ) );

   *this = psiRotation;
   *this *= thetaRotation;
   *this *= phiRotation;
}

// ===================================================================
// rotationMatrix(): Convert unit quaternion to a 3x3 rotation matrix.
// Requires 27 mult + 12 add operations.
// ===================================================================
void SAIQuaternion::rotationMatrix( SAIMatrix& dest ) const
{
  dest.setSize( 3, 3 );

  // Column 0
  dest[0][0]=1.0 - 2*( m_vector[1] * m_vector[1] + m_vector[2] * m_vector[2] );
  dest[1][0]=2*( m_vector[0] * m_vector[1] + m_scalar*m_vector[2] );
  dest[2][0]=2*( m_vector[0] * m_vector[2] - m_scalar*m_vector[1] );

  // Column 1
  dest[0][1]=2*( m_vector[0] * m_vector[1] - m_scalar*m_vector[2] );
  dest[1][1]=1.0 - 2*( m_vector[0] * m_vector[0] + m_vector[2] * m_vector[2] );
  dest[2][1]=2*( m_vector[1] * m_vector[2] + m_scalar*m_vector[0] );

  // Column 2
  dest[0][2]=2*( m_vector[0] * m_vector[2] + m_scalar*m_vector[1] );
  dest[1][2]=2*( m_vector[1] * m_vector[2] - m_scalar*m_vector[0] );
  dest[2][2]=1.0 - 2*( m_vector[0] * m_vector[0] + m_vector[1] * m_vector[1] );
}

// ===================================================================
// column(): Similar to rotationMatrix(), but only returns one column.
// ===================================================================
void SAIQuaternion::column( int col, SAIVector& dest ) const
{
  assert( col >= 0 && col < 3 );
  dest.setSize( 3 );

  if( col == 0 )
  {
    dest[0]=1.0 - 2*( m_vector[1] * m_vector[1] + m_vector[2] * m_vector[2] );
    dest[1]=2*( m_vector[0] * m_vector[1] + m_scalar*m_vector[2] );
    dest[2]=2*( m_vector[0] * m_vector[2] - m_scalar*m_vector[1] );
  }
  else if( col == 1 )
  {
    dest[0]=2*( m_vector[0] * m_vector[1] - m_scalar*m_vector[2] );
    dest[1]=1.0 - 2*( m_vector[0] * m_vector[0] + m_vector[2] * m_vector[2] );
    dest[2]=2*( m_vector[1] * m_vector[2] + m_scalar*m_vector[0] );
  }
  else // if( col == 2 )
  {
    dest[0]=2*( m_vector[0] * m_vector[2] + m_scalar*m_vector[1] );
    dest[1]=2*( m_vector[1] * m_vector[2] - m_scalar*m_vector[0] );
    dest[2]=1.0 - 2*( m_vector[0] * m_vector[0] + m_vector[1] * m_vector[1] );
  }
}

// ===================================================================
// operator==(): Compare two unit quaternions to see if they create
// the same rotation, with a roundoff tolerance of
// SAI_QUATERNION_EPSILON.
//
// Note that -q produces the same rotation as +q, so this method will
// say they are identical.
// ===================================================================
int SAIQuaternion::operator==( const SAIQuaternion& rhs )
{
  Float mag2 = dot( rhs );
  return (mag2 > SAI_QUATERNION_COS_THRESHHOLD
    || mag2 < -SAI_QUATERNION_COS_THRESHHOLD );
}

// ===================================================================
// multiply(): Multiply two quaternions.
// Requires 16 mult + 8 add operations.
// ===================================================================
void SAIQuaternion::multiply( const SAIQuaternion& rhs,
                             SAIQuaternion& dest ) const
{
  SAIVector3 tmpV;

  dest.m_scalar = m_scalar * rhs.m_scalar - m_vector.dot( rhs.m_vector );

  rhs.m_vector.multiply( m_scalar, dest.m_vector );
  m_vector.multiply( rhs.m_scalar, tmpV );
  dest.m_vector += tmpV;
  m_vector.cross( rhs.m_vector, tmpV );
  dest.m_vector += tmpV;
}

// ===================================================================
// multiply(): Apply a unit quaternion to a 3x1 vector, to rotate the
// vector.
//
// If the quaternion is (w,v) and the vector is x, then this operation
// takes the quaternion product (w,v) * (0,x) * (w,-v), and discards
// the scalar part of the result.  If you do the math, this reduces to
// x * (w^2 - v.v) + 2w * (v X x) + 2v * (v . x).
//
// Requires 23 mult + 15 add operations
// ===================================================================
void SAIQuaternion::multiply( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( rhs.size() == 3 );
  dest.setSize( 3 );

  SAIVector3 tmpV;
  rhs.multiply( m_scalar * m_scalar - m_vector.dot( m_vector ), dest );
  m_vector.cross( rhs, tmpV );
  tmpV *= (m_scalar + m_scalar);
  dest += tmpV;
  m_vector.multiply( 2 * m_vector.dot( rhs ), tmpV );
  dest += tmpV;
}

// ===================================================================
// angularError(): Find the instantaneous angular error between this
// orientation and a desired orientation.  dPhi = E_inv *( q - qd )
//
// this and qd should both be unit quaternions.  This operation is the
// following equation, where E(q) and q_tilde are 4x3 matrices such
// that q_tilde * q = 0:
//
//    dPhi = E(q)^-1 * (q - qd) = (2 * q_tilde^T) * (q - qd)
//         = -2 * q_tilde^T * qd
//    dPhi = -0.5 * ( R[x] X Rd[x]  +  R[y] X Rd[y]  +  R[z] X Rd[z] )
//
// The value of q_tilde depends on who you ask.  According to
// Featherstone & Mirtich, it is:
//
//     q_tilde = [ -x -y -z
//                  w -z  y
//                  z  w -x
//                 -y  x  w ]
//
// But according to Khatib, it is:
//     q_tilde = [ -x -y -z
//                  w  z -y
//                 -z  w  x
//                  y -x  w ]
//
// This routine uses the Khatib definition.
// ===================================================================
void
SAIQuaternion::angularError( const SAIQuaternion& qd, SAIVector& dPhi ) const
{
  dPhi.setSize( 3 );

  dPhi[0] = -2 * ( - m_vector[0] * qd.m_scalar
                   + m_scalar    * qd.m_vector[0]
                   - m_vector[2] * qd.m_vector[1]
                   + m_vector[1] * qd.m_vector[2] );
  dPhi[1] = -2 * ( - m_vector[1] * qd.m_scalar
                   + m_vector[2] * qd.m_vector[0]
                   + m_scalar    * qd.m_vector[1]
                   - m_vector[0] * qd.m_vector[2] );
  dPhi[2] = -2 * ( - m_vector[2] * qd.m_scalar
                   - m_vector[1] * qd.m_vector[0]
                   + m_vector[0] * qd.m_vector[1]
                   + m_scalar    * qd.m_vector[2] );
}

// ===================================================================
// velocity(): Convert an angular veloctity (omega) into the
// derivative of the orientation quaternion (dq).
//
// This operation is the following equation, where E(q) and q_tilde
// are defined as in angularError():
//
//     dq = E(q) * omega
//        = 0.5 * q_tilde * omega (Khatib's)
// ===================================================================
void SAIQuaternion::velocity( const SAIVector& omega, SAIQuaternion& dq ) const
{
  assert( omega.size() == 3 );

  dq.values((- m_vector[0] * omega[0]
             - m_vector[1] * omega[1]
             - m_vector[2] * omega[2]) * .5,

            (+ m_scalar    * omega[0]
             + m_vector[2] * omega[1]  // -
             - m_vector[1] * omega[2]) * .5, // +

            (- m_vector[2] * omega[0]  // +
             + m_scalar    * omega[1]  
             + m_vector[0] * omega[2]) * .5, // -

            (+ m_vector[1] * omega[0]  // -
             - m_vector[0] * omega[1]  // +
             + m_scalar    * omega[2]) * .5 );
}

// ===================================================================
// display(): Display the quaternion
// ===================================================================
void SAIQuaternion::display( const char* name ) const
{
  if( name != NULL )
  {
    printf( "%s =\n", name );
  }

  printf( "[%.6f, (%.6f %.6f %.6f)]",
          m_scalar, m_vector[0], m_vector[1], m_vector[2] );

  if( name != NULL )
  {
    printf( "\n" );
  }
}

std::ostream& operator<<(std::ostream& os, const SAIQuaternion& q)
{
  os << "[" << q.m_scalar << ", (" << q.m_vector[0] << " " 
    << q.m_vector[1] << " " << q.m_vector[2] << ")]";
  return os;
}

std::istream& operator>>(std::istream& is, SAIQuaternion& q)
{
  is >> q.m_scalar;
  is >> q.m_vector[0];
  is >> q.m_vector[1];
  is >> q.m_vector[2];
  return is;
}

// ============================================================================
// interpolate(): interpolates between q1 and q2 to the specified fraction
// ============================================================================
SAIQuaternion SAIQuaternion::interpolate( const SAIQuaternion& q10, const SAIQuaternion& q20, double fraction )
{
  Float w2,x2,y2,z2;

  Float fCosTheta = q10.dot( q20 );
  if( fCosTheta >= 0 )
  {  w2 = q20.m_scalar; x2 = q20.m_vector[0]; y2 = q20.m_vector[1]; z2 = q20.m_vector[2]; }
  else
  {  w2 = -q20.m_scalar; x2 = -q20.m_vector[0]; y2 = -q20.m_vector[1]; z2 = -q20.m_vector[2]; fCosTheta = -fCosTheta; }


  Float r, s;
  // calculate interpolation factors
  if( fCosTheta > SAI_QUATERNION_COS_THRESHHOLD )
  { // use linear interpolation
    r = 1 - fraction;
    s = fraction;
  }
  else
  { // calculate spherical factors
    Float alpha = acos( fCosTheta );
    Float phi = 1/alpha;
    r = sin( (1-fraction )*alpha ) * phi;
    s = sin( fraction*alpha ) * phi;
  }


  // set the interpolated quaternion
  SAIQuaternion q;
  q.m_scalar    = r*q10.m_scalar    + s*w2;
  q.m_vector[0] = r*q10.m_vector[0] + s*x2;
  q.m_vector[1] = r*q10.m_vector[1] + s*y2;
  q.m_vector[2] = r*q10.m_vector[2] + s*z2;

  // normalize the result
  return q.unit(); 
}

SAIVector SAIQuaternion::vecForm() const {
  SAIVector v(4);
  v[0] = m_scalar;
  v[1] = m_vector[0];
  v[2] = m_vector[1];
  v[3] = m_vector[2];
  return v;
}

SAIMatrix SAIQuaternion::EMatrix() const {
  SAIMatrix E(4,3);
  SAIVector v = vecForm();

  E.setRow(0, SAIVector3(-v[1], -v[2], -v[3]));
  E.setRow(1, SAIVector3( v[0],  v[3], -v[2]));
  E.setRow(2, SAIVector3(-v[3],  v[0],  v[1]));
  E.setRow(3, SAIVector3( v[2], -v[1],  v[0]));

  E.multiply(0.5);
  return E;
}
