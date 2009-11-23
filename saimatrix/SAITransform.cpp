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
// 11/05/97: K.C. Chang: created.
// *******************************************************************


#include "SAITransform.h"

// ===================================================================
// multiply():  Combine two transformations into one, using the rule
// [r1, p1] * [r2, p2] = [r1*r2, r1*p2 + p1]
// ===================================================================
void SAITransform::multiply( const SAITransform& rhs, SAITransform& dest ) const
{
  m_rot.multiply( rhs.m_rot, dest.m_rot );
  m_rot.multiply( rhs.m_trans, dest.m_trans );
  dest.m_trans += m_trans;
}

// ===================================================================
// multiply():  Apply a transformation to a 3x1 vector, using the rule
// [r, p] * v = r*v + p
// ===================================================================
void SAITransform::multiply( const SAIVector& rhs, SAIVector& dest ) const
{
  assert( rhs.size() == 3 );
  dest.setSize( 3 );

  m_rot.multiply( rhs, dest );
  dest += m_trans;
}

// ===================================================================
// inverse():  Find the inverse of a transformation, using the rule
// ~[r, p] = [~r, -(~r * p)]
// ===================================================================
void SAITransform::inverse( SAITransform& dest ) const
{
  m_rot.inverse( dest.m_rot );
  dest.m_rot.multiply( -m_trans, dest.m_trans );
}

// ===================================================================
// error(): Given a desired position/orientation, find the 6-element
// vector that contains the positional error & instantaneous angular
// error between this frame and the desired frame.
//
// dest = this - Td
// ===================================================================
void SAITransform::error( const SAITransform& Td, SAIVector& dest )
{
  dest.setSize( 6 );
  dest.setSubvector( 0, m_trans - Td.m_trans );
  dest.setSubvector( 3, m_rot.angularError( Td.m_rot ) );
}

// ===================================================================
// Xform(): Transform a force/moment vector from frame i to j, given
// a transformation from i to j.  Equivalent to multiplying by the
// matrix X:
//
//           [ R           | 0 ]
//           [ P.cross * R | R ] 
// ===================================================================
void SAITransform::Xform( const SAIVector6& rhs, SAIVector6& dest ) const
{
  m_rot.multiply( rhs.linearPart(), dest.linearPart() );
  m_trans.cross( dest.linearPart(), dest.angularPart() );
  dest.angularPart() += m_rot * rhs.angularPart();
}

void SAITransform::Xform( const SAIVector& rhs, SAIVector& dest ) const
{
  SAIVector6 tmpRhs(rhs);
  SAIVector6 tmpDest;
  Xform( tmpRhs, tmpDest );
  dest = tmpDest;
}

// ===================================================================
// XformT(): Transform a linear/angular velocity vector from frame j
// to i, given a transformation from i to j.  Equivalent to
// multiplying by the transpose of the matrix X given in Xform():
//
//           [ R^T | -R^T * P.cross ] 
//           [ 0   | R^T            ]
// ===================================================================
void SAITransform::XformT( const SAIVector6& rhs, SAIVector6& dest ) const
{
  ( ~m_rot ).multiply( rhs.linearPart() - m_trans.cross( rhs.angularPart() ),
                       dest.linearPart() );
  ( ~m_rot ).multiply( rhs.angularPart(), dest.angularPart() );
}

void SAITransform::XformT( const SAIVector& rhs, SAIVector& dest ) const
{
  SAIVector6 tmpRhs(rhs);
  SAIVector6 tmpDest;
  Xform( tmpRhs, tmpDest );
  dest = tmpDest;
}

// ===================================================================
// display(): Display the transform
// ===================================================================
void SAITransform::display( const char* name ) const
{
  if( name != NULL )
  {
    printf("%s =\n", name );
  }

  printf("[rot=");
  m_rot.display();
  printf(",\n trans=");
  m_trans.display();
  printf("]\n");
}

std::ostream& operator<<(std::ostream& os, const SAITransform& t)
{
  os << "pos: " << t.m_trans << endl
    << "rot: " << t.m_rot << endl;
  return os;
}
