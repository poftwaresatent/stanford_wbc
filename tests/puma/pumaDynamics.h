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
// pumaDynamics.h
//
// This file calculates the matrices needed for puma dynamics in joint
// space, including the Jacobian, mass matrix, coriolis/centrifugal
// vector, and gravity vector.  The dJ matrix is the derivative dJ/dt,
// used to convert the B matrix to operational space.
// *******************************************************************
#ifndef PUMA_DYNAMICS_H
#define PUMA_DYNAMICS_H

#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>

void getPumaDynamics( const SAIVector& q, const SAIVector& dq,
                      SAIMatrix& J, SAIMatrix& dJ,
                      SAIMatrix& A, SAIVector& B, SAIVector& G );

void getPumaSingularities( Float sbound, const SAIVector& q,
                           bool& headLock, bool& elbowLock, bool& wristLock );

#endif // PUMA_DYNAMICS_H
